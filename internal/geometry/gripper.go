package geometry

import (
	"encoding/json"
	"fmt"
	"sync"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// GripperJointLimits is joint 6's software-frame range in radians (about
// -11.5 to 109 degrees); 1.9 is open, -0.2 is closed.
var GripperJointLimits = [2]float64{-0.2, 1.9}

// GripperMountToTCPMM is the grasp point between the jaw tips measured from
// the tool frame: the URDF's hand_tcp (115.428) minus the tool plane (52.035).
const GripperMountToTCPMM = 63.393

// GripperModel is the gripper's kinematic model: zero DoF, one "body" link
// carrying the closed jaw as a box or as its per-slab bounding-polytope envelope, and a
// "tcp" leaf. Built as SVA JSON and parsed back so it ships to viam-server
// with its geometry (a model assembled in memory transmits as UNSPECIFIED).
func GripperModel(collision, name string) (referenceframe.Model, error) {
	geom := &spatialmath.GeometryConfig{TranslationOffset: JawMeshCenter, Label: "jaw"}
	if collision == CollisionMesh {
		ply, err := meshFS.ReadFile("meshes/gripper_jaw_collision.ply")
		if err != nil {
			return nil, err
		}
		geom.Type, geom.MeshData, geom.MeshContentType = spatialmath.MeshType, ply, "ply"
	} else {
		geom.Type, geom.X, geom.Y, geom.Z = spatialmath.BoxType, JawBox.X, JawBox.Y, JawBox.Z
	}
	cfg := &referenceframe.ModelConfigJSON{
		Name: name, KinParamType: "SVA",
		Links: []referenceframe.LinkConfig{
			{ID: "body", Parent: referenceframe.World, Geometry: geom},
			{ID: "tcp", Parent: "body", Translation: r3.Vector{Z: GripperMountToTCPMM}},
		},
	}
	raw, err := json.Marshal(cfg)
	if err != nil {
		return nil, fmt.Errorf("serializing the gripper model: %w", err)
	}
	return referenceframe.UnmarshalModelJSON(raw, name)
}

// JawPose is the pose of the jaw mesh in the tool frame for a software-frame
// jaw angle: the mesh anchor rotated about the hinge. The software range maps
// linearly onto the URDF joint range; both describe the same travel with
// different zeros.
func JawPose(softwareRad float64) spatialmath.Pose {
	lo, hi := GripperJointLimits[0], GripperJointLimits[1]
	s := (clampf(softwareRad, lo, hi) - lo) / (hi - lo)
	theta := JawURDFMin + s*(JawURDFMax-JawURDFMin)
	rot := spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{Theta: theta, RX: JawAxis.X, RY: JawAxis.Y, RZ: JawAxis.Z})
	aboutPivot := spatialmath.Compose(spatialmath.NewPoseFromPoint(JawPivot),
		spatialmath.Compose(rot, spatialmath.NewPoseFromPoint(JawPivot.Mul(-1))))
	return spatialmath.Compose(aboutPivot, spatialmath.NewPoseFromPoint(JawMeshCenter))
}

// jawMesh parses the 420 KB full-resolution jaw PLY once; the viewer polls
// Geometries, so re-parsing per call would be wasteful.
var jawMesh = sync.OnceValues(func() (*spatialmath.Mesh, error) {
	ply, err := meshFS.ReadFile("meshes/gripper_jaw.ply")
	if err != nil {
		return nil, err
	}
	return spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), &commonpb.Mesh{ContentType: "ply", Mesh: ply}, "gripper_jaw")
})

// GripperMeshes returns the full-resolution jaw mesh posed at jawRad, for the
// gripper's Geometries (the 3D viewer). Collision uses GripperModel instead.
func GripperMeshes(jawRad float64) ([]spatialmath.Geometry, error) {
	m, err := jawMesh()
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{m.Transform(JawPose(jawRad))}, nil
}

func clampf(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
