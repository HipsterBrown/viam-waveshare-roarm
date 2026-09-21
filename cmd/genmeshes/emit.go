package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// The arm chain in kinematic order. Each link carries the origin of the joint
// leading OUT of it (an SVA link runs from its parent frame to where the next
// joint sits); base_link's outgoing origin is zero so it carries the incoming
// fixed world_to_base_link instead; link5's outgoing joint is the jaw, which
// is not part of the arm, so link5 is identity and the tool leaf carries the
// wrist-to-tool offset.
var armLinks = []string{"base_link", "link1", "link2", "link3", "link4", "link5"}

// toolOffsetMM is the jaw pivot plane on the roll axis: link5_to_gripper_link's
// origin z without its lateral y offset.
const toolOffsetMM = 52.035

// linkGeometry is what the emitter puts on each link: a box (centre c, size)
// or a mesh (PLY bytes at centre c), both derived from the link's STL.
type linkGeometry struct {
	Center r3.Vector
	Size   r3.Vector // box mode
	PLY    []byte    // mesh mode, nil for box mode
	Label  string
}

// linkTransform returns the SVA translation and orientation for link L.
func linkTransform(joints []urdfJoint, link string) (spatialmath.Pose, error) {
	out, hasOut := jointByParent(joints, link)
	in, hasIn := jointByChild(joints, link)
	switch {
	case link == "base_link":
		if !hasIn || !hasOut {
			return nil, fmt.Errorf("base_link needs an incoming fixed joint and an outgoing revolute joint")
		}
		return spatialmath.Compose(originPose(in), originPose(out)), nil
	case link == "link5":
		return spatialmath.NewZeroPose(), nil
	case hasOut:
		return originPose(out), nil
	}
	return nil, fmt.Errorf("link %s has no outgoing revolute joint", link)
}

// emitModel writes an SVA ModelConfigJSON for the chain with the given per-link
// geometry. name is the model name inside the file ("roarm_m3").
func emitModel(joints []urdfJoint, geoms map[string]linkGeometry, name string) ([]byte, error) {
	cfg := &referenceframe.ModelConfigJSON{Name: name, KinParamType: "SVA"}
	parent := referenceframe.World
	for i, link := range armLinks {
		pose, err := linkTransform(joints, link)
		if err != nil {
			return nil, err
		}
		oc, err := spatialmath.NewOrientationConfig(pose.Orientation())
		if err != nil {
			return nil, err
		}
		lc := referenceframe.LinkConfig{ID: link, Parent: parent, Translation: round3(pose.Point()), Orientation: oc}
		if g, ok := geoms[link]; ok {
			lc.Geometry = geometryConfig(g)
		}
		cfg.Links = append(cfg.Links, lc)
		if i == len(armLinks)-1 {
			break
		}
		out, _ := jointByParent(joints, link)
		cfg.Joints = append(cfg.Joints, referenceframe.JointConfig{
			ID: out.Name, Type: "revolute", Parent: link,
			Axis: spatialmath.AxisConfig(out.Axis),
			Min:  out.Lower * 180 / math.Pi, Max: out.Upper * 180 / math.Pi,
		})
		parent = out.Name
	}
	zero, _ := spatialmath.NewOrientationConfig(spatialmath.NewZeroOrientation())
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID: "tool", Parent: "link5", Translation: r3.Vector{Z: toolOffsetMM}, Orientation: zero,
	})
	return json.MarshalIndent(cfg, "", "  ")
}

func geometryConfig(g linkGeometry) *spatialmath.GeometryConfig {
	zero, _ := spatialmath.NewOrientationConfig(spatialmath.NewZeroOrientation())
	gc := &spatialmath.GeometryConfig{TranslationOffset: round1(g.Center), OrientationOffset: *zero, Label: g.Label}
	if g.PLY != nil {
		gc.Type = spatialmath.MeshType
		gc.MeshData = g.PLY
		gc.MeshContentType = "ply"
		return gc
	}
	gc.Type = spatialmath.BoxType
	s := round1(g.Size)
	gc.X, gc.Y, gc.Z = s.X, s.Y, s.Z
	return gc
}

// round1 rounds geometry to 0.1 mm so the JSON stays readable and stable
// across runs; round3 keeps link transforms at the URDF's 0.001 mm.
func round1(v r3.Vector) r3.Vector {
	f := func(x float64) float64 { return math.Round(x*10) / 10 }
	return r3.Vector{X: f(v.X), Y: f(v.Y), Z: f(v.Z)}
}

func round3(v r3.Vector) r3.Vector { return r3.Vector{X: r(v.X), Y: r(v.Y), Z: r(v.Z)} }

// mm formats a vector for the report.
func mm(v r3.Vector) string { return fmt.Sprintf("(%.1f, %.1f, %.1f)", v.X, v.Y, v.Z) }

// Jaw: gripper_link.stl is the moving jaw, hinged on link5_to_gripper_link.
// It is aligned into the tool frame (link5 frame translated by toolOffsetMM
// along +Z) at jaw angle 0 (closed), centred like the arm links, and written
// at full resolution (viewer) and hull-decimated (collision).
func emitJaw(joints []urdfJoint, world map[string]spatialmath.Pose, meshDir, out string, hullTris int) error {
	jaw, ok := jointByChild(joints, "gripper_link")
	if !ok {
		return fmt.Errorf("URDF has no joint whose child is gripper_link")
	}
	tris, err := readSTLFile(filepath.Join(meshDir, "gripper_link.stl"))
	if err != nil {
		return err
	}
	toolWorld := spatialmath.Compose(world["link5"], spatialmath.NewPoseFromPoint(r3.Vector{Z: toolOffsetMM}))
	aligned := transformTris(tris, spatialmath.Compose(spatialmath.PoseInverse(toolWorld), world["gripper_link"]))
	lo, hi := aabb(aligned)
	c := lo.Add(hi).Mul(0.5)
	centred := translateTris(aligned, c.Mul(-1))

	full := toMesh(centred, "gripper_jaw").TrianglesToPLYBytes(false)
	hull, err := toMesh(centred, "gripper_jaw").ConservativeDecimate(hullTris)
	if err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(out, "meshes", "gripper_jaw.ply"), full, 0o644); err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(out, "meshes", "gripper_jaw_collision.ply"), hull.TrianglesToPLYBytes(false), 0o644); err != nil {
		return err
	}

	// Hinge in the tool frame: pivot is the joint origin minus the tool
	// offset; axis is the URDF child-frame axis rotated by the joint's rpy.
	pivot := jaw.XYZ.Sub(r3.Vector{Z: toolOffsetMM})
	axis := spatialmath.Compose(spatialmath.NewPoseFromOrientation(originPose(jaw).Orientation()),
		spatialmath.NewPoseFromPoint(jaw.Axis)).Point()
	size := round1(hi.Sub(lo))
	src := fmt.Sprintf(`// Code generated by cmd/genmeshes from roarm_m3.xacro; DO NOT EDIT.

package geometry

import "github.com/golang/geo/r3"

// Jaw hinge and mesh anchor, in the arm's tool frame (millimetres, radians).
var (
	JawPivot      = r3.Vector{X: %g, Y: %g, Z: %g}
	JawAxis       = r3.Vector{X: %g, Y: %g, Z: %g}
	JawMeshCenter = r3.Vector{X: %g, Y: %g, Z: %g}
	JawBox        = r3.Vector{X: %g, Y: %g, Z: %g}
)

const (
	JawURDFMin = %g
	JawURDFMax = %g
)
`, r(pivot.X), r(pivot.Y), r(pivot.Z), r(axis.X), r(axis.Y), r(axis.Z),
		r(c.X), r(c.Y), r(c.Z), size.X, size.Y, size.Z, jaw.Lower, jaw.Upper)
	fmt.Printf("jaw: %d tris, hull %d, pivot %s axis %s centre %s size %s\n",
		len(tris), len(hull.Triangles()), mm(pivot), mm(axis), mm(c), mm(size))
	return os.WriteFile(filepath.Join(out, "gripper_jaw.go"), []byte(src), 0o644)
}

// r rounds to 0.001 for the generated Go source.
func r(x float64) float64 { return math.Round(x*1000) / 1000 }
