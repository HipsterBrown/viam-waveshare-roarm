package geometry

import (
	"encoding/json"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Gripper geometry in millimetres, in the arm's tool frame (+Z is the
// approach axis, pointing out of the gripper mount).
//
// GripperMountToTCPMM is the Waveshare URDF's hand_tcp (z 115.428 from the
// wrist-roll frame) minus the gripper pivot (z 52.035), i.e. the grasp point
// between the closed jaw tips measured from the mount. The box is the jaw
// envelope with the jaws closed; its depth is 70 mm (the spec's earlier 60 mm
// box ended short of the 63.4 mm TCP, and a grasp point outside the collision
// volume is useless to the planner). Bench task B5 re-measures all four values.
const (
	GripperMountToTCPMM = 63.4
	GripperBoxX         = 70.0
	GripperBoxY         = 40.0
	GripperBoxZ         = 70.0
)

// GripperJointLimits is joint 6's software-frame range in radians (about
// -11.5 to 109 degrees). Joints 1-5 take their limits from roarm_m3.json.
var GripperJointLimits = [2]float64{-0.2, 1.9}

// GripperModel returns the gripper's kinematic model: zero DoF, one
// link carrying the jaw-envelope box, and a "tcp" leaf at the grasp point.
//
// It is built as SVA JSON and parsed back rather than assembled in memory
// because a component ships its kinematics to viam-server as
// ModelConfig().OriginalFile.Bytes; a hand-assembled model has none and is
// transmitted as UNSPECIFIED, which drops the gripper's collision geometry
// from the frame system (viam-server never calls Geometries() for arm,
// gantry, or gripper components). Zero DoF on purpose: a jaw DoF would be a
// variable the planner could drive.
func GripperModel(name string) (referenceframe.Model, error) {
	box, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: GripperBoxZ / 2}),
		r3.Vector{X: GripperBoxX, Y: GripperBoxY, Z: GripperBoxZ},
		"gripper_body",
	)
	if err != nil {
		return nil, err
	}
	geomCfg, err := spatialmath.NewGeometryConfig(box)
	if err != nil {
		return nil, fmt.Errorf("gripper box to geometry config: %w", err)
	}
	cfg := &referenceframe.ModelConfigJSON{
		Name:         name,
		KinParamType: "SVA",
		Links: []referenceframe.LinkConfig{
			{ID: "body", Parent: referenceframe.World, Geometry: geomCfg},
			{ID: "tcp", Parent: "body", Translation: r3.Vector{Z: GripperMountToTCPMM}},
		},
	}
	jsonBytes, err := json.Marshal(cfg)
	if err != nil {
		return nil, fmt.Errorf("serializing the gripper model: %w", err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
}
