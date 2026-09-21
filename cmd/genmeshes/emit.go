package main

import (
	"encoding/json"
	"fmt"
	"math"

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
// or a mesh (PLY bytes at centre c). Task 3 fills these from the STLs; Task 2
// keeps the current boxes.
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
		lc := referenceframe.LinkConfig{ID: link, Parent: parent, Translation: pose.Point(), Orientation: oc}
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
	gc := &spatialmath.GeometryConfig{TranslationOffset: round1(g.Center), Label: g.Label}
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

// round1 rounds to 0.1 mm so the JSON stays readable and stable across runs.
func round1(v r3.Vector) r3.Vector {
	r := func(x float64) float64 { return math.Round(x*10) / 10 }
	return r3.Vector{X: r(v.X), Y: r(v.Y), Z: r(v.Z)}
}
