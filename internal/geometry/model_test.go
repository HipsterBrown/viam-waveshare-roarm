package geometry

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
)

// homePose is the arm extended: elbow at 90 degrees, everything else zero.
var homePose = []referenceframe.Input{0, 0, math.Pi / 2, 0, 0}

func TestModelLoads(t *testing.T) {
	model, err := ArmModel("roarm_m3")
	if err != nil {
		t.Fatalf("failed to parse kinematic model: %v", err)
	}
	if model == nil {
		t.Fatal("nil model returned without error")
	}
	if got := len(model.DoF()); got != 5 {
		t.Fatalf("expected 5 DoF, got %d", got)
	}
}

// The JSON limits are the module's only joint limits (rdk's arm client
// validates against them too), so they must match the URDF's mechanical
// limits and nothing else may widen them.
func TestModelJointLimitsAreTheURDFLimits(t *testing.T) {
	model, err := ArmModel("roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	wantDeg := [][2]float64{{-180, 180}, {-90, 90}, {-57.3, 169.0}, {-90, 90}, {-180, 180}}
	for i, l := range model.DoF() {
		if math.Abs(l.Min*180/math.Pi-wantDeg[i][0]) > 0.1 || math.Abs(l.Max*180/math.Pi-wantDeg[i][1]) > 0.1 {
			t.Fatalf("joint %d limits %.1f..%.1f deg, want %v", i+1, l.Min*180/math.Pi, l.Max*180/math.Pi, wantDeg[i])
		}
	}
}

// modelWithoutTool parses roarm_m3.json with the tool link removed, so the
// tool offset can be measured against the bare chain.
func modelWithoutTool(t *testing.T) referenceframe.Model {
	t.Helper()
	m := &referenceframe.ModelConfigJSON{}
	if err := json.Unmarshal(roarmModelJson, m); err != nil {
		t.Fatal(err)
	}
	links := m.Links[:0]
	for _, l := range m.Links {
		if l.ID != "tool" {
			links = append(links, l)
		}
	}
	m.Links = links
	model, err := m.ParseConfig("bare")
	if err != nil {
		t.Fatal(err)
	}
	return model
}

// The tool frame sits ToolOffsetMM beyond the link5 (wrist-roll) frame,
// further from the base than link5 itself (i.e. along +Z of the roll frame,
// not into link4). The bare model's leaf is link5. If this fails after a JSON
// edit, the tool translation has the wrong sign or is on the wrong axis.
func TestToolFrameIsTheGripperMount(t *testing.T) {
	withTool, err := ArmModel("roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	bare := modelWithoutTool(t)
	pTool, err := withTool.Transform(homePose)
	if err != nil {
		t.Fatal(err)
	}
	pJoint, err := bare.Transform(homePose)
	if err != nil {
		t.Fatal(err)
	}
	d := pTool.Point().Sub(pJoint.Point()).Norm()
	if math.Abs(d-ToolOffsetMM) > 0.01 {
		t.Fatalf("tool is %.3f mm from link5, want %.3f", d, ToolOffsetMM)
	}
	if pTool.Point().Norm() <= pJoint.Point().Norm() {
		t.Fatalf("tool (%v) is not further from the base than link5 (%v): translation sign is wrong", pTool.Point(), pJoint.Point())
	}
}

// The whole chain, pinned to Waveshare's URDF: at the zero pose the tool sits
// ToolOffsetMM along link5's +Z from link5's world origin (45.147, 0, 557.113).
// cmd/genmeshes prints the same world positions when it regenerates
// roarm_m3.json; if this drifts, the two have diverged.
func TestChainMatchesURDF(t *testing.T) {
	model, err := ArmModel("roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	p, err := model.Transform(make([]referenceframe.Input, len(model.DoF())))
	if err != nil {
		t.Fatal(err)
	}
	want := r3.Vector{X: 45.148, Y: 0, Z: 609.148}
	if p.Point().Sub(want).Norm() > 0.1 {
		t.Fatalf("tool at zero pose is %v, want %v", p.Point(), want)
	}
}
