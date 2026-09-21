package geometry

import (
	"math"
	"testing"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
)

func TestGripperModelIsZeroDoFWithTCPLeaf(t *testing.T) {
	m, err := GripperModel("g")
	if err != nil {
		t.Fatal(err)
	}
	if len(m.DoF()) != 0 {
		t.Fatalf("expected 0 DoF, got %d", len(m.DoF()))
	}
	pose, err := m.Transform([]referenceframe.Input{})
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(pose.Point().Z-GripperMountToTCPMM) > 0.01 || pose.Point().X != 0 || pose.Point().Y != 0 {
		t.Fatalf("model leaf is at %v, want (0,0,%.1f)", pose.Point(), GripperMountToTCPMM)
	}
}

func TestGripperModelCarriesTheJawBox(t *testing.T) {
	m, err := GripperModel("g")
	if err != nil {
		t.Fatal(err)
	}
	gif, err := m.Geometries([]referenceframe.Input{})
	if err != nil {
		t.Fatal(err)
	}
	geoms := gif.Geometries()
	if len(geoms) != 1 {
		t.Fatalf("expected 1 geometry, got %d", len(geoms))
	}
	if math.Abs(geoms[0].Pose().Point().Z-GripperBoxZ/2) > 0.01 {
		t.Fatalf("box centre at %v, want z=%.1f", geoms[0].Pose().Point(), GripperBoxZ/2)
	}
}

// A model without OriginalFile bytes transmits as UNSPECIFIED and viam-server
// drops its geometry. This is the whole reason the model is JSON-built.
func TestGripperModelSurvivesTheModuleBoundary(t *testing.T) {
	m, err := GripperModel("g")
	if err != nil {
		t.Fatal(err)
	}
	resp := referenceframe.KinematicModelToProtobuf(m)
	if resp.Format != commonpb.KinematicsFileFormat_KINEMATICS_FILE_FORMAT_SVA {
		t.Fatalf("format %v, want SVA", resp.Format)
	}
	back, err := referenceframe.UnmarshalModelJSON(resp.KinematicsData, "g")
	if err != nil {
		t.Fatalf("server-side re-parse failed: %v", err)
	}
	gif, err := back.Geometries([]referenceframe.Input{})
	if err != nil || len(gif.Geometries()) != 1 {
		t.Fatalf("geometry did not survive the round trip: %v, %v", err, gif)
	}
}
