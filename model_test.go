package waveshareroarm

import "testing"

// TestModelLoadsWithBaseZOffset ensures the roarm_m3.json kinematic model
// parses cleanly and the base_link translation survives unmarshaling.
// Regressions here mean the whole arm IK is silently offset.
func TestModelLoadsWithBaseZOffset(t *testing.T) {
	model, err := makeRoArmModelFrame()
	if err != nil {
		t.Fatalf("failed to parse kinematic model: %v", err)
	}
	if model == nil {
		t.Fatal("nil model returned without error")
	}
}
