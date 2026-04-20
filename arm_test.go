package waveshareroarm

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
)

func mustLoadModel(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := makeRoArmModelFrame()
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestMoveClampsToJointLimits(t *testing.T) {
	fc := &fakeController{}
	// Pre-populate GetJointRadians response so MoveToJointPositions can read current.
	fc.Feedback = FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	// ask for joint 1 at 10 rad (way over the +3.3 limit)
	positions := []referenceframe.Input{
		{Value: 10.0}, // should clamp to 3.3
		{Value: 0}, {Value: 0}, {Value: 0}, {Value: 0},
	}
	if err := r.MoveToJointPositions(context.Background(), positions, nil); err != nil {
		t.Fatal(err)
	}
	if fc.LastRadians[0] != 3.3 {
		t.Fatalf("expected clamp to 3.3, got %v", fc.LastRadians[0])
	}
}

func TestMoveClampsBelowMinimum(t *testing.T) {
	fc := &fakeController{}
	fc.Feedback = FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	// Joint 1 min is -3.3
	positions := []referenceframe.Input{
		{Value: -10.0}, // should clamp to -3.3
		{Value: 0}, {Value: 0}, {Value: 0}, {Value: 0},
	}
	if err := r.MoveToJointPositions(context.Background(), positions, nil); err != nil {
		t.Fatal(err)
	}
	if fc.LastRadians[0] != -3.3 {
		t.Fatalf("expected clamp to -3.3, got %v", fc.LastRadians[0])
	}
}

func TestMoveRejectsWrongLengthInput(t *testing.T) {
	fc := &fakeController{}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	err := r.MoveToJointPositions(context.Background(), []referenceframe.Input{{Value: 0}}, nil)
	if err == nil {
		t.Fatal("expected error for wrong length")
	}
}

func TestMovePreservesGripperPosition(t *testing.T) {
	fc := &fakeController{}
	// Gripper currently at some position; MoveToJointPositions must preserve it.
	fc.Feedback = FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0.5}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	positions := []referenceframe.Input{
		{Value: 0.1}, {Value: 0.2}, {Value: 0.3}, {Value: 0.4}, {Value: 0.5},
	}
	if err := r.MoveToJointPositions(context.Background(), positions, nil); err != nil {
		t.Fatal(err)
	}
	if len(fc.LastRadians) < 6 {
		t.Fatalf("expected 6 joint positions sent, got %d", len(fc.LastRadians))
	}
	if fc.LastRadians[5] != 0.5 {
		t.Fatalf("expected gripper preserved at 0.5, got %v", fc.LastRadians[5])
	}
}
