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

func newTestArm(t *testing.T, fc *fakeController) *roarmM3 {
	t.Helper()
	return &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
}

func TestDoCommand_SetTorque(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_torque",
		"enable":  true,
	})
	if err != nil {
		t.Fatal(err)
	}
	if fc.LastTorque == nil || *fc.LastTorque != true {
		t.Fatalf("expected torque=true, got %v", fc.LastTorque)
	}
	if out["success"] != true {
		t.Fatalf("expected success=true, got %v", out)
	}
}

func TestDoCommand_SetTorqueMissingParam(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "set_torque"})
	if err == nil {
		t.Fatal("expected error for missing enable param")
	}
}

func TestDoCommand_SetLED(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command":    "set_led",
		"brightness": float64(128),
	})
	if err != nil {
		t.Fatal(err)
	}
	if fc.LastLED == nil || *fc.LastLED != 128 {
		t.Fatalf("expected LED=128, got %v", fc.LastLED)
	}
}

func TestDoCommand_SetLEDMissingParam(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "set_led"})
	if err == nil {
		t.Fatal("expected error for missing brightness param")
	}
}

func TestDoCommand_MoveToHome(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "move_to_home"})
	if err != nil {
		t.Fatal(err)
	}
	if fc.HomeCalls != 1 {
		t.Fatalf("expected 1 home call, got %d", fc.HomeCalls)
	}
}

func TestDoCommand_GetFeedback(t *testing.T) {
	fc := &fakeController{
		Feedback: FeedbackData{X: 1, Y: 2, Z: 3, B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6},
	}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "get_feedback"})
	if err != nil {
		t.Fatal(err)
	}
	pos, ok := out["position"].(map[string]interface{})
	if !ok {
		t.Fatalf("expected position map, got %v", out)
	}
	if pos["x"] != 1.0 {
		t.Fatalf("expected x=1, got %v", pos["x"])
	}
	joints, ok := out["joints"].(map[string]interface{})
	if !ok {
		t.Fatalf("expected joints map")
	}
	if joints["base"] != 0.1 {
		t.Fatalf("expected base=0.1, got %v", joints["base"])
	}
}

func TestDoCommand_SetSpeed(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_speed",
		"value":   float64(60),
	})
	if err != nil {
		t.Fatal(err)
	}
	if out["speed_set"] != 60.0 {
		t.Fatalf("expected speed_set=60, got %v", out)
	}
	// Internal speed should be updated (60 deg/s * 10 = 600 internal)
	if r.defaultSpeed != 600 {
		t.Fatalf("expected defaultSpeed=600, got %d", r.defaultSpeed)
	}
}

func TestDoCommand_SetSpeedOutOfRange(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_speed",
		"value":   float64(1000),
	})
	if err == nil {
		t.Fatal("expected error for out-of-range speed")
	}
}

func TestDoCommand_SetAcceleration(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_acceleration",
		"value":   float64(100),
	})
	if err != nil {
		t.Fatal(err)
	}
	if out["acceleration_set"] != 100.0 {
		t.Fatalf("expected acceleration_set=100, got %v", out)
	}
	// Internal accel: 100 * 0.5 = 50 units
	if r.defaultAcc != 50 {
		t.Fatalf("expected defaultAcc=50, got %d", r.defaultAcc)
	}
}

func TestDoCommand_SetAccelerationOutOfRange(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": "set_acceleration",
		"value":   float64(1000),
	})
	if err == nil {
		t.Fatal("expected error for out-of-range acceleration")
	}
}

func TestDoCommand_GetMotionParams(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "get_motion_params"})
	if err != nil {
		t.Fatal(err)
	}
	// Default speed in units = 500 → 50 deg/s
	if out["current_speed_degs_per_sec"] != 50.0 {
		t.Fatalf("expected current_speed_degs_per_sec=50, got %v", out["current_speed_degs_per_sec"])
	}
	// Default accel in units = 50 → 100 deg/s^2
	if out["current_acceleration_degs_per_sec_per_sec"] != 100.0 {
		t.Fatalf("expected current_acceleration_degs_per_sec_per_sec=100, got %v", out["current_acceleration_degs_per_sec_per_sec"])
	}
}

func TestDoCommand_UnknownCommand(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "nonsense"})
	if err == nil {
		t.Fatal("expected error for unknown command")
	}
}

func TestArmStopHoldsCurrentPosition(t *testing.T) {
	fc := &fakeController{
		Feedback: FeedbackData{B: 0.5, S: 0.3, E: 0.1, Wrist: 0.2, R: 0.4, G: 0.0},
	}
	r := &roarmM3{
		controller: fc, defaultSpeed: 500, defaultAcc: 50,
		jointLimits: RoArmM3JointLimits[:5],
		logger:      logging.NewTestLogger(t),
		opMgr:       operation.NewSingleOperationManager(),
	}
	if err := r.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	want := []float64{0.5, 0.3, 0.1, 0.2, 0.4, 0.0}
	for i, v := range want {
		if fc.LastRadians[i] != v {
			t.Fatalf("joint %d: got %v, want %v", i, fc.LastRadians[i], v)
		}
	}
	const expectedStopSpeed = 100
	if fc.LastSpeed != expectedStopSpeed {
		t.Fatalf("expected stop speed %d, got %d", expectedStopSpeed, fc.LastSpeed)
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
