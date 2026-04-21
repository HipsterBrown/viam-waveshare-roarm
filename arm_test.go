package waveshareroarm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
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

// TestEndPositionDoesNotDeadlock exercises EndPosition -> CurrentInputs ->
// JointPositions, which previously re-entered r.mu and deadlocked under
// sync.Mutex. A timeout ctx bounds any regression.
func TestEndPositionDoesNotDeadlock(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: 500,
		defaultAcc:   50,
		jointLimits:  RoArmM3JointLimits[:5],
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if _, err := r.EndPosition(ctx, nil); err != nil {
		t.Fatalf("EndPosition returned error: %v", err)
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

func TestArmName(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	r.name = resource.Name{}
	_ = r.Name()
}

func TestArmJointPositions(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6}}
	r := newTestArm(t, fc)
	inputs, err := r.JointPositions(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 5 {
		t.Fatalf("expected 5 arm joints, got %d", len(inputs))
	}
	if inputs[0].Value != 0.1 {
		t.Fatalf("expected joint 0 at 0.1, got %v", inputs[0].Value)
	}
}

func TestArmCurrentInputs(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5}}
	r := newTestArm(t, fc)
	inputs, err := r.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 5 {
		t.Fatalf("expected 5 inputs, got %d", len(inputs))
	}
}

func TestArmKinematics(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	m, err := r.Kinematics(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if m == nil {
		t.Fatal("expected non-nil kinematics")
	}
}

func TestArmIsMoving(t *testing.T) {
	fc := &fakeController{MoveDeadline: time.Now().Add(200 * time.Millisecond)}
	r := newTestArm(t, fc)
	moving, err := r.IsMoving(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if !moving {
		t.Fatal("expected moving=true")
	}
}

// TestArmClose verifies that once Close is called, subsequent operations
// refuse to touch hardware. EndPosition and Geometries are intentionally
// omitted here: they call CurrentInputs under their own mutex, which on the
// closed path would still succeed because the closed check happens first —
// but testing them on a non-closed arm tickles a separate pre-existing
// deadlock (EndPosition locks mu before CurrentInputs→JointPositions
// re-locks it) that is out of scope for this phase.
func TestArmGeometriesAfterClose(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	ctx, cancel := context.WithCancel(context.Background())
	r.cancelCtx, r.cancelFunc = ctx, cancel
	_ = r.Close(context.Background())
	if _, err := r.Geometries(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
}

func TestArmEndPositionAfterClose(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	ctx, cancel := context.WithCancel(context.Background())
	r.cancelCtx, r.cancelFunc = ctx, cancel
	_ = r.Close(context.Background())
	if _, err := r.EndPosition(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
}

func TestArmClose(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	ctx, cancel := context.WithCancel(context.Background())
	r.cancelCtx, r.cancelFunc = ctx, cancel
	if err := r.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
	// Second close is idempotent.
	if err := r.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
	// Operations after close should fail.
	if _, err := r.JointPositions(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
	if err := r.Stop(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
	if _, err := r.IsMoving(context.Background()); err == nil {
		t.Fatal("expected error after close")
	}
	if _, err := r.DoCommand(context.Background(), map[string]interface{}{}); err == nil {
		t.Fatal("expected error after close")
	}
	if err := r.MoveToJointPositions(context.Background(), nil, nil); err == nil {
		t.Fatal("expected error after close")
	}
	if err := r.MoveThroughJointPositions(context.Background(), nil, nil, nil); err == nil {
		t.Fatal("expected error after close")
	}
	if err := r.MoveToPosition(context.Background(), nil, nil); err == nil {
		t.Fatal("expected error after close")
	}
}

func TestArmMoveThroughJointPositions(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := newTestArm(t, fc)
	positions := [][]referenceframe.Input{
		{{Value: 0.1}, {Value: 0}, {Value: 0}, {Value: 0}, {Value: 0}},
		{{Value: 0.2}, {Value: 0}, {Value: 0}, {Value: 0}, {Value: 0}},
	}
	if err := r.MoveThroughJointPositions(context.Background(), positions, nil, nil); err != nil {
		t.Fatal(err)
	}
}

func TestArmGoToInputs(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := newTestArm(t, fc)
	step1 := []referenceframe.Input{{Value: 0.05}, {Value: 0}, {Value: 0}, {Value: 0}, {Value: 0}}
	if err := r.GoToInputs(context.Background(), step1); err != nil {
		t.Fatal(err)
	}
}

func TestArmNewClientFromConn(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	_, err := r.NewClientFromConn(context.Background(), nil, "", resource.Name{}, nil)
	if err == nil {
		t.Fatal("expected unsupported error")
	}
}

func TestNewRoArmM3_RejectsBadSpeed(t *testing.T) {
	conf := resource.Config{
		Name:                "arm",
		API:                 resource.APINamespace("rdk").WithType("component").WithSubtype("arm"),
		ConvertedAttributes: &RoArmM3Config{Host: "1.2.3.4", SpeedDegsPerSec: 1000},
	}
	_, err := newRoArmM3(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err == nil {
		t.Fatal("expected error for bad speed")
	}
}

func TestNewRoArmM3_RejectsBadAccel(t *testing.T) {
	conf := resource.Config{
		Name:                "arm",
		API:                 resource.APINamespace("rdk").WithType("component").WithSubtype("arm"),
		ConvertedAttributes: &RoArmM3Config{Host: "1.2.3.4", AccelerationDegsPerSec: 1000},
	}
	_, err := newRoArmM3(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err == nil {
		t.Fatal("expected error for bad accel")
	}
}

func TestNewRoArmM3_ConstructsHTTP(t *testing.T) {
	conf := resource.Config{
		Name:                "arm",
		API:                 resource.APINamespace("rdk").WithType("component").WithSubtype("arm"),
		ConvertedAttributes: &RoArmM3Config{Host: "127.0.0.1:0"},
	}
	// Should construct successfully in HTTP mode (no connection is attempted).
	armRes, err := newRoArmM3(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	defer armRes.Close(context.Background())
	if armRes.Name().Name != "arm" {
		t.Fatalf("expected name=arm, got %v", armRes.Name())
	}
}

func TestArmReconfigure_MotionOnly(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	// Seed existing config so Reconfigure's needsReopen logic can compare.
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	// Provide an equivalent Host so we don't attempt to reopen.
	conf := resource.Config{
		Name: "arm",
		ConvertedAttributes: &RoArmM3Config{
			Host:                   "1.2.3.4",
			SpeedDegsPerSec:        60,
			AccelerationDegsPerSec: 120,
		},
	}
	if err := r.Reconfigure(context.Background(), nil, conf); err != nil {
		t.Fatal(err)
	}
	// speed 60 deg/s → 600 units; accel 120 → 60 units
	if r.defaultSpeed != 600 {
		t.Fatalf("expected defaultSpeed=600, got %d", r.defaultSpeed)
	}
	if r.defaultAcc != 60 {
		t.Fatalf("expected defaultAcc=60, got %d", r.defaultAcc)
	}
}

func TestArmReconfigure_RejectsBadSpeed(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	conf := resource.Config{
		ConvertedAttributes: &RoArmM3Config{
			Host:            "1.2.3.4",
			SpeedDegsPerSec: 1000, // out of range
		},
	}
	if err := r.Reconfigure(context.Background(), nil, conf); err == nil {
		t.Fatal("expected error for out-of-range speed")
	}
}

func TestArmReconfigure_RejectsBadAccel(t *testing.T) {
	fc := &fakeController{}
	r := newTestArm(t, fc)
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	conf := resource.Config{
		ConvertedAttributes: &RoArmM3Config{
			Host:                   "1.2.3.4",
			AccelerationDegsPerSec: 1000, // out of range
		},
	}
	if err := r.Reconfigure(context.Background(), nil, conf); err == nil {
		t.Fatal("expected error for out-of-range accel")
	}
}

func TestArmDoCommand_ExtraOverrides(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := newTestArm(t, fc)
	// Exercise the extra-override path in MoveToJointPositions for speed/acceleration.
	positions := []referenceframe.Input{
		{Value: 0.1}, {Value: 0}, {Value: 0}, {Value: 0}, {Value: 0},
	}
	extra := map[string]interface{}{
		"speed":        float64(30),
		"acceleration": float64(50),
	}
	if err := r.MoveToJointPositions(context.Background(), positions, extra); err != nil {
		t.Fatal(err)
	}
	// speed=30 deg/s * 10 = 300 units
	if fc.LastSpeed != 300 {
		t.Fatalf("expected speed=300, got %d", fc.LastSpeed)
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
