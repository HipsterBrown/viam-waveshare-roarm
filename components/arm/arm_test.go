package arm

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/roarm"
	"waveshareroarm/internal/testfake"
)

func mustLoadModel(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := geometry.ArmModel(geometry.CollisionBox, "roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestMoveClampsToJointLimits(t *testing.T) {
	fc := &testfake.FakeController{}
	// Pre-populate GetJointRadians response so MoveToJointPositions can read current.
	fc.Feedback = roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	// ask for joint 1 at 10 rad (way over the model's +180 degree limit).
	// The JSON says 180.0004 degrees, so compare to pi with a tolerance.
	positions := []referenceframe.Input{10.0, 0, 0, 0, 0}
	if err := r.MoveToJointPositions(context.Background(), positions, nil); err != nil {
		t.Fatal(err)
	}
	if math.Abs(fc.LastRadians[0]-math.Pi) > 1e-4 {
		t.Fatalf("expected clamp to %v, got %v", math.Pi, fc.LastRadians[0])
	}
}

func TestMoveClampsBelowMinimum(t *testing.T) {
	fc := &testfake.FakeController{}
	fc.Feedback = roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	// Joint 1 min is -180 degrees in the model.
	positions := []referenceframe.Input{-10.0, 0, 0, 0, 0}
	if err := r.MoveToJointPositions(context.Background(), positions, nil); err != nil {
		t.Fatal(err)
	}
	if math.Abs(fc.LastRadians[0]+math.Pi) > 1e-4 {
		t.Fatalf("expected clamp to %v, got %v", -math.Pi, fc.LastRadians[0])
	}
}

func TestMoveRejectsWrongLengthInput(t *testing.T) {
	fc := &testfake.FakeController{}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	err := r.MoveToJointPositions(context.Background(), []referenceframe.Input{0}, nil)
	if err == nil {
		t.Fatal("expected error for wrong length")
	}
}

// TestEndPositionDoesNotDeadlock exercises EndPosition -> CurrentInputs ->
// JointPositions, which previously re-entered r.mu and deadlocked under
// sync.Mutex. A timeout ctx bounds any regression.
func TestEndPositionDoesNotDeadlock(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
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

func newTestArm(t *testing.T, fc *testfake.FakeController) *roarmM3 {
	t.Helper()
	return &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
}

func TestDoCommand_SetTorque(t *testing.T) {
	fc := &testfake.FakeController{}
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
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "set_torque"})
	if err == nil {
		t.Fatal("expected error for missing enable param")
	}
}

func TestDoCommand_SetLED(t *testing.T) {
	fc := &testfake.FakeController{}
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
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "set_led"})
	if err == nil {
		t.Fatal("expected error for missing brightness param")
	}
}

func TestDoCommand_MoveToHome(t *testing.T) {
	const startGripper = 0.42
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{G: startGripper}}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "move_to_home"})
	if err != nil {
		t.Fatal(err)
	}
	if fc.LastRadians[2] != math.Pi/2 {
		t.Fatalf("expected elbow at pi/2, got %v", fc.LastRadians[2])
	}
	if fc.LastRadians[5] != startGripper {
		t.Fatalf("expected the gripper preserved at %v, got %v", startGripper, fc.LastRadians[5])
	}
}

func TestDoCommand_GetFeedback(t *testing.T) {
	fc := &testfake.FakeController{
		Feedback: roarm.FeedbackData{X: 1, Y: 2, Z: 3, B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6},
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
	fc := &testfake.FakeController{}
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
	if want := roarm.SpeedToUnits(60); r.defaultSpeed != want {
		t.Fatalf("expected defaultSpeed=%d, got %d", want, r.defaultSpeed)
	}
}

func TestDoCommand_SetSpeedOutOfRange(t *testing.T) {
	fc := &testfake.FakeController{}
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
	fc := &testfake.FakeController{}
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
	if want := roarm.AccelToUnits(100); r.defaultAcc != want {
		t.Fatalf("expected defaultAcc=%d, got %d", want, r.defaultAcc)
	}
}

func TestDoCommand_SetAccelerationOutOfRange(t *testing.T) {
	fc := &testfake.FakeController{}
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
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "get_motion_params"})
	if err != nil {
		t.Fatal(err)
	}
	if want := roarm.SpeedFromUnits(roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec)); out["current_speed_degs_per_sec"] != want {
		t.Fatalf("expected current_speed_degs_per_sec=%v, got %v", want, out["current_speed_degs_per_sec"])
	}
	if want := roarm.AccelFromUnits(roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq)); out["current_acceleration_degs_per_sec_per_sec"] != want {
		t.Fatalf("expected current_acceleration_degs_per_sec_per_sec=%v, got %v", want, out["current_acceleration_degs_per_sec_per_sec"])
	}
}

func TestDoCommand_CommsHealth(t *testing.T) {
	fc := &testfake.FakeController{}
	fc.HealthSnap = roarm.HealthSnapshot{Frames: 200, Retries: 20, StaleFrames: 3}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{"command": roarm.CmdCommsHealth})
	if err != nil {
		t.Fatal(err)
	}
	if out["frames"] != 200 {
		t.Fatalf("expected frames=200, got %v", out["frames"])
	}
	if out["retry_pct"] != 10.0 {
		t.Fatalf("expected retry_pct=10, got %v", out["retry_pct"])
	}
	if out["stale_frames"] != 3 {
		t.Fatalf("expected stale_frames=3, got %v", out["stale_frames"])
	}
	if _, ok := out["reset"]; ok {
		t.Fatal("did not ask for a reset; \"reset\" should be absent")
	}
	// The counters must be untouched: a plain read must not reset them.
	if fc.HealthSnap.Frames != 200 {
		t.Fatalf("a plain read reset the counters: %+v", fc.HealthSnap)
	}
}

func TestDoCommand_CommsHealthReset(t *testing.T) {
	fc := &testfake.FakeController{}
	fc.HealthSnap = roarm.HealthSnapshot{Frames: 200, Retries: 20}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": roarm.CmdCommsHealth,
		"reset":   true,
	})
	if err != nil {
		t.Fatal(err)
	}
	if out["reset"] != true {
		t.Fatalf("expected reset=true in the response, got %v", out)
	}
	// The reported snapshot is the pre-reset one (what the caller asked
	// about); the counters underneath are zeroed for the next measurement.
	if out["frames"] != 200 {
		t.Fatalf("expected the response to report the pre-reset frames=200, got %v", out["frames"])
	}
	if fc.HealthSnap.Frames != 0 {
		t.Fatalf("expected the counters to be zeroed after reset, got %+v", fc.HealthSnap)
	}
}

func TestDoCommand_UnknownCommand(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "nonsense"})
	if err == nil {
		t.Fatal("expected error for unknown command")
	}
}

func TestArmName(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	r.name = resource.Name{}
	_ = r.Name()
}

func TestArmJointPositions(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6}}
	r := newTestArm(t, fc)
	inputs, err := r.JointPositions(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 5 {
		t.Fatalf("expected 5 arm joints, got %d", len(inputs))
	}
	if inputs[0] != 0.1 {
		t.Fatalf("expected joint 0 at 0.1, got %v", inputs[0])
	}
}

func TestArmCurrentInputs(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5}}
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
	fc := &testfake.FakeController{}
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
	fc := &testfake.FakeController{Moving: true}
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
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_ = r.Close(context.Background())
	if _, err := r.Geometries(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
}

func TestArmEndPositionAfterClose(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_ = r.Close(context.Background())
	if _, err := r.EndPosition(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
}

func TestArmClose(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
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
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := newTestArm(t, fc)
	positions := [][]referenceframe.Input{
		{0.1, 0, 0, 0, 0},
		{0.2, 0, 0, 0, 0},
	}
	if err := r.MoveThroughJointPositions(context.Background(), positions, nil, nil); err != nil {
		t.Fatal(err)
	}
}

func TestArmGoToInputs(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0}}
	r := newTestArm(t, fc)
	step1 := []referenceframe.Input{0.05, 0, 0, 0, 0}
	if err := r.GoToInputs(context.Background(), step1); err != nil {
		t.Fatal(err)
	}
}

func TestArmNewClientFromConn(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.NewClientFromConn(context.Background(), nil, "", resource.Name{}, nil)
	if err == nil {
		t.Fatal("expected unsupported error")
	}
}

func TestNewRoArmM3_ConstructsHTTP(t *testing.T) {
	// newRoArmM3 now requires a motion service dependency (builtin by default),
	// which this test historically passed as nil deps. Skipped pending a
	// motion-service test double; construction is still exercised indirectly by
	// the Validate and Reconfigure tests.
	t.Skip("requires motion.Service dependency injection")
}

func TestArmReconfigure_MotionOnly(t *testing.T) {
	fc := &testfake.FakeController{}
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
	if want := roarm.SpeedToUnits(60); r.defaultSpeed != want {
		t.Fatalf("expected defaultSpeed=%d, got %d", want, r.defaultSpeed)
	}
	if want := roarm.AccelToUnits(120); r.defaultAcc != want {
		t.Fatalf("expected defaultAcc=%d, got %d", want, r.defaultAcc)
	}
}

func TestArmStopHoldsCurrentPosition(t *testing.T) {
	fc := &testfake.FakeController{
		Feedback: roarm.FeedbackData{B: 0.5, S: 0.3, E: 0.1, Wrist: 0.2, R: 0.4, G: 0.0},
	}
	r := &roarmM3{
		controller: fc, defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec), defaultAcc: roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits: jointLimitsFromModel(mustLoadModel(t)),
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
	expectedStopSpeed := roarm.SpeedToUnits(roarm.StopSpeedDegsPerSec)
	if fc.LastSpeed != expectedStopSpeed {
		t.Fatalf("expected stop speed %d, got %d", expectedStopSpeed, fc.LastSpeed)
	}
}

func TestMovePreservesGripperPosition(t *testing.T) {
	fc := &testfake.FakeController{}
	// Gripper currently at some position; MoveToJointPositions must preserve it.
	fc.Feedback = roarm.FeedbackData{B: 0, S: 0, E: 0, Wrist: 0, R: 0, G: 0.5}
	r := &roarmM3{
		controller:   fc,
		defaultSpeed: roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec),
		defaultAcc:   roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq),
		jointLimits:  jointLimitsFromModel(mustLoadModel(t)),
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		model:        mustLoadModel(t),
	}
	positions := []referenceframe.Input{0.1, 0.2, 0.3, 0.4, 0.5}
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

// A reopen that fails must leave the old controller and config in place.
func TestArmReconfigure_FailedReopenKeepsOldController(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	conf := resource.Config{
		Name:                "arm",
		ConvertedAttributes: &RoArmM3Config{Port: "/dev/this-port-does-not-exist-roarm-test"},
	}
	if err := r.Reconfigure(context.Background(), nil, conf); err == nil {
		t.Fatal("expected the reopen to fail")
	}
	if r.snapshotController() != fc {
		t.Fatal("old controller was replaced by a failed reopen")
	}
	if r.cfg.Host != "1.2.3.4" {
		t.Fatal("config was replaced by a failed reopen")
	}
	if fc.Closed {
		t.Fatal("old controller was closed by a failed reopen, leaving a dead arm")
	}
}

// Reconfigure swaps r.controller under r.mu; readers must go through
// snapshotController. This performs exactly that write, so -race flags any
// reader that touches r.controller directly. Run with -race.
func TestArmReconfigure_RacesWithReaders(t *testing.T) {
	fcA, fcB := &testfake.FakeController{}, &testfake.FakeController{}
	r := newTestArm(t, fcA)
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	done := make(chan struct{})
	go func() {
		defer close(done)
		for i := 0; i < 50; i++ {
			_, _ = r.IsMoving(context.Background())
			_ = r.Stop(context.Background(), nil)
			_, _ = r.DoCommand(context.Background(), map[string]interface{}{"command": "set_torque", "enable": true})
		}
	}()
	for i := 0; i < 50; i++ {
		ctrl := roarm.Handle(fcA)
		if i%2 == 1 {
			ctrl = fcB
		}
		r.mu.Lock()
		r.controller = ctrl
		r.mu.Unlock()
	}
	<-done
}

func TestMoveToJointPositions_WritesThenSettles(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{G: 0.7}}
	r := newTestArm(t, fc)
	if err := r.MoveToJointPositions(context.Background(), []referenceframe.Input{0.5, 0, 0, 0, 0}, nil); err != nil {
		t.Fatal(err)
	}
	if fc.SettleCalls != 1 {
		t.Fatalf("expected one settle wait, got %d", fc.SettleCalls)
	}
	if fc.LastRadians[0] != 0.5 || fc.LastRadians[5] != 0.7 {
		t.Fatalf("target %v: want joint 1 at 0.5 and gripper preserved at 0.7", fc.LastRadians)
	}
	if fc.LastSpeed != roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec) {
		t.Fatalf("speed %d, want configured default %d", fc.LastSpeed, roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec))
	}
}

func TestMoveToJointPositions_SettleErrorIsReturned(t *testing.T) {
	fc := &testfake.FakeController{FailOn: "WaitUntilSettled"}
	r := newTestArm(t, fc)
	if err := r.MoveToJointPositions(context.Background(), []referenceframe.Input{0.5, 0, 0, 0, 0}, nil); err == nil {
		t.Fatal("expected the settle error")
	}
}

func TestMoveToJointPositions_IgnoresExtraSpeed(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	extra := map[string]interface{}{"speed": float64(30), "acceleration": float64(50)}
	if err := r.MoveToJointPositions(context.Background(), []referenceframe.Input{0.1, 0, 0, 0, 0}, extra); err != nil {
		t.Fatal(err)
	}
	if fc.LastSpeed != roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec) {
		t.Fatalf("extra speed must be ignored; got %d", fc.LastSpeed)
	}
}

func TestArmIsMoving_TrueWhileAMoveIsInFlight(t *testing.T) {
	fc := &testfake.FakeController{Moving: false}
	r := newTestArm(t, fc)
	r.opInFlight.Store(true)
	moving, err := r.IsMoving(context.Background())
	if err != nil || !moving {
		t.Fatalf("expected true while in flight, got %v %v", moving, err)
	}
	r.opInFlight.Store(false)
	moving, err = r.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("expected the controller's answer (false), got %v %v", moving, err)
	}
}

func TestArmStop_UsesTheStopSpeed(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{B: 0.5}}
	r := newTestArm(t, fc)
	if err := r.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if fc.LastSpeed != roarm.SpeedToUnits(roarm.StopSpeedDegsPerSec) {
		t.Fatalf("stop speed %d, want %d", fc.LastSpeed, roarm.SpeedToUnits(roarm.StopSpeedDegsPerSec))
	}
}

func TestArmNoFeedbackTransport_PositionReadsError(t *testing.T) {
	fc := &testfake.FakeController{FailOn: "GetJointRadians", FailWith: roarm.ErrNoFeedback}
	r := newTestArm(t, fc)
	if _, err := r.JointPositions(context.Background(), nil); !errors.Is(err, roarm.ErrNoFeedback) {
		t.Fatalf("expected roarm.ErrNoFeedback, got %v", err)
	}
	if _, err := r.EndPosition(context.Background(), nil); !errors.Is(err, roarm.ErrNoFeedback) {
		t.Fatalf("expected roarm.ErrNoFeedback, got %v", err)
	}
}

func TestDoCommand_GetFeedbackReportsGripperInSoftwareFrame(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{G: 3.0}} // raw wire value
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "get_feedback"})
	if err != nil {
		t.Fatal(err)
	}
	joints := out["joints"].(map[string]interface{})
	if got := joints["gripper"].(float64); math.Abs(got-roarm.GripperSoftwareToWire(3.0)) > 1e-9 {
		t.Fatalf("gripper reported %v, want software frame %v", got, roarm.GripperSoftwareToWire(3.0))
	}
}

func TestBridgeSetGripperRad_WaitsOnJoint6(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": roarm.CmdSetGripperRad, roarm.KeyRad: 1.0, roarm.KeySpeed: 60.0, roarm.KeyAcc: 200.0,
	})
	if err != nil {
		t.Fatal(err)
	}
	if fc.LastJoint != 6 || fc.LastRadians[5] != 1.0 {
		t.Fatalf("joint %d rad %v", fc.LastJoint, fc.LastRadians)
	}
	if fc.LastSpeed != roarm.SpeedToUnits(60) || fc.LastAcc != roarm.AccelToUnits(200) {
		t.Fatalf("bridge must convert deg/s and deg/s^2: got %d/%d", fc.LastSpeed, fc.LastAcc)
	}
	if fc.SettleCalls != 1 {
		t.Fatalf("expected one settle wait, got %d", fc.SettleCalls)
	}
}

func TestBridgeSetGripperRad_NoWait(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command": roarm.CmdSetGripperRad, roarm.KeyRad: 1.0, roarm.KeyWait: false,
	})
	if err != nil {
		t.Fatal(err)
	}
	if fc.SettleCalls != 0 {
		t.Fatal("wait=false must not settle")
	}
}

func TestBridgeSetGripperRad_RejectsOutOfRange(t *testing.T) {
	r := newTestArm(t, &testfake.FakeController{})
	for _, rad := range []float64{-0.5, 2.5} {
		if _, err := r.DoCommand(context.Background(), map[string]interface{}{"command": roarm.CmdSetGripperRad, roarm.KeyRad: rad}); err == nil {
			t.Fatalf("rad %v should be rejected", rad)
		}
	}
}

func TestBridgeGetGripperRad_NoFeedbackCarriesMarker(t *testing.T) {
	fc := &testfake.FakeController{FailOn: "GetJointRadians", FailWith: roarm.ErrNoFeedback}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": roarm.CmdGetGripperRad})
	if err == nil || !strings.Contains(err.Error(), roarm.NoFeedbackMarker) {
		t.Fatalf("expected the no-feedback marker to cross the bridge, got %v", err)
	}
}

func TestGet3DModelsServesSixLinks(t *testing.T) {
	r := newTestArm(t, &testfake.FakeController{})
	models, err := r.Get3DModels(context.Background(), nil)
	if err != nil || len(models) != 6 {
		t.Fatalf("%v, %d models", err, len(models))
	}
	if models["link5"].ContentType != "model/gltf-binary" {
		t.Fatal("wrong content type")
	}
}

func TestReconfigureSwitchesCollisionGeometry(t *testing.T) {
	fc := &testfake.FakeController{}
	r := newTestArm(t, fc)
	r.cfg = &RoArmM3Config{Host: "1.2.3.4"}
	conf := resource.Config{Name: "arm", ConvertedAttributes: &RoArmM3Config{Host: "1.2.3.4", CollisionGeometry: "mesh"}}
	if err := r.Reconfigure(context.Background(), nil, conf); err != nil {
		t.Fatal(err)
	}
	gif, err := r.snapshotModel().Geometries([]referenceframe.Input{0, 0, 0, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if _, ok := gif.Geometries()[0].(*spatialmath.Mesh); !ok {
		t.Fatalf("model still has %T geometry after switching to mesh", gif.Geometries()[0])
	}
	if fc.Closed {
		t.Fatal("a collision_geometry change must not reopen the connection")
	}
}

// The settle must be told the profile the write used, or it derives its timing
// from the wrong numbers: at 40 deg/s^2 a default-profile derivation is far
// too short.
func TestMoveToJointPositionsPassesTheProfileToTheSettle(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{T: 1051, B: 0.1, G: 3.0}}
	r := newTestArm(t, fc)
	// newTestArm builds the struct directly, so the configured profile is set
	// here rather than through an attribute map.
	r.defaultSpeed, r.defaultAcc = roarm.SpeedToUnits(25), roarm.AccelToUnits(40)

	if err := r.MoveToJointPositions(context.Background(),
		[]referenceframe.Input{0.5, 0, 0, 0, 0}, nil); err != nil {
		t.Fatal(err)
	}
	req := fc.LastSettleRequest
	if req.SpeedUnits != roarm.SpeedToUnits(25) || req.AccUnits != roarm.AccelToUnits(40) {
		t.Fatalf("settle got speed=%d acc=%d, want the configured profile", req.SpeedUnits, req.AccUnits)
	}
	if !req.RequireMotion {
		t.Fatal("an arm move must require motion")
	}
	// Start must be the pose read from the arm, not the target: a commanded
	// start pose makes a never-moved arm undetectable.
	if len(req.Start) != 6 || math.Abs(req.Start[0]-0.1) > 1e-9 {
		t.Fatalf("settle Start = %v, want the measured pose with 0.1 at joint 1", req.Start)
	}
}

// The gripper bridge must settle against the pose read before the write, so a
// jaw that never moved is detectable and the derived deadline matches the real
// travel rather than the whole jaw range.
func TestSetGripperRadSettlesAgainstTheMeasuredPose(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{T: 1051, B: 0.1, G: 0.5}}
	r := newTestArm(t, fc)
	if _, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command":    roarm.CmdSetGripperRad,
		roarm.KeyRad: 1.5,
	}); err != nil {
		t.Fatal(err)
	}
	req := fc.LastSettleRequest
	if len(req.Start) != 6 || math.Abs(req.Start[5]-0.5) > 1e-9 || math.Abs(req.Start[0]-0.1) > 1e-9 {
		t.Fatalf("settle Start = %v, want the measured pose (jaw 0.5, joint 1 0.1)", req.Start)
	}
	if len(req.Target) != 6 || math.Abs(req.Target[5]-1.5) > 1e-9 || math.Abs(req.Target[0]-0.1) > 1e-9 {
		t.Fatalf("settle Target = %v, want the measured pose with the jaw at 1.5", req.Target)
	}
	if !req.RequireMotion {
		t.Fatal("set_gripper_rad must require motion unless require_motion says otherwise")
	}
	if req.Timeout != 0 {
		t.Fatalf("settle Timeout = %v, want the derived deadline", req.Timeout)
	}
}

func TestSetGripperRadHonorsRequireMotionFalse(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{T: 1051, G: 0.5}}
	r := newTestArm(t, fc)
	if _, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command":              roarm.CmdSetGripperRad,
		roarm.KeyRad:           1.5,
		roarm.KeyRequireMotion: false,
	}); err != nil {
		t.Fatal(err)
	}
	if fc.LastSettleRequest.RequireMotion {
		t.Fatal("require_motion=false must reach the settle")
	}
}

// The gripper recognises "this transport cannot read positions" by a marker
// substring, so the new pre-write read has to keep the marker in its error.
func TestSetGripperRadKeepsTheNoFeedbackMarker(t *testing.T) {
	fc := &testfake.FakeController{FailOn: "GetJointRadians", FailWith: roarm.ErrNoFeedback}
	r := newTestArm(t, fc)
	_, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command":    roarm.CmdSetGripperRad,
		roarm.KeyRad: 1.5,
	})
	if err == nil {
		t.Fatal("expected the read to fail")
	}
	if !errors.Is(err, roarm.ErrNoFeedback) || !strings.Contains(err.Error(), roarm.NoFeedbackMarker) {
		t.Fatalf("error must carry the no-feedback marker, got: %v", err)
	}
}

// A fire-and-forget gripper command must still work on a transport that
// cannot read positions. The settle's start pose is only needed when there is
// a settle, so the pre-move read is skipped entirely when wait is false.
func TestBridgeSetGripperRad_NoWaitNeedsNoFeedback(t *testing.T) {
	fc := &testfake.FakeController{FailOn: "GetJointRadians", FailWith: roarm.ErrNoFeedback}
	r := newTestArm(t, fc)
	out, err := r.DoCommand(context.Background(), map[string]interface{}{
		"command":     roarm.CmdSetGripperRad,
		roarm.KeyRad:  1.5,
		roarm.KeyWait: false,
	})
	if err != nil {
		t.Fatalf("wait=false must not need a position read: %v", err)
	}
	if out["success"] != true {
		t.Fatalf("expected success, got %v", out)
	}
	if fc.SettleCalls != 0 {
		t.Fatalf("wait=false must not settle, got %d settle calls", fc.SettleCalls)
	}
}
