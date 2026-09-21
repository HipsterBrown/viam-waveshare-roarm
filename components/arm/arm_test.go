package arm

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/planning"
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

// MoveThroughJointPositions must route the resolved profile all the way to the
// write and the settle. Delegating to the public MoveToJointPositions
// re-snapshots the configured defaults and discards it.
func TestMoveThroughJointPositionsRoutesMoveOptionsToTheWrite(t *testing.T) {
	fc := &testfake.FakeController{Feedback: roarm.FeedbackData{T: 1051, G: 3.0}}
	r := newTestArm(t, fc) // configured defaults: 50 deg/s, 100 deg/s^2
	opts := &rdkarm.MoveOptions{MaxVelRads: 10 * math.Pi / 180}
	if err := r.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{{0.2, 0, 0, 0, 0}}, opts, nil); err != nil {
		t.Fatal(err)
	}
	if got := fc.LastSpeed; got != roarm.SpeedToUnits(10) {
		t.Fatalf("the write used speed %d units, want %d (10 deg/s)", got, roarm.SpeedToUnits(10))
	}
	if got := fc.LastSettleRequest.SpeedUnits; got != roarm.SpeedToUnits(10) {
		t.Fatalf("the settle got speed %d units, want the same %d the write used", got, roarm.SpeedToUnits(10))
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
	conf := resource.Config{
		Name:                "arm",
		API:                 resource.APINamespace("rdk").WithType("component").WithSubtype("arm"),
		ConvertedAttributes: &RoArmM3Config{Host: "127.0.0.1:0"},
	}
	deps := resource.Dependencies{motion.Named("builtin"): &fakeMotion{}}
	// Should construct successfully in HTTP mode (no connection is attempted).
	armRes, err := newRoArmM3(context.Background(), deps, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	defer armRes.Close(context.Background())
	if armRes.Name().Name != "arm" {
		t.Fatalf("expected name=arm, got %v", armRes.Name())
	}
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

// fakeMotion is the smallest motion.Service that records the last MoveReq. The embedded
// interface is nil: only Move is ever called, and any other method panicking is the
// correct answer for a stub.
type fakeMotion struct {
	motion.Service
	last    motion.MoveReq
	calls   int
	moveErr error
}

func (f *fakeMotion) Move(ctx context.Context, req motion.MoveReq) (bool, error) {
	f.calls++
	f.last = req
	if f.moveErr != nil {
		return false, f.moveErr
	}
	return true, nil
}

// newPlanningArm is newTestArm plus the two things MoveToPosition needs: a name (the
// destination frame is derived from it) and a motion service to plan against.
func newPlanningArm(t *testing.T, cfg *RoArmM3Config) (*roarmM3, *fakeMotion) {
	t.Helper()
	r := newTestArm(t, &testfake.FakeController{})
	r.name = rdkarm.Named("myarm")
	fm := &fakeMotion{}
	r.motion = fm
	r.cfg = cfg
	r.goalCloud = planning.ResolveGoalCloudConfig(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM, r.logger)
	return r, fm
}

var testGoalPose = spatialmath.NewPose(
	r3.Vector{X: 300, Y: 0, Z: 200},
	&spatialmath.OrientationVectorDegrees{OZ: -1},
)

// Row 1: with no extra, the destination carries the cone built from the config, and no
// goal_metric_type reaches the planner (the hardcoded position_only is gone).
func TestMoveToPositionSendsTheConfiguredCone(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{OrientationToleranceDeg: 15, PositionToleranceMM: 2})
	if err := r.MoveToPosition(context.Background(), testGoalPose, nil); err != nil {
		t.Fatal(err)
	}
	dest := fm.last.Destination
	if dest.Parent() != "myarm_origin" {
		t.Errorf("destination frame = %q, want %q", dest.Parent(), "myarm_origin")
	}
	if dest.GoalCloud == nil {
		t.Fatal("no goal cloud on the destination: orientation would still be ignored")
	}
	// The configured tolerances, not the defaults, must be what reaches the planner.
	if dest.GoalCloud.X != 2 || dest.GoalCloud.Y != 2 || dest.GoalCloud.Z != 2 {
		t.Errorf("positional leeway = (%v, %v, %v), want 2 on each axis",
			dest.GoalCloud.X, dest.GoalCloud.Y, dest.GoalCloud.Z)
	}
	if want := 1 - math.Cos(15*math.Pi/180); math.Abs(dest.GoalCloud.OZ-want) > 1e-12 {
		t.Errorf("OZ = %v, want %v (a 15deg cone)", dest.GoalCloud.OZ, want)
	}
	if _, ok := fm.last.Extra["goal_metric_type"]; ok {
		t.Error("goal_metric_type must no longer be sent: the cone replaces position_only")
	}
}

// An unset pair resolves to the package defaults rather than a zero-leeway cloud no IK
// solution lands inside.
func TestMoveToPositionDefaultsTheConeWhenUnset(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{})
	if err := r.MoveToPosition(context.Background(), testGoalPose, nil); err != nil {
		t.Fatal(err)
	}
	dest := fm.last.Destination
	// Guard before dereferencing: a missing cloud is the exact regression this test
	// catches, and a nil deref would take the whole test binary down with it.
	if dest.GoalCloud == nil {
		t.Fatal("no goal cloud on the destination: orientation would still be ignored")
	}
	if dest.GoalCloud.X != 1.0 {
		t.Errorf("positional leeway = %v, want the 1.0mm default", dest.GoalCloud.X)
	}
	if want := 1 - math.Cos(30*math.Pi/180); math.Abs(dest.GoalCloud.OZ-want) > 1e-12 {
		t.Errorf("OZ = %v, want %v (the 30deg default cone)", dest.GoalCloud.OZ, want)
	}
}

// Row 2: the caller's goal_metric_type wins. No cloud is sent, and the key reaches the
// planner so the old position_only behavior is still available.
func TestMoveToPositionHonorsGoalMetricTypeInExtra(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{})
	extra := map[string]interface{}{"goal_metric_type": "position_only"}
	if err := r.MoveToPosition(context.Background(), testGoalPose, extra); err != nil {
		t.Fatal(err)
	}
	dest := fm.last.Destination
	if dest.GoalCloud != nil {
		t.Error("no cloud may be sent with position_only: orientScale=0 makes its leeways meaningless")
	}
	if got := fm.last.Extra["goal_metric_type"]; got != "position_only" {
		t.Errorf("goal_metric_type = %v, want it forwarded to the planner", got)
	}
}

// Row 3: a raw pose_cloud replaces the cone, and pose_cloud is consumed rather than
// forwarded (it is not a planner key).
func TestMoveToPositionHonorsPoseCloudInExtra(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{})
	extra := map[string]interface{}{
		"pose_cloud": map[string]interface{}{"x": 5.0, "oz": 0.25, "theta": 10.0},
	}
	if err := r.MoveToPosition(context.Background(), testGoalPose, extra); err != nil {
		t.Fatal(err)
	}
	dest := fm.last.Destination
	if dest.GoalCloud == nil {
		t.Fatal("the caller's pose_cloud must reach the destination")
	}
	if dest.GoalCloud.X != 5.0 || dest.GoalCloud.OZ != 0.25 || dest.GoalCloud.Theta != 10.0 {
		t.Errorf("goal cloud = %+v, want the caller's cloud verbatim", *dest.GoalCloud)
	}
	if _, ok := fm.last.Extra["pose_cloud"]; ok {
		t.Error("pose_cloud is consumed here, not a planner key")
	}
}

// Row 4: both keys together are incoherent, so the move is rejected before the planner is
// ever called.
func TestMoveToPositionRejectsBothExtraKeys(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{})
	err := r.MoveToPosition(context.Background(), testGoalPose, map[string]interface{}{
		"pose_cloud":       map[string]interface{}{"oz": 0.5},
		"goal_metric_type": "position_only",
	})
	if err == nil {
		t.Fatal("expected an error for pose_cloud plus goal_metric_type")
	}
	if !strings.Contains(err.Error(), "pose_cloud") || !strings.Contains(err.Error(), "goal_metric_type") {
		t.Errorf("the error must name both offending keys: %v", err)
	}
	if fm.calls != 0 {
		t.Errorf("motion.Move was called %d times; an incoherent request must not reach the planner", fm.calls)
	}
}

// A planning failure on the cone path must point the caller at both tolerances and at the
// escape hatch, since this is a breaking change: goals that planned under position_only
// may now fail.
func TestMoveToPositionFailureNamesBothTolerances(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{OrientationToleranceDeg: 15, PositionToleranceMM: 2})
	fm.moveErr = errors.New("no IK solution")
	err := r.MoveToPosition(context.Background(), testGoalPose, nil)
	if err == nil {
		t.Fatal("expected the planner's error to propagate")
	}
	for _, want := range []string{
		"orientation_tolerance_deg=15", "position_tolerance_mm=2",
		"no IK solution", "goal_metric_type", "0.127.0",
	} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error must mention %q: %v", want, err)
		}
	}
}

// Reconfigure must pick the tolerances up: they sit beside the other reconfigurable motion
// settings, so a config edit that does not rebuild the arm still has to take effect.
func TestArmReconfigurePicksUpGoalCloudTolerances(t *testing.T) {
	r, fm := newPlanningArm(t, &RoArmM3Config{Host: "1.2.3.4"})
	conf := resource.Config{
		Name: "arm",
		ConvertedAttributes: &RoArmM3Config{
			Host:                    "1.2.3.4",
			OrientationToleranceDeg: 45,
			PositionToleranceMM:     3,
		},
	}
	if err := r.Reconfigure(context.Background(), nil, conf); err != nil {
		t.Fatal(err)
	}
	if err := r.MoveToPosition(context.Background(), testGoalPose, nil); err != nil {
		t.Fatal(err)
	}
	dest := fm.last.Destination
	if dest.GoalCloud.X != 3 {
		t.Errorf("positional leeway = %v, want the reconfigured 3", dest.GoalCloud.X)
	}
	if want := 1 - math.Cos(45*math.Pi/180); math.Abs(dest.GoalCloud.OZ-want) > 1e-12 {
		t.Errorf("OZ = %v, want %v (the reconfigured 45deg cone)", dest.GoalCloud.OZ, want)
	}
}

func TestArmValidateRejectsBadGoalCloudTolerances(t *testing.T) {
	for name, cfg := range map[string]*RoArmM3Config{
		"orientation above 180": {Host: "h", OrientationToleranceDeg: 181},
		"negative orientation":  {Host: "h", OrientationToleranceDeg: -1},
		"negative position":     {Host: "h", PositionToleranceMM: -1},
		"NaN orientation":       {Host: "h", OrientationToleranceDeg: math.NaN()},
		"NaN position":          {Host: "h", PositionToleranceMM: math.NaN()},
	} {
		t.Run(name, func(t *testing.T) {
			if _, _, err := cfg.Validate("p"); err == nil {
				t.Error("want a validation error, got nil")
			}
		})
	}
	if _, _, err := (&RoArmM3Config{Host: "h", OrientationToleranceDeg: 180, PositionToleranceMM: 0}).Validate("p"); err != nil {
		t.Errorf("180 degrees and an unset position tolerance are both legal: %v", err)
	}
}
