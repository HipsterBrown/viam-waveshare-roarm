package gripper

import (
	"context"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/roarm"
	"waveshareroarm/internal/testfake"
)

// newTestGripper builds a gripper wired to a testfake.FakeArmRPC client, mirroring
// what the constructor does in production (where rdkarm.FromDependencies
// returns an rdkarm.Arm gRPC client, not the local *roarmM3 struct).
func newTestGripper(t *testing.T, fa *testfake.FakeArmRPC) *roarmM3Gripper {
	t.Helper()
	return &roarmM3Gripper{
		armClient: fa,
		logger:    logging.NewTestLogger(t),
		model:     mustBuildGripperModel(t),
		opMgr:     operation.NewSingleOperationManager(),
		sleep:     func(context.Context, time.Duration) error { return nil },
	}
}

func mustBuildGripperModel(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := geometry.GripperModel("test-gripper")
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestGripperOpenClearsHolding(t *testing.T) {
	fa := &testfake.FakeArmRPC{Joint6Rad: 0.3, HoldStill: true}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	_, _ = g.Grab(ctx, nil)
	if !g.holding.Load() {
		t.Fatal("expected holding after grab")
	}
	_ = g.Open(ctx, nil)
	if g.holding.Load() {
		t.Fatal("expected holding=false after open")
	}
}

func TestGripperGetPosition(t *testing.T) {
	// Joint6Rad=0 (software frame) → 0 degrees.
	fa := &testfake.FakeArmRPC{Joint6Rad: 0}
	g := newTestGripper(t, fa)
	pos, err := g.GetPosition(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if pos != 0 {
		t.Fatalf("expected 0 degrees, got %v", pos)
	}
}

func TestGripperStop(t *testing.T) {
	fa := &testfake.FakeArmRPC{Joint6Rad: 0.7}
	g := newTestGripper(t, fa)
	if err := g.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if fa.StopCalls != 1 {
		t.Fatalf("expected 1 stop_gripper call, got %d", fa.StopCalls)
	}
	if fa.LastCommand != "stop_gripper" {
		t.Fatalf("expected last command stop_gripper, got %q", fa.LastCommand)
	}
}

func TestGripperKinematicsIsZeroDoF(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	m, err := g.Kinematics(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if m == nil {
		t.Fatal("expected non-nil kinematics")
	}
	if len(m.DoF()) != 0 {
		t.Fatalf("expected 0 DoF, got %d", len(m.DoF()))
	}
}

func TestGripperGeometries(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	geos, err := g.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(geos) != 1 {
		t.Fatalf("expected 1 geometry, got %d", len(geos))
	}
	if !strings.HasSuffix(geos[0].Label(), ":body") {
		t.Fatalf("expected a label ending in :body, got %q", geos[0].Label())
	}
}

func TestGripperCurrentInputs(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{Joint6Rad: 0})
	inputs, err := g.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 0 {
		t.Fatalf("expected 0 inputs, got %d", len(inputs))
	}
}

func TestGripperClose(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	if err := g.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
	// After close, Open should return errGripperClosed.
	if err := g.Open(context.Background(), nil); err == nil {
		t.Fatal("expected error after close")
	}
	// Second close is idempotent.
	if err := g.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestGripperDoCommand_GetPosition(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{Joint6Rad: 0})
	out, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "get_position"})
	if err != nil {
		t.Fatal(err)
	}
	if out["position_degrees"] != 0.0 {
		t.Fatalf("expected position_degrees=0, got %v", out["position_degrees"])
	}
}

func TestGripperDoCommand_Unknown(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "nonsense"})
	if err == nil {
		t.Fatal("expected error for unknown command")
	}
}

func TestGripperSetPosition_CancelledContext(t *testing.T) {
	fa := &testfake.FakeArmRPC{}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_ = g.SetPosition(ctx, 50, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq)
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperName(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	_ = g.Name()
}

func TestGripperDoCommand_SetPosition(t *testing.T) {
	fa := &testfake.FakeArmRPC{}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	out, err := g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position",
		"degrees": float64(50),
		"speed":   float64(50),
		"acc":     float64(100),
	})
	_ = out
	_ = err
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperDoCommand_SetPositionMissingDegrees(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "set_position"})
	if err == nil {
		t.Fatal("expected error for missing degrees")
	}
}

func TestGripperGoToInputs_Empty(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	if err := g.GoToInputs(context.Background()); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
}

func TestGripperGoToInputs_RejectsNonEmpty(t *testing.T) {
	fa := &testfake.FakeArmRPC{}
	g := newTestGripper(t, fa)
	if err := g.GoToInputs(context.Background(), []referenceframe.Input{0.5}); err == nil {
		t.Fatal("expected an error for a non-empty input set")
	}
	if fa.LastCommand != "" {
		t.Fatalf("expected no command dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperAfterClose_ReturnErrors(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	_ = g.Close(context.Background())
	if _, err := g.Grab(context.Background(), nil); err == nil {
		t.Fatal("expected error")
	}
	if err := g.Stop(context.Background(), nil); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.IsMoving(context.Background()); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.GetPosition(context.Background()); err == nil {
		t.Fatal("expected error")
	}
	if err := g.SetPosition(context.Background(), 0, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.CurrentInputs(context.Background()); err == nil {
		t.Fatal("expected error")
	}
	if err := g.GoToInputs(context.Background(), []referenceframe.Input{0}); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.DoCommand(context.Background(), map[string]interface{}{}); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.Geometries(context.Background(), nil); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.IsHoldingSomething(context.Background(), nil); err == nil {
		t.Fatal("expected error")
	}
}

func TestGripperOpenSendsJointLimitAndWaits(t *testing.T) {
	fa := &testfake.FakeArmRPC{}
	g := newTestGripper(t, fa)
	if err := g.Open(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if fa.LastSetRad != gripperOpenRad || fa.LastWait != true {
		t.Fatalf("rad %v wait %v", fa.LastSetRad, fa.LastWait)
	}
	if fa.LastSetSpeed != roarm.DefaultGripperSpeedDegsPerSec || fa.LastSetAcc != roarm.DefaultGripperAccDegsPerSecSq {
		t.Fatalf("speed %v acc %v: want physical-unit defaults", fa.LastSetSpeed, fa.LastSetAcc)
	}
}

func TestGripperGrabOnObjectReturnsTrue(t *testing.T) {
	// The jaw stops at 0.3 rad instead of the closed limit: something is in it.
	fa := &testfake.FakeArmRPC{Joint6Rad: 0.3, HoldStill: true}
	g := newTestGripper(t, fa)
	grabbed, err := g.Grab(context.Background(), nil)
	if err != nil || !grabbed {
		t.Fatalf("grabbed=%v err=%v", grabbed, err)
	}
	hs, _ := g.IsHoldingSomething(context.Background(), nil)
	if !hs.IsHoldingSomething {
		t.Fatal("holding state not recorded")
	}
}

func TestGripperGrabEmptyReturnsFalse(t *testing.T) {
	fa := &testfake.FakeArmRPC{Joint6Rad: 1.0} // free to close all the way
	g := newTestGripper(t, fa)
	grabbed, err := g.Grab(context.Background(), nil)
	if err != nil || grabbed {
		t.Fatalf("grabbed=%v err=%v", grabbed, err)
	}
}

func TestGripperGrabReturnsPromptly(t *testing.T) {
	// No fixed sleeps remain: with an instant fake, Grab is sub-100ms.
	g := newTestGripper(t, &testfake.FakeArmRPC{Joint6Rad: 1.0})
	start := time.Now()
	if _, err := g.Grab(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if time.Since(start) > 100*time.Millisecond {
		t.Fatalf("Grab took %v; a fixed sleep is still in the path", time.Since(start))
	}
}

func TestGripperSetPosition_RangeIsTheJointLimit(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{})
	ctx := context.Background()
	if err := g.SetPosition(ctx, -20, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err == nil {
		t.Fatal("-20 degrees is below the joint limit")
	}
	if err := g.SetPosition(ctx, 108, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err != nil {
		t.Fatalf("108 degrees is inside the joint limit (108.9): %v", err)
	}
	if err := g.SetPosition(ctx, 120, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err == nil {
		t.Fatal("120 degrees is above the joint limit")
	}
}

func TestGripperIsMoving_FromTwoReads(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{Joint6Series: []float64{0.2, 0.6}})
	moving, err := g.IsMoving(context.Background())
	if err != nil || !moving {
		t.Fatalf("expected moving, got %v %v", moving, err)
	}
	g = newTestGripper(t, &testfake.FakeArmRPC{Joint6Series: []float64{0.2, 0.201}})
	moving, err = g.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("expected still, got %v %v", moving, err)
	}
}

func TestGripperIsMoving_DoesNotReportArmMotion(t *testing.T) {
	// The arm says it is moving; the jaw is not. The gripper must say false.
	g := newTestGripper(t, &testfake.FakeArmRPC{Joint6Rad: 0.5, ArmMoving: true})
	moving, err := g.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("gripper reported arm motion: %v %v", moving, err)
	}
}

func TestGripperIsMoving_NoFeedbackIsFalse(t *testing.T) {
	g := newTestGripper(t, &testfake.FakeArmRPC{DoCommandError: roarm.ErrNoFeedback})
	moving, err := g.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("expected false, nil on a no-feedback transport; got %v %v", moving, err)
	}
}
func TestGripperValidateRequiresArmDep(t *testing.T) {
	cfg := &RoArmGripperConfig{}
	deps, _, err := cfg.Validate("grippers.0")
	if err == nil {
		t.Fatal("expected error when arm is unset")
	}
	if len(deps) != 0 {
		t.Fatal("expected no deps when arm is unset")
	}
}

func TestGripperValidateReturnsArmAsDep(t *testing.T) {
	cfg := &RoArmGripperConfig{Arm: "my-arm"}
	deps, _, err := cfg.Validate("grippers.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(deps) != 1 || deps[0] != "my-arm" {
		t.Fatalf("expected [my-arm], got %v", deps)
	}
}
