package waveshareroarm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
)

// newTestGripper builds a gripper wired to a fakeArmRPC client, mirroring
// what the constructor does in production (where arm.FromDependencies
// returns an arm.Arm gRPC client, not the local *roarmM3 struct).
func newTestGripper(t *testing.T, fa *fakeArmRPC) *roarmM3Gripper {
	t.Helper()
	return &roarmM3Gripper{
		armClient: fa,
		logger:    logging.NewTestLogger(t),
		opMgr:     operation.NewSingleOperationManager(),
	}
}

func TestGripperOpenSendsJointLimit(t *testing.T) {
	fa := &fakeArmRPC{}
	g := newTestGripper(t, fa)
	// Cancel immediately so the ctx-aware select in Open aborts after the DoCommand.
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_ = g.Open(ctx, nil)
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad, got %q", fa.LastCommand)
	}
	if fa.LastSetRad != gripperOpenRad {
		t.Fatalf("expected gripperOpenRad=%v, got %v", gripperOpenRad, fa.LastSetRad)
	}
}

func TestGripperGrabWithBlockedReturnsTrue(t *testing.T) {
	// After Grab writes -0.2, the gripper reads back; return a value that
	// indicates the jaw didn't fully close (blocked by an object).
	fa := &fakeArmRPC{Joint6Rad: 0.3}
	g := newTestGripper(t, fa)
	// Prevent the Grab from overwriting Joint6Rad back to -0.2.
	// The test models: commanded -0.2, but object blocks → current ≈ 0.3.
	// We need the fake to keep returning 0.3 even after set_gripper_rad.
	// Hack: replace the DoCommand to not update Joint6Rad for set. We'll
	// instead call Grab, then explicitly reset Joint6Rad before the read.
	// Simpler: override after the set-call via a small sleep isn't viable
	// in a unit test, so use the hook below.
	fa.Joint6Rad = 0.3
	// Bypass the set side-effect by snapshotting and restoring.
	g.armClient = &blockedArmRPC{fake: fa}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	grabbed, err := g.Grab(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if !grabbed {
		t.Fatal("expected grabbed=true for blocked gripper")
	}
	st, _ := g.IsHoldingSomething(ctx, nil)
	if !st.IsHoldingSomething {
		t.Fatal("expected holding=true")
	}
}

// blockedArmRPC wraps fakeArmRPC but ignores set_gripper_rad side effects,
// simulating a gripper whose jaw is blocked and cannot reach the commanded
// position. get_gripper_rad still returns the configured Joint6Rad.
type blockedArmRPC struct {
	fake *fakeArmRPC
}

func (b *blockedArmRPC) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, _ := cmd["command"].(string)
	if name == "set_gripper_rad" {
		// Record the command but don't overwrite Joint6Rad.
		b.fake.mu.Lock()
		b.fake.LastCommand = name
		rad, _ := cmd["rad"].(float64)
		b.fake.LastSetRad = rad
		b.fake.mu.Unlock()
		return map[string]interface{}{"success": true}, nil
	}
	return b.fake.DoCommand(ctx, cmd)
}

func (b *blockedArmRPC) IsMoving(ctx context.Context) (bool, error) {
	return b.fake.IsMoving(ctx)
}

func TestGripperGrabFullyClosedReturnsFalse(t *testing.T) {
	// Jaw reaches commanded grabRad exactly → no object in the way.
	fa := &fakeArmRPC{Joint6Rad: gripperGrabRad}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	grabbed, err := g.Grab(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if grabbed {
		t.Fatal("expected grabbed=false when fully closed")
	}
}

func TestGripperOpenClearsHolding(t *testing.T) {
	fa := &fakeArmRPC{Joint6Rad: 0.3}
	g := newTestGripper(t, fa)
	g.armClient = &blockedArmRPC{fake: fa}
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
	fa := &fakeArmRPC{Joint6Rad: 0}
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
	fa := &fakeArmRPC{Joint6Rad: 0.7}
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

func TestGripperIsMoving(t *testing.T) {
	fa := &fakeArmRPC{MoveDeadline: time.Now().Add(500 * time.Millisecond)}
	g := newTestGripper(t, fa)
	moving, err := g.IsMoving(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if !moving {
		t.Fatal("expected moving")
	}
}

func TestGripperModelFrameAndKinematics(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	g.model = mustLoadModel(t)
	if g.ModelFrame() == nil {
		t.Fatal("expected non-nil model frame")
	}
	m, err := g.Kinematics(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if m == nil {
		t.Fatal("expected non-nil kinematics")
	}
}

func TestGripperGeometries(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	geos, err := g.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(geos) == 0 {
		t.Fatal("expected at least one geometry")
	}
}

func TestGripperCurrentInputs(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{Joint6Rad: 0})
	inputs, err := g.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 1 {
		t.Fatalf("expected 1 input, got %d", len(inputs))
	}
}

func TestGripperClose(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
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
	g := newTestGripper(t, &fakeArmRPC{Joint6Rad: 0})
	out, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "get_position"})
	if err != nil {
		t.Fatal(err)
	}
	if out["position_degrees"] != 0.0 {
		t.Fatalf("expected position_degrees=0, got %v", out["position_degrees"])
	}
}

func TestGripperDoCommand_Unknown(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "nonsense"})
	if err == nil {
		t.Fatal("expected error for unknown command")
	}
}

func TestGripperSetPosition_RangeCheck(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	if err := g.SetPosition(context.Background(), -20, 500, 50); err == nil {
		t.Fatal("expected error for -20 degrees")
	}
	if err := g.SetPosition(context.Background(), 200, 500, 50); err == nil {
		t.Fatal("expected error for 200 degrees")
	}
}

func TestGripperSetPosition_CancelledContext(t *testing.T) {
	fa := &fakeArmRPC{}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_ = g.SetPosition(ctx, 50, 500, 50)
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperName(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	_ = g.Name()
}

func TestGripperDoCommand_SetPosition(t *testing.T) {
	fa := &fakeArmRPC{}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	out, err := g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position",
		"degrees": float64(50),
		"speed":   float64(500),
		"acc":     float64(50),
	})
	_ = out
	_ = err
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperDoCommand_SetPositionMissingDegrees(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "set_position"})
	if err == nil {
		t.Fatal("expected error for missing degrees")
	}
}

func TestGripperGoToInputs_Empty(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	if err := g.GoToInputs(context.Background()); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
}

func TestGripperGoToInputs_WrongLength(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
	err := g.GoToInputs(context.Background(), nil)
	if err == nil {
		t.Fatal("expected error for wrong length")
	}
	err = g.GoToInputs(context.Background(), []referenceframe.Input{{Value: 0}, {Value: 1}})
	if err == nil {
		t.Fatal("expected error for inputSet length != 1")
	}
}

func TestGripperGoToInputs_Valid(t *testing.T) {
	fa := &fakeArmRPC{}
	g := newTestGripper(t, fa)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_ = g.GoToInputs(ctx, []referenceframe.Input{{Value: 0.5}})
	if fa.LastCommand != "set_gripper_rad" {
		t.Fatalf("expected set_gripper_rad dispatched, got %q", fa.LastCommand)
	}
}

func TestGripperAfterClose_ReturnErrors(t *testing.T) {
	g := newTestGripper(t, &fakeArmRPC{})
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
	if err := g.SetPosition(context.Background(), 0, 500, 50); err == nil {
		t.Fatal("expected error")
	}
	if _, err := g.CurrentInputs(context.Background()); err == nil {
		t.Fatal("expected error")
	}
	if err := g.GoToInputs(context.Background(), []referenceframe.Input{{Value: 0}}); err == nil {
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
