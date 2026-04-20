package waveshareroarm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
)

func TestGripperOpenSendsJointLimit(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	// Use a short-timeout context so we don't wait 1s for the sleep.
	ctx, cancel := context.WithCancel(context.Background())
	cancel() // cancel immediately; the ctx-aware select in Open will abort after SetJointRadian.
	_ = g.Open(ctx, nil)
	if fc.LastJoint != 6 {
		t.Fatalf("expected joint 6, got %d", fc.LastJoint)
	}
	if fc.LastRadians[5] != gripperOpenRad {
		t.Fatalf("expected gripperOpenRad=%v, got %v", gripperOpenRad, fc.LastRadians[5])
	}
}

func TestGripperGrabWithBlockedReturnsTrue(t *testing.T) {
	fc := &fakeController{
		Feedback: FeedbackData{G: 0.3}, // didn't close fully
	}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	// Use a timeout context that still lets the 1s sleep complete
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	grabbed, err := g.Grab(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if !grabbed {
		t.Fatal("expected grabbed=true for blocked gripper")
	}
	// IsHoldingSomething should reflect it.
	st, _ := g.IsHoldingSomething(ctx, nil)
	if !st.IsHoldingSomething {
		t.Fatal("expected holding=true")
	}
}

func TestGripperGrabFullyClosedReturnsFalse(t *testing.T) {
	fc := &fakeController{
		Feedback: FeedbackData{G: gripperGrabRad}, // closed all the way
	}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
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
	fc := &fakeController{Feedback: FeedbackData{G: 0.3}}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
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
	// G=0 rad → 0 degrees
	fc := &fakeController{Feedback: FeedbackData{G: 0}}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	pos, err := g.GetPosition(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if pos != 0 {
		t.Fatalf("expected 0 degrees, got %v", pos)
	}
}

func TestGripperStop(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{G: 0.7}}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	if err := g.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if fc.LastJoint != 6 {
		t.Fatalf("expected joint 6, got %d", fc.LastJoint)
	}
	if fc.LastRadians[5] != 0.7 {
		t.Fatalf("expected stop at 0.7, got %v", fc.LastRadians[5])
	}
}

func TestGripperIsMoving(t *testing.T) {
	fc := &fakeController{MoveDeadline: time.Now().Add(500 * time.Millisecond)}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	moving, err := g.IsMoving(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if !moving {
		t.Fatal("expected moving")
	}
}

func TestGripperModelFrameAndKinematics(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	// Set model for testing (mimic constructor).
	g.model = nil // will be nil until we assign
	// Create a simple model via the real constructor path's helper.
	// We can construct our own model or check the Kinematics path tolerates nil.
	// Set model explicitly:
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
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	geos, err := g.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(geos) == 0 {
		t.Fatal("expected at least one geometry")
	}
}

func TestGripperCurrentInputs(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{G: 0}}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	inputs, err := g.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 1 {
		t.Fatalf("expected 1 input, got %d", len(inputs))
	}
}

func TestGripperClose(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
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
	fc := &fakeController{Feedback: FeedbackData{G: 0}}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	out, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "get_position"})
	if err != nil {
		t.Fatal(err)
	}
	if out["position_degrees"] != 0.0 {
		t.Fatalf("expected position_degrees=0, got %v", out["position_degrees"])
	}
}

func TestGripperDoCommand_Unknown(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "nonsense"})
	if err == nil {
		t.Fatal("expected error for unknown command")
	}
}

func TestGripperSetPosition_RangeCheck(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	if err := g.SetPosition(context.Background(), -20, 500, 50); err == nil {
		t.Fatal("expected error for -20 degrees")
	}
	if err := g.SetPosition(context.Background(), 200, 500, 50); err == nil {
		t.Fatal("expected error for 200 degrees")
	}
}

func TestGripperSetPosition_CancelledContext(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel() // cancel before call; select picks ctx.Done() after SetJointRadian
	_ = g.SetPosition(ctx, 50, 500, 50)
	// Command should still have been sent.
	if fc.LastJoint != 6 {
		t.Fatalf("expected joint 6 set, got %d", fc.LastJoint)
	}
}

func TestGripperName(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	_ = g.Name()
}

func TestGripperDoCommand_SetPosition(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel() // abort early after the SetJointRadian call
	out, err := g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position",
		"degrees": float64(50),
		"speed":   float64(500),
		"acc":     float64(50),
	})
	// SetPosition may return ctx.Err() when cancelled; just confirm the command reached the fake.
	_ = out
	_ = err
	if fc.LastJoint != 6 {
		t.Fatalf("expected joint 6 dispatched, got %d", fc.LastJoint)
	}
}

func TestGripperDoCommand_SetPositionMissingDegrees(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	_, err := g.DoCommand(context.Background(), map[string]interface{}{"command": "set_position"})
	if err == nil {
		t.Fatal("expected error for missing degrees")
	}
}

func TestGripperGoToInputs_Empty(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	if err := g.GoToInputs(context.Background()); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
}

func TestGripperGoToInputs_WrongLength(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	err := g.GoToInputs(context.Background(), nil)
	if err == nil {
		t.Fatal("expected error for wrong length")
	}
	// With an explicit bad inputSet (length != 1)
	err = g.GoToInputs(context.Background(), []referenceframe.Input{{Value: 0}, {Value: 1}})
	if err == nil {
		t.Fatal("expected error for inputSet length != 1")
	}
}

func TestGripperGoToInputs_Valid(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel() // abort quickly after SetJointRadian
	_ = g.GoToInputs(ctx, []referenceframe.Input{{Value: 0.5}})
	if fc.LastJoint != 6 {
		t.Fatalf("expected joint 6 set, got %d", fc.LastJoint)
	}
}

func TestGripperAfterClose_ReturnErrors(t *testing.T) {
	fc := &fakeController{}
	g := &roarmM3Gripper{
		controller: fc, logger: logging.NewTestLogger(t),
		opMgr: operation.NewSingleOperationManager(),
	}
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
