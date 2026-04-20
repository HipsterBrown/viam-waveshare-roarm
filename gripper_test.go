package waveshareroarm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
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
