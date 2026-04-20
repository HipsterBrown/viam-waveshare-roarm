package waveshareroarm

import (
	"testing"
	"time"
)

func TestMotionTrackerIsMovingBeforeDeadline(t *testing.T) {
	tr := newMotionTracker()
	tr.recordMove(time.Now().Add(200 * time.Millisecond))
	if !tr.isMoving(time.Now()) {
		t.Fatal("expected moving before deadline")
	}
}

func TestMotionTrackerNotMovingAfterDeadline(t *testing.T) {
	tr := newMotionTracker()
	tr.recordMove(time.Now().Add(-10 * time.Millisecond))
	if tr.isMoving(time.Now()) {
		t.Fatal("expected not moving after deadline")
	}
}

func TestMotionTrackerNotMovingBeforeFirstRecord(t *testing.T) {
	tr := newMotionTracker()
	if tr.isMoving(time.Now()) {
		t.Fatal("expected not moving before any recordMove call")
	}
}
