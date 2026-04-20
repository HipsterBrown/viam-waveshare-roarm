package waveshareroarm

import (
	"encoding/json"
	"testing"
	"time"
)

func TestCommandMarshalJSON_WithData(t *testing.T) {
	cmd := &Command{
		T: JOINT_RADIAN_CTRL,
		Data: map[string]interface{}{
			"joint": 1,
			"rad":   0.5,
		},
	}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if int(out["T"].(float64)) != JOINT_RADIAN_CTRL {
		t.Fatal("T mismatch")
	}
	if int(out["joint"].(float64)) != 1 {
		t.Fatal("joint mismatch")
	}
	if out["rad"].(float64) != 0.5 {
		t.Fatal("rad mismatch")
	}
}

func TestCommandMarshalJSON_EmptyData(t *testing.T) {
	cmd := &Command{T: FEEDBACK_GET, Data: map[string]interface{}{}}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if len(out) != 1 {
		t.Fatalf("expected 1 key, got %v", out)
	}
}

func TestCommandMarshalJSON_NilData(t *testing.T) {
	cmd := &Command{T: FEEDBACK_GET}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if len(out) != 1 {
		t.Fatalf("expected 1 key, got %v", out)
	}
}

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
