package waveshareroarm

import (
	"encoding/json"
	"testing"
	"time"
)

func TestDurationUnmarshalString(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`"5s"`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != 5*time.Second {
		t.Fatalf("got %v, want 5s", time.Duration(d))
	}
}

func TestDurationUnmarshalNumber(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`1000000000`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != time.Second {
		t.Fatalf("got %v, want 1s", time.Duration(d))
	}
}

func TestDurationUnmarshalEmpty(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`""`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != 0 {
		t.Fatalf("got %v, want 0", time.Duration(d))
	}
}
