package roarm

import (
	"context"
	"errors"
	"testing"
	"time"
)

// scriptedReads returns each slice in turn, then repeats the last one. The
// returned counter is the total number of reads.
func scriptedReads(seq ...[]float64) (func(context.Context) ([]float64, error), *int) {
	calls := 0
	return func(context.Context) ([]float64, error) {
		i := calls
		calls++
		if i >= len(seq) {
			i = len(seq) - 1
		}
		return seq[i], nil
	}, &calls
}

func noSleep(context.Context, time.Duration) error { return nil }

var all6 = []bool{true, true, true, true, true, true}

func TestSettle_ReturnsWhenWithinTolerance(t *testing.T) {
	target := []float64{1, 0, 0, 0, 0, 0}
	read, _ := scriptedReads(
		[]float64{0.5, 0, 0, 0, 0, 0},
		[]float64{0.9, 0, 0, 0, 0, 0},
		[]float64{0.99, 0, 0, 0, 0, 0},
	)
	got, stalled, err := waitUntilSettled(context.Background(), read, noSleep, target, all6, time.Second)
	if err != nil || stalled {
		t.Fatalf("err=%v stalled=%v", err, stalled)
	}
	if got[0] != 0.99 {
		t.Fatalf("expected the settled read, got %v", got)
	}
}

func TestSettle_StallEndsTheWait(t *testing.T) {
	target := []float64{1, 0, 0, 0, 0, 0}
	// Stops at 0.7 (an obstacle). Two identical polls in a row is a stall.
	read, _ := scriptedReads(
		[]float64{0.3, 0, 0, 0, 0, 0},
		[]float64{0.7, 0, 0, 0, 0, 0},
		[]float64{0.7, 0, 0, 0, 0, 0},
	)
	got, stalled, err := waitUntilSettled(context.Background(), read, noSleep, target, all6, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !stalled || got[0] != 0.7 {
		t.Fatalf("expected stall at 0.7, got stalled=%v pos=%v", stalled, got)
	}
}

func TestSettle_FirstPollCannotStall(t *testing.T) {
	// The first read may still show the pre-command position; a stall needs two reads.
	target := []float64{1, 0, 0, 0, 0, 0}
	read, calls := scriptedReads(
		[]float64{0, 0, 0, 0, 0, 0},
		[]float64{0.5, 0, 0, 0, 0, 0},
		[]float64{1, 0, 0, 0, 0, 0},
	)
	_, stalled, err := waitUntilSettled(context.Background(), read, noSleep, target, all6, time.Second)
	if err != nil || stalled {
		t.Fatalf("err=%v stalled=%v", err, stalled)
	}
	if *calls < 3 {
		t.Fatalf("expected at least 3 reads, got %d", *calls)
	}
}

func TestSettle_MaskIgnoresOtherJoints(t *testing.T) {
	// Joint 6 is far from its target but unmasked; joints 1-5 are settled.
	target := []float64{0, 0, 0, 0, 0, 1.9}
	read, _ := scriptedReads([]float64{0, 0, 0, 0, 0, -0.2})
	_, _, err := waitUntilSettled(context.Background(), read, noSleep, target, ArmMask, time.Second)
	if err != nil {
		t.Fatalf("masked-out joint should not block settle: %v", err)
	}
}

func TestSettle_TimesOut(t *testing.T) {
	target := []float64{1, 0, 0, 0, 0, 0}
	// Creeps 0.01 per poll (more than StallRad) toward a target it never
	// reaches inside the poll budget: never within tolerance, never stalled.
	i := 0.0
	read := func(context.Context) ([]float64, error) { i += 0.01; return []float64{i, 0, 0, 0, 0, 0}, nil }
	_, _, err := waitUntilSettled(context.Background(), read, noSleep, target, all6, 200*time.Millisecond)
	if err == nil {
		t.Fatal("expected timeout")
	}
}

func TestSettle_ReadErrorPropagates(t *testing.T) {
	boom := errors.New("boom")
	read := func(context.Context) ([]float64, error) { return nil, boom }
	_, _, err := waitUntilSettled(context.Background(), read, noSleep, []float64{0, 0, 0, 0, 0, 0}, all6, time.Second)
	if !errors.Is(err, boom) {
		t.Fatalf("expected boom, got %v", err)
	}
}

func TestSettle_ShortFeedbackIsAnError(t *testing.T) {
	read := func(context.Context) ([]float64, error) { return []float64{0, 0}, nil }
	_, _, err := waitUntilSettled(context.Background(), read, noSleep, []float64{0, 0, 0, 0, 0, 0}, all6, time.Second)
	if err == nil {
		t.Fatal("expected short-feedback error")
	}
}

func TestSettleTimeoutFor(t *testing.T) {
	// 90 degrees at 50 deg/s is 1.8 s; times 2 is 3.6 s.
	got := SettleTimeoutFor(90*3.14159265/180, SpeedToUnits(50))
	if got < 3500*time.Millisecond || got > 3700*time.Millisecond {
		t.Fatalf("got %v want ~3.6s", got)
	}
	if SettleTimeoutFor(0, SpeedToUnits(50)) != minSettleTimeout {
		t.Fatal("floor")
	}
	if SettleTimeoutFor(100, SpeedToUnits(3)) != maxSettleTimeout {
		t.Fatal("cap")
	}
}

func TestMaxTravel(t *testing.T) {
	a := []float64{0, 0, 0, 0, 0, 0}
	b := []float64{0.1, -0.5, 0, 0, 0, 2}
	if got := MaxTravel(a, b, nil); got != 2 {
		t.Fatalf("nil mask = all joints: got %v", got)
	}
	if got := MaxTravel(a, b, ArmMask); got != 0.5 {
		t.Fatalf("arm mask: got %v", got)
	}
}
