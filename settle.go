package waveshareroarm

import (
	"context"
	"fmt"
	"math"
	"time"
)

// The firmware reports joint positions but no moving flag, so "the move is
// done" is derived from position: either every joint is within settleTolRad
// of its target, or nothing moved between two polls (the arm stopped short).
const (
	settlePollInterval = 50 * time.Millisecond
	settleTolRad       = 0.02  // ~1.1 degrees
	stallRad           = 0.005 // ~0.3 degrees between consecutive polls
	minSettleTimeout   = 500 * time.Millisecond
	maxSettleTimeout   = 15 * time.Second
	// isMovingProbeGap separates the two position samples IsMoving compares.
	isMovingProbeGap = 40 * time.Millisecond
)

// Joint masks for WaitUntilSettled: the arm settles on joints 1-5 while the
// gripper is commanded to hold, and the gripper settles on joint 6 alone.
var (
	armMask     = []bool{true, true, true, true, true, false}
	gripperMask = []bool{false, false, false, false, false, true}
)

// settleTimeoutFor is the one place the settle timeout policy lives: twice
// the time the longest travel takes at speedUnits, floored and capped.
func settleTimeoutFor(travelRad float64, speedUnits int) time.Duration {
	radPerSec := speedFromUnits(speedUnits) * math.Pi / 180
	if radPerSec <= 0 {
		return maxSettleTimeout
	}
	d := time.Duration(2 * travelRad / radPerSec * float64(time.Second))
	if d < minSettleTimeout {
		return minSettleTimeout
	}
	if d > maxSettleTimeout {
		return maxSettleTimeout
	}
	return d
}

// maxTravel returns the largest |a[i]-b[i]| over the masked joints (nil mask
// means every joint). Slices shorter than the mask are compared as far as
// they go.
func maxTravel(a, b []float64, mask []bool) float64 {
	m := 0.0
	for i := 0; i < len(a) && i < len(b); i++ {
		if mask != nil && (i >= len(mask) || !mask[i]) {
			continue
		}
		if d := math.Abs(a[i] - b[i]); d > m {
			m = d
		}
	}
	return m
}

// waitUntilSettled is the transport-free core of RoArmController.WaitUntilSettled.
// It sleeps one poll interval, reads, and repeats until the masked joints are
// within settleTolRad of target (settled), or two consecutive reads agree
// within stallRad (stalled; returned as ok with stalled=true), or the poll
// budget implied by timeout is spent (error). The first read is never
// considered a stall because it may still show the pre-command position.
func waitUntilSettled(
	ctx context.Context,
	read func(context.Context) ([]float64, error),
	sleep func(context.Context, time.Duration) error,
	target []float64,
	mask []bool,
	timeout time.Duration,
) (positions []float64, stalled bool, err error) {
	polls := int(math.Ceil(float64(timeout) / float64(settlePollInterval)))
	if polls < 2 {
		polls = 2
	}
	var prev []float64
	for i := 0; i < polls; i++ {
		if err := sleep(ctx, settlePollInterval); err != nil {
			return nil, false, err
		}
		cur, err := read(ctx)
		if err != nil {
			return nil, false, err
		}
		if len(cur) < len(target) {
			return nil, false, fmt.Errorf("short feedback (got %d joints, want %d)", len(cur), len(target))
		}
		if maxTravel(cur, target, mask) <= settleTolRad {
			return cur, false, nil
		}
		if prev != nil && maxTravel(cur, prev, mask) <= stallRad {
			return cur, true, nil
		}
		prev = cur
	}
	return nil, false, fmt.Errorf("timed out after %v waiting for joints to settle", timeout)
}

// sleepCtx sleeps for d or until ctx ends.
func sleepCtx(ctx context.Context, d time.Duration) error {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}
