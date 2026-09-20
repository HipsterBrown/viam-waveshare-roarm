package waveshareroarm

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
)

// streamStartGapRad is how far the arm may already be from a stream's first
// point before that point is written on schedule. Further than this, one
// settled move closes the gap first, then the clock starts.
const streamStartGapRad = 5 * math.Pi / 180

// clock is the wall clock a streamed trajectory is scheduled against. The
// zero value is the real clock; tests substitute both funcs so no test
// sleeps trajectory time.
type clock struct {
	now        func() time.Time
	sleepUntil func(context.Context, time.Time) error
}

func (c clock) Time() time.Time {
	if c.now != nil {
		return c.now()
	}
	return time.Now()
}

// WaitUntil blocks until t or ctx ends. A deadline already past returns at
// once with ctx.Err(), so a cancelled stream still stops on a late point.
func (c clock) WaitUntil(ctx context.Context, t time.Time) error {
	if c.sleepUntil != nil {
		return c.sleepUntil(ctx, t)
	}
	d := t.Sub(c.Time())
	if d <= 0 {
		return ctx.Err()
	}
	return sleepCtx(ctx, d)
}

// checkTrajectoryTime validates a point's Time against the previous point's:
// the first (prev < 0) must be 0 and every later one strictly after its
// predecessor.
func checkTrajectoryTime(prev, t time.Duration) error {
	switch {
	case prev < 0 && t != 0:
		return fmt.Errorf("first trajectory point must have Time 0, got %v", t)
	case prev >= 0 && t <= prev:
		return fmt.Errorf("trajectory point Time %v must exceed the previous point's %v", t, prev)
	}
	return nil
}

// MoveThroughJointPositionsStreamed writes each point as one joint command
// when the PREVIOUS point's time arrives, at the speed that covers the
// segment's longest joint travel by this point's own time. The firmware
// interpolates on-device toward each goal, so writing goal k at T(k-1) with
// speed travel/(T(k)-T(k-1)) is what puts the arm at point k at T(k); writing
// it at T(k) would trail the schedule by one segment. Point 0 is written at
// start. No feedback is read inside the loop; the arm settles once after the
// last point, with a timeout sized from the last segment. Late points are
// written immediately, never dropped. Constraints and extra are ignored.
//
// Only the FIRST point is gated (a settled move if the arm is more than
// streamStartGapRad away). A large jump between later points reaches the
// firmware as written.
func (r *roarmM3) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	if r.closed.Load() {
		return errClosed
	}
	ctx, done := r.opMgr.New(ctx)
	defer done()
	r.opInFlight.Store(true)
	defer r.opInFlight.Store(false)

	r.mu.Lock()
	speed, acc, limits := r.defaultSpeed, r.defaultAcc, r.jointLimits
	r.mu.Unlock()
	ctrl := r.snapshotController()

	var start, wall time.Time
	var gate, maxLate time.Duration
	var last []float64 // full 6-joint target of the previous point
	var gripper float64
	var lastTravelRad float64 // longest arm-joint travel of the last segment
	lastSpeed := speed
	prev := time.Duration(-1)
	idx, late := 0, 0
	for {
		// Not `range batches`: Stop cancels ctx but cannot close the channel,
		// and CancelRunning blocks until this returns.
		var batch []arm.TrajectoryPoint
		var ok bool
		select {
		case <-ctx.Done():
			return ctx.Err()
		case batch, ok = <-batches:
		}
		if !ok {
			break
		}
		for _, p := range batch {
			if err := checkTrajectoryTime(prev, p.Time); err != nil {
				return err
			}
			if len(p.Positions) != len(limits) {
				return fmt.Errorf("trajectory point has %d joints, want %d", len(p.Positions), len(limits))
			}
			clamped, _ := clampToLimits(p.Positions, limits)
			segSpeed := speed
			if idx == 0 {
				wall = r.clock.Time()
				current, err := ctrl.GetJointRadians(ctx)
				if err != nil {
					return fmt.Errorf("streamed: read position before the first point: %w", err)
				}
				if len(current) < 6 {
					return fmt.Errorf("streamed: short feedback (got %d joints)", len(current))
				}
				gripper = current[5]
				first := append(append([]float64(nil), clamped...), gripper)
				if maxTravel(current, first, armMask) > streamStartGapRad {
					r.logger.Debugf("streamed trajectory starts %.1f deg away; settled move to its first point",
						maxTravel(current, first, armMask)*180/math.Pi)
					if err := r.moveAndSettle(ctx, ctrl, current, first, speed, acc); err != nil {
						return err
					}
				}
				start = r.clock.Time()
				gate = start.Sub(wall)
			} else {
				lastTravelRad = maxTravel(last, clamped, armMask)
				segSpeed = speedToUnits(lastTravelRad * 180 / math.Pi / (p.Time - prev).Seconds())
			}
			// Point 0 goes out at start; point k goes out when point k-1 is
			// due, so the firmware has the whole segment to reach it.
			due := start
			if idx > 0 {
				due = start.Add(prev)
			}
			if behind := r.clock.Time().Sub(due); behind >= time.Millisecond {
				late++
				if behind > maxLate {
					maxLate = behind
				}
			}
			if err := r.clock.WaitUntil(ctx, due); err != nil {
				return err
			}
			target := append(append([]float64(nil), clamped...), gripper)
			if err := ctrl.SetJointRadians(ctx, target, segSpeed, acc); err != nil {
				return fmt.Errorf("streamed: point %d: %w", idx, err)
			}
			prev, idx, last, lastSpeed = p.Time, idx+1, target, segSpeed
		}
		if len(batch) == 0 {
			continue
		}
		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	if idx == 0 {
		return nil
	}
	settleFrom := r.clock.Time()
	// The last point was written when its predecessor was due, so the arm
	// still has that whole segment to travel; size the wait from it.
	_, err := ctrl.WaitUntilSettled(ctx, last, armMask, settleTimeoutFor(lastTravelRad, lastSpeed))
	now := r.clock.Time()
	r.logger.Infof("streamed %d points over %v: gate %v, late %d (max %v), settle %v, wall %v",
		idx, prev, gate.Round(time.Millisecond), late, maxLate.Round(time.Millisecond),
		now.Sub(settleFrom).Round(time.Millisecond), now.Sub(wall).Round(time.Millisecond))
	return err
}
