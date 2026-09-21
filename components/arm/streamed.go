package arm

import (
	"context"
	"fmt"
	"math"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"

	"waveshareroarm/internal/roarm"
)

// streamStartGapRad is how far the arm may already be from a stream's first
// point before that point is written on schedule. Further than this, one
// settled move closes the gap first, then the clock starts.
const streamStartGapRad = 5 * math.Pi / 180

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
	batches <-chan []rdkarm.TrajectoryPoint,
	responses chan<- rdkarm.Response,
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
	var last []float64    // full 6-joint target of the previous point
	var current []float64 // pose measured before the first point; the settle's start-gate
	var gripper float64
	var lastTravelRad float64 // longest arm-joint travel of the last segment
	lastSpeed := speed
	prev := time.Duration(-1)
	idx, late := 0, 0
	for {
		// Not `range batches`: Stop cancels ctx but cannot close the channel,
		// and CancelRunning blocks until this returns.
		var batch []rdkarm.TrajectoryPoint
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
			if err := roarm.CheckTrajectoryTime(prev, p.Time); err != nil {
				return err
			}
			if len(p.Positions) != len(limits) {
				return fmt.Errorf("trajectory point has %d joints, want %d", len(p.Positions), len(limits))
			}
			clamped, _ := clampToLimits(p.Positions, limits)
			segSpeed := speed
			if idx == 0 {
				wall = r.clock.Time()
				var err error
				current, err = readAllJointRadians(ctx, ctrl)
				if err != nil {
					return fmt.Errorf("streamed: read position before the first point: %w", err)
				}
				gripper = current[5]
				first := append(append([]float64(nil), clamped...), gripper)
				if roarm.MaxTravel(current, first, roarm.ArmMask) > streamStartGapRad {
					r.logger.Debugf("streamed trajectory starts %.1f deg away; settled move to its first point",
						roarm.MaxTravel(current, first, roarm.ArmMask)*180/math.Pi)
					if err := r.moveAndSettle(ctx, ctrl, current, first, speed, acc); err != nil {
						return err
					}
				}
				start = r.clock.Time()
				gate = start.Sub(wall)
			} else {
				lastTravelRad = roarm.MaxTravel(last, clamped, roarm.ArmMask)
				segSpeed = roarm.SpeedToUnits(lastTravelRad * 180 / math.Pi / (p.Time - prev).Seconds())
			}
			// Point 0 goes out at start; point k goes out when point k-1 is
			// due, so the firmware has the whole segment to reach it.
			due := start
			if idx > 0 {
				due = start.Add(prev)
			}
			// Late means written after the point's OWN time. A live producer
			// hands point k over at T(k), one segment after the T(k-1) write
			// slot, and that is on time, not late; the arm then arrives one
			// segment behind, which is the best any goal-following controller
			// can do without knowing the next point in advance.
			if behind := r.clock.Time().Sub(start.Add(p.Time)); behind >= time.Millisecond {
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
		case responses <- rdkarm.Response{}:
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
	req := roarm.SettleRequest{
		Start:         current,
		Target:        last,
		Mask:          roarm.ArmMask,
		SpeedUnits:    lastSpeed,
		AccUnits:      acc,
		RequireMotion: true,
	}
	_, err := ctrl.WaitUntilSettled(ctx, req)
	now := r.clock.Time()
	r.logger.Infof("streamed %d points over %v: gate %v, late %d (max %v), settle %v, wall %v",
		idx, prev, gate.Round(time.Millisecond), late, maxLate.Round(time.Millisecond),
		now.Sub(settleFrom).Round(time.Millisecond), now.Sub(wall).Round(time.Millisecond))
	return err
}
