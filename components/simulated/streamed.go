package simulated

import (
	"context"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"

	"waveshareroarm/internal/roarm"
)

// MoveThroughJointPositionsStreamed re-targets the interpolator at each point's scheduled
// time and returns once the arm has converged on the last one. Constraints are ignored.
// Batches are received with a select on ctx, not range: a cancelled ctx does not close the
// channel.
func (s *simulatedArm) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []rdkarm.TrajectoryPoint,
	responses chan<- rdkarm.Response,
	_ map[string]interface{},
) error {
	var start time.Time
	prev := time.Duration(-1) // no previous point yet
	idx := 0
	for {
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
			if idx == 0 {
				start = s.clock.Time()
			}
			if err := s.clock.WaitUntil(ctx, start.Add(p.Time)); err != nil {
				return err
			}
			// A re-target must not erase a Stop that landed between points.
			if err := s.startMove(ctx, p.Positions, idx > 0); err != nil {
				return err
			}
			prev, idx = p.Time, idx+1
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
	return s.awaitOperation(ctx)
}
