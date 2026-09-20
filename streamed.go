package waveshareroarm

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/arm"
)

// MoveThroughJointPositionsStreamed is implemented in Task 11.
func (r *roarmM3) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	return errors.ErrUnsupported
}
