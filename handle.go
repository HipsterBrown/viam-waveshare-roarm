package waveshareroarm

import (
	"context"
	"time"
)

// RoArmHandle is the narrow interface the arm consumes to talk to its
// serial/HTTP controller. It exists as a seam so tests can inject a
// fakeController. Sibling resources (e.g. gripper) do not use this
// interface; they hold an arm.Arm gRPC client and invoke joint-6
// operations through the arm's DoCommand bridge.
type RoArmHandle interface {
	SetTorque(ctx context.Context, enable bool) error
	SetLED(ctx context.Context, brightness int) error
	SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error
	SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error
	GetJointRadians(ctx context.Context) ([]float64, error)
	GetFeedback(ctx context.Context) (*FeedbackData, error)
	WaitUntilSettled(ctx context.Context, target []float64, mask []bool, timeout time.Duration) ([]float64, error)
	IsMoving(ctx context.Context) (bool, error)
	Close(ctx context.Context) error
}
