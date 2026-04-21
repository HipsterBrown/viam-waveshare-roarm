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
	MoveToHome(ctx context.Context) error
	SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error
	SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error
	GetJointRadians(ctx context.Context) ([]float64, error)
	GetFeedback(ctx context.Context) (*FeedbackData, error)
	TestConnection(ctx context.Context) error
	IsMoving(ctx context.Context) (bool, error)
	NoteMotionDeadline(deadline time.Time)
	Close(ctx context.Context) error
}
