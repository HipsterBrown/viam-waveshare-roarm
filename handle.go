package waveshareroarm

import (
	"context"
	"time"
)

// RoArmHandle is the narrow interface other resources (like the gripper)
// consume when borrowing the arm's serial/HTTP handle.
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
}
