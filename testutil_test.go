package waveshareroarm

import (
	"context"
	"sync"
	"time"
)

// Compile-time check that fakeController satisfies RoArmHandle.
var _ RoArmHandle = (*fakeController)(nil)

// fakeController implements RoArmHandle for tests.
type fakeController struct {
	mu            sync.Mutex
	LastSpeed     int
	LastAcc       int
	LastRadians   []float64
	LastJoint     int
	LastTorque    *bool
	LastLED       *int
	HomeCalls     int
	FeedbackCalls int
	Feedback      FeedbackData
	FailOn        string // method name to return error from, empty = never
	MoveDeadline  time.Time
}

func (f *fakeController) err(method string) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.FailOn == method {
		return &fakeErr{method}
	}
	return nil
}

type fakeErr struct{ m string }

func (e *fakeErr) Error() string { return "fake: forced error from " + e.m }

func (f *fakeController) SetTorque(ctx context.Context, enable bool) error {
	if err := f.err("SetTorque"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.LastTorque = &enable
	return nil
}

func (f *fakeController) SetLED(ctx context.Context, brightness int) error {
	if err := f.err("SetLED"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.LastLED = &brightness
	return nil
}

func (f *fakeController) MoveToHome(ctx context.Context) error {
	if err := f.err("MoveToHome"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.HomeCalls++
	return nil
}

func (f *fakeController) SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error {
	if err := f.err("SetJointRadian"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.LastJoint, f.LastSpeed, f.LastAcc = joint, speed, acc
	if cap(f.LastRadians) < 6 {
		f.LastRadians = make([]float64, 6)
	}
	f.LastRadians = f.LastRadians[:6]
	f.LastRadians[joint-1] = radian
	return nil
}

func (f *fakeController) SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error {
	if err := f.err("SetJointRadians"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.LastRadians = append([]float64(nil), radians...)
	f.LastSpeed, f.LastAcc = speed, acc
	return nil
}

func (f *fakeController) GetJointRadians(ctx context.Context) ([]float64, error) {
	if err := f.err("GetJointRadians"); err != nil {
		return nil, err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	return []float64{f.Feedback.B, f.Feedback.S, f.Feedback.E, f.Feedback.Wrist, f.Feedback.R, f.Feedback.G}, nil
}

func (f *fakeController) GetFeedback(ctx context.Context) (*FeedbackData, error) {
	if err := f.err("GetFeedback"); err != nil {
		return nil, err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.FeedbackCalls++
	fb := f.Feedback
	return &fb, nil
}

func (f *fakeController) TestConnection(ctx context.Context) error { return f.err("TestConnection") }

func (f *fakeController) IsMoving(ctx context.Context) (bool, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return time.Now().Before(f.MoveDeadline), nil
}

func (f *fakeController) NoteMotionDeadline(deadline time.Time) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.MoveDeadline = deadline
}

func (f *fakeController) Close(ctx context.Context) error { return nil }
