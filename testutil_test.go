package waveshareroarm

import (
	"context"
	"fmt"
	"sync"
	"time"
)

// Compile-time check that fakeController satisfies RoArmHandle.
var _ RoArmHandle = (*fakeController)(nil)

// fakeController implements RoArmHandle for tests.
type fakeController struct {
	mu                sync.Mutex
	LastSpeed         int
	LastAcc           int
	LastRadians       []float64
	LastJoint         int
	LastTorque        *bool
	LastLED           *int
	FeedbackCalls     int
	Feedback          FeedbackData
	FailOn            string // method name to return error from, empty = never
	FailWith          error  // error FailOn returns; nil means a generic fake error
	Moving            bool   // what IsMoving reports
	HoldStill         bool   // when true, SetJointRadian(s) do not update Feedback (a blocked jaw, a stalled arm)
	SettleCalls       int
	LastSettleTimeout time.Duration // timeout passed to the most recent WaitUntilSettled
	WriteCount        int           // SetJointRadian(s) calls
	Closed            bool          // set by Close
}

func (f *fakeController) err(method string) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.FailOn == method {
		if f.FailWith != nil {
			return f.FailWith
		}
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

func (f *fakeController) SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error {
	if err := f.err("SetJointRadian"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.WriteCount++
	f.LastJoint, f.LastSpeed, f.LastAcc = joint, speed, acc
	if cap(f.LastRadians) < 6 {
		f.LastRadians = make([]float64, 6)
	}
	f.LastRadians = f.LastRadians[:6]
	f.LastRadians[joint-1] = radian
	if !f.HoldStill {
		cur := f.currentLocked()
		cur[joint-1] = radian
		f.setFeedback(cur)
	}
	return nil
}

func (f *fakeController) SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error {
	if err := f.err("SetJointRadians"); err != nil {
		return err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.WriteCount++
	f.LastRadians = append([]float64(nil), radians...)
	f.LastSpeed, f.LastAcc = speed, acc
	if !f.HoldStill {
		f.setFeedback(radians)
	}
	return nil
}

func (f *fakeController) GetJointRadians(ctx context.Context) ([]float64, error) {
	if err := f.err("GetJointRadians"); err != nil {
		return nil, err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.currentLocked(), nil
}

// currentLocked returns the six Feedback joints; the mutex must be held.
func (f *fakeController) currentLocked() []float64 {
	return []float64{f.Feedback.B, f.Feedback.S, f.Feedback.E, f.Feedback.Wrist, f.Feedback.R, f.Feedback.G}
}

// setFeedback writes radians into the Feedback frame joints (software frame).
func (f *fakeController) setFeedback(radians []float64) {
	fb := &f.Feedback
	for i, v := range radians {
		switch i {
		case 0:
			fb.B = v
		case 1:
			fb.S = v
		case 2:
			fb.E = v
		case 3:
			fb.Wrist = v
		case 4:
			fb.R = v
		case 5:
			fb.G = v
		}
	}
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

func (f *fakeController) WaitUntilSettled(ctx context.Context, target []float64, mask []bool, timeout time.Duration) ([]float64, error) {
	if err := f.err("WaitUntilSettled"); err != nil {
		return nil, err
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.SettleCalls++
	f.LastSettleTimeout = timeout
	return f.currentLocked(), nil
}

func (f *fakeController) IsMoving(ctx context.Context) (bool, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.Moving, nil
}

func (f *fakeController) Close(ctx context.Context) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.Closed = true
	return nil
}

// fakeArmRPC implements the narrow armRPC interface the gripper consumes.
// Joint6Rad is the simulated position; a set moves it there unless HoldStill
// (a blocked jaw). Joint6Series, when non-empty, is returned one value per
// get before falling back to Joint6Rad, so IsMoving's two reads can differ.
type fakeArmRPC struct {
	mu             sync.Mutex
	Joint6Rad      float64
	Joint6Series   []float64
	HoldStill      bool
	ArmMoving      bool
	LastCommand    string
	LastSetRad     float64
	LastSetSpeed   float64
	LastSetAcc     float64
	LastWait       bool
	StopCalls      int
	DoCommandError error
}

func (f *fakeArmRPC) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.DoCommandError != nil {
		return nil, f.DoCommandError
	}
	name, _ := cmd["command"].(string)
	f.LastCommand = name
	switch name {
	case cmdGetGripperRad:
		if len(f.Joint6Series) > 0 {
			v := f.Joint6Series[0]
			f.Joint6Series = f.Joint6Series[1:]
			return map[string]interface{}{keyRad: v}, nil
		}
		return map[string]interface{}{keyRad: f.Joint6Rad}, nil
	case cmdSetGripperRad:
		rad, _ := cmd[keyRad].(float64)
		f.LastSetRad = rad
		f.LastSetSpeed, _ = cmd[keySpeed].(float64)
		f.LastSetAcc, _ = cmd[keyAcc].(float64)
		f.LastWait = true
		if w, ok := cmd[keyWait].(bool); ok {
			f.LastWait = w
		}
		if !f.HoldStill {
			f.Joint6Rad = rad
		}
		return map[string]interface{}{"success": true}, nil
	case cmdStopGripper:
		f.StopCalls++
		f.LastSetRad = f.Joint6Rad
		return map[string]interface{}{"success": true}, nil
	}
	return nil, fmt.Errorf("fakeArmRPC: unknown command %q", name)
}

func (f *fakeArmRPC) IsMoving(ctx context.Context) (bool, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.ArmMoving, nil
}
