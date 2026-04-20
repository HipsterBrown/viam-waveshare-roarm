package waveshareroarm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	RoArmM3Gripper = resource.NewModel("hipsterbrown", "waveshare-roarm", "gripper")
)

const (
	// gripperOpenRad / gripperGrabRad match the joint-6 limits from
	// controller.RoArmM3JointLimits. Using the actual limit extremes
	// avoids the off-by-range problem that existed when these were
	// hardcoded degree values (100, -10) combined with the pi-minus-radian
	// transform.
	gripperOpenRad = 1.9  // upper limit of joint 6 (fully open)
	gripperGrabRad = -0.2 // lower limit of joint 6 (fully closed)
)

// RoArmGripperConfig configuration for the RoArm-M3 gripper.
// The gripper borrows the serial/HTTP handle from the arm it is paired
// with, so we only need a reference to the arm resource.
type RoArmGripperConfig struct {
	// Arm is the name of the arm resource supplying the controller.
	Arm string `json:"arm"`
}

// Validate validates the gripper config and declares the arm dependency.
func (cfg *RoArmGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("%s: must specify arm dependency", path)
	}
	return []string{cfg.Arm}, nil, nil
}

// roarmM3Gripper represents the RoArm-M3 gripper
type roarmM3Gripper struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	controller RoArmHandle
	model      referenceframe.Model
	opMgr      *operation.SingleOperationManager

	// State management
	mu       sync.Mutex
	isMoving atomic.Bool
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		RoArmM3Gripper,
		resource.Registration[gripper.Gripper, *RoArmGripperConfig]{
			Constructor: newRoArmM3Gripper,
		},
	)
}

func newRoArmM3Gripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*RoArmGripperConfig](conf)
	if err != nil {
		return nil, err
	}

	armRes, err := arm.FromDependencies(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("gripper %s: could not find arm %q in deps: %w", conf.ResourceName(), cfg.Arm, err)
	}
	armImpl, ok := armRes.(*roarmM3)
	if !ok {
		return nil, fmt.Errorf("gripper %s: arm %q is not a roarm-m3 arm", conf.ResourceName(), cfg.Arm)
	}
	handle := armImpl.Handle()

	g := &roarmM3Gripper{
		name:       conf.ResourceName(),
		logger:     logger,
		controller: handle,
		model:      referenceframe.NewSimpleModel("roarm_m3_gripper"),
		opMgr:      operation.NewSingleOperationManager(),
	}

	return g, nil
}

func (g *roarmM3Gripper) Name() resource.Name {
	return g.name
}

// Open opens the gripper (sets to 100 degrees - fully open)
func (g *roarmM3Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Open position: joint-6 upper limit (fully open).
	err := g.controller.SetJointRadian(ctx, 6, gripperOpenRad, 500, 50)
	if err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	// Wait for movement to complete
	select {
	case <-time.After(1 * time.Second):
	case <-ctx.Done():
		return ctx.Err()
	}

	g.logger.Debug("Gripper opened")
	return nil
}

// Grab closes the gripper to grab an object (sets to -10 degrees - fully closed)
func (g *roarmM3Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Grab position: joint-6 lower limit (fully closed).
	err := g.controller.SetJointRadian(ctx, 6, gripperGrabRad, 500, 50)
	if err != nil {
		return false, fmt.Errorf("failed to grab with gripper: %w", err)
	}

	// Wait for movement to complete
	select {
	case <-time.After(1 * time.Second):
	case <-ctx.Done():
		return false, ctx.Err()
	}

	// Check if something was grabbed by reading the gripper position.
	// If the gripper couldn't fully close to the lower limit, it likely
	// grabbed something.
	radians, err := g.controller.GetJointRadians(ctx)
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		// Assume grab was successful if we can't read position
		return true, nil
	}
	if len(radians) < 6 {
		return true, nil
	}
	gripperRad := radians[5]
	// If we couldn't fully close to the lower limit, something is in the way.
	grabbed := gripperRad > gripperGrabRad+0.05 // ~3 degree margin

	if grabbed {
		g.logger.Debug("Gripper successfully grabbed an object")
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop stops the gripper movement
func (g *roarmM3Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.opMgr.CancelRunning(ctx)
	positions, err := g.controller.GetJointRadians(ctx)
	if err != nil {
		return fmt.Errorf("gripper stop: read position: %w", err)
	}
	if len(positions) < 6 {
		return fmt.Errorf("gripper stop: short feedback (got %d joints)", len(positions))
	}
	return g.controller.SetJointRadian(ctx, 6, positions[5], 100, 50)
}

// IsMoving returns whether the gripper is currently moving
func (g *roarmM3Gripper) IsMoving(ctx context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

// ModelFrame returns the reference frame model for the gripper
func (g *roarmM3Gripper) ModelFrame() referenceframe.Model {
	return g.model
}

// Additional helper methods for gripper control

// GetPosition returns the current gripper position in degrees (-10 to 100).
// Reads the 6th joint radian from the shared controller and converts.
func (g *roarmM3Gripper) GetPosition(ctx context.Context) (float64, error) {
	radians, err := g.controller.GetJointRadians(ctx)
	if err != nil {
		return 0, fmt.Errorf("failed to read gripper position: %w", err)
	}
	if len(radians) < 6 {
		return 0, fmt.Errorf("expected at least 6 joint positions, got %d", len(radians))
	}
	return radians[5] * 180.0 / math.Pi, nil
}

// SetPosition sets the gripper to a specific position (-10 to 100 degrees).
// Internally this commands joint 6 directly via the narrow handle.
func (g *roarmM3Gripper) SetPosition(ctx context.Context, angleDegrees float64, speed, acc int) error {
	if angleDegrees < -10 || angleDegrees > 100 {
		return fmt.Errorf("gripper angle must be between -10 and 100 degrees, got %.1f", angleDegrees)
	}

	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	radians := angleDegrees * math.Pi / 180.0
	if err := g.controller.SetJointRadian(ctx, 6, radians, speed, acc); err != nil {
		return fmt.Errorf("failed to set gripper position: %w", err)
	}

	// Calculate wait time based on speed
	moveTime := time.Duration(float64(time.Second) * 2000.0 / float64(speed))
	if moveTime > 5*time.Second {
		moveTime = 5 * time.Second
	}
	if moveTime < 100*time.Millisecond {
		moveTime = 100 * time.Millisecond
	}

	select {
	case <-time.After(moveTime):
	case <-ctx.Done():
		return ctx.Err()
	}

	return nil
}

// Close releases any gripper-local resources. The underlying controller is
// owned by the arm and must not be closed here.
func (g *roarmM3Gripper) Close(context.Context) error {
	return nil
}

func (g *roarmM3Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	position, err := g.GetPosition(ctx)
	if err != nil {
		return nil, err
	}

	// Convert degrees to radians
	radians := position * math.Pi / 180.0

	return []referenceframe.Input{
		{Value: radians},
	}, nil
}

func (g *roarmM3Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	if len(inputs) == 0 {
		return nil
	}

	for _, inputSet := range inputs {
		if len(inputSet) != 1 {
			return fmt.Errorf("expected 1 input for gripper, got %d", len(inputSet))
		}

		// Convert radians to degrees
		degrees := inputSet[0].Value * 180.0 / math.Pi

		if err := g.SetPosition(ctx, degrees, 500, 50); err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}
	}

	return nil
}

func (g *roarmM3Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_position":
		position, err := g.GetPosition(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{
			"position_degrees": position,
			"position_radians": position * math.Pi / 180.0,
		}, nil

	case "set_position":
		degrees, ok := cmd["degrees"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_position command requires 'degrees' number parameter")
		}
		speed := 500
		acc := 50
		if s, ok := cmd["speed"].(float64); ok {
			speed = int(s)
		}
		if a, ok := cmd["acc"].(float64); ok {
			acc = int(a)
		}
		err := g.SetPosition(ctx, degrees, speed, acc)
		return map[string]interface{}{"success": err == nil}, err

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (g *roarmM3Gripper) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, errors.ErrUnsupported
}

func (g *roarmM3Gripper) IsHoldingSomething(ctx context.Context, _ map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, errors.ErrUnsupported
}

func (g *roarmM3Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}
