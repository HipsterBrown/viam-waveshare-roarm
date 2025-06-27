package waveshareroarm

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	RoArmM3Gripper = resource.NewModel("hipsterbrown", "waveshare-roarm", "gripper")
)

// RoArmGripperConfig configuration for the RoArm-M3 gripper
type RoArmGripperConfig struct {
	// HTTP configuration
	Host string `json:"host,omitempty"`

	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Common configuration
	Timeout time.Duration `json:"timeout,omitempty"`
}

// Validate validates the gripper config
func (cfg *RoArmGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" && cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify either host for HTTP or port for serial communication")
	}
	if cfg.Host != "" && cfg.Port != "" {
		return nil, nil, fmt.Errorf("cannot specify both host and port, choose either HTTP or serial communication")
	}
	return nil, nil, nil
}

// roarmM3Gripper represents the RoArm-M3 gripper
type roarmM3Gripper struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	controller *RoArmController
	model      referenceframe.Model

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

	// Create controller configuration
	controllerConfig := &RoArmConfig{
		Host:     cfg.Host,
		Port:     cfg.Port,
		Baudrate: cfg.Baudrate,
		Timeout:  cfg.Timeout,
		Logger:   logger,
	}

	controller, err := NewRoArmController(controllerConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create RoArm controller: %w", err)
	}

	g := &roarmM3Gripper{
		name:       conf.ResourceName(),
		logger:     logger,
		controller: controller,
		model:      referenceframe.NewSimpleModel("roarm_m3_gripper"),
	}

	return g, nil
}

func (g *roarmM3Gripper) Name() resource.Name {
	return g.name
}

// Open opens the gripper (sets to -10 degrees - fully open)
func (g *roarmM3Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Open position: -10 degrees (fully open based on SDK limits)
	err := g.controller.SetGripperPosition(-10, 500, 50)
	if err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(1 * time.Second)

	g.logger.Debug("Gripper opened")
	return nil
}

// Grab closes the gripper to grab an object (sets to 100 degrees - fully closed)
func (g *roarmM3Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Grab position: 100 degrees (fully closed based on SDK limits)
	err := g.controller.SetGripperPosition(100, 500, 50)
	if err != nil {
		return false, fmt.Errorf("failed to grab with gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(1 * time.Second)

	// Check if something was grabbed by reading the gripper position
	// If the gripper couldn't close fully, it likely grabbed something
	position, err := g.controller.GetGripperPosition()
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		// Assume grab was successful if we can't read position
		return true, nil
	}

	// If the gripper is significantly less than 100 degrees, something is blocking it
	grabbed := position < 90.0

	if grabbed {
		g.logger.Debug("Gripper successfully grabbed an object")
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop stops the gripper movement
func (g *roarmM3Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	// The RoArm controller doesn't have a direct stop command
	// Movement will complete, but we mark as not moving
	g.logger.Debug("Gripper stop requested")
	return nil
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

// GetPosition returns the current gripper position (0-90 degrees)
func (g *roarmM3Gripper) GetPosition(ctx context.Context) (float64, error) {
	position, err := g.controller.GetGripperPosition()
	if err != nil {
		return 0, fmt.Errorf("failed to read gripper position: %w", err)
	}

	return position, nil
}

// SetPosition sets the gripper to a specific position (-10 to 100 degrees)
func (g *roarmM3Gripper) SetPosition(ctx context.Context, angleDegrees float64, speed, acc int) error {
	if angleDegrees < -10 || angleDegrees > 100 {
		return fmt.Errorf("gripper angle must be between -10 and 100 degrees, got %.1f", angleDegrees)
	}

	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	err := g.controller.SetGripperPosition(angleDegrees, speed, acc)
	if err != nil {
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

	time.Sleep(moveTime)

	return nil
}

func (g *roarmM3Gripper) Close(context.Context) error {
	return g.controller.Close()
}

func (g *roarmM3Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	position, err := g.GetPosition(ctx)
	if err != nil {
		return nil, err
	}

	// Convert degrees to radians
	radians := position * 3.14159265359 / 180.0

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
		degrees := inputSet[0].Value * 180.0 / 3.14159265359

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
			"position_radians": position * 3.14159265359 / 180.0,
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
