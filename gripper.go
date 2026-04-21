package waveshareroarm

import (
	"context"
	stdlib_errors "errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
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

// armRPC is the narrow slice of the arm.Arm gRPC client the gripper consumes.
// Dependencies resolved via resource.Dependencies give us a gRPC client, not
// the local *roarmM3 struct, so every controller interaction must round-trip
// through DoCommand / IsMoving on that client.
type armRPC interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
	IsMoving(ctx context.Context) (bool, error)
}

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

	name      resource.Name
	logger    logging.Logger
	armClient armRPC
	model     referenceframe.Model
	opMgr     *operation.SingleOperationManager

	// State management
	mu      sync.Mutex
	holding atomic.Bool
	closed  atomic.Bool
}

var errGripperClosed = stdlib_errors.New("gripper closed")

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

	g := &roarmM3Gripper{
		name:      conf.ResourceName(),
		logger:    logger,
		armClient: armRes,
		model:     referenceframe.NewSimpleModel("roarm_m3_gripper"),
		opMgr:     operation.NewSingleOperationManager(),
	}

	return g, nil
}

func (g *roarmM3Gripper) Name() resource.Name {
	return g.name
}

// setGripperRad commands joint 6 to a software-frame radian via the arm's
// DoCommand bridge. Speed/acc defaults mirror the arm's internal defaults.
func (g *roarmM3Gripper) setGripperRad(ctx context.Context, rad float64, speed, acc int) error {
	_, err := g.armClient.DoCommand(ctx, map[string]interface{}{
		"command": "set_gripper_rad",
		"rad":     rad,
		"speed":   float64(speed),
		"acc":     float64(acc),
	})
	return err
}

// getGripperRad reads the current software-frame gripper position via the
// arm's DoCommand bridge.
func (g *roarmM3Gripper) getGripperRad(ctx context.Context) (float64, error) {
	out, err := g.armClient.DoCommand(ctx, map[string]interface{}{
		"command": "get_gripper_rad",
	})
	if err != nil {
		return 0, err
	}
	rad, ok := out["rad"].(float64)
	if !ok {
		return 0, fmt.Errorf("get_gripper_rad: missing or non-numeric 'rad' in response: %v", out)
	}
	return rad, nil
}

// Open opens the gripper (moves joint 6 to its upper limit (fully open))
func (g *roarmM3Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	if g.closed.Load() {
		return errGripperClosed
	}
	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	if err := g.setGripperRad(ctx, gripperOpenRad, 500, 50); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}
	g.holding.Store(false)

	// Wait for movement to complete
	select {
	case <-time.After(1 * time.Second):
	case <-ctx.Done():
		return ctx.Err()
	}

	g.logger.Debug("Gripper opened")
	return nil
}

// Grab closes the gripper to grab an object (moves joint 6 to its lower limit (fully closed))
func (g *roarmM3Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	if g.closed.Load() {
		return false, errGripperClosed
	}
	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	if err := g.setGripperRad(ctx, gripperGrabRad, 500, 50); err != nil {
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
	gripperRad, err := g.getGripperRad(ctx)
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		// Assume grab was successful if we can't read position
		return true, nil
	}
	grabbed := gripperRad > gripperGrabRad+0.05 // ~3 degree margin

	g.holding.Store(grabbed)

	if grabbed {
		g.logger.Debug("Gripper successfully grabbed an object")
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop stops the gripper movement
func (g *roarmM3Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	if g.closed.Load() {
		return errGripperClosed
	}
	g.opMgr.CancelRunning(ctx)
	_, err := g.armClient.DoCommand(ctx, map[string]interface{}{
		"command": "stop_gripper",
	})
	return err
}

// IsMoving returns whether the gripper is currently moving
func (g *roarmM3Gripper) IsMoving(ctx context.Context) (bool, error) {
	if g.closed.Load() {
		return false, errGripperClosed
	}
	return g.armClient.IsMoving(ctx)
}

// ModelFrame returns the reference frame model for the gripper
func (g *roarmM3Gripper) ModelFrame() referenceframe.Model {
	return g.model
}

// Additional helper methods for gripper control

// GetPosition returns the current gripper position in degrees (-10 to 100).
// Reads the 6th joint radian from the shared controller and converts.
func (g *roarmM3Gripper) GetPosition(ctx context.Context) (float64, error) {
	if g.closed.Load() {
		return 0, errGripperClosed
	}
	rad, err := g.getGripperRad(ctx)
	if err != nil {
		return 0, fmt.Errorf("failed to read gripper position: %w", err)
	}
	return rad * 180.0 / math.Pi, nil
}

// SetPosition sets the gripper to a specific position (-10 to 100 degrees).
// Internally this commands joint 6 directly via the arm's DoCommand bridge.
func (g *roarmM3Gripper) SetPosition(ctx context.Context, angleDegrees float64, speed, acc int) error {
	if g.closed.Load() {
		return errGripperClosed
	}
	if angleDegrees < -10 || angleDegrees > 100 {
		return fmt.Errorf("gripper angle must be between -10 and 100 degrees, got %.1f", angleDegrees)
	}

	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	radians := angleDegrees * math.Pi / 180.0
	if err := g.setGripperRad(ctx, radians, speed, acc); err != nil {
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
func (g *roarmM3Gripper) Close(ctx context.Context) error {
	if !g.closed.CompareAndSwap(false, true) {
		return nil
	}
	g.opMgr.CancelRunning(ctx)
	return nil
}

func (g *roarmM3Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	if g.closed.Load() {
		return nil, errGripperClosed
	}
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
	if g.closed.Load() {
		return errGripperClosed
	}
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
	if g.closed.Load() {
		return nil, errGripperClosed
	}
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
	if g.closed.Load() {
		return nil, errGripperClosed
	}
	// ~70mm wide, 40mm tall, 60mm deep jaw envelope, offset 30mm beyond the wrist.
	offset := spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 30})
	box, err := spatialmath.NewBox(offset, r3.Vector{X: 70, Y: 40, Z: 60}, "gripper-box")
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{box}, nil
}

func (g *roarmM3Gripper) IsHoldingSomething(ctx context.Context, _ map[string]interface{}) (gripper.HoldingStatus, error) {
	if g.closed.Load() {
		return gripper.HoldingStatus{}, errGripperClosed
	}
	return gripper.HoldingStatus{IsHoldingSomething: g.holding.Load()}, nil
}

func (g *roarmM3Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}
