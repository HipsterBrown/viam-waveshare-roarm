package gripper

import (
	"context"
	stdlib_errors "errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"
	rdkgripper "go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/roarm"
)

var (
	Model = resource.NewModel("hipsterbrown", "waveshare-roarm", "gripper")
)

var (
	// gripperOpenRad / gripperGrabRad are the joint-6 limits from
	// geometry.GripperJointLimits. Using the actual limit extremes avoids the
	// off-by-range problem that existed when these were hardcoded degree
	// values (100, -10) combined with the pi-minus-radian transform.
	gripperOpenRad = geometry.GripperJointLimits[1] // upper limit of joint 6 (fully open)
	gripperGrabRad = geometry.GripperJointLimits[0] // lower limit of joint 6 (fully closed)
)

// grabMarginRad is how far short of the closed limit the jaw has to stop
// (~3 degrees) before Grab calls it an object rather than an empty close.
const grabMarginRad = 0.05

// armRPC is the narrow slice of the rdkarm.Arm gRPC client the gripper consumes.
// Dependencies resolved via resource.Dependencies give us a gRPC client, not
// the local *roarmM3 struct, so every joint-6 interaction round-trips through
// DoCommand on that client (see internal/roarm/bridge.go).
type armRPC interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// RoArmGripperConfig configuration for the RoArm-M3 gripper.
// The gripper borrows the serial/HTTP handle from the arm it is paired
// with, so we only need a reference to the arm resource.
type RoArmGripperConfig struct {
	// Arm is the name of the arm resource supplying the controller.
	Arm string `json:"arm"`

	// CollisionGeometry selects the jaw's collision shape: "" or "box" for a
	// bounding box, "mesh" for the decimated jaw hull.
	CollisionGeometry string `json:"collision_geometry,omitempty"`
}

// Validate validates the gripper config and declares the arm dependency.
func (cfg *RoArmGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("%s: must specify arm dependency", path)
	}
	if err := geometry.ValidateCollision(cfg.CollisionGeometry); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
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
	// sleep is the seam IsMoving's two-sample probe uses; tests make it
	// instant. Production uses roarm.SleepCtx.
	sleep func(context.Context, time.Duration) error

	// State management
	mu         sync.Mutex
	holding    atomic.Bool
	closed     atomic.Bool
	opInFlight atomic.Bool
}

var errGripperClosed = stdlib_errors.New("gripper closed")

func init() {
	resource.RegisterComponent(
		rdkgripper.API,
		Model,
		resource.Registration[rdkgripper.Gripper, *RoArmGripperConfig]{
			Constructor: newRoArmM3Gripper,
		},
	)
}

func newRoArmM3Gripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (rdkgripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*RoArmGripperConfig](conf)
	if err != nil {
		return nil, err
	}

	armRes, err := rdkarm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("gripper %s: could not find arm %q in deps: %w", conf.ResourceName(), cfg.Arm, err)
	}

	model, err := geometry.GripperModel(cfg.CollisionGeometry, conf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	g := &roarmM3Gripper{
		name:      conf.ResourceName(),
		logger:    logger,
		armClient: armRes,
		model:     model,
		opMgr:     operation.NewSingleOperationManager(),
		sleep:     roarm.SleepCtx,
	}

	return g, nil
}

func (g *roarmM3Gripper) Name() resource.Name {
	return g.name
}

func (g *roarmM3Gripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, nil
}

// setGripperRad commands joint 6 to a software-frame radian via the arm's
// DoCommand bridge. See internal/roarm/bridge.go for the protocol.
// The arm side settles on joint 6 before returning, so no sleep follows.
func (g *roarmM3Gripper) setGripperRad(ctx context.Context, rad, speedDegs, accDegs float64) error {
	g.opInFlight.Store(true)
	defer g.opInFlight.Store(false)
	_, err := g.armClient.DoCommand(ctx, map[string]interface{}{
		"command":      roarm.CmdSetGripperRad,
		roarm.KeyRad:   rad,
		roarm.KeySpeed: speedDegs,
		roarm.KeyAcc:   accDegs,
		roarm.KeyWait:  true,
	})
	return err
}

// getGripperRad reads the current software-frame gripper position via the
// arm's DoCommand bridge.
func (g *roarmM3Gripper) getGripperRad(ctx context.Context) (float64, error) {
	out, err := g.armClient.DoCommand(ctx, map[string]interface{}{
		"command": roarm.CmdGetGripperRad,
	})
	if err != nil {
		return 0, err
	}
	rad, ok := out[roarm.KeyRad].(float64)
	if !ok {
		return 0, fmt.Errorf("%s: missing or non-numeric %q in response: %v", roarm.CmdGetGripperRad, roarm.KeyRad, out)
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

	if err := g.setGripperRad(ctx, gripperOpenRad, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}
	g.holding.Store(false)

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

	if err := g.setGripperRad(ctx, gripperGrabRad, roarm.DefaultGripperSpeedDegsPerSec, roarm.DefaultGripperAccDegsPerSecSq); err != nil {
		return false, fmt.Errorf("failed to grab with gripper: %w", err)
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
	grabbed := gripperRad > gripperGrabRad+grabMarginRad

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
		"command": roarm.CmdStopGripper,
	})
	return err
}

// IsMoving reports whether joint 6 is moving, from two position reads a
// probe gap apart. It never consults the arm's own IsMoving: the arm may be
// swinging while the jaw is still.
func (g *roarmM3Gripper) IsMoving(ctx context.Context) (bool, error) {
	if g.closed.Load() {
		return false, errGripperClosed
	}
	if g.opInFlight.Load() {
		return true, nil
	}
	a, err := g.getGripperRad(ctx)
	if err != nil {
		return false, noFeedbackAsFalse(err)
	}
	if err := g.sleep(ctx, roarm.IsMovingProbeGap); err != nil {
		return false, err
	}
	b, err := g.getGripperRad(ctx)
	if err != nil {
		return false, noFeedbackAsFalse(err)
	}
	return math.Abs(a-b) > roarm.StallRad, nil
}

// noFeedbackAsFalse maps the no-feedback bridge error (which arrives as
// text after gRPC) to nil so IsMoving answers false, matching the arm.
func noFeedbackAsFalse(err error) error {
	if strings.Contains(err.Error(), roarm.NoFeedbackMarker) {
		return nil
	}
	return err
}

// Additional helper methods for gripper control

// GetPosition returns the current gripper position in degrees.
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

// SetPosition sets the gripper to a specific position, in degrees within
// joint 6's limits.
// Internally this commands joint 6 directly via the arm's DoCommand bridge.
func (g *roarmM3Gripper) SetPosition(ctx context.Context, angleDegrees, speedDegs, accDegs float64) error {
	if g.closed.Load() {
		return errGripperClosed
	}
	radians := angleDegrees * math.Pi / 180.0
	if radians < geometry.GripperJointLimits[0] || radians > geometry.GripperJointLimits[1] {
		return fmt.Errorf("gripper angle must be between %.1f and %.1f degrees, got %.1f",
			geometry.GripperJointLimits[0]*180/math.Pi, geometry.GripperJointLimits[1]*180/math.Pi, angleDegrees)
	}

	g.mu.Lock()
	defer g.mu.Unlock()

	ctx, done := g.opMgr.New(ctx)
	defer done()

	if err := g.setGripperRad(ctx, radians, speedDegs, accDegs); err != nil {
		return fmt.Errorf("failed to set gripper position: %w", err)
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
	return []referenceframe.Input{}, nil
}

func (g *roarmM3Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	if g.closed.Load() {
		return errGripperClosed
	}
	for _, inputSet := range inputs {
		if len(inputSet) != 0 {
			return fmt.Errorf("the gripper model has no degrees of freedom; use the set_position DoCommand to move the jaw, got %d inputs", len(inputSet))
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
		speed := roarm.DefaultGripperSpeedDegsPerSec
		acc := roarm.DefaultGripperAccDegsPerSecSq
		if s, ok := cmd["speed"].(float64); ok {
			speed = s
		}
		if a, ok := cmd["acc"].(float64); ok {
			acc = a
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
	jaw, err := g.getGripperRad(ctx)
	if err != nil {
		// The viewer is the only consumer; draw the jaw closed rather than fail.
		g.logger.Debugf("Geometries: jaw read failed, drawing closed: %v", err)
		jaw = geometry.GripperJointLimits[0]
	}
	return geometry.GripperMeshes(jaw)
}

func (g *roarmM3Gripper) IsHoldingSomething(ctx context.Context, _ map[string]interface{}) (rdkgripper.HoldingStatus, error) {
	if g.closed.Load() {
		return rdkgripper.HoldingStatus{}, errGripperClosed
	}
	return rdkgripper.HoldingStatus{IsHoldingSomething: g.holding.Load()}, nil
}

func (g *roarmM3Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}
