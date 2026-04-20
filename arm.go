package waveshareroarm

import (
	"context"
	_ "embed"
	"encoding/json"
	stdlib_errors "errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

var (
	RoArmM3 = resource.NewModel("hipsterbrown", "waveshare-roarm", "arm")

	errClosed = stdlib_errors.New("arm closed")
)

//go:embed roarm_m3.json
var roarmModelJson []byte

func init() {
	resource.RegisterComponent(arm.API, RoArmM3,
		resource.Registration[arm.Arm, *RoArmM3Config]{
			Constructor: newRoArmM3,
		},
	)
}

// RoArmM3Config represents the configuration for the RoArm-M3
type RoArmM3Config struct {
	// HTTP configuration
	Host string `json:"host,omitempty"`

	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Common configuration
	HTTPTimeout   Duration `json:"http_timeout,omitempty"`
	SerialTimeout Duration `json:"serial_timeout,omitempty"`

	// Motion configuration
	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
}

var validBaudrates = map[int]bool{
	0: true, // zero means "use default" (115200)
	9600: true, 19200: true, 38400: true, 57600: true,
	115200: true, 230400: true, 921600: true, 1000000: true,
}

// Validate ensures all parts of the config are valid
func (cfg *RoArmM3Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" && cfg.Port == "" {
		return nil, nil, fmt.Errorf("%s: must specify either host or port", path)
	}
	if cfg.Host != "" && cfg.Port != "" {
		return nil, nil, fmt.Errorf("%s: cannot specify both host and port", path)
	}
	if !validBaudrates[cfg.Baudrate] {
		return nil, nil, fmt.Errorf("%s: baudrate %d not supported", path, cfg.Baudrate)
	}
	return nil, nil, nil
}

type roarmM3 struct {
	name       resource.Name
	logger     logging.Logger
	cfg        *RoArmM3Config
	opMgr      *operation.SingleOperationManager
	controller *RoArmController

	mu          sync.RWMutex
	model       referenceframe.Model
	jointLimits [][2]float64

	// Motion configuration
	defaultSpeed int
	defaultAcc   int

	closed atomic.Bool

	cancelCtx  context.Context
	cancelFunc func()
}

func makeRoArmModelFrame() (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     roarmModelJson,
			Extension: "json",
		},
	}
	err := json.Unmarshal(roarmModelJson, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	return m.ParseConfig("roarm_m3")
}

func newRoArmM3(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*RoArmM3Config](rawConf)
	if err != nil {
		return nil, err
	}

	// Validate and set default motion parameters
	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = 50 // Default speed in degrees per second
	}
	if speedDegsPerSec < 3 || speedDegsPerSec > 180 {
		return nil, fmt.Errorf("speed_degs_per_sec must be between 3 and 180 degrees/second, got %.1f", speedDegsPerSec)
	}

	accelerationDegsPerSec := conf.AccelerationDegsPerSec
	if accelerationDegsPerSec == 0 {
		accelerationDegsPerSec = 100 // Default acceleration in degrees per second^2
	}
	if accelerationDegsPerSec < 10 || accelerationDegsPerSec > 500 {
		return nil, fmt.Errorf("acceleration_degs_per_sec_per_sec must be between 10 and 500 degrees/second^2, got %.1f", accelerationDegsPerSec)
	}

	// Convert degrees/sec to internal speed units (approximate conversion based on RoArm SDK).
	// Preserve the 30-unit (~3 deg/s) floor for safety margin beyond the helper's minSpeedUnits=1.
	defaultSpeed := speedToUnits(float64(speedDegsPerSec))
	if defaultSpeed < 30 {
		defaultSpeed = 30
	}

	// Convert degrees/sec^2 to internal acceleration units.
	defaultAcc := accelToUnits(float64(accelerationDegsPerSec))

	// Create controller configuration
	controllerConfig := &RoArmConfig{
		Host:          conf.Host,
		Port:          conf.Port,
		Baudrate:      conf.Baudrate,
		HTTPTimeout:   conf.HTTPTimeout,
		SerialTimeout: conf.SerialTimeout,
		Logger:        logger,
	}

	controller, err := NewRoArmController(controllerConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create RoArm controller: %w", err)
	}

	model, err := makeRoArmModelFrame()
	if err != nil {
		_ = controller.Close(ctx) // Clean up on error
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	arm := &roarmM3{
		name:         rawConf.ResourceName(),
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
		logger:       logger,
		controller:   controller,
		model:        model,
		jointLimits:  RoArmM3JointLimits[:5], // Only first 5 joints
		defaultSpeed: defaultSpeed,
		defaultAcc:   defaultAcc,
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}

	logger.Infof("RoArm-M3 configured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		speedDegsPerSec, defaultSpeed, accelerationDegsPerSec, defaultAcc)

	return arm, nil
}

func (r *roarmM3) Name() resource.Name {
	return r.name
}

func (r *roarmM3) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	return nil, stdlib_errors.ErrUnsupported
}

func (r *roarmM3) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	if r.closed.Load() {
		return nil, errClosed
	}
	r.mu.RLock()
	defer r.mu.RUnlock()

	inputs, err := r.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := r.model.Transform(inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

func (r *roarmM3) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	if err := motion.MoveArm(ctx, r.logger, r, pose); err != nil {
		return err
	}
	return nil
}

func (r *roarmM3) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	ctx, done := r.opMgr.New(ctx)
	defer done()

	if len(positions) != 5 {
		return fmt.Errorf("expected 5 joint positions for arm, got %d", len(positions))
	}

	values := make([]float64, len(positions))
	for i, input := range positions {
		values[i] = input.Value
	}

	// Validate input ranges and clamp positions for the 5 arm joints
	clampedPositions := make([]float64, len(values))
	for i, pos := range values {
		min, max := r.jointLimits[i][0], r.jointLimits[i][1]

		// Validate and clamp the position
		if pos < min || pos > max {
			r.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				i+1, pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clampedPositions[i] = math.Max(min, math.Min(max, pos))
	}

	// Get current gripper position to preserve it
	currentFullPositions, err := r.controller.GetJointRadians(ctx)
	if err != nil {
		return fmt.Errorf("MoveToJointPositions: read current positions: %w", err)
	}
	if len(currentFullPositions) < 6 {
		return fmt.Errorf("MoveToJointPositions: short feedback (got %d joints)", len(currentFullPositions))
	}
	currentGripperPos := currentFullPositions[5] // Joint 6 (gripper)

	// Create full 6-joint array with arm positions + current gripper position
	fullPositions := make([]float64, 6)
	copy(fullPositions, clampedPositions)
	fullPositions[5] = currentGripperPos // Preserve gripper position

	// Use configured speed and acceleration
	speed := r.defaultSpeed
	acc := r.defaultAcc

	// Check for speed/acceleration overrides in extra parameters
	if extra != nil {
		if speedOverride, ok := extra["speed"]; ok {
			if speedVal, ok := speedOverride.(float64); ok {
				// Convert from degrees/sec to internal units; preserve 30-unit safety floor.
				speed = speedToUnits(speedVal)
				if speed < 30 {
					speed = 30
				}
			}
		}
		if accOverride, ok := extra["acceleration"]; ok {
			if accVal, ok := accOverride.(float64); ok {
				// Convert from degrees/sec^2 to internal units.
				acc = accelToUnits(accVal)
			}
		}
	}

	// Send command to controller with all 6 joints (including preserved gripper)
	if err := r.controller.SetJointRadians(ctx, fullPositions, speed, acc); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}

	// Calculate wait time based on movement distance and configured speed
	currentArmPositions := make([]float64, 5)
	if len(currentFullPositions) >= 5 {
		copy(currentArmPositions, currentFullPositions[:5])
	}

	maxMovement := 0.0
	for i, target := range clampedPositions {
		movement := math.Abs(target - currentArmPositions[i])
		if movement > maxMovement {
			maxMovement = movement
		}
	}

	// Calculate move time based on configured speed (convert internal units back to rad/sec)
	speedDegPerSec := speedFromUnits(speed)
	speedRadPerSec := speedDegPerSec * math.Pi / 180.0
	moveTimeSeconds := maxMovement / speedRadPerSec
	if moveTimeSeconds < 0.1 {
		moveTimeSeconds = 0.1 // Minimum move time
	}
	if moveTimeSeconds > 10.0 {
		moveTimeSeconds = 10.0 // Maximum move time for safety
	}

	// Refine the motion tracker deadline with our better per-move estimate so
	// IsMoving reflects reality more closely than the controller's worst-case
	// fallback.
	moveDuration := time.Duration(moveTimeSeconds * float64(time.Second))
	r.controller.NoteMotionDeadline(time.Now().Add(moveDuration))

	// Wait for movement to complete
	select {
	case <-time.After(moveDuration):
	case <-ctx.Done():
		return ctx.Err()
	}

	return nil
}

func (r *roarmM3) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	for _, jointPositions := range positions {
		if err := r.MoveToJointPositions(ctx, jointPositions, extra); err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

func (r *roarmM3) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	if r.closed.Load() {
		return nil, errClosed
	}
	r.mu.RLock()
	defer r.mu.RUnlock()

	// Get all joint positions from controller (includes gripper)
	allRadians, err := r.controller.GetJointRadians(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to read joint positions: %w", err)
	}

	// Only return the first 5 joints (arm joints, excluding gripper)
	if len(allRadians) < 5 {
		return nil, fmt.Errorf("expected at least 5 joint positions, got %d", len(allRadians))
	}

	armRadians := allRadians[:5]

	// Convert to Viam input format
	positions := make([]referenceframe.Input, 5)
	for i, radian := range armRadians {
		positions[i] = referenceframe.Input{Value: radian}
	}

	return positions, nil
}

func (r *roarmM3) Stop(ctx context.Context, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	r.opMgr.CancelRunning(ctx)

	current, err := r.controller.GetJointRadians(ctx)
	if err != nil {
		return fmt.Errorf("stop: read current positions: %w", err)
	}
	if len(current) < 6 {
		return fmt.Errorf("stop: short feedback from controller (got %d joints)", len(current))
	}
	const stopSpeed = 100 // internal units, ~10 deg/s — gentle soft stop
	return r.controller.SetJointRadians(ctx, current, stopSpeed, r.defaultAcc)
}

func (r *roarmM3) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return r.model, nil
}

func (r *roarmM3) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return r.JointPositions(ctx, nil)
}

func (r *roarmM3) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return r.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (r *roarmM3) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if r.closed.Load() {
		return nil, errClosed
	}
	// Handle custom commands specific to RoArm
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := r.controller.SetTorque(ctx, enable)
		return map[string]interface{}{"success": err == nil}, err

	case "set_led":
		brightness, ok := cmd["brightness"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_led command requires 'brightness' number parameter")
		}
		err := r.controller.SetLED(ctx, int(brightness))
		return map[string]interface{}{"success": err == nil}, err

	case "move_to_home":
		err := r.controller.MoveToHome(ctx)
		return map[string]interface{}{"success": err == nil}, err

	case "get_feedback":
		feedback, err := r.controller.GetFeedback(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{
			"position": map[string]interface{}{
				"x": feedback.X,
				"y": feedback.Y,
				"z": feedback.Z,
			},
			"joints": map[string]interface{}{
				"base":     feedback.B,
				"shoulder": feedback.S,
				"elbow":    feedback.E,
				"wrist":    feedback.Wrist,
				"roll":     feedback.R,
				"gripper":  feedback.G,
			},
			"torques": map[string]interface{}{
				"base":     feedback.TB,
				"shoulder": feedback.TS,
				"elbow":    feedback.TE,
				"wrist":    feedback.TT,
				"roll":     feedback.TR,
				"gripper":  feedback.TG,
			},
		}, nil

	case "set_speed":
		speed, ok := cmd["value"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_speed requires 'value' number")
		}
		if speed < 3 || speed > 180 {
			return nil, fmt.Errorf("speed out of range: %.1f", speed)
		}
		r.mu.Lock()
		r.defaultSpeed = speedToUnits(speed)
		if r.defaultSpeed < 30 {
			r.defaultSpeed = 30
		}
		r.mu.Unlock()
		return map[string]interface{}{"speed_set": speed}, nil

	case "set_acceleration":
		acc, ok := cmd["value"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_acceleration requires 'value' number")
		}
		if acc < 10 || acc > 500 {
			return nil, fmt.Errorf("accel out of range: %.1f", acc)
		}
		r.mu.Lock()
		r.defaultAcc = accelToUnits(acc)
		r.mu.Unlock()
		return map[string]interface{}{"acceleration_set": acc}, nil

	case "get_motion_params":
		r.mu.Lock()
		defer r.mu.Unlock()
		return map[string]interface{}{
			"current_speed_degs_per_sec":                speedFromUnits(r.defaultSpeed),
			"current_acceleration_degs_per_sec_per_sec": accelFromUnits(r.defaultAcc),
		}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (r *roarmM3) IsMoving(ctx context.Context) (bool, error) {
	if r.closed.Load() {
		return false, errClosed
	}
	return r.controller.IsMoving(ctx)
}

func (r *roarmM3) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	if r.closed.Load() {
		return nil, errClosed
	}
	inputs, err := r.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := r.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (r *roarmM3) Close(ctx context.Context) error {
	if !r.closed.CompareAndSwap(false, true) {
		return nil
	}
	r.cancelFunc()
	r.opMgr.CancelRunning(ctx)
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.controller != nil {
		return r.controller.Close(ctx)
	}
	return nil
}

// Handle returns the underlying controller for sibling resources (e.g. gripper).
// Package-private: not part of the public Viam API.
func (r *roarmM3) Handle() RoArmHandle {
	return r.controller
}

// Reconfigure updates the arm's configuration. Connectivity-affecting fields
// (Host/Port/Baudrate/timeouts) trigger a controller reopen. Motion-only
// changes (speed/acceleration) update in place without tearing down the link.
func (r *roarmM3) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*RoArmM3Config](conf)
	if err != nil {
		return err
	}

	// Validate motion parameters before we touch anything.
	speedDegsPerSec := newConf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = 50
	}
	if speedDegsPerSec < 3 || speedDegsPerSec > 180 {
		return fmt.Errorf("speed_degs_per_sec must be between 3 and 180 degrees/second, got %.1f", speedDegsPerSec)
	}

	accelerationDegsPerSec := newConf.AccelerationDegsPerSec
	if accelerationDegsPerSec == 0 {
		accelerationDegsPerSec = 100
	}
	if accelerationDegsPerSec < 10 || accelerationDegsPerSec > 500 {
		return fmt.Errorf("acceleration_degs_per_sec_per_sec must be between 10 and 500 degrees/second^2, got %.1f", accelerationDegsPerSec)
	}

	r.mu.Lock()
	defer r.mu.Unlock()

	needsReopen := r.cfg == nil ||
		r.cfg.Host != newConf.Host ||
		r.cfg.Port != newConf.Port ||
		r.cfg.Baudrate != newConf.Baudrate ||
		r.cfg.HTTPTimeout != newConf.HTTPTimeout ||
		r.cfg.SerialTimeout != newConf.SerialTimeout

	if needsReopen {
		if r.controller != nil {
			_ = r.controller.Close(ctx)
		}
		ctrl, err := NewRoArmController(&RoArmConfig{
			Host:          newConf.Host,
			Port:          newConf.Port,
			Baudrate:      newConf.Baudrate,
			HTTPTimeout:   newConf.HTTPTimeout,
			SerialTimeout: newConf.SerialTimeout,
			Logger:        r.logger,
		})
		if err != nil {
			return err
		}
		r.controller = ctrl
	}

	// Motion params always update. Preserve 30-unit safety floor on speed.
	defaultSpeed := speedToUnits(float64(speedDegsPerSec))
	if defaultSpeed < 30 {
		defaultSpeed = 30
	}
	defaultAcc := accelToUnits(float64(accelerationDegsPerSec))

	r.defaultSpeed = defaultSpeed
	r.defaultAcc = defaultAcc
	r.cfg = newConf

	r.logger.Infof("RoArm-M3 reconfigured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		speedDegsPerSec, defaultSpeed, accelerationDegsPerSec, defaultAcc)

	return nil
}
