package waveshareroarm

import (
	"context"
	_ "embed"
	"encoding/json"
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
	Timeout time.Duration `json:"timeout,omitempty"`

	// Motion configuration
	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *RoArmM3Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" && cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify either host for HTTP or port for serial communication")
	}
	if cfg.Host != "" && cfg.Port != "" {
		return nil, nil, fmt.Errorf("cannot specify both host and port, choose either HTTP or serial communication")
	}
	return nil, nil, nil
}

type roarmM3 struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *RoArmM3Config
	opMgr      *operation.SingleOperationManager
	controller *SafeRoArmController

	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	model       referenceframe.Model
	jointLimits [][2]float64

	// Motion configuration
	defaultSpeed int
	defaultAcc   int

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

	// Convert degrees/sec to internal speed units (approximate conversion based on RoArm SDK)
	// RoArm speed range is 1-4096, where ~50 deg/sec ≈ 500 units
	defaultSpeed := int(speedDegsPerSec * 10)
	if defaultSpeed < 30 {
		defaultSpeed = 30
	}
	if defaultSpeed > 4096 {
		defaultSpeed = 4096
	}

	// Convert degrees/sec^2 to internal acceleration units
	// RoArm acceleration range is 1-254, where ~100 deg/sec^2 ≈ 50 units
	defaultAcc := int(accelerationDegsPerSec * 0.5)
	if defaultAcc < 1 {
		defaultAcc = 1
	}
	if defaultAcc > 254 {
		defaultAcc = 254
	}

	// Create controller configuration
	controllerConfig := &RoArmConfig{
		Host:     conf.Host,
		Port:     conf.Port,
		Baudrate: conf.Baudrate,
		Timeout:  conf.Timeout,
		Logger:   logger,
	}

	controller, err := GetSharedController(controllerConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared RoArm controller: %w", err)
	}

	model, err := makeRoArmModelFrame()
	if err != nil {
		ReleaseSharedController() // Clean up on error
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
	panic("not implemented")
}

func (r *roarmM3) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	inputs, err := r.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := referenceframe.ComputeOOBPosition(r.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

func (r *roarmM3) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if err := motion.MoveArm(ctx, r.logger, r, pose); err != nil {
		return err
	}
	return nil
}

func (r *roarmM3) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	r.moveLock.Lock()
	defer r.moveLock.Unlock()

	r.isMoving.Store(true)
	defer r.isMoving.Store(false)

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
	currentGripperPos := 0.0 // Default neutral position
	currentFullPositions, err := r.controller.GetJointRadians()
	if err != nil {
		r.logger.Warnf("Failed to get current gripper position, using neutral: %v", err)
	} else if len(currentFullPositions) >= 6 {
		currentGripperPos = currentFullPositions[5] // Joint 6 (gripper)
	}

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
				// Convert from degrees/sec to internal units
				speed = int(speedVal * 10)
				if speed < 30 {
					speed = 30
				}
				if speed > 4096 {
					speed = 4096
				}
			}
		}
		if accOverride, ok := extra["acceleration"]; ok {
			if accVal, ok := accOverride.(float64); ok {
				// Convert from degrees/sec^2 to internal units
				acc = int(accVal * 0.5)
				if acc < 1 {
					acc = 1
				}
				if acc > 254 {
					acc = 254
				}
			}
		}
	}

	// Send command to controller with all 6 joints (including preserved gripper)
	if err := r.controller.SetJointRadians(fullPositions, speed, acc); err != nil {
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
	speedRadPerSec := float64(speed) / 10.0 * math.Pi / 180.0 // Convert to rad/sec
	moveTimeSeconds := maxMovement / speedRadPerSec
	if moveTimeSeconds < 0.1 {
		moveTimeSeconds = 0.1 // Minimum move time
	}
	if moveTimeSeconds > 10.0 {
		moveTimeSeconds = 10.0 // Maximum move time for safety
	}

	// Wait for movement to complete
	time.Sleep(time.Duration(moveTimeSeconds * float64(time.Second)))

	return nil
}

func (r *roarmM3) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
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
	r.mu.RLock()
	defer r.mu.RUnlock()

	// Get all joint positions from controller (includes gripper)
	allRadians, err := r.controller.GetJointRadians()
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
	r.isMoving.Store(false)
	// Note: The RoArm controller doesn't have a direct stop command
	// Movement will complete, but we mark as not moving
	return nil
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
	// Handle custom commands specific to RoArm
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := r.controller.SetTorque(enable)
		return map[string]interface{}{"success": err == nil}, err

	case "set_led":
		brightness, ok := cmd["brightness"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_led command requires 'brightness' number parameter")
		}
		err := r.controller.SetLED(int(brightness))
		return map[string]interface{}{"success": err == nil}, err

	case "move_to_home":
		err := r.controller.MoveToHome()
		return map[string]interface{}{"success": err == nil}, err

	case "get_feedback":
		feedback, err := r.controller.GetFeedback()
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
				"wrist":    feedback.T_,
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

	case "controller_status":
		refCount, hasController, configSummary := GetControllerStatus()
		return map[string]interface{}{
			"ref_count":      refCount,
			"has_controller": hasController,
			"config":         configSummary,
		}, nil

	default:
		// Check for speed and acceleration setting (similar to xArm module)
		result := make(map[string]interface{})
		changed := false

		if speedVal, ok := cmd["set_speed"]; ok {
			if speed, ok := speedVal.(float64); ok {
				if speed < 3 || speed > 180 {
					return nil, fmt.Errorf("speed must be between 3 and 180 degrees/second, got %.1f", speed)
				}
				r.mu.Lock()
				r.defaultSpeed = int(speed * 10)
				if r.defaultSpeed < 30 {
					r.defaultSpeed = 30
				}
				if r.defaultSpeed > 4096 {
					r.defaultSpeed = 4096
				}
				r.mu.Unlock()
				result["speed_set"] = speed
				changed = true
			} else {
				return nil, fmt.Errorf("set_speed requires a number value")
			}
		}

		if accVal, ok := cmd["set_acceleration"]; ok {
			if acc, ok := accVal.(float64); ok {
				if acc < 10 || acc > 500 {
					return nil, fmt.Errorf("acceleration must be between 10 and 500 degrees/second^2, got %.1f", acc)
				}
				r.mu.Lock()
				r.defaultAcc = int(acc * 0.5)
				if r.defaultAcc < 1 {
					r.defaultAcc = 1
				}
				if r.defaultAcc > 254 {
					r.defaultAcc = 254
				}
				r.mu.Unlock()
				result["acceleration_set"] = acc
				changed = true
			} else {
				return nil, fmt.Errorf("set_acceleration requires a number value")
			}
		}

		if getParams, ok := cmd["get_motion_params"]; ok && getParams.(bool) {
			r.mu.RLock()
			speedDegsPerSec := float64(r.defaultSpeed) / 10.0
			accDegsPerSec := float64(r.defaultAcc) / 0.5
			r.mu.RUnlock()

			result["current_speed_degs_per_sec"] = speedDegsPerSec
			result["current_acceleration_degs_per_sec_per_sec"] = accDegsPerSec
			changed = true
		}

		if changed {
			return result, nil
		}

		return nil, fmt.Errorf("unknown command: %v", cmd)
	}
}

func (r *roarmM3) IsMoving(ctx context.Context) (bool, error) {
	return r.isMoving.Load(), nil
}

func (r *roarmM3) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
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

func (r *roarmM3) Close(context.Context) error {
	r.cancelFunc()
	ReleaseSharedController()
	return nil
}
