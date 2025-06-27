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
	controller *RoArmController

	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	jointPos    []float64
	model       referenceframe.Model
	jointLimits [][2]float64

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

	// Create controller configuration
	controllerConfig := &RoArmConfig{
		Host:     conf.Host,
		Port:     conf.Port,
		Baudrate: conf.Baudrate,
		Timeout:  conf.Timeout,
		Logger:   logger,
	}

	controller, err := NewRoArmController(controllerConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create RoArm controller: %w", err)
	}

	model, err := makeRoArmModelFrame()
	if err != nil {
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	arm := &roarmM3{
		name:        rawConf.ResourceName(),
		cfg:         conf,
		opMgr:       operation.NewSingleOperationManager(),
		logger:      logger,
		controller:  controller,
		jointPos:    make([]float64, 5), // Only 5 joints for the arm
		model:       model,
		jointLimits: RoArmM3JointLimits[:5], // Only first 5 joints
		cancelCtx:   cancelCtx,
		cancelFunc:  cancelFunc,
	}

	// Initialize arm to home position
	if err := arm.controller.MoveToHome(); err != nil {
		logger.Warnf("Failed to move to home position: %v", err)
	}

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

	// Calculate movement time based on maximum joint movement for arm joints only
	currentArmPositions := r.jointPos
	if len(currentArmPositions) == 0 {
		currentArmPositions = make([]float64, 5)
	}

	maxMovement := 0.0
	for i, target := range clampedPositions {
		if i < len(currentArmPositions) {
			movement := math.Abs(target - currentArmPositions[i])
			if movement > maxMovement {
				maxMovement = movement
			}
		}
	}

	// Calculate move time based on radians per second
	// Speed: 20-30 rad/sec (conservative for safety)
	moveTimeSeconds := maxMovement / 20.0 // 20 rad/sec
	if moveTimeSeconds < 0.1 {
		moveTimeSeconds = 0.1 // Minimum move time
	}
	if moveTimeSeconds > 5.0 {
		moveTimeSeconds = 5.0 // Maximum move time
	}

	// Convert to appropriate speed and acceleration values
	speed := int(100 + (moveTimeSeconds * 200)) // Scale speed based on move time
	acc := 50                                   // Fixed acceleration

	// Send command to controller with all 6 joints (including preserved gripper)
	if err := r.controller.SetJointRadians(fullPositions, speed, acc); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}

	// Update cached joint positions (arm joints only)
	r.mu.Lock()
	copy(r.jointPos, clampedPositions)
	r.mu.Unlock()

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
	r.logger.Info("Getting joint positions")
	r.mu.RLock()
	defer r.mu.RUnlock()

	// Get all joint positions from controller (includes gripper)
	allRadians, err := r.controller.GetJointRadians()
	r.logger.Info("Got all radians")
	if err != nil {
		r.logger.Warnf("failed to read joint positions, using cached values: %v", err)
		return nil, err
		// allRadians = make([]float64, 6)
		// if len(r.jointPos) == 5 {
		// 	copy(allRadians, r.jointPos)
		// }
	}

	// Only return the first 5 joints (arm joints, excluding gripper)
	armRadians := allRadians[:5]
	if len(allRadians) < 5 {
		// Fallback to cached values if we don't have enough data
		armRadians = r.jointPos
		if len(armRadians) < 5 {
			armRadians = make([]float64, 5) // Default to zeros
		}
	}

	// r.logger.Info("Updating cached positions")
	// // Update cached positions (arm joints only)
	// r.mu.RUnlock()
	// r.mu.Lock()
	// if len(r.jointPos) == 5 {
	// 	copy(r.jointPos, armRadians)
	// }
	// r.mu.Unlock()
	// r.mu.RLock()
	// r.logger.Info("Updated cached positions")

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

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
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
	return r.controller.Close()
}
