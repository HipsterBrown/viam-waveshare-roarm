package arm

import (
	"context"
	stdlib_errors "errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"

	commonpb "go.viam.com/api/common/v1"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/roarm"
)

// homeInputs is the arm's home pose: extended, elbow at 90 degrees.
var homeInputs = []referenceframe.Input{0, 0, math.Pi / 2, 0, 0}

var (
	Model = resource.NewModel("hipsterbrown", "waveshare-roarm", "arm")

	errClosed = stdlib_errors.New("arm closed")
)

func init() {
	resource.RegisterComponent(rdkarm.API, Model,
		resource.Registration[rdkarm.Arm, *RoArmM3Config]{
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
	HTTPTimeout   roarm.Duration `json:"http_timeout,omitempty"`
	SerialTimeout roarm.Duration `json:"serial_timeout,omitempty"`

	// Motion configuration
	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`

	Motion string `json:"motion,omitempty"`
}

var validBaudrates = map[int]bool{
	0:    true, // zero means "use default" (115200)
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

	if s := cfg.SpeedDegsPerSec; s != 0 && (s < roarm.MinSpeedDegsPerSec || s > roarm.MaxSpeedDegsPerSec) {
		return nil, nil, fmt.Errorf("%s: speed_degs_per_sec must be between %.0f and %.0f, got %.1f", path, roarm.MinSpeedDegsPerSec, roarm.MaxSpeedDegsPerSec, s)
	}
	if a := cfg.AccelerationDegsPerSec; a != 0 && (a < roarm.MinAccelDegsPerSecSq || a > roarm.MaxAccelDegsPerSecSq) {
		return nil, nil, fmt.Errorf("%s: acceleration_degs_per_sec_per_sec must be between %.0f and %.0f, got %.1f", path, roarm.MinAccelDegsPerSecSq, roarm.MaxAccelDegsPerSecSq, a)
	}

	return []string{motion.Named(cfg.motionName()).String()}, nil, nil
}

// motionName returns the configured motion service name, defaulting to "builtin".
func (cfg *RoArmM3Config) motionName() string {
	if cfg.Motion != "" {
		return cfg.Motion
	}
	return "builtin"
}

// motionDefaults applies the zero-means-default rule and returns the
// configured speed and acceleration in firmware units. Ranges are enforced
// in Validate, so this never fails.
func (cfg *RoArmM3Config) motionDefaults() (speedUnits, accUnits int) {
	speed := float64(cfg.SpeedDegsPerSec)
	if speed == 0 {
		speed = roarm.DefaultSpeedDegsPerSec
	}
	acc := float64(cfg.AccelerationDegsPerSec)
	if acc == 0 {
		acc = roarm.DefaultAccelDegsPerSecSq
	}
	return roarm.SpeedToUnits(speed), roarm.AccelToUnits(acc)
}

type roarmM3 struct {
	name       resource.Name
	logger     logging.Logger
	cfg        *RoArmM3Config
	opMgr      *operation.SingleOperationManager
	controller roarm.Handle

	mu          sync.Mutex
	model       referenceframe.Model
	jointLimits [][2]float64

	// Motion configuration
	defaultSpeed int
	defaultAcc   int

	closed atomic.Bool
	// opInFlight is true between the start and end of a commanded move, so
	// IsMoving answers true even before the servos report motion.
	opInFlight atomic.Bool

	// clock schedules streamed trajectories; the zero value is the real clock.
	clock roarm.Clock

	motion motion.Service
}

// jointLimitsFromModel reads the five arm joint limits (radians) from the
// kinematic model, which is the single source of truth: the rdk arm client
// validates remote calls against these same limits before they reach us.
func jointLimitsFromModel(m referenceframe.Model) [][2]float64 {
	dof := m.DoF()
	limits := make([][2]float64, len(dof))
	for i, l := range dof {
		limits[i] = [2]float64{l.Min, l.Max}
	}
	return limits
}

func newRoArmM3(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (rdkarm.Arm, error) {
	conf, err := resource.NativeConfig[*RoArmM3Config](rawConf)
	if err != nil {
		return nil, err
	}

	defaultSpeed, defaultAcc := conf.motionDefaults()

	// Create controller configuration
	controllerConfig := &roarm.Config{
		Host:          conf.Host,
		Port:          conf.Port,
		Baudrate:      conf.Baudrate,
		HTTPTimeout:   conf.HTTPTimeout,
		SerialTimeout: conf.SerialTimeout,
		Logger:        logger,
	}

	controller, err := roarm.NewController(controllerConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create RoArm controller: %w", err)
	}

	model, err := geometry.ArmModel("roarm_m3")
	if err != nil {
		_ = controller.Close(ctx) // Clean up on error
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	ms, err := motion.FromProvider(deps, conf.motionName())
	if err != nil {
		return nil, err
	}

	arm := &roarmM3{
		name:         rawConf.ResourceName(),
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
		logger:       logger,
		controller:   controller,
		model:        model,
		jointLimits:  jointLimitsFromModel(model),
		defaultSpeed: defaultSpeed,
		defaultAcc:   defaultAcc,
		motion:       ms,
	}

	logger.Infof("RoArm-M3 configured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		roarm.SpeedFromUnits(defaultSpeed), defaultSpeed, roarm.AccelFromUnits(defaultAcc), defaultAcc)

	return arm, nil
}

func (r *roarmM3) Name() resource.Name {
	return r.name
}

func (r *roarmM3) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, nil
}

func (r *roarmM3) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (rdkarm.Arm, error) {
	return nil, stdlib_errors.ErrUnsupported
}

func (r *roarmM3) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	if r.closed.Load() {
		return nil, errClosed
	}

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

	planExtra := map[string]any{"goal_metric_type": "position_only"}
	for k, v := range extra {
		planExtra[k] = v
	}

	_, err := r.motion.Move(
		ctx,
		motion.MoveReq{
			ComponentName: r.Name().Name,
			Destination:   referenceframe.NewPoseInFrame(fmt.Sprintf("%v_origin", r.Name().Name), pose),
			Extra:         planExtra,
		},
	)
	return err
}

func (r *roarmM3) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	ctx, done := r.opMgr.New(ctx)
	defer done()
	r.opInFlight.Store(true)
	defer r.opInFlight.Store(false)

	// Snapshot motion params under the mutex so concurrent Reconfigure /
	// DoCommand writers can't race with us reading them here.
	r.mu.Lock()
	speed := r.defaultSpeed
	acc := r.defaultAcc
	jointLimits := r.jointLimits
	r.mu.Unlock()

	if len(positions) != len(jointLimits) {
		return fmt.Errorf("expected %d joint positions for arm, got %d", len(jointLimits), len(positions))
	}

	clamped, hits := clampToLimits(positions, jointLimits)
	if len(hits) > 0 {
		r.logger.Warnf("clamped to joint limits; the executed path will deviate from the requested one: %s", strings.Join(hits, "; "))
	}

	ctrl := r.snapshotController()
	current, err := readAllJointRadians(ctx, ctrl)
	if err != nil {
		return fmt.Errorf("MoveToJointPositions: read current positions: %w", err)
	}
	target := make([]float64, 6)
	copy(target, clamped)
	target[5] = current[5] // preserve the gripper

	return r.moveAndSettle(ctx, ctrl, current, target, speed, acc)
}

// clampToLimits clamps positions into limits and reports each clamp as a
// human-readable string so callers can log once per move.
func clampToLimits(positions []referenceframe.Input, limits [][2]float64) ([]float64, []string) {
	clamped := make([]float64, len(positions))
	var hits []string
	for i, pos := range positions {
		lo, hi := limits[i][0], limits[i][1]
		clamped[i] = math.Max(lo, math.Min(hi, float64(pos)))
		if clamped[i] != float64(pos) {
			hits = append(hits, fmt.Sprintf("joint %d %.1f° -> [%.1f°, %.1f°]",
				i+1, pos*180/math.Pi, lo*180/math.Pi, hi*180/math.Pi))
		}
	}
	return clamped, hits
}

// moveAndSettle writes a full 6-joint target and blocks until joints 1-5
// settle. Callers own the opInFlight flag (the streamed path calls this from
// inside a longer in-flight window, so it must not clear the flag itself).
func (r *roarmM3) moveAndSettle(ctx context.Context, ctrl roarm.Handle, current, target []float64, speed, acc int) error {
	if err := ctrl.SetJointRadians(ctx, target, speed, acc); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}
	timeout := roarm.SettleTimeoutFor(roarm.MaxTravel(current, target, roarm.ArmMask), speed)
	if _, err := ctrl.WaitUntilSettled(ctx, target, roarm.ArmMask, timeout); err != nil {
		return fmt.Errorf("arm did not settle: %w", err)
	}
	return nil
}

func (r *roarmM3) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *rdkarm.MoveOptions, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	if options != nil {
		r.logger.Debug("MoveOptions are not yet honored by this module (sub-project 3); using configured speed and acceleration")
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

	// r.controller is swapped only in Reconfigure, which takes r.mu.Lock().
	// Briefly lock to snapshot, then release before the blocking serial I/O
	// so concurrent callers (e.g. EndPosition) don't deadlock on re-entry.
	allRadians, err := readAllJointRadians(ctx, r.snapshotController())
	if err != nil {
		return nil, fmt.Errorf("failed to read joint positions: %w", err)
	}

	positions := make([]referenceframe.Input, 5)
	copy(positions, allRadians[:5])
	return positions, nil
}

// snapshotController returns r.controller under r.mu. r.controller is swapped
// only in Reconfigure, which takes r.mu.Lock(); snapshotting and releasing
// before the blocking serial I/O prevents deadlocks when other methods
// (e.g. EndPosition) re-enter through CurrentInputs.
func (r *roarmM3) snapshotController() roarm.Handle {
	r.mu.Lock()
	ctrl := r.controller
	r.mu.Unlock()
	return ctrl
}

// readAllJointRadians reads all 6 joints from ctrl and enforces the
// 6-element invariant for callers that index into the slice.
func readAllJointRadians(ctx context.Context, ctrl roarm.Handle) ([]float64, error) {
	radians, err := ctrl.GetJointRadians(ctx)
	if err != nil {
		return nil, err
	}
	if len(radians) < 6 {
		return nil, fmt.Errorf("short feedback (got %d joints)", len(radians))
	}
	return radians, nil
}

func (r *roarmM3) Stop(ctx context.Context, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	r.opMgr.CancelRunning(ctx)

	// Snapshot defaultAcc under the mutex so Reconfigure / DoCommand writers
	// can't race with us reading it.
	ctrl := r.snapshotController()
	r.mu.Lock()
	acc := r.defaultAcc
	r.mu.Unlock()

	current, err := readAllJointRadians(ctx, ctrl)
	if err != nil {
		return fmt.Errorf("stop: read current positions: %w", err)
	}
	stopSpeed := roarm.SpeedToUnits(roarm.StopSpeedDegsPerSec) // gentle soft stop
	return ctrl.SetJointRadians(ctx, current, stopSpeed, acc)
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
		err := r.snapshotController().SetTorque(ctx, enable)
		return map[string]interface{}{"success": err == nil}, err

	case "set_led":
		brightness, ok := cmd["brightness"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_led command requires 'brightness' number parameter")
		}
		err := r.snapshotController().SetLED(ctx, int(brightness))
		return map[string]interface{}{"success": err == nil}, err

	case "move_to_home":
		err := r.MoveToJointPositions(ctx, homeInputs, nil)
		return map[string]interface{}{"success": err == nil}, err

	case "get_feedback":
		feedback, err := r.snapshotController().GetFeedback(ctx)
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
				"gripper":  roarm.GripperSoftwareToWire(feedback.G),
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
		if speed < roarm.MinSpeedDegsPerSec || speed > roarm.MaxSpeedDegsPerSec {
			return nil, fmt.Errorf("speed out of range: %.1f", speed)
		}
		r.mu.Lock()
		r.defaultSpeed = roarm.SpeedToUnits(speed)
		r.mu.Unlock()
		return map[string]interface{}{"speed_set": speed}, nil

	case "set_acceleration":
		acc, ok := cmd["value"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_acceleration requires 'value' number")
		}
		if acc < roarm.MinAccelDegsPerSecSq || acc > roarm.MaxAccelDegsPerSecSq {
			return nil, fmt.Errorf("accel out of range: %.1f", acc)
		}
		r.mu.Lock()
		r.defaultAcc = roarm.AccelToUnits(acc)
		r.mu.Unlock()
		return map[string]interface{}{"acceleration_set": acc}, nil

	case "get_motion_params":
		r.mu.Lock()
		defer r.mu.Unlock()
		return map[string]interface{}{
			"current_speed_degs_per_sec":                roarm.SpeedFromUnits(r.defaultSpeed),
			"current_acceleration_degs_per_sec_per_sec": roarm.AccelFromUnits(r.defaultAcc),
		}, nil

	// Gripper ↔ arm DoCommand bridge. See internal/roarm/bridge.go.
	case roarm.CmdGetGripperRad:
		radians, err := readAllJointRadians(ctx, r.snapshotController())
		if err != nil {
			return nil, fmt.Errorf("%s: %w", roarm.CmdGetGripperRad, err)
		}
		return map[string]interface{}{roarm.KeyRad: radians[5]}, nil

	case roarm.CmdSetGripperRad:
		rad, ok := cmd[roarm.KeyRad].(float64)
		if !ok {
			return nil, fmt.Errorf("%s requires %q number", roarm.CmdSetGripperRad, roarm.KeyRad)
		}
		if rad < geometry.GripperJointLimits[0] || rad > geometry.GripperJointLimits[1] {
			return nil, fmt.Errorf("%s: %.3f rad is outside the gripper range [%.2f, %.2f]", roarm.CmdSetGripperRad, rad, geometry.GripperJointLimits[0], geometry.GripperJointLimits[1])
		}
		speed := roarm.SpeedToUnits(roarm.DefaultGripperSpeedDegsPerSec)
		acc := roarm.AccelToUnits(roarm.DefaultGripperAccDegsPerSecSq)
		if v, ok := cmd[roarm.KeySpeed].(float64); ok {
			speed = roarm.SpeedToUnits(v)
		}
		if v, ok := cmd[roarm.KeyAcc].(float64); ok {
			acc = roarm.AccelToUnits(v)
		}
		wait := true
		if w, ok := cmd[roarm.KeyWait].(bool); ok {
			wait = w
		}
		ctrl := r.snapshotController()
		if err := ctrl.SetJointRadian(ctx, 6, rad, speed, acc); err != nil {
			return nil, err
		}
		if wait {
			target := make([]float64, 6)
			target[5] = rad
			fullTravel := geometry.GripperJointLimits[1] - geometry.GripperJointLimits[0]
			if _, err := ctrl.WaitUntilSettled(ctx, target, roarm.GripperMask, roarm.SettleTimeoutFor(fullTravel, speed)); err != nil {
				return nil, fmt.Errorf("%s: gripper did not settle: %w", roarm.CmdSetGripperRad, err)
			}
		}
		return map[string]interface{}{"success": true}, nil

	case roarm.CmdStopGripper:
		// Soft hold: read the current software-frame gripper position and
		// re-send it as the target at a gentle speed. Matches arm-level Stop.
		ctrl := r.snapshotController()
		radians, err := readAllJointRadians(ctx, ctrl)
		if err != nil {
			return nil, fmt.Errorf("%s: read position: %w", roarm.CmdStopGripper, err)
		}
		if err := ctrl.SetJointRadian(ctx, 6, radians[5], roarm.SpeedToUnits(roarm.StopSpeedDegsPerSec), roarm.AccelToUnits(roarm.DefaultGripperAccDegsPerSecSq)); err != nil {
			return nil, err
		}
		return map[string]interface{}{"success": true}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (r *roarmM3) IsMoving(ctx context.Context) (bool, error) {
	if r.closed.Load() {
		return false, errClosed
	}
	if r.opInFlight.Load() {
		return true, nil
	}
	return r.snapshotController().IsMoving(ctx)
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

func (r *roarmM3) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return nil, nil
}

func (r *roarmM3) Close(ctx context.Context) error {
	if !r.closed.CompareAndSwap(false, true) {
		return nil
	}
	r.opMgr.CancelRunning(ctx)
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.controller != nil {
		return r.controller.Close(ctx)
	}
	return nil
}

// Reconfigure updates the arm's configuration. Connectivity-affecting fields
// (Host/Port/Baudrate/timeouts) trigger a controller reopen. Motion-only
// changes (speed/acceleration) update in place without tearing down the link.
func (r *roarmM3) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*RoArmM3Config](conf)
	if err != nil {
		return err
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
		ctrl, err := roarm.NewController(&roarm.Config{
			Host:          newConf.Host,
			Port:          newConf.Port,
			Baudrate:      newConf.Baudrate,
			HTTPTimeout:   newConf.HTTPTimeout,
			SerialTimeout: newConf.SerialTimeout,
			Logger:        r.logger,
		})
		if err != nil {
			// Keep the old controller and config: the arm stays usable and a
			// later Reconfigure with a working config still sees the diff.
			return fmt.Errorf("reconfigure: open new connection: %w", err)
		}
		if r.controller != nil {
			_ = r.controller.Close(ctx)
		}
		r.controller = ctrl
	}

	// Motion params always update.
	defaultSpeed, defaultAcc := newConf.motionDefaults()

	r.defaultSpeed = defaultSpeed
	r.defaultAcc = defaultAcc
	r.cfg = newConf

	r.logger.Infof("RoArm-M3 reconfigured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		roarm.SpeedFromUnits(defaultSpeed), defaultSpeed, roarm.AccelFromUnits(defaultAcc), defaultAcc)

	return nil
}
