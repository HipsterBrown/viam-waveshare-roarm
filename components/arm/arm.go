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
	"waveshareroarm/internal/planning"
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

	// OrientationToleranceDeg is the approach-axis cone half-angle in degrees.
	// Zero or unset means the default (30); an explicit 0 does NOT mean
	// "demand an exact match", because a zero-leeway goal cloud is one no IK
	// solution realistically lands inside. For a near-exact orientation pass a
	// small non-zero value, or a raw pose_cloud in extra.
	OrientationToleranceDeg float64 `json:"orientation_tolerance_deg,omitempty"`
	// PositionToleranceMM is the per-axis positional leeway of the goal cloud.
	// Zero or unset means the default (1.0).
	PositionToleranceMM float64 `json:"position_tolerance_mm,omitempty"`

	// CollisionGeometry selects the collision shapes of the kinematic model:
	// "" or "box" for one bounding box per link, "mesh" for per-slab bounding-polytope envelopes.
	CollisionGeometry string `json:"collision_geometry,omitempty"`
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

	if err := geometry.ValidateCollision(cfg.CollisionGeometry); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}

	if err := planning.ValidateGoalCloudTolerances(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
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

	// goalCloud is the resolved approach-axis cone MoveToPosition plans against.
	goalCloud planning.GoalCloudConfig

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

	model, err := geometry.ArmModel(conf.CollisionGeometry, rawConf.ResourceName().ShortName())
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
		goalCloud:    planning.ResolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
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

	pose, err := r.snapshotModel().Transform(inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

// MoveToPosition always waits for the arm to arrive, and a waitAtEnd/wait in
// extra does not change that. This call delegates to the motion service, whose
// generic execute path drives the arm through GoToInputs -- which the RDK's
// InputEnabled interface defines with no extra map, so the flag is dropped
// before it returns here. That is the right outcome anyway: a planned path
// executed without settling between waypoints would collapse to a straight
// line to the final one, skipping the obstacle avoidance the plan existed for.
//
// The motion service's TELEOP executor is the exception -- it type-asserts the
// resource to arm.Arm and calls MoveThroughJointPositions directly with
// {"waitAtEnd": false, "interpolate": false}, which this module honours. See
// MoveThroughJointPositions and roarm.InterpolateArg.
func (r *roarmM3) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}

	r.mu.Lock()
	goalCfg := r.goalCloud
	r.mu.Unlock()

	dest, planExtra, path, err := planning.BuildMoveDestination(
		fmt.Sprintf("%v_origin", r.Name().Name), pose, goalCfg, extra)
	if err != nil {
		return err
	}

	_, err = r.motion.Move(
		ctx,
		motion.MoveReq{
			ComponentName: r.Name().Name,
			Destination:   dest,
			Extra:         planExtra,
		},
	)
	return planning.WrapMoveErr(err, path, goalCfg)
}

// MoveToJointPositions moves the arm to positions and, by default, blocks
// until the firmware's feedback shows it there.
//
// `extra: {"waitAtEnd": false}` (or `{"wait": false}`) returns as soon as the
// goal is on the wire. The return then means "the arm was told", NOT "the arm
// arrived" -- ask IsMoving or JointPositions for that. It is for a teleop loop
// whose command interval is shorter than a move takes.
//
// A second call SUPERSEDES the first rather than queueing behind it: opMgr.New
// cancels the in-flight move's context, and the new goal write replaces the
// firmware's goal outright. Nothing is left half-applied -- a joint command is
// a single write of all six targets, and the firmware interpolates from
// wherever the arm currently is -- so the arm is always tracking exactly one
// goal, the most recent. That is what a teleop loop wants; it does mean a
// caller cannot assume an earlier goal was ever reached.
func (r *roarmM3) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	r.mu.Lock()
	speed, acc := r.defaultSpeed, r.defaultAcc
	r.mu.Unlock()
	return r.moveToJointPositionsAt(ctx, positions, speed, acc, roarm.WaitArg(extra))
}

// moveToJointPositionsAt is MoveToJointPositions with the speed and
// acceleration (firmware units) supplied by the caller instead of snapshotted
// from the configured defaults, so MoveThroughJointPositions can route a
// resolved MoveOptions profile all the way to the write and the settle.
func (r *roarmM3) moveToJointPositionsAt(ctx context.Context, positions []referenceframe.Input, speed, acc int, wait bool) error {
	if r.closed.Load() {
		return errClosed
	}
	ctx, done := r.opMgr.New(ctx)
	defer done()
	r.opInFlight.Store(true)
	defer r.opInFlight.Store(false)

	// Snapshot jointLimits under the mutex so a concurrent Reconfigure can't
	// race with us reading it here.
	r.mu.Lock()
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

	return r.moveAndSettle(ctx, ctrl, current, target, speed, acc, wait)
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

// moveAndSettle writes a full 6-joint target and, when wait is true, blocks
// until joints 1-5 settle. With wait false it returns as soon as the goal is
// on the wire and performs no settle at all, so none of the settle's
// diagnostics (stopped short, never moved) are evaluated.
// Callers own the opInFlight flag (the streamed path calls this from
// inside a longer in-flight window, so it must not clear the flag itself).
func (r *roarmM3) moveAndSettle(ctx context.Context, ctrl roarm.Handle, current, target []float64, speed, acc int, wait bool) error {
	if err := ctrl.SetJointRadians(ctx, target, speed, acc); err != nil {
		return fmt.Errorf("failed to move arm: %w", err)
	}
	if !wait {
		return nil
	}
	req := roarm.SettleRequest{
		Target:        target,
		Start:         current,
		Mask:          roarm.ArmMask,
		SpeedUnits:    speed,
		AccUnits:      acc,
		RequireMotion: true,
	}
	if _, err := ctrl.WaitUntilSettled(ctx, req); err != nil {
		return fmt.Errorf("arm did not settle: %w", err)
	}
	return nil
}

func (r *roarmM3) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *rdkarm.MoveOptions, extra map[string]interface{}) error {
	if r.closed.Load() {
		return errClosed
	}
	r.mu.Lock()
	defSpeed, defAcc, joints := roarm.SpeedFromUnits(r.defaultSpeed), roarm.AccelFromUnits(r.defaultAcc), len(r.jointLimits)
	r.mu.Unlock()
	speedDegs, accDegs, err := roarm.ResolveMoveProfile(options, joints, defSpeed, defAcc, r.logger)
	if err != nil {
		return err
	}
	speed, acc := roarm.SpeedToUnits(speedDegs), roarm.AccelToUnits(accDegs)

	wait := roarm.WaitArg(extra)
	// interpolate=false says the waypoints are a route to the last one, not a
	// path to trace, so collapse them to the endpoint and write once. This is
	// what the builtin motion service's teleop executor sends alongside
	// waitAtEnd=false, and the two belong together: without a settle between
	// them the intermediate writes are superseded within a millisecond of bus
	// time and never reach the arm as motion, so writing them would be pure
	// bus traffic pretending to be a path. See roarm.InterpolateArg.
	if !roarm.InterpolateArg(extra) && len(positions) > 1 {
		positions = positions[len(positions)-1:]
	}
	for i, jointPositions := range positions {
		// Only the FINAL waypoint's settle is optional. An intermediate
		// waypoint that is not settled is superseded before the arm gets
		// anywhere near it, so under interpolate=true (a path the caller does
		// want traced) the settle is the only thing making the waypoint real.
		if err := r.moveToJointPositionsAt(ctx, jointPositions, speed, acc, wait || i+1 < len(positions)); err != nil {
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

// snapshotModel returns r.model under r.mu; Reconfigure swaps it when
// collision_geometry changes.
func (r *roarmM3) snapshotModel() referenceframe.Model {
	r.mu.Lock()
	m := r.model
	r.mu.Unlock()
	return m
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
	return r.snapshotModel(), nil
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

	case roarm.CmdCommsHealth:
		ctrl := r.snapshotController()
		out := ctrl.Health().Map()
		if reset, ok := cmd["reset"].(bool); ok && reset {
			ctrl.ResetHealth()
			out["reset"] = true
		}
		return out, nil

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
		wait := roarm.WaitArg(cmd)
		requireMotion := true
		if v, ok := cmd[roarm.KeyRequireMotion].(bool); ok {
			requireMotion = v
		}
		ctrl := r.snapshotController()
		// Read before the write, but only when there is a settle to feed: the
		// settle needs a measured start pose to detect a jaw that never moved,
		// and the real travel is what makes its derived deadline mean
		// anything. Reading unconditionally would cost a frame the
		// fire-and-forget caller never asked for, and would turn wait=false
		// into an error on a transport that cannot read positions at all.
		var before []float64
		if wait {
			var err error
			before, err = readAllJointRadians(ctx, ctrl)
			if err != nil {
				return nil, fmt.Errorf("%s: read position before the move: %w", roarm.CmdSetGripperRad, err)
			}
		}
		if err := ctrl.SetJointRadian(ctx, 6, rad, speed, acc); err != nil {
			return nil, err
		}
		if wait {
			target := append([]float64(nil), before...)
			target[5] = rad
			req := roarm.SettleRequest{
				Target:        target,
				Start:         before,
				Mask:          roarm.GripperMask,
				SpeedUnits:    speed,
				AccUnits:      acc,
				RequireMotion: requireMotion,
			}
			if _, err := ctrl.WaitUntilSettled(ctx, req); err != nil {
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
	gif, err := r.snapshotModel().Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (r *roarmM3) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return geometry.ArmMeshes(), nil
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

	if r.cfg == nil || r.cfg.CollisionGeometry != newConf.CollisionGeometry {
		model, err := geometry.ArmModel(newConf.CollisionGeometry, r.name.ShortName())
		if err != nil {
			return err
		}
		r.model = model
		r.jointLimits = jointLimitsFromModel(model)
	}

	// Motion params always update.
	defaultSpeed, defaultAcc := newConf.motionDefaults()

	r.defaultSpeed = defaultSpeed
	r.defaultAcc = defaultAcc
	r.goalCloud = planning.ResolveGoalCloudConfig(newConf.OrientationToleranceDeg, newConf.PositionToleranceMM, r.logger)
	r.cfg = newConf

	r.logger.Infof("RoArm-M3 reconfigured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		roarm.SpeedFromUnits(defaultSpeed), defaultSpeed, roarm.AccelFromUnits(defaultAcc), defaultAcc)

	return nil
}
