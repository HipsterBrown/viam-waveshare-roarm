// Package simulated implements hipsterbrown:waveshare-roarm:simulated, the
// hardware-free arm model. It uses no controller and no serial port,
// interpolating joint motion over time, and exists for development, testing,
// and 3D visualization without a physical robot.
package simulated

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	commonpb "go.viam.com/api/common/v1"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
	"waveshareroarm/internal/roarm"
)

// Model is the model triplet for the hardware-free simulated RoArm-M3 arm.
var Model = resource.NewModel("hipsterbrown", "waveshare-roarm", "simulated")

// homeInputs is the arm's home pose: extended, elbow at 90 degrees. It matches
// the hardware arm's.
var homeInputs = []referenceframe.Input{0, 0, math.Pi / 2, 0, 0}

// defaultSimSpeedDegsPerSec is the joint travel speed used when speed_degs_per_sec is unset.
const defaultSimSpeedDegsPerSec = 90.0

// timeSimulationInterval is how often the background goroutine advances the arm's position.
const timeSimulationInterval = 10 * time.Millisecond

func init() {
	resource.RegisterComponent(rdkarm.API, Model,
		resource.Registration[rdkarm.Arm, *SimulatedArmConfig]{
			Constructor: newSimulatedArm,
		},
	)
}

// SimulatedArmConfig configures a simulated RoArm-M3 arm. The simulated arm needs no
// hardware: it shares the kinematics and meshes with the hipsterbrown:waveshare-roarm:arm
// model and emulates joint motion in software, which makes it useful for testing configs,
// motion plans, and the 3D scene viewer without a physical robot.
type SimulatedArmConfig struct {
	// SpeedDegsPerSec is how fast each joint travels toward its target, in degrees per
	// second. Defaults to defaultSimSpeedDegsPerSec when unset.
	SpeedDegsPerSec float64 `json:"speed_degs_per_sec,omitempty"`

	// Motion is the name of the motion service used to plan MoveToPosition requests.
	// Defaults to "builtin".
	Motion string `json:"motion,omitempty"`

	// SimulateTime controls whether a background goroutine advances the arm's position in
	// real time. Defaults to true. Tests set it false to drive the simulated clock
	// deterministically via updateForTime.
	SimulateTime *bool `json:"simulate_time,omitempty"`

	// CollisionGeometry selects the collision shapes of the kinematic model:
	// "" or "box" for one bounding box per link, "mesh" for per-slab bounding-polytope envelopes.
	CollisionGeometry string `json:"collision_geometry,omitempty"`
}

// Validate ensures all parts of the config are valid and declares the motion service
// dependency consumed by MoveToPosition.
func (cfg *SimulatedArmConfig) Validate(path string) ([]string, []string, error) {
	if cfg.SpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("%s: speed_degs_per_sec must not be negative, got %.1f", path, cfg.SpeedDegsPerSec)
	}
	if err := geometry.ValidateCollision(cfg.CollisionGeometry); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	return []string{motion.Named(cfg.motionName()).String()}, nil, nil
}

// motionName returns the configured motion service name, defaulting to "builtin".
func (cfg *SimulatedArmConfig) motionName() string {
	if cfg.Motion != "" {
		return cfg.Motion
	}
	return "builtin"
}

// simOperation tracks the state of an in-flight MoveToJointPositions request.
//
// Logical states/invariants:
//  1. Default constructed -- no operation in flight.
//  2. Operation started -> targetInputs != nil, done == false, stopped == false.
//  3. Operation successful -> done == true.
//  4. Operation stopped -> stopped == true.
type simOperation struct {
	// targetInputs is the goal joint configuration in radians.
	targetInputs []float64
	// speed is this move's joint travel speed in radians per second, resolved by
	// startMove from the requested MoveOptions (or the arm's configured speed, when
	// unrequested).
	speed   float64
	done    bool
	stopped bool
}

func (op simOperation) isMoving() bool {
	return op.targetInputs != nil && !op.done && !op.stopped
}

// simulatedArm is a hardware-free RoArm-M3. It shares the kinematics with the
// hipsterbrown:waveshare-roarm:arm model and interpolates joint motion over time,
// mirroring the behavior of rdk's builtin "fake" arm.
type simulatedArm struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	model  referenceframe.Model

	// motion plans MoveToPosition requests. It is nil when the arm was constructed
	// without dependencies (e.g. in unit tests); MoveToPosition is then unavailable but
	// joint-level simulation still works.
	motion motion.Service

	// speed is the joint travel speed in radians per second.
	speed float64

	// clock schedules streamed trajectory points. Zero value is the real clock; tests only.
	clock roarm.Clock

	// lifetime management
	closed     atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
	workers    sync.WaitGroup

	// mu guards the fields below.
	mu          sync.Mutex
	currInputs  []float64 // current joint positions in radians, length == model DoF
	lastUpdated time.Time
	operation   simOperation
}

func newSimulatedArm(
	ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger,
) (rdkarm.Arm, error) {
	conf, err := resource.NativeConfig[*SimulatedArmConfig](rawConf)
	if err != nil {
		return nil, err
	}

	model, err := geometry.ArmModel(conf.CollisionGeometry, rawConf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = defaultSimSpeedDegsPerSec
	}

	// Resolve the motion service for MoveToPosition. deps is nil in unit tests; the
	// joint-level simulation does not need it, so resolution is skipped in that case.
	var ms motion.Service
	if deps != nil {
		ms, err = motion.FromProvider(deps, conf.motionName())
		if err != nil {
			return nil, err
		}
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	sim := &simulatedArm{
		name:       rawConf.ResourceName(),
		logger:     logger,
		model:      model,
		motion:     ms,
		speed:      speedDegsPerSec * math.Pi / 180.0,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		currInputs: make([]float64, len(model.DoF())),
	}

	// SimulateTime defaults to true so a deployed arm advances on its own.
	if conf.SimulateTime == nil || *conf.SimulateTime {
		// Avoid ever letting the zero value of lastUpdated be visible, lest the first
		// movement be unpredictable.
		sim.lastUpdated = time.Now()
		sim.startTimeSimulation()
	}

	logger.Debugf("simulated RoArm-M3 configured with speed: %.1f deg/s", speedDegsPerSec)
	return sim, nil
}

// startTimeSimulation launches a background goroutine that advances the arm's position
// against a realtime clock until the arm is closed.
func (s *simulatedArm) startTimeSimulation() {
	s.workers.Add(1)
	go func() {
		defer s.workers.Done()
		ticker := time.NewTicker(timeSimulationInterval)
		defer ticker.Stop()
		for {
			select {
			case <-s.cancelCtx.Done():
				return
			case <-ticker.C:
				s.updateForTime(time.Now())
			}
		}
	}()
}

// updateForTime advances the simulated joint positions to the given wall-clock time. It
// is called by the background goroutine when simulate_time is true, and directly by
// tests for a deterministic clock when it is false.
func (s *simulatedArm) updateForTime(now time.Time) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.operation.isMoving() {
		s.lastUpdated = now
		return
	}

	timeSinceLastUpdate := now.Sub(s.lastUpdated)
	s.lastUpdated = now

	// Find the maximum joint travel distance. Because all joints move at the same top
	// speed, this maps to how long the whole movement takes.
	var maxDist float64
	for jointIdx, currJointInp := range s.currInputs {
		maxDist = math.Max(maxDist, math.Abs(s.operation.targetInputs[jointIdx]-currJointInp))
	}

	const epsilon = 1e-9
	if maxDist < epsilon {
		s.operation.done = true
		return
	}

	// Scale each joint's speed so that every joint finishes its travel at the same time.
	// This matches rdk's motion-planning interpolation.
	modifiedSpeeds := make([]float64, len(s.currInputs))
	for jointIdx, currJointInp := range s.currInputs {
		diffRads := math.Abs(s.operation.targetInputs[jointIdx] - currJointInp)
		modifiedSpeeds[jointIdx] = (diffRads / maxDist) * s.operation.speed
	}

	// anyJointStillMoving stays false only when every joint has reached its target.
	anyJointStillMoving := false
	for jointIdx, currJointInp := range s.currInputs {
		// Signed remaining distance to the target.
		diffRads := s.operation.targetInputs[jointIdx] - currJointInp

		// How far this joint could travel since the last update, capped at diffRads.
		toTravelRads := timeSinceLastUpdate.Seconds() * modifiedSpeeds[jointIdx]
		if toTravelRads > math.Abs(diffRads)-epsilon {
			// We can travel at least as far as we need to; snap to the target.
			s.currInputs[jointIdx] = s.operation.targetInputs[jointIdx]
		} else {
			if diffRads < 0 {
				// toTravelRads is always positive; flip it to travel the other way.
				toTravelRads = -toTravelRads
			}
			s.currInputs[jointIdx] = currJointInp + toTravelRads
			anyJointStillMoving = true
		}
	}

	if !anyJointStillMoving {
		s.operation.done = true
	}
}

func (s *simulatedArm) Name() resource.Name {
	return s.name
}

// EndPosition returns the pose of the end effector at the current joint positions.
func (s *simulatedArm) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	pose, err := s.model.Transform(inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}
	return pose, nil
}

// MoveToPosition moves the arm's end effector to the target pose using the motion
// service, matching the hardware model: the goal is position-only, so the planner is
// free to choose the tool orientation that reaches it.
func (s *simulatedArm) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if s.motion == nil {
		return errors.New("MoveToPosition requires a motion service, which was not available at construction")
	}

	planExtra := map[string]any{"goal_metric_type": "position_only"}
	for k, v := range extra {
		planExtra[k] = v
	}

	_, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.name.Name,
		Destination:   referenceframe.NewPoseInFrame(fmt.Sprintf("%v_origin", s.name.Name), pose),
		Extra:         planExtra,
	})
	return err
}

// MoveToJointPositions starts a move to the given joint configuration and blocks until it
// completes, the arm is stopped, or the context is canceled.
func (s *simulatedArm) MoveToJointPositions(
	ctx context.Context, positions []referenceframe.Input, extra map[string]interface{},
) error {
	if err := s.startMove(ctx, positions, false, 0); err != nil {
		return err
	}
	return s.awaitOperation(ctx)
}

// startMove validates positions and replaces the in-flight operation without waiting. With
// keepStop, an operation already stopped is not replaced: the caller reports the stop.
// speedRadPerSec is this move's joint travel speed; 0 resolves to the arm's configured
// speed, so updateForTime can read the operation's speed unconditionally.
func (s *simulatedArm) startMove(
	ctx context.Context, positions []referenceframe.Input, keepStop bool, speedRadPerSec float64,
) error {
	if len(positions) != len(s.model.DoF()) {
		return fmt.Errorf("expected %d joint positions for the RoArm-M3 arm, got %d",
			len(s.model.DoF()), len(positions))
	}
	if err := rdkarm.CheckDesiredJointPositions(ctx, s, positions); err != nil {
		return err
	}

	target := make([]float64, len(positions))
	copy(target, positions)

	s.mu.Lock()
	defer s.mu.Unlock()
	if keepStop && s.operation.stopped {
		return errors.New("stopped before reaching target")
	}
	if speedRadPerSec == 0 {
		speedRadPerSec = s.speed
	}
	s.operation = simOperation{targetInputs: target, speed: speedRadPerSec}
	return nil
}

// awaitOperation blocks until the in-flight operation completes, is stopped, or ctx ends.
func (s *simulatedArm) awaitOperation(ctx context.Context) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-s.cancelCtx.Done():
			return s.cancelCtx.Err()
		default:
			s.mu.Lock()
			done, stopped := s.operation.done, s.operation.stopped
			s.mu.Unlock()

			if done {
				return nil
			}
			if stopped {
				return errors.New("stopped before reaching target")
			}
			time.Sleep(time.Millisecond)
		}
	}
}

// MoveThroughJointPositions moves the arm through each joint configuration in order,
// honoring a requested speed. Acceleration is resolved (for its per-joint validation) but
// otherwise ignored: the simulator interpolates at constant speed with no ramp.
func (s *simulatedArm) MoveThroughJointPositions(
	ctx context.Context, positions [][]referenceframe.Input, options *rdkarm.MoveOptions, _ map[string]interface{},
) error {
	speedDegs, _, err := roarm.ResolveMoveProfile(options, len(s.model.DoF()),
		s.speed*180/math.Pi, roarm.DefaultAccelDegsPerSecSq, s.logger)
	if err != nil {
		return err
	}
	speedRadPerSec := speedDegs * math.Pi / 180

	for _, goal := range positions {
		if err := s.startMove(ctx, goal, false, speedRadPerSec); err != nil {
			return err
		}
		if err := s.awaitOperation(ctx); err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

// JointPositions returns the current joint positions in radians.
func (s *simulatedArm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	ret := make([]referenceframe.Input, len(s.currInputs))
	copy(ret, s.currInputs)
	return ret, nil
}

func (s *simulatedArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *simulatedArm) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *simulatedArm) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

// Stop ends any in-flight movement. The arm holds its current position.
func (s *simulatedArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Only set stopped while moving, otherwise the distinction between "reached the
	// goal" and "was stopped" is lost.
	if s.operation.isMoving() {
		s.operation.stopped = true
	}
	return nil
}

func (s *simulatedArm) IsMoving(ctx context.Context) (bool, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	return s.operation.isMoving(), nil
}

// Geometries returns the arm's geometries at the current joint positions.
func (s *simulatedArm) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

// Get3DModels returns the RoArm-M3 link meshes for the 3D scene viewer.
func (s *simulatedArm) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return geometry.ArmMeshes(), nil
}

func (s *simulatedArm) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, nil
}

func (s *simulatedArm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "move_to_home":
		err := s.MoveToJointPositions(ctx, homeInputs, nil)
		return map[string]interface{}{"success": err == nil}, err
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd)
	}
}

func (s *simulatedArm) Close(ctx context.Context) error {
	if s.closed.Swap(true) {
		return nil
	}
	s.cancelFunc()
	s.workers.Wait()
	return nil
}
