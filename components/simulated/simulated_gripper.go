package simulated

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	rdkgripper "go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
)

// GripperModel is the model triplet for the hardware-free simulated RoArm-M3 gripper.
var GripperModel = resource.NewModel("hipsterbrown", "waveshare-roarm", "simulated-gripper")

func init() {
	resource.RegisterComponent(rdkgripper.API, GripperModel,
		resource.Registration[rdkgripper.Gripper, *SimulatedGripperConfig]{
			Constructor: newSimulatedGripper,
		},
	)
}

// SimulatedGripperConfig configures a simulated RoArm-M3 gripper. It needs no arm and no
// serial port: it interpolates the jaw angle in software and serves the same jaw mesh as
// the hipsterbrown:waveshare-roarm:gripper model, which makes it useful for testing
// configs and the 3D scene viewer without a physical robot.
type SimulatedGripperConfig struct {
	// SpeedDegsPerSec is how fast the jaw travels toward its target, in degrees per
	// second. Defaults to defaultSimSpeedDegsPerSec when unset.
	SpeedDegsPerSec float64 `json:"speed_degs_per_sec,omitempty"`

	// SimulateTime controls whether a background goroutine advances the jaw in real
	// time. Defaults to true. Tests set it false to drive the simulated clock
	// deterministically via updateForTime.
	SimulateTime *bool `json:"simulate_time,omitempty"`

	// CollisionGeometry selects the jaw's collision shape in the kinematic model:
	// "" or "box" for a bounding box, "mesh" for the jaw's bounding-polytope envelope.
	CollisionGeometry string `json:"collision_geometry,omitempty"`
}

// Validate ensures all parts of the config are valid. The simulated gripper has no
// dependencies.
func (cfg *SimulatedGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.SpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("%s: speed_degs_per_sec must not be negative, got %.1f", path, cfg.SpeedDegsPerSec)
	}
	if err := geometry.ValidateCollision(cfg.CollisionGeometry); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	return nil, nil, nil
}

// simulatedGripper is a hardware-free RoArm-M3 gripper. The jaw angle is kept in the same
// software-frame radians as the hardware model (geometry.GripperJointLimits) and
// interpolated toward its target over time, so the jaw animates in the 3D viewer.
type simulatedGripper struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	model  referenceframe.Model

	// speed is the jaw travel speed in radians per second.
	speed float64

	// lifetime management
	closed     atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
	workers    sync.WaitGroup

	// mu guards the fields below. jawRad and targetRad are software-frame radians
	// within geometry.GripperJointLimits.
	mu          sync.Mutex
	jawRad      float64
	targetRad   float64
	lastUpdated time.Time
}

func newSimulatedGripper(
	ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger,
) (rdkgripper.Gripper, error) {
	conf, err := resource.NativeConfig[*SimulatedGripperConfig](rawConf)
	if err != nil {
		return nil, err
	}

	model, err := geometry.GripperModel(conf.CollisionGeometry, rawConf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = defaultSimSpeedDegsPerSec
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	g := &simulatedGripper{
		name:       rawConf.ResourceName(),
		logger:     logger,
		model:      model,
		speed:      speedDegsPerSec * math.Pi / 180.0,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		// The jaw starts closed, like a powered-on arm holding its grip.
		jawRad:      geometry.GripperJointLimits[0],
		targetRad:   geometry.GripperJointLimits[0],
		lastUpdated: time.Now(),
	}

	// SimulateTime defaults to true so a deployed gripper advances on its own.
	if conf.SimulateTime == nil || *conf.SimulateTime {
		g.startTimeSimulation()
	}

	logger.Debugf("simulated RoArm-M3 gripper configured with speed: %.1f deg/s", speedDegsPerSec)
	return g, nil
}

// startTimeSimulation launches a background goroutine that advances the jaw against a
// realtime clock until the gripper is closed.
func (g *simulatedGripper) startTimeSimulation() {
	g.workers.Add(1)
	go func() {
		defer g.workers.Done()
		ticker := time.NewTicker(timeSimulationInterval)
		defer ticker.Stop()
		for {
			select {
			case <-g.cancelCtx.Done():
				return
			case <-ticker.C:
				g.updateForTime(time.Now())
			}
		}
	}()
}

// updateForTime advances the jaw toward its target at g.speed. It is called by the
// background goroutine when simulate_time is true, and directly by tests for a
// deterministic clock when it is false.
func (g *simulatedGripper) updateForTime(now time.Time) {
	g.mu.Lock()
	defer g.mu.Unlock()

	elapsed := now.Sub(g.lastUpdated).Seconds()
	g.lastUpdated = now

	diff := g.targetRad - g.jawRad
	step := g.speed * elapsed
	if math.Abs(diff) <= step {
		g.jawRad = g.targetRad
		return
	}
	g.jawRad += math.Copysign(step, diff)
}

// moveTo sets the jaw target and blocks until the jaw reaches it, the gripper is closed,
// or the context is canceled. Stop makes the target the current angle, which ends the
// wait where the jaw stands.
func (g *simulatedGripper) moveTo(ctx context.Context, rad float64) error {
	g.mu.Lock()
	g.targetRad = rad
	g.mu.Unlock()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-g.cancelCtx.Done():
			return g.cancelCtx.Err()
		default:
			g.mu.Lock()
			done := g.jawRad == g.targetRad
			g.mu.Unlock()
			if done {
				return nil
			}
			time.Sleep(time.Millisecond)
		}
	}
}

func (g *simulatedGripper) Name() resource.Name {
	return g.name
}

// Open moves the jaw to its open limit, interpolating over time.
func (g *simulatedGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	return g.moveTo(ctx, geometry.GripperJointLimits[1])
}

// Grab closes the jaw, interpolating over time. It always reports false: a simulated
// gripper never actually grasps an object.
func (g *simulatedGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, g.moveTo(ctx, geometry.GripperJointLimits[0])
}

// IsHoldingSomething always reports not-holding, for the same reason as Grab.
func (g *simulatedGripper) IsHoldingSomething(
	ctx context.Context, extra map[string]interface{},
) (rdkgripper.HoldingStatus, error) {
	return rdkgripper.HoldingStatus{}, nil
}

// Stop halts the jaw where it currently stands.
func (g *simulatedGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.targetRad = g.jawRad
	return nil
}

// IsMoving reports whether the jaw is mid-travel.
func (g *simulatedGripper) IsMoving(ctx context.Context) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.jawRad != g.targetRad, nil
}

// Geometries serves the jaw mesh posed at the current jaw angle, so the jaw animates in
// the 3D viewer.
func (g *simulatedGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	g.mu.Lock()
	jaw := g.jawRad
	g.mu.Unlock()
	return geometry.GripperMeshes(jaw)
}

// Kinematics returns the gripper's static model, whose leaf frame is the TCP between the
// jaw tips. See geometry.GripperModel for why the gripper needs one at all.
func (g *simulatedGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

func (g *simulatedGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *simulatedGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	for _, inputSet := range inputs {
		if len(inputSet) != 0 {
			return fmt.Errorf("the gripper model has no degrees of freedom; use the set_position DoCommand to move the jaw, got %d inputs", len(inputSet))
		}
	}
	return nil
}

func (g *simulatedGripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, nil
}

// DoCommand mirrors the hardware gripper's get_position and set_position commands, in
// degrees. set_position blocks until the jaw reaches the requested angle.
func (g *simulatedGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_position":
		g.mu.Lock()
		jaw := g.jawRad
		g.mu.Unlock()
		return map[string]interface{}{
			"position_degrees": jaw * 180.0 / math.Pi,
			"position_radians": jaw,
		}, nil

	case "set_position":
		degrees, ok := cmd["degrees"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_position command requires 'degrees' number parameter")
		}
		err := g.setPosition(ctx, degrees)
		return map[string]interface{}{"success": err == nil}, err

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// setPosition drives the jaw to an angle in degrees, rejecting angles outside joint 6's
// limits exactly as the hardware gripper does.
func (g *simulatedGripper) setPosition(ctx context.Context, degrees float64) error {
	radians := degrees * math.Pi / 180.0
	if radians < geometry.GripperJointLimits[0] || radians > geometry.GripperJointLimits[1] {
		return fmt.Errorf("gripper angle must be between %.1f and %.1f degrees, got %.1f",
			geometry.GripperJointLimits[0]*180/math.Pi, geometry.GripperJointLimits[1]*180/math.Pi, degrees)
	}
	return g.moveTo(ctx, radians)
}

func (g *simulatedGripper) Close(ctx context.Context) error {
	if g.closed.Swap(true) {
		return nil
	}
	g.cancelFunc()
	g.workers.Wait()
	return nil
}
