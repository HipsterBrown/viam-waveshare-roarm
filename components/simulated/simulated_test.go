package simulated

import (
	"context"
	"math"
	"strings"
	"testing"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"

	"waveshareroarm/internal/roarm"
)

// The simulated arm must satisfy the whole rdk arm interface, streaming included.
var _ rdkarm.Arm = (*simulatedArm)(nil)

// newTestSimArm constructs a simulated arm with the simulated clock disabled, so tests
// drive time deterministically via updateForTime. speedRadPerSec sets the joint speed in
// radians per second (1.0 makes interpolation arithmetic exact).
func newTestSimArm(t *testing.T, speedRadPerSec float64) *simulatedArm {
	t.Helper()
	simulateTime := false
	conf := resource.Config{
		Name:  "testSimArm",
		API:   rdkarm.API,
		Model: Model,
		ConvertedAttributes: &SimulatedArmConfig{
			SpeedDegsPerSec: speedRadPerSec * 180.0 / math.Pi,
			SimulateTime:    &simulateTime,
		},
	}
	// deps is nil: the joint-level simulation needs no motion service.
	a, err := newSimulatedArm(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := a.Close(context.Background()); err != nil {
			t.Fatal(err)
		}
	})
	return a.(*simulatedArm)
}

func waitForMoving(t *testing.T, a rdkarm.Arm) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		moving, err := a.IsMoving(context.Background())
		if err != nil {
			t.Fatal(err)
		}
		if moving {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("arm never started moving")
}

func assertInputs(t *testing.T, got []referenceframe.Input, want []float64, msg string) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("%s: expected %d joints, got %d", msg, len(want), len(got))
	}
	for i := range want {
		if math.Abs(float64(got[i])-want[i]) > 1e-9 {
			t.Fatalf("%s: expected %v, got %v", msg, want, got)
		}
	}
}

func TestSimulatedConfigValidate(t *testing.T) {
	deps, optional, err := (&SimulatedArmConfig{}).Validate("")
	if err != nil {
		t.Fatal(err)
	}
	if optional != nil {
		t.Fatalf("expected no optional deps, got %v", optional)
	}
	if len(deps) != 1 || deps[0] != "rdk:service:motion/builtin" {
		t.Fatalf("expected the builtin motion dependency, got %v", deps)
	}

	deps, _, err = (&SimulatedArmConfig{Motion: "myMotion"}).Validate("")
	if err != nil {
		t.Fatal(err)
	}
	if len(deps) != 1 || deps[0] != "rdk:service:motion/myMotion" {
		t.Fatalf("expected the myMotion dependency, got %v", deps)
	}

	if _, _, err := (&SimulatedArmConfig{SpeedDegsPerSec: -1}).Validate(""); err == nil {
		t.Fatal("expected a negative speed to be rejected")
	}
	if _, _, err := (&SimulatedArmConfig{CollisionGeometry: "hull"}).Validate(""); err == nil {
		t.Fatal("expected an unknown collision_geometry to be rejected")
	}
	if _, _, err := (&SimulatedArmConfig{CollisionGeometry: "mesh"}).Validate(""); err != nil {
		t.Fatalf("mesh collision geometry should be accepted: %v", err)
	}
}

func TestSimulatedKinematics(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)

	model, err := sim.Kinematics(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if len(model.DoF()) != 5 {
		t.Fatalf("expected 5 joints, got %d", len(model.DoF()))
	}

	inputs, err := sim.CurrentInputs(ctx)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0, 0, 0, 0, 0}, "fresh arm")
}

func TestSimulatedMoveToJointPositions(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0) // 1 radian/second

	// Joint 1 must travel 1.0 rad (the farthest), so the move takes 1 second. Joint 0
	// travels half as far, so it moves at half speed.
	target := []referenceframe.Input{0.5, -1.0, 0, 0, 0}

	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveToJointPositions(ctx, target, nil) }()
	waitForMoving(t, sim)

	// Time has not advanced yet; the arm has not moved.
	inputs, err := sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0, 0, 0, 0, 0}, "before the clock advances")

	// Advance the simulated clock half a second: the move should be half complete.
	base := time.Time{}
	sim.updateForTime(base.Add(500 * time.Millisecond))
	inputs, err = sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0.25, -0.5, 0, 0, 0}, "half way")

	moving, err := sim.IsMoving(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if !moving {
		t.Fatal("expected the arm to still be moving")
	}
	select {
	case <-moveErr:
		t.Fatal("MoveToJointPositions returned before the move completed")
	default:
	}

	// Advance to one second: the move should be complete and the call should return.
	sim.updateForTime(base.Add(time.Second))
	inputs, err = sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0.5, -1.0, 0, 0, 0}, "at the target")

	select {
	case err := <-moveErr:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after the move completed")
	}

	if moving, err = sim.IsMoving(ctx); err != nil {
		t.Fatal(err)
	} else if moving {
		t.Fatal("expected the arm to have stopped moving")
	}
}

func TestSimulatedMoveRejectsWrongJointCount(t *testing.T) {
	sim := newTestSimArm(t, 1.0)
	if err := sim.MoveToJointPositions(context.Background(), []referenceframe.Input{0, 0, 0}, nil); err == nil {
		t.Fatal("expected a wrong joint count to be rejected")
	}
}

func TestSimulatedStop(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)

	moveErr := make(chan error, 1)
	go func() {
		moveErr <- sim.MoveToJointPositions(ctx, []referenceframe.Input{0.5, -1.0, 0, 0, 0}, nil)
	}()
	waitForMoving(t, sim)

	if err := sim.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}

	select {
	case err := <-moveErr:
		if err == nil || !strings.Contains(err.Error(), "stopped before reaching target") {
			t.Fatalf("expected a stop error, got %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after Stop")
	}

	if moving, err := sim.IsMoving(ctx); err != nil {
		t.Fatal(err)
	} else if moving {
		t.Fatal("a stopped arm is not moving")
	}
}

func TestSimulatedEndPosition(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)

	pose, err := sim.EndPosition(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if pose == nil {
		t.Fatal("expected a pose")
	}

	sim.mu.Lock()
	sim.currInputs = []float64{1.0, 0, 0, 0, 0}
	sim.mu.Unlock()

	moved, err := sim.EndPosition(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if pose.Point().ApproxEqual(moved.Point()) {
		t.Fatalf("end position should change after a joint moves, still %v", moved.Point())
	}
}

func TestSimulatedGet3DModels(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)

	models, err := sim.Get3DModels(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(models) != 6 {
		t.Fatalf("expected 6 link meshes, got %d", len(models))
	}
	for _, part := range []string{"base_link", "link1", "link2", "link3", "link4", "link5"} {
		mesh, ok := models[part]
		if !ok {
			t.Fatalf("missing mesh for %q", part)
		}
		if mesh.ContentType != "model/gltf-binary" {
			t.Fatalf("%q: expected a GLB, got %q", part, mesh.ContentType)
		}
		if len(mesh.Mesh) == 0 {
			t.Fatalf("%q: empty mesh bytes", part)
		}
	}
}

func TestSimulatedGeometries(t *testing.T) {
	sim := newTestSimArm(t, 1.0)
	geoms, err := sim.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(geoms) == 0 {
		t.Fatal("expected the model's collision geometries")
	}
}

func TestSimulatedTimeSimulation(t *testing.T) {
	ctx := context.Background()
	// With simulate_time left at its default (true), the background goroutine advances
	// the arm on its own, so MoveToJointPositions completes without manual updateForTime.
	conf := resource.Config{
		Name:  "testSimArm",
		API:   rdkarm.API,
		Model: Model,
		ConvertedAttributes: &SimulatedArmConfig{
			SpeedDegsPerSec: 2000, // fast, so the move finishes quickly
		},
	}
	a, err := newSimulatedArm(ctx, nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := a.Close(ctx); err != nil {
			t.Fatal(err)
		}
	}()

	if err := a.MoveToJointPositions(ctx, []referenceframe.Input{0.3, -0.3, 0.2, 0, 0}, nil); err != nil {
		t.Fatal(err)
	}
	inputs, err := a.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0.3, -0.3, 0.2, 0, 0}, "after a real-time move")

	if moving, err := a.IsMoving(ctx); err != nil {
		t.Fatal(err)
	} else if moving {
		t.Fatal("the move is over")
	}
}

func TestSimulatedDoCommand(t *testing.T) {
	ctx := context.Background()
	conf := resource.Config{
		Name:                "testSimArm",
		API:                 rdkarm.API,
		Model:               Model,
		ConvertedAttributes: &SimulatedArmConfig{SpeedDegsPerSec: 2000},
	}
	a, err := newSimulatedArm(ctx, nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := a.Close(ctx); err != nil {
			t.Fatal(err)
		}
	}()

	out, err := a.DoCommand(ctx, map[string]interface{}{"command": "move_to_home"})
	if err != nil {
		t.Fatal(err)
	}
	if out["success"] != true {
		t.Fatalf("expected success, got %v", out)
	}
	inputs, err := a.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0, 0, math.Pi / 2, 0, 0}, "home")

	if _, err := a.DoCommand(ctx, map[string]interface{}{"command": "set_led"}); err == nil {
		t.Fatal("expected an unknown command to be rejected")
	}
}

// The stream re-targets the interpolator per point without waiting; the call returns only
// once the arm has converged on the LAST point.
func TestSimulatedStreamedFollowsPointsAndAcksPerBatch(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0) // 1 rad/s

	epoch := time.Date(2026, 9, 20, 12, 0, 0, 0, time.UTC)
	var deadlines []time.Time // written by the call goroutine; read only after it returns
	sim.clock = roarm.Clock{
		Now:        func() time.Time { return epoch },
		SleepUntil: func(ctx context.Context, d time.Time) error { deadlines = append(deadlines, d); return ctx.Err() },
	}

	in := make(chan []rdkarm.TrajectoryPoint)
	out := make(chan rdkarm.Response)
	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveThroughJointPositionsStreamed(ctx, in, out, nil) }()

	in <- []rdkarm.TrajectoryPoint{pt(0, 0.1), pt(10*time.Millisecond, 0.2)}
	<-out
	in <- []rdkarm.TrajectoryPoint{} // no ack
	in <- []rdkarm.TrajectoryPoint{pt(20*time.Millisecond, 0.5)}
	<-out
	close(in)

	// Nothing moves until the simulated clock is pumped, and the call is still waiting.
	inputs, err := sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0, 0, 0, 0, 0}, "before the clock is pumped")
	select {
	case <-moveErr:
		t.Fatal("returned before the arm arrived")
	default:
	}

	base := time.Time{}
	sim.updateForTime(base.Add(250 * time.Millisecond))
	inputs, err = sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0.25, 0, 0, 0, 0}, "heading for the last point")

	sim.updateForTime(base.Add(500 * time.Millisecond))
	select {
	case err := <-moveErr:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("did not return after converging on the last point")
	}
	inputs, err = sim.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	assertInputs(t, inputs, []float64{0.5, 0, 0, 0, 0}, "at the last point")

	want := []time.Time{epoch, epoch.Add(10 * time.Millisecond), epoch.Add(20 * time.Millisecond)}
	if len(deadlines) != len(want) {
		t.Fatalf("expected %v deadlines, got %v", want, deadlines)
	}
	for i := range want {
		if !deadlines[i].Equal(want[i]) {
			t.Fatalf("expected %v deadlines, got %v", want, deadlines)
		}
	}
}

func pt(at time.Duration, q float64) rdkarm.TrajectoryPoint {
	return rdkarm.TrajectoryPoint{Time: at, Positions: []referenceframe.Input{q, 0, 0, 0, 0}}
}

func TestSimulatedStreamedRejectsNonZeroFirstTime(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)

	in := make(chan []rdkarm.TrajectoryPoint)
	out := make(chan rdkarm.Response)
	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveThroughJointPositionsStreamed(ctx, in, out, nil) }()

	in <- []rdkarm.TrajectoryPoint{pt(10*time.Millisecond, 0.1)}
	select {
	case err := <-moveErr:
		if err == nil {
			t.Fatal("expected the first point to be rejected")
		}
	case <-time.After(time.Second):
		t.Fatal("did not reject the first point")
	}
	if moving, err := sim.IsMoving(ctx); err != nil {
		t.Fatal(err)
	} else if moving {
		t.Fatal("nothing was targeted")
	}
}

// Stop must end a stream between points, not be erased by the next re-target.
func TestSimulatedStreamedStopEndsTheStream(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	sim.clock = roarm.Clock{
		Now:        func() time.Time { return time.Time{} },
		SleepUntil: func(ctx context.Context, _ time.Time) error { return ctx.Err() },
	}

	in := make(chan []rdkarm.TrajectoryPoint)
	out := make(chan rdkarm.Response)
	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveThroughJointPositionsStreamed(ctx, in, out, nil) }()

	in <- []rdkarm.TrajectoryPoint{pt(0, 0.1)}
	<-out
	if err := sim.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	in <- []rdkarm.TrajectoryPoint{pt(10*time.Millisecond, 0.2)} // must not be applied

	select {
	case err := <-moveErr:
		if err == nil || !strings.Contains(err.Error(), "stopped") {
			t.Fatalf("expected a stop error, got %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("stream outlived Stop")
	}
	// The stopped target is the FIRST point, untouched by the second.
	sim.mu.Lock()
	target := append([]float64(nil), sim.operation.targetInputs...)
	sim.mu.Unlock()
	if math.Abs(target[0]-0.1) > 1e-9 {
		t.Fatalf("expected the first point to stay the target, got %v", target)
	}
}

func TestSimulatedMoveToPositionRequiresMotionService(t *testing.T) {
	s := &simulatedArm{name: rdkarm.Named("simarm"), logger: logging.NewTestLogger(t)}
	// Pin the guard's identity, not merely "an error": without asserting the message,
	// removing the guard is still caught, but only as a nil-pointer panic that takes the
	// whole test binary down.
	err := s.MoveToPosition(context.Background(), nil, nil)
	if err == nil || !strings.Contains(err.Error(), "requires a motion service") {
		t.Fatalf("expected the nil-motion guard to fire, got %v", err)
	}
}
