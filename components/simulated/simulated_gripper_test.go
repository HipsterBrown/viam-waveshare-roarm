package simulated

import (
	"context"
	"math"
	"strings"
	"testing"
	"time"

	rdkgripper "go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"waveshareroarm/internal/geometry"
)

// The simulated gripper must satisfy the whole rdk gripper interface.
var _ rdkgripper.Gripper = (*simulatedGripper)(nil)

// newTestSimGripper constructs a simulated gripper with the simulated clock disabled, so
// tests drive time deterministically via updateForTime.
func newTestSimGripper(t *testing.T) *simulatedGripper {
	t.Helper()
	simulateTime := false
	conf := resource.Config{
		Name:  "testSimGripper",
		API:   rdkgripper.API,
		Model: GripperModel,
		ConvertedAttributes: &SimulatedGripperConfig{
			SimulateTime: &simulateTime,
		},
	}
	g, err := newSimulatedGripper(context.Background(), nil, conf, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := g.Close(context.Background()); err != nil {
			t.Fatal(err)
		}
	})
	return g.(*simulatedGripper)
}

// drive runs a blocking gripper move while stepping the simulated clock until it returns.
func drive(t *testing.T, g *simulatedGripper, move func() error) {
	t.Helper()
	done := make(chan error, 1)
	go func() { done <- move() }()

	start := time.Now()
	for tick := 1; tick < 10000; tick++ {
		select {
		case err := <-done:
			if err != nil {
				t.Fatal(err)
			}
			return
		default:
		}
		g.updateForTime(start.Add(time.Duration(tick) * timeSimulationInterval))
		time.Sleep(time.Millisecond)
	}
	t.Fatal("gripper move did not converge")
}

func assertJaw(t *testing.T, g *simulatedGripper, want float64, msg string) {
	t.Helper()
	g.mu.Lock()
	got := g.jawRad
	g.mu.Unlock()
	if math.Abs(got-want) > 1e-9 {
		t.Fatalf("%s: expected jaw at %v rad, got %v", msg, want, got)
	}
}

func TestSimulatedGripperConfigValidate(t *testing.T) {
	deps, optional, err := (&SimulatedGripperConfig{}).Validate("")
	if err != nil {
		t.Fatal(err)
	}
	if len(deps) != 0 || len(optional) != 0 {
		t.Fatalf("expected no dependencies, got %v %v", deps, optional)
	}

	if _, _, err := (&SimulatedGripperConfig{SpeedDegsPerSec: -1}).Validate("gripper"); err == nil {
		t.Fatal("expected an error for a negative speed")
	}
	if _, _, err := (&SimulatedGripperConfig{CollisionGeometry: "bogus"}).Validate("gripper"); err == nil {
		t.Fatal("expected an error for an unknown collision_geometry")
	}
	if _, _, err := (&SimulatedGripperConfig{CollisionGeometry: geometry.CollisionMesh}).Validate("gripper"); err != nil {
		t.Fatal(err)
	}
}

// TestSimulatedGripperOpenGrab drives Open and Grab to their joint limits.
func TestSimulatedGripperOpenGrab(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t)

	assertJaw(t, g, geometry.GripperJointLimits[0], "a fresh gripper starts closed")

	drive(t, g, func() error { return g.Open(ctx, nil) })
	assertJaw(t, g, geometry.GripperJointLimits[1], "after Open")

	var grabbed bool
	drive(t, g, func() error {
		var err error
		grabbed, err = g.Grab(ctx, nil)
		return err
	})
	assertJaw(t, g, geometry.GripperJointLimits[0], "after Grab")
	if grabbed {
		t.Fatal("a simulated gripper never grasps anything; Grab should report false")
	}

	moving, err := g.IsMoving(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if moving {
		t.Fatal("the gripper should be still once Grab returns")
	}

	holding, err := g.IsHoldingSomething(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if holding.IsHoldingSomething {
		t.Fatal("a simulated gripper never holds anything")
	}
}

// TestSimulatedGripperGeometries checks the jaw mesh follows the simulated jaw angle.
func TestSimulatedGripperGeometries(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t)

	closed, err := g.Geometries(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(closed) != 1 {
		t.Fatalf("expected one jaw mesh, got %d", len(closed))
	}
	if _, ok := closed[0].(*spatialmath.Mesh); !ok {
		t.Fatalf("expected a mesh geometry, got %T", closed[0])
	}

	drive(t, g, func() error { return g.Open(ctx, nil) })

	open, err := g.Geometries(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	if spatialmath.PoseAlmostEqual(closed[0].Pose(), open[0].Pose()) {
		t.Fatalf("the jaw mesh should be posed differently open vs closed, both at %v", open[0].Pose())
	}
}

// TestSimulatedGripperStop halts the jaw mid-travel.
func TestSimulatedGripperStop(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t)

	openErr := make(chan error, 1)
	go func() { openErr <- g.Open(ctx, nil) }()

	// Wait for Open to register its target before stepping the clock.
	deadline := time.Now().Add(2 * time.Second)
	for {
		moving, err := g.IsMoving(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if moving {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("Open never started moving")
		}
		time.Sleep(time.Millisecond)
	}

	// One short step, then stop: the jaw should hold well short of open.
	start := time.Now()
	g.updateForTime(start.Add(timeSimulationInterval))
	if err := g.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-openErr:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("Open did not return after Stop")
	}

	g.mu.Lock()
	jaw := g.jawRad
	g.mu.Unlock()
	if jaw >= geometry.GripperJointLimits[1] {
		t.Fatalf("Stop should have halted the jaw short of open, got %v", jaw)
	}
	moving, err := g.IsMoving(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if moving {
		t.Fatal("the gripper should not report moving after Stop")
	}
}

func TestSimulatedGripperDoCommand(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t)

	out, err := g.DoCommand(ctx, map[string]interface{}{"command": "get_position"})
	if err != nil {
		t.Fatal(err)
	}
	wantDeg := geometry.GripperJointLimits[0] * 180 / math.Pi
	if deg, ok := out["position_degrees"].(float64); !ok || math.Abs(deg-wantDeg) > 1e-9 {
		t.Fatalf("expected position_degrees %v, got %v", wantDeg, out["position_degrees"])
	}
	if rad, ok := out["position_radians"].(float64); !ok || math.Abs(rad-geometry.GripperJointLimits[0]) > 1e-9 {
		t.Fatalf("expected position_radians %v, got %v", geometry.GripperJointLimits[0], out["position_radians"])
	}

	// Out of range in both directions, with the same limits as the hardware gripper.
	for _, degrees := range []float64{
		geometry.GripperJointLimits[0]*180/math.Pi - 1,
		geometry.GripperJointLimits[1]*180/math.Pi + 1,
	} {
		_, err := g.DoCommand(ctx, map[string]interface{}{"command": "set_position", "degrees": degrees})
		if err == nil {
			t.Fatalf("expected an out-of-range error for %.1f degrees", degrees)
		}
	}
	if _, err := g.DoCommand(ctx, map[string]interface{}{"command": "set_position"}); err == nil {
		t.Fatal("expected an error when 'degrees' is missing")
	}
	if _, err := g.DoCommand(ctx, map[string]interface{}{"command": "bogus"}); err == nil {
		t.Fatal("expected an error for an unknown command")
	}

	// An in-range set_position drives the jaw to the requested angle.
	const targetDeg = 30.0
	drive(t, g, func() error {
		out, err := g.DoCommand(ctx, map[string]interface{}{"command": "set_position", "degrees": targetDeg})
		if err != nil {
			return err
		}
		if ok, _ := out["success"].(bool); !ok {
			t.Errorf("expected success true, got %v", out)
		}
		return nil
	})
	assertJaw(t, g, targetDeg*math.Pi/180, "after set_position")
}

func TestSimulatedGripperKinematicsAndInputs(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t)

	model, err := g.Kinematics(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if model == nil {
		t.Fatal("expected a kinematic model")
	}

	inputs, err := g.CurrentInputs(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if len(inputs) != 0 {
		t.Fatalf("expected no inputs, got %v", inputs)
	}
	if err := g.GoToInputs(ctx, []referenceframe.Input{}); err != nil {
		t.Fatal(err)
	}
	err = g.GoToInputs(ctx, []referenceframe.Input{0})
	if err == nil || !strings.Contains(err.Error(), "degrees of freedom") {
		t.Fatalf("expected a no-DoF error for a non-empty input set, got %v", err)
	}
}
