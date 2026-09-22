package planning

import (
	"math"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

func TestResolveGoalCloudConfigDefaults(t *testing.T) {
	// Zero and unset are indistinguishable for a float64 with omitempty; both default.
	got := ResolveGoalCloudConfig(0, 0, nil)
	if got.OrientationToleranceDeg != defaultOrientationToleranceDeg {
		t.Errorf("OrientationToleranceDeg = %v, want %v", got.OrientationToleranceDeg, defaultOrientationToleranceDeg)
	}
	if got.PositionToleranceMM != defaultPositionToleranceMM {
		t.Errorf("PositionToleranceMM = %v, want %v", got.PositionToleranceMM, defaultPositionToleranceMM)
	}
}

func TestResolveGoalCloudConfigKeepsExplicitValues(t *testing.T) {
	got := ResolveGoalCloudConfig(12.5, 0.25, nil)
	if got.OrientationToleranceDeg != 12.5 {
		t.Errorf("OrientationToleranceDeg = %v, want 12.5", got.OrientationToleranceDeg)
	}
	if got.PositionToleranceMM != 0.25 {
		t.Errorf("PositionToleranceMM = %v, want 0.25", got.PositionToleranceMM)
	}
}

// The warnings are the production path -- both tests above pass a nil logger and so
// exercise only the early return. ResolveGoalCloudConfig is the SINGLE owner of these
// thresholds; nothing downstream re-checks them, so nothing downstream would catch a
// regression either.
func TestResolveGoalCloudConfigWarnsAtSaturationBoundary(t *testing.T) {
	// The boundary is acos(-0.999) = 177.4374412669 -- exactly the band a literal
	// ">= 177.44" check would miss, which is why poseCloudSaturationCos is a cosine.
	logger, logs := logging.NewObservedTestLogger(t)
	ResolveGoalCloudConfig(177.438, 0, logger)
	if n := logs.FilterMessageSnippet("saturates").Len(); n != 1 {
		t.Errorf("just inside the boundary must warn: got %d warnings, want 1", n)
	}

	logger, logs = logging.NewObservedTestLogger(t)
	ResolveGoalCloudConfig(177.40, 0, logger)
	if n := logs.FilterMessageSnippet("saturates").Len(); n != 0 {
		t.Errorf("just outside must not warn: got %d warnings, want 0", n)
	}
}

func TestResolveGoalCloudConfigWarnsOnLargePositionTolerance(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	ResolveGoalCloudConfig(0, 25, logger)
	if n := logs.FilterMessageSnippet("is large").Len(); n != 1 {
		t.Errorf("got %d warnings, want 1", n)
	}

	logger, logs = logging.NewObservedTestLogger(t)
	ResolveGoalCloudConfig(0, 10, logger)
	if n := logs.FilterMessageSnippet("is large").Len(); n != 0 {
		t.Errorf("at the threshold must not warn: got %d warnings, want 0", n)
	}
}

func TestResolveGoalCloudConfigDefaultsDoNotWarn(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	ResolveGoalCloudConfig(0, 0, logger)
	if n := logs.Len(); n != 0 {
		t.Errorf("the defaults must be quiet: got %d log entries, want 0", n)
	}
}

// testGoal is a tool pointing straight down, a typical RoArm-M3 grasp pose.
func testGoal() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 300, Y: 0, Z: 200},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0},
	)
}

// tiltedBy returns testGoal() tilted by deg about the goal's local (ax, ay, 0) axis. The
// rotation composes onto the goal, so it applies in the goal's own frame -- and since the
// goal points straight down, that local frame is not the world frame.
func tiltedBy(ax, ay, deg float64) spatialmath.Pose {
	return spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
		&spatialmath.R4AA{RX: ax, RY: ay, RZ: 0, Theta: utils.DegToRad(deg)}))
}

// coneCloud builds the cloud under test for a cone half-angle. Its position tolerance is
// deliberately 2.5 rather than 1: that keeps it distinct from the unconstrained OX/OY
// sentinel of 1, so the position -> X/Y/Z mapping stays legible and a transposition of the
// two would be caught.
func coneCloud(tolDeg float64) *referenceframe.PoseCloud {
	return coneToPoseCloud(GoalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: 2.5})
}

// TestConeToPoseCloudFields pins the complete field set: X, Y, Z, OX, OY, OZ, Theta is the
// WHOLE struct in rdk v1.6.0. Do not look for a ReferenceFrame field -- v0.123.0 had one,
// v1.0.0 removed it, and the struct's free-floating doc comment about reference frames
// misleadingly survives.
func TestConeToPoseCloudFields(t *testing.T) {
	c := coneCloud(30)
	// The position tolerance maps to all three positional axes...
	if c.X != 2.5 {
		t.Errorf("X = %v, want 2.5", c.X)
	}
	if c.Y != 2.5 {
		t.Errorf("Y = %v, want 2.5", c.Y)
	}
	if c.Z != 2.5 {
		t.Errorf("Z = %v, want 2.5", c.Z)
	}
	// ...while OX/OY are deliberately unconstrained: OZ alone defines the cone.
	if c.OX != 1.0 {
		t.Errorf("OX = %v, want 1.0 (unconstrained)", c.OX)
	}
	if c.OY != 1.0 {
		t.Errorf("OY = %v, want 1.0 (unconstrained)", c.OY)
	}
	if want := 1 - math.Cos(utils.DegToRad(30)); math.Abs(c.OZ-want) > 1e-12 {
		t.Errorf("OZ = %v, want %v", c.OZ, want)
	}
	// MUST be 180: Theta encodes tilt AZIMUTH, not roll. See the spec.
	if c.Theta != 180.0 {
		t.Errorf("Theta = %v, want 180", c.Theta)
	}
}

func TestConeBoundsTiltIsotropically(t *testing.T) {
	c := coneCloud(30)
	for _, azDeg := range []float64{0, 45, 90, 135, 180, 270} {
		ax, ay := math.Cos(utils.DegToRad(azDeg)), math.Sin(utils.DegToRad(azDeg))
		if !c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 29)) {
			t.Errorf("29deg tilt at azimuth %.0f should be accepted", azDeg)
		}
		if c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 31)) {
			t.Errorf("31deg tilt at azimuth %.0f should be rejected", azDeg)
		}
	}
}

func TestConeBoundsAbove90Degrees(t *testing.T) {
	// The cone does NOT go degenerate above 90, despite OZ = 1-cos(a) exceeding 1.
	if !coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 89)) {
		t.Error("cone 90 should accept an 89deg tilt")
	}
	if coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 91)) {
		t.Error("cone 90 should reject a 91deg tilt")
	}
	if !coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 119)) {
		t.Error("cone 120 should accept a 119deg tilt")
	}
	if coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 121)) {
		t.Error("cone 120 should reject a 121deg tilt")
	}
}

// TestConeSaturationBoundary is the empirical anchor for saturation: it brackets the
// boundary with literal MEASURED degrees, owing nothing to poseCloudSaturationCos. Its
// sibling TestSaturationConstantMatchesConeMapping is the drift guard, deriving the same
// boundary FROM that constant. Both are needed: this one would still pass if the constant
// were wrong, and the sibling would still pass if both the constant and the mapping drifted
// together.
func TestConeSaturationBoundary(t *testing.T) {
	// Saturation begins at acos(-0.999) = 177.4374412669, NOT 179.
	if coneCloud(177.40).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)) {
		t.Error("177.40 must still reject a 180deg tilt")
	}
	if !coneCloud(177.44).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)) {
		t.Error("177.44 must be saturated")
	}
}

// TestSaturationConstantMatchesConeMapping ties poseCloudSaturationCos to the cone
// mapping. The constant is only correct while coneToPoseCloud maps the cone to
// OZ = 1-cos(a); if that formula ever changes, ResolveGoalCloudConfig would keep warning at
// the wrong angle with nothing to catch it. The tests above use literal degrees and would
// not notice. This one derives the angle FROM the constant, so the two cannot drift apart.
func TestSaturationConstantMatchesConeMapping(t *testing.T) {
	satDeg := math.Acos(poseCloudSaturationCos) * 180 / math.Pi // 177.4374...
	if !coneCloud(satDeg+0.001).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)) {
		t.Error("at the angle where ResolveGoalCloudConfig warns, the cone must actually be saturated")
	}
	if coneCloud(satDeg-0.01).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)) {
		t.Error("just below that angle the cone must still bound tilt")
	}
}

func TestConeRollIsFree(t *testing.T) {
	c := coneCloud(30)
	// 180 is the actual boundary case: |Theta| = 180 against a leeway of 180 + 0.001.
	for _, roll := range []float64{0, 45, 90, 179, 180} {
		p := spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
			&spatialmath.R4AA{RX: 0, RY: 0, RZ: 1, Theta: utils.DegToRad(roll)}))
		if !c.PoseInCloud(testGoal(), p) {
			t.Errorf("roll %.0f should be accepted", roll)
		}
	}
}

func TestConeTiltPlusRollStaysBounded(t *testing.T) {
	c := coneCloud(30)
	for _, roll := range []float64{0, 70, 170} {
		for _, tc := range []struct {
			tilt float64
			want bool
		}{{20, true}, {29, true}, {31, false}} {
			p := spatialmath.Compose(testGoal(), spatialmath.Compose(
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RX: 1, Theta: utils.DegToRad(tc.tilt)}),
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RZ: 1, Theta: utils.DegToRad(roll)})))
			if got := c.PoseInCloud(testGoal(), p); got != tc.want {
				t.Errorf("tilt %.0f roll %.0f: PoseInCloud = %v, want %v", tc.tilt, roll, got, tc.want)
			}
		}
	}
}

// effectiveConeDeg measures the cone rdk's PoseInCloud actually enforces, by binary
// search on acceptance.
func effectiveConeDeg(c *referenceframe.PoseCloud) float64 {
	lo, hi := 0.0, 180.0
	for i := 0; i < 60; i++ {
		mid := (lo + hi) / 2
		if c.PoseInCloud(testGoal(), tiltedBy(1, 0, mid)) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

func TestConeEffectiveWideningFormula(t *testing.T) {
	// rdk ADDS its 0.001 epsilon to the leeway, so the cone actually enforced is always
	// slightly wider than requested. Comparing measurement (LHS, from rdk's real
	// PoseInCloud) against our model (RHS) is not tautological: it pins that our
	// understanding of rdk's behavior is right, for any angle rather than six points.
	// The sample spans all three regimes: 0 is epsilon-dominated, 2.6 is the crossover
	// where the requested leeway equals the epsilon, 90+ is epsilon-negligible.
	for _, requested := range []float64{0, 1, 2.6, 10, 30, 90, 150} {
		want := math.Acos(math.Cos(utils.DegToRad(requested))-0.001) * 180 / math.Pi
		if got := effectiveConeDeg(coneCloud(requested)); math.Abs(got-want) > 0.01 {
			t.Errorf("effective cone for requested %.1f: got %v, want %v", requested, got, want)
		}
	}
	// One literal anchor, independent of the formula: at tolerance 0 the epsilon alone
	// still admits ~2.5626deg. That is the floor a caller cannot get below.
	if got := effectiveConeDeg(coneCloud(0)); math.Abs(got-2.5626) > 0.01 {
		t.Errorf("effective cone at tolerance 0: got %v, want ~2.5626", got)
	}
}

func TestConePositionalBoxNotRadius(t *testing.T) {
	// Deliberately NOT coneCloud (which uses 2.5): the bounds below are measured values
	// tied to a 1.0mm tolerance, so this test pins its own.
	c := coneToPoseCloud(GoalCloudConfig{OrientationToleranceDeg: 30, PositionToleranceMM: 1.0})
	offset := func(dx, dy, dz float64) spatialmath.Pose {
		g := testGoal()
		return spatialmath.NewPose(
			r3.Vector{X: g.Point().X + dx, Y: g.Point().Y + dy, Z: g.Point().Z + dz},
			g.Orientation())
	}
	// The epsilon is additive: at X=1.0 the true bound is 1.001mm.
	if !c.PoseInCloud(testGoal(), offset(1.0009, 0, 0)) {
		t.Error("1.0009mm offset should be accepted (the bound is 1.001mm)")
	}
	if c.PoseInCloud(testGoal(), offset(1.0015, 0, 0)) {
		t.Error("1.0015mm offset should be rejected")
	}
	// Per-axis box, not a radius: all three axes at the bound is accepted at norm ~1.73mm.
	if !c.PoseInCloud(testGoal(), offset(1.0005, 1.0005, 1.0005)) {
		t.Error("all three axes at the bound should be accepted: the cloud is a box, not a radius")
	}
}

// TestRDKContractThetaReportsTiltAzimuth documents why coneToPoseCloud sets Theta: 180, by
// pinning the mechanism directly: the relative Theta reports a tilt's AZIMUTH, not its roll.
//
// A characterization test for rdk's Theta semantics, not a guard on coneToPoseCloud (which
// overwrites Theta, so a mutation there is a no-op here -- TestConeBoundsTiltIsotropically
// guards the mapping end-to-end). It earns its place by pinning the rdk contract nothing else
// asserts: ovX.Theta == 90 and ovY.Theta == 0. Should an rdk upgrade change those semantics,
// the acceptance tests would fail with vague mismatches while this one names the broken
// contract outright.
//
// The tempting shorthand -- "a smaller Theta rejects tilts" -- is FALSE: Theta=0 below rejects
// a tilt about X but would ACCEPT one about Y (azimuth 90 reports Theta=0). It actually carves
// out a wedge of tilt DIRECTIONS, and since reported Theta sweeps the full [-180, 180] as
// azimuth goes around, only a leeway of 180 admits every azimuth.
func TestRDKContractThetaReportsTiltAzimuth(t *testing.T) {
	broken := coneCloud(30)
	broken.Theta = 0 // the "obvious simplification"
	if broken.PoseInCloud(testGoal(), tiltedBy(1, 0, 5)) {
		t.Error("with Theta=0 even a 5deg tilt is rejected, defeating the cone entirely")
	}

	// Proof of the cause: a pure tilt about X reports Theta=90, about Y reports Theta=0.
	ovX := spatialmath.PoseBetween(testGoal(), tiltedBy(1, 0, 5)).Orientation().OrientationVectorDegrees()
	if math.Abs(ovX.Theta-90.0) > 1e-6 {
		t.Errorf("tilt about X reports Theta=90, not roll: got %v", ovX.Theta)
	}
	ovY := spatialmath.PoseBetween(testGoal(), tiltedBy(0, 1, 5)).Orientation().OrientationVectorDegrees()
	if math.Abs(ovY.Theta-0.0) > 1e-6 {
		t.Errorf("tilt about Y reports Theta=0: got %v", ovY.Theta)
	}
}

// TestRDKContractZeroLeewayDemandsMicronMatch documents why X/Y/Z must be > 0: the 0.001
// epsilon means a zero leeway demands a match within 1 micron -- which no IK solution will
// realistically achieve, making the cloud useless in practice even though it does technically
// accept an exact match (measured: it still accepts 0.0009mm, and rejects 0.0011mm).
//
// This is a characterization test for rdk's epsilon rule, not a guard on coneToPoseCloud
// (it overwrites X/Y/Z, so a mutation there is a no-op here -- TestConePositionalBoxNotRadius
// is what guards the mapping). It earns its place by demonstrating the failure mode that
// motivates defaultPositionToleranceMM: the cloud a caller gets with zero leeway.
func TestRDKContractZeroLeewayDemandsMicronMatch(t *testing.T) {
	broken := coneCloud(30)
	broken.X, broken.Y, broken.Z = 0, 0, 0
	g := testGoal()
	nudged := spatialmath.NewPose(
		r3.Vector{X: g.Point().X + 0.01, Y: g.Point().Y, Z: g.Point().Z}, g.Orientation())
	if broken.PoseInCloud(g, nudged) {
		t.Error("with zero positional leeway even a 0.01mm offset is rejected")
	}
}

func TestParsePoseCloudValid(t *testing.T) {
	// extra arrives via structpb, so every JSON number is a float64.
	//
	// Every value is DISTINCT and all seven are asserted: identical values (e.g. x==y)
	// would let a setter transposition survive undetected, and coneToPoseCloud's OX:1/OY:1
	// means no other test would catch it either.
	got, err := parsePoseCloud(map[string]interface{}{
		"x": 2.0, "y": 3.0, "z": 0.5, "ox": 0.25, "oy": 0.5, "oz": 0.1, "theta": 180.0,
	})
	if err != nil {
		t.Fatalf("parsePoseCloud: %v", err)
	}
	if got.X != 2.0 {
		t.Errorf("X = %v, want 2.0", got.X)
	}
	if got.Y != 3.0 {
		t.Errorf("Y = %v, want 3.0", got.Y)
	}
	if got.Z != 0.5 {
		t.Errorf("Z = %v, want 0.5", got.Z)
	}
	if got.OX != 0.25 {
		t.Errorf("OX = %v, want 0.25", got.OX)
	}
	if got.OY != 0.5 {
		t.Errorf("OY = %v, want 0.5", got.OY)
	}
	if got.OZ != 0.1 {
		t.Errorf("OZ = %v, want 0.1", got.OZ)
	}
	if got.Theta != 180.0 {
		t.Errorf("Theta = %v, want 180.0", got.Theta)
	}
}

func TestParsePoseCloudOmittedFieldsStayZero(t *testing.T) {
	got, err := parsePoseCloud(map[string]interface{}{"oz": 0.1})
	if err != nil {
		t.Fatalf("parsePoseCloud: %v", err)
	}
	if got.X != 0.0 {
		t.Errorf("omitted fields stay zero -- sharp, but faithful passthrough: X = %v", got.X)
	}
}

func TestParsePoseCloudRejects(t *testing.T) {
	for name, input := range map[string]interface{}{
		"not an object":       "nope",
		"docs-style casing":   map[string]interface{}{"OX": 1.0},
		"protobuf casing":     map[string]interface{}{"o_x": 1.0},
		"not a field in v1.6": map[string]interface{}{"reference_frame": 1.0},
		"unknown key":         map[string]interface{}{"wobble": 1.0},
		"non-numeric":         map[string]interface{}{"oz": "0.1"},
		"negative":            map[string]interface{}{"oz": -0.1},
		"NaN":                 map[string]interface{}{"oz": math.NaN()},
		"Inf":                 map[string]interface{}{"oz": math.Inf(1)},
		"nil value":           nil,
	} {
		t.Run(name, func(t *testing.T) {
			if _, err := parsePoseCloud(input); err == nil {
				t.Error("want an error, got nil")
			}
		})
	}
}

func TestBuildMoveDestinationConePath(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := BuildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}
	if path != pathCone {
		t.Errorf("path = %v, want pathCone", path)
	}
	if dest.Parent() != "myarm_origin" {
		t.Errorf("the frame name is passed through unmodified: got %q", dest.Parent())
	}
	if dest.GoalCloud == nil {
		t.Fatal("dest.GoalCloud is nil")
	}
	if want := coneToPoseCloud(cfg); *dest.GoalCloud != *want {
		t.Errorf("the destination carries the config's cone: got %+v, want %+v", *dest.GoalCloud, *want)
	}
	if _, ok := planExtra["goal_metric_type"]; ok {
		t.Error("the cone replaces position_only: goal_metric_type must not be sent")
	}
}

func TestBuildMoveDestinationForwardsCallerExtras(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	_, planExtra, _, err := BuildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"timeout": 5.0})
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}
	if planExtra["timeout"] != 5.0 {
		t.Errorf("unrelated caller keys still pass through: timeout = %v", planExtra["timeout"])
	}
}

func TestBuildMoveDestinationRawCloudPath(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := BuildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}, "timeout": 5.0})
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}
	if path != pathRawCloud {
		t.Errorf("path = %v, want pathRawCloud", path)
	}
	if dest.GoalCloud == nil {
		t.Fatal("dest.GoalCloud is nil")
	}
	if dest.GoalCloud.OZ != 0.5 {
		t.Errorf("the raw cloud replaces the cone entirely: OZ = %v, want 0.5", dest.GoalCloud.OZ)
	}
	if _, ok := planExtra["pose_cloud"]; ok {
		t.Error("pose_cloud is not a planner key")
	}
	if planExtra["timeout"] != 5.0 {
		t.Errorf("timeout = %v, want 5.0", planExtra["timeout"])
	}
}

func TestBuildMoveDestinationMetricTypePath(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := BuildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"goal_metric_type": "position_only"})
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}
	if path != pathMetricType {
		t.Errorf("path = %v, want pathMetricType", path)
	}
	if dest.GoalCloud != nil {
		t.Error("no cloud: position_only sets orientScale=0, so a cloud's orientation leeways would stop mattering")
	}
	if planExtra["goal_metric_type"] != "position_only" {
		t.Errorf("the caller's metric is forwarded: got %v", planExtra["goal_metric_type"])
	}
}

func TestBuildMoveDestinationRejectsBothKeys(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	_, _, _, err := BuildMoveDestination("a_origin", testGoal(), cfg, map[string]interface{}{
		"pose_cloud":       map[string]interface{}{"oz": 0.5},
		"goal_metric_type": "position_only",
	})
	if err == nil {
		t.Fatal("incoherent: a raw cloud plus a metric that ignores clouds; want an error")
	}
	// The message is the entire UX here -- there is no fallback -- so pin that it names
	// both offending keys rather than leaving the caller to guess which one to drop.
	if !strings.Contains(err.Error(), "pose_cloud") {
		t.Errorf("error must name pose_cloud: %v", err)
	}
	if !strings.Contains(err.Error(), "goal_metric_type") {
		t.Errorf("error must name goal_metric_type: %v", err)
	}
}

func TestBuildMoveDestinationDoesNotMutateCallerExtra(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	extra := map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}}
	_, _, _, err := BuildMoveDestination("a_origin", testGoal(), cfg, extra)
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}
	if _, ok := extra["pose_cloud"]; !ok {
		t.Error("the caller's map must not be mutated")
	}
}

func TestBuildMoveDestinationPropagatesParseError(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, _, err := BuildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"OX": 1.0}})
	if err == nil {
		t.Fatal("want an error, got nil")
	}
	if dest != nil {
		t.Errorf("dest = %v, want nil", dest)
	}
	if planExtra != nil {
		t.Errorf("planExtra = %v, want nil", planExtra)
	}
}

func TestBuildMoveDestinationReturnsPathInvalidOnError(t *testing.T) {
	cfg := ResolveGoalCloudConfig(0, 0, nil)
	_, _, path, err := BuildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"OX": 1.0}})
	if err == nil {
		t.Fatal("want an error, got nil")
	}
	if path != pathInvalid {
		t.Errorf("an error must not return pathCone, or WrapMoveErr would blame the cone for a parse error: path = %v", path)
	}
	// The guarantee that makes this matter: WrapMoveErr must not dress it up as a cone failure.
	if got := WrapMoveErr(err, path, cfg); got != err {
		t.Errorf("pathInvalid must pass the error through unwrapped: got %v", got)
	}
}

// TestGoalCloudSurvivesSerialization guards the module-boundary gotcha: the cloud is only
// useful if it crosses gRPC. An in-process check would pass even if it never shipped.
func TestGoalCloudSurvivesSerialization(t *testing.T) {
	cfg := ResolveGoalCloudConfig(30, 1.0, nil)
	dest, _, _, err := BuildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	if err != nil {
		t.Fatalf("BuildMoveDestination: %v", err)
	}

	proto := referenceframe.PoseInFrameToProtobuf(dest)
	if proto.GoalCloud == nil {
		t.Fatal("the cloud must be serialized onto the wire")
	}

	got := referenceframe.ProtobufToPoseInFrame(proto)
	if got.GoalCloud == nil {
		t.Fatal("the cloud must survive the round trip")
	}

	want := dest.GoalCloud
	for _, f := range []struct {
		name      string
		want, got float64
	}{
		{"X", want.X, got.GoalCloud.X},
		{"Y", want.Y, got.GoalCloud.Y},
		{"Z", want.Z, got.GoalCloud.Z},
		{"OX", want.OX, got.GoalCloud.OX},
		{"OY", want.OY, got.GoalCloud.OY},
		{"OZ", want.OZ, got.GoalCloud.OZ},
		{"Theta", want.Theta, got.GoalCloud.Theta},
	} {
		if math.Abs(f.got-f.want) > 1e-9 {
			t.Errorf("%s = %v, want %v", f.name, f.got, f.want)
		}
	}
}
