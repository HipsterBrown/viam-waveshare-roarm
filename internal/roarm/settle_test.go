package roarm

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"
	"time"
)

// vclock is a virtual clock: sleeping advances it, so a sweep over 30-second
// deadlines runs instantly. Reads may advance it too, to model read latency.
type vclock struct{ t time.Time }

func newVclock() *vclock { return &vclock{t: time.Date(2026, 9, 21, 12, 0, 0, 0, time.UTC)} }

func (v *vclock) clock() Clock {
	return Clock{
		Now: func() time.Time { return v.t },
		SleepUntil: func(_ context.Context, to time.Time) error {
			if to.After(v.t) {
				v.t = to
			}
			return nil
		},
	}
}

// rampPos is where a healthy joint is `t` seconds into a move of `travel`
// radians commanded at v rad/s and a rad/s^2: a trapezoid when the move
// reaches cruise, a triangle when acceleration limits it.
func rampPos(t, travel, v, a float64) float64 {
	tRamp := v / a
	if travel < v*v/a {
		tHalf := math.Sqrt(travel / a)
		switch {
		case t <= tHalf:
			return 0.5 * a * t * t
		case t >= 2*tHalf:
			return travel
		default:
			rem := 2*tHalf - t
			return travel - 0.5*a*rem*rem
		}
	}
	d1 := v * v / (2 * a)
	tCruise := (travel - 2*d1) / v
	switch {
	case t <= tRamp:
		return 0.5 * a * t * t
	case t <= tRamp+tCruise:
		return d1 + v*(t-tRamp)
	case t <= 2*tRamp+tCruise:
		rem := 2*tRamp + tCruise - t
		return travel - 0.5*a*rem*rem
	default:
		return travel
	}
}

// sweep is the validated config range plus the travels that matter.
var (
	sweepTravelsDeg = []float64{1, 2, 5, 15, 45, 90, 170}
	sweepSpeedsDeg  = []float64{3, 25, 50, 90, 180}
	sweepAccelsDeg  = []float64{10, 25, 50, 100, 300, 500}
)

func req(travelRad float64, speedDeg, accelDeg float64) SettleRequest {
	return SettleRequest{
		Start:         []float64{0, 0, 0, 0, 0, 0},
		Target:        []float64{travelRad, 0, 0, 0, 0, 0},
		Mask:          ArmMask,
		SpeedUnits:    SpeedToUnits(speedDeg),
		AccUnits:      AccelToUnits(accelDeg),
		RequireMotion: true,
	}
}

// Audit 2.1, as a guarantee across the whole validated config range: a healthy
// arm always arrives, and never reports a stall or a timeout. Before this task
// the settle ended on poll 2 with the arm 0% of the way to its target at any
// acceleration below about 76 deg/s^2.
func TestHealthyRampAlwaysArrives(t *testing.T) {
	for _, travelDeg := range sweepTravelsDeg {
		for _, sp := range sweepSpeedsDeg {
			for _, ac := range sweepAccelsDeg {
				travel := travelDeg * math.Pi / 180
				r := req(travel, sp, ac)
				v := SpeedFromUnits(r.SpeedUnits) * math.Pi / 180
				a := AccelFromUnits(r.AccUnits) * math.Pi / 180
				vc := newVclock()
				start := vc.t
				read := func(context.Context) ([]float64, error) {
					return []float64{rampPos(vc.t.Sub(start).Seconds(), travel, v, a), 0, 0, 0, 0, 0}, nil
				}
				res, err := waitUntilSettled(context.Background(), read, vc.clock(), r)
				if err != nil {
					t.Fatalf("%.0f deg at %.0f deg/s, %.0f deg/s^2: %v", travelDeg, sp, ac, err)
				}
				if res.Outcome != SettleArrived {
					t.Fatalf("%.0f deg at %.0f deg/s, %.0f deg/s^2: outcome %v after %v",
						travelDeg, sp, ac, res.Outcome, res.Elapsed)
				}
			}
		}
	}
}

// The invariant an upper clamp would re-break: a derived deadline is never
// shorter than the move it is meant to cover. Includes a full joint sweep,
// which is where the old 15-second ceiling truncated healthy moves.
func TestDeadlineCoversTheModelledDuration(t *testing.T) {
	for _, travelDeg := range append(sweepTravelsDeg, 360) {
		for _, sp := range sweepSpeedsDeg {
			for _, ac := range sweepAccelsDeg {
				p, err := planSettle(req(travelDeg*math.Pi/180, sp, ac))
				if err != nil {
					t.Fatal(err)
				}
				if p.Deadline < p.Duration {
					t.Fatalf("%.0f deg at %.0f deg/s, %.0f deg/s^2: deadline %v < duration %v",
						travelDeg, sp, ac, p.Deadline, p.Duration)
				}
			}
		}
	}
}

// Audit 2.2: an arm that never moves is an error, not a success. Travels start
// at 5 degrees because the never-moved branch needs the target to be more than
// 2*settleTolRad (2.29 deg) away; smaller travels are covered below.
func TestBlockedArmIsAnError(t *testing.T) {
	for _, travelDeg := range []float64{5, 15, 45, 90, 170} {
		for _, sp := range sweepSpeedsDeg {
			for _, ac := range sweepAccelsDeg {
				r := req(travelDeg*math.Pi/180, sp, ac)
				p, err := planSettle(r)
				if err != nil {
					t.Fatal(err)
				}
				vc := newVclock()
				start := vc.t
				stuck := func(context.Context) ([]float64, error) { return []float64{0, 0, 0, 0, 0, 0}, nil }
				res, err := waitUntilSettled(context.Background(), stuck, vc.clock(), r)
				if err == nil {
					t.Fatalf("%.0f deg at %.0f deg/s, %.0f deg/s^2: blocked arm reported %v", travelDeg, sp, ac, res.Outcome)
				}
				if !errors.Is(err, ErrArmDidNotMove) {
					t.Fatalf("want the never-moved error, got %v", err)
				}
				// The window's samples already exist when the grace elapses, so
				// detection lands on the first poll at or after it.
				if el := vc.t.Sub(start); el > p.Grace+2*settlePollInterval {
					t.Fatalf("%.0f deg at %.0f deg/s, %.0f deg/s^2: detected after %v, grace %v", travelDeg, sp, ac, el, p.Grace)
				}
			}
		}
	}
}

// A sub-tolerance target is legitimately "already there", so the sweep above
// starts at 5 degrees. Pinned so nobody loosens the never-moved condition to
// make a smaller travel fail.
func TestBlockedArmBelowTolerance(t *testing.T) {
	for _, tc := range []struct {
		travelDeg float64
		want      SettleOutcome
	}{{1, SettleArrived}, {2, SettleStopped}} {
		vc := newVclock()
		stuck := func(context.Context) ([]float64, error) { return []float64{0, 0, 0, 0, 0, 0}, nil }
		res, err := waitUntilSettled(context.Background(), stuck, vc.clock(), req(tc.travelDeg*math.Pi/180, 50, 100))
		if err != nil {
			t.Fatalf("%.0f deg: %v", tc.travelDeg, err)
		}
		if res.Outcome != tc.want {
			t.Fatalf("%.0f deg: outcome %v, want %v", tc.travelDeg, res.Outcome, tc.want)
		}
	}
}

// An arm that makes real progress and then stops short is a success with a
// warning, which is the case the stall check exists for.
func TestStoppedShortIsASuccess(t *testing.T) {
	travel := 45 * math.Pi / 180
	// 3 degrees short: more than 2*settleTolRad (2.3 deg), so it is a genuine
	// stop rather than an arrival inside the tolerance.
	stopAt := travel - 3*math.Pi/180
	vc := newVclock()
	start := vc.t
	read := func(context.Context) ([]float64, error) {
		return []float64{math.Min(stopAt, 0.873*vc.t.Sub(start).Seconds()), 0, 0, 0, 0, 0}, nil
	}
	res, err := waitUntilSettled(context.Background(), read, vc.clock(), req(travel, 50, 100))
	if err != nil {
		t.Fatal(err)
	}
	if res.Outcome != SettleStopped {
		t.Fatalf("outcome %v, want stopped short", res.Outcome)
	}
}

// RequireMotion false is how Grab closes onto an object: the jaw legitimately
// stops without reaching the closed limit, and grabMarginRad (0.05) exceeds
// 2*settleTolRad (0.04), so the never-moved rule would otherwise fire on a
// correct hold.
func TestBlockedJawWithoutRequireMotion(t *testing.T) {
	r := req(45*math.Pi/180, 50, 100)
	r.RequireMotion = false
	vc := newVclock()
	stuck := func(context.Context) ([]float64, error) { return []float64{0, 0, 0, 0, 0, 0}, nil }
	res, err := waitUntilSettled(context.Background(), stuck, vc.clock(), r)
	if err != nil || res.Outcome != SettleStopped {
		t.Fatalf("want a stopped-short success, got %v %v", res.Outcome, err)
	}
}

// Audit 2.6: the budget is a wall-clock deadline, not a poll count, even when
// every read is slower than the poll interval.
func TestDeadlineIsWallClock(t *testing.T) {
	travel := 45 * math.Pi / 180
	r := req(travel, 50, 100)
	p, _ := planSettle(r)
	vc := newVclock()
	start := vc.t
	// Each read costs 200 ms of clock, four times the poll interval. The arm
	// creeps at 0.05 rad/s: fast enough to clear StallRad over the 200 ms
	// window so it is never called stalled, far too slow to cover 45 degrees
	// inside the deadline.
	read := func(context.Context) ([]float64, error) {
		vc.t = vc.t.Add(200 * time.Millisecond)
		return []float64{0.05 * vc.t.Sub(start).Seconds(), 0, 0, 0, 0, 0}, nil
	}
	res, err := waitUntilSettled(context.Background(), read, vc.clock(), r)
	if err == nil {
		t.Fatalf("expected a timeout, got %v", res.Outcome)
	}
	if el := vc.t.Sub(start); el > p.Deadline+500*time.Millisecond {
		t.Fatalf("overran its %v deadline: %v", p.Deadline, el)
	}
	for _, want := range []string{"did not settle", "from the target"} {
		if !strings.Contains(err.Error(), want) {
			t.Fatalf("the timeout error should report the real elapsed time and remaining distance: %v", err)
		}
	}
	if res.SlowestRead < 200*time.Millisecond {
		t.Fatalf("SlowestRead %v should have caught the slow reads", res.SlowestRead)
	}
}

// The stall reference is tested directly: the abstain case (no sample old
// enough) lasts a single poll, so it is not observable through a whole settle.
// The obvious wrong fallback -- compare with the immediately previous sample --
// is the audit 2.1 bug, so the guard is worth pinning where it lives.
func TestStallReference(t *testing.T) {
	base := time.Date(2026, 9, 21, 12, 0, 0, 0, time.UTC)
	at := func(ms int) time.Time { return base.Add(time.Duration(ms) * time.Millisecond) }
	samples := []settleSample{
		{at: at(0), pos: []float64{0}},
		{at: at(100), pos: []float64{1}},
		{at: at(200), pos: []float64{2}},
		{at: at(300), pos: []float64{3}},
	}

	// Cutoff at 250 ms: the 200 ms sample is the newest one old enough, and
	// everything before it is dropped.
	kept, ref, ok := stallReference(samples, at(250))
	if !ok || ref.pos[0] != 2 {
		t.Fatalf("ref = %v, ok = %v; want the 200 ms sample", ref.pos, ok)
	}
	if len(kept) != 2 {
		t.Fatalf("kept %d samples, want the reference plus the ones inside the window", len(kept))
	}

	// A cutoff before every sample: nothing is old enough, so the check must
	// abstain rather than hand back the previous sample.
	if _, _, ok := stallReference(samples, base.Add(-time.Second)); ok {
		t.Fatal("expected an abstention when no sample is old enough")
	}

	// An exact match counts as old enough: the boundary is inclusive, which is
	// what makes the window exactly Window long rather than one poll longer.
	if _, ref, ok := stallReference(samples, at(200)); !ok || ref.pos[0] != 2 {
		t.Fatalf("a sample exactly at the cutoff should be usable, got %v %v", ref, ok)
	}
}

// A non-positive profile is a programming error: the derivation divides by
// both, so zero would give an infinite deadline and a settle that never exits.
func TestPlanRejectsANonPositiveProfile(t *testing.T) {
	for _, r := range []SettleRequest{
		{Start: []float64{0}, Target: []float64{1}, SpeedUnits: 0, AccUnits: 11},
		{Start: []float64{0}, Target: []float64{1}, SpeedUnits: 569, AccUnits: 0},
	} {
		if _, err := planSettle(r); err == nil {
			t.Fatalf("expected an error for speed=%d acc=%d", r.SpeedUnits, r.AccUnits)
		}
	}
}

// The worked table in the spec, recomputed from the code's own constants so a
// change to the model shows up here rather than in a stale literal.
func TestPlanWorkedValues(t *testing.T) {
	p, err := planSettle(req(45*math.Pi/180, 50, 100))
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("default profile, 45 deg: window %v grace %v deadline %v duration %v", p.Window, p.Grace, p.Deadline, p.Duration)
	if p.Window != 4*settlePollInterval {
		t.Fatalf("window %v should be at its floor for a fast profile", p.Window)
	}
	// 15 degrees is acceleration-limited, so its grace takes sqrt(travel/a),
	// which is shorter than v/a.
	short, _ := planSettle(req(15*math.Pi/180, 50, 100))
	if short.Grace >= p.Grace {
		t.Fatalf("an acceleration-limited move should have a shorter grace: %v vs %v", short.Grace, p.Grace)
	}
}

func TestSettle_ReadErrorPropagates(t *testing.T) {
	boom := errors.New("boom")
	read := func(context.Context) ([]float64, error) { return nil, boom }
	vc := newVclock()
	if _, err := waitUntilSettled(context.Background(), read, vc.clock(), req(45*math.Pi/180, 50, 100)); !errors.Is(err, boom) {
		t.Fatalf("expected boom, got %v", err)
	}
}

func TestSettle_ShortFeedbackIsAnError(t *testing.T) {
	read := func(context.Context) ([]float64, error) { return []float64{0, 0}, nil }
	vc := newVclock()
	_, err := waitUntilSettled(context.Background(), read, vc.clock(), req(45*math.Pi/180, 50, 100))
	if err == nil || !strings.Contains(err.Error(), "short feedback") {
		t.Fatalf("expected short-feedback error, got %v", err)
	}
}

func TestMaxTravel(t *testing.T) {
	a := []float64{0, 0, 0, 0, 0, 0}
	b := []float64{0.1, -0.5, 0, 0, 0, 2}
	if got := MaxTravel(a, b, nil); got != 2 {
		t.Fatalf("nil mask = all joints: got %v", got)
	}
	if got := MaxTravel(a, b, ArmMask); got != 0.5 {
		t.Fatalf("arm mask: got %v", got)
	}
}
