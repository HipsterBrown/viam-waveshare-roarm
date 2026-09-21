package roarm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"
)

// The firmware reports joint positions but no moving flag, so "the move is
// done" is derived from position. Every quantity that depends on the commanded
// profile is derived in planSettle from one set of inputs: a fixed per-poll
// threshold cannot express "still moving" for an arbitrary profile, which is
// how a healthy arm accelerating from rest used to be mistaken for a finished
// move at any acceleration below about 76 deg/s^2.
const (
	settlePollInterval = 50 * time.Millisecond
	settleTolRad       = 0.02  // ~1.1 degrees: close enough to call it arrived
	StallRad           = 0.005 // ~0.3 degrees: the motion floor over a window
	minSettleTimeout   = 500 * time.Millisecond
	maxStallWindow     = 2 * time.Second

	// stallSpeedFraction is the share of the commanded speed a healthy arm is
	// assumed to actually achieve when sizing the stall window. Pessimistic on
	// purpose: over-sizing the window only delays a stall verdict, while
	// under-sizing it calls a slow-but-moving arm stalled.
	stallSpeedFraction = 0.25
	// stallWindowTravelRad is how far a healthy arm must move within one
	// window for the arm to count as moving.
	stallWindowTravelRad = 4 * StallRad

	// settleBudgetWarnFraction is the share of its deadline a settle may use
	// before it warns. With a ramp-aware deadline a healthy move sits near
	// 0.5, so 0.75 means the arm is materially slower than commanded.
	settleBudgetWarnFraction = 0.75

	// IsMovingProbeGap separates the two position samples IsMoving compares.
	IsMovingProbeGap = 40 * time.Millisecond
)

// Joint masks for a settle: the arm judges joints 1-5 while the gripper is
// commanded to hold, the gripper judges joint 6 alone. Any 6-element mask is
// valid; cmd/cli builds a single-joint one.
var (
	ArmMask     = []bool{true, true, true, true, true, false}
	GripperMask = []bool{false, false, false, false, false, true}
)

// ErrArmDidNotMove is returned when a settle sees no movement at all from the
// pose measured before the write. Exported so the health counters and a
// caller wanting to retry can tell it from a plain timeout.
var ErrArmDidNotMove = errors.New("the arm did not move")

// SettleOutcome is how a settle ended.
type SettleOutcome int

const (
	// SettleArrived: every masked joint is within settleTolRad of its target.
	SettleArrived SettleOutcome = iota
	// SettleStopped: the arm made real progress and then stopped short. This is
	// the case the stall check exists for (a servo settling under load, a jaw
	// closing onto an object); callers treat it as success.
	SettleStopped
)

func (o SettleOutcome) String() string {
	if o == SettleArrived {
		return "arrived"
	}
	return "stopped short"
}

// SettleRequest is everything the settle needs to tell "arrived", "stopped
// short" and "never moved" apart.
type SettleRequest struct {
	// Target and Start are full 6-joint poses: Target is what was commanded,
	// Start is the pose measured immediately BEFORE the write. Start is what
	// makes "the arm never moved" detectable, so it must be a measured pose
	// and never a commanded one.
	Target, Start []float64
	Mask          []bool
	// SpeedUnits and AccUnits are the profile actually commanded, in firmware
	// units. Both must be positive: the derivation divides by them.
	SpeedUnits, AccUnits int
	// RequireMotion makes "the arm never left Start" an error. True for arm
	// moves and gripper opens; false for Grab, where closing onto an object
	// and not moving is the expected outcome.
	RequireMotion bool
	// Timeout overrides the derived deadline only; the window and grace still
	// derive from the profile. Zero derives everything, which is what every
	// production caller passes. A test forcing a short timeout on a slow
	// profile hits the deadline before the grace and so never reaches the
	// stall or never-moved branches.
	Timeout time.Duration
}

// SettleResult reports how a settle ended and what it cost, so a caller can
// log or warn without re-deriving anything.
type SettleResult struct {
	Positions   []float64
	Outcome     SettleOutcome
	Elapsed     time.Duration
	Deadline    time.Duration
	SlowestRead time.Duration
	Polls       int
	// Retries is filled by Controller.WaitUntilSettled from its own counters;
	// the transport-free core does not know about retries.
	Retries int
}

// settlePlan is the timing derived for one settle.
type settlePlan struct {
	Travel   float64
	Duration time.Duration
	Window   time.Duration
	Grace    time.Duration
	Deadline time.Duration
}

// planSettle derives every timing quantity from the travel and the commanded
// profile.
func planSettle(req SettleRequest) (settlePlan, error) {
	if req.SpeedUnits <= 0 || req.AccUnits <= 0 {
		return settlePlan{}, fmt.Errorf(
			"settle needs a positive speed and acceleration in firmware units, got speed=%d acc=%d "+
				"(0 means MAXIMUM to the firmware but is not a usable profile here: the derivation divides by both)",
			req.SpeedUnits, req.AccUnits)
	}
	if len(req.Start) == 0 || len(req.Target) == 0 {
		return settlePlan{}, fmt.Errorf("settle needs both a Start and a Target pose (got %d and %d joints)",
			len(req.Start), len(req.Target))
	}

	// SpeedFromUnits is deg/s and AccelFromUnits is deg/s^2; the poses and
	// StallRad are radians, so both need converting or every derived value is
	// out by 57x.
	v := SpeedFromUnits(req.SpeedUnits) * math.Pi / 180
	a := AccelFromUnits(req.AccUnits) * math.Pi / 180

	// travel is the worst single masked joint, not a sum: the firmware applies
	// one speed to every joint in a T:102 command, so the shorter-travel joints
	// finish early and the longest one sets the duration.
	p := settlePlan{Travel: MaxTravel(req.Start, req.Target, req.Mask)}

	// The modelled duration of the commanded move: a trapezoid when it reaches
	// cruise, a triangle when acceleration limits it. A speed-only 2*travel/v
	// is what used to make a short move at the default profile exceed its own
	// deadline.
	if p.Travel >= v*v/a {
		p.Duration = seconds(p.Travel/v + v/a)
	} else {
		p.Duration = seconds(2 * math.Sqrt(p.Travel/a))
	}

	// The window over which a healthy arm must cover more than StallRad.
	p.Window = min(max(seconds(stallWindowTravelRad/(stallSpeedFraction*v)), 4*settlePollInterval), maxStallWindow)

	// The ramp, then one window. For a move too short to reach cruise the real
	// ramp is sqrt(travel/a), not v/a; using v/a there put the grace past the
	// deadline for a third of the validated range.
	p.Grace = seconds(math.Min(v/a, math.Sqrt(p.Travel/a))) + p.Window

	// No upper clamp: a ceiling truncates healthy slow moves (a 360-degree
	// joint sweep at 3 deg/s genuinely takes 120 s), both inputs are bounded,
	// and a stopped arm is caught by the grace within 6.8 s anywhere in the
	// range rather than by this deadline. The caller's context is the backstop.
	p.Deadline = max(2*p.Duration, p.Grace+p.Window, minSettleTimeout)
	if req.Timeout > 0 {
		p.Deadline = req.Timeout
	}
	return p, nil
}

func seconds(f float64) time.Duration { return time.Duration(f * float64(time.Second)) }

// MaxTravel returns the largest |a[i]-b[i]| over the masked joints (nil mask
// means every joint). Slices shorter than the mask are compared as far as
// they go.
func MaxTravel(a, b []float64, mask []bool) float64 {
	m := 0.0
	for i := 0; i < len(a) && i < len(b); i++ {
		if mask != nil && (i >= len(mask) || !mask[i]) {
			continue
		}
		if d := math.Abs(a[i] - b[i]); d > m {
			m = d
		}
	}
	return m
}

// settleSample is one timestamped position read.
type settleSample struct {
	at  time.Time
	pos []float64
}

// waitUntilSettled is the transport-free core. It polls until the masked
// joints arrive, stop moving, or the derived deadline passes.
func waitUntilSettled(
	ctx context.Context,
	read func(context.Context) ([]float64, error),
	clk Clock,
	req SettleRequest,
) (SettleResult, error) {
	plan, err := planSettle(req)
	if err != nil {
		return SettleResult{}, err
	}
	start := clk.Time()
	deadline := start.Add(plan.Deadline)
	res := SettleResult{Deadline: plan.Deadline}
	var samples []settleSample

	for {
		if err := clk.WaitUntil(ctx, clk.Time().Add(settlePollInterval)); err != nil {
			return res, err
		}
		if clk.Time().After(deadline) {
			remaining := 0.0
			if len(samples) > 0 {
				remaining = MaxTravel(samples[len(samples)-1].pos, req.Target, req.Mask)
			}
			return res, fmt.Errorf("the arm did not settle within its %v budget (elapsed %v, %d polls, still %.1f deg from the target)",
				plan.Deadline, res.Elapsed.Round(time.Millisecond), res.Polls, remaining*180/math.Pi)
		}

		readStart := clk.Time()
		// Bound the read by the settle's own deadline, so one poll cannot
		// spend several frame timeouts inside the transport's retry loop.
		pollCtx, cancel := context.WithDeadline(ctx, deadline)
		cur, err := read(pollCtx)
		cancel()
		if err != nil {
			return res, err
		}
		if len(cur) < len(req.Target) {
			return res, fmt.Errorf("short feedback (got %d joints, want %d)", len(cur), len(req.Target))
		}
		now := clk.Time()
		res.Polls++
		res.Positions = cur
		res.Elapsed = now.Sub(start)
		if cost := now.Sub(readStart); cost > res.SlowestRead {
			res.SlowestRead = cost
		}

		samples = append(samples, settleSample{at: now, pos: cur})
		var ref settleSample
		var haveRef bool
		samples, ref, haveRef = stallReference(samples, now.Add(-plan.Window))

		if MaxTravel(cur, req.Target, req.Mask) <= settleTolRad {
			res.Outcome = SettleArrived
			return res, nil
		}
		if res.Elapsed < plan.Grace {
			continue
		}
		if !haveRef || MaxTravel(cur, ref.pos, req.Mask) > StallRad {
			continue
		}

		moved := MaxTravel(req.Start, cur, req.Mask)
		remaining := MaxTravel(cur, req.Target, req.Mask)
		if req.RequireMotion && moved <= StallRad && remaining > 2*settleTolRad {
			return res, fmt.Errorf("%w: still %.1f deg from the target after %v; "+
				"check that torque is enabled, that the workspace is clear, and the link's health via the comms_health command",
				ErrArmDidNotMove, remaining*180/math.Pi, res.Elapsed.Round(time.Millisecond))
		}
		res.Outcome = SettleStopped
		return res, nil
	}
}

// stallReference drops the samples older than the window and returns the
// newest remaining one at or before the cutoff: the sample the stall check
// compares the current position against. samples must not be empty.
//
// It reports false while every sample is newer than the cutoff, which happens
// when one read takes longer than the whole window. The check then abstains
// for that poll rather than fall back to the immediately previous sample,
// because comparing consecutive samples is exactly the bug this design
// replaces (audit 2.1).
func stallReference(samples []settleSample, cutoff time.Time) ([]settleSample, settleSample, bool) {
	keep := 0
	for i, sm := range samples {
		if sm.at.After(cutoff) {
			break
		}
		keep = i
	}
	samples = samples[keep:]
	if samples[0].at.After(cutoff) {
		return samples, settleSample{}, false
	}
	return samples, samples[0], true
}

// SleepCtx sleeps for d or until ctx ends.
func SleepCtx(ctx context.Context, d time.Duration) error {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}
