package roarm

import (
	"math"
	"strings"
	"testing"

	rdkarm "go.viam.com/rdk/components/arm"
)

func rads(deg float64) float64 { return deg * math.Pi / 180 }

func TestResolveMoveProfile(t *testing.T) {
	const defSpeed, defAccel = 50.0, 100.0
	for _, tc := range []struct {
		name               string
		opts               *rdkarm.MoveOptions
		wantSpeed, wantAcc float64
	}{
		{"nil options keep the configured profile", nil, defSpeed, defAccel},
		// rdk's moveOptionsFromProtobuf writes DegToRad(0) for an absent proto
		// field, so a caller who sets only acceleration arrives with
		// MaxVelRads == 0. Clamping that into [3, 180] would run the move at
		// 3 deg/s, a 17x unrequested slowdown.
		{"zero means unset, not minimum", &rdkarm.MoveOptions{MaxAccRads: rads(200)}, defSpeed, 200},
		{"negative means unset", &rdkarm.MoveOptions{MaxVelRads: rads(-10)}, defSpeed, defAccel},
		{"both scalars honored", &rdkarm.MoveOptions{MaxVelRads: rads(20), MaxAccRads: rads(40)}, 20, 40},
		{"speed clamps to the validated range", &rdkarm.MoveOptions{MaxVelRads: rads(500)}, 180, defAccel},
		{"speed clamps at the bottom", &rdkarm.MoveOptions{MaxVelRads: rads(0.5)}, 3, defAccel},
		{"acceleration clamps to the validated range", &rdkarm.MoveOptions{MaxAccRads: rads(5000)}, defSpeed, 500},
		// The firmware's T:102 carries one spd and one acc for every joint, so
		// a per-joint request reduces to its tightest entry: the whole move
		// slows to the most restrictive joint rather than any joint exceeding
		// its cap.
		{"per-joint slices reduce to their minimum", &rdkarm.MoveOptions{
			MaxVelRadsJoints: []float64{rads(60), rads(15), rads(90), rads(45), rads(30)},
		}, 15, defAccel},
		{"an all-zero slice is unset", &rdkarm.MoveOptions{
			MaxVelRadsJoints: []float64{0, 0, 0, 0, 0},
		}, defSpeed, defAccel},
		// arm.proto documents the per-joint field as making the scalar ignored.
		{"a per-joint slice overrides the scalar", &rdkarm.MoveOptions{
			MaxVelRads:       rads(90),
			MaxVelRadsJoints: []float64{rads(10), rads(10), rads(10), rads(10), rads(10)},
		}, 10, defAccel},
	} {
		t.Run(tc.name, func(t *testing.T) {
			speed, acc, err := ResolveMoveProfile(tc.opts, 5, defSpeed, defAccel, nil)
			if err != nil {
				t.Fatal(err)
			}
			if math.Abs(speed-tc.wantSpeed) > 1e-6 || math.Abs(acc-tc.wantAcc) > 1e-6 {
				t.Fatalf("got %.3f deg/s, %.3f deg/s^2; want %.3f, %.3f", speed, acc, tc.wantSpeed, tc.wantAcc)
			}
		})
	}
}

// A wrong-length slice is the caller's bug and silently using part of it would
// hide it; the error names both counts.
func TestResolveMoveProfileRejectsAWrongLengthSlice(t *testing.T) {
	_, _, err := ResolveMoveProfile(&rdkarm.MoveOptions{MaxAccRadsJoints: []float64{1, 2}}, 5, 50, 100, nil)
	if err == nil {
		t.Fatal("expected an error for a 2-entry slice on a 5-joint arm")
	}
	for _, want := range []string{"2", "5"} {
		if !strings.Contains(err.Error(), want) {
			t.Fatalf("the error should name both counts: %v", err)
		}
	}
}

// Honoring a TCP speed cap needs a Jacobian, which this module does not have.
// Ignoring it silently would be worse than saying so.
func TestResolveMoveProfileIgnoresTCPSpeed(t *testing.T) {
	v := 0.05
	speed, acc, err := ResolveMoveProfile(&rdkarm.MoveOptions{MaxTCPSpeedMPerSec: &v}, 5, 50, 100, nil)
	if err != nil || speed != 50 || acc != 100 {
		t.Fatalf("MaxTCPSpeedMPerSec should be ignored, not applied: %v %v %v", speed, acc, err)
	}
}
