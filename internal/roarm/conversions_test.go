package roarm

import (
	"math"
	"testing"
)

func TestStepsPerDegreeIsServoResolution(t *testing.T) {
	if math.Abs(stepsPerDegree-11.3778) > 0.001 {
		t.Fatalf("stepsPerDegree = %v, want 4096/360", stepsPerDegree)
	}
	if math.Abs(accUnitDegsPerSecSq-8.789) > 0.001 {
		t.Fatalf("accUnitDegsPerSecSq = %v, want 100 steps/s^2 in degrees", accUnitDegsPerSecSq)
	}
}

func TestSpeedRoundTrip(t *testing.T) {
	// One firmware unit is 1/11.38 deg/s, so the round trip is exact to half a unit.
	for _, want := range []float64{10, 50, 100, 180} {
		got := SpeedFromUnits(SpeedToUnits(want))
		if math.Abs(got-want) > 0.5/stepsPerDegree {
			t.Fatalf("round trip for %.1f: got %.4f", want, got)
		}
	}
}

func TestSpeedClamp(t *testing.T) {
	if SpeedToUnits(0) != minSpeedUnits {
		t.Fatalf("low clamp: got %d want %d", SpeedToUnits(0), minSpeedUnits)
	}
	if minSpeedUnits != 34 { // round(3 deg/s * 11.38)
		t.Fatalf("minSpeedUnits = %d, want 34", minSpeedUnits)
	}
	if SpeedToUnits(1e9) != maxSpeedUnits {
		t.Fatal("high clamp")
	}
	if SpeedToUnits(50) != 569 {
		t.Fatalf("50 deg/s = %d units, want 569", SpeedToUnits(50))
	}
}

func TestAccelRoundTrip(t *testing.T) {
	// One acceleration unit is ~8.8 deg/s^2, so the round trip is exact to half a unit.
	for _, want := range []float64{10, 100, 500} {
		got := AccelFromUnits(AccelToUnits(want))
		if math.Abs(got-want) > accUnitDegsPerSecSq/2 {
			t.Fatalf("round trip for %.1f: got %.3f", want, got)
		}
	}
}

func TestAccelClamp(t *testing.T) {
	if AccelToUnits(0) != minAccelUnits {
		t.Fatal("low clamp")
	}
	if AccelToUnits(1e9) != maxAccelUnits {
		t.Fatal("high clamp")
	}
	if AccelToUnits(100) != 11 {
		t.Fatalf("100 deg/s^2 = %d units, want 11", AccelToUnits(100))
	}
}

func TestNoConversionEmitsTheMaximumSentinel(t *testing.T) {
	// 0 means MAXIMUM to the firmware for both fields; the clamps must never produce it.
	if SpeedToUnits(-5) == 0 || AccelToUnits(-5) == 0 {
		t.Fatal("conversion emitted 0")
	}
}
