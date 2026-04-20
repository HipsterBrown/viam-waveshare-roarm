package waveshareroarm

import (
	"math"
	"testing"
)

func TestSpeedRoundTrip(t *testing.T) {
	cases := []float64{10, 50, 100, 180}
	for _, want := range cases {
		got := speedFromUnits(speedToUnits(want))
		if math.Abs(got-want) > 0.01 {
			t.Fatalf("round trip for %.1f: got %.3f", want, got)
		}
	}
}

func TestSpeedClamp(t *testing.T) {
	if speedToUnits(0) != minSpeedUnits {
		t.Fatal("low clamp")
	}
	if speedToUnits(1e9) != maxSpeedUnits {
		t.Fatal("high clamp")
	}
}

func TestAccelRoundTrip(t *testing.T) {
	for _, want := range []float64{10, 100, 500} {
		got := accelFromUnits(accelToUnits(want))
		if math.Abs(got-want) > 0.1 {
			t.Fatalf("round trip for %.1f: got %.3f", want, got)
		}
	}
}

func TestAccelClamp(t *testing.T) {
	if accelToUnits(0) != minAccelUnits {
		t.Fatal("low clamp")
	}
	if accelToUnits(1e9) != maxAccelUnits {
		t.Fatal("high clamp")
	}
}
