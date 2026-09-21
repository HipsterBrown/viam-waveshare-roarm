package roarm

import "math"

// The firmware's spd and acc fields are ST3215 servo register values: spd is
// steps per second on a 4096-step-per-revolution encoder, acc is in units of
// 100 steps/s^2. For both, 0 means MAXIMUM, so the clamps never emit 0.
// Source: waveshareteam/waveshare_roarm_sdk (doc/roarm_m3_en.md, roarm.py).
const (
	stepsPerDegree      = 4096.0 / 360.0
	accUnitDegsPerSecSq = 100.0 / stepsPerDegree

	MinSpeedDegsPerSec   = 3.0
	MaxSpeedDegsPerSec   = 180.0
	MinAccelDegsPerSecSq = 10.0
	MaxAccelDegsPerSecSq = 500.0

	maxSpeedUnits                = 4096
	minAccelUnits, maxAccelUnits = 1, 254

	// Motion defaults, in physical units; converted where they are sent.
	DefaultSpeedDegsPerSec        = 50.0
	DefaultAccelDegsPerSecSq      = 100.0
	DefaultGripperSpeedDegsPerSec = 50.0
	DefaultGripperAccDegsPerSecSq = 100.0
	// StopSpeedDegsPerSec is the gentle speed Stop re-sends the current
	// position at, for both the arm and the gripper.
	StopSpeedDegsPerSec = 10.0
)

// minSpeedUnits is the firmware value of MinSpeedDegsPerSec (34).
var minSpeedUnits = int(math.Round(MinSpeedDegsPerSec * stepsPerDegree))

func clamp(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func SpeedToUnits(degPerSec float64) int {
	return clamp(int(math.Round(degPerSec*stepsPerDegree)), minSpeedUnits, maxSpeedUnits)
}

func SpeedFromUnits(units int) float64 { return float64(units) / stepsPerDegree }

func AccelToUnits(degPerSec2 float64) int {
	return clamp(int(math.Round(degPerSec2/accUnitDegsPerSecSq)), minAccelUnits, maxAccelUnits)
}

func AccelFromUnits(units int) float64 { return float64(units) * accUnitDegsPerSecSq }
