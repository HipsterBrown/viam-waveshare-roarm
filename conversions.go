package waveshareroarm

import "math"

// The firmware's spd and acc fields are ST3215 servo register values: spd is
// steps per second on a 4096-step-per-revolution encoder, acc is in units of
// 100 steps/s^2. For both, 0 means MAXIMUM, so the clamps never emit 0.
// Source: waveshareteam/waveshare_roarm_sdk (doc/roarm_m3_en.md, roarm.py).
const (
	stepsPerDegree      = 4096.0 / 360.0
	accUnitDegsPerSecSq = 100.0 / stepsPerDegree

	minSpeedDegsPerSec   = 3.0
	maxSpeedDegsPerSec   = 180.0
	minAccelDegsPerSecSq = 10.0
	maxAccelDegsPerSecSq = 500.0

	maxSpeedUnits                = 4096
	minAccelUnits, maxAccelUnits = 1, 254

	// Motion defaults, in physical units; converted where they are sent.
	defaultSpeedDegsPerSec        = 50.0
	defaultAccelDegsPerSecSq      = 100.0
	defaultGripperSpeedDegsPerSec = 50.0
	defaultGripperAccDegsPerSecSq = 100.0
	// stopSpeedDegsPerSec is the gentle speed Stop re-sends the current
	// position at, for both the arm and the gripper.
	stopSpeedDegsPerSec = 10.0
)

// minSpeedUnits is the firmware value of minSpeedDegsPerSec (34).
var minSpeedUnits = int(math.Round(minSpeedDegsPerSec * stepsPerDegree))

func clamp(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func speedToUnits(degPerSec float64) int {
	return clamp(int(math.Round(degPerSec*stepsPerDegree)), minSpeedUnits, maxSpeedUnits)
}

func speedFromUnits(units int) float64 { return float64(units) / stepsPerDegree }

func accelToUnits(degPerSec2 float64) int {
	return clamp(int(math.Round(degPerSec2/accUnitDegsPerSecSq)), minAccelUnits, maxAccelUnits)
}

func accelFromUnits(units int) float64 { return float64(units) * accUnitDegsPerSecSq }

// Exported for cmd/cli. Everything inside the package uses the lowercase forms.
func SpeedToUnits(degPerSec float64) int  { return speedToUnits(degPerSec) }
func AccelToUnits(degPerSec2 float64) int { return accelToUnits(degPerSec2) }
func DefaultSpeedUnits() int              { return speedToUnits(defaultSpeedDegsPerSec) }
func DefaultAccelUnits() int              { return accelToUnits(defaultAccelDegsPerSecSq) }
