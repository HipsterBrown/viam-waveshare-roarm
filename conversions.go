package waveshareroarm

const (
	// speedUnitsPerDegPerSec is an EMPIRICAL constant pending hardware
	// calibration. Source: a hand-written comment in the initial import
	// ("RoArm speed range is 1-4096, where ~50 deg/sec ≈ 500 units").
	// TODO(calibration): replace with measured per-joint values — see
	// cmd/cli calibrate-speed and docs/firmware-units.md.
	speedUnitsPerDegPerSec  = 10.0
	accelUnitsPerDegPerSec2 = 0.5

	minSpeedUnits, maxSpeedUnits = 1, 4096
	minAccelUnits, maxAccelUnits = 1, 254
)

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
	return clamp(int(degPerSec*speedUnitsPerDegPerSec), minSpeedUnits, maxSpeedUnits)
}

func speedFromUnits(units int) float64 {
	return float64(units) / speedUnitsPerDegPerSec
}

func accelToUnits(degPerSec2 float64) int {
	return clamp(int(degPerSec2*accelUnitsPerDegPerSec2), minAccelUnits, maxAccelUnits)
}

func accelFromUnits(units int) float64 {
	return float64(units) / accelUnitsPerDegPerSec2
}
