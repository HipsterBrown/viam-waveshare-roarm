package roarm

import (
	"fmt"
	"math"

	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
)

// ResolveMoveProfile turns an arm.MoveOptions into the speed (deg/s) and
// acceleration (deg/s^2) a move will use, clamped to the range the config
// validates. joints is the arm's degrees of freedom, needed to check the
// per-joint slices. logger may be nil.
//
// Lives here rather than in a new internal/motion package: components/arm and
// components/simulated both import rdk's services/motion unaliased as
// `motion`, and MoveToPosition needs motion.MoveReq in the same file, so that
// name would force an alias into every file that touches either.
func ResolveMoveProfile(o *rdkarm.MoveOptions, joints int, defSpeedDegs, defAccelDegs float64, logger logging.Logger) (float64, float64, error) {
	speed, acc := defSpeedDegs, defAccelDegs
	if o == nil {
		return speed, acc, nil
	}

	// Zero or negative means unset for both scalars. This is load-bearing, not
	// defensive: rdk's moveOptionsFromProtobuf writes DegToRad(0) for an
	// absent proto field, so a caller who sets only acceleration arrives here
	// with MaxVelRads == 0, and clamping that into the configured range would
	// run the move at MinSpeedDegsPerSec, a 17x unrequested slowdown.
	if v, err := resolveLimit(o.MaxVelRads, o.MaxVelRadsJoints, joints, "max_vel_rads"); err != nil {
		return 0, 0, err
	} else if v > 0 {
		speed = math.Max(MinSpeedDegsPerSec, math.Min(MaxSpeedDegsPerSec, v*180/math.Pi))
	}
	if v, err := resolveLimit(o.MaxAccRads, o.MaxAccRadsJoints, joints, "max_acc_rads"); err != nil {
		return 0, 0, err
	} else if v > 0 {
		acc = math.Max(MinAccelDegsPerSecSq, math.Min(MaxAccelDegsPerSecSq, v*180/math.Pi))
	}

	if o.MaxTCPSpeedMPerSec != nil && *o.MaxTCPSpeedMPerSec > 0 && logger != nil {
		logger.Debugf("ignoring max_tcp_speed_m_per_sec=%g: honoring a tool-frame speed cap needs a Jacobian, "+
			"which this module does not compute; the move uses the joint-space profile instead",
			*o.MaxTCPSpeedMPerSec)
	}
	return speed, acc, nil
}

// resolveLimit reduces a scalar-plus-per-joint pair to one value. A per-joint
// slice wins over the scalar (arm.proto documents the scalar as ignored when
// the per-joint field is set), and reduces to its smallest positive entry: the
// firmware's T:102 carries one spd and one acc for every joint, so the whole
// move must slow to the most restrictive joint rather than let any joint
// exceed its cap. Returns 0 for "unset".
func resolveLimit(scalar float64, perJoint []float64, joints int, name string) (float64, error) {
	if len(perJoint) == 0 {
		return scalar, nil
	}
	if len(perJoint) != joints {
		return 0, fmt.Errorf("%s_joints has %d entries but this arm has %d joints", name, len(perJoint), joints)
	}
	minPositive := 0.0
	for _, v := range perJoint {
		if v > 0 && (minPositive == 0 || v < minPositive) {
			minPositive = v
		}
	}
	// An all-zero slice is unset, and falls back to the scalar, which is
	// itself zero-means-unset.
	if minPositive == 0 {
		return scalar, nil
	}
	return minPositive, nil
}
