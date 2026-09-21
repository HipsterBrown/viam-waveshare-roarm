package roarm

// DoCommand vocabulary: the gripper <-> arm bridge, plus the arm's own
// operator-facing commands.
//
// The gripper resolves its arm dependency through the resource framework and
// receives an arm.Arm gRPC client, not a direct reference to *roarmM3. As a
// result every joint-6 interaction has to cross the gRPC boundary via the
// arm's DoCommand. The constants here are the private protocol between the
// two resources; they must stay in lock-step across components/arm/arm.go (producer),
// components/gripper/gripper.go (consumer), and internal/testfake (fake).
//
// CmdCommsHealth is not part of that bridge: it is an operator command on the
// arm, kept here so every DoCommand name in the module has one home.
//
// Speeds cross the bridge in deg/s and accelerations in deg/s^2; the arm side
// converts to firmware units. Gripper speed/acc defaults live in
// conversions.go with every other physical-unit constant.
const (
	CmdGetGripperRad = "get_gripper_rad"
	CmdSetGripperRad = "set_gripper_rad"
	CmdStopGripper   = "stop_gripper"

	// CmdCommsHealth reports the link health counters (see health.go) and,
	// with {"reset": true}, zeroes them for a clean bench measurement.
	CmdCommsHealth = "comms_health"

	KeyRad   = "rad"
	KeySpeed = "speed" // deg/s
	KeyAcc   = "acc"   // deg/s^2
	KeyWait  = "wait"  // bool, default true: block until joint 6 settles

	// KeyWaitAtEnd is the arm-move spelling of KeyWait, and the one the RDK
	// itself sends: the builtin motion service's teleop executor calls
	// MoveThroughJointPositions with {"waitAtEnd": false, "interpolate": false}
	// on every tick (rdk services/motion/builtin/teleop.go). The xArm module
	// uses the same key. KeyWait stays accepted as an alias so this module's
	// own gripper vocabulary still reads the same on the arm.
	KeyWaitAtEnd = "waitAtEnd"

	// KeyInterpolate is a bool, default true: whether a multi-waypoint call is
	// a PATH to trace or just a TARGET to reach. See InterpolateArg.
	KeyInterpolate = "interpolate"

	// KeyRequireMotion is a bool, default true: whether the arm's settle should
	// treat "joint 6 never left where it started" as an error. Grab sends
	// false because closing onto an object legitimately stops the jaw early,
	// and a jaw that meets the object immediately has not moved at all. The
	// settle's own tolerances would forgive most of those, but not one where
	// the object sits at the open limit, and a successful grab must never be
	// reported as a failure.
	KeyRequireMotion = "require_motion"

	// NoFeedbackMarker is the substring the gripper looks for in a bridge
	// error to recognise "this transport cannot read positions" after the
	// error has crossed gRPC as plain text. ErrNoFeedback embeds it.
	NoFeedbackMarker = "transport returns no feedback"
)

// WaitArg reads the optional "waitAtEnd" (RDK/xArm spelling) or "wait" (this
// module's gripper spelling) key out of a DoCommand or `extra` map. It
// defaults to true: blocking until the arm settles is the guarantee every
// existing caller already has, so giving it up is opt-in. "waitAtEnd" wins if
// both are present.
func WaitArg(m map[string]interface{}) bool {
	if w, ok := m[KeyWaitAtEnd].(bool); ok {
		return w
	}
	if w, ok := m[KeyWait].(bool); ok {
		return w
	}
	return true
}

// InterpolateArg reads the optional "interpolate" key, default true.
//
// True means the waypoints are a path to be traced, so each one is a place the
// arm must actually reach. False means they are only a route to the last one,
// and the caller does not care which way the arm gets there.
//
// It matters because this arm has exactly one motion primitive: write a goal,
// and the firmware interpolates toward it on-device. A second write supersedes
// the first outright. So a waypoint is only physically distinguishable if the
// arm is given time to arrive at it -- and time to arrive is precisely what
// WaitArg=false is asking us not to spend. Interpolate is what separates the
// two requests: with interpolate=false the trajectory is collapsed to its
// endpoint and written once, instead of writing N goals that supersede each
// other within a millisecond of bus time and only pretend to trace a path.
func InterpolateArg(m map[string]interface{}) bool {
	if v, ok := m[KeyInterpolate].(bool); ok {
		return v
	}
	return true
}
