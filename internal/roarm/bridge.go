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

	// KeyRequireMotion is a bool, default true: whether the arm's settle should
	// treat "joint 6 never left where it started" as an error. Grab sends false
	// because closing onto an object legitimately stops the jaw early, and the
	// gripper's grab margin (0.05 rad) is wider than the settle's arrival
	// tolerance, so a successful grab would otherwise look like a failure.
	KeyRequireMotion = "require_motion"

	// NoFeedbackMarker is the substring the gripper looks for in a bridge
	// error to recognise "this transport cannot read positions" after the
	// error has crossed gRPC as plain text. ErrNoFeedback embeds it.
	NoFeedbackMarker = "transport returns no feedback"
)
