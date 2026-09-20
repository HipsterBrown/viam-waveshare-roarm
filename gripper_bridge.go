package waveshareroarm

// Gripper <-> arm DoCommand bridge.
//
// The gripper resolves its arm dependency through the resource framework and
// receives an arm.Arm gRPC client, not a direct reference to *roarmM3. As a
// result every joint-6 interaction has to cross the gRPC boundary via the
// arm's DoCommand. The constants here are the private protocol between the
// two resources; they must stay in lock-step across arm.go (producer),
// gripper.go (consumer), and testutil_test.go (fake).
//
// Speeds cross the bridge in deg/s and accelerations in deg/s^2; the arm side
// converts to firmware units. Gripper speed/acc defaults live in
// conversions.go with every other physical-unit constant.
const (
	cmdGetGripperRad = "get_gripper_rad"
	cmdSetGripperRad = "set_gripper_rad"
	cmdStopGripper   = "stop_gripper"

	keyRad   = "rad"
	keySpeed = "speed" // deg/s
	keyAcc   = "acc"   // deg/s^2
	keyWait  = "wait"  // bool, default true: block until joint 6 settles

	// noFeedbackMarker is the substring the gripper looks for in a bridge
	// error to recognise "this transport cannot read positions" after the
	// error has crossed gRPC as plain text. errNoFeedback embeds it.
	noFeedbackMarker = "transport returns no feedback"
)
