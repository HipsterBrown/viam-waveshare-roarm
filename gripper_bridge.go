package waveshareroarm

// Gripper ↔ arm DoCommand bridge.
//
// The gripper resolves its arm dependency through the resource framework and
// receives an arm.Arm gRPC client, not a direct reference to *roarmM3. As a
// result every joint-6 interaction has to cross the gRPC boundary via the
// arm's DoCommand. The constants here are the private protocol between the
// two resources; they must stay in lock-step across arm.go (producer),
// gripper.go (consumer), and testutil_test.go (fake).
const (
	cmdGetGripperRad = "get_gripper_rad"
	cmdSetGripperRad = "set_gripper_rad"
	cmdStopGripper   = "stop_gripper"

	keyRad   = "rad"
	keySpeed = "speed"
	keyAcc   = "acc"

	defaultGripperSpeed = 500 // internal units, see speedFromUnits
	defaultGripperAcc   = 50  // internal units, see accelFromUnits
	stopGripperSpeed    = 100 // gentle soft stop, matches arm-level Stop
)
