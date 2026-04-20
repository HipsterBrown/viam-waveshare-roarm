package waveshareroarm

import (
	"testing"
	"time"
)

func TestArmValidateRejectsUnknownBaudrate(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 1152000}
	_, _, err := cfg.Validate("arms.0")
	if err == nil {
		t.Fatal("expected error for bad baudrate")
	}
}

func TestArmValidateAcceptsKnownBaudrate(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 115200}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestArmValidateAcceptsZeroBaudrate(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 0}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestArmConfigAcceptsSplitTimeouts(t *testing.T) {
	cfg := &RoArmM3Config{
		Host:          "1.2.3.4",
		HTTPTimeout:   Duration(5 * time.Second),
		SerialTimeout: Duration(500 * time.Millisecond),
	}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestGripperValidateRequiresArmDep(t *testing.T) {
	cfg := &RoArmGripperConfig{}
	deps, _, err := cfg.Validate("grippers.0")
	if err == nil {
		t.Fatal("expected error when arm is unset")
	}
	if len(deps) != 0 {
		t.Fatal("expected no deps when arm is unset")
	}
}

func TestGripperValidateReturnsArmAsDep(t *testing.T) {
	cfg := &RoArmGripperConfig{Arm: "my-arm"}
	deps, _, err := cfg.Validate("grippers.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(deps) != 1 || deps[0] != "my-arm" {
		t.Fatalf("expected [my-arm], got %v", deps)
	}
}
