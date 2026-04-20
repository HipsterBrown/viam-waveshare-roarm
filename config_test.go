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
