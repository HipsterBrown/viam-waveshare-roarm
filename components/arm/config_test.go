package arm

import (
	"strings"
	"testing"
	"time"

	"waveshareroarm/internal/roarm"
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
		HTTPTimeout:   roarm.Duration(5 * time.Second),
		SerialTimeout: roarm.Duration(500 * time.Millisecond),
	}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestArmValidateRejectsBothHostAndPort(t *testing.T) {
	cfg := &RoArmM3Config{Host: "1.2.3.4", Port: "/dev/ttyUSB0"}
	_, _, err := cfg.Validate("arms.0")
	if err == nil {
		t.Fatal("expected error")
	}
}

func TestArmValidateRejectsNeither(t *testing.T) {
	cfg := &RoArmM3Config{}
	_, _, err := cfg.Validate("arms.0")
	if err == nil {
		t.Fatal("expected error")
	}
}

func TestArmValidateAcceptsHTTPConfig(t *testing.T) {
	cfg := &RoArmM3Config{Host: "1.2.3.4"}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
}

func TestArmValidateErrorIncludesPath(t *testing.T) {
	cfg := &RoArmM3Config{}
	_, _, err := cfg.Validate("my.path.0")
	if err == nil {
		t.Fatal("expected error")
	}
	if !strings.Contains(err.Error(), "my.path.0") {
		t.Fatalf("expected path in error, got: %v", err)
	}
}

func TestArmValidateRejectsOutOfRangeSpeed(t *testing.T) {
	for _, v := range []float32{1, 200} {
		cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", SpeedDegsPerSec: v}
		if _, _, err := cfg.Validate("arms.0"); err == nil || !strings.Contains(err.Error(), "speed_degs_per_sec") {
			t.Fatalf("speed %v: expected a speed_degs_per_sec error, got %v", v, err)
		}
	}
}

func TestArmValidateRejectsOutOfRangeAcceleration(t *testing.T) {
	for _, v := range []float32{5, 600} {
		cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", AccelerationDegsPerSec: v}
		if _, _, err := cfg.Validate("arms.0"); err == nil || !strings.Contains(err.Error(), "acceleration_degs_per_sec_per_sec") {
			t.Fatalf("accel %v: expected an acceleration error, got %v", v, err)
		}
	}
}

func TestArmValidateAcceptsZeroMotionParams(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0"}
	if _, _, err := cfg.Validate("arms.0"); err != nil {
		t.Fatalf("zero means default: %v", err)
	}
}
