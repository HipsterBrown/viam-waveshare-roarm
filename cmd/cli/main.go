// Command roarm-cli is a bench tool that drives the RoArm-M3 controller
// directly, bypassing viam-server. It exists for the hardware checks in the
// module's release checklist (unit verification, settle latency).
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"sort"
	"strconv"
	"time"

	"go.viam.com/rdk/logging"

	"waveshareroarm/internal/roarm"
)

func usage() {
	fmt.Fprintln(os.Stderr, `usage: roarm-cli [--host=X | --port=Y] <subcommand> [args...]
  ping                                  send a feedback request, print OK
  feedback                              print one feedback frame
  move <joint> <rad> [deg_per_sec] [deg_per_sec2]
                                        move one joint (1-6, software frame); no limit checks
  gripper <rad>                         move joint 6 (software frame)
  torque <on|off>                       enable or disable servo torque
  health [reset]                        print the link health counters, optionally zeroing them
  time-move <joint> <from_rad> <to_rad> <deg_per_sec> [deg_per_sec2]
                                        move to from_rad, settle, then move to to_rad while
                                        polling feedback; print elapsed, implied deg/s and
                                        each settle's outcome. Acceleration defaults to the
                                        module default; pass a low one to exercise the ramp.`)
	os.Exit(2)
}

func main() {
	host := flag.String("host", "", "RoArm HTTP host (e.g. 192.168.4.1)")
	port := flag.String("port", "", "Serial port (e.g. /dev/tty.usbserial-xxx)")
	baud := flag.Int("baudrate", 115200, "Serial baud rate")
	flag.Parse()
	if flag.NArg() < 1 {
		usage()
	}

	logger := logging.NewLogger("roarm-cli")
	ctrl, err := roarm.NewController(&roarm.Config{
		Host: *host, Port: *port, Baudrate: *baud, Logger: logger,
	})
	if err != nil {
		log.Fatal(err)
	}
	ctx := context.Background()
	defer ctrl.Close(ctx)

	args := flag.Args()
	switch args[0] {
	case "ping":
		if _, err := ctrl.GetFeedback(ctx); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "feedback":
		fb, err := ctrl.GetFeedback(ctx)
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("%+v\n", fb)
	case "move":
		if len(args) < 3 {
			usage()
		}
		joint := atoi(args[1])
		rad := atof(args[2])
		speed, acc := roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec), roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq)
		if len(args) > 3 {
			speed = roarm.SpeedToUnits(atof(args[3]))
		}
		if len(args) > 4 {
			acc = roarm.AccelToUnits(atof(args[4]))
		}
		if err := ctrl.SetJointRadian(ctx, joint, rad, speed, acc); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "gripper":
		if len(args) < 2 {
			usage()
		}
		if err := ctrl.SetJointRadian(ctx, 6, atof(args[1]), roarm.SpeedToUnits(roarm.DefaultSpeedDegsPerSec), roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq)); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "torque":
		if len(args) < 2 {
			usage()
		}
		on := args[1] == "on" || args[1] == "true" || args[1] == "1"
		if err := ctrl.SetTorque(ctx, on); err != nil {
			log.Fatal(err)
		}
		fmt.Printf("torque %v\n", map[bool]string{true: "on", false: "off"}[on])
	case "health":
		h := ctrl.Health().Map()
		keys := make([]string, 0, len(h))
		for k := range h {
			keys = append(keys, k)
		}
		sort.Strings(keys)
		for _, k := range keys {
			fmt.Printf("%-20s %v\n", k, h[k])
		}
		if len(args) > 1 && args[1] == "reset" {
			ctrl.ResetHealth()
			fmt.Println("(counters reset)")
		}
	case "time-move":
		if len(args) < 5 {
			usage()
		}
		accel := 0.0 // 0 means the module default
		if len(args) > 5 {
			accel = atof(args[5])
		}
		timeMove(ctx, ctrl, atoi(args[1]), atof(args[2]), atof(args[3]), atof(args[4]), accel)
	default:
		usage()
	}
}

// timeMove is bench task B1/B2: it parks the joint at from, then commands
// `to` at degPerSec and polls feedback every 50 ms until the joint is within
// tolerance or stops moving, printing the elapsed time and the implied speed.
// timeMove parks the joint at from, then commands to at the given profile and
// reports how long the settle actually took. degPerSecSq is optional: it
// matters because the settle's window, grace and deadline all derive from the
// acceleration, and a low one is the case that used to report a move complete
// the instant it started.
func timeMove(ctx context.Context, ctrl *roarm.Controller, joint int, from, to, degPerSec, degPerSecSq float64) {
	if degPerSecSq <= 0 {
		degPerSecSq = roarm.DefaultAccelDegsPerSecSq
	}
	speed := roarm.SpeedToUnits(degPerSec)
	acc := roarm.AccelToUnits(degPerSecSq)
	mask := make([]bool, 6)
	mask[joint-1] = true

	park := func(target float64) []float64 {
		// Read before the write: Start must be a measured pose, and building
		// the target from it keeps the other five joints where they are
		// (a zero-filled target commanded them all to 0).
		start, err := ctrl.GetJointRadians(ctx)
		if err != nil {
			log.Fatal(err)
		}
		if err := ctrl.SetJointRadian(ctx, joint, target, speed, acc); err != nil {
			log.Fatal(err)
		}
		t := append([]float64(nil), start...)
		t[joint-1] = target
		res, err := ctrl.WaitUntilSettled(ctx, roarm.SettleRequest{
			Target: t, Start: start, Mask: mask, SpeedUnits: speed, AccUnits: acc, RequireMotion: true,
		})
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("  settle: %v in %v (%d polls, %d retries, slowest read %v)\n",
			res.Outcome, res.Elapsed.Round(time.Millisecond), res.Polls, res.Retries, res.SlowestRead.Round(time.Millisecond))
		return res.Positions
	}
	park(from)
	time.Sleep(300 * time.Millisecond)

	startedAt := time.Now()
	pos := park(to)
	elapsed := time.Since(startedAt)
	travelDeg := math.Abs(to-from) * 180 / math.Pi
	fmt.Printf("joint %d: %.1f deg in %v -> %.1f deg/s (commanded %.1f deg/s at %.0f deg/s^2; %d/%d units); final %.4f rad\n",
		joint, travelDeg, elapsed.Round(time.Millisecond), travelDeg/elapsed.Seconds(),
		degPerSec, degPerSecSq, speed, acc, pos[joint-1])
	// Health counters live on the Controller, so they only mean anything
	// within one process. This subcommand is the only one that does enough
	// reads for the ratio to be worth printing.
	h := ctrl.Health()
	fmt.Printf("  link: %d frames, %d retries (%.1f%%), %d exhausted, %d invalid, %d stale, %d read timeouts\n",
		h.Frames, h.Retries, h.RetryPct(), h.RetriesExhausted, h.InvalidFrames, h.StaleFrames, h.ReadTimeouts)
}

func atoi(s string) int {
	v, err := strconv.Atoi(s)
	if err != nil {
		log.Fatalf("bad integer %q", s)
	}
	return v
}

func atof(s string) float64 {
	v, err := strconv.ParseFloat(s, 64)
	if err != nil {
		log.Fatalf("bad number %q", s)
	}
	return v
}
