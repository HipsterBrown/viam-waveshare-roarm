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
  time-move <joint> <from_rad> <to_rad> <deg_per_sec>
                                        move to from_rad, settle, then move to to_rad while
                                        polling feedback; print elapsed and implied deg/s`)
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
	case "time-move":
		if len(args) < 5 {
			usage()
		}
		timeMove(ctx, ctrl, atoi(args[1]), atof(args[2]), atof(args[3]), atof(args[4]))
	default:
		usage()
	}
}

// timeMove is bench task B1/B2: it parks the joint at from, then commands
// `to` at degPerSec and polls feedback every 50 ms until the joint is within
// tolerance or stops moving, printing the elapsed time and the implied speed.
func timeMove(ctx context.Context, ctrl *roarm.Controller, joint int, from, to, degPerSec float64) {
	speed := roarm.SpeedToUnits(degPerSec)
	acc := roarm.AccelToUnits(roarm.DefaultAccelDegsPerSecSq)
	mask := make([]bool, 6)
	mask[joint-1] = true

	park := func(target float64) []float64 {
		if err := ctrl.SetJointRadian(ctx, joint, target, speed, acc); err != nil {
			log.Fatal(err)
		}
		t := make([]float64, 6)
		t[joint-1] = target
		req := roarm.SettleRequest{
			Start:         t,
			Target:        t,
			Mask:          mask,
			SpeedUnits:    speed,
			AccUnits:      acc,
			RequireMotion: true,
			Timeout:       15 * time.Second,
		}
		res, err := ctrl.WaitUntilSettled(ctx, req)
		if err != nil {
			log.Fatal(err)
		}
		return res.Positions
	}
	park(from)
	time.Sleep(300 * time.Millisecond)

	startedAt := time.Now()
	pos := park(to)
	elapsed := time.Since(startedAt)
	travelDeg := math.Abs(to-from) * 180 / math.Pi
	fmt.Printf("joint %d: %.1f deg in %v -> %.1f deg/s (commanded %.1f deg/s, %d units); final %.4f rad\n",
		joint, travelDeg, elapsed.Round(time.Millisecond), travelDeg/elapsed.Seconds(), degPerSec, speed, pos[joint-1])
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
