package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"strconv"

	waveshareroarm "waveshareroarm"

	"go.viam.com/rdk/logging"
)

func main() {
	host := flag.String("host", "", "RoArm HTTP host (e.g. 192.168.1.10)")
	port := flag.String("port", "", "Serial port (e.g. /dev/tty.usbserial-xxx)")
	baud := flag.Int("baudrate", 115200, "Serial baud rate")
	flag.Parse()

	if flag.NArg() < 1 {
		fmt.Fprintln(os.Stderr, "usage: roarm-cli [--host=X | --port=Y] <subcommand> [args...]")
		fmt.Fprintln(os.Stderr, "subcommands: ping, feedback, home, move, gripper")
		os.Exit(2)
	}

	logger := logging.NewLogger("roarm-cli")
	ctrl, err := waveshareroarm.NewRoArmController(&waveshareroarm.RoArmConfig{
		Host: *host, Port: *port, Baudrate: *baud, Logger: logger,
	})
	if err != nil {
		log.Fatal(err)
	}
	defer ctrl.Close(context.Background())

	ctx := context.Background()
	args := flag.Args()
	switch args[0] {
	case "ping":
		if err := ctrl.TestConnection(ctx); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "feedback":
		fb, err := ctrl.GetFeedback(ctx)
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("%+v\n", fb)
	case "home":
		if err := ctrl.MoveToHome(ctx); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "move":
		if len(args) < 3 {
			log.Fatal("usage: move <joint> <rad> [speed] [acc]")
		}
		joint, _ := strconv.Atoi(args[1])
		rad, _ := strconv.ParseFloat(args[2], 64)
		speed, acc := 500, 50
		if len(args) > 3 {
			speed, _ = strconv.Atoi(args[3])
		}
		if len(args) > 4 {
			acc, _ = strconv.Atoi(args[4])
		}
		if err := ctrl.SetJointRadian(ctx, joint, rad, speed, acc); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "gripper":
		if len(args) < 2 {
			log.Fatal("usage: gripper <rad>")
		}
		rad, _ := strconv.ParseFloat(args[1], 64)
		if err := ctrl.SetJointRadian(ctx, 6, rad, 500, 50); err != nil {
			log.Fatal(err)
		}
		fmt.Println("OK")
	case "sweep", "calibrate-speed", "calibrate-accel":
		log.Fatalf("%s: implemented in Phase 7 (hardware calibration)", args[0])
	default:
		log.Fatalf("unknown subcommand: %s", args[0])
	}
}
