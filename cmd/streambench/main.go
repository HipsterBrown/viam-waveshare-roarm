// Command streambench drives MoveThroughJointPositionsStreamed on a RoArm-M3
// arm running under viam-server, for bench task B6: a sine sweep on one joint
// streamed at a fixed rate, optionally interrupted by Stop.
//
//	go run ./cmd/streambench --address <machine>.viam.cloud --api-key-id ... --api-key ... --arm arm
//	go run ./cmd/streambench --address localhost:8080 --insecure --arm arm --stop-after 2.5
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

func main() {
	address := flag.String("address", "", "machine address (app.viam.com address or host:port)")
	keyID := flag.String("api-key-id", "", "API key ID (omit with --insecure for a local server)")
	key := flag.String("api-key", "", "API key")
	insecure := flag.Bool("insecure", false, "plain connection without credentials (local viam-server)")
	armName := flag.String("arm", "arm", "arm component name")
	joint := flag.Int("joint", 1, "joint to sweep, 1-5")
	hz := flag.Float64("hz", 10, "points per second")
	seconds := flag.Float64("seconds", 5, "trajectory length")
	amplitude := flag.Float64("amplitude", 0.3, "sine amplitude in radians around the current position")
	prebatch := flag.Bool("prebatch", false, "send the whole trajectory in one batch instead of pacing one point per batch")
	stopAfter := flag.Float64("stop-after", 0, "call Stop this many seconds into the stream (0 = never)")
	flag.Parse()
	if *address == "" || *joint < 1 || *joint > 5 {
		flag.Usage()
		os.Exit(2)
	}

	ctx := context.Background()
	logger := logging.NewLogger("streambench")
	var opts []client.RobotClientOption
	if *insecure {
		opts = append(opts, client.WithDialOptions(rpc.WithInsecure()))
	} else {
		opts = append(opts, client.WithDialOptions(rpc.WithEntityCredentials(*keyID, rpc.Credentials{
			Type: rpc.CredentialsTypeAPIKey, Payload: *key,
		})))
	}
	machine, err := client.New(ctx, *address, logger, opts...)
	if err != nil {
		log.Fatal(err)
	}
	defer machine.Close(ctx)

	a, err := arm.FromProvider(machine, *armName)
	if err != nil {
		log.Fatal(err)
	}
	start, err := a.JointPositions(ctx, nil)
	if err != nil {
		log.Fatal(err)
	}
	fmt.Printf("start joints: %v\n", start)

	// Sine sweep on one joint around where it is now; point 0 is the current pose so the
	// module's 5 degree start gate does not fire (pass a far --amplitude offset to test it).
	n := int(*seconds * *hz)
	points := make([]arm.TrajectoryPoint, n)
	for i := range points {
		t := float64(i) / *hz
		q := make([]referenceframe.Input, len(start))
		copy(q, start)
		q[*joint-1] += *amplitude * math.Sin(2*math.Pi*t / *seconds)
		points[i] = arm.TrajectoryPoint{Time: time.Duration(t * float64(time.Second)), Positions: q}
	}

	streamCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	batches := make(chan []arm.TrajectoryPoint)
	responses := make(chan arm.Response, n)

	go func() {
		defer close(batches)
		if *prebatch {
			batches <- points
			return
		}
		// One point per batch, released at its own time: what a live producer does.
		t0 := time.Now()
		for _, p := range points {
			if d := time.Until(t0.Add(p.Time)); d > 0 {
				time.Sleep(d)
			}
			select {
			case batches <- []arm.TrajectoryPoint{p}:
			case <-streamCtx.Done():
				return
			}
		}
	}()

	if *stopAfter > 0 {
		go func() {
			time.Sleep(time.Duration(*stopAfter * float64(time.Second)))
			fmt.Println("calling Stop")
			if err := a.Stop(ctx, nil); err != nil {
				fmt.Printf("Stop error: %v\n", err)
			}
		}()
	}

	wall := time.Now()
	err = a.MoveThroughJointPositionsStreamed(streamCtx, batches, responses, nil)
	elapsed := time.Since(wall)
	cancel()
	close(responses)
	acks := 0
	for range responses {
		acks++
	}

	end, endErr := a.JointPositions(ctx, nil)
	fmt.Printf("streamed %d points over %.1fs: wall %v, acks %d, err %v\n", n, *seconds, elapsed.Round(time.Millisecond), acks, err)
	fmt.Printf("end joints: %v (err %v)\n", end, endErr)
	fmt.Println("compare with the module's log line: 'streamed N points over ...: gate ..., late ..., settle ..., wall ...'")
}
