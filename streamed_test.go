package waveshareroarm

import (
	"context"
	"errors"
	"math"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

var streamEpoch = time.Date(2026, 9, 20, 12, 0, 0, 0, time.UTC)

// streamTestArm pins the clock to streamEpoch and makes every sleep return
// at once while recording its deadline. fc.Feedback is the arm's position
// for the start gate.
func streamTestArm(t *testing.T, fc *fakeController) (*roarmM3, func() []time.Time) {
	t.Helper()
	r := newTestArm(t, fc)
	var mu sync.Mutex
	var deadlines []time.Time
	r.clock = clock{
		now: func() time.Time { return streamEpoch },
		sleepUntil: func(ctx context.Context, d time.Time) error {
			mu.Lock()
			deadlines = append(deadlines, d)
			mu.Unlock()
			return ctx.Err()
		},
	}
	return r, func() []time.Time {
		mu.Lock()
		defer mu.Unlock()
		return append([]time.Time(nil), deadlines...)
	}
}

func pt(at time.Duration, q float64) arm.TrajectoryPoint {
	return arm.TrajectoryPoint{Time: at, Positions: []referenceframe.Input{q, 0, 0, 0, 0}}
}

// runStream owns both channels the way the rdk server does: feeds batches in
// order, closes the input, drains acks.
func runStream(ctx context.Context, r *roarmM3, batches ...[]arm.TrajectoryPoint) (int, error) {
	in := make(chan []arm.TrajectoryPoint)
	out := make(chan arm.Response)
	go func() {
		defer close(in)
		for _, b := range batches {
			select {
			case in <- b:
			case <-ctx.Done():
				return
			}
		}
	}()
	acks := 0
	drained := make(chan struct{})
	go func() {
		defer close(drained)
		for range out {
			acks++
		}
	}()
	err := r.MoveThroughJointPositionsStreamed(ctx, in, out, nil)
	close(out)
	<-drained
	return acks, err
}

func TestStreamed_OneWritePerPointAtScheduledTimes(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{G: 0.4}}
	r, deadlines := streamTestArm(t, fc)
	acks, err := runStream(context.Background(), r,
		[]arm.TrajectoryPoint{pt(0, 0.01), pt(100*time.Millisecond, 0.02), pt(200*time.Millisecond, 0.03)})
	if err != nil {
		t.Fatal(err)
	}
	if acks != 1 {
		t.Fatalf("acks %d, want 1 per non-empty batch", acks)
	}
	if fc.WriteCount != 3 {
		t.Fatalf("writes %d, want 3 (one per point, no gate move within 5 degrees)", fc.WriteCount)
	}
	if fc.LastRadians[0] != 0.03 || fc.LastRadians[5] != 0.4 {
		t.Fatalf("last target %v: want joint 1 at 0.03 and gripper preserved", fc.LastRadians)
	}
	d := deadlines()
	// Point k is written when point k-1 is due: 0, 0, +100 ms.
	want := []time.Time{streamEpoch, streamEpoch, streamEpoch.Add(100 * time.Millisecond)}
	if len(d) != 3 {
		t.Fatalf("sleeps %v, want %v", d, want)
	}
	for i := range want {
		if !d[i].Equal(want[i]) {
			t.Fatalf("sleep %d at %v, want %v", i, d[i], want[i])
		}
	}
	if fc.SettleCalls != 1 {
		t.Fatalf("settle calls %d, want exactly one at the end", fc.SettleCalls)
	}
}

func TestStreamed_SegmentSpeedMatchesTravelOverTime(t *testing.T) {
	fc := &fakeController{}
	r, _ := streamTestArm(t, fc)
	// 0.1 rad in 100 ms is 1 rad/s = 57.3 deg/s.
	_, err := runStream(context.Background(), r,
		[]arm.TrajectoryPoint{pt(0, 0), pt(100*time.Millisecond, 0.1)})
	if err != nil {
		t.Fatal(err)
	}
	if want := speedToUnits(0.1 * 180 / math.Pi / 0.1); fc.LastSpeed != want {
		t.Fatalf("segment speed %d units, want %d", fc.LastSpeed, want)
	}
}

func TestStreamed_GatesAFarFirstPoint(t *testing.T) {
	fc := &fakeController{Feedback: FeedbackData{B: 1.0}} // 57 degrees from the first point
	r, _ := streamTestArm(t, fc)
	_, err := runStream(context.Background(), r, []arm.TrajectoryPoint{pt(0, 0), pt(50*time.Millisecond, 0.01)})
	if err != nil {
		t.Fatal(err)
	}
	// gate move + 2 points
	if fc.WriteCount != 3 {
		t.Fatalf("writes %d, want 3 (gate + 2 points)", fc.WriteCount)
	}
	// gate settle + final settle
	if fc.SettleCalls != 2 {
		t.Fatalf("settle calls %d, want 2", fc.SettleCalls)
	}
}

func TestStreamed_RejectsBadTimes(t *testing.T) {
	r, _ := streamTestArm(t, &fakeController{})
	if _, err := runStream(context.Background(), r, []arm.TrajectoryPoint{pt(10*time.Millisecond, 0)}); err == nil {
		t.Fatal("first point must be at time 0")
	}
	r, _ = streamTestArm(t, &fakeController{})
	if _, err := runStream(context.Background(), r, []arm.TrajectoryPoint{pt(0, 0), pt(0, 0.01)}); err == nil {
		t.Fatal("times must strictly increase")
	}
}

func TestStreamed_EmptyStreamIsANoOp(t *testing.T) {
	fc := &fakeController{}
	r, _ := streamTestArm(t, fc)
	acks, err := runStream(context.Background(), r)
	if err != nil || acks != 0 || fc.WriteCount != 0 || fc.SettleCalls != 0 {
		t.Fatalf("err=%v acks=%d writes=%d settles=%d", err, acks, fc.WriteCount, fc.SettleCalls)
	}
}

func TestStreamed_EmptyBatchIsNotAcked(t *testing.T) {
	r, _ := streamTestArm(t, &fakeController{})
	acks, err := runStream(context.Background(), r, []arm.TrajectoryPoint{}, []arm.TrajectoryPoint{pt(0, 0)})
	if err != nil || acks != 1 {
		t.Fatalf("err=%v acks=%d, want 1", err, acks)
	}
}

func TestStreamed_StopCancelsAndStopsWriting(t *testing.T) {
	fc := &fakeController{}
	r, _ := streamTestArm(t, fc)
	ctx, cancel := context.WithCancel(context.Background())
	in := make(chan []arm.TrajectoryPoint)
	out := make(chan arm.Response, 10)
	errCh := make(chan error, 1)
	go func() { errCh <- r.MoveThroughJointPositionsStreamed(ctx, in, out, nil) }()
	in <- []arm.TrajectoryPoint{pt(0, 0.01)}
	<-out
	cancel() // what Stop's opMgr.CancelRunning does to the op context
	select {
	case err := <-errCh:
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("want context.Canceled, got %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("stream did not return after cancel while waiting on the producer")
	}
	if fc.WriteCount != 1 {
		t.Fatalf("writes after cancel: %d", fc.WriteCount)
	}
}

func TestStreamed_ClampsToModelLimits(t *testing.T) {
	fc := &fakeController{}
	r, _ := streamTestArm(t, fc)
	_, err := runStream(context.Background(), r, []arm.TrajectoryPoint{
		{Time: 0, Positions: []referenceframe.Input{0, 0, 0, 0, 0}},
		{Time: 50 * time.Millisecond, Positions: []referenceframe.Input{10, 0, 0, 0, 0}},
	})
	if err != nil {
		t.Fatal(err)
	}
	// The model's joint 1 limit is 180.0004 degrees, so compare to pi with a
	// tolerance, as arm_test.go's clamp tests do.
	if fc.LastRadians[0] > math.Pi+1e-4 {
		t.Fatalf("joint 1 written at %v, above the model limit", fc.LastRadians[0])
	}
}

// A sparse trajectory's last segment can be long; the final settle must wait
// for it rather than the 500 ms floor.
func TestStreamed_FinalSettleWaitsForTheLastSegment(t *testing.T) {
	fc := &fakeController{}
	r, _ := streamTestArm(t, fc)
	// 1 rad in 2 s: the write goes out at t=0 and the arm needs ~2 s.
	_, err := runStream(context.Background(), r,
		[]arm.TrajectoryPoint{pt(0, 0), pt(2*time.Second, 1.0)})
	if err != nil {
		t.Fatal(err)
	}
	if fc.LastSettleTimeout < 3500*time.Millisecond {
		t.Fatalf("final settle timeout %v; want about 2x the 2 s segment", fc.LastSettleTimeout)
	}
}
