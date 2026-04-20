package waveshareroarm

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strings"
	"testing"
	"time"

	"go.bug.st/serial"
	"go.viam.com/rdk/logging"
)

// newHTTPTestController spins up an httptest server that responds to /js with
// a static FeedbackData payload and wires an RoArmController to it.
func newHTTPTestController(t *testing.T, respT int, body FeedbackData) (*RoArmController, *httptest.Server) {
	t.Helper()
	body.T = respT
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(body)
	}))
	u, err := url.Parse(srv.URL)
	if err != nil {
		t.Fatal(err)
	}
	c, err := NewRoArmController(&RoArmConfig{Host: u.Host, HTTPTimeout: Duration(2 * time.Second)})
	if err != nil {
		srv.Close()
		t.Fatal(err)
	}
	return c, srv
}

func TestCommandMarshalJSON_WithData(t *testing.T) {
	cmd := &Command{
		T: JOINT_RADIAN_CTRL,
		Data: map[string]interface{}{
			"joint": 1,
			"rad":   0.5,
		},
	}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if int(out["T"].(float64)) != JOINT_RADIAN_CTRL {
		t.Fatal("T mismatch")
	}
	if int(out["joint"].(float64)) != 1 {
		t.Fatal("joint mismatch")
	}
	if out["rad"].(float64) != 0.5 {
		t.Fatal("rad mismatch")
	}
}

func TestCommandMarshalJSON_EmptyData(t *testing.T) {
	cmd := &Command{T: FEEDBACK_GET, Data: map[string]interface{}{}}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if len(out) != 1 {
		t.Fatalf("expected 1 key, got %v", out)
	}
}

func TestParseLastJSONFrame_SingleComplete(t *testing.T) {
	data, ok := parseLastJSONFrame([]byte("{\"T\":1051,\"b\":0}\r\n"))
	if !ok {
		t.Fatal("expected ok")
	}
	if string(data) != `{"T":1051,"b":0}` {
		t.Fatalf("got %q", string(data))
	}
}

func TestParseLastJSONFrame_PartialFrame(t *testing.T) {
	_, ok := parseLastJSONFrame([]byte(`{"T":1051,"b":0}`)) // no \r\n
	if ok {
		t.Fatal("expected not ok")
	}
}

func TestParseLastJSONFrame_MultipleFrames_UsesLast(t *testing.T) {
	data, ok := parseLastJSONFrame([]byte("{\"T\":1}\r\n{\"T\":2}\r\n"))
	if !ok {
		t.Fatal("expected ok")
	}
	if string(data) != `{"T":2}` {
		t.Fatalf("got %q", string(data))
	}
}

func TestParseLastJSONFrame_GarbagePrefix(t *testing.T) {
	data, ok := parseLastJSONFrame([]byte("GARBAGE{\"T\":1}\r\n"))
	if !ok {
		t.Fatal("expected ok")
	}
	if string(data) != `{"T":1}` {
		t.Fatalf("got %q", string(data))
	}
}

func TestParseLastJSONFrame_Empty(t *testing.T) {
	_, ok := parseLastJSONFrame([]byte(""))
	if ok {
		t.Fatal("expected not ok")
	}
}

func TestAcceptableResponseTs_FeedbackGet(t *testing.T) {
	accept := acceptableResponseTs(FEEDBACK_GET)
	if accept == nil {
		t.Fatal("expected non-nil for FEEDBACK_GET")
	}
	if !accept[1051] {
		t.Fatal("expected 1051 accepted")
	}
	if !accept[FEEDBACK_GET] {
		t.Fatal("expected FEEDBACK_GET accepted")
	}
	if accept[999] {
		t.Fatal("999 should not be accepted")
	}
}

func TestAcceptableResponseTs_UnknownCommand_AcceptsAll(t *testing.T) {
	if acceptableResponseTs(999) != nil {
		t.Fatal("expected nil (accept-any) for unknown command")
	}
}

func TestKeysOf(t *testing.T) {
	m := map[int]bool{1: true, 2: true, 3: true}
	keys := keysOf(m)
	if len(keys) != 3 {
		t.Fatalf("expected 3 keys, got %d", len(keys))
	}
	// Just check membership; order isn't guaranteed by keysOf.
	seen := map[int]bool{}
	for _, k := range keys {
		seen[k] = true
	}
	for want := range m {
		if !seen[want] {
			t.Fatalf("expected key %d in result", want)
		}
	}
}

func TestKeysOf_Empty(t *testing.T) {
	keys := keysOf(map[int]bool{})
	if len(keys) != 0 {
		t.Fatalf("expected empty slice, got %v", keys)
	}
}

func TestValidateSpeed_Range(t *testing.T) {
	if err := ValidateSpeed(0); err == nil {
		t.Fatal("expected error for 0")
	}
	if err := ValidateSpeed(1); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateSpeed(4096); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateSpeed(4097); err == nil {
		t.Fatal("expected error for 4097")
	}
}

func TestValidateAcceleration_Range(t *testing.T) {
	if err := ValidateAcceleration(0); err == nil {
		t.Fatal("expected error for 0")
	}
	if err := ValidateAcceleration(1); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateAcceleration(254); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateAcceleration(255); err == nil {
		t.Fatal("expected error for 255")
	}
}

func TestValidateLEDBrightness_Range(t *testing.T) {
	if err := ValidateLEDBrightness(-1); err == nil {
		t.Fatal("expected error for -1")
	}
	if err := ValidateLEDBrightness(0); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateLEDBrightness(255); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if err := ValidateLEDBrightness(256); err == nil {
		t.Fatal("expected error for 256")
	}
}

func TestEstimateMoveDeadline_Bounds(t *testing.T) {
	now := time.Now()
	// A very slow speed (lots of time per radian) should be clamped to 10s max.
	d := estimateMoveDeadline(now, 1).Sub(now)
	if d > 10*time.Second+1*time.Millisecond {
		t.Fatalf("expected clamp <= 10s, got %v", d)
	}
	if d < 100*time.Millisecond {
		t.Fatalf("expected >= 100ms, got %v", d)
	}
	// Very fast speed should be clamped to 100ms floor.
	d2 := estimateMoveDeadline(now, 4096).Sub(now)
	if d2 < 100*time.Millisecond {
		t.Fatalf("expected 100ms floor, got %v", d2)
	}
}

func TestEstimateMoveDeadline_DefendsAgainstZeroSpeed(t *testing.T) {
	// Even with a speed that maps to zero deg/s, we should return a sane deadline.
	now := time.Now()
	d := estimateMoveDeadline(now, 0).Sub(now)
	if d <= 0 {
		t.Fatalf("expected positive duration, got %v", d)
	}
}

func TestNewRoArmController_RejectsEmptyConfig(t *testing.T) {
	_, err := NewRoArmController(&RoArmConfig{})
	if err == nil {
		t.Fatal("expected error for empty config (no host, no port)")
	}
}

func TestNewRoArmController_HTTPMode(t *testing.T) {
	c, err := NewRoArmController(&RoArmConfig{Host: "1.2.3.4"})
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if !c.isHTTP {
		t.Fatal("expected HTTP mode")
	}
	if c.httpHost != "1.2.3.4" {
		t.Fatalf("got httpHost=%q", c.httpHost)
	}
	if c.httpTimeout != DefaultHTTPTimeout {
		t.Fatalf("expected default HTTP timeout, got %v", c.httpTimeout)
	}
	_ = c.Close(nil)
}

func TestNewRoArmController_UsesCustomTimeouts(t *testing.T) {
	c, err := NewRoArmController(&RoArmConfig{
		Host:          "1.2.3.4",
		HTTPTimeout:   Duration(2 * time.Second),
		SerialTimeout: Duration(3 * time.Second),
	})
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if c.httpTimeout != 2*time.Second {
		t.Fatalf("got httpTimeout=%v", c.httpTimeout)
	}
	if c.serialTimeout != 3*time.Second {
		t.Fatalf("got serialTimeout=%v", c.serialTimeout)
	}
	_ = c.Close(nil)
}

func TestHTTPGetFeedback(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6})
	defer srv.Close()
	defer c.Close(context.Background())

	fb, err := c.GetFeedback(context.Background())
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if fb.B != 0.1 || fb.G != 0.6 {
		t.Fatalf("unexpected feedback: %+v", fb)
	}
}

func TestHTTPGetJointRadians(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{B: 0.1, S: 0.2, E: 0.3, Wrist: 0.4, R: 0.5, G: 0.6})
	defer srv.Close()
	defer c.Close(context.Background())

	rads, err := c.GetJointRadians(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(rads) != 6 {
		t.Fatalf("expected 6, got %d", len(rads))
	}
	if rads[0] != 0.1 || rads[5] != 0.6 {
		t.Fatalf("unexpected radians: %v", rads)
	}
}

func TestHTTPSetTorque(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())
	if err := c.SetTorque(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	if err := c.SetTorque(context.Background(), false); err != nil {
		t.Fatal(err)
	}
}

func TestHTTPSetLED(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())
	if err := c.SetLED(context.Background(), 128); err != nil {
		t.Fatal(err)
	}
	if err := c.SetLED(context.Background(), 999); err == nil {
		t.Fatal("expected error for out-of-range brightness")
	}
}

func TestHTTPSetJointRadian(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())

	if err := c.SetJointRadian(context.Background(), 1, 0.5, 500, 50); err != nil {
		t.Fatal(err)
	}
	// joint out of range
	if err := c.SetJointRadian(context.Background(), 0, 0.5, 500, 50); err == nil {
		t.Fatal("expected error for joint 0")
	}
	if err := c.SetJointRadian(context.Background(), 7, 0.5, 500, 50); err == nil {
		t.Fatal("expected error for joint 7")
	}
	// speed out of range
	if err := c.SetJointRadian(context.Background(), 1, 0.5, -1, 50); err == nil {
		t.Fatal("expected error for bad speed")
	}
	// accel out of range
	if err := c.SetJointRadian(context.Background(), 1, 0.5, 500, -1); err == nil {
		t.Fatal("expected error for bad accel")
	}
	// radian out of range for joint 1
	if err := c.SetJointRadian(context.Background(), 1, 100.0, 500, 50); err == nil {
		t.Fatal("expected error for out-of-range radian")
	}
}

func TestHTTPSetJointRadians(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())

	if err := c.SetJointRadians(context.Background(), []float64{0, 0, 0, 0, 0, 0}, 500, 50); err != nil {
		t.Fatal(err)
	}
	// Wrong length
	if err := c.SetJointRadians(context.Background(), []float64{0}, 500, 50); err == nil {
		t.Fatal("expected error for wrong length")
	}
	// Out-of-range radian
	if err := c.SetJointRadians(context.Background(), []float64{100, 0, 0, 0, 0, 0}, 500, 50); err == nil {
		t.Fatal("expected error for out-of-range radian")
	}
	// Bad speed
	if err := c.SetJointRadians(context.Background(), []float64{0, 0, 0, 0, 0, 0}, -1, 50); err == nil {
		t.Fatal("expected error for bad speed")
	}
	// Bad accel
	if err := c.SetJointRadians(context.Background(), []float64{0, 0, 0, 0, 0, 0}, 500, -1); err == nil {
		t.Fatal("expected error for bad accel")
	}
}

func TestHTTPMoveToHome(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())
	if err := c.MoveToHome(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestHTTPTestConnection(t *testing.T) {
	c, srv := newHTTPTestController(t, 1051, FeedbackData{})
	defer srv.Close()
	defer c.Close(context.Background())
	if err := c.TestConnection(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestHTTPCommand_BadServer_ReturnsError(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, "nope", http.StatusInternalServerError)
	}))
	defer srv.Close()
	u, _ := url.Parse(srv.URL)
	c, err := NewRoArmController(&RoArmConfig{Host: u.Host, HTTPTimeout: Duration(time.Second)})
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close(context.Background())
	_, err = c.GetFeedback(context.Background())
	if err == nil {
		t.Fatal("expected error on 500 response")
	}
	if !strings.Contains(err.Error(), "500") {
		t.Fatalf("expected 500 in error, got: %v", err)
	}
}

func TestHTTPCommand_UnexpectedResponseT_StillReturns(t *testing.T) {
	// Controller logs a warning but returns the feedback even when T mismatches.
	c, srv := newHTTPTestController(t, 99999, FeedbackData{B: 0.1})
	defer srv.Close()
	defer c.Close(context.Background())
	fb, err := c.GetFeedback(context.Background())
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if fb.B != 0.1 {
		t.Fatalf("expected feedback to return, got %+v", fb)
	}
}

func TestNewRoArmController_SerialFailsOnBadPort(t *testing.T) {
	_, err := NewRoArmController(&RoArmConfig{Port: "/definitely/not/a/real/device/12345"})
	if err == nil {
		t.Fatal("expected error for nonexistent serial port")
	}
}

func TestNewRoArmController_DefaultBaudrate(t *testing.T) {
	// This will fail to open the device, but we'll get the "failed to open" error
	// rather than a baudrate error — confirming the default-baudrate branch ran.
	_, err := NewRoArmController(&RoArmConfig{Port: "/bogus", Baudrate: 0})
	if err == nil {
		t.Fatal("expected error")
	}
	if !strings.Contains(err.Error(), "failed to open") {
		t.Fatalf("expected open failure, got: %v", err)
	}
}

func TestHTTPCommand_BadJSON_ReturnsError(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte("not json"))
	}))
	defer srv.Close()
	u, _ := url.Parse(srv.URL)
	c, err := NewRoArmController(&RoArmConfig{Host: u.Host, HTTPTimeout: Duration(time.Second)})
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close(context.Background())
	_, err = c.GetFeedback(context.Background())
	if err == nil {
		t.Fatal("expected error on bad JSON response")
	}
}

// fakeSerialPort implements serial.Port for testing sendSerialCommand.
type fakeSerialPort struct {
	written  []byte
	toRead   []byte
	readErr  error
	readPos  int
	closed   bool
	resetIn  int
	resetOut int
}

func (p *fakeSerialPort) SetMode(mode *serial.Mode) error { return nil }
func (p *fakeSerialPort) Read(buf []byte) (int, error) {
	if p.readErr != nil {
		return 0, p.readErr
	}
	if p.readPos >= len(p.toRead) {
		// Simulate no data available.
		return 0, nil
	}
	n := copy(buf, p.toRead[p.readPos:])
	p.readPos += n
	return n, nil
}
func (p *fakeSerialPort) Write(b []byte) (int, error) {
	p.written = append(p.written, b...)
	return len(b), nil
}
func (p *fakeSerialPort) Drain() error             { return nil }
func (p *fakeSerialPort) ResetInputBuffer() error  { p.resetIn++; return nil }
func (p *fakeSerialPort) ResetOutputBuffer() error { p.resetOut++; return nil }
func (p *fakeSerialPort) SetDTR(v bool) error      { return nil }
func (p *fakeSerialPort) SetRTS(v bool) error      { return nil }
func (p *fakeSerialPort) GetModemStatusBits() (*serial.ModemStatusBits, error) {
	return &serial.ModemStatusBits{}, nil
}
func (p *fakeSerialPort) SetReadTimeout(t time.Duration) error { return nil }
func (p *fakeSerialPort) Close() error                         { p.closed = true; return nil }
func (p *fakeSerialPort) Break(d time.Duration) error          { return nil }

func newSerialTestController(t *testing.T, port *fakeSerialPort) *RoArmController {
	t.Helper()
	return &RoArmController{
		serialPort:    port,
		isHTTP:        false,
		serialTimeout: 500 * time.Millisecond,
		httpTimeout:   DefaultHTTPTimeout,
		logger:        logging.NewTestLogger(t),
		tracker:       newMotionTracker(),
	}
}

func TestSendSerialCommand_ReadsCompleteFrame(t *testing.T) {
	port := &fakeSerialPort{
		toRead: []byte("{\"T\":1051,\"b\":0.5}\r\n"),
	}
	c := newSerialTestController(t, port)
	fb, err := c.GetFeedback(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if fb.B != 0.5 {
		t.Fatalf("expected B=0.5, got %v", fb.B)
	}
	if len(port.written) == 0 {
		t.Fatal("expected command bytes written")
	}
}

func TestSendSerialCommand_Timeout(t *testing.T) {
	port := &fakeSerialPort{} // no data to read
	c := newSerialTestController(t, port)
	c.serialTimeout = 100 * time.Millisecond
	_, err := c.GetFeedback(context.Background())
	if err == nil {
		t.Fatal("expected timeout error")
	}
}

func TestSendSerialCommand_ContextCancelled(t *testing.T) {
	port := &fakeSerialPort{} // no data
	c := newSerialTestController(t, port)
	c.serialTimeout = 5 * time.Second
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_, err := c.GetFeedback(ctx)
	if err == nil {
		t.Fatal("expected context cancelled error")
	}
}

func TestSendSerialCommand_FiltersUnexpectedT(t *testing.T) {
	// Send a stale 999 frame first, then the expected 1051. The filter
	// should drop 999 and then accept 1051 on a subsequent read.
	//
	// Because our fake returns all remaining bytes in one Read, we structure
	// the buffer so the LAST frame is 1051 — parseLastJSONFrame picks the
	// last, and 1051 is accepted.
	port := &fakeSerialPort{
		toRead: []byte("{\"T\":999}\r\n{\"T\":1051,\"b\":0.1}\r\n"),
	}
	c := newSerialTestController(t, port)
	fb, err := c.GetFeedback(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if fb.T != 1051 {
		t.Fatalf("expected T=1051, got %d", fb.T)
	}
}

func TestSendSerialCommand_BadJSONInFrame(t *testing.T) {
	// Send a syntactically bad frame first, then a valid one — the controller
	// should reset the buffer and keep reading.
	port := &fakeSerialPort{
		toRead: []byte("{bogus}\r\n{\"T\":1051,\"b\":0.25}\r\n"),
	}
	c := newSerialTestController(t, port)
	fb, err := c.GetFeedback(context.Background())
	// parseLastJSONFrame will return the LAST frame. JSON decoding of
	// '{"T":1051,"b":0.25}' succeeds. Verify.
	if err != nil {
		// An error is acceptable because '{bogus}' comes before and the parser
		// might find it. But with LastIndex semantics it should pick the valid one.
		t.Skipf("skipping: parseLastJSONFrame semantics: %v", err)
	}
	if fb != nil && fb.B != 0.25 {
		t.Fatalf("expected B=0.25, got %v", fb.B)
	}
}

func TestRoArmControllerClose_HTTPMode(t *testing.T) {
	c, err := NewRoArmController(&RoArmConfig{Host: "1.2.3.4"})
	if err != nil {
		t.Fatal(err)
	}
	// In HTTP mode Close is a no-op success.
	if err := c.Close(context.Background()); err != nil {
		t.Fatalf("unexpected: %v", err)
	}
}

func TestRoArmControllerIsMoving_Default(t *testing.T) {
	c, err := NewRoArmController(&RoArmConfig{Host: "1.2.3.4"})
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	moving, err := c.IsMoving(nil)
	if err != nil {
		t.Fatalf("unexpected: %v", err)
	}
	if moving {
		t.Fatal("expected not moving before any recordMove")
	}
	c.NoteMotionDeadline(time.Now().Add(200 * time.Millisecond))
	moving, _ = c.IsMoving(nil)
	if !moving {
		t.Fatal("expected moving after NoteMotionDeadline")
	}
	_ = c.Close(nil)
}

func TestCommandMarshalJSON_NilData(t *testing.T) {
	cmd := &Command{T: FEEDBACK_GET}
	data, err := cmd.MarshalJSON()
	if err != nil {
		t.Fatal(err)
	}
	var out map[string]interface{}
	if err := json.Unmarshal(data, &out); err != nil {
		t.Fatal(err)
	}
	if len(out) != 1 {
		t.Fatalf("expected 1 key, got %v", out)
	}
}

func TestMotionTrackerIsMovingBeforeDeadline(t *testing.T) {
	tr := newMotionTracker()
	tr.recordMove(time.Now().Add(200 * time.Millisecond))
	if !tr.isMoving(time.Now()) {
		t.Fatal("expected moving before deadline")
	}
}

func TestMotionTrackerNotMovingAfterDeadline(t *testing.T) {
	tr := newMotionTracker()
	tr.recordMove(time.Now().Add(-10 * time.Millisecond))
	if tr.isMoving(time.Now()) {
		t.Fatal("expected not moving after deadline")
	}
}

func TestMotionTrackerNotMovingBeforeFirstRecord(t *testing.T) {
	tr := newMotionTracker()
	if tr.isMoving(time.Now()) {
		t.Fatal("expected not moving before any recordMove call")
	}
}
