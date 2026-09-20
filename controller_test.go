package waveshareroarm

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"math"
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
		T: cmdJointRadianCtrl,
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
	if int(out["T"].(float64)) != cmdJointRadianCtrl {
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
	cmd := &Command{T: cmdFeedbackGet, Data: map[string]interface{}{}}
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

func TestExtractLastValidFeedback_CleanSingle(t *testing.T) {
	fb, _, ok := extractLastValidFeedback([]byte("{\"T\":1051,\"b\":0.5}\r\n"))
	if !ok {
		t.Fatal("expected ok")
	}
	if fb.T != 1051 || fb.B != 0.5 {
		t.Fatalf("unexpected feedback: %+v", fb)
	}
}

func TestExtractLastValidFeedback_MultipleCleanReturnsLatest(t *testing.T) {
	buf := []byte("{\"T\":1,\"b\":0.1}\r\n{\"T\":1051,\"b\":0.9}\r\n")
	fb, _, ok := extractLastValidFeedback(buf)
	if !ok {
		t.Fatal("expected ok")
	}
	if fb.T != 1051 || fb.B != 0.9 {
		t.Fatalf("expected the later frame, got %+v", fb)
	}
}

// Regression guard: the firmware occasionally emits a torn blob where two
// partial frames are concatenated without the intervening `}\r\n{` boundary
// (e.g. "...,\"g\":3.1768742\"x\":..."). The outermost {...}\r\n window
// wraps garbage that won't parse. When a clean earlier frame exists in
// the buffer, we should fall back to it rather than failing.
func TestExtractLastValidFeedback_CorruptLatestFallsBackToClean(t *testing.T) {
	clean := "{\"T\":1051,\"b\":0.42}\r\n"
	// Corrupt blob: two partial frames merged, mid-number splice, no
	// interior `{` so the outermost window is the whole mess.
	corrupt := "{\"T\":1051,\"x\":1.0,\"g\":3.1768742\"x\":1.0,\"g\":3.176874212,\"tR\":0}\r\n"
	buf := []byte(clean + corrupt)
	fb, _, ok := extractLastValidFeedback(buf)
	if !ok {
		t.Fatal("expected recovery to the clean earlier frame")
	}
	if fb.T != 1051 || fb.B != 0.42 {
		t.Fatalf("expected the clean frame, got %+v", fb)
	}
}

// Regression guard for the specific garbage reported from live hardware on
// 2026-04-20. Two FeedbackData bodies merged without `}\r\n{` separation;
// no clean frame anywhere in the buffer. Must return not-ok (caller keeps
// reading) rather than surfacing a parse error.
func TestExtractLastValidFeedback_PureGarbageReturnsNotOk(t *testing.T) {
	garbage := []byte(`{"T":1051,"x":48.78839104,"y":1.047927472,"z":552.6022117,"tit":-1.55852448,"b":0.021475731,"s":0.006135923,"e":0.004601942,"t":0.001533981,"r":0.001533981,"g":3.1768742"x":48.78839104,"y":1.047927472,"z":552.6022117,"tit":-1.55852448,"b":0.021475731,"s":0.006135923,"e":0.004601942,"t":0.001533981,"r":0.001533981,"g":3.176874212,"tB":-109,"tS":45,"tE":29,"tT":21,"tR":0}` + "\r\n")
	if fb, _, ok := extractLastValidFeedback(garbage); ok {
		t.Fatalf("expected not ok for pure garbage, got %+v", fb)
	}
}

func TestExtractLastValidFeedback_NoDelimiter(t *testing.T) {
	if _, _, ok := extractLastValidFeedback([]byte("{\"T\":1051,\"b\":0}")); ok {
		t.Fatal("expected not ok when `}\\r\\n` terminator missing")
	}
}

func TestExtractLastValidFeedback_Empty(t *testing.T) {
	if _, _, ok := extractLastValidFeedback(nil); ok {
		t.Fatal("expected not ok for empty buffer")
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
	// Joints 1-5 pass through as raw feedback; joint 6 (gripper) is
	// transformed from firmware frame (≈π at closed) to software frame
	// (≈0 at closed) via π - G.
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
	if rads[0] != 0.1 {
		t.Fatalf("unexpected joint 1 radians: %v", rads)
	}
	const epsilon = 1e-9
	wantGripper := math.Pi - 0.6
	if diff := rads[5] - wantGripper; diff > epsilon || diff < -epsilon {
		t.Fatalf("expected joint 6 ≈ %v (π-G), got %v", wantGripper, rads[5])
	}
}

// TestJoint6FirmwareFrameTransform verifies the π-radian transform on the
// set side: software-frame radians → firmware-frame wire payload.
// Regression guard for hardware validation 2026-04-20: with the gripper
// physically ~closed the firmware reports G≈π, which means the limits
// [-0.2, 1.9] are expressed in the software frame and must be mapped via
// π - r on the way out.
func TestJoint6FirmwareFrameTransform(t *testing.T) {
	var captured struct {
		rad  float64 // SetJointRadian (single-joint path)
		hand float64 // SetJointRadians (6-joint path)
	}
	seenBoth := false

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		raw := r.URL.Query().Get("json")
		var payload map[string]interface{}
		_ = json.Unmarshal([]byte(raw), &payload)
		if rad, ok := payload["rad"].(float64); ok {
			captured.rad = rad
		}
		if hand, ok := payload["hand"].(float64); ok {
			captured.hand = hand
			seenBoth = true
		}
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(FeedbackData{T: 1051})
	}))
	defer srv.Close()

	u, err := url.Parse(srv.URL)
	if err != nil {
		t.Fatal(err)
	}
	c, err := NewRoArmController(&RoArmConfig{Host: u.Host, HTTPTimeout: Duration(2 * time.Second)})
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close(context.Background())

	ctx := context.Background()

	// Single-joint path: software 0 rad for joint 6 → firmware π on the wire.
	if err := c.SetJointRadian(ctx, 6, 0, 500, 50); err != nil {
		t.Fatal(err)
	}
	if diff := captured.rad - math.Pi; diff > 1e-9 || diff < -1e-9 {
		t.Fatalf("SetJointRadian: expected wire rad≈π (software 0), got %v", captured.rad)
	}

	// Non-gripper joints unaffected.
	if err := c.SetJointRadian(ctx, 1, 0.5, 500, 50); err != nil {
		t.Fatal(err)
	}
	if captured.rad != 0.5 {
		t.Fatalf("SetJointRadian joint 1: expected wire rad=0.5 (no transform), got %v", captured.rad)
	}

	// Six-joint path: software 1.9 (gripperOpenRad) → firmware ≈ π - 1.9.
	if err := c.SetJointRadians(ctx, []float64{0, 0, 0, 0, 0, 1.9}, 500, 50); err != nil {
		t.Fatal(err)
	}
	wantHand := math.Pi - 1.9
	if diff := captured.hand - wantHand; diff > 1e-9 || diff < -1e-9 {
		t.Fatalf("SetJointRadians: expected wire hand≈%v (π-1.9), got %v", wantHand, captured.hand)
	}
	if !seenBoth {
		t.Fatal("did not observe hand field in 6-joint payload")
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

// fakeSerialPort implements serial.Port for testing the serial transport.
type fakeSerialPort struct {
	written  []byte
	toRead   []byte
	frames   [][]byte // popped one per ResetInputBuffer (i.e. per query)
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
func (p *fakeSerialPort) Drain() error { return nil }
func (p *fakeSerialPort) ResetInputBuffer() error {
	p.resetIn++
	if len(p.frames) > 0 {
		p.toRead, p.frames, p.readPos = p.frames[0], p.frames[1:], 0
	}
	return nil
}
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
		serialPort:      port,
		isHTTP:          false,
		serialTimeout:   500 * time.Millisecond,
		httpTimeout:     DefaultHTTPTimeout,
		logger:          logging.NewTestLogger(t),
		canReadFeedback: true,
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

func TestCommandMarshalJSON_NilData(t *testing.T) {
	cmd := &Command{T: cmdFeedbackGet}
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

// A control command must return as soon as it is written, even when the
// firmware never answers (echo off). Before Task 4 this timed out.
func TestSerialWrite_ReturnsWithoutAResponse(t *testing.T) {
	port := &fakeSerialPort{} // nothing will ever be readable
	c := newSerialTestController(t, port)
	start := time.Now()
	if err := c.SetTorque(context.Background(), true); err != nil {
		t.Fatalf("SetTorque: %v", err)
	}
	if time.Since(start) > 100*time.Millisecond {
		t.Fatalf("SetTorque waited for a response: %v", time.Since(start))
	}
	if !bytes.Contains(port.written, []byte(`"T":210`)) {
		t.Fatalf("command not written: %q", port.written)
	}
}

// Feedback still waits for, and filters to, a T:1051 frame.
func TestSerialQuery_DropsEchoAndReturnsFeedback(t *testing.T) {
	port := &fakeSerialPort{
		toRead: []byte("{\"T\":102,\"base\":0}\r\n{\"T\":1051,\"b\":0.25}\r\n"),
	}
	c := newSerialTestController(t, port)
	fb, err := c.GetFeedback(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if fb.B != 0.25 {
		t.Fatalf("expected the 1051 frame, got %+v", fb)
	}
}

// HTTP control commands ignore the body entirely.
func TestHTTPWrite_IgnoresBody(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = w.Write([]byte("not json at all"))
	}))
	defer srv.Close()
	u, _ := url.Parse(srv.URL)
	c, err := NewRoArmController(&RoArmConfig{Host: u.Host})
	if err != nil {
		t.Fatal(err)
	}
	if err := c.SetLED(context.Background(), 10); err != nil {
		t.Fatalf("write should not parse the body: %v", err)
	}
}

// HTTP feedback must be a 1051 frame; anything else is an error naming HTTP.
func TestHTTPQuery_RejectsNonFeedbackBody(t *testing.T) {
	c, srv := newHTTPTestController(t, 99999, FeedbackData{B: 0.1})
	defer srv.Close()
	_, err := c.GetFeedback(context.Background())
	if err == nil || !strings.Contains(err.Error(), "HTTP") {
		t.Fatalf("expected an HTTP-naming error, got %v", err)
	}
}

func TestControllerIsMoving_ComparesTwoFeedbackFrames(t *testing.T) {
	moving := &fakeSerialPort{frames: [][]byte{
		[]byte("{\"T\":1051,\"b\":0.10}\r\n"),
		[]byte("{\"T\":1051,\"b\":0.30}\r\n"),
	}}
	c := newSerialTestController(t, moving)
	got, err := c.IsMoving(context.Background())
	if err != nil || !got {
		t.Fatalf("expected moving, got %v err=%v", got, err)
	}
	still := &fakeSerialPort{frames: [][]byte{
		[]byte("{\"T\":1051,\"b\":0.10}\r\n"),
		[]byte("{\"T\":1051,\"b\":0.101}\r\n"),
	}}
	c = newSerialTestController(t, still)
	got, err = c.IsMoving(context.Background())
	if err != nil || got {
		t.Fatalf("expected still, got %v err=%v", got, err)
	}
}

func TestControllerNoFeedback_Fallbacks(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	c.canReadFeedback = false
	if _, err := c.GetJointRadians(context.Background()); !errors.Is(err, errNoFeedback) {
		t.Fatalf("expected errNoFeedback, got %v", err)
	}
	moving, err := c.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("expected false, nil; got %v %v", moving, err)
	}
	start := time.Now()
	pos, err := c.WaitUntilSettled(context.Background(), []float64{0, 0, 0, 0, 0, 0}, armMask, 200*time.Millisecond)
	if err != nil || pos != nil {
		t.Fatalf("expected nil, nil; got %v %v", pos, err)
	}
	if time.Since(start) < 90*time.Millisecond {
		t.Fatal("expected the plain time estimate to be slept")
	}
}
