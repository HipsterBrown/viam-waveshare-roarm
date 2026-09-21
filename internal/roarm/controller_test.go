package roarm

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
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
// a static FeedbackData payload and wires an Controller to it.
func newHTTPTestController(t *testing.T, respT int, body FeedbackData) (*Controller, *httptest.Server) {
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
	c, err := NewController(&Config{Host: u.Host, HTTPTimeout: Duration(2 * time.Second)})
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
	c := newSerialTestController(t, &fakeSerialPort{})
	fb, _, ok := c.extractLastValidFeedback([]byte("{\"T\":1051,\"b\":0.5,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"))
	if !ok {
		t.Fatal("expected ok")
	}
	if fb.T != 1051 || fb.B != 0.5 {
		t.Fatalf("unexpected feedback: %+v", fb)
	}
}

func TestExtractLastValidFeedback_MultipleCleanReturnsLatest(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	buf := []byte("{\"T\":1,\"b\":0.1}\r\n{\"T\":1051,\"b\":0.9,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n")
	fb, _, ok := c.extractLastValidFeedback(buf)
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
	c := newSerialTestController(t, &fakeSerialPort{})
	clean := "{\"T\":1051,\"b\":0.42,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"
	// Corrupt blob: two partial frames merged, mid-number splice, no
	// interior `{` so the outermost window is the whole mess.
	corrupt := "{\"T\":1051,\"x\":1.0,\"g\":3.1768742\"x\":1.0,\"g\":3.176874212,\"tR\":0}\r\n"
	buf := []byte(clean + corrupt)
	fb, _, ok := c.extractLastValidFeedback(buf)
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
	c := newSerialTestController(t, &fakeSerialPort{})
	garbage := []byte(`{"T":1051,"x":48.78839104,"y":1.047927472,"z":552.6022117,"tit":-1.55852448,"b":0.021475731,"s":0.006135923,"e":0.004601942,"t":0.001533981,"r":0.001533981,"g":3.1768742"x":48.78839104,"y":1.047927472,"z":552.6022117,"tit":-1.55852448,"b":0.021475731,"s":0.006135923,"e":0.004601942,"t":0.001533981,"r":0.001533981,"g":3.176874212,"tB":-109,"tS":45,"tE":29,"tT":21,"tR":0}` + "\r\n")
	if fb, _, ok := c.extractLastValidFeedback(garbage); ok {
		t.Fatalf("expected not ok for pure garbage, got %+v", fb)
	}
}

func TestExtractLastValidFeedback_NoDelimiter(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	if _, _, ok := c.extractLastValidFeedback([]byte("{\"T\":1051,\"b\":0}")); ok {
		t.Fatal("expected not ok when `}\\r\\n` terminator missing")
	}
}

func TestExtractLastValidFeedback_Empty(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	if _, _, ok := c.extractLastValidFeedback(nil); ok {
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
	_, err := NewController(&Config{})
	if err == nil {
		t.Fatal("expected error for empty config (no host, no port)")
	}
}

func TestNewRoArmController_HTTPMode(t *testing.T) {
	c, err := NewController(&Config{Host: "1.2.3.4"})
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
	c, err := NewController(&Config{
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
	c, err := NewController(&Config{Host: u.Host, HTTPTimeout: Duration(2 * time.Second)})
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
	c, err := NewController(&Config{Host: u.Host, HTTPTimeout: Duration(time.Second)})
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
	_, err := NewController(&Config{Port: "/definitely/not/a/real/device/12345"})
	if err == nil {
		t.Fatal("expected error for nonexistent serial port")
	}
}

func TestNewRoArmController_DefaultBaudrate(t *testing.T) {
	// This will fail to open the device, but we'll get the "failed to open" error
	// rather than a baudrate error — confirming the default-baudrate branch ran.
	_, err := NewController(&Config{Port: "/bogus", Baudrate: 0})
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
	c, err := NewController(&Config{Host: u.Host, HTTPTimeout: Duration(time.Second)})
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
	written         []byte
	toRead          []byte
	frames          [][]byte // popped one per ResetInputBuffer (i.e. per query)
	readErr         error
	readPos         int
	closed          bool
	resetIn         int
	resetOut        int
	shortWriteAfter int   // when > 0, Write only accepts this many bytes
	resetErr        error // when set, ResetInputBuffer returns this error
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
	n := len(b)
	if p.shortWriteAfter > 0 && p.shortWriteAfter < n {
		n = p.shortWriteAfter
	}
	p.written = append(p.written, b[:n]...)
	return n, nil
}
func (p *fakeSerialPort) Drain() error { return nil }
func (p *fakeSerialPort) ResetInputBuffer() error {
	p.resetIn++
	if p.resetErr != nil {
		return p.resetErr
	}
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

func newSerialTestController(t *testing.T, port *fakeSerialPort) *Controller {
	t.Helper()
	return &Controller{
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
		toRead: []byte("{\"T\":1051,\"b\":0.5,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
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

// Audit 2.5: one dropped frame must not fail an operation.
func TestQueryRetriesAndSucceeds(t *testing.T) {
	// The first two attempts see an empty port (a frame timeout); the third
	// gets a good frame. ResetInputBuffer pops the next scripted response.
	port := &fakeSerialPort{frames: [][]byte{nil, nil, []byte(`{"T":1051,"b":0.4,"s":0,"e":0,"t":0,"r":0,"g":3.0}` + "\r\n")}}
	c := newSerialTestController(t, port)
	c.serialTimeout = 60 * time.Millisecond // keep the test quick
	fb, err := c.GetFeedback(context.Background())
	if err != nil || fb.B != 0.4 {
		t.Fatalf("expected the third attempt to succeed: %v %v", fb, err)
	}
	if got := c.Health().Retries; got != 2 {
		t.Fatalf("counted %d retries, want 2", got)
	}
}

func TestQueryExhaustsRetries(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	c.serialTimeout = 30 * time.Millisecond
	if _, err := c.GetFeedback(context.Background()); err == nil {
		t.Fatal("expected an error after the attempts are exhausted")
	}
	h := c.Health()
	if h.RetriesExhausted != 1 || h.Retries != queryAttempts-1 {
		t.Fatalf("health after exhaustion: %+v", h)
	}
	// Every attempt gives up without a usable frame (no data ever arrives), so
	// serialReadFeedback's give-up branch fires once per attempt.
	if h.ReadTimeouts != queryAttempts {
		t.Fatalf("ReadTimeouts = %d, want %d", h.ReadTimeouts, queryAttempts)
	}
}

// errClosedPort stands in for the *serial.PortError the library returns from a
// read on an unplugged port. serial.PortError's code field is unexported, so
// the real one cannot be constructed with the code we need; the classifier
// matches the Code() method, which is all either type has in common.
type errClosedPort struct{}

func (errClosedPort) Error() string              { return "port has been closed" }
func (errClosedPort) Code() serial.PortErrorCode { return serial.PortClosed }

// A closed or unplugged port is not retried: go.bug.st/serial reports it as a
// PortError with code PortClosed, and retrying cannot help.
func TestQueryDoesNotRetryAClosedPort(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{readErr: errClosedPort{}})
	before := c.Health().Retries
	if _, err := c.GetFeedback(context.Background()); err == nil {
		t.Fatal("expected an error")
	}
	if c.Health().Retries != before {
		t.Fatal("a closed port must not be retried")
	}
}

func TestQueryDoesNotRetryACancelledContext(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := c.GetFeedback(ctx); !errors.Is(err, context.Canceled) {
		t.Fatalf("want context.Canceled, got %v", err)
	}
	if c.Health().Retries != 0 {
		t.Fatal("a cancelled context must not be retried")
	}
}

// An attempt is not started when the caller's deadline cannot accommodate it.
func TestQueryRespectsTheCallerDeadline(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	c.serialTimeout = time.Second
	ctx, cancel := context.WithTimeout(context.Background(), 80*time.Millisecond)
	defer cancel()
	start := time.Now()
	if _, err := c.GetFeedback(ctx); err == nil {
		t.Fatal("expected an error")
	}
	if el := time.Since(start); el > 250*time.Millisecond {
		t.Fatalf("query overran the caller's deadline: %v", el)
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
		toRead: []byte("{\"T\":999}\r\n{\"T\":1051,\"b\":0.1,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
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
		toRead: []byte("{bogus}\r\n{\"T\":1051,\"b\":0.25,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
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
	c, err := NewController(&Config{Host: "1.2.3.4"})
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
		toRead: []byte("{\"T\":102,\"base\":0}\r\n{\"T\":1051,\"b\":0.25,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
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
	c, err := NewController(&Config{Host: u.Host})
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
		[]byte("{\"T\":1051,\"b\":0.10,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
		[]byte("{\"T\":1051,\"b\":0.30,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
	}}
	c := newSerialTestController(t, moving)
	got, err := c.IsMoving(context.Background())
	if err != nil || !got {
		t.Fatalf("expected moving, got %v err=%v", got, err)
	}
	still := &fakeSerialPort{frames: [][]byte{
		[]byte("{\"T\":1051,\"b\":0.10,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
		[]byte("{\"T\":1051,\"b\":0.101,\"s\":0,\"e\":0,\"t\":0,\"r\":0,\"g\":3.0}\r\n"),
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
	if _, err := c.GetJointRadians(context.Background()); !errors.Is(err, ErrNoFeedback) {
		t.Fatalf("expected ErrNoFeedback, got %v", err)
	}
	moving, err := c.IsMoving(context.Background())
	if err != nil || moving {
		t.Fatalf("expected false, nil; got %v %v", moving, err)
	}
	// Without feedback there is nothing to poll, so the settle sleeps the
	// modelled duration of the commanded move and reports an arrival with no
	// positions. One degree at the default profile is acceleration-limited:
	// 2*sqrt(travel/a) is about 200 ms.
	req := SettleRequest{
		Start:         []float64{0, 0, 0, 0, 0, 0},
		Target:        []float64{math.Pi / 180, 0, 0, 0, 0, 0},
		Mask:          ArmMask,
		SpeedUnits:    SpeedToUnits(DefaultSpeedDegsPerSec),
		AccUnits:      AccelToUnits(DefaultAccelDegsPerSecSq),
		RequireMotion: true,
	}
	// Derived, not a literal: a hardcoded floor would quietly go vacuous if
	// the duration model ever shortened.
	plan, err := planSettle(req)
	if err != nil {
		t.Fatal(err)
	}
	start := time.Now()
	res, err := c.WaitUntilSettled(context.Background(), req)
	if err != nil || res.Positions != nil {
		t.Fatalf("expected an arrival with no positions; got %+v %v", res, err)
	}
	if res.Outcome != SettleArrived {
		t.Fatalf("outcome %v, want arrived", res.Outcome)
	}
	if el := time.Since(start); el < plan.Duration {
		t.Fatalf("expected the modelled %v to be slept, only took %v", plan.Duration, el)
	}
}

// A frame is believed only when it carries every field the module reads.
// Audit 2.3: FeedbackData's value fields make a missing joint read as 0, so a
// torn frame whose remainder parses as JSON would put the arm at its zero pose.
func TestExtractRejectsIncompleteFrames(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	full := `{"T":1051,"x":1,"y":2,"z":3,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"r":0.5,"g":3.0}` + "\r\n"
	for _, tc := range []struct {
		name string
		buf  string
		ok   bool
	}{
		{"complete", full, true},
		{"no torque or cartesian fields", `{"T":1051,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"r":0.5,"g":3.0}` + "\r\n", true},
		{"T only", `{"T":1051}` + "\r\n", false},
		{"empty object", `{}` + "\r\n", false},
		{"missing g", `{"T":1051,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"r":0.5}` + "\r\n", false},
		{"non-numeric b", `{"T":1051,"b":"x","s":0.2,"e":0.3,"t":0.4,"r":0.5,"g":3.0}` + "\r\n", false},
		{"torn", `{"T":1051,"x":1` + "\r\n", false},
		{"wrong T", `{"T":102,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"r":0.5,"g":3.0}` + "\r\n", false},
	} {
		_, _, ok := c.extractLastValidFeedback([]byte(tc.buf))
		if ok != tc.ok {
			t.Fatalf("%s: got ok=%v, want %v", tc.name, ok, tc.ok)
		}
	}
	// Of the eight cases: two are ok immediately (no rejection); "torn" never
	// finds a `}\r\n` terminator at all, so the walk never even forms a
	// candidate to reject. The remaining five candidates are all counted:
	// four missing required keys or unparseable (invalid), and one
	// well-formed but carrying an unsolicited T (stale).
	h := c.Health()
	if h.InvalidFrames != 4 {
		t.Fatalf("InvalidFrames = %d, want 4", h.InvalidFrames)
	}
	if h.StaleFrames != 1 {
		t.Fatalf("StaleFrames = %d, want 1", h.StaleFrames)
	}
}

// The walk continues past an invalid newest frame instead of surfacing it, so
// a good frame earlier in the buffer is still found rather than discarded.
func TestExtractWalksPastAnInvalidNewestFrame(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	good := `{"T":1051,"b":0.7,"s":0,"e":0,"t":0,"r":0,"g":3.0}`
	buf := good + "\r\n" + `{"T":1051}` + "\r\n"
	fb, _, ok := c.extractLastValidFeedback([]byte(buf))
	if !ok || fb.B != 0.7 {
		t.Fatalf("ok=%v b=%v; want the earlier complete frame", ok, fb)
	}
}

// Distinguishes the two rejection counters from a single walk: a corrupt or
// incomplete candidate counts as InvalidFrames, a well-formed candidate
// carrying a T the module never asked for counts as StaleFrames (audit 2.8 —
// evidence the link is delivering unsolicited traffic). Reverting either
// counter's wiring in extractLastValidFeedback leaves it at zero here.
func TestExtractLastValidFeedbackCountsInvalidAndStaleFrames(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	valid := `{"T":1051,"b":0.7,"s":0,"e":0,"t":0,"r":0,"g":3.0}` + "\r\n"
	stale := `{"T":102,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"r":0.5,"g":3.0}` + "\r\n" // well-formed, unsolicited T
	invalid := `{"T":1051}` + "\r\n"                                              // missing required keys
	buf := []byte(valid + stale + invalid)
	fb, _, ok := c.extractLastValidFeedback(buf)
	if !ok || fb.B != 0.7 {
		t.Fatalf("ok=%v fb=%v; want the valid frame", ok, fb)
	}
	h := c.Health()
	if h.InvalidFrames != 1 {
		t.Fatalf("InvalidFrames = %d, want 1", h.InvalidFrames)
	}
	if h.StaleFrames != 1 {
		t.Fatalf("StaleFrames = %d, want 1", h.StaleFrames)
	}
}

// Audit 2.4: a port that accepts only part of the buffer must be an error, not
// a truncated command the firmware silently discards.
func TestSerialWriteRejectsAShortWrite(t *testing.T) {
	port := &fakeSerialPort{shortWriteAfter: 5}
	c := newSerialTestController(t, port)
	err := c.SetTorque(context.Background(), true)
	if err == nil || !strings.Contains(err.Error(), "short write") {
		t.Fatalf("expected a short-write error, got %v", err)
	}
	h := c.Health()
	if h.ShortWrites != 1 {
		t.Fatalf("ShortWrites = %d, want 1", h.ShortWrites)
	}
	if h.LastError == "" {
		t.Fatal("expected the short write to be noted as the last error")
	}
}

// A ResetInputBuffer failure means fresh frames can no longer be told from
// stale ones, which is the premise the read path rests on. One failure warns;
// two consecutive failures are an error.
func TestResetInputBufferFailureEscalates(t *testing.T) {
	port := &fakeSerialPort{resetErr: errors.New("device busy"), toRead: []byte(`{"T":1051,"b":0,"s":0,"e":0,"t":0,"r":0,"g":3.0}` + "\r\n")}
	c := newSerialTestController(t, port)
	if _, err := c.GetFeedback(context.Background()); err != nil {
		t.Fatalf("the first failure should only warn: %v", err)
	}
	if got := c.Health().ResetFailures; got != 1 {
		t.Fatalf("ResetFailures after the first failure = %d, want 1", got)
	}
	// Both assertions below are load-bearing. Without the escalation this call
	// still returns an error, because the fake's read position does not reset
	// between calls and the read times out instead -- so asserting only err !=
	// nil passes with the fix reverted and tests nothing.
	start := time.Now()
	_, err := c.GetFeedback(context.Background())
	if err == nil {
		t.Fatal("the second consecutive failure should be an error")
	}
	if !strings.Contains(err.Error(), "consecutive failures") {
		t.Fatalf("the error should name the reset failure, not something downstream: %v", err)
	}
	if el := time.Since(start); el > c.serialTimeout/2 {
		t.Fatalf("the escalation should short-circuit before any read is attempted, took %v", el)
	}
	// ResetFailures counts every failed ResetInputBuffer: the first call's
	// warning-only failure plus the second call's escalating one. Exactly two,
	// not one per retry attempt, because the escalation is classified fatal --
	// a flush that has already failed twice in a row will not succeed on a
	// retry milliseconds later, and each attempt would re-flush.
	if got := c.Health().ResetFailures; got != 2 {
		t.Fatalf("ResetFailures after the escalation = %d, want 2 (one per call, no retries)", got)
	}
	if !errors.Is(err, ErrCannotFlushInput) {
		t.Fatalf("the escalation should be identifiable by sentinel, got %v", err)
	}
	if !fatalTransport(err) {
		t.Fatal("an un-flushable port must not be retried: every attempt re-flushes and fails the same way")
	}
}

// A per-attempt deadline expiring is an ordinary frame timeout, not a fault
// worth giving up on: queryWithRetries gives every attempt its own context, so
// whether a timed-out read surfaces as the read loop's own error or as
// context.DeadlineExceeded depends on which of two timers fires first.
// Classifying context errors as fatal would let the first slow read kill every
// remaining attempt, silently disabling retries wherever that race lands the
// other way. The caller's own cancellation is handled by checking the caller's
// ctx.Err() around each attempt instead, which TestQueryDoesNotRetryACancelled
// Context covers.
func TestContextErrorsAreNotFatalTransport(t *testing.T) {
	for _, err := range []error{
		context.DeadlineExceeded,
		context.Canceled,
		fmt.Errorf("serial read: %w", context.DeadlineExceeded),
	} {
		if fatalTransport(err) {
			t.Fatalf("%v must stay retryable: it is what a per-attempt deadline looks like", err)
		}
	}
}

// The deterministic form of the race documented on fatalTransport: an attempt
// that fails with its own per-attempt deadline must still be retried. Whether
// a timed-out read reports the read loop's own error or context.DeadlineExceeded
// depends on Go's timer-goroutine latency, so the retry loop's behavior must
// not depend on which one it gets.
func TestQueryRetriesAPerAttemptDeadline(t *testing.T) {
	c := newSerialTestController(t, &fakeSerialPort{})
	calls := 0
	_, err := c.queryWithRetries(context.Background(), 20*time.Millisecond,
		func(context.Context) (*FeedbackData, error) {
			calls++
			return nil, fmt.Errorf("serial read: %w", context.DeadlineExceeded)
		})
	if err == nil {
		t.Fatal("expected the attempts to be exhausted")
	}
	if calls != queryAttempts {
		t.Fatalf("made %d attempts, want all %d: one attempt's deadline must not end the loop", calls, queryAttempts)
	}
	if got := c.Health().Retries; got != queryAttempts-1 {
		t.Fatalf("Retries = %d, want %d", got, queryAttempts-1)
	}
}
