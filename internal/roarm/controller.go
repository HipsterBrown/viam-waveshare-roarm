package roarm

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net/http"
	"net/url"
	"os"
	"strings"
	"sync"
	"time"

	"go.bug.st/serial"
	"go.viam.com/rdk/logging"
)

// GripperSoftwareToWire converts joint 6 between the software reference frame
// (used by Viam APIs and joint limits) and the firmware wire frame. The
// firmware reports raw "closed" at roughly π rad, so the two frames are
// related by r_wire = π - r_software. The transform is its own inverse, so
// the same function converts either direction.
func GripperSoftwareToWire(r float64) float64 { return math.Pi - r }

// Firmware command types (the "T" field). Only the ones this module sends.
const (
	cmdFeedbackGet      = 105 // answered with a T:1051 frame
	cmdJointRadianCtrl  = 101 // one joint: joint, rad, spd, acc
	cmdJointsRadianCtrl = 102 // all joints: base..hand, spd, acc
	cmdLEDCtrl          = 114
	cmdTorqueSet        = 210
	feedbackFrameT      = 1051

	// Default timeouts
	DefaultHTTPTimeout   = 5 * time.Second
	DefaultSerialTimeout = 1 * time.Second

	// queryAttempts is how many times a feedback request is tried before the
	// caller sees an error. Audit 2.5: a 15-second settle can poll hundreds of
	// times, so a single dropped frame must not fail the whole operation.
	queryAttempts   = 3
	queryRetryDelay = 20 * time.Millisecond
	// serialChunkTimeout is the port's read timeout: short so the read loop
	// re-checks its context and its own deadline about every 20 ms. The frame
	// timeout is serialTimeout and is owned by serialReadFeedback.
	serialChunkTimeout = 20 * time.Millisecond

	// maxFrameLength caps the read buffer: older bytes cannot be part of the
	// frame still being assembled.
	maxFrameLength = 512
)

// errCannotFlushInput reports that the serial input buffer can no longer be
// flushed, so a fresh frame cannot be told from a stale one.
var errCannotFlushInput = errors.New("cannot flush the serial input buffer")

// fatalTransport reports whether err is worth no retry: the port is gone, the
// input buffer can no longer be flushed, or the command could not be put on
// the wire. go.bug.st/serial never returns io.EOF (a read timeout is (0, nil));
// an unplug is a PortError whose Code is PortClosed.
func fatalTransport(err error) bool {
	if err == nil {
		return false
	}
	// Context errors are deliberately NOT classified here. Every attempt runs
	// under its own context with the per-attempt budget, so an ordinary frame
	// timeout surfaces as context.DeadlineExceeded just as often as it
	// surfaces as the read loop's own timeout error, depending on which of the
	// two timers fires first. Calling that fatal would let the first slow read
	// kill every remaining attempt, silently disabling retries on a machine
	// where the race lands the other way. The caller's own cancellation is
	// handled structurally instead: queryWithRetries checks the CALLER's
	// ctx.Err() around each attempt.
	// A flush that has already failed twice in a row will not succeed on a
	// retry milliseconds later, and each attempt re-flushes, so retrying only
	// multiplies the failures before reporting the same thing.
	if errors.Is(err, errCannotFlushInput) {
		return true
	}
	var pe interface{ Code() serial.PortErrorCode }
	if errors.As(err, &pe) && pe.Code() == serial.PortClosed {
		return true
	}
	return strings.Contains(err.Error(), "short write")
}

// Command represents a JSON command to send to the RoArm
type Command struct {
	T int `json:"T"` // Command type
	// Dynamic fields based on command type
	Data map[string]interface{} `json:"-"`
}

// MarshalJSON implements custom JSON marshaling to flatten the command
func (c *Command) MarshalJSON() ([]byte, error) {
	result := map[string]interface{}{
		"T": c.T,
	}
	for k, v := range c.Data {
		result[k] = v
	}
	return json.Marshal(result)
}

// FeedbackData represents the feedback response from the RoArm
type FeedbackData struct {
	T     int     `json:"T"`
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	Tit   float64 `json:"tit"` // Pitch
	B     float64 `json:"b"`   // Joint 1 (base)
	S     float64 `json:"s"`   // Joint 2 (shoulder)
	E     float64 `json:"e"`   // Joint 3 (elbow)
	Wrist float64 `json:"t"`   // Joint 4 (wrist)
	R     float64 `json:"r"`   // Joint 5 (roll)
	G     float64 `json:"g"`   // Joint 6 (gripper)
	TB    float64 `json:"tB"`  // Torque Joint 1
	TS    float64 `json:"tS"`  // Torque Joint 2
	TE    float64 `json:"tE"`  // Torque Joint 3
	TT    float64 `json:"tT"`  // Torque Joint 4
	TR    float64 `json:"tR"`  // Torque Joint 5
	TG    float64 `json:"tG"`  // Torque Joint 6
}

// Controller handles communication with the WaveShare RoArm-M3
type Controller struct {
	mu            sync.Mutex
	logger        logging.Logger
	httpHost      string
	httpClient    *http.Client
	serialPort    serial.Port
	isHTTP        bool
	httpTimeout   time.Duration
	serialTimeout time.Duration
	verboseWire   bool
	// canReadFeedback is true when this transport can answer a T:105
	// feedback request. Serial always can; HTTP depends on the firmware.
	canReadFeedback bool
	// resetFailures counts consecutive ResetInputBuffer failures. Guarded by
	// mu, since serialWrite (the only place it's touched) is only called
	// with mu held.
	resetFailures int
	// health holds the link health counters. See health.go.
	health HealthSnapshot
	// clock is the wall clock a settle polls against. The zero value is the
	// real clock; tests substitute it to drive settle timing.
	clock Clock
}

// Config represents the configuration for the RoArm controller
type Config struct {
	// HTTP configuration
	Host string `json:"host,omitempty"`

	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Common configuration
	HTTPTimeout   Duration       `json:"http_timeout,omitempty"`
	SerialTimeout Duration       `json:"serial_timeout,omitempty"`
	Logger        logging.Logger `json:"-"` // Logger for debugging
}

// NewController creates a new RoArm controller
func NewController(config *Config) (*Controller, error) {
	controller := &Controller{
		httpTimeout:   DefaultHTTPTimeout,
		serialTimeout: DefaultSerialTimeout,
		logger:        config.Logger,
		verboseWire:   os.Getenv("ROARM_WIRE_TRACE") == "1",
	}

	// Use default logger if none provided
	if controller.logger == nil {
		controller.logger = logging.NewLogger("roarm_controller")
	}

	if config.HTTPTimeout.ToStdDuration() > 0 {
		controller.httpTimeout = config.HTTPTimeout.ToStdDuration()
	}
	if config.SerialTimeout.ToStdDuration() > 0 {
		controller.serialTimeout = config.SerialTimeout.ToStdDuration()
	}

	// Determine communication method
	if config.Host != "" {
		// HTTP mode
		controller.isHTTP = true
		controller.canReadFeedback = HTTPSupportsFeedback
		controller.httpHost = config.Host
		// No Timeout here: each request's deadline comes from its context
		// (httpGet's per-request WithTimeout, itself now bounded per attempt
		// by queryWithRetries), so a slow attempt doesn't also need a second,
		// client-wide clock racing it.
		controller.httpClient = &http.Client{}
	} else if config.Port != "" {
		// Serial mode
		controller.isHTTP = false
		controller.canReadFeedback = true
		baudrate := config.Baudrate
		if baudrate == 0 {
			baudrate = 115200 // Default baudrate
		}

		// Configure serial mode
		mode := &serial.Mode{
			BaudRate: baudrate,
			DataBits: 8,
			Parity:   serial.NoParity,
			StopBits: serial.OneStopBit,
		}

		port, err := serial.Open(config.Port, mode)
		if err != nil {
			return nil, fmt.Errorf("failed to open serial port: %w", err)
		}

		// Set read timeout. This is the port's per-read chunk timeout, not the
		// frame timeout: short, so serialReadFeedback's loop re-checks its
		// context and deadline often instead of blocking for a whole second.
		if err := port.SetReadTimeout(serialChunkTimeout); err != nil {
			port.Close()
			return nil, fmt.Errorf("failed to set read timeout: %w", err)
		}

		controller.serialPort = port
	} else {
		return nil, fmt.Errorf("must specify either host for HTTP or port for serial communication")
	}

	return controller, nil
}

// Close closes the controller connection
func (c *Controller) Close(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.isHTTP && c.serialPort != nil {
		return c.serialPort.Close()
	}
	return nil
}

// extractLastValidFeedback walks `}\r\n`-delimited frames in buf from most
// recent to oldest, attempting to parse each as FeedbackData. The firmware
// occasionally emits torn output — e.g. two partial frames concatenated
// with no `}\r\n{` boundary between them — which would make the outermost
// `{...}\r\n` window a corrupt blob. When that happens, skip the corrupt
// window and fall back to any clean earlier frame in the buffer rather
// than surfacing a parse error.
//
// Returns the parsed feedback, the raw JSON slice (for debug logging),
// and ok=true when a clean frame was found. ok=false means no complete,
// parseable frame exists yet — the caller should keep reading.
//
// A Controller receiver so every rejected candidate can be counted: a corrupt
// or incomplete frame counts as InvalidFrames, a well-formed frame of the
// wrong T counts as StaleFrames (audit 2.8 — traffic the module never asked
// for). The one production caller, serialReadFeedback, already holds c.mu.
func (c *Controller) extractLastValidFeedback(buf []byte) (*FeedbackData, []byte, bool) {
	frameEnd := []byte("}\r\n")
	frameStart := []byte("{")
	windowEnd := len(buf)
	for windowEnd > 0 {
		endIdx := bytes.LastIndex(buf[:windowEnd], frameEnd)
		if endIdx < 0 {
			return nil, nil, false
		}
		startIdx := bytes.LastIndex(buf[:endIdx], frameStart)
		if startIdx < 0 {
			// No `{` before this `}\r\n`; try earlier `}\r\n` terminators.
			windowEnd = endIdx
			continue
		}
		candidate := buf[startIdx : endIdx+1]
		if fb, ok, wrongT := parseFeedbackFrame(candidate); ok {
			return fb, candidate, true
		} else if wrongT {
			c.health.StaleFrames++
		} else {
			c.health.InvalidFrames++
		}
		// The candidate is corrupt, incomplete, or a frame the module did not
		// ask for; narrow the search to content strictly before its `{` so the
		// next iteration considers earlier frames.
		windowEnd = startIdx
	}
	return nil, nil, false
}

// feedbackRequiredKeys are the fields the module reads from every frame: the
// frame type and the six joint angles. A frame missing any of them is treated
// exactly like a corrupt one, because FeedbackData's value fields would
// otherwise turn a missing angle into a confident 0 (audit 2.3). The Cartesian
// and torque fields stay optional: only get_feedback reports them, and a
// missing one there is cosmetic.
var feedbackRequiredKeys = []string{"T", "b", "s", "e", "t", "r", "g"}

// parseFeedbackFrame decodes one frame and accepts it only when it is complete
// and its T is one the firmware sends in reply to a feedback request. Decoding
// through a map rather than straight into FeedbackData is what makes a missing
// field visible; every value in a frame is a number, so the map decode is exact.
// Note the decode error is checked BEFORE the required keys: json.Unmarshal
// leaves a map partly populated when it fails on a later value, so checking
// the keys first would accept a frame it had already rejected.
//
// wrongT separates a well-formed frame of the wrong type from a corrupt one:
// the first is an unsolicited frame the module did not ask for (audit 2.8) and
// the second is a torn one, and the health counters keep them apart.
func parseFeedbackFrame(raw []byte) (fb *FeedbackData, ok, wrongT bool) {
	var m map[string]float64
	if err := json.Unmarshal(raw, &m); err != nil {
		return nil, false, false
	}
	for _, k := range feedbackRequiredKeys {
		if _, present := m[k]; !present {
			return nil, false, false
		}
	}
	if !feedbackResponseTs[int(m["T"])] {
		return nil, false, true
	}
	var out FeedbackData
	if err := json.Unmarshal(raw, &out); err != nil {
		return nil, false, false
	}
	return &out, true, false
}

// feedbackResponseTs are the frame types the firmware sends in reply to a
// cmdFeedbackGet. Nothing else is ever waited for (see write vs query).
var feedbackResponseTs = map[int]bool{feedbackFrameT: true, cmdFeedbackGet: true}

// HTTPSupportsFeedback records whether this firmware's /js endpoint returns a
// T:1051 body for a T:105 request. Bench task B9 decides it. When false the
// controller runs in HTTP mode with canReadFeedback=false (see settle.go).
const HTTPSupportsFeedback = true

// ErrNoFeedback is returned by every position read on a transport that
// cannot return feedback. Its message contains NoFeedbackMarker so the
// gripper can recognise it after it has crossed the DoCommand boundary.
var ErrNoFeedback = errors.New(NoFeedbackMarker + "; position reads need a serial connection")

// write sends a control command and returns once it is on the wire. The
// firmware never answers these (only FEEDBACK_GET gets a reply), so waiting
// would only ever time out or mistake a command echo for an answer.
func (c *Controller) write(ctx context.Context, cmd *Command) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	cmdBytes, err := json.Marshal(cmd)
	if err != nil {
		return fmt.Errorf("failed to marshal command: %w", err)
	}
	if c.isHTTP {
		_, err := c.httpGet(ctx, cmdBytes)
		return err
	}
	return c.serialWrite(cmdBytes)
}

// query sends FEEDBACK_GET and waits for the T:1051 frame that answers it.
// Audit 2.5: a dropped frame is common enough (a 15-second settle can poll
// hundreds of times) that failing the whole operation on one is wrong, so
// each transport gets up to queryAttempts tries via queryWithRetries.
func (c *Controller) query(ctx context.Context) (*FeedbackData, error) {
	if !c.canReadFeedback {
		return nil, ErrNoFeedback
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	cmdBytes, err := json.Marshal(&Command{T: cmdFeedbackGet, Data: map[string]interface{}{}})
	if err != nil {
		return nil, fmt.Errorf("failed to marshal command: %w", err)
	}
	if c.isHTTP {
		return c.queryWithRetries(ctx, c.httpTimeout, func(attemptCtx context.Context) (*FeedbackData, error) {
			return c.queryOnceHTTP(attemptCtx, cmdBytes)
		})
	}
	return c.queryWithRetries(ctx, c.serialTimeout, func(attemptCtx context.Context) (*FeedbackData, error) {
		return c.queryOnce(attemptCtx, cmdBytes)
	})
}

// queryWithRetries runs attempt up to queryAttempts times: the same retry
// shape for both transports, just with each one's own per-attempt timeout
// and read. A non-fatal error (fatalTransport reports false) is retried
// after queryRetryDelay; a fatal one (closed port, unflushable port, short
// write) is returned immediately. mu is held by query for the whole call.
func (c *Controller) queryWithRetries(ctx context.Context, perAttempt time.Duration, attempt func(context.Context) (*FeedbackData, error)) (*FeedbackData, error) {
	var lastErr error
	attempts := 0
	for i := 0; i < queryAttempts; i++ {
		// The CALLER's context, never the per-attempt one: this is what
		// separates "the operation was cancelled" from "this frame timed out".
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if i > 0 {
			c.health.Retries++
			c.logger.Debugf("retrying the feedback request (attempt %d of %d) after: %v", i+1, queryAttempts, lastErr)
			if err := SleepCtx(ctx, queryRetryDelay); err != nil {
				return nil, err
			}
		}
		budget := c.attemptBudget(ctx, perAttempt)
		if budget <= 0 {
			break // not enough of the caller's deadline left to try again
		}
		attempts++
		attemptCtx, cancel := context.WithTimeout(ctx, budget)
		fb, err := attempt(attemptCtx)
		cancel()
		if err == nil {
			c.health.Frames++
			return fb, nil
		}
		lastErr = err
		if cerr := ctx.Err(); cerr != nil {
			return nil, cerr
		}
		if fatalTransport(err) {
			c.health.TransportErrors++
			c.noteError(err)
			return nil, err
		}
	}
	c.health.RetriesExhausted++
	c.noteError(lastErr)
	// attempts, not queryAttempts: a caller whose deadline ran out mid-loop
	// makes fewer, and a log claiming three tries when it made one sends the
	// next reader looking for a flaky link instead of a tight deadline.
	c.logger.Warnf("the feedback request failed %d times: %v", attempts, lastErr)
	return nil, fmt.Errorf("feedback request failed after %d attempts: %w", attempts, lastErr)
}

// attemptBudget is how long one attempt may take: the frame timeout, or
// whatever is left of the caller's deadline if that is shorter. The settle
// sets a per-poll deadline, which is what keeps a 300 ms settle from spending
// three frame timeouts inside one read.
func (c *Controller) attemptBudget(ctx context.Context, perAttempt time.Duration) time.Duration {
	budget := perAttempt
	if dl, ok := ctx.Deadline(); ok {
		if left := time.Until(dl); left < budget {
			budget = left
		}
	}
	return budget
}

// queryOnce writes FEEDBACK_GET and reads the reply: one attempt.
func (c *Controller) queryOnce(ctx context.Context, cmdBytes []byte) (*FeedbackData, error) {
	if err := c.serialWrite(cmdBytes); err != nil {
		return nil, err
	}
	return c.serialReadFeedback(ctx)
}

// queryOnceHTTP performs the /js feedback request and validates the reply:
// one attempt.
func (c *Controller) queryOnceHTTP(ctx context.Context, cmdBytes []byte) (*FeedbackData, error) {
	body, err := c.httpGet(ctx, cmdBytes)
	if err != nil {
		return nil, err
	}
	var fb FeedbackData
	if err := json.Unmarshal(body, &fb); err != nil || !feedbackResponseTs[fb.T] {
		return nil, fmt.Errorf("HTTP transport did not return a feedback frame (T=%d, body %q); this firmware may not support feedback over HTTP", fb.T, string(body))
	}
	return &fb, nil
}

// httpGet performs the /js?json= request and returns the raw body.
func (c *Controller) httpGet(ctx context.Context, cmdBytes []byte) ([]byte, error) {
	requestURL := fmt.Sprintf("http://%s/js?json=%s", c.httpHost, url.QueryEscape(string(cmdBytes)))
	reqCtx, cancel := context.WithTimeout(ctx, c.httpTimeout)
	defer cancel()
	req, err := http.NewRequestWithContext(reqCtx, "GET", requestURL, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create HTTP request: %w", err)
	}
	req.Header.Set("User-Agent", "roarm-go-client/1.0")
	req.Header.Set("Accept", "application/json")
	if c.verboseWire {
		c.logger.Debugf("HTTP GET %s", requestURL)
	}
	resp, err := c.httpClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("HTTP request failed: %w", err)
	}
	defer resp.Body.Close()
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("failed to read response: %w", err)
	}
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("HTTP request failed with status %d: %s", resp.StatusCode, string(body))
	}
	return body, nil
}

// serialWrite flushes stale input, then writes one newline-terminated frame.
func (c *Controller) serialWrite(cmdBytes []byte) error {
	cmdBytes = append(cmdBytes, '\n')
	if c.verboseWire {
		c.logger.Debugf("Sending serial command: %s", string(cmdBytes))
	}
	if err := c.serialPort.ResetInputBuffer(); err != nil {
		c.resetFailures++
		c.health.ResetFailures++
		// One failure is a warning; a second in a row means the module can no
		// longer tell a fresh frame from a stale one, which is the premise the
		// whole read path rests on (audit 2.7).
		if c.resetFailures >= 2 {
			return fmt.Errorf("%w (%d consecutive failures), so fresh and stale frames are indistinguishable: %w",
				errCannotFlushInput, c.resetFailures, err)
		}
		c.logger.Warnf("ResetInputBuffer failed, continuing once: %v", err)
	} else {
		c.resetFailures = 0
	}
	n, err := c.serialPort.Write(cmdBytes)
	if err != nil {
		return fmt.Errorf("failed to write to serial port: %w", err)
	}
	if n != len(cmdBytes) {
		err := fmt.Errorf("short write to serial port: %d of %d bytes; the firmware will discard the truncated command", n, len(cmdBytes))
		c.health.ShortWrites++
		c.noteError(err)
		return err
	}
	return nil
}

// serialReadFeedback reads until a T:1051 feedback frame arrives, dropping
// stale or echoed frames, or until the read times out or ctx is cancelled.
// It no longer retries a read error itself (audit 2.8): the port's read
// timeout (serialChunkTimeout) is short, so the n==0 "nothing yet" branch
// paces itself without a sleep, and a real read error is handed straight to
// queryWithRetries, which is the one place that now decides whether an
// attempt is worth repeating.
func (c *Controller) serialReadFeedback(ctx context.Context) (*FeedbackData, error) {
	// Read response with proper frame detection (based on Python ReadLine class)
	buffer := make([]byte, 256)
	responseBuffer := bytes.Buffer{}
	startTime := time.Now()
	// totalBudget is this attempt's frame timeout: whatever is left of the
	// caller's deadline (attemptCtx, set by queryWithRetries), falling back
	// to serialTimeout when ctx carries none (e.g. called directly in tests).
	totalBudget := c.serialTimeout
	if dl, ok := ctx.Deadline(); ok {
		totalBudget = time.Until(dl)
	}

	for {
		// Honor caller cancellation (e.g. Reconfigure/Close, RPC deadline).
		select {
		case <-ctx.Done():
			// Not a ReadTimeout: a cancelled caller is not a link fault, and
			// counting it would inflate the one counter that says whether the
			// cable is losing frames.
			return nil, ctx.Err()
		default:
		}

		// Check for timeout
		if time.Since(startTime) > totalBudget {
			c.health.ReadTimeouts++
			return nil, fmt.Errorf("timeout waiting for serial response")
		}

		// Read available data
		n, err := c.serialPort.Read(buffer)
		if err != nil {
			// A read timeout arrives as (0, nil), not an error; go.bug.st/serial
			// never returns io.EOF. Anything reaching here is a real transport
			// fault (e.g. an unplugged port), so hand it straight back rather
			// than retrying in a loop that would spin at full speed.
			return nil, fmt.Errorf("serial read: %w", err)
		}
		if n == 0 {
			// serialChunkTimeout (the port's read timeout) already paces this
			// branch; a sleep here would just add latency on top of it.
			continue
		}

		responseBuffer.Write(buffer[:n])
		if c.verboseWire {
			c.logger.Debugf("Received serial data: %s", string(buffer[:n]))
		}

		// Limit buffer size to prevent unbounded growth
		// Keep only the last maxFrameLength bytes. Writing a sub-slice of the
		// buffer's own array back into it after Reset is a forward copy, which
		// bytes.Buffer does with copy, so it needs no intermediate.
		if responseBuffer.Len() > maxFrameLength {
			data := responseBuffer.Bytes()
			responseBuffer.Reset()
			responseBuffer.Write(data[len(data)-maxFrameLength:])
		}

		// Look for the most recent valid JSON frame. When the firmware
		// emits a torn blob (two partial frames merged without a
		// `}\r\n{` boundary) extractLastValidFeedback walks earlier
		// `}\r\n` terminators, so we only surface clean frames.
		feedback, jsonData, ok := c.extractLastValidFeedback(responseBuffer.Bytes())
		if !ok {
			if c.verboseWire {
				c.logger.Debugf("no complete, valid feedback frame yet in: %s", responseBuffer.String())
			}
			continue
		}
		if c.verboseWire {
			c.logger.Debugf("Parsing JSON response: %s", string(jsonData))
		}

		return feedback, nil
	}
}

// WaitUntilSettled blocks until the masked joints reach their target or stop
// moving, deriving its own timing from the request (see settle.go). It owns
// the warnings, because it holds the logger and the retry counters.
func (c *Controller) WaitUntilSettled(ctx context.Context, req SettleRequest) (SettleResult, error) {
	if !c.canReadFeedback {
		// No feedback to poll: sleep the modelled duration, which is a closer
		// estimate than half the deadline (the deadline carries a 2x margin).
		plan, err := planSettle(req)
		if err != nil {
			return SettleResult{}, err
		}
		return SettleResult{Outcome: SettleArrived, Elapsed: plan.Duration, Deadline: plan.Deadline}, SleepCtx(ctx, plan.Duration)
	}

	before := c.retryCount()
	res, err := waitUntilSettled(ctx, c.GetJointRadians, c.clock, req)
	res.Retries = c.retryCount() - before
	c.noteSettle(res, err)
	if err != nil {
		return res, err
	}
	if res.Outcome == SettleStopped {
		remaining := MaxTravel(res.Positions, req.Target, req.Mask)
		c.logger.Warnf("the arm stopped %.1f deg short of its target after %v (%d polls); it may be loaded, obstructed, or at a joint limit",
			remaining*180/math.Pi, res.Elapsed.Round(time.Millisecond), res.Polls)
	}
	c.warnSettleTiming(res)
	return res, nil
}

// warnSettleTiming emits at most one timing warning per settle, in priority
// order: a settle near its budget means the arm is slower than the profile it
// was commanded with; a read slower than the poll interval means settle timing
// is dominated by read latency; retries mean the link is lossy.
func (c *Controller) warnSettleTiming(res SettleResult) {
	switch {
	case res.Elapsed > time.Duration(settleBudgetWarnFraction*float64(res.Deadline)):
		c.logger.Warnf("the settle used %v of its %v budget (%d polls); the arm is slower than its commanded profile implies, "+
			"so speed_degs_per_sec or acceleration_degs_per_sec_per_sec may not match reality",
			res.Elapsed.Round(time.Millisecond), res.Deadline.Round(time.Millisecond), res.Polls)
	case res.SlowestRead > settlePollInterval:
		c.logger.Warnf("the slowest position read in this settle took %v, longer than the %v poll interval; "+
			"settle timing is dominated by read latency and the effective poll rate is below the configured one",
			res.SlowestRead.Round(time.Millisecond), settlePollInterval)
	case res.Retries > 0:
		c.logger.Warnf("%d of %d position reads in this settle needed a retry; see the comms_health command",
			res.Retries, res.Polls)
	}
}

// IsMoving compares two position samples IsMovingProbeGap apart. On a
// transport that cannot read feedback it reports false.
func (c *Controller) IsMoving(ctx context.Context) (bool, error) {
	if !c.canReadFeedback {
		return false, nil
	}
	a, err := c.GetJointRadians(ctx)
	if err != nil {
		return false, err
	}
	if err := SleepCtx(ctx, IsMovingProbeGap); err != nil {
		return false, err
	}
	b, err := c.GetJointRadians(ctx)
	if err != nil {
		return false, err
	}
	return MaxTravel(a, b, nil) > StallRad, nil
}

// SetTorque enables or disables torque for all joints
func (c *Controller) SetTorque(ctx context.Context, enable bool) error {
	cmd := &Command{
		T: cmdTorqueSet,
		Data: map[string]interface{}{
			"cmd": 0,
		},
	}
	if enable {
		cmd.Data["cmd"] = 1
	}

	return c.write(ctx, cmd)
}

// SetLED controls the LED brightness (0-255)
func (c *Controller) SetLED(ctx context.Context, brightness int) error {
	if err := ValidateLEDBrightness(brightness); err != nil {
		return err
	}

	cmd := &Command{
		T: cmdLEDCtrl,
		Data: map[string]interface{}{
			"led": brightness,
		},
	}

	return c.write(ctx, cmd)
}

// SetJointRadian moves a single joint to the specified radian position
func (c *Controller) SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error {
	if joint < 1 || joint > 6 {
		return fmt.Errorf("joint must be 1-6, got %d", joint)
	}

	wireRadian := radian
	if joint == 6 {
		wireRadian = GripperSoftwareToWire(radian)
	}

	cmd := &Command{
		T: cmdJointRadianCtrl,
		Data: map[string]interface{}{
			"joint": joint,
			"rad":   wireRadian,
			"spd":   speed,
			"acc":   acc,
		},
	}

	if err := c.write(ctx, cmd); err != nil {
		return err
	}

	return nil
}

// SetJointRadians moves all joints to the specified radian positions
func (c *Controller) SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error {
	if len(radians) != 6 {
		return fmt.Errorf("expected 6 joint positions, got %d", len(radians))
	}

	cmd := &Command{
		T: cmdJointsRadianCtrl,
		Data: map[string]interface{}{
			"base":     radians[0],
			"shoulder": radians[1],
			"elbow":    radians[2],
			"wrist":    radians[3],
			"roll":     radians[4],
			"hand":     GripperSoftwareToWire(radians[5]),
			"spd":      speed,
			"acc":      acc,
		},
	}

	if err := c.write(ctx, cmd); err != nil {
		return err
	}

	return nil
}

// GetJointRadians returns the current joint positions in radians
func (c *Controller) GetJointRadians(ctx context.Context) ([]float64, error) {
	feedback, err := c.query(ctx)
	// query holds c.mu for its whole body under a defer; checkLinkHealth takes
	// the same mutex, so it must run only after query has returned and
	// released it — never from inside query.
	c.checkLinkHealth()
	if err != nil {
		return nil, err
	}

	radians := []float64{
		feedback.B,                        // Joint 1
		feedback.S,                        // Joint 2
		feedback.E,                        // Joint 3
		feedback.Wrist,                    // Joint 4
		feedback.R,                        // Joint 5
		GripperSoftwareToWire(feedback.G), // Joint 6 (gripper)
	}

	return radians, nil
}

// GetFeedback returns the full feedback data from the arm
func (c *Controller) GetFeedback(ctx context.Context) (*FeedbackData, error) {
	feedback, err := c.query(ctx)
	// See the comment in GetJointRadians: this must run after query returns.
	c.checkLinkHealth()
	return feedback, err
}

// ValidateLEDBrightness validates LED brightness parameter (0-255)
func ValidateLEDBrightness(brightness int) error {
	if brightness < 0 || brightness > 255 {
		return fmt.Errorf("LED brightness must be between 0 and 255, got %d", brightness)
	}
	return nil
}
