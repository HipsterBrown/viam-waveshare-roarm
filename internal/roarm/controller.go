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
)

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
		controller.httpClient = &http.Client{
			Timeout: controller.httpTimeout,
		}
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

		// Set read timeout
		if err := port.SetReadTimeout(controller.serialTimeout); err != nil {
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
func extractLastValidFeedback(buf []byte) (*FeedbackData, []byte, bool) {
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
		if fb, ok, _ := parseFeedbackFrame(candidate); ok {
			return fb, candidate, true
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
	if err := c.serialWrite(cmdBytes); err != nil {
		return nil, err
	}
	return c.serialReadFeedback(ctx)
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
		// One failure is a warning; a second in a row means the module can no
		// longer tell a fresh frame from a stale one, which is the premise the
		// whole read path rests on (audit 2.7).
		if c.resetFailures >= 2 {
			return fmt.Errorf("cannot flush the serial input buffer (%d consecutive failures), so fresh and stale frames are indistinguishable: %w", c.resetFailures, err)
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
		return fmt.Errorf("short write to serial port: %d of %d bytes; the firmware will discard the truncated command", n, len(cmdBytes))
	}
	return nil
}

// serialReadFeedback reads until a T:1051 feedback frame arrives, dropping
// stale or echoed frames, or until the read times out or ctx is cancelled.
func (c *Controller) serialReadFeedback(ctx context.Context) (*FeedbackData, error) {
	// Read response with proper frame detection (based on Python ReadLine class)
	buffer := make([]byte, 256)
	responseBuffer := bytes.Buffer{}
	maxFrameLength := 512
	startTime := time.Now()

	for {
		// Honor caller cancellation (e.g. Reconfigure/Close, RPC deadline).
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		default:
		}

		// Check for timeout
		if time.Since(startTime) > c.serialTimeout {
			return nil, fmt.Errorf("timeout waiting for serial response")
		}

		// Read available data
		n, err := c.serialPort.Read(buffer)
		if err != nil {
			if errors.Is(err, io.EOF) {
				return nil, fmt.Errorf("serial port closed or unplugged: %w", err)
			}
			if time.Since(startTime) > c.serialTimeout {
				return nil, fmt.Errorf("timeout reading from serial port: %w", err)
			}
			c.logger.Warnf("serial read error (will retry): %v", err)
			time.Sleep(10 * time.Millisecond)
			continue
		}
		if n == 0 {
			time.Sleep(1 * time.Millisecond)
			continue
		}

		responseBuffer.Write(buffer[:n])
		if c.verboseWire {
			c.logger.Debugf("Received serial data: %s", string(buffer[:n]))
		}

		// Limit buffer size to prevent unbounded growth
		if responseBuffer.Len() > maxFrameLength {
			// Keep only the last maxFrameLength bytes
			data := responseBuffer.Bytes()
			responseBuffer.Reset()
			if len(data) > maxFrameLength {
				responseBuffer.Write(data[len(data)-maxFrameLength:])
			} else {
				responseBuffer.Write(data)
			}
		}

		// Look for the most recent valid JSON frame. When the firmware
		// emits a torn blob (two partial frames merged without a
		// `}\r\n{` boundary) extractLastValidFeedback walks earlier
		// `}\r\n` terminators, so we only surface clean frames.
		feedback, jsonData, ok := extractLastValidFeedback(responseBuffer.Bytes())
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

// WaitUntilSettled blocks until the masked joints reach target or stop
// moving. See settle.go. On a transport that cannot read feedback it sleeps
// the plain time estimate (half the timeout) and returns nil positions.
func (c *Controller) WaitUntilSettled(ctx context.Context, target []float64, mask []bool, timeout time.Duration) ([]float64, error) {
	if !c.canReadFeedback {
		return nil, SleepCtx(ctx, timeout/2)
	}
	pos, stalled, err := waitUntilSettled(ctx, c.GetJointRadians, SleepCtx, target, mask, timeout)
	if stalled {
		c.logger.Debugf("settle: joints stopped short of target (at %v, wanted %v)", pos, target)
	}
	return pos, err
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
	return c.query(ctx)
}

// ValidateLEDBrightness validates LED brightness parameter (0-255)
func ValidateLEDBrightness(brightness int) error {
	if brightness < 0 || brightness > 255 {
		return fmt.Errorf("LED brightness must be between 0 and 255, got %d", brightness)
	}
	return nil
}
