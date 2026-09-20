package waveshareroarm

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

// gripperSoftwareToWire converts joint 6 between the software reference frame
// (used by Viam APIs and joint limits) and the firmware wire frame. The
// firmware reports raw "closed" at roughly π rad, so the two frames are
// related by r_wire = π - r_software. The transform is its own inverse, so
// the same function converts either direction.
func gripperSoftwareToWire(r float64) float64 { return math.Pi - r }

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

// gripperJointLimits is joint 6's software-frame range in radians (about
// -11.5 to 109 degrees). Joints 1-5 take their limits from roarm_m3.json.
var gripperJointLimits = [2]float64{-0.2, 1.9}

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

// RoArmController handles communication with the WaveShare RoArm-M3
type RoArmController struct {
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
}

// RoArmConfig represents the configuration for the RoArm controller
type RoArmConfig struct {
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

// NewRoArmController creates a new RoArm controller
func NewRoArmController(config *RoArmConfig) (*RoArmController, error) {
	controller := &RoArmController{
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
		controller.canReadFeedback = httpSupportsFeedback
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
func (c *RoArmController) Close(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.isHTTP && c.serialPort != nil {
		return c.serialPort.Close()
	}
	return nil
}

// parseLastJSONFrame finds the last complete JSON frame in buf, delimited
// by '{' and "}\r\n". Returns the JSON bytes (including the closing brace)
// and true if a complete frame is present.
func parseLastJSONFrame(buf []byte) (jsonData []byte, ok bool) {
	frameStart := []byte("{")
	frameEnd := []byte("}\r\n")
	endIndex := bytes.LastIndex(buf, frameEnd)
	if endIndex < 0 {
		return nil, false
	}
	startIndex := bytes.LastIndex(buf[:endIndex], frameStart)
	if startIndex < 0 || startIndex >= endIndex {
		return nil, false
	}
	return buf[startIndex : endIndex+1], true
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
		var fb FeedbackData
		if err := json.Unmarshal(candidate, &fb); err == nil {
			return &fb, candidate, true
		}
		// Candidate is corrupt — narrow the search to content strictly
		// before its `{` so the next iteration considers earlier frames.
		windowEnd = startIdx
	}
	return nil, nil, false
}

// feedbackResponseTs are the frame types the firmware sends in reply to a
// cmdFeedbackGet. Nothing else is ever waited for (see write vs query).
var feedbackResponseTs = map[int]bool{feedbackFrameT: true, cmdFeedbackGet: true}

// httpSupportsFeedback records whether this firmware's /js endpoint returns a
// T:1051 body for a T:105 request. Bench task B9 decides it. When false the
// controller runs in HTTP mode with canReadFeedback=false (see settle.go).
const httpSupportsFeedback = true

// errNoFeedback is returned by every position read on a transport that
// cannot return feedback. Its message contains noFeedbackMarker so the
// gripper can recognise it after it has crossed the DoCommand boundary.
var errNoFeedback = errors.New(noFeedbackMarker + "; position reads need a serial connection")

// write sends a control command and returns once it is on the wire. The
// firmware never answers these (only FEEDBACK_GET gets a reply), so waiting
// would only ever time out or mistake a command echo for an answer.
func (c *RoArmController) write(ctx context.Context, cmd *Command) error {
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
func (c *RoArmController) query(ctx context.Context) (*FeedbackData, error) {
	if !c.canReadFeedback {
		return nil, errNoFeedback
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
func (c *RoArmController) httpGet(ctx context.Context, cmdBytes []byte) ([]byte, error) {
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
func (c *RoArmController) serialWrite(cmdBytes []byte) error {
	cmdBytes = append(cmdBytes, '\n')
	if c.verboseWire {
		c.logger.Debugf("Sending serial command: %s", string(cmdBytes))
	}
	if err := c.serialPort.ResetInputBuffer(); err != nil {
		c.logger.Warnf("ResetInputBuffer failed: %v", err)
	}
	if _, err := c.serialPort.Write(cmdBytes); err != nil {
		return fmt.Errorf("failed to write to serial port: %w", err)
	}
	return nil
}

// serialReadFeedback reads until a T:1051 feedback frame arrives, dropping
// stale or echoed frames, or until the read times out or ctx is cancelled.
func (c *RoArmController) serialReadFeedback(ctx context.Context) (*FeedbackData, error) {
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
			continue
		}
		if c.verboseWire {
			c.logger.Debugf("Parsing JSON response: %s", string(jsonData))
		}

		// Only accept response frames whose T matches what the firmware
		// is expected to send for this request T. Stale streaming frames
		// (e.g. unsolicited 1051 feedback) are dropped and we keep reading.
		if !feedbackResponseTs[feedback.T] {
			c.logger.Warnf("dropping stale/unexpected frame T=%d while waiting for feedback", feedback.T)
			responseBuffer.Reset()
			continue
		}

		return feedback, nil
	}
}

// WaitUntilSettled blocks until the masked joints reach target or stop
// moving. See settle.go. On a transport that cannot read feedback it sleeps
// the plain time estimate (half the timeout) and returns nil positions.
func (c *RoArmController) WaitUntilSettled(ctx context.Context, target []float64, mask []bool, timeout time.Duration) ([]float64, error) {
	if !c.canReadFeedback {
		return nil, sleepCtx(ctx, timeout/2)
	}
	pos, stalled, err := waitUntilSettled(ctx, c.GetJointRadians, sleepCtx, target, mask, timeout)
	if stalled {
		c.logger.Debugf("settle: joints stopped short of target (at %v, wanted %v)", pos, target)
	}
	return pos, err
}

// IsMoving compares two position samples isMovingProbeGap apart. On a
// transport that cannot read feedback it reports false.
func (c *RoArmController) IsMoving(ctx context.Context) (bool, error) {
	if !c.canReadFeedback {
		return false, nil
	}
	a, err := c.GetJointRadians(ctx)
	if err != nil {
		return false, err
	}
	if err := sleepCtx(ctx, isMovingProbeGap); err != nil {
		return false, err
	}
	b, err := c.GetJointRadians(ctx)
	if err != nil {
		return false, err
	}
	return maxTravel(a, b, nil) > stallRad, nil
}

// SetTorque enables or disables torque for all joints
func (c *RoArmController) SetTorque(ctx context.Context, enable bool) error {
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
func (c *RoArmController) SetLED(ctx context.Context, brightness int) error {
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
func (c *RoArmController) SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error {
	if joint < 1 || joint > 6 {
		return fmt.Errorf("joint must be 1-6, got %d", joint)
	}

	// Validate speed and acceleration
	if err := ValidateSpeed(speed); err != nil {
		return err
	}
	if err := ValidateAcceleration(acc); err != nil {
		return err
	}

	wireRadian := radian
	if joint == 6 {
		wireRadian = gripperSoftwareToWire(radian)
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
func (c *RoArmController) SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error {
	if len(radians) != 6 {
		return fmt.Errorf("expected 6 joint positions, got %d", len(radians))
	}

	// Validate speed and acceleration parameters
	if err := ValidateSpeed(speed); err != nil {
		return err
	}
	if err := ValidateAcceleration(acc); err != nil {
		return err
	}

	cmd := &Command{
		T: cmdJointsRadianCtrl,
		Data: map[string]interface{}{
			"base":     radians[0],
			"shoulder": radians[1],
			"elbow":    radians[2],
			"wrist":    radians[3],
			"roll":     radians[4],
			"hand":     gripperSoftwareToWire(radians[5]),
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
func (c *RoArmController) GetJointRadians(ctx context.Context) ([]float64, error) {
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
		gripperSoftwareToWire(feedback.G), // Joint 6 (gripper)
	}

	return radians, nil
}

// GetFeedback returns the full feedback data from the arm
func (c *RoArmController) GetFeedback(ctx context.Context) (*FeedbackData, error) {
	return c.query(ctx)
}

// ValidateSpeed validates speed parameter (1-4096 as per SDK)
func ValidateSpeed(speed int) error {
	if speed < 1 || speed > 4096 {
		return fmt.Errorf("speed must be between 1 and 4096, got %d", speed)
	}
	return nil
}

// ValidateAcceleration validates acceleration parameter (1-254 as per SDK)
func ValidateAcceleration(acc int) error {
	if acc < 1 || acc > 254 {
		return fmt.Errorf("acceleration must be between 1 and 254, got %d", acc)
	}
	return nil
}

// ValidateLEDBrightness validates LED brightness parameter (0-255)
func ValidateLEDBrightness(brightness int) error {
	if brightness < 0 || brightness > 255 {
		return fmt.Errorf("LED brightness must be between 0 and 255, got %d", brightness)
	}
	return nil
}

// TestConnection tests the connection by sending a feedback request
func (c *RoArmController) TestConnection(ctx context.Context) error {
	c.logger.Debug("Testing connection...")

	_, err := c.GetFeedback(ctx)
	if err != nil {
		return fmt.Errorf("connection test failed: %w", err)
	}

	c.logger.Debug("Connection test successful")
	return nil
}
