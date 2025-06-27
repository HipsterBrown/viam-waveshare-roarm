package waveshareroarm

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"net/url"
	"sync"
	"time"

	"go.bug.st/serial"
	"go.viam.com/rdk/logging"
)

const (
	// Command types matching the Python SDK
	ECHO_SET                     = 605
	MIDDLE_SET                   = 502
	LED_CTRL                     = 114
	TORQUE_SET                   = 210
	DYNAMIC_ADAPTATION_SET       = 112
	FEEDBACK_GET                 = 105
	JOINT_RADIAN_CTRL            = 101
	JOINTS_RADIAN_CTRL           = 102
	JOINT_ANGLE_CTRL             = 121
	JOINTS_ANGLE_CTRL            = 122
	GRIPPER_MODE_SET             = 222
	POSE_CTRL                    = 1041
	WIFI_ON_BOOT                 = 401
	AP_SET                       = 402
	STA_SET                      = 403
	APSTA_SET                    = 404
	WIFI_CONFIG_CREATE_BY_STATUS = 406
	WIFI_CONFIG_CREATE_BY_INPUT  = 407
	WIFI_STOP                    = 408

	// Default timeouts
	DefaultHTTPTimeout   = 5 * time.Second
	DefaultSerialTimeout = 1 * time.Second
)

// Joint limits for RoArm-M3 (in radians) - from waveshare_roarm_sdk utils.py
var (
	RoArmM3JointLimits = [][2]float64{
		{-3.3, 3.3}, // Joint 1: ±189° (wider than ±180°)
		{-1.9, 1.9}, // Joint 2: ±109° (wider than ±90°)
		{-1.2, 3.3}, // Joint 3: -69° to 189°
		{-1.9, 1.9}, // Joint 4: ±109° (wider than ±90°)
		{-3.3, 3.3}, // Joint 5: ±189° (wider than ±180°)
		{-0.2, 1.9}, // Joint 6: -11° to 109° (gripper)
	}

	// Angle limits for reference (in degrees) - from waveshare_roarm_sdk utils.py
	RoArmM3AngleLimits = [][2]float64{
		{-190, 190}, // Joint 1
		{-110, 110}, // Joint 2
		{-70, 190},  // Joint 3
		{-110, 110}, // Joint 4
		{-190, 190}, // Joint 5
		{-10, 100},  // Joint 6 (gripper)
	}
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
	T   int     `json:"T"`
	X   float64 `json:"x"`
	Y   float64 `json:"y"`
	Z   float64 `json:"z"`
	Tit float64 `json:"tit"` // Pitch
	B   float64 `json:"b"`   // Joint 1 (base)
	S   float64 `json:"s"`   // Joint 2 (shoulder)
	E   float64 `json:"e"`   // Joint 3 (elbow)
	T_  float64 `json:"t"`   // Joint 4 (wrist)
	R   float64 `json:"r"`   // Joint 5 (roll)
	G   float64 `json:"g"`   // Joint 6 (gripper)
	TB  float64 `json:"tB"`  // Torque Joint 1
	TS  float64 `json:"tS"`  // Torque Joint 2
	TE  float64 `json:"tE"`  // Torque Joint 3
	TT  float64 `json:"tT"`  // Torque Joint 4
	TR  float64 `json:"tR"`  // Torque Joint 5
	TG  float64 `json:"tG"`  // Torque Joint 6
}

// RoArmController handles communication with the WaveShare RoArm-M3
type RoArmController struct {
	mu         sync.RWMutex
	logger     logging.Logger
	httpHost   string
	httpClient *http.Client
	serialPort serial.Port
	isHTTP     bool
	timeout    time.Duration
}

// RoArmConfig represents the configuration for the RoArm controller
type RoArmConfig struct {
	// HTTP configuration
	Host string `json:"host,omitempty"`

	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Common configuration
	Timeout time.Duration  `json:"timeout,omitempty"`
	Logger  logging.Logger `json:"-"` // Logger for debugging
}

// NewRoArmController creates a new RoArm controller
func NewRoArmController(config *RoArmConfig) (*RoArmController, error) {
	controller := &RoArmController{
		timeout: DefaultHTTPTimeout,
		logger:  config.Logger,
	}

	// Use default logger if none provided
	if controller.logger == nil {
		controller.logger = logging.NewLogger("roarm_controller")
	}

	if config.Timeout > 0 {
		controller.timeout = config.Timeout
	}

	// Determine communication method
	if config.Host != "" {
		// HTTP mode
		controller.isHTTP = true
		controller.httpHost = config.Host
		controller.httpClient = &http.Client{
			Timeout: controller.timeout,
		}
	} else if config.Port != "" {
		// Serial mode
		controller.isHTTP = false
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
		if err := port.SetReadTimeout(DefaultSerialTimeout); err != nil {
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
func (c *RoArmController) Close() error {
	if !c.isHTTP && c.serialPort != nil {
		return c.serialPort.Close()
	}
	return nil
}

// sendCommand sends a command to the RoArm and returns the response
func (c *RoArmController) sendCommand(cmd *Command) (*FeedbackData, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	cmdBytes, err := json.Marshal(cmd)
	if err != nil {
		return nil, fmt.Errorf("failed to marshal command: %w", err)
	}

	if c.isHTTP {
		return c.sendHTTPCommand(cmdBytes)
	}
	return c.sendSerialCommand(cmdBytes)
}

// sendHTTPCommand sends a command via HTTP
func (c *RoArmController) sendHTTPCommand(cmdBytes []byte) (*FeedbackData, error) {
	// URL encode the JSON command
	encodedCmd := url.QueryEscape(string(cmdBytes))
	requestURL := fmt.Sprintf("http://%s/js?json=%s", c.httpHost, encodedCmd)

	// Create request with context for timeout
	ctx, cancel := context.WithTimeout(context.Background(), c.timeout)
	defer cancel()

	req, err := http.NewRequestWithContext(ctx, "GET", requestURL, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create HTTP request: %w", err)
	}

	// Set appropriate headers
	req.Header.Set("User-Agent", "roarm-go-client/1.0")
	req.Header.Set("Accept", "application/json")

	c.logger.Debugf("Sending HTTP request: %s", requestURL)

	resp, err := c.httpClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("HTTP request failed: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(resp.Body)
		return nil, fmt.Errorf("HTTP request failed with status %d: %s", resp.StatusCode, string(body))
	}

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("failed to read response: %w", err)
	}

	c.logger.Debugf("Received HTTP response: %s", string(body))

	var feedback FeedbackData
	if err := json.Unmarshal(body, &feedback); err != nil {
		return nil, fmt.Errorf("failed to unmarshal response: %w, body: %s", err, string(body))
	}

	return &feedback, nil
}

// sendSerialCommand sends a command via serial port
func (c *RoArmController) sendSerialCommand(cmdBytes []byte) (*FeedbackData, error) {
	// Add newline to command
	cmdBytes = append(cmdBytes, '\n')

	c.logger.Debugf("Sending serial command: %s", string(cmdBytes))

	// Write command
	_, err := c.serialPort.Write(cmdBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to write to serial port: %w", err)
	}

	// Read response with proper frame detection (based on Python ReadLine class)
	buffer := make([]byte, 256)
	responseBuffer := bytes.Buffer{}
	frameStart := []byte("{")
	frameEnd := []byte("}\r\n")
	maxFrameLength := 512
	startTime := time.Now()

	for {
		// Check for timeout
		if time.Since(startTime) > c.timeout {
			return nil, fmt.Errorf("timeout waiting for serial response")
		}

		// Read available data
		n, err := c.serialPort.Read(buffer)
		if err != nil {
			// Handle timeout gracefully - the serial library manages read timeouts
			if time.Since(startTime) > c.timeout {
				return nil, fmt.Errorf("timeout reading from serial port")
			}
			continue // Keep trying until our overall timeout
		}

		if n > 0 {
			responseBuffer.Write(buffer[:n])
			c.logger.Debugf("Received serial data: %s", string(buffer[:n]))

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
		}

		// Look for complete JSON frame
		response := responseBuffer.Bytes()
		endIndex := bytes.LastIndex(response, frameEnd)

		if endIndex >= 0 {
			// Find the start of the JSON object before the end
			startIndex := bytes.LastIndex(response[:endIndex], frameStart)
			if startIndex >= 0 && startIndex < endIndex {
				// Extract the JSON data
				jsonData := response[startIndex : endIndex+1] // Include the closing brace

				c.logger.Debugf("Parsing JSON response: %s", string(jsonData))

				var feedback FeedbackData
				if err := json.Unmarshal(jsonData, &feedback); err != nil {
					// Log the error but continue trying to read more data
					c.logger.Warnf("Failed to unmarshal JSON response: %v, data: %s", err, string(jsonData))
					// Clear the buffer and continue
					responseBuffer.Reset()
					continue
				}

				return &feedback, nil
			}
		}
	}
}

// SetTorque enables or disables torque for all joints
func (c *RoArmController) SetTorque(enable bool) error {
	cmd := &Command{
		T: TORQUE_SET,
		Data: map[string]interface{}{
			"cmd": 0,
		},
	}
	if enable {
		cmd.Data["cmd"] = 1
	}

	_, err := c.sendCommand(cmd)
	return err
}

// SetLED controls the LED brightness (0-255)
func (c *RoArmController) SetLED(brightness int) error {
	if err := ValidateLEDBrightness(brightness); err != nil {
		return err
	}

	cmd := &Command{
		T: LED_CTRL,
		Data: map[string]interface{}{
			"led": brightness,
		},
	}

	_, err := c.sendCommand(cmd)
	return err
}

// MoveToHome moves the arm to the home position
func (c *RoArmController) MoveToHome() error {
	// Home position: all joints at 0 except joint 3 at 90 degrees
	homePositions := []float64{0, 0, math.Pi / 2, 0, 0, 0}
	return c.SetJointRadians(homePositions, 100, 50)
}

// SetJointRadian moves a single joint to the specified radian position
func (c *RoArmController) SetJointRadian(joint int, radian float64, speed, acc int) error {
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

	// Validate joint limits
	limits := RoArmM3JointLimits[joint-1]
	if radian < limits[0] || radian > limits[1] {
		return fmt.Errorf("joint %d radian %.3f out of range [%.3f, %.3f]",
			joint, radian, limits[0], limits[1])
	}

	// Apply coordinate transformation for gripper (joint 6) - matching Python SDK
	transformedRadian := radian
	if joint == 6 {
		transformedRadian = math.Pi - radian
	}

	cmd := &Command{
		T: JOINT_RADIAN_CTRL,
		Data: map[string]interface{}{
			"joint": joint,
			"rad":   transformedRadian,
			"spd":   speed,
			"acc":   acc,
		},
	}

	_, err := c.sendCommand(cmd)
	return err
}

// SetJointRadians moves all joints to the specified radian positions
func (c *RoArmController) SetJointRadians(radians []float64, speed, acc int) error {
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

	// Validate all joint limits
	for i, radian := range radians {
		limits := RoArmM3JointLimits[i]
		if radian < limits[0] || radian > limits[1] {
			return fmt.Errorf("joint %d radian %.3f out of range [%.3f, %.3f]",
				i+1, radian, limits[0], limits[1])
		}
	}

	// Apply coordinate transformation for gripper (joint 6) - matching Python SDK
	transformedRadians := make([]float64, 6)
	copy(transformedRadians, radians)
	transformedRadians[5] = math.Pi - transformedRadians[5] // Joint 6 (gripper)

	cmd := &Command{
		T: JOINTS_RADIAN_CTRL,
		Data: map[string]interface{}{
			"base":     transformedRadians[0],
			"shoulder": transformedRadians[1],
			"elbow":    transformedRadians[2],
			"wrist":    transformedRadians[3],
			"roll":     transformedRadians[4],
			"hand":     transformedRadians[5],
			"spd":      speed,
			"acc":      acc,
		},
	}

	_, err := c.sendCommand(cmd)
	return err
}

// GetJointRadians returns the current joint positions in radians
func (c *RoArmController) GetJointRadians() ([]float64, error) {
	cmd := &Command{
		T:    FEEDBACK_GET,
		Data: map[string]interface{}{},
	}

	feedback, err := c.sendCommand(cmd)
	if err != nil {
		return nil, err
	}

	// Extract joint positions and apply coordinate transformations
	radians := []float64{
		feedback.B,           // Joint 1
		feedback.S,           // Joint 2
		feedback.E,           // Joint 3
		feedback.T_,          // Joint 4
		feedback.R,           // Joint 5
		math.Pi - feedback.G, // Joint 6 (gripper) - reverse transformation
	}

	return radians, nil
}

// SetGripperPosition sets the gripper position (-10 to 100 degrees)
func (c *RoArmController) SetGripperPosition(angleDegrees float64, speed, acc int) error {
	if angleDegrees < -10 || angleDegrees > 100 {
		return fmt.Errorf("gripper angle must be -10 to 100 degrees, got %.1f", angleDegrees)
	}

	radians := angleDegrees * math.Pi / 180.0
	return c.SetJointRadian(6, radians, speed, acc)
}

// GetGripperPosition returns the current gripper position in degrees
func (c *RoArmController) GetGripperPosition() (float64, error) {
	radians, err := c.GetJointRadians()
	if err != nil {
		return 0, err
	}

	angleDegrees := radians[5] * 180.0 / math.Pi
	return angleDegrees, nil
}

// GetFeedback returns the full feedback data from the arm
func (c *RoArmController) GetFeedback() (*FeedbackData, error) {
	cmd := &Command{
		T:    FEEDBACK_GET,
		Data: map[string]interface{}{},
	}

	return c.sendCommand(cmd)
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
func (c *RoArmController) TestConnection() error {
	c.logger.Debug("Testing connection...")

	_, err := c.GetFeedback()
	if err != nil {
		return fmt.Errorf("connection test failed: %w", err)
	}

	c.logger.Debug("Connection test successful")
	return nil
}

// GetAvailableSerialPorts returns a list of available serial ports
func GetAvailableSerialPorts() ([]string, error) {
	return serial.GetPortsList()
}
