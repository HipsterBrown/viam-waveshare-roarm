package waveshareroarm

import (
	"fmt"
	"sync"
)

var (
	globalController *SafeRoArmController
	controllerMutex  sync.RWMutex
	refCount         int
	lastError        error
	currentConfig    *RoArmConfig
)

// SafeRoArmController wraps RoArmController with thread-safe operations
type SafeRoArmController struct {
	*RoArmController
	mu sync.RWMutex
}

// Thread-safe wrapper methods for all RoArmController operations

func (s *SafeRoArmController) SetTorque(enable bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.SetTorque(enable)
}

func (s *SafeRoArmController) SetLED(brightness int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.SetLED(brightness)
}

func (s *SafeRoArmController) MoveToHome() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.MoveToHome()
}

func (s *SafeRoArmController) SetJointRadian(joint int, radian float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.SetJointRadian(joint, radian, speed, acc)
}

func (s *SafeRoArmController) SetJointRadians(radians []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.SetJointRadians(radians, speed, acc)
}

func (s *SafeRoArmController) GetJointRadians() ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.RoArmController.GetJointRadians()
}

func (s *SafeRoArmController) SetGripperPosition(angleDegrees float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.SetGripperPosition(angleDegrees, speed, acc)
}

func (s *SafeRoArmController) GetGripperPosition() (float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.RoArmController.GetGripperPosition()
}

func (s *SafeRoArmController) GetFeedback() (*FeedbackData, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.RoArmController.GetFeedback()
}

func (s *SafeRoArmController) TestConnection() error {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.RoArmController.TestConnection()
}

func (s *SafeRoArmController) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.RoArmController.Close()
}

// configsEqual compares two RoArmConfig structs for equality
func configsEqual(a, b *RoArmConfig) bool {
	if a == nil && b == nil {
		return true
	}
	if a == nil || b == nil {
		return false
	}

	// Compare all fields except Logger (which can't be compared directly)
	return a.Host == b.Host &&
		a.Port == b.Port &&
		a.Baudrate == b.Baudrate &&
		a.Timeout == b.Timeout
}

// GetSharedController returns a shared controller instance with reference counting
func GetSharedController(config *RoArmConfig) (*SafeRoArmController, error) {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	// If we have a cached error and no controller, return the error
	if globalController == nil && lastError != nil {
		return nil, fmt.Errorf("cached controller creation error: %w", lastError)
	}

	// If controller exists, check config compatibility
	if globalController != nil {
		if !configsEqual(currentConfig, config) {
			return nil, fmt.Errorf("cannot create controller with different configuration while existing controller is in use (refCount: %d)", refCount)
		}
		refCount++
		return globalController, nil
	}

	// Create new controller
	controller, err := NewRoArmController(config)
	if err != nil {
		lastError = err
		return nil, err
	}

	// Wrap in thread-safe controller
	globalController = &SafeRoArmController{
		RoArmController: controller,
	}

	// Store config and clear any previous error
	currentConfig = &RoArmConfig{
		Host:     config.Host,
		Port:     config.Port,
		Baudrate: config.Baudrate,
		Timeout:  config.Timeout,
		Logger:   config.Logger,
	}
	lastError = nil
	refCount = 1

	return globalController, nil
}

// ReleaseSharedController decrements the reference count and closes controller if needed
func ReleaseSharedController() {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	refCount--
	if refCount <= 0 && globalController != nil {
		if err := globalController.Close(); err != nil {
			// Log error but don't panic - we're in cleanup mode
			if currentConfig != nil && currentConfig.Logger != nil {
				currentConfig.Logger.Warnf("Error closing shared controller: %v", err)
			}
		}
		globalController = nil
		currentConfig = nil
		refCount = 0
		lastError = nil
	}
}

// ForceCloseSharedController forcefully closes the shared controller regardless of reference count
// This should be used during graceful shutdown scenarios
func ForceCloseSharedController() error {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	var err error
	if globalController != nil {
		err = globalController.Close()
		globalController = nil
		currentConfig = nil
		refCount = 0
		lastError = nil
	}

	return err
}

// GetControllerStatus returns information about the current controller state
func GetControllerStatus() (refCount int, hasController bool, configSummary string) {
	controllerMutex.RLock()
	defer controllerMutex.RUnlock()

	hasController = globalController != nil

	if currentConfig != nil {
		if currentConfig.Host != "" {
			configSummary = fmt.Sprintf("HTTP: %s", currentConfig.Host)
		} else if currentConfig.Port != "" {
			configSummary = fmt.Sprintf("Serial: %s@%d", currentConfig.Port, currentConfig.Baudrate)
		}
	}

	return refCount, hasController, configSummary
}
