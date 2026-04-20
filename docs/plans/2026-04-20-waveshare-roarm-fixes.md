# Waveshare RoArm-M3 Remediation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Address all Critical, Important, and Minor findings from the 2026-04-20 code review of the `viam-waveshare-roarm` module.

**Architecture:** Delete the global-singleton serial manager in favor of gripper-depends-on-arm dependency injection with a narrow `RoArmHandle` interface. Replace the fake `time.Sleep` motion model with a real `SingleOperationManager` + position-delta `IsMoving` tracker and a hold-current-position `Stop`. Fix protocol correctness (input-buffer reset, response T-correlation, gripper transform). Extract speed/accel conversions for later hardware calibration via a rewritten `cmd/cli`. Add comprehensive unit tests.

**Tech Stack:** Go 1.x, Viam Go SDK (`go.viam.com/rdk`), `go.bug.st/serial`, standard `net/http`, `testing` + `testify`.

**Reference:** See `docs/plans/2026-04-20-waveshare-roarm-fixes-design.md` for architectural context.

---

## Conventions

- **TDD:** For any new/changed behavior, write the test first, run it failing, implement, run it passing, commit.
- **Commits:** One task = one commit. Use conventional-commit prefix (`feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `chore:`).
- **Verification after each task:** `go build ./...` and `go vet ./...` must pass. If tests exist for the touched files, `go test ./...` must pass.
- **Running tests:** From repo root, `go test ./... -v`.
- **No hardware required** for Phases 1–6. Phase 7 is hardware-gated and optional.

---

## Phase 1 — Config + Validate + meta cleanup

### Task 1.1: Duration type that parses strings and numbers

**Files:**
- Create: `duration.go`
- Test: `duration_test.go`

**Step 1: Write failing test**

`duration_test.go`:
```go
package waveshareroarm

import (
	"encoding/json"
	"testing"
	"time"
)

func TestDurationUnmarshalString(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`"5s"`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != 5*time.Second {
		t.Fatalf("got %v, want 5s", time.Duration(d))
	}
}

func TestDurationUnmarshalNumber(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`1000000000`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != time.Second {
		t.Fatalf("got %v, want 1s", time.Duration(d))
	}
}

func TestDurationUnmarshalEmpty(t *testing.T) {
	var d Duration
	if err := json.Unmarshal([]byte(`""`), &d); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if time.Duration(d) != 0 {
		t.Fatalf("got %v, want 0", time.Duration(d))
	}
}
```

**Step 2: Run test, expect failure** (undefined `Duration`).

```
go test ./... -run TestDuration -v
```

**Step 3: Implement**

`duration.go`:
```go
package waveshareroarm

import (
	"encoding/json"
	"fmt"
	"time"
)

// Duration wraps time.Duration so it can be configured as either a
// human-readable string (e.g. "5s") or a raw nanosecond integer.
type Duration time.Duration

func (d *Duration) UnmarshalJSON(data []byte) error {
	if len(data) == 0 || string(data) == "null" {
		return nil
	}
	// Try integer nanoseconds first.
	var n int64
	if err := json.Unmarshal(data, &n); err == nil {
		*d = Duration(n)
		return nil
	}
	var s string
	if err := json.Unmarshal(data, &s); err != nil {
		return fmt.Errorf("duration must be string or number: %w", err)
	}
	if s == "" {
		*d = 0
		return nil
	}
	parsed, err := time.ParseDuration(s)
	if err != nil {
		return fmt.Errorf("invalid duration %q: %w", s, err)
	}
	*d = Duration(parsed)
	return nil
}

func (d Duration) MarshalJSON() ([]byte, error) {
	return json.Marshal(time.Duration(d).String())
}

func (d Duration) ToStdDuration() time.Duration { return time.Duration(d) }
```

**Step 4: Run test, expect pass**

**Step 5: Commit**

```bash
git add duration.go duration_test.go
git commit -m "feat: add Duration type accepting both string and numeric JSON"
```

---

### Task 1.2: Migrate `RoArmM3Config.Timeout` to the new Duration type

**Files:**
- Modify: `arm.go` (RoArmM3Config, newRoArmM3)
- Modify: `controller.go` (RoArmConfig, NewRoArmController, timeout math)

**Steps:**

1. In `arm.go`, change `Timeout time.Duration \`json:"timeout,omitempty"\`` to `Timeout Duration \`json:"timeout,omitempty"\``. Pass `conf.Timeout.ToStdDuration()` when constructing controllerConfig.
2. In `controller.go`, change `RoArmConfig.Timeout` type to `Duration`, update `NewRoArmController` to call `.ToStdDuration()` before assigning to `controller.timeout`.
3. Run `go build ./...`. Expect clean.
4. Commit:

```bash
git add arm.go controller.go
git commit -m "refactor: use Duration type for Timeout config field"
```

---

### Task 1.3: Baudrate whitelist in `Validate`

**Files:**
- Modify: `arm.go` (`RoArmM3Config.Validate`)
- Create test: `config_test.go`

**Step 1: Test first**

`config_test.go`:
```go
package waveshareroarm

import "testing"

func TestArmValidateRejectsUnknownBaudrate(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 1152000}
	_, _, err := cfg.Validate("arms.0")
	if err == nil {
		t.Fatal("expected error for bad baudrate")
	}
}

func TestArmValidateAcceptsKnownBaudrate(t *testing.T) {
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 115200}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestArmValidateAcceptsZeroBaudrate(t *testing.T) {
	// Zero means default; should not error.
	cfg := &RoArmM3Config{Port: "/dev/ttyUSB0", Baudrate: 0}
	_, _, err := cfg.Validate("arms.0")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}
```

**Step 2: Run; expect fail**

**Step 3: Implement**

In `arm.go`, extend `Validate`:
```go
var validBaudrates = map[int]bool{
	0: true, // treat as default
	9600: true, 19200: true, 38400: true, 57600: true,
	115200: true, 230400: true, 921600: true, 1000000: true,
}

func (cfg *RoArmM3Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" && cfg.Port == "" {
		return nil, nil, fmt.Errorf("%s: must specify either host or port", path)
	}
	if cfg.Host != "" && cfg.Port != "" {
		return nil, nil, fmt.Errorf("%s: cannot specify both host and port", path)
	}
	if !validBaudrates[cfg.Baudrate] {
		return nil, nil, fmt.Errorf("%s: baudrate %d not supported", path, cfg.Baudrate)
	}
	return nil, nil, nil
}
```

**Step 4: Run tests; expect pass**

**Step 5: Commit**

```bash
git add arm.go config_test.go
git commit -m "feat: validate baudrate against known list in arm config"
```

---

### Task 1.4: Meta.json gripper description

**Files:**
- Modify: `meta.json`

Replace the placeholder `"Provide a short..."` with:
```
"short_description": "Control the parallel gripper (joint 6) of the RoArm-M3 5-DOF+gripper arm"
```

Commit:
```bash
git add meta.json
git commit -m "docs: replace gripper short_description placeholder in meta.json"
```

---

### Task 1.5: Lowercase translation keys in `roarm_m3.json` (base_link)

**Files:**
- Modify: `roarm_m3.json` (lines 261-263)

Change outer `base_link` translation:
```json
"translation": {
  "x": 0,
  "y": 0,
  "z": 70
}
```

(Also audit the geometry-nested translations on lines 74-78, 114-118, 154-158, 194-198, 234-238. If the Viam schema for geometry accepts uppercase keys, leave them; otherwise lowercase. Verify by running `go build ./... && ./bin/waveshare-roarm` and checking the arm loads — OR by printing `model.Geometries(zeroInputs)` in a new test in Task 6.4.)

Commit:
```bash
git add roarm_m3.json
git commit -m "fix: lowercase base_link translation keys so z=70 actually applies"
```

---

### Task 1.6: Fix `link4_to_gipper` typo

**Files:**
- Modify: `roarm_m3.json` (line 58)

Change `"id": "link4_to_gipper"` → `"id": "link4_to_gripper"`.

Commit:
```bash
git add roarm_m3.json
git commit -m "fix: typo link4_to_gipper -> link4_to_gripper in kinematic model"
```

---

### Task 1.7: Split `timeout` into `http_timeout` and `serial_timeout`

**Files:**
- Modify: `arm.go` (RoArmM3Config fields + constructor wiring)
- Modify: `controller.go` (RoArmConfig fields + constructor, Timeout usage)

Replace single `Timeout Duration` with:
```go
HTTPTimeout   Duration `json:"http_timeout,omitempty"`
SerialTimeout Duration `json:"serial_timeout,omitempty"`
```

In `controller.go`, store two separate fields on `RoArmController`: `httpTimeout`, `serialTimeout`. HTTP code path uses `httpTimeout`; serial code path uses `serialTimeout`. Defaults: 5s for HTTP, 1s for serial.

Update `config_test.go` with a test for each.

Commit:
```bash
git add arm.go controller.go config_test.go
git commit -m "feat: split timeout into http_timeout and serial_timeout"
```

---

## Phase 2 — Protocol correctness

### Task 2.1: Replace `3.14159265359` literals with `math.Pi`

**Files:**
- Modify: `gripper.go` (lines 237, 255, 278)

Add `import "math"`; replace each literal with `math.Pi`.

Commit:
```bash
git add gripper.go
git commit -m "refactor: use math.Pi instead of hardcoded literal in gripper"
```

---

### Task 2.2: Add response frame `T`-correlation filter

**Files:**
- Modify: `controller.go` (sendSerialCommand, Command type)
- Create test in: `controller_test.go` (set up scaffold)

**Plan:**

1. Add a helper that returns the set of acceptable response-`T` values for a given request-`T`. For the RoArm firmware, `FEEDBACK_GET` (T=105) responds with `T=1051`; most set-commands echo the request T or return a feedback frame. Start with a permissive allow-list:

```go
// acceptableResponseTs returns the set of response T values the firmware
// is expected to send in reply to a given request T. An empty map means
// accept any frame (fallback).
func acceptableResponseTs(requestT int) map[int]bool {
	switch requestT {
	case FEEDBACK_GET:
		return map[int]bool{1051: true, FEEDBACK_GET: true}
	default:
		return nil // accept any
	}
}
```

2. In `sendSerialCommand`, when a JSON frame parses successfully, check whether `feedback.T` is in the acceptable set. If not, log a warning, reset the buffer, and continue reading.
3. Pass the request T into `sendSerialCommand`. Thread through `sendCommand`.
4. Hardware-free test: see Phase 6 frame-parser tests.

Commit:
```bash
git add controller.go
git commit -m "feat: filter serial response frames by expected T value"
```

---

### Task 2.3: Call `ResetInputBuffer()` before every serial write

**Files:**
- Modify: `controller.go` (sendSerialCommand)

At the top of `sendSerialCommand`, before `Write`:
```go
if err := c.serialPort.ResetInputBuffer(); err != nil {
	c.logger.Warnf("ResetInputBuffer failed: %v", err)
}
```

Commit:
```bash
git add controller.go
git commit -m "fix: clear serial input buffer before sending command"
```

---

### Task 2.4: Fix serial read loop (EOF, sleep on empty read)

**Files:**
- Modify: `controller.go` (sendSerialCommand read loop, lines 283-341)

Replace the read loop body with:
```go
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
```

Add `"errors"` and `"io"` to imports as needed.

Commit:
```bash
git add controller.go
git commit -m "fix: handle EOF and empty reads in serial read loop"
```

---

### Task 2.5: Delete the `π - radian` gripper transform

**Files:**
- Modify: `controller.go` (`SetJointRadian`, `SetJointRadians`, `GetJointRadians`)

Remove the three places that apply `math.Pi - radian` for joint 6:
- `SetJointRadian`: delete lines 405-409 (set `transformedRadian = radian` unconditionally — i.e. just use `radian` directly).
- `SetJointRadians`: delete line 451 (`transformedRadians[5] = math.Pi - transformedRadians[5]`).
- `GetJointRadians`: line 490, change `math.Pi - feedback.G` → `feedback.G`.

Commit:
```bash
git add controller.go
git commit -m "fix: remove gripper pi-minus-radian transform pending hardware validation"
```

---

### Task 2.6: Update gripper `Open`/`Grab` to use joint-limit extremes

**Files:**
- Modify: `gripper.go` (Open, Grab)
- Add: named constants in `gripper.go`

Add near the top of `gripper.go`:
```go
const (
	// gripperOpenRad / gripperGrabRad match the joint-6 limits from
	// controller.RoArmM3JointLimits. Using the actual limit extremes
	// avoids the off-by-range problem that existed when these were
	// hardcoded degree values (100, -10) combined with the pi-minus-radian
	// transform.
	gripperOpenRad = 1.9  // upper limit of joint 6 (fully open)
	gripperGrabRad = -0.2 // lower limit of joint 6 (fully closed)
)
```

In `Open`, change the call from `SetGripperPosition(100, 500, 50)` to:
```go
err := g.controller.SetJointRadian(6, gripperOpenRad, 500, 50)
```

In `Grab`, same but with `gripperGrabRad`. Update the post-grab `grabbed` check to use radian comparison (e.g. `grabbed := positionRad > gripperGrabRad + 0.05`).

Also change the `GetPosition` return type to radians (caller now uses radians). Or keep degrees but compute from the new `GetJointRadians[5]` value (no transform applied).

Commit:
```bash
git add gripper.go
git commit -m "refactor: use joint-limit radian constants for gripper Open/Grab"
```

---

### Task 2.7: Derive `RoArmM3AngleLimits` from radians (or delete if unused)

**Files:**
- Modify: `controller.go`

Run `grep -n RoArmM3AngleLimits .` to confirm no call sites. If unused, delete the declaration (lines 58-65). If used, replace with:
```go
var RoArmM3AngleLimits = func() [][2]float64 {
	out := make([][2]float64, len(RoArmM3JointLimits))
	for i, l := range RoArmM3JointLimits {
		out[i] = [2]float64{l[0] * 180.0 / math.Pi, l[1] * 180.0 / math.Pi}
	}
	return out
}()
```

Commit:
```bash
git add controller.go
git commit -m "refactor: derive RoArmM3AngleLimits from radian limits (or delete if unused)"
```

---

## Phase 3 — Controller ownership rewrite

This phase is the biggest architectural change. Land as one large atomic commit (or split into 3a-ownership, 3b-deps, 3c-reconfigure) because intermediate states won't compile.

### Task 3.1: Define `RoArmHandle` interface

**Files:**
- Create: `handle.go`

```go
package waveshareroarm

import "context"

// RoArmHandle is the narrow interface other resources (like the gripper)
// consume when borrowing the arm's serial/HTTP handle.
type RoArmHandle interface {
	SetTorque(ctx context.Context, enable bool) error
	SetLED(ctx context.Context, brightness int) error
	MoveToHome(ctx context.Context) error
	SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error
	SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error
	GetJointRadians(ctx context.Context) ([]float64, error)
	GetFeedback(ctx context.Context) (*FeedbackData, error)
	TestConnection(ctx context.Context) error
}
```

Commit after Task 3.2 (interface + first impl together).

---

### Task 3.2: Make `RoArmController` own its mutex and accept `ctx` on every method

**Files:**
- Modify: `controller.go` (every method: add `ctx context.Context` parameter, collapse `sync.RWMutex` to `sync.Mutex`, honor ctx deadlines in HTTP calls)

Changes:
1. `RoArmController.mu` changes from `sync.RWMutex` to `sync.Mutex`.
2. `sendCommand(cmd *Command)` → `sendCommand(ctx context.Context, cmd *Command)`. Use this ctx for HTTP request (`http.NewRequestWithContext(ctx, ...)`) and as a cancellation signal inside the serial read loop (`select { case <-ctx.Done(): return nil, ctx.Err(); default: }` inside the loop).
3. Every public method gains `ctx context.Context` as its first parameter: `SetTorque`, `SetLED`, `MoveToHome`, `SetJointRadian`, `SetJointRadians`, `GetJointRadians`, `GetFeedback`, `TestConnection`, `SetGripperPosition`, `GetGripperPosition`, `Close`.
4. Controller satisfies `RoArmHandle` by virtue of its method set.

Update all call sites in `arm.go`, `gripper.go` to pass `ctx`.

---

### Task 3.3: Delete `manager.go` and all `Safe…` / `SharedController` references

**Files:**
- Delete: `manager.go`
- Modify: `arm.go` (remove `controller *SafeRoArmController`, use `*RoArmController` directly; remove `GetSharedController`/`ReleaseSharedController` calls)
- Modify: `gripper.go` (see Task 3.4)

In `arm.go`:
```go
type roarmM3 struct {
	// remove: controller *SafeRoArmController
	controller *RoArmController
	...
}

// In newRoArmM3:
controller, err := NewRoArmController(controllerConfig) // no singleton
if err != nil { return nil, err }
```

Remove the `controller_status` DoCommand branch in both `arm.go` and `gripper.go` (it referenced `GetControllerStatus`, which is gone).

---

### Task 3.4: Gripper config: declare arm as dependency, fetch from deps

**Files:**
- Modify: `gripper.go` (RoArmGripperConfig, Validate, newRoArmM3Gripper)

Replace the host/port/baudrate/timeout fields with an arm name:
```go
type RoArmGripperConfig struct {
	Arm string `json:"arm"` // name of the arm resource supplying the controller
}

func (cfg *RoArmGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("%s: must specify arm dependency", path)
	}
	return []string{cfg.Arm}, nil, nil // required dep
}
```

In `newRoArmM3Gripper`:
```go
cfg, err := resource.NativeConfig[*RoArmGripperConfig](conf)
if err != nil { return nil, err }

armRes, err := arm.FromDependencies(deps, cfg.Arm)
if err != nil {
	return nil, fmt.Errorf("gripper %s: could not find arm %q in deps: %w", conf.ResourceName(), cfg.Arm, err)
}
armImpl, ok := armRes.(*roarmM3)
if !ok {
	return nil, fmt.Errorf("gripper %s: arm %q is not a roarm-m3 arm", conf.ResourceName(), cfg.Arm)
}
handle := armImpl.Handle() // package-private accessor on *roarmM3
```

Add to `arm.go`:
```go
// Handle returns the underlying controller for sibling resources (e.g. gripper).
// Package-private: not part of the public Viam API.
func (r *roarmM3) Handle() RoArmHandle {
	return r.controller
}
```

---

### Task 3.5: Implement real `Reconfigure` on the arm

**Files:**
- Modify: `arm.go` (remove `resource.AlwaysRebuild`, add `Reconfigure` method)

Replace embedded `resource.AlwaysRebuild` with nothing. Add:

```go
func (r *roarmM3) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*RoArmM3Config](conf)
	if err != nil { return err }

	r.mu.Lock()
	defer r.mu.Unlock()

	// Only reopen hardware if connectivity changed.
	needsReopen := r.cfg == nil ||
		r.cfg.Host != newConf.Host ||
		r.cfg.Port != newConf.Port ||
		r.cfg.Baudrate != newConf.Baudrate ||
		r.cfg.HTTPTimeout != newConf.HTTPTimeout ||
		r.cfg.SerialTimeout != newConf.SerialTimeout

	if needsReopen {
		if r.controller != nil {
			_ = r.controller.Close(ctx)
		}
		ctrl, err := NewRoArmController(&RoArmConfig{
			Host: newConf.Host, Port: newConf.Port,
			Baudrate: newConf.Baudrate,
			HTTPTimeout: newConf.HTTPTimeout, SerialTimeout: newConf.SerialTimeout,
			Logger: r.logger,
		})
		if err != nil { return err }
		r.controller = ctrl
	}

	// Motion params always update.
	r.defaultSpeed = speedToUnits(float64(newConf.SpeedDegsPerSec))
	r.defaultAcc = accelToUnits(float64(newConf.AccelerationDegsPerSec))
	r.cfg = newConf
	return nil
}
```

Gripper stays `resource.AlwaysRebuild` — rebuilding it is cheap.

---

### Task 3.6: Commit Phase 3 together

Because 3.1-3.5 span multiple files and intermediate states don't compile, land as a single commit:

```bash
git add -A
go build ./... && go vet ./...
git commit -m "refactor: gripper depends_on arm, delete manager singleton, add Reconfigure"
```

If the change is too large for one commit, split as (a) 3.1+3.2+3.3 (interface + controller + delete manager, adjusting gripper minimally), (b) 3.4 (gripper deps), (c) 3.5 (Reconfigure). The key is that each commit leaves `go build ./...` clean.

---

## Phase 4 — Motion semantics

### Task 4.1: Plumb `opMgr` into arm `MoveToJointPositions` and `Stop`

**Files:**
- Modify: `arm.go`

At the top of `MoveToJointPositions`:
```go
ctx, done := r.opMgr.New(ctx)
defer done()
```

Replace `time.Sleep(time.Duration(moveTimeSeconds * float64(time.Second)))` with:
```go
select {
case <-time.After(time.Duration(moveTimeSeconds * float64(time.Second))):
case <-ctx.Done():
	return ctx.Err()
}
```

In `Stop`:
```go
func (r *roarmM3) Stop(ctx context.Context, extra map[string]interface{}) error {
	r.opMgr.CancelRunning(ctx)
	// Read current joint positions and re-send with low speed → soft stop.
	current, err := r.controller.GetJointRadians(ctx)
	if err != nil {
		return fmt.Errorf("stop: read current positions: %w", err)
	}
	const stopSpeed = 100 // internal units (~10 deg/s)
	return r.controller.SetJointRadians(ctx, current, stopSpeed, r.defaultAcc)
}
```

Commit:
```bash
git add arm.go
git commit -m "feat: arm Stop actually holds current position and cancels pending ops"
```

---

### Task 4.2: Same for gripper

**Files:**
- Modify: `gripper.go`

Add an `opMgr *operation.SingleOperationManager` field to `roarmM3Gripper`, initialize in constructor. Wrap `Open`, `Grab`, `SetPosition` with `opMgr.New(ctx)`. Replace their `time.Sleep` with ctx-aware `select`. Implement real `Stop`:

```go
func (g *roarmM3Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.opMgr.CancelRunning(ctx)
	positions, err := g.controller.GetJointRadians(ctx)
	if err != nil {
		return fmt.Errorf("gripper stop: read position: %w", err)
	}
	if len(positions) < 6 { return fmt.Errorf("gripper stop: short feedback") }
	return g.controller.SetJointRadian(ctx, 6, positions[5], 100, 50)
}
```

Commit:
```bash
git add gripper.go
git commit -m "feat: gripper Stop actually holds current position and cancels pending ops"
```

---

### Task 4.3: Motion-tracker + real `IsMoving` for arm

**Files:**
- Modify: `controller.go` (add motion tracker)
- Modify: `arm.go` (IsMoving uses tracker)
- Create test: `controller_test.go` (basic tracker logic)

**Step 1: Tests**

In `controller_test.go`, add:
```go
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
```

**Step 2: Implement** in `controller.go`:
```go
type motionTracker struct {
	mu       sync.Mutex
	deadline time.Time
}

func newMotionTracker() *motionTracker { return &motionTracker{} }

func (m *motionTracker) recordMove(deadline time.Time) {
	m.mu.Lock(); defer m.mu.Unlock()
	m.deadline = deadline
}

func (m *motionTracker) isMoving(now time.Time) bool {
	m.mu.Lock(); defer m.mu.Unlock()
	return now.Before(m.deadline)
}
```

Add `tracker *motionTracker` field to `RoArmController`; initialize in `NewRoArmController`. In `SetJointRadians` / `SetJointRadian`, call `c.tracker.recordMove(time.Now().Add(estimatedDuration))` where `estimatedDuration` comes from the existing max-delta calculation.

**Step 3: Arm wires it in**

Arm `IsMoving`:
```go
func (r *roarmM3) IsMoving(ctx context.Context) (bool, error) {
	return r.controller.IsMoving(ctx), nil
}
```

Expose on controller:
```go
func (c *RoArmController) IsMoving(ctx context.Context) bool {
	return c.tracker.isMoving(time.Now())
}
```

Remove `atomic.Bool isMoving` and `moveLock sync.Mutex` from `roarmM3` — they're replaced by the tracker. Same cleanup in gripper.

**Step 4: Tests pass**; commit

```bash
git add controller.go arm.go gripper.go controller_test.go
git commit -m "feat: motion tracker backs IsMoving with deadline-based estimation"
```

---

### Task 4.4: Position-delta refinement for `IsMoving`

**Files:**
- Modify: `controller.go`

Extend `motionTracker` with a cached last-read position sample. On `IsMoving`, if we're before the deadline *and* the cache is older than 200ms, take a fresh `GetFeedback` read and compare — declare not-moving early if all joint deltas are < 0.5° over the measurement interval. This is an optimization; the deadline-based version from Task 4.3 is already correct conservatively.

Keep the implementation simple — this can be a skip if it's tricky. Document the intended behavior in a comment.

Optional commit:
```bash
git add controller.go
git commit -m "feat: short-circuit IsMoving when position delta falls below threshold"
```

---

## Phase 5 — Code hygiene sweep

### Task 5.1: Create `conversions.go` with speed/accel helpers

**Files:**
- Create: `conversions.go`
- Create test: `conversions_test.go`

**Step 1: Tests**

```go
package waveshareroarm

import (
	"math"
	"testing"
)

func TestSpeedRoundTrip(t *testing.T) {
	cases := []float64{10, 50, 100, 180}
	for _, want := range cases {
		got := speedFromUnits(speedToUnits(want))
		if math.Abs(got-want) > 0.01 {
			t.Fatalf("round trip for %.1f: got %.3f", want, got)
		}
	}
}

func TestSpeedClamp(t *testing.T) {
	if speedToUnits(0) != minSpeedUnits { t.Fatal("low clamp") }
	if speedToUnits(1e9) != maxSpeedUnits { t.Fatal("high clamp") }
}

func TestAccelRoundTrip(t *testing.T) {
	for _, want := range []float64{10, 100, 500} {
		got := accelFromUnits(accelToUnits(want))
		if math.Abs(got-want) > 0.1 {
			t.Fatalf("round trip for %.1f: got %.3f", want, got)
		}
	}
}
```

**Step 2: Implement**

```go
package waveshareroarm

const (
	// speedUnitsPerDegPerSec is an EMPIRICAL constant pending hardware
	// calibration. Source: a hand-written comment in the initial import
	// ("RoArm speed range is 1-4096, where ~50 deg/sec ≈ 500 units").
	// TODO(calibration): replace with measured per-joint values — see
	// cmd/cli calibrate-speed and docs/firmware-units.md.
	speedUnitsPerDegPerSec  = 10.0
	accelUnitsPerDegPerSec2 = 0.5

	minSpeedUnits, maxSpeedUnits = 1, 4096
	minAccelUnits, maxAccelUnits = 1, 254
)

func clamp(v, lo, hi int) int {
	if v < lo { return lo }
	if v > hi { return hi }
	return v
}

func speedToUnits(degPerSec float64) int {
	return clamp(int(degPerSec*speedUnitsPerDegPerSec), minSpeedUnits, maxSpeedUnits)
}
func speedFromUnits(units int) float64 {
	return float64(units) / speedUnitsPerDegPerSec
}
func accelToUnits(degPerSec2 float64) int {
	return clamp(int(degPerSec2*accelUnitsPerDegPerSec2), minAccelUnits, maxAccelUnits)
}
func accelFromUnits(units int) float64 {
	return float64(units) / accelUnitsPerDegPerSec2
}
```

**Step 3: Run tests; pass**

**Step 4: Commit**

```bash
git add conversions.go conversions_test.go
git commit -m "feat: extract speed/accel conversions into named constants"
```

---

### Task 5.2: Migrate all call sites to conversion helpers

**Files:**
- Modify: `arm.go` (5 sites: lines 130-136, 140-146, 274-281, 286-293, 317, 461-467, 482-488, 499-500)

Replace every inline `* 10`, `/ 10.0`, `* 0.5`, `/ 0.5` with the helpers. For the sleep-time estimation on line 317:
```go
speedDegPerSec := speedFromUnits(speed)
speedRadPerSec := speedDegPerSec * math.Pi / 180.0
```

Commit:
```bash
git add arm.go
git commit -m "refactor: migrate arm.go to conversion helpers"
```

---

### Task 5.3: `NewClientFromConn` returns `errors.ErrUnsupported` instead of panicking

**Files:**
- Modify: `arm.go` (line 194-196)

```go
func (r *roarmM3) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	return nil, errors.ErrUnsupported
}
```

Import: `"github.com/pkg/errors"` already present; use Go stdlib `errors` package. Adjust imports accordingly or use `errors.New("NewClientFromConn not supported on native module")`.

Commit:
```bash
git add arm.go
git commit -m "fix: return error from NewClientFromConn instead of panicking"
```

---

### Task 5.4: `EndPosition` uses `ComputePosition` (not OOB)

**Files:**
- Modify: `arm.go` (line 207)

Change `referenceframe.ComputeOOBPosition(r.model, inputs)` → `referenceframe.ComputePosition(r.model, inputs)`.

Commit:
```bash
git add arm.go
git commit -m "fix: EndPosition uses strict ComputePosition instead of OOB variant"
```

---

### Task 5.5: Gripper bounding-box geometry + `IsHoldingSomething` cache

**Files:**
- Modify: `gripper.go`

Add bounding-box geometry in the constructor — approximate jaw extent 60mm beyond wrist, 40mm tall, 70mm wide. Use `spatialmath.NewBox` and attach to a single-link model.

```go
import "go.viam.com/rdk/spatialmath"

// newRoArmM3Gripper:
boxDims := r3.Vector{X: 70, Y: 40, Z: 60}
offset := spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 30})
box, err := spatialmath.NewBox(offset, boxDims, "gripper-box")
if err != nil { return nil, err }
// attach via referenceframe.NewStaticFrameWithGeometry(...) and NewSimpleModelFrame as applicable
```

(Exact API: consult `go.viam.com/rdk/referenceframe` for current constructors — the goal is a model that returns a single geometry from `model.Geometries(inputs)`.)

Add `holding atomic.Bool` to `roarmM3Gripper`. In `Grab`, after computing `grabbed`, `g.holding.Store(grabbed)`. In `Open`, `g.holding.Store(false)`.

Replace `IsHoldingSomething`:
```go
func (g *roarmM3Gripper) IsHoldingSomething(ctx context.Context, _ map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{IsHoldingSomething: g.holding.Load()}, nil
}
```

Replace `Geometries` to return the real geometry instead of `errors.ErrUnsupported`.

Commit:
```bash
git add gripper.go
git commit -m "feat: gripper reports bounding-box geometry and tracks holding state"
```

---

### Task 5.6: `MoveToJointPositions` aborts on gripper read failure

**Files:**
- Modify: `arm.go` (lines 252-258)

Change:
```go
currentFullPositions, err := r.controller.GetJointRadians(ctx)
if err != nil {
	return fmt.Errorf("MoveToJointPositions: read current positions: %w", err)
}
if len(currentFullPositions) < 6 {
	return fmt.Errorf("MoveToJointPositions: short feedback (got %d joints)", len(currentFullPositions))
}
currentGripperPos := currentFullPositions[5]
```

No more silent "use 0 as default".

Commit:
```bash
git add arm.go
git commit -m "fix: abort MoveToJointPositions on gripper-read failure instead of closing gripper"
```

---

### Task 5.7: Close paths honor `ctx`, reject new ops

**Files:**
- Modify: `arm.go` (Close), `gripper.go` (Close)

Add a `closed atomic.Bool` to both. At the top of every method that calls the controller, check `if r.closed.Load() { return errors.New("resource closed") }`. In `Close`:

```go
func (r *roarmM3) Close(ctx context.Context) error {
	if !r.closed.CompareAndSwap(false, true) { return nil }
	r.cancelFunc()
	r.opMgr.CancelRunning(ctx)
	r.mu.Lock(); defer r.mu.Unlock()
	if r.controller != nil { return r.controller.Close(ctx) }
	return nil
}
```

Gripper `Close` is symmetric but does not close the shared controller — the arm owns it.

Commit:
```bash
git add arm.go gripper.go
git commit -m "fix: Close honors ctx, rejects new ops, and respects ownership"
```

---

### Task 5.8: Normalize `DoCommand` to single-shape dispatch

**Files:**
- Modify: `arm.go` (DoCommand)

Rewrite the `default:` branch of the switch to be explicit cases under `cmd["command"]`:

```go
case "set_speed":
	speed, ok := cmd["value"].(float64)
	if !ok { return nil, fmt.Errorf("set_speed requires 'value' number") }
	if speed < 3 || speed > 180 { return nil, fmt.Errorf("speed out of range: %.1f", speed) }
	r.mu.Lock(); r.defaultSpeed = speedToUnits(speed); r.mu.Unlock()
	return map[string]interface{}{"speed_set": speed}, nil

case "set_acceleration":
	acc, ok := cmd["value"].(float64)
	if !ok { return nil, fmt.Errorf("set_acceleration requires 'value' number") }
	if acc < 10 || acc > 500 { return nil, fmt.Errorf("accel out of range: %.1f", acc) }
	r.mu.Lock(); r.defaultAcc = accelToUnits(acc); r.mu.Unlock()
	return map[string]interface{}{"acceleration_set": acc}, nil

case "get_motion_params":
	r.mu.RLock(); defer r.mu.RUnlock()
	return map[string]interface{}{
		"current_speed_degs_per_sec": speedFromUnits(r.defaultSpeed),
		"current_acceleration_degs_per_sec_per_sec": accelFromUnits(r.defaultAcc),
	}, nil
```

Delete the final bare `default: if changed { return ...` branch. Unknown commands fall through to `return nil, fmt.Errorf("unknown command: %v", cmd["command"])`.

README.md should be updated to reflect the new shape — but that can be a later docs task (see Phase 7 closeout).

Commit:
```bash
git add arm.go
git commit -m "refactor: normalize DoCommand to single cmd[\"command\"] dispatch shape"
```

---

### Task 5.9: Rename `FeedbackData.T_` to `Wrist`

**Files:**
- Modify: `controller.go` (struct), `arm.go` (DoCommand "get_feedback" branch)

Change `T_ float64 \`json:"t"\`` → `Wrist float64 \`json:"t"\``. Update `GetJointRadians` reference (`feedback.T_` → `feedback.Wrist`) and `DoCommand` (`feedback.T_` → `feedback.Wrist`).

Commit:
```bash
git add controller.go arm.go
git commit -m "refactor: rename FeedbackData.T_ to Wrist"
```

---

### Task 5.10: Demote per-frame serial logging

**Files:**
- Modify: `controller.go` (lines 267, 301, 327, 332)

Wrap each `c.logger.Debugf("Received serial data: ...")` and `"Parsing JSON response: ..."` behind a trace flag, or simply change `Debugf` → `Logger` method check, e.g.:

```go
// Only log raw bytes at explicit verbose level.
if c.verboseWire {
	c.logger.Debugf("Received serial data: %s", string(buffer[:n]))
}
```

Add `verboseWire bool` to `RoArmController`, defaulting to `false`; can be toggled via an env var `ROARM_WIRE_TRACE=1` read in `NewRoArmController`.

Commit:
```bash
git add controller.go
git commit -m "refactor: gate per-frame serial logs behind verbose flag"
```

---

### Task 5.11: Remove `GetAvailableSerialPorts`

**Files:**
- Modify: `controller.go` (lines 564-567)

Delete. If a caller exists (check with `grep`), move the logic into `cmd/cli/main.go` where it actually belongs.

Commit:
```bash
git add controller.go
git commit -m "chore: remove unused GetAvailableSerialPorts"
```

---

### Task 5.12: Rewrite `cmd/cli/main.go`

**Files:**
- Rewrite: `cmd/cli/main.go`

Replace the uncompilable stub with a flag-based CLI that exercises `RoArmController` directly:

```go
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"strconv"

	waveshareroarm "github.com/hipsterbrown/viam-waveshare-roarm"
	"go.viam.com/rdk/logging"
)

func main() {
	host := flag.String("host", "", "RoArm HTTP host (e.g. 192.168.1.10)")
	port := flag.String("port", "", "Serial port (e.g. /dev/tty.usbserial-xxx)")
	baud := flag.Int("baudrate", 115200, "Serial baud rate")
	flag.Parse()

	if flag.NArg() < 1 {
		fmt.Fprintln(os.Stderr, "usage: roarm-cli [--host=X | --port=Y] <subcommand> [args...]")
		fmt.Fprintln(os.Stderr, "subcommands: ping, feedback, home, move, sweep, calibrate-speed, calibrate-accel, gripper")
		os.Exit(2)
	}

	logger := logging.NewLogger("roarm-cli")
	ctrl, err := waveshareroarm.NewRoArmController(&waveshareroarm.RoArmConfig{
		Host: *host, Port: *port, Baudrate: *baud, Logger: logger,
	})
	if err != nil { log.Fatal(err) }
	defer ctrl.Close(context.Background())

	ctx := context.Background()
	args := flag.Args()
	switch args[0] {
	case "ping":
		if err := ctrl.TestConnection(ctx); err != nil { log.Fatal(err) }
		fmt.Println("OK")
	case "feedback":
		fb, err := ctrl.GetFeedback(ctx)
		if err != nil { log.Fatal(err) }
		fmt.Printf("%+v\n", fb)
	case "home":
		if err := ctrl.MoveToHome(ctx); err != nil { log.Fatal(err) }
	case "move":
		// move <joint> <rad> [speed] [acc]
		if len(args) < 3 { log.Fatal("move <joint> <rad> [speed] [acc]") }
		joint, _ := strconv.Atoi(args[1])
		rad, _ := strconv.ParseFloat(args[2], 64)
		speed, acc := 500, 50
		if len(args) > 3 { speed, _ = strconv.Atoi(args[3]) }
		if len(args) > 4 { acc, _ = strconv.Atoi(args[4]) }
		if err := ctrl.SetJointRadian(ctx, joint, rad, speed, acc); err != nil { log.Fatal(err) }
	case "gripper":
		if len(args) < 2 { log.Fatal("gripper <rad>") }
		rad, _ := strconv.ParseFloat(args[1], 64)
		if err := ctrl.SetJointRadian(ctx, 6, rad, 500, 50); err != nil { log.Fatal(err) }
	case "sweep", "calibrate-speed", "calibrate-accel":
		log.Fatalf("%s: implemented in Phase 7", args[0])
	default:
		log.Fatalf("unknown subcommand: %s", args[0])
	}
}
```

Verify `go build ./cmd/cli/` succeeds. The package-import string needs to match the module path in `go.mod` — check with `head -1 go.mod`. If it's `module waveshareroarm`, change the import to bare `"waveshareroarm"`.

Commit:
```bash
git add cmd/cli/main.go
git commit -m "feat: rewrite cmd/cli as a bench-testing tool using RoArmController directly"
```

---

### Task 5.13: Collapse concurrency primitives

**Files:**
- Modify: `arm.go`, `gripper.go`

After all prior changes, remove now-redundant locks:
- Arm: remove `moveLock sync.Mutex` and `isMoving atomic.Bool`. Keep `mu sync.Mutex` (not RWMutex) for fields like `cfg`, `defaultSpeed`, `defaultAcc`.
- Gripper: remove `isMoving atomic.Bool`. Keep `mu sync.Mutex`.

Re-run `go build ./... && go vet ./...`.

Commit:
```bash
git add arm.go gripper.go
git commit -m "refactor: collapse redundant mutexes after IsMoving refactor"
```

---

## Phase 6 — Tests

All tests are pure-Go with no hardware. The fake controller is shared across suites.

### Task 6.1: Shared `fakeController` in `testutil_test.go`

**Files:**
- Create: `testutil_test.go`

```go
package waveshareroarm

import (
	"context"
	"sync"
	"time"
)

type fakeController struct {
	mu          sync.Mutex
	LastSpeed   int
	LastAcc     int
	LastRadians []float64
	Feedback    FeedbackData
	FailOn      string // method name to return error from, empty = never
	Delay       time.Duration
}

func (f *fakeController) err(method string) error {
	if f.FailOn == method {
		return &fakeErr{method}
	}
	return nil
}

type fakeErr struct{ m string }
func (e *fakeErr) Error() string { return "fake: forced error from " + e.m }

func (f *fakeController) SetTorque(ctx context.Context, enable bool) error { return f.err("SetTorque") }
func (f *fakeController) SetLED(ctx context.Context, brightness int) error { return f.err("SetLED") }
func (f *fakeController) MoveToHome(ctx context.Context) error { return f.err("MoveToHome") }
func (f *fakeController) SetJointRadian(ctx context.Context, joint int, radian float64, speed, acc int) error {
	if err := f.err("SetJointRadian"); err != nil { return err }
	f.mu.Lock(); defer f.mu.Unlock()
	f.LastSpeed, f.LastAcc = speed, acc
	return nil
}
func (f *fakeController) SetJointRadians(ctx context.Context, radians []float64, speed, acc int) error {
	if err := f.err("SetJointRadians"); err != nil { return err }
	f.mu.Lock(); defer f.mu.Unlock()
	f.LastRadians = append([]float64(nil), radians...)
	f.LastSpeed, f.LastAcc = speed, acc
	return nil
}
func (f *fakeController) GetJointRadians(ctx context.Context) ([]float64, error) {
	if err := f.err("GetJointRadians"); err != nil { return nil, err }
	f.mu.Lock(); defer f.mu.Unlock()
	return []float64{f.Feedback.B, f.Feedback.S, f.Feedback.E, f.Feedback.Wrist, f.Feedback.R, f.Feedback.G}, nil
}
func (f *fakeController) GetFeedback(ctx context.Context) (*FeedbackData, error) {
	if err := f.err("GetFeedback"); err != nil { return nil, err }
	f.mu.Lock(); defer f.mu.Unlock()
	fb := f.Feedback
	return &fb, nil
}
func (f *fakeController) TestConnection(ctx context.Context) error { return f.err("TestConnection") }
```

Commit:
```bash
git add testutil_test.go
git commit -m "test: add fakeController satisfying RoArmHandle"
```

---

### Task 6.2: `controller_test.go` — Command marshaling

**Files:**
- Create/extend: `controller_test.go`

Tests:
- `TestCommandMarshalJSON_WithData` — cmd with `{T: 101, Data: {"joint": 1, "rad": 0.5}}` marshals to `{"T":101,"joint":1,"rad":0.5}`.
- `TestCommandMarshalJSON_EmptyData` — cmd with empty Data marshals to `{"T":105}` (no stray fields).
- `TestCommandMarshalJSON_NilData` — cmd with nil Data marshals cleanly.

Commit:
```bash
git add controller_test.go
git commit -m "test: Command JSON marshaling"
```

---

### Task 6.3: `controller_test.go` — Serial frame parser

**Files:**
- Extend: `controller_test.go`

Extract the frame-detection logic from `sendSerialCommand` into a pure function `parseLastJSONFrame(buf []byte) (jsonData []byte, remaining []byte, ok bool)`. Use it from `sendSerialCommand`. Then test directly:

- Single complete frame: `{"T":1051,"B":0,...}\r\n` → parses.
- Partial frame (no trailing `\r\n`): returns not-ok.
- Two concatenated frames: returns the *last* one (this matches existing behavior).
- Garbage before frame: recovers (uses `LastIndex` for `{`).
- Oversize buffer: the trim branch keeps the tail.

Commit:
```bash
git add controller.go controller_test.go
git commit -m "test: serial frame parser unit tests + extract parseLastJSONFrame"
```

---

### Task 6.4: `controller_test.go` — Response T filter

**Files:**
- Extend: `controller_test.go`

Test that `acceptableResponseTs(FEEDBACK_GET)` contains `1051`. Test that an out-of-set frame is rejected by the filter helper (separate the filter from the IO loop — make it testable).

Commit:
```bash
git add controller.go controller_test.go
git commit -m "test: response T-value filter"
```

---

### Task 6.5: `config_test.go` — comprehensive Validate coverage

**Files:**
- Extend: `config_test.go`

Cases already covered in Tasks 1.1-1.7 for baudrate. Add:
- Arm: host AND port set → error.
- Arm: neither host nor port → error.
- Arm: valid HTTP config (host, no port) → no error.
- Arm: valid serial config (port, no host) → no error.
- Gripper: missing `arm` field → error; returns empty deps.
- Gripper: with `arm: "foo"` → no error; returns `["foo"]` as required deps.
- `Duration` UnmarshalJSON: already tested in Task 1.1.

Commit:
```bash
git add config_test.go
git commit -m "test: exhaustive arm + gripper Validate coverage"
```

---

### Task 6.6: `arm_test.go` — joint-limit clamping

**Files:**
- Create: `arm_test.go`

Construct a `roarmM3` with a `fakeController` as its handle. Call `MoveToJointPositions` with a position 2× over the joint limit. Assert the `fakeController.LastRadians` reflects the clamped value, not the original. Repeat for below-limit.

This may require a test-only constructor or an exported helper — define `newRoarmM3ForTest(handle RoArmHandle, cfg *RoArmM3Config) *roarmM3` in the package.

Commit:
```bash
git add arm.go arm_test.go
git commit -m "test: arm joint-limit clamping"
```

---

### Task 6.7: `arm_test.go` — `Stop` sends hold-current-position

Test: set `fakeController.Feedback.B = 0.5, S = 0.3, ...`; call `arm.Stop(ctx, nil)`; assert `fakeController.LastRadians == {0.5, 0.3, ...}` and `LastSpeed` matches the hold-stop speed constant.

Commit:
```bash
git add arm_test.go
git commit -m "test: arm Stop reads and re-sends current positions"
```

---

### Task 6.8: `arm_test.go` — `DoCommand` dispatch

Test each command: `set_speed`, `set_acceleration`, `get_motion_params`, `set_led`, `set_torque`, `move_to_home`, `get_feedback`. Assert fake controller received the expected call. Assert unknown command returns an error.

Commit:
```bash
git add arm_test.go
git commit -m "test: arm DoCommand dispatch coverage"
```

---

### Task 6.9: `gripper_test.go` — transform identity + `IsHoldingSomething`

**Files:**
- Create: `gripper_test.go`

Cases:
- `SetPosition(1.5 rad)` → `fakeController.LastRadians[5] == 1.5` (no transform applied).
- `Open(ctx)` → fake sees `gripperOpenRad`.
- `Grab(ctx)` with `Feedback.G = gripperGrabRad` (closed fully, not held) → returns `false`, `IsHoldingSomething` = false.
- `Grab(ctx)` with `Feedback.G = 0.3` (blocked partway) → returns `true`, `IsHoldingSomething` = true.
- `Open(ctx)` after hold → clears holding state.

Commit:
```bash
git add gripper_test.go
git commit -m "test: gripper transform identity and holding-state cache"
```

---

### Task 6.10: Coverage check

Run `go test ./... -coverprofile=coverage.out && go tool cover -func=coverage.out`. Aim for >70% on `controller.go`, `arm.go`, `gripper.go`. Add targeted tests for any gaps.

Commit any follow-up tests individually.

Commit the final state:
```bash
git add -A
git commit -m "test: final coverage pass"
```

---

## Phase 7 — Speed/acceleration calibration (hardware-gated, optional)

Run only when a physical RoArm-M3 is on the bench.

### Task 7.1: Document firmware units

**Files:**
- Create: `docs/firmware-units.md`

Capture: source of the 10× and 0.5× constants, documented firmware range (1–4096 speed, 1–254 accel), known unknowns, link to `waveshare_roarm_sdk` Python source for reconciliation. Committable without hardware.

---

### Task 7.2: Add velocity fields to `FeedbackData` if firmware provides them

**Files:**
- Modify: `controller.go`

Inspect a `T=1051` frame on hardware (`roarm-cli feedback`). If velocity fields are present (e.g. `"vB"`, `"vS"`), add them to `FeedbackData`. If not, skip this task.

---

### Task 7.3: Implement `calibrate-speed` subcommand

**Files:**
- Modify: `cmd/cli/main.go`

Logic per design doc Step 4:
1. Move joint to home, settle 1s.
2. For command speed ∈ `{100, 200, 500, 1000, 2000, 4000}` units:
   - Record start position + time.
   - Move to other extreme of joint range.
   - Poll `GetFeedback` at 50Hz for `~timeout` seconds, log `(t, position)` pairs.
   - Stop when delta < 0.1° for 500ms.
3. Compute peak and average deg/sec per trial.
4. Fit linear model; print candidate `speedUnitsPerDegPerSec`.
5. Write CSV to `calibration-joint{N}.csv`.

### Task 7.4: `calibrate-accel` analogue

Same structure, varying accel while fixing speed, measuring rise time.

### Task 7.5: Run calibration, update constants, commit CSV

Run per-joint. If variation < 10%, update scalar constants in `conversions.go`. If >10%, migrate to `[6]float64` arrays and thread joint index through `speedToUnits`/`accelToUnits`.

Commit CSV to `testdata/calibration-joint{N}.csv`.

### Task 7.6: Regression test

**Files:**
- Extend: `conversions_test.go`

Load `testdata/calibration-joint0.csv`. For each row, assert `speedFromUnits(units)` is within ±10% of the measured average. Future refactors that break conversion fail CI.

---

## Closing

After Phase 6 lands:

1. Update `README.md` to reflect:
   - Gripper config now requires `arm: "<arm-name>"` instead of host/port.
   - `http_timeout` and `serial_timeout` replace `timeout`.
   - Accepted baudrate list.
   - Any DoCommand shape changes.
2. Bump `meta.json` version if applicable.
3. Squash-merge feature branch if using one, otherwise land as sequence of the per-task commits.
4. Request final code review via `@superpowers:code-reviewer`.

---

## Verification at the end

```bash
go build ./...                    # clean
go vet ./...                      # clean
go test ./... -v                  # all pass
go test ./... -coverprofile=coverage.out
go tool cover -func=coverage.out  # ≥70% on controller/arm/gripper
./bin/roarm-cli --port=... ping   # round-trips on hardware
./bin/roarm-cli --port=... home   # arm moves and settles
```
