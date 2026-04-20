# Waveshare RoArm-M3 Module — Remediation Design

**Date:** 2026-04-20
**Scope:** All issues surfaced by the 2026-04-20 code review (Critical, Important, Minor).
**Backwards compatibility:** None — break config schema freely. Module has <10 commits and is not yet registry-polished.

## Framing

The module targets the **RoArm-M3 ESP32 firmware's JSON API** (`{"T":101,...}` over HTTP or newline-delimited JSON over serial at 115200). It does *not* speak the ST3215 binary protocol directly; the ESP32 firmware does that internally. The Python reference for this abstraction layer is `waveshare_roarm_sdk`, not `waveshare_stservo_python`.

## Architecture changes

### Controller ownership

Delete `manager.go` entirely. The arm constructor opens the serial/HTTP handle and owns it for its lifetime. The gripper declares the arm in `Validate` (`deps` return includes the arm's `resource.Name`), pulls the arm from `resource.Dependencies` in its constructor, and borrows the controller through a narrow interface:

```go
type RoArmHandle interface {
    SendCommand(ctx context.Context, cmd Command) (*FeedbackData, error)
    GetFeedback(ctx context.Context) (*FeedbackData, error)
    Close(ctx context.Context) error
}
```

The arm exposes this handle via a package-private accessor. Only one `sync.Mutex` guards serial I/O. `SafeRoArmController` and the double-mutex pattern are removed.

### Reconfigure

Replace `resource.AlwaysRebuild` on the arm with a real `Reconfigure` that compares old/new config and only reopens the port when `host`, `port`, or `baudrate` changes. Gripper stays `AlwaysRebuild` — it owns no hardware of its own and rebuilds are cheap.

## Protocol correctness

- **Serial read loop**: before every `Write`, call `ResetInputBuffer()`. Treat `io.EOF` and closed-port errors as fatal. Sleep 1ms on `(n=0, err=nil)` to avoid spinning. Verify response `T` matches the expected echo set for the command; drop non-matching frames.
- **Gripper transform**: delete the `π - radian` mapping. Map `gripper_position_rad → firmware "hand" rad` as identity. Sanity-check against joint limits `-0.2` to `1.9` rad. Update `Open` to the upper limit (~1.9) and `Grab` to the lower limit (~-0.2) rather than the current 100°/-10° degree values that fall outside the joint-limit band. If later hardware calibration proves a real offset, reintroduce with a comment recording the measured values and date.
- **Kinematic model** (`roarm_m3.json`): lowercase all `X/Y/Z` translation keys. Fix the `"link4_to_gipper"` typo.
- **Constants**: replace all `3.14159265359` literals with `math.Pi`. Derive `RoArmM3AngleLimits` from `RoArmM3JointLimits` at init, or delete if no live consumer remains.

## Motion semantics

- **Stop**: reads current joint positions via `GetFeedback`, then re-sends them as the new target with low speed (~10°/s) and default acceleration. Arm decelerates and holds. Gripper `Stop` does the same for joint 6. An emergency torque-off path can be added later as a `DoCommand` if needed.
- **IsMoving**: backed by a motion tracker on the controller. After each `SetJointRadians`, the controller records an estimated completion deadline. `IsMoving()` returns true when either (a) we're before the deadline and a cached recent `GetFeedback` shows joint-position delta > 0.5° since the last sample, or (b) the cache is older than 200ms (forces a fresh read). This prevents serial thrash when motion planner polls at 10Hz while still giving real answers.
- **SingleOperationManager**: wire `opMgr.New(ctx)` at the top of every `MoveToJointPositions` and gripper `SetPosition`. Replace `time.Sleep(...)` with `select { case <-time.After(...): case <-ctx.Done(): return ctx.Err() }`. `Stop` calls `opMgr.CancelRunning(ctx)` before sending the hold command.

## Config and connectivity

- **`time.Duration` JSON**: switch `Timeout` to a custom type (or use a Viam-provided duration wrapper) that accepts both `"5s"` strings and raw nanosecond ints.
- **Validation**: reject baudrates outside `{9600, 19200, 38400, 57600, 115200, 230400, 921600, 1000000}`. Include `path` in every `Validate` error message. Split `timeout` into `http_timeout` and `serial_timeout` with separate defaults.
- **Reconnect**: controller gains a `reconnect()` method invoked on transient serial errors with exponential backoff (100ms → 1s → 5s, cap 30s). Three consecutive transient failures trigger one reconnect attempt; propagate error if reconnect fails. HTTP mode relies on Go's `http.Client` defaults.
- **Rate limiting**: enforce a minimum inter-command gap (configurable, default 20ms) inside the controller mutex to protect the serial bus.

## Speed and acceleration conversion

The current formulas (`degsPerSec * 10` for speed, `degsPerSec * 0.5` for acceleration) are admitted guesses duplicated across five call sites and used to compute motion sleep durations.

### Step 1 — extraction (no behavior change)

New file `conversions.go` with named constants and four pure functions:

```go
const (
    // Empirical, pending calibration (see cmd/cli calibrate-speed).
    speedUnitsPerDegPerSec  = 10.0
    accelUnitsPerDegPerSec2 = 0.5
)

func speedToUnits(degsPerSec float64) int
func speedFromUnits(units int) float64
func accelToUnits(degsPerSec2 float64) int
func accelFromUnits(units int) float64
```

All five existing call sites migrate. Unit tests cover round-trip (`speedFromUnits(speedToUnits(x)) ≈ x`) and clamp behavior at range boundaries.

### Step 2 — document known unknowns

`docs/firmware-units.md` captures: source of the 10× and 0.5× constants (a code comment in the initial import), documented firmware range (speed 1–4096, acceleration 1–254), open questions (per-joint variation, load dependence, fixed-point format). Link to `waveshare_roarm_sdk` Python source as the next place to verify the unit definition.

### Step 3 — instrument feedback for velocity

Check whether `FeedbackData` already carries per-joint velocity. If yes, add those fields to the struct — they become calibration ground truth. If not, fall back to position-differentiation sampling.

### Step 4 — calibration CLI

`cmd/cli calibrate-speed --joint=N` (see CLI section) runs a logarithmic sweep over command speed values, samples feedback at ~50Hz, fits a linear model `deg_per_sec = f(units)`, prints the fitted constant, and writes a CSV. `calibrate-accel` does the same for rise time. Run per-joint; gear ratios and loads differ.

### Step 5 — wire calibrated values back

If per-joint variation is <10%, update the scalar constants and remove the `TODO`. If >10%, replace with `[6]float64` arrays indexed by joint and plumb a `joint` parameter through `speedToUnits`/`accelToUnits`.

### Step 6 — regression test from calibration CSV

Check a representative CSV into `testdata/`. Add a test that loads it and asserts `speedFromUnits(u)` matches the measured average within tolerance. Future conversion changes that break this assertion fail CI.

## Command-line tool

Rewrite `cmd/cli/main.go` (currently uncompilable) as a real bench-testing CLI that exercises `RoArmController` directly, bypassing the Viam resource layer:

```
roarm-cli --host=192.168.1.10                  # HTTP mode
roarm-cli --port=/dev/tty.usbserial-xxx        # serial mode

Subcommands:
  ping                                        # confirm link (T=300)
  feedback                                    # one-shot GetFeedback dump
  home                                        # move to zero pose
  move --joint=N --rad=X                      # single-joint set
  sweep --joint=N --from=A --to=B --steps=K   # stepped sweep
  calibrate-speed --joint=N                   # speed calibration
  calibrate-accel --joint=N                   # acceleration calibration
  gripper --rad=X                             # direct gripper transform test
```

Output is plain text so a shell script can diff expected vs actual.

## Code hygiene

- Remove `GetAvailableSerialPorts`, unused `RoArmM3AngleLimits` constants.
- `NewClientFromConn`: return `errors.ErrUnsupported` instead of panicking.
- `EndPosition`: switch to `referenceframe.ComputePosition` (non-OOB).
- Gripper: add a bounding-box geometry approximating jaw extent (~60mm beyond wrist). Implement `IsHoldingSomething` from cached `grabbed` state set by `Grab`.
- `MoveToJointPositions`: abort on gripper-read failure rather than silently defaulting to 0 (which closes the gripper ~100° every arm move on transient failure).
- `Close` paths: honor `ctx`, take resource mutex, reject new operations after close, wait briefly for in-flight ops.
- Concurrency primitives: collapse to one `sync.Mutex` per resource plus one in the controller. Remove `RWMutex`, `moveLock`, and the `atomic.Bool` for `isMoving`.
- `DoCommand`: normalize to single-shape `cmd["command"]` dispatch; migrate `set_speed`, `set_acceleration`, `get_motion_params` into that shape.
- `meta.json`: real gripper description.
- `FeedbackData.T_`: rename to `Wrist` with `json:"t"`.
- Logging: demote per-frame serial dumps from `Debugf` to a trace-equivalent verbosity.

## Tests

Pure Go unit tests, no hardware:

- `controller_test.go` — `Command.MarshalJSON` (empty Data flattening), frame parser (partial frames, multi-frame concatenation, garbage prefix, missing trailing `\r\n`, oversize trim), JSON response `T`-correlation filter.
- `config_test.go` — arm + gripper `Validate` (missing deps, mutually exclusive host/port, bad baudrate, duration string parsing).
- `arm_test.go` — joint-limit clamping in `MoveToJointPositions`, speed/accel conversion round-trip, `IsMoving` state machine against a fake controller, `Stop` sends hold-current-position, `DoCommand` dispatch.
- `gripper_test.go` — transform round-trip (identity), `Grab`/`Open` target values, `IsHoldingSomething` cache.
- `testutil_test.go` — shared `fakeController` test double implementing `RoArmHandle`.

Coverage target: 70%+ on `controller.go`, `arm.go`, `gripper.go`. `go test ./...` clean in CI.

## Phasing

Each phase compiles and is landable as its own commit/PR.

1. **Config + Validate + meta cleanup** — `time.Duration` type, baudrate whitelist, `path` in errors, `meta.json` description, kinematic JSON key casing, `link4_to_gipper` typo.
2. **Protocol correctness** — `ResetInputBuffer`, response `T` filter, read-loop fix, gripper transform removal, `math.Pi`, constant cleanup.
3. **Controller ownership rewrite** — delete `manager.go`, add `RoArmHandle` interface, gripper `depends_on` arm, real `Reconfigure`.
4. **Motion semantics** — real `Stop`, real `IsMoving`, `SingleOperationManager` wiring, ctx-aware sleeps, motion deadline tracker.
5. **Code hygiene sweep** — dead-code removal, Close paths, speed/accel extraction (`conversions.go`), geometry + `IsHoldingSomething`, `DoCommand` normalization, `NewClientFromConn`, mutex collapse, `cmd/cli/main.go` rewrite.
6. **Tests** — add all unit tests; fix any issues they surface.
7. **(Hardware-required, optional)** Speed/accel calibration: run CLI sweeps per joint, update constants or per-joint arrays, check in CSV, add regression test.

## Success criteria

- `go build ./...` and `go vet ./...` clean.
- `go test ./...` passes with >70% coverage on the three main files.
- Gripper + arm reconfigure cleanly when changing port.
- `Stop` visibly decelerates and holds the arm on bench.
- `IsMoving` reports false within 200ms of the arm settling.
- `cmd/cli` subcommands all execute against hardware.
