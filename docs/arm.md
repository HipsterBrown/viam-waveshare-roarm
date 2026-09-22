[← Part of the waveshare-roarm module](../README.md)

# Model hipsterbrown:waveshare-roarm:arm

The arm component controls the first 5 joints of the RoArm-M3: base, shoulder, elbow, wrist, and roll.

### Configuration

```json
{
  "host": "192.168.4.1",
  "speed_degs_per_sec": 50,
  "acceleration_degs_per_sec_per_sec": 100
}
```

Or for serial communication:

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "speed_degs_per_sec": 50,
  "acceleration_degs_per_sec_per_sec": 100
}
```

### Attributes

The following attributes are available for the arm component:

| Name                                | Type             | Inclusion    | Description                                                                                                    |
|-------------------------------------|------------------|--------------|----------------------------------------------------------------------------------------------------------------|
| `host`                              | string           | Optional*    | The IP address of the RoArm-M3 for HTTP communication.                                                        |
| `port`                              | string           | Optional*    | The serial port for direct communication.                                                                     |
| `baudrate`                          | int              | Optional     | The baud rate for serial communication. Default is `115200`.                                                  |
| `http_timeout`                      | duration         | Optional     | HTTP communication timeout. Accepts a duration string (e.g. `"5s"`) or integer nanoseconds. Default is `5s`.  |
| `serial_timeout`                    | duration         | Optional     | Serial communication timeout. Accepts a duration string (e.g. `"1s"`) or integer nanoseconds. Default is `1s`.|
| `speed_degs_per_sec`                | float32          | Optional     | The rotational speed for arm movements (must be between 3 and 180). Default is `50` degrees/second. Validated at config time.  |
| `acceleration_degs_per_sec_per_sec` | float32          | Optional     | The acceleration for arm movements (must be between 10 and 500). Default is `100` degrees/second^2. Validated at config time.  |
| `motion`                            | string           | Optional     | The name of the motion service used to plan `MoveToPosition` requests. Default is `builtin`.                  |
| `orientation_tolerance_deg`         | float64          | Optional     | The approach-axis cone half-angle, in degrees, that `MoveToPosition` plans against. Default is `30`. Must be in `[0, 180]`. An explicit `0` means the default, **not** an exact orientation match: a zero-leeway goal region is one no inverse-kinematics solution realistically lands inside, so every move would fail to plan. For a near-exact orientation pass a small non-zero value. Validated at config time. |
| `position_tolerance_mm`             | float64          | Optional     | The per-axis positional leeway of the goal region, in millimeters. Default is `1.0`. Must not be negative; an explicit `0` means the default, for the same reason as above. This is a box, not a radius, so the worst-case corner is sqrt(3) times this value. Validated at config time. |
| `collision_geometry`                | string           | Optional     | `box` (default) or `mesh`. `box` uses one axis-aligned box per link sized from the CAD mesh; `mesh` uses a tighter envelope per link: the CAD mesh cut into 6 slabs along its length, each replaced by a 26-sided bounding polytope that chamfers the corners and edges the mesh does not reach, for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

*Either `host` or `port` must be specified, but not both.

> **Approach axis on `MoveToPosition`.** The Cartesian goal is an approach-axis cone: the planner must reach the goal point with the tool's approach axis within `orientation_tolerance_deg` of the requested orientation, while roll about that axis is left free. Earlier versions ignored orientation entirely, so a goal that planned before may now fail to plan; widen either tolerance, or use one of the escape hatches below. Goal regions need viam-server 0.127.0 or newer; older servers silently ignore them and plan against the exact pose.
>
> Two `extra` keys override the cone, and passing both is an error:
>
> - `{"goal_metric_type": "position_only"}` restores the old orientation-free behavior. No goal region is sent, because `position_only` scores orientation at zero weight.
> - `{"pose_cloud": {"x": 2, "y": 2, "z": 2, "ox": 1, "oy": 1, "oz": 0.13, "theta": 180}}` sends a goal region verbatim. Keys are all lowercase and any of the seven may be omitted, but an omitted leeway is zero, which is effectively an exact match on that axis.

> **Motion completion.** The firmware reports joint positions but no moving flag, so a move is judged finished from position feedback rather than after a computed delay. Every move returns once the joints are within about 1 degree of the target. A move that makes real progress and then stops short of it also returns successfully, with a warning naming how far short it stopped, since that is what a loaded servo or an obstacle looks like. A move where the arm never leaves its starting position returns an error instead: that is torque left off, a jam, a joint limit, or a command that never arrived, and none of those are success. How long a move is given, and how far it must travel to count as moving, are both derived from the speed and acceleration it was commanded with, so a deliberately slow move is not mistaken for a stalled one. `IsMoving` compares two feedback samples 40 ms apart and reports true while a move call is in flight.
>
> **`MoveThroughJointPositions` and `MoveOptions`.** A speed or acceleration limit passed in `MoveOptions` (`MaxVelRads`/`MaxVelRadsJoints`, `MaxAccRads`/`MaxAccRadsJoints`) is honored for the whole move, clamped to the configured `[3, 180]` deg/s and `[10, 500]` deg/s² ranges; a per-joint slice wins over the scalar and the move slows to its most restrictive joint. `MaxTCPSpeedMPerSec` is not honored: capping tool-frame speed needs a Jacobian this module does not compute, so a request that sets only that field runs at the configured default speed and logs a debug line saying so.
>
> **Non-blocking moves: `waitAtEnd` and `interpolate`.** `MoveToJointPositions` and `MoveThroughJointPositions` accept two optional booleans in `extra`, both defaulting to `true`:
>
> ```json
> { "waitAtEnd": false, "interpolate": false }
> ```
>
> - **`waitAtEnd`** — block until the move settles, or return as soon as the goal is on the wire. `wait` is accepted as an alias, since that is the spelling this module's own `set_gripper_rad` uses; `waitAtEnd` is the RDK-originated spelling and wins if both are present.
> - **`interpolate`** — whether a multi-waypoint call is a *path to trace* or only a *route to its last point*. With `false` the waypoints are collapsed to the endpoint and written once.
>
> **These are not an invention of this module.** The builtin motion service's teleop executor sends exactly `{"waitAtEnd": false, "interpolate": false}` to any component it can type-assert to `arm.Arm`, on every tick (rdk `services/motion/builtin/teleop.go`); it only falls back to `GoToInputs` for components that are not arms. Before this was honored, the driver settled on every tick and teleop queued roughly five commands per completed move. Setting `teleop_interpolate_override` on the motion service flips it back to `{"waitAtEnd": true, "interpolate": true}`.
>
> **This changes what a successful return means.** It means *the arm was told*, not *the arm arrived*. There is no settle, so none of the diagnostics above apply: a goal the arm stops short of, or never starts moving toward, returns success. Ask `IsMoving` or `JointPositions` if you need to know where the arm actually is — `IsMoving` reads the hardware on this path, so it still reports the truth.
>
> **A second command supersedes the first**, it does not queue behind it and is not rejected. The in-flight move's context is cancelled and the new goal write replaces the firmware's goal outright. Nothing is left half-applied: one joint command is a single write of all six targets and the firmware interpolates from wherever the arm currently is, so the arm always tracks exactly one goal — the most recent. The cost is that a caller cannot assume any earlier goal was reached. That is the right trade for teleop and the wrong one for a planned path.
>
> **Why `interpolate` exists, and why the two flags belong together.** This arm has exactly one motion primitive: write a goal, and the firmware interpolates toward it on-device. A second write supersedes the first outright. So an intermediate waypoint is only physically distinguishable if the arm is given time to arrive at it — and time to arrive is exactly what `waitAtEnd: false` asks us not to spend. With `interpolate: false` the trajectory is therefore collapsed to its endpoint and written once, rather than writing N goals that supersede each other within a millisecond of bus time and only pretend to trace a path. With `interpolate` left at its default `true`, intermediate waypoints still settle and only the final settle is skipped, so a path the caller does want traced keeps its shape.
>
> **`MoveToPosition` does not honor either flag.** It delegates to the motion service, whose generic execute path drives the arm through `GoToInputs`, which the RDK defines without an `extra` map. A planned path executed without settling between waypoints would collapse to a straight line to the final one, skipping the obstacle avoidance the plan existed for, so this is the correct outcome rather than a gap to be plumbed around. The teleop executor is the exception above: it bypasses `GoToInputs` and calls `MoveThroughJointPositions` directly.
>
> **Streaming trajectories.** `MoveThroughJointPositionsStreamed` (viam-server 1.1.0 or newer) writes each trajectory point on the producer's schedule, at a per-segment speed that covers the segment's travel in its time slot. If the arm starts more than 5 degrees from the first point, one ordinary move closes the gap before the clock starts. Late points are written, not dropped; `Constraints` are ignored. One log line per stream reports gate, late, settle, and wall time.

### 3D models

`Get3DModels` serves one GLB per link (`base_link`, `link1` to `link5`) derived from Waveshare's CAD; the 3D scene tab renders them at each link's geometry pose.

### Communication Methods

The RoArm-M3 supports two communication methods:

#### Serial Communication
For direct USB/serial connection:

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "serial_timeout": "1s",
  "speed_degs_per_sec": 50,
  "acceleration_degs_per_sec_per_sec": 100
}
```

#### HTTP Communication
For wireless control over WiFi:

```json
{
  "host": "192.168.4.1",
  "http_timeout": "5s",
  "speed_degs_per_sec": 50,
  "acceleration_degs_per_sec_per_sec": 100
}
```

Position feedback over HTTP was verified on the bench (a T:105 request to `/js?json=` returns a T:1051 frame), so HTTP mode supports position reads, settle detection, and `IsMoving`. Serial is still recommended for streamed trajectories: each point is one HTTP round trip over the arm's WiFi.


### DoCommand

The module provides several custom commands accessible through the `DoCommand` interface:

#### Set Speed
Change the rotational speed of the joints (3-180 degrees/second):

```json
{
    "command": "set_speed",
    "value": 75
}
```

#### Set Acceleration
Change the acceleration of joints (10-500 degrees/second²):

```json
{
    "command": "set_acceleration",
    "value": 150
}
```

#### Get Motion Parameters
Retrieve current speed and acceleration settings:

```json
{
    "command": "get_motion_params"
}
```

#### Set Torque Control
Enable or disable joint torque:

```json
{
    "command": "set_torque",
    "enable": true
}
```

#### LED Control
Set LED brightness (0-255):

```json
{
    "command": "set_led",
    "brightness": 128
}
```

#### Move to Home Position
Return the arm to its home configuration:

```json
{
    "command": "move_to_home"
}
```

#### Get Full Feedback
Retrieve comprehensive arm status including positions, torques, and Cartesian coordinates:

```json
{
    "command": "get_feedback"
}
```

> [!NOTE]
> When invoking `DoCommand` via `viam machines part run`, the full request body must wrap the command map under the proto `command` field — for example:
> ```
> viam machines part run --part <part-id> --component <arm-name> \
>   --method DoCommand --data '{"command":{"command":"get_feedback"}}'
> ```

#### Checking the link

```json
{
    "command": "comms_health"
}
```

Returns the cumulative counters described in `internal/roarm/health.go`: `retry_pct` climbing means a lossy cable; `never_moved` means the arm is torqued off, obstructed, or at a limit; `stale_frames` above zero means the module is reading unsolicited firmware traffic. A retry rate over 5% across at least 100 reads also logs a rate-limited warning naming this command. Add `"reset": true` to zero the counters for a clean bench measurement; the response still reports the pre-reset values.

#### Gripper RPC bridge (internal)

The following commands exist to let the companion gripper component
control joint 6 through this arm. They are not intended for direct user
use, but are documented here for completeness:

| Command | Params | Returns | Notes |
|---|---|---|---|
| `get_gripper_rad` | — | `{"rad": <float>}` | Current joint-6 position, software frame. |
| `set_gripper_rad` | `{"rad": <float>, "speed"?: deg/s, "acc"?: deg/s², "wait"?: bool}` | `{"success": true}` | Commands joint 6. `wait` (default `true`) blocks until the jaw settles. Rejects `rad` outside `[-0.2, 1.9]`. |
| `stop_gripper` | — | `{"success": true}` | Soft-hold: re-sends the current joint-6 position at 10 deg/s. |

## Joint Limits and Specifications

The module enforces these joint limits, taken from the kinematic model (`roarm_m3.json`, which mirrors the Waveshare URDF). Joint 6 belongs to the [gripper](gripper.md), in the software frame documented there:

| Joint | Range (Radians) | Range (Degrees) | Description |
|-------|-----------------|-----------------|-------------|
| 1     | -3.14 to 3.14   | -180° to 180°   | Base rotation |
| 2     | -1.57 to 1.57   | -90° to 90°     | Shoulder |
| 3     | -1.0 to 2.95    | -57° to 169°    | Elbow |
| 4     | -1.57 to 1.57   | -90° to 90°     | Wrist tilt |
| 5     | -3.14 to 3.14   | -180° to 180°   | Wrist rotation |
| 6     | -0.2 to 1.9     | -11° to 109°    | Gripper |
