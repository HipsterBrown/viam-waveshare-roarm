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
| `collision_geometry`                | string           | Optional     | `box` (default) or `mesh`. `box` uses one axis-aligned box per link sized from the CAD mesh; `mesh` uses a hull-decimated mesh per link for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

*Either `host` or `port` must be specified, but not both.

> **Motion completion.** Every move returns once the firmware's position feedback shows the joints within about 1 degree of the target, or stopped moving (an obstacle), rather than after a computed delay. `IsMoving` compares two feedback samples 40 ms apart and reports true while a move call is in flight.
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
