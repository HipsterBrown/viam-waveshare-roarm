# Viam WaveShare RoArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [WaveShare's](https://www.waveshare.com/) RoArm-M3 5-DOF + Gripper collaborative robotic arm.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This waveshare-roarm module is particularly useful in applications that require a RoArm-M3 to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separately through your own code.

Navigate to the **CONFIGURE** tab of your machine's page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / hipsterbrown:waveshare-roarm:arm` model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

> [!NOTE]
> Before configuring your RoArm-M3, you must [add a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

## Model hipsterbrown:waveshare-roarm:arm

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

*Either `host` or `port` must be specified, but not both.

> **Motion completion.** Every move returns once the firmware's position feedback shows the joints within about 1 degree of the target, or stopped moving (an obstacle), rather than after a computed delay. `IsMoving` compares two feedback samples 40 ms apart and reports true while a move call is in flight.
>
> **Streaming trajectories.** `MoveThroughJointPositionsStreamed` (viam-server 1.1.0 or newer) writes each trajectory point on the producer's schedule, at a per-segment speed that covers the segment's travel in its time slot. If the arm starts more than 5 degrees from the first point, one ordinary move closes the gap before the clock starts. Late points are written, not dropped; `Constraints` are ignored. One log line per stream reports gate, late, settle, and wall time.

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


## Model hipsterbrown:waveshare-roarm:gripper

The gripper component controls the 6th joint of the RoArm-M3, which functions as a parallel gripper.

The gripper does not own its own hardware connection. It holds an arm-component client (obtained via the `arm` dependency) and routes every joint-6 operation — opening, closing, position reads, soft-stop — through the arm's `DoCommand` RPC bridge. The only gripper attribute is the name of the arm it pairs with; no `host`/`port`/`baudrate`/timeout fields are required (or accepted).

### Configuration

```json
{
  "name": "my-gripper",
  "api": "rdk:component:gripper",
  "model": "hipsterbrown:waveshare-roarm:gripper",
  "attributes": {
    "arm": "my-arm"
  },
  "depends_on": ["my-arm"]
}
```

### Attributes

| Name  | Type   | Inclusion | Description                                                                                          |
|-------|--------|-----------|------------------------------------------------------------------------------------------------------|
| `arm` | string | Required  | The name of the arm resource this gripper shares hardware with. Must refer to a `waveshare-roarm:arm`. |

### DoCommand

The module provides several custom commands accessible through the `DoCommand` interface:

#### Get Gripper Position
Get the current gripper position in degrees:

```json
{
    "command": "get_position"
}
```

#### Set Gripper Position
Set the gripper to a specific position (-11.5 to 108.9 degrees):

```json
{
    "command": "set_position",
    "degrees": 45,
    "speed": 500,
    "acc": 50
}
```

`degrees` must be between `-11.5` and `108.9`. `speed` (deg/s, default `50`), `acc` (deg/s², default `100`), and `wait` (default `true`) are optional.

## Frame system

The arm's frame is the gripper mount: a `tool` link 52 mm beyond the wrist-roll axis along the roll axis. Parent the gripper to the arm with no offset:

```json
"frame": { "parent": "<arm-name>" }
```

The gripper reports a zero-DoF kinematic model whose leaf, `tcp`, is the grasp point between the closed jaw tips, 63.4 mm beyond the mount along the approach axis. `GetPose("<gripper>", "world")` and motion requests that target the gripper resolve there. The same model carries a 70 x 40 x 70 mm box for the jaw envelope, which is what makes the gripper an obstacle for motion planning: viam-server takes collision geometry for arm and gripper components from their kinematic model. Do not add a compensating translation to the gripper's `frame`.

## Joint Limits and Specifications

The module enforces these joint limits, taken from the kinematic model (`roarm_m3.json`, which mirrors the Waveshare URDF). Joint 6 is the gripper, in the software frame:

| Joint | Range (Radians) | Range (Degrees) | Description |
|-------|-----------------|-----------------|-------------|
| 1     | -3.14 to 3.14   | -180° to 180°   | Base rotation |
| 2     | -1.57 to 1.57   | -90° to 90°     | Shoulder |
| 3     | -1.0 to 2.95    | -57° to 169°    | Elbow |
| 4     | -1.57 to 1.57   | -90° to 90°     | Wrist tilt |
| 5     | -3.14 to 3.14   | -180° to 180°   | Wrist rotation |
| 6     | -0.2 to 1.9     | -11° to 109°    | Gripper |

## WiFi Configuration

The RoArm-M3 can operate in different WiFi modes:

### AP Mode (Access Point)
The arm creates its own WiFi network:
- **SSID**: Usually `RoArm-M3_XXXXXX`
- **Default IP**: `192.168.4.1`
- **Password**: Check the arm's documentation or display

### STA Mode (Station)
The arm connects to your existing WiFi network. You'll need to configure this through the arm's web interface or using the WaveShare RoArm SDK.

## Troubleshooting

### Connection Issues

1. **HTTP Connection Failed**:
   - Verify the arm is powered on and the IP address is correct
   - Ensure your computer is on the same network as the arm
   - Try pinging the IP address: `ping 192.168.4.1`

2. **Serial Connection Failed**:
   - Check that the USB cable is properly connected
   - Verify the correct port (Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`; Windows: `COM3`, `COM4`, etc.)
   - Ensure no other applications are using the serial port

### Performance Tips

- To trace raw serial frames while debugging wire issues, set the `ROARM_WIRE_TRACE=1` environment variable before starting the module. Each sent command and received buffer will be logged at debug level.

### Hardware notes

Measured on a RoArm-M3 over USB serial at 115200 baud:

- Speed and acceleration units: joints 1, 2 and 5 commanded at 20, 50 and 100 deg/s with `cmd/cli time-move` ran within 10% of the commanded speed, confirming the 4096 steps/rev conversion; no constant adjustment was needed.
- Settle detection: a 30 degree move at 50 deg/s returned within 150 ms of the arm stopping, polling feedback every 50 ms with no frame errors.
- Joint limits: joints 2 and 3 reach the model limits (±90°, -57° to 169°) without binding.
- Geometry: the 52 mm mount offset, 63 mm mount-to-jaw-tip distance, and 70 x 40 x 70 mm jaw envelope match the physical arm.
- Gripper: `Open`, an empty `Grab` (false) and a `Grab` on an object (true) each return when the jaw stops; gripper `IsMoving` stays false while only the arm moves.
- Reconfigure: a speed-only change does not reopen the serial port; a bad port leaves the previous connection working and reports the error.
- Streamed trajectories (50 points at 10 Hz, ±0.3 rad sine on joint 1, `cmd/streambench`):
  - live producer, one point per batch at its own time: wall 5.30 s for a 4.9 s trajectory (34 ms start gate, 460 ms final settle), 0 late points. The arm follows one segment (100 ms) behind a live producer, which is inherent: a goal can only be written once it is known.
  - whole trajectory in one batch: wall 4.90 s, 103 ms final settle, 0 late points. Each point is written when its predecessor is due, so the arm arrives on schedule.
  - `Stop` 2.5 s into the stream: the call returned `context canceled` at 2.50 s after 25 acknowledged batches and the arm held.
  - no frame-corruption warnings at 10 writes per second.

### Data Robustness

The module reads joint positions from the firmware's JSON feedback stream.
Two robustness layers are applied to handle occasional wire-level issues:

- **Torn-frame recovery**: if the firmware occasionally emits a torn blob
  (two partial frames merged without a `}\r\n{` boundary between them), the
  reader walks `}\r\n` terminators from most recent to oldest and falls
  back to any clean earlier frame in the buffer rather than surfacing a
  parse error.
- **Gripper frame transform**: joint 6 uses a firmware frame that is
  offset and inverted from the software frame (`raw closed ≈ π rad`).
  All gripper reads and writes apply the `π − r` transform so that the
  software API stays in the `[-0.2, 1.9]` rad ( `~−11°` to `~109°` )
  convention inherited from the upstream Waveshare Python SDK.

## Migrating from 0.x

- Requires viam-server 1.1.0 or newer.
- Joint limits are read from the kinematic model (±180°, ±90°, -57° to 169°, ±90°, ±180°) and may be narrower than before.
- The arm's frame is now the gripper mount (`tool`), not the wrist-roll axis. Remove any offset you added to the gripper's `frame`.
- `MoveToJointPositions` no longer accepts `speed` / `acceleration` in `extra`; use the `set_speed` and `set_acceleration` commands.
- The gripper's `set_position` range is now -11.5 to 108.9 degrees, and its `speed` / `acc` parameters are in deg/s and deg/s². `GoToInputs` no longer drives the jaw.
- `move_to_home` runs at the configured speed (default 50 deg/s), keeps the gripper where it is, and returns after the arm settles.
- `IsMoving` on arm and gripper is read from hardware; the gripper no longer reports arm motion.
- Speed and acceleration attributes are validated at config time.

## WaveShare RoArm-M3 Resources

The following documents will be useful for developers and users:

- [WaveShare RoArm-M3 Product Page](https://www.waveshare.com/roarm-m3.htm)
- [RoArm-M3 User Manual](https://www.waveshare.com/wiki/RoArm-M3)

## Hardware Setup

1. **Power**: Connect the 12V power adapter to the arm's base
2. **Communication**: 
   - For WiFi: Connect to the arm's WiFi network or configure it to connect to yours
   - For USB: Connect the USB-C cable between the arm and your computer
3. **Initial Position**: Manually position the arm in a safe configuration before powering on

## Safety Notes

> [!WARNING]
> - Always ensure the arm's workspace is clear before operation
> - The arm can move quickly - maintain safe distances during operation
> - Use the torque control features to enable safe manual positioning when needed
