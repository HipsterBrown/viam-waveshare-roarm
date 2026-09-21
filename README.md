# Viam WaveShare RoArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [WaveShare's](https://www.waveshare.com/) RoArm-M3 5-DOF + Gripper collaborative robotic arm.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This waveshare-roarm module is particularly useful in applications that require a RoArm-M3 to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separately through your own code.

Navigate to the **CONFIGURE** tab of your machine's page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / hipsterbrown:waveshare-roarm:arm` model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

> [!NOTE]
> Before configuring your RoArm-M3, you must [add a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

## Models

- [`hipsterbrown:waveshare-roarm:arm`](docs/arm.md) — controls the first 5 joints of the RoArm-M3: base, shoulder, elbow, wrist, and roll.
- [`hipsterbrown:waveshare-roarm:gripper`](docs/gripper.md) — controls the 6th joint (parallel gripper), routed through the arm's `DoCommand` RPC bridge.
- [`hipsterbrown:waveshare-roarm:simulated`](docs/simulated.md) — hardware-free arm sharing the same kinematics and meshes as `arm`, for testing and 3D visualization without a physical robot.
- [`hipsterbrown:waveshare-roarm:simulated-gripper`](docs/simulated.md) — hardware-free gripper sharing the same jaw mesh as `gripper`, for testing and 3D visualization without a physical robot.

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
- Geometry: the 52.035 mm mount offset, 63.393 mm mount-to-jaw-tip distance, and the 25.2 x 39.4 x 77.8 mm jaw collision box match the physical arm.
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

- `MoveToPosition` now constrains the tool's approach direction instead of ignoring orientation entirely: it sends an approach-axis cone, 30 degrees half-angle by default, so a goal that planned before may now fail to plan. Pass `{"goal_metric_type": "position_only"}` in `extra` to restore the old orientation-free behavior. To widen the cone instead, use the new `orientation_tolerance_deg` and `position_tolerance_mm` attributes on [`arm`](docs/arm.md) and [`simulated`](docs/simulated.md); an explicit `0` for either one means "use the default," not "demand an exact match."
- A move that fails to reach its target now returns an error instead of succeeding silently. A move that makes genuine progress and then stops short still succeeds, with a warning logged. If your code relied on `MoveToJointPositions` always returning `nil`, it will now see errors it never saw before, and every one of them was already real.
- Requires viam-server 1.1.0 or newer.
- Joint limits are read from the kinematic model (±180°, ±90°, -57° to 169°, ±90°, ±180°) and may be narrower than before.
- The arm's frame is now the gripper mount (`tool`), not the wrist-roll axis. Remove any offset you added to the gripper's `frame`.
- `MoveToJointPositions` no longer accepts `speed` / `acceleration` in `extra`; use the `set_speed` and `set_acceleration` commands.
- The gripper's `set_position` range is now -11.5 to 108.9 degrees, and its `speed` / `acc` parameters are in deg/s and deg/s². `GoToInputs` no longer drives the jaw.
- `move_to_home` runs at the configured speed (default 50 deg/s), keeps the gripper where it is, and returns after the arm settles.
- `IsMoving` on arm and gripper is read from hardware; the gripper no longer reports arm motion.
- Speed and acceleration attributes are validated at config time.
- Kinematics now come from Waveshare's URDF. For the same joint angles, reported and planned poses move by up to about 39 mm at the wrist compared with 1.x (link lengths 236.8 and 144.6 mm instead of 250 and 160, plus a 15.1 mm wrist offset). The wrist-roll joint frame is `link4_to_link5` (was `link4_to_gripper`) under a new `link5` link; `tool` is unchanged.
- The gripper's collision geometry is the moving jaw only; the fixed jaw belongs to the arm's `link5`.

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
