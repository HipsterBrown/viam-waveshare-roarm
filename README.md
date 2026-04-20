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
| `speed_degs_per_sec`                | float32          | Optional     | The rotational speed for arm movements (must be between 3 and 180). Default is `50` degrees/second.           |
| `acceleration_degs_per_sec_per_sec` | float32          | Optional     | The acceleration for arm movements (must be between 10 and 500). Default is `100` degrees/second^2.           |

*Either `host` or `port` must be specified, but not both.

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


## Model hipsterbrown:waveshare-roarm:gripper

The gripper component controls the 6th joint of the RoArm-M3, which functions as a parallel gripper.

The gripper shares hardware (HTTP/serial transport) with the arm it is paired with by borrowing the arm's controller handle. As a result the gripper configuration only needs a reference to the arm it depends on — no `host`/`port`/`baudrate`/timeout fields are required (or accepted).

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
Set the gripper to a specific position (-10 to 100 degrees):

```json
{
    "command": "set_position",
    "degrees": 45,
    "speed": 500,
    "acc": 50
}
```

## Frame System

The gripper exposes a bounding-box geometry (roughly 70 mm × 40 mm × 60 mm, offset 30 mm beyond the wrist) that the motion service can treat as an obstacle during planning. For that geometry to be placed correctly in the world, the gripper's parent frame must be declared at the machine-config level — typically `link4` of the arm, which is the end of the arm's kinematic chain.

Add a `frame` block to the gripper component in your machine config:

```json
"frame": {
  "parent": "<arm-name>:link4",
  "translation": {"x": 0, "y": 0, "z": 0},
  "orientation": {"type": "euler_angles", "value": {"pitch": 0, "roll": 0, "yaw": 0}}
}
```

Replace `<arm-name>` with the `name` of your `waveshare-roarm:arm` component. Without this frame declaration, the gripper's bounding box will not be attached to the arm's end effector and motion planning will not account for it.

## Joint Limits and Specifications

The RoArm-M3 has the following joint limits:

| Joint | Range (Radians) | Range (Degrees) | Description |
|-------|-----------------|-----------------|-------------|
| 1     | -3.3 to 3.3     | -189° to 189°   | Base rotation |
| 2     | -1.9 to 1.9     | -109° to 109°   | Shoulder |
| 3     | -1.2 to 3.3     | -69° to 189°    | Elbow |
| 4     | -1.9 to 1.9     | -109° to 109°   | Wrist tilt |
| 5     | -3.3 to 3.3     | -189° to 189°   | Wrist rotation |
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

- Use HTTP communication for better performance when possible
- Joint position caching has been removed for more consistent real-time feedback

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
