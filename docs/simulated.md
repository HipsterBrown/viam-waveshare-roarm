[← Part of the waveshare-roarm module](../README.md)

# Simulated models

`hipsterbrown:waveshare-roarm:simulated` and `hipsterbrown:waveshare-roarm:simulated-gripper` are hardware-free arm and gripper models. They need no serial port or WiFi connection: they share the same kinematics and meshes as the hardware [`arm`](arm.md) and [`gripper`](gripper.md) models and interpolate joint/jaw motion in software, which makes them useful for testing configs, motion plans, and the 3D scene viewer without a physical robot.

### `hipsterbrown:waveshare-roarm:simulated` attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `speed_degs_per_sec` | float64 | Optional | How fast each joint travels toward its target, in degrees/second. Default is `90`. |
| `motion` | string | Optional | The name of the motion service used to plan `MoveToPosition` requests. Default is `builtin`. |
| `simulate_time` | bool | Optional | Whether a background goroutine advances the arm's position in real time. Default is `true`. |
| `collision_geometry` | string | Optional | `box` (default) or `mesh`. `box` uses one axis-aligned box per link sized from the CAD mesh; `mesh` uses a hull-decimated mesh per link for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

### `hipsterbrown:waveshare-roarm:simulated-gripper` attributes

The simulated gripper needs no `arm` attribute: it is fully independent hardware-wise and does not pair with an arm resource.

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `speed_degs_per_sec` | float64 | Optional | How fast the jaw travels toward its target, in degrees/second. Default is `90`. |
| `simulate_time` | bool | Optional | Whether a background goroutine advances the jaw in real time. Default is `true`. |
| `collision_geometry` | string | Optional | `box` (default) or `mesh`. `box` uses an axis-aligned box sized from the CAD mesh; `mesh` uses the hull-decimated jaw mesh for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

### Example: simulated arm with a simulated gripper

Parent the simulated gripper to the simulated arm the same way you would the hardware models:

```json
{
  "components": [
    {
      "name": "sim-arm",
      "api": "rdk:component:arm",
      "model": "hipsterbrown:waveshare-roarm:simulated"
    },
    {
      "name": "sim-gripper",
      "api": "rdk:component:gripper",
      "model": "hipsterbrown:waveshare-roarm:simulated-gripper",
      "frame": { "parent": "sim-arm" }
    }
  ]
}
```

`Grab` on the simulated gripper always returns `false`: the jaw closes toward its grab limit, but a simulated gripper never actually grasps an object.
