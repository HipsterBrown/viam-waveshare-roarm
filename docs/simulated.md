[← Part of the waveshare-roarm module](../README.md)

# Simulated models

`hipsterbrown:waveshare-roarm:simulated` and `hipsterbrown:waveshare-roarm:simulated-gripper` are hardware-free arm and gripper models. They need no serial port or WiFi connection: they share the same kinematics and meshes as the hardware [`arm`](arm.md) and [`gripper`](gripper.md) models and interpolate joint/jaw motion in software, which makes them useful for testing configs, motion plans, and the 3D scene viewer without a physical robot.

### `hipsterbrown:waveshare-roarm:simulated` attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `speed_degs_per_sec` | float64 | Optional | How fast each joint travels toward its target, in degrees/second. Default is `90`. |
| `motion` | string | Optional | The name of the motion service used to plan `MoveToPosition` requests. Default is `builtin`. |
| `orientation_tolerance_deg` | float64 | Optional | The approach-axis cone half-angle, in degrees, that `MoveToPosition` plans against. Default is `30`. Must be in `[0, 180]`. An explicit `0` means the default, **not** an exact orientation match: a zero-leeway goal region is one no inverse-kinematics solution realistically lands inside, so every move would fail to plan. For a near-exact orientation pass a small non-zero value. |
| `position_tolerance_mm` | float64 | Optional | The per-axis positional leeway of the goal region, in millimeters. Default is `1.0`. Must not be negative; an explicit `0` means the default, for the same reason as above. This is a box, not a radius, so the worst-case corner is sqrt(3) times this value. |
| `simulate_time` | bool | Optional | Whether a background goroutine advances the arm's position in real time. Default is `true`. |
| `collision_geometry` | string | Optional | `box` (default) or `mesh`. `box` uses one axis-aligned box per link sized from the CAD mesh; `mesh` uses a tighter envelope per link: the CAD mesh cut into 6 slabs along its length, each replaced by a 26-sided bounding polytope that chamfers the corners and edges the mesh does not reach, for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

`MoveToPosition` plans against the same approach-axis cone as the hardware [`arm`](arm.md) model: the tool's approach axis must arrive within `orientation_tolerance_deg` of the requested orientation, with roll left free. Earlier versions ignored orientation entirely, so a goal that planned before may now fail to plan; widen either tolerance, or pass `{"goal_metric_type": "position_only"}` in `extra` to restore the old behavior, or a raw `{"pose_cloud": {...}}` to send a goal region verbatim. Passing both keys is an error. Goal regions need viam-server 0.127.0 or newer; older servers silently ignore them and plan against the exact pose.

`MoveThroughJointPositions` honors a requested speed, slowing (or speeding up, within the `[3, 180]` deg/s clamp) the interpolation for that move. It ignores a requested acceleration: the simulator interpolates at constant speed with no ramp.

### `hipsterbrown:waveshare-roarm:simulated-gripper` attributes

The simulated gripper needs no `arm` attribute: it is fully independent hardware-wise and does not pair with an arm resource.

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `speed_degs_per_sec` | float64 | Optional | How fast the jaw travels toward its target, in degrees/second. Default is `90`. |
| `simulate_time` | bool | Optional | Whether a background goroutine advances the jaw in real time. Default is `true`. |
| `collision_geometry` | string | Optional | `box` (default) or `mesh`. `box` uses an axis-aligned box sized from the CAD mesh; `mesh` uses a tighter envelope of per-slab bounding polytopes around the jaw for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

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
