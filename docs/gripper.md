[← Part of the waveshare-roarm module](../README.md)

# Model hipsterbrown:waveshare-roarm:gripper

The gripper component controls the 6th joint of the RoArm-M3, which functions as a parallel gripper.

The gripper does not own its own hardware connection. It holds an arm-component client (obtained via the `arm` dependency) and routes every joint-6 operation — opening, closing, position reads, soft-stop — through the arm's `DoCommand` RPC bridge. The only required gripper attribute is the name of the arm it pairs with; no `host`/`port`/`baudrate`/timeout fields are required (or accepted).

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
|-------|--------|-----------|--------------------------------------------------------------------------------------------------------|
| `arm` | string | Required  | The name of the arm resource this gripper shares hardware with. Must refer to a `waveshare-roarm:arm`. |
| `collision_geometry` | string | Optional | `box` (default) or `mesh`. `box` uses one axis-aligned box per link sized from the CAD mesh; `mesh` uses a hull-decimated mesh per link for planning. Both place their geometry at the same point, so the 3D scene looks the same either way. `mesh` costs more planning time. |

The gripper's `Geometries` serves the moving jaw's full-resolution mesh posed at its live angle, for the 3D scene and live obstacle checks. The fixed jaw is not modeled separately — it's part of the arm's [`link5`](arm.md) model.

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

The arm's `tool` frame is 52.035 mm along the wrist-roll (`link5`) axis. Parent the gripper to the arm with no offset:

```json
"frame": { "parent": "<arm-name>" }
```

The gripper reports a zero-DoF kinematic model whose leaf, `tcp`, is the grasp point between the closed jaw tips — the TCP is 63.39 mm beyond the `tool` frame along the approach axis. `GetPose("<gripper>", "world")` and motion requests that target the gripper resolve there. The model's collision geometry (see `collision_geometry` above) covers only the moving jaw; the fixed jaw's collision geometry belongs to the arm's `link5`, which is what makes the gripper an obstacle for motion planning: viam-server takes collision geometry for arm and gripper components from their kinematic model. Do not add a compensating translation to the gripper's `frame`.
