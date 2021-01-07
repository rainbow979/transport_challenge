# Transport

`from transport_challenge import Transport`

Transport challenge API.

**This extends the Magnebot API. Please read the [Magnebot API documentation](https://github.com/alters-mit/magnebot/blob/main/doc/magnebot_controller.md).**

The Magnebot API includes:

- `init_scene()`
- `turn_by()`
- `turn_to()`
- `move_by()`
- `move_to()`
- `reach_for()`
- `grasp()`
- `drop()`
- `reset_arm()`
- `rotate_camera()`
- `reset_camera()`
- `add_camera()`
- `get_occupancy_position()`
- `end()`

This API includes the following changes and additions:

- Procedurally add **containers** and **target objects** to the scene. Containers are boxes without lids that can hold objects; see the `containers` field. Target objects are small objects that are in navigable positions; see the `target_objects` field. Containers and target objects otherwise behave identically to any other object in terms of physics, segmentation colors, etc. and will appear in the output data alongside the other objects.
- Higher-level actions to pick up target objects and put them in containers.

```python
from transport_challenge import Transport

m = Transport()
# Initializes the scene.
status = m.init_scene(scene="2a", layout=1)
print(status) # ActionStatus.success

# Prints the current position of the Magnebot.
print(m.state.magnebot_transform.position)

# Prints a list of all container IDs.
print(m.containers)
```

***

## Class Variables

| Variable | Type | Description |
| --- | --- | --- |
| `TARGET_OBJECT_MASS ` |  | The mass of each target object. |

***

## Fields

- `target_objects` The IDs of each target object in the scene.

- `containers` The IDs of each container in the scene.

***

## Functions

### Arm Articulation

_These functions move and bend the joints of the Magnebots's arms._

#### pick_up

**`self.pick_up(target, arm)`**

Grasp an object and lift it up. This combines the actions `grasp()` and `reset_arm()`.

Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`
- `cannot_reach`
- `failed_to_grasp` (Either because the motion failed or because the magnet is already holding an object.)
- `failed_to_bend`


| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| target |  int |  | The ID of the target object. |
| arm |  Arm |  | The arm of the magnet that will try to grasp the object. |

_Returns:_  An `ActionStatus` indicating if the magnet at the end of the `arm` is holding the `target` and if not, why.

#### reset_arm

**`self.reset_arm(arm)`**

**`self.reset_arm(arm, reset_torso=True)`**

Reset an arm to its neutral position. Overrides `Magnebot.reset_arm()`.

If the arm is holding a container, it will try to align the bottom of the container with the floor.
This will be somewhat slow the first time the Magnebot does this for this held container.
The Magnebot will also raise itself somewhat higher than it normally would to allow it to align the container.

Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`
- `failed_to_bend`


| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| arm |  Arm |  | The arm that will be reset. |
| reset_torso |  bool  | True | If True, rotate and slide the torso to its neutral rotation and height. |

_Returns:_  An `ActionStatus` indicating if the arm reset and if not, why.

#### put_in

**`self.put_in()`**

Put an object in a container. In order to put an object in a container:

- The Magnebot must be holding a container with one magnet.
- The Magnebot must be holding a target object with another magnet.

This is a multistep action, combining many motions and may require more time than other actions.

Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`
- `not_holding` (If the Magnebot isn't holding a container or target object.)
- `not_in` (If the target object didn't land in the container.)

_Returns:_  An `ActionStatus` indicating if the target object is in the container and if not, why.

#### pour_out

**`self.pour_out()`**

Pour out all of the objects in a container held by one of the Magnebot's magnets.

The Magnebot will extend the arm holding the container and then flip its elbow and wrist.

The action ends when any objects that were in the container stop moving.

Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`
- `not_holding` (If the Magnebot isn't holding a container.)
- `still_in` (If there are objects still in the container.)

_Returns:_  An `ActionStatus` indicating whether the container is now empty and if not, why.

***

