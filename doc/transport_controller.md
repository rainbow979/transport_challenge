# Transport

`from transport_challenge import Transport`

Transport challenge API.

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

**This extends the Magnebot API. Please read the [Magnebot API documentation](https://github.com/alters-mit/magnebot/blob/main/doc/magnebot_controller.md).**

This API includes the following changes and additions:

- Procedurally add **containers** and **target objects** to the scene. Containers are boxes without lids that can hold objects; see the `containers` field. Target objects are small objects that must be transported to the goal zone; see the `target_objects` field. These containers and target objects are included alongside all other objects in [`self.objects_static` and `self.state`](https://github.com/alters-mit/magnebot/blob/main/doc/magnebot_controller.md#fields).    - Higher-level actions to pick up target objects and put them in containers.
- A few new actions: `pick_up()`, `put_in()`, and `pour_out()`
- Modified behavior for certain Magnebot actions such as `reset_arm()`
- An interaction budget. The field `num_actions` increments by an action's "cost" at the end of the action:

| Action | Cost |
| --- | --- |
| `init_scene()` | 0 |
| `turn_by()` | 1 |
| `turn_to()` | 1 |
| `move_by()` | 1 |
| `move_to()` | 2 |
| `reset_position()` | 1 |
| `reach_for()` | 1 |
| `grasp()` | 1 |
| `drop()` | 1 |
| `reset_arm()` | 1 |
| `rotate_camera()` | 0 |
| `reset_camera()` | 0 |
| `add_camera()` | 0 |
| `end()` | 0 |
| `pick_up()` | 2 |
| `put_in()` | 1 |
| `pour_out()` | 1 |

***

## Class Variables

| Variable | Type | Description |
| --- | --- | --- |
| `TARGET_OBJECT_MASS` | float | The mass of each target object. |
| `GOAL_ZONE_RADIUS` | float | The goal zone is a circle defined by `self.goal_center` and this radius value. |

***

## Fields

- `target_objects` The IDs of each target object in the scene.

- `containers` The IDs of each container in the scene.

- `num_actions` The total number of actions taken by the Magnebot.

- `goal_position` The challenge is successful when the Magnebot moves all of the target objects to the the goal zone, which is defined by this position and `Transport.GOAL_ZONE_RADIUS`. This value is set in `init_scene()`.

- `goal_room` The room that `self.goal_position` is in. [See here](https://github.com/alters-mit/magnebot/tree/main/doc/images/rooms) for images of the rooms. This value is set in `init_scene()`.

- `done` If True, the Magnebot successfully transported all of the objects to the goal zone. This is updated at the end of every action, including actions with 0 cost.

***

## Functions

### Transport Challenge

_These functions are unique to the Transport Challenge API._

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

### Inherited from Magnebot

_These functions are inherited from the Magnebot API but include additional functionality. Read the Magnebot API for a list of all available functions._

#### init_scene

**`self.init_scene(scene, layout)`**

**`self.init_scene(scene, layout, room=None, goal_room=None)`**

This is the same function as `Magnebot.init_scene()` but it adds target objects and containers to the scene.

When `init_scene()` is called, 8-12 target objects will be randomly placed on the floor of a randomly-selected room. Then, there is a 25% chance of adding one container per room.


Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| scene |  str |  | The name of an interior floorplan scene. Each number (1, 2, etc.) has a different shape, different rooms, etc. Each letter (a, b, c) is a cosmetically distinct variant with the same floorplan. |
| layout |  int |  | The furniture layout of the floorplan. Each number (0, 1, 2) will populate the floorplan with different furniture in different positions. |
| room |  int  | None | The index of the room that the Magnebot will spawn in the center of. If None, the room will be chosen randomly. |
| goal_room |  int  | None | The goal room. If None, this is chosen randomly. See field descriptions of `goal_room` and `goal_position` in this document. |

_Returns:_  An `ActionStatus` (always success).

#### reset_arm

**`self.reset_arm(arm)`**

**`self.reset_arm(arm, reset_torso=True)`**

This is the same as `Magnebot.reset_arm()` unless the arm is holding a container.

If the arm is holding a container, it will try to align the bottom of the container with the floor. This will be somewhat slow the first time the Magnebot does this for this held container.

Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

- `success`
- `failed_to_bend`


| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| arm |  Arm |  | The arm that will be reset. |
| reset_torso |  bool  | True | If True, rotate and slide the torso to its neutral rotation and height. |

_Returns:_  An `ActionStatus` indicating if the arm reset and if not, why.

***

