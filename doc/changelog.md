# Changelog

## 0.1.4

### `Transport`

- `put_in()` will immediately stop moving the arm holding the object if the magnet or the object is within the container.
- Improved the speed of `put_in()` and `pour_out()`.
- Fixed: `put_in()` is too slow because the arm holding the object isn't sufficiently above the container.

## 0.1.3

### `Transport`

- Added optional parameter `skip_frames` (required in Magnebot 0.4.0)
- Improved the speed of `put_in()`.
- Fixed: `pick_up()` doesn't reset the arm if the magnet fails to grasp the target object.
- Fixed: `pick_up()` often misses the object (it now aims for positions on the top of the object).
- Fixed: `put_in()` sometimes doesn't stop the Magnebot's wheels from turning.
- Fixed: `put_in()` often aims for the wrong target position above the container.
- Fixed: `pour_out()` sometimes doesn't stop the Magnebot's wheels from turning.
- Fixed: while objects are being dropped into a container during `put_in()`, they sometimes glitch (now they use the `discrete` collision detection mode).

### Example controllers

- Fixed: `single_room.py` doesn't work. Added simple navigation to make multiple attempts to pick up the target object.

### Test controllers

- Removed: `level.py`

## 0.1.2

### `Transport` 

- Set `launch_build` default value to False (was True)
- Fixed: `put_in()` often thinks that an object isn't in a container when it actually is.
- Fixed: `put_in()` sometimes fails because the magnet holding the target object is in the way. Now, the wrist of that magnet will tilt down, allowing for a clearer trajectory.
- Fixed: Sometimes in `put_in()` the arm that was holding the target object gets caught on the container. Now, the arm that was holding the target object resets first, rather than vice versa.
- Backend:
  - `_get_objects_in_container()` uses trigger event data (`self._trigger_events`) to determine if an object is in a container rather than `Overlap` output data. It no longer advances the simulation an extra frame.

## 0.1.1

### `Transport` 

- Added optional parameter `img_is_png` to the constructor
- Renamed `num_actions` to `action_cost`
- Added: `get_target_objects_in_goal_zone()`

### Example controllers

- Cleaned up the code of `single_room.py`

### Documentation

- Added links to controllers in README