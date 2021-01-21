# Changelog

## 0.1.2

### `Transport` class

- Set `launch_build` default value to False (was True)
- Fixed: `put_in()` often thinks that an object isn't in a container when it actually is.
- Fixed: `put_in()` sometimes fails because the magnet holding the target object is in the way. Now, the wrist of that magnet will tilt down, allowing for a clearer trajectory.
- Fixed: Sometimes in `put_in()` the arm that was holding the target object gets caught on the container. Now, the arm that was holding the target object resets first, rather than vice versa.
- Backend:
  - `_get_objects_in_container()` uses trigger event data (`self._trigger_events`) to determine if an object is in a container rather than `Overlap` output data. It no longer advances the simulation an extra frame.

## 0.1.1

### `Transport` class

- Added optional parameter `img_is_png` to the constructor
- Renamed `num_actions` to `action_cost`
- Added: `get_target_objects_in_goal_zone()`

### Example controllers

- Cleaned up the code of `single_room.py`

### Documentation

- Added links to controllers in README