# Transport Challenge

The **Transport Challenge API** is an extension of the [Magnebot API](https://github.com/alters-mit/magnebot) which in turn is built on the [TDW simulation platform](https://github.com/threedworld-mit/tdw). In the Transport Challenge API, the Magnebot must transport **target objects** with the aid of **containers** from one room to the **goal zone**.

**[Read the API documentation here.](doc/transport_controller.md)**

<img src="doc/images/api_hierarchy.png" style="zoom:67%;" />

# Installation

1. [Follow instructions for installing the Magnebot API](https://github.com/alters-mit/magnebot)
2. Clone this repo
3. `cd path/to/transport_challenge` (Replace `path/to` with the actual path)
4. `pip3 install -e .`

# Usage

1. Run this controller:

```python
from transport_challenge import Transport

m = Transport()
# Initializes the scene.
status = m.init_scene(scene="2a", layout=1)
print(status)  # ActionStatus.success

# Prints a list of all container IDs.
print(m.containers)
# Prints a list of all target object IDs.
print(m.target_objects)

m.end()
```

2. [Launch the TDW build.](https://github.com/threedworld-mit/tdw/blob/master/Documentation/getting_started.md)

# Concepts

- **Target objects** are small objects that must be transported. At the start of the simulation, the target objects are scattered on the floor of a single randomly-chosen room. See: `self.target_objects`
- **Containers** are box-shaped objects without lids that can hold target objects. At the start of the simulation, there is a 25% chance that a container will be placed on the floor of a room. See: `self.containers` 
- The **goal zone** is a circle defined by a position and a radius in the center of a room and the scene. If all of the target objects are in this circle, the task is successful.

# Documentation

**[Read the API documentation here.](doc/transport_controller.md)** The API includes all of the actions in the Magnebot API plus actions unique to the Transport Challenge API: `pick_up()`, `put_in()`, and `pour_out()`.

# Examples

See: `controllers/examples/single_room.py`

