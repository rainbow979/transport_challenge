import random
from json import loads
from csv import DictReader
from typing import List, Dict, Tuple, Optional
import numpy as np
from tdw.py_impact import ObjectInfo, AudioMaterial
from tdw.librarian import ModelLibrarian
from tdw.tdw_utils import TDWUtils, QuaternionUtils
from tdw.output_data import Overlap
from magnebot import Magnebot, Arm, ActionStatus, ArmJoint
from magnebot.scene_state import SceneState
from magnebot.util import get_data
from magnebot.paths import ROOM_MAPS_DIRECTORY, OCCUPANCY_MAPS_DIRECTORY, SCENE_BOUNDS_PATH
from transport_challenge.paths import TARGET_OBJECT_MATERIALS_PATH, TARGET_OBJECTS_PATH, CONTAINERS_PATH


class Transport(Magnebot):
    """
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
    - `drop_all()`
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

    """

    """:class_var
    The mass of each target object.
    """
    TARGET_OBJECT_MASS = 0.25

    # The scale factor of each container relative to its original size.
    __CONTAINER_SCALE = {"x": 0.6, "y": 0.4, "z": 0.6}

    # The mass of a container.
    __CONTAINER_MASS = 1

    # The model librarian.
    __LIBRARIAN = ModelLibrarian()

    # The value of the torso prismatic joint while the Magnebot is holding a container.
    __TORSO_PRISMATIC_CONTAINER = 1.2

    # Load a list of visual materials for target objects.
    __TARGET_OBJECT_MATERIALS = TARGET_OBJECT_MATERIALS_PATH.read_text(encoding="utf-8").split("\n")

    def __init__(self, port: int = 1071, launch_build: bool = True, screen_width: int = 256, screen_height: int = 256,
                 debug: bool = False, auto_save_images: bool = False, images_directory: str = "images"):
        super().__init__(port=port, launch_build=launch_build, screen_width=screen_width, screen_height=screen_height,
                         debug=debug, auto_save_images=auto_save_images, images_directory=images_directory)
        """:field
        The IDs of each target object in the scene.
        """
        self.target_objects: List[int] = list()
        """:field
        The IDs of each container in the scene.
        """
        self.containers: List[int] = list()

        # Cached IK solution for resetting an arm holding a container.
        self._container_arm_reset_angles: Dict[Arm, np.array] = dict()

        # Get all possible target objects. Key = name. Value = scale.
        self._target_objects: Dict[str, float] = dict()
        with open(str(TARGET_OBJECTS_PATH.resolve())) as csvfile:
            reader = DictReader(csvfile)
            for row in reader:
                self._target_objects[row["name"]] = float(row["scale"])
        self._target_object_names = list(self._target_objects.keys())

    def pick_up(self, target: int, arm: Arm) -> ActionStatus:
        """
        Grasp an object and lift it up. This combines the actions `grasp()` and `reset_arm()`.

        Possible [return values](action_status.md):

        - `success`
        - `cannot_reach`
        - `failed_to_grasp`
        - `failed_to_bend`

        :param target: The ID of the target object.
        :param arm: The arm of the magnet that will try to grasp the object.

        :return: An `ActionStatus` indicating if the magnet at the end of the `arm` is holding the `target` and if not, why.
        """

        if target in self.state.held[arm]:
            if self._debug:
                print(f"Already holding {target}")
            return ActionStatus.success

        status = self.grasp(target=target, arm=arm)
        if status != ActionStatus.success:
            self._end_action()
            return status
        return self.reset_arm(arm=arm, reset_torso=True)

    def reset_arm(self, arm: Arm, reset_torso: bool = True) -> ActionStatus:
        """
        Reset an arm to its neutral position. Overrides `Magnebot.reset_arm()`.

        If the arm is holding a container, it will try to align the bottom of the container with the floor.
        This will be somewhat slow the first time the Magnebot does this for this held container.
        The Magnebot will also raise itself somewhat higher than it normally would to allow it to align the container.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`
        - `failed_to_bend`

        :param arm: The arm that will be reset.
        :param reset_torso: If True, rotate and slide the torso to its neutral rotation and height.

        :return: An `ActionStatus` indicating if the arm reset and if not, why.
        """

        # Use cached angles to reset an arm holding a container.
        if arm in self._container_arm_reset_angles:
            return super().reset_arm(arm=arm, reset_torso=reset_torso)

        status = super().reset_arm(arm=arm, reset_torso=reset_torso)
        for object_id in self.state.held[arm]:
            if object_id in self.containers:
                self._start_action()
                magnet_down = QuaternionUtils.get_up_direction(
                    self.state.body_part_transforms[self.magnebot_static.magnets[arm]].rotation)
                # Orient the container to be level with the floor.
                self._start_ik_orientation(orientation=magnet_down, arm=arm, orientation_mode="Y",
                                           object_id=object_id,
                                           fixed_torso_prismatic=Transport.__TORSO_PRISMATIC_CONTAINER)
                status = self._do_arm_motion()
                self._end_action()

                # Cache the arm angles so we can next time immediately reset to this position.
                self._container_arm_reset_angles[arm] = np.array([np.rad2deg(a) for a in
                                                                  self._get_initial_angles(arm=arm)[1:-1]])

                return status
        return status

    def put_in(self) -> ActionStatus:
        """
        Put an object in a container. In order to put an object in a container:

        - The Magnebot must be holding a container with one magnet.
        - The Magnebot must be holding a target object with another magnet.

        This is a multistep action, combining many motions and may require more time than other actions.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`
        - `not_holding` (If the Magnebot isn't holding a container or target object.)
        - `failed_to_grasp` (If the target object didn't land in the container.)

        :return: An `ActionStatus` indicating if the target object is in the container and if not, why.
        """

        # Get the arm holding each object.
        container_arm: Optional[Arm] = None
        container_id = -1
        # Get an arm holding a container.
        for arm in self.state.held:
            for o_id in self.state.held[arm]:
                if container_arm is None and o_id in self.containers:
                    container_id = o_id
                    container_arm = arm
        if container_arm is None:
            if self._debug:
                print("Magnebot isn't holding a container.")
            return ActionStatus.not_holding
        # Check whether the opposite arm is holding a target object.
        object_arm = Arm.left if container_arm == Arm.right else Arm.right
        object_id = None
        for o_id in self.state.held[object_arm]:
            if o_id in self.target_objects:
                object_id = o_id
        if object_id is None:
            if self._debug:
                print(f"Magnebot is holding a container with the {container_arm.name} magnet but isn't holding a "
                      f"target object with the {object_arm.name} magnet.")
            return ActionStatus.not_holding

        self._start_action()
        state = SceneState(resp=self.communicate([]))
        # Bring the container approximately to center.
        self._start_ik(target={"x": 0.1 * (1 if container_arm is Arm.right else -1), "y": 0.4, "z": 0.5},
                       arm=container_arm, absolute=False, allow_column=False, state=state,
                       fixed_torso_prismatic=Transport.__TORSO_PRISMATIC_CONTAINER)
        self._do_arm_motion()
        state = SceneState(resp=self.communicate([]))
        # Move the target object to be over the container.
        target = state.object_transforms[container_id].position + (QuaternionUtils.UP * 0.3)
        self._start_ik(target=TDWUtils.array_to_vector3(target), arm=Arm.left, allow_column=False, state=state,
                       absolute=True, fixed_torso_prismatic=Transport.__TORSO_PRISMATIC_CONTAINER)
        self._do_arm_motion()
        # Drop the object.
        self._append_drop_commands(object_id=object_id, arm=object_arm)
        state_0 = SceneState(resp=self.communicate([]))
        moving = True
        # Set a maximum number of frames to prevent an infinite loop.
        num_frames = 0
        # Wait for the object to stop moving.
        while moving and num_frames < 200:
            moving = False
            state_1 = SceneState(resp=self.communicate([]))
            # Stop if the object somehow fell below the floor.
            if state_1.object_transforms[object_id].position[1] < -1:
                self._end_action()
                return ActionStatus.not_holding
            if np.linalg.norm(state_0.object_transforms[object_id].position -
                              state_1.object_transforms[object_id].position) > 0.001:
                moving = True
            num_frames += 1
            state_0 = state_1

        # Reset the arms.
        self.reset_arm(arm=object_arm, reset_torso=False)
        self.reset_arm(arm=container_arm, reset_torso=False)

        # Check if the object is in the container.
        resp = self.communicate({"$type": "send_overlap_box",
                                 "position": TDWUtils.array_to_vector3(
                                     self.state.object_transforms[container_id].position),
                                 "rotation": TDWUtils.array_to_vector4(
                                     self.state.object_transforms[container_id].rotation),
                                 "half_extents": TDWUtils.array_to_vector3(self.objects_static[container_id].size)})
        overlap = get_data(resp=resp, d_type=Overlap)
        overlap_ids = [int(o_id) for o_id in overlap.get_object_ids() if int(o_id) != container_id]
        self._end_action()
        if object_id in overlap_ids:
            return ActionStatus.success
        else:
            if self._debug:
                print(f"Object {object_id} isn't in container {container_id}")
            return ActionStatus.failed_to_grasp

    def drop(self, target: int, arm: Arm) -> ActionStatus:
        status = super().drop(target=target, arm=arm)
        if status == ActionStatus.success:
            # Remove the cached container arm angles.
            if arm in self._container_arm_reset_angles:
                del self._container_arm_reset_angles[arm]
        return status

    def drop_all(self) -> ActionStatus:
        self._container_arm_reset_angles.clear()
        return super().drop_all()

    def get_scene_init_commands(self, scene: str, layout: int, audio: bool) -> List[dict]:
        # Clear the list of target objects and containers.
        self.target_objects.clear()
        self.containers.clear()
        commands = super().get_scene_init_commands(scene=scene, layout=layout, audio=audio)

        # Load the map of the rooms in the scene, the occupancy map, and the scene bounds.
        room_map = np.load(str(ROOM_MAPS_DIRECTORY.joinpath(f"{scene[0]}.npy").resolve()))
        self.occupancy_map = np.load(str(OCCUPANCY_MAPS_DIRECTORY.joinpath(f"{scene[0]}_{layout}.npy").resolve()))
        self._scene_bounds = loads(SCENE_BOUNDS_PATH.read_text())[scene[0]]

        # Sort all free positions on the occupancy map by room.
        rooms: Dict[int, List[Tuple[int, int]]] = dict()
        for ix, iy in np.ndindex(room_map.shape):
            room_index = room_map[ix][iy]
            if room_index not in rooms:
                rooms[room_index] = list()
            if self.occupancy_map[ix][iy] == 0:
                rooms[room_index].append((ix, iy))
        # Choose a random room.
        room_positions: List[Tuple[int, int]] = random.choice(list(rooms.values()))

        # Add target objects to the room.
        for i in range(random.randint(8, 12)):
            ix, iy = random.choice(room_positions)
            # Get the (x, z) coordinates for this position.
            # The y coordinate is in `ys_map`.
            x, z = self.get_occupancy_position(ix, iy)
            self._add_target_object(model_name=random.choice(self._target_object_names),
                                    position={"x": x, "y": 0, "z": z})

        # Add containers throughout the scene.
        containers = CONTAINERS_PATH.read_text(encoding="utf-8").split("\n")
        for room_index in list(rooms.keys()):
            # Maybe don't add a container in this room.
            if random.random() < 0.25:
                continue
            # Get a random position in the room.
            ix, iy = random.choice(rooms[room_index])

            # Get the (x, z) coordinates for this position.
            # The y coordinate is in `ys_map`.
            x, z = self.get_occupancy_position(ix, iy)
            container_name = random.choice(containers)
            self._add_container(model_name=container_name,
                                position={"x": x, "y": 0, "z": z},
                                rotation={"x": 0, "y": random.uniform(-179, 179), "z": 0})
        return commands

    def _add_container(self, model_name: str, position: Dict[str, float] = None,
                       rotation: Dict[str, float] = None) -> int:
        """
        Add a container. Cache the ID.

        :param model_name: The name of the container.
        :param position: The initial position of the container.
        :param rotation: The initial rotation of the container.

        :return: The ID of the container.
        """

        object_id = self._add_object(position=position,
                                     rotation=rotation,
                                     scale=Transport.__CONTAINER_SCALE,
                                     audio=self._OBJECT_AUDIO[model_name],
                                     model_name=model_name)
        self.containers.append(object_id)
        # Set a light mass for each container.
        self._object_init_commands[object_id].append({"$type": "set_mass",
                                                      "id": object_id,
                                                      "mass": Transport.__CONTAINER_MASS})
        return object_id

    def _add_target_object(self, model_name: str, position: Dict[str, float]) -> int:
        """
        Add a targt object. Cache  the ID.

        :param model_name: The name of the target object.
        :param position: The initial position of the target object.

        :return: The ID of the target object.
        """
        # Set custom object info for the target objects.
        audio = ObjectInfo(name=model_name, mass=Transport.TARGET_OBJECT_MASS,
                           material=AudioMaterial.ceramic, resonance=0.6, amp=0.01, library="models_core.json",
                           bounciness=0.5)
        scale = self._target_objects[model_name]
        # Add the object.
        object_id = self._add_object(position=position,
                                     rotation={"x": 0, "y": random.uniform(-179, 179), "z": 0},
                                     scale={"x": scale, "y": scale, "z": scale},
                                     audio=audio,
                                     model_name=model_name)
        self.target_objects.append(object_id)
        # Set a random visual material for each target object.
        visual_material = random.choice(Transport.__TARGET_OBJECT_MATERIALS)
        substructure = Transport.__LIBRARIAN.get_record(model_name).substructure
        self._object_init_commands[object_id].extend(TDWUtils.set_visual_material(substructure=substructure,
                                                                                  material=visual_material,
                                                                                  object_id=object_id,
                                                                                  c=self))
        return object_id

    def _get_reset_arm_commands(self, arm: Arm, reset_torso: bool) -> List[dict]:
        if arm in self._container_arm_reset_angles:
            self._append_ik_commands(angles=self._container_arm_reset_angles[arm], arm=arm)
            return list()
        else:
            return super()._get_reset_arm_commands(arm=arm, reset_torso=reset_torso)

    def _start_move_or_turn(self) -> None:
        """
        Start a move or turn action.
        """

        if len(self._container_arm_reset_angles) > 0:
            # Move the torso up to its default height to prevent anything from dragging.
            self._next_frame_commands.append({"$type": "set_prismatic_target",
                                              "joint_id": self.magnebot_static.arm_joints[ArmJoint.torso],
                                              "target": Transport.__TORSO_PRISMATIC_CONTAINER})
        else:
            # Move the torso up to its default height to prevent anything from dragging.
            self._next_frame_commands.append({"$type": "set_prismatic_target",
                                              "joint_id": self.magnebot_static.arm_joints[ArmJoint.torso],
                                              "target": Magnebot._DEFAULT_TORSO_Y})
        self._do_arm_motion()
