from json import loads
from csv import DictReader
from typing import List, Dict, Tuple, Optional, Union
import numpy as np
from tdw.py_impact import ObjectInfo, AudioMaterial
from tdw.librarian import ModelLibrarian
from tdw.tdw_utils import TDWUtils
from magnebot import Magnebot, Arm, ActionStatus, ArmJoint
from magnebot.scene_state import SceneState
from magnebot.paths import ROOM_MAPS_DIRECTORY, OCCUPANCY_MAPS_DIRECTORY, SCENE_BOUNDS_PATH, SPAWN_POSITIONS_PATH
from transport_challenge.paths import TARGET_OBJECT_MATERIALS_PATH, TARGET_OBJECTS_PATH, CONTAINERS_PATH


class Transport(Magnebot):
    """
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
    - An interaction budget. The field `action_cost` increments by an action's "cost" at the end of the action:

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
    | `get_target_objects_in_goal_zone()` | 0 |
    """

    """:class_var
    The mass of each target object.
    """
    TARGET_OBJECT_MASS: float = 0.25

    """:class_var
    The goal zone is a circle defined by `self.goal_center` and this radius value.
    """
    GOAL_ZONE_RADIUS: float = 1

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

    def __init__(self, port: int = 1071, launch_build: bool = False, screen_width: int = 256, screen_height: int = 256,
                 debug: bool = False, auto_save_images: bool = False, images_directory: str = "images",
                 random_seed: int = None, img_is_png: bool = True, skip_frames: int = 20):
        """
        :param port: The socket port. [Read this](https://github.com/threedworld-mit/tdw/blob/master/Documentation/getting_started.md#command-line-arguments) for more information.
        :param launch_build: If True, the build will launch automatically on the default port (1071). If False, you will need to launch the build yourself (for example, from a Docker container).
        :param screen_width: The width of the screen in pixels.
        :param screen_height: The height of the screen in pixels.
        :param auto_save_images: If True, automatically save images to `images_directory` at the end of every action.
        :param images_directory: The output directory for images if `auto_save_images == True`.
        :param debug: If True, enable debug mode. This controller will output messages to the console, including any warnings or errors sent by the build. It will also create 3D plots of arm articulation IK solutions.
        :param random_seed: The random seed used for setting the start position of the Magnebot, the goal room, and the target objects and containers.
        :param img_is_png: If True, the `img` pass images will be .png files. If False, the `img` pass images will be .jpg files, which are smaller; the build will run approximately 2% faster.
        :param skip_frames: The build will return output data this many frames per `communicate()` call. This will greatly speed up the simulation. If you want to render every frame, set this to 0.
        """

        super().__init__(port=port, launch_build=launch_build, screen_width=screen_width, screen_height=screen_height,
                         debug=debug, auto_save_images=auto_save_images, images_directory=images_directory,
                         random_seed=random_seed, img_is_png=img_is_png, skip_frames=skip_frames)
        """:field
        The IDs of each target object in the scene.
        """
        self.target_objects: List[int] = list()
        """:field
        The IDs of each container in the scene.
        """
        self.containers: List[int] = list()
        """:field
        The total number of actions taken by the Magnebot.
        """
        self.action_cost: int = 0

        """:field
         The challenge is successful when the Magnebot moves all of the target objects to the the goal zone, which is defined by this position and `Transport.GOAL_ZONE_RADIUS`. This value is set in `init_scene()`.
        """
        self.goal_position: np.array = np.array([0, 0, 0])
        """:field
        The room that `self.goal_position` is in. [See here](https://github.com/alters-mit/magnebot/tree/main/doc/images/rooms) for images of the rooms. This value is set in `init_scene()`.
        """
        self.goal_room: int = 0

        """:field
        If True, the Magnebot successfully transported all of the objects to the goal zone. This is updated at the end of every action, including actions with 0 cost.
        """
        self.done: bool = False

        # Cached IK solution for resetting an arm holding a container.
        self._container_arm_reset_angles: Dict[Arm, np.array] = dict()

        # Get all possible target objects. Key = name. Value = scale.
        self._target_objects: Dict[str, float] = dict()
        with open(str(TARGET_OBJECTS_PATH.resolve())) as csvfile:
            reader = DictReader(csvfile)
            for row in reader:
                self._target_objects[row["name"]] = float(row["scale"])
        self._target_object_names = list(self._target_objects.keys())

    def init_scene(self, scene: str, layout: int, room: int = None, goal_room: int = None) -> ActionStatus:
        """
        This is the same function as `Magnebot.init_scene()` but it adds target objects and containers to the scene.

        When `init_scene()` is called, 8-12 target objects will be randomly placed on the floor of a randomly-selected room. Then, there is a 25% chance of adding one container per room.

        :param scene: The name of an interior floorplan scene. Each number (1, 2, etc.) has a different shape, different rooms, etc. Each letter (a, b, c) is a cosmetically distinct variant with the same floorplan.
        :param layout: The furniture layout of the floorplan. Each number (0, 1, 2) will populate the floorplan with different furniture in different positions.
        :param room: The index of the room that the Magnebot will spawn in the center of. If None, the room will be chosen randomly.
        :param goal_room: The goal room. If None, this is chosen randomly. See field descriptions of `goal_room` and `goal_position` in this document.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`

        :return: An `ActionStatus` (always success).
        """

        # Set the room of the goal.
        rooms = np.unique(np.load(str(ROOM_MAPS_DIRECTORY.joinpath(f"{scene[0]}.npy").resolve())))
        if goal_room is None:
            self.goal_room = int(self._rng.choice(rooms))
        else:
            assert goal_room in rooms, f"Not a valid room: {goal_room}"
            self.goal_room = goal_room

        # The goal position is the center of the room.
        self.goal_position = TDWUtils.vector3_to_array(loads(SPAWN_POSITIONS_PATH.read_text())[scene[0]][str(layout)]
                                                       [str(self.goal_room)])
        if self._debug:
            print(f"Goal position: {self.goal_position}")
        return super().init_scene(scene=scene, layout=layout, room=room)

    def pick_up(self, target: int, arm: Arm) -> ActionStatus:
        """
        Grasp an object and lift it up. This combines the actions `grasp()` and `reset_arm()`.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`
        - `cannot_reach`
        - `failed_to_grasp` (Either because the motion failed or because the magnet is already holding a different object.)
        - `failed_to_bend`

        :param target: The ID of the target object.
        :param arm: The arm of the magnet that will try to grasp the object.

        :return: An `ActionStatus` indicating if the magnet at the end of the `arm` is holding the `target` and if not, why.
        """

        if target in self.state.held[arm]:
            if self._debug:
                print(f"Already holding {target}")
            return ActionStatus.success
        if len(self.state.held[arm]) > 0:
            if self._debug:
                print(f"Already holding an object in {arm.name}")
            return ActionStatus.failed_to_grasp

        # This will increment `self.action_cost`.
        grasp_status = self.grasp(target=target, arm=arm)
        reset_status = self.reset_arm(arm=arm, reset_torso=True)
        self._end_action()
        if grasp_status != ActionStatus.success:
            return grasp_status
        return reset_status

    def reset_arm(self, arm: Arm, reset_torso: bool = True) -> ActionStatus:
        """
        This is the same as `Magnebot.reset_arm()` unless the arm is holding a container.

        If the arm is holding a container, it will try to align the bottom of the container with the floor. This will be somewhat slow the first time the Magnebot does this for this held container.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`
        - `failed_to_bend`

        :param arm: The arm that will be reset.
        :param reset_torso: If True, rotate and slide the torso to its neutral rotation and height.

        :return: An `ActionStatus` indicating if the arm reset and if not, why.
        """

        self.action_cost += 1

        # Use cached angles to reset an arm holding a container.
        if arm in self._container_arm_reset_angles:
            return super().reset_arm(arm=arm, reset_torso=reset_torso)

        status = super().reset_arm(arm=arm, reset_torso=reset_torso)
        for object_id in self.state.held[arm]:
            # If the arm is holding a container, orient the container to be level with the floor.
            if object_id in self.containers:
                rot = self.state.object_transforms[object_id].rotation
                # Source: https://answers.unity.com/questions/416169/finding-pitchrollyaw-from-quaternions.html
                x_rot = -np.rad2deg(np.arctan2(2 * rot[0] * rot[3] - 2 * rot[1] * rot[2],
                                               1 - 2 * rot[0] * rot[0] - 2 * rot[2] * rot[2]))
                if x_rot > 90:
                    x_rot = x_rot - 180
                elif x_rot > 0:
                    x_rot = -x_rot

                if self._debug:
                    print(f"Setting x rotation of container wrist to {x_rot}")

                # Get the commands to reset the arm.
                self._next_frame_commands.extend(self._get_reset_arm_commands(arm=arm, reset_torso=reset_torso))

                # Get the ID of the wrist.
                if arm == Arm.right:
                    wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_right]
                else:
                    wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_left]

                temp = list()
                for cmd in self._next_frame_commands:
                    # Adjust the wrist to level off the container.
                    if cmd["$type"] == "set_spherical_target" and cmd["joint_id"] == wrist_id:
                        cmd["target"] = {"x": x_rot, "y": 0, "z": 0}
                    temp.append(cmd)
                self._next_frame_commands = temp
                self._start_action()
                # Bend the arm.
                self._do_arm_motion()
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
        - `not_in` (If the target object didn't land in the container.)

        :return: An `ActionStatus` indicating if the target object is in the container and if not, why.
        """

        # Get the arm holding each object.
        container_arm, container_id = self._get_container_arm()
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
        self._next_frame_commands.append({"$type": "set_immovable",
                                          "immovable": True})
        # Move the other arm out of the way.
        if container_arm == Arm.right:
            elbow_id = self.magnebot_static.arm_joints[ArmJoint.elbow_left]
        else:
            elbow_id = self.magnebot_static.arm_joints[ArmJoint.elbow_right]
        self._next_frame_commands.append({"$type": "set_revolute_target",
                                          "joint_id": elbow_id,
                                          "target": 115})
        state = SceneState(resp=self.communicate([]))
        # Bring the container approximately to center.
        ct = {"x": 0.1 * (-1 if container_arm is Arm.right else 1), "y": 0.4, "z": 0.5}
        self._start_ik(target=ct,
                       arm=container_arm, absolute=False, allow_column=False, state=state,
                       fixed_torso_prismatic=Transport.__TORSO_PRISMATIC_CONTAINER, do_prismatic_first=False)
        self._do_arm_motion()
        state = SceneState(resp=self.communicate([]))
        # Move the target object to be over the container.
        target = np.copy(state.object_transforms[container_id].position)
        target[1] += 0.4
        self._start_ik(target=TDWUtils.array_to_vector3(target), arm=object_arm, allow_column=False, state=state,
                       absolute=True, fixed_torso_prismatic=Transport.__TORSO_PRISMATIC_CONTAINER, object_id=object_id)
        # Get the ID of the wrist.
        if object_arm == Arm.right:
            wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_right]
        else:
            wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_left]
        # Move the wrist down for a better angle.
        self._next_frame_commands.extend([{"$type": "set_spherical_target",
                                           "joint_id": wrist_id,
                                           "target": {"x": -45, "y": 0, "z": 0}}])
        self._do_arm_motion()
        # Drop the object.
        self._append_drop_commands(object_id=object_id, arm=object_arm)
        # Set the detection mode to discrete. This will make physics less buggy.
        self._next_frame_commands.append({"$type": "set_object_collision_detection_mode",
                                          "id": int(object_id),
                                          "mode": "discrete"})
        # Wait for the object to fall (hopefully into the container).
        self._wait_until_objects_stop(object_ids=[object_id], state=SceneState(self.communicate([])))

        # Reset the arms.
        self.reset_arm(arm=object_arm, reset_torso=False)
        self.reset_arm(arm=container_arm, reset_torso=True)
        self.action_cost -= 1

        in_container = self._get_objects_in_container(container_id=container_id)
        # If the object isn't in in the container, set the detection mode to the default.
        if object_id not in in_container:
            self._next_frame_commands.append({"$type": "set_object_collision_detection_mode",
                                              "id": int(object_id),
                                              "mode": "continuous_dynamic"})
        self._end_action()
        if object_id in in_container:
            return ActionStatus.success
        else:
            if self._debug:
                print(f"Object {object_id} isn't in container {container_id}")
            return ActionStatus.not_in

    def pour_out(self) -> ActionStatus:
        """
        Pour out all of the objects in a container held by one of the Magnebot's magnets.

        The Magnebot will extend the arm holding the container and then flip its elbow and wrist.

        The action ends when any objects that were in the container stop moving.

        Possible [return values](https://github.com/alters-mit/magnebot/blob/main/doc/action_status.md):

        - `success`
        - `not_holding` (If the Magnebot isn't holding a container.)
        - `still_in` (If there are objects still in the container.)

        :return: An `ActionStatus` indicating whether the container is now empty and if not, why.
        """
        container_arm, container_id = self._get_container_arm()
        if container_arm is None:
            if self._debug:
                print("Magnebot isn't holding a container.")
            return ActionStatus.not_holding
        # Get all of the objects currently in the container.
        in_container_0 = self._get_objects_in_container(container_id=container_id)
        self._start_action()

        # Get the joint IDs.
        if container_arm == Arm.right:
            shoulder_id = self.magnebot_static.arm_joints[ArmJoint.shoulder_right]
            elbow_id = self.magnebot_static.arm_joints[ArmJoint.elbow_right]
            wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_right]
        else:
            shoulder_id = self.magnebot_static.arm_joints[ArmJoint.shoulder_left]
            elbow_id = self.magnebot_static.arm_joints[ArmJoint.elbow_left]
            wrist_id = self.magnebot_static.arm_joints[ArmJoint.wrist_left]

        # Extend the arm.
        self._next_frame_commands.extend([{"$type": "set_spherical_target",
                                           "joint_id": shoulder_id,
                                           "target": {"x": -90, "y": 0, "z": 0}},
                                          {"$type": "set_revolute_target",
                                           "joint_id": elbow_id,
                                           "target": 0}])
        self._do_arm_motion()
        # Flip the wrist.
        self._next_frame_commands.extend([{"$type": "set_spherical_target",
                                           "joint_id": wrist_id,
                                           "target": {"x": 90, "y": 0, "z": 0}},
                                          {"$type": "set_revolute_target",
                                           "joint_id": elbow_id,
                                           "target": 35}])
        self._do_arm_motion()
        # Wait for the objects to fall out (by this point, they likely already have).
        self._wait_until_objects_stop(in_container_0, state=SceneState(self.communicate([])))
        self._next_frame_commands.extend(self._get_reset_arm_commands(arm=container_arm, reset_torso=False))
        self._do_arm_motion()
        in_container_1 = self._get_objects_in_container(container_id=container_id)

        # Reset the collision detection mode of objects that were poured out.
        for object_id in in_container_0:
            if object_id not in in_container_1:
                self._next_frame_commands.append({"$type": "set_object_collision_detection_mode",
                                                  "id": int(object_id),
                                                  "mode": "continuous_dynamic"})
        self._end_action()
        self.action_cost += 1
        if len(in_container_1) == 0:
            return ActionStatus.success
        else:
            return ActionStatus.still_in

    def get_target_objects_in_goal_zone(self) -> List[int]:
        """
        :return: A list of IDs of all of the target objects currently in the goal zone.
        """

        objects: List[int] = list()

        # Objects that are still being held by the Magnebot don't count.
        held: List[int] = list()
        for arm in self.state.held:
            for object_id in self.state.held[arm]:
                if object_id in self.target_objects:
                    held.append(object_id)
        # The object must be in the goal zone and on the floor.
        for object_id in self.target_objects:
            if self.state.object_transforms[object_id].position[1] <= 0.1 and \
                    np.linalg.norm(self.state.object_transforms[object_id].position - self.goal_position) <= \
                    Transport.GOAL_ZONE_RADIUS:
                objects.append(object_id)
        return objects

    def drop(self, target: int, arm: Arm, wait_for_objects: bool = True) -> ActionStatus:
        status = super().drop(target=target, arm=arm, wait_for_objects=wait_for_objects)
        if status == ActionStatus.success:
            # Remove the cached container arm angles.
            if arm in self._container_arm_reset_angles:
                del self._container_arm_reset_angles[arm]
        self.action_cost += 1
        return status

    def turn_by(self, angle: float, aligned_at: float = 3) -> ActionStatus:
        self.action_cost += 1
        return super().turn_by(angle=angle, aligned_at=aligned_at)

    def turn_to(self, target: Union[int, Dict[str, float]], aligned_at: float = 3) -> ActionStatus:
        self.action_cost += 1
        return super().turn_to(target=target, aligned_at=aligned_at)

    def move_by(self, distance: float, arrived_at: float = 0.3) -> ActionStatus:
        self.action_cost += 1
        return super().move_by(distance=distance, arrived_at=arrived_at)

    def reach_for(self, target: Dict[str, float], arm: Arm, absolute: bool = True, arrived_at: float = 0.125) -> ActionStatus:
        self.action_cost += 1
        return super().reach_for(target=target, arm=arm, absolute=absolute, arrived_at=arrived_at)

    def grasp(self, target: int, arm: Arm) -> ActionStatus:
        self.action_cost += 1
        return super().grasp(target=target, arm=arm)

    def reset_position(self) -> ActionStatus:
        self.action_cost += 1
        return super().reset_position()

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
        target_room_index = self._rng.choice(np.array(list(rooms.keys())))
        target_room_positions: np.array = np.array(rooms[target_room_index])
        used_target_object_positions: List[Tuple[int, int]] = list()

        # Add target objects to the room.
        for i in range(self._rng.randint(8, 12)):
            got_position = False
            ix, iy = -1, -1
            # Get a position where there isn't a target object.
            while not got_position:
                ix, iy = target_room_positions[self._rng.randint(0, len(target_room_positions))]
                got_position = True
                for utop in used_target_object_positions:
                    if utop[0] == ix and utop[1] == iy:
                        got_position = False
            used_target_object_positions.append((ix, iy))
            # Get the (x, z) coordinates for this position.
            x, z = self.get_occupancy_position(ix, iy)
            self._add_target_object(model_name=self._rng.choice(self._target_object_names),
                                    position={"x": x, "y": 0, "z": z})

        # Add containers throughout the scene.
        containers = CONTAINERS_PATH.read_text(encoding="utf-8").split("\n")
        for room_index in list(rooms.keys()):
            # Maybe don't add a container in this room.
            if self._rng.random() < 0.25:
                continue
            # Get a random position in the room.
            room_positions: np.array = np.array(rooms[room_index])
            got_position = False
            ix, iy = -1, -1
            # Get a position where there isn't a target object.
            while not got_position:
                ix, iy = room_positions[self._rng.randint(0, len(room_positions))]
                got_position = True
                for utop in used_target_object_positions:
                    if utop[0] == ix and utop[1] == iy:
                        got_position = False
            # Get the (x, z) coordinates for this position.
            x, z = self.get_occupancy_position(ix, iy)
            container_name = self._rng.choice(containers)
            self._add_container(model_name=container_name,
                                position={"x": x, "y": 0, "z": z},
                                rotation={"x": 0, "y": self._rng.uniform(-179, 179), "z": 0})
        return commands

    def _cache_static_data(self, resp: List[bytes]) -> None:
        # Reset the action counter and challenge status.
        self.action_cost = 0
        self.done = False
        super()._cache_static_data(resp=resp)

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
        self._object_init_commands[object_id].append({"$type": "add_trigger_collider",
                                                      "id": object_id,
                                                      "shape": "cube",
                                                      "enter": True,
                                                      "stay": True,
                                                      "exit": False})
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
                                     rotation={"x": 0, "y": self._rng.uniform(-179, 179), "z": 0},
                                     scale={"x": scale, "y": scale, "z": scale},
                                     audio=audio,
                                     model_name=model_name)
        self.target_objects.append(object_id)
        # Set a random visual material for each target object.
        visual_material = self._rng.choice(Transport.__TARGET_OBJECT_MATERIALS)
        substructure = Transport.__LIBRARIAN.get_record(model_name).substructure
        self._object_init_commands[object_id].extend(TDWUtils.set_visual_material(substructure=substructure,
                                                                                  material=visual_material,
                                                                                  object_id=object_id,
                                                                                  c=self,
                                                                                  quality="low"))
        return object_id

    def _get_reset_arm_commands(self, arm: Arm, reset_torso: bool) -> List[dict]:
        if arm in self._container_arm_reset_angles:
            self._append_ik_commands(angles=self._container_arm_reset_angles[arm], arm=arm)
            return list()
        else:
            return super()._get_reset_arm_commands(arm=arm, reset_torso=reset_torso)

    def _get_container_arm(self) -> Tuple[Arm, int]:
        """
        :return: Tuple: The arm holding a container, if any; the container ID.
        """

        container_arm: Optional[Arm] = None
        container_id = -1
        # Get an arm holding a container.
        for arm in self.state.held:
            for o_id in self.state.held[arm]:
                if container_arm is None and o_id in self.containers:
                    container_id = o_id
                    container_arm = arm
        return container_arm, container_id

    def _get_objects_in_container(self, container_id: int) -> List[int]:
        """
        :param container_id: The ID of the container.

        :return: A list of objects in the container.
        """

        if container_id not in self._trigger_events:
            return list()
        else:
            return self._trigger_events[container_id]

    def _is_challenge_done(self) -> bool:
        """
        :return: True if all of the objects have been transported to the goal zone.
        """

        return len(self.get_target_objects_in_goal_zone()) == len(self.target_objects)

    def _end_action(self) -> None:
        super()._end_action()
        self.done = self._is_challenge_done()

    def _get_bounds_sides(self, target: int) -> Tuple[List[np.array], List[bytes]]:
        sides, resp = super()._get_bounds_sides(target=target)
        # Set the y value to the highest point.
        max_y = -np.inf
        for s in sides:
            if s[1] > max_y:
                max_y = s[1]
        sides = [np.array((s[0], max_y, s[2])) for s in sides]
        # Don't try to pick up the top or bottom of a container.
        if target in self.containers:
            sides = sides[:-2]

        return sides, resp
