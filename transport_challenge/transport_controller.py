import random
from json import loads
from csv import DictReader
from typing import List, Dict, Tuple
import numpy as np
from tdw.py_impact import ObjectInfo, AudioMaterial
from tdw.librarian import ModelLibrarian
from tdw.tdw_utils import TDWUtils
from tdw.output_data import Bounds
from magnebot import Magnebot, Arm, ActionStatus
from magnebot.scene_state import SceneState
from magnebot.paths import ROOM_MAPS_DIRECTORY, OCCUPANCY_MAPS_DIRECTORY, SCENE_BOUNDS_PATH
from transport_challenge.paths import TARGET_OBJECT_MATERIALS_PATH, TARGET_OBJECTS_PATH, CONTAINERS_PATH
from magnebot.util import get_data


class Transport(Magnebot):
    TARGET_OBJECT_MASS = 0.25

    # The scale factor of each container relative to its original size.
    __CONTAINER_SCALE = {"x": 0.6, "y": 0.4, "z": 0.6}

    # The mass of a container.
    __CONTAINER_MASS = 1

    # The model librarian.
    __LIBRARIAN = ModelLibrarian()

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

    def pick_up(self, target: int, arm: Arm) -> ActionStatus:
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
        status = super().reset_arm(arm=arm, reset_torso=reset_torso)
        for object_id in self.state.held[arm]:
            if object_id in self.containers:
                # Orient
                self._start_ik_orientation(orientation=[0, 0, -1], arm=arm, orientation_mode="Y", object_id=object_id,
                                           torso_prismatic=1.2)
                status = self._do_arm_motion()
                self._end_action()
                return status
        return status

    def get_scene_init_commands(self, scene: str, layout: int, audio: bool) -> List[dict]:
        # Clear the list of target objects and containers.
        self.target_objects.clear()
        self.containers.clear()
        commands = super().get_scene_init_commands(scene=scene, layout=layout, audio=audio)
        # Get all possible target objects. Key = name. Value = scale.
        target_objects: Dict[str, float] = dict()
        with open(str(TARGET_OBJECTS_PATH.resolve())) as csvfile:
            reader = DictReader(csvfile)
            for row in reader:
                target_objects[row["name"]] = float(row["scale"])
        target_object_names = list(target_objects.keys())

        # Load the map of the rooms in the scene, the occupancy map, and the scene bounds.
        room_map = np.load(str(ROOM_MAPS_DIRECTORY.joinpath(f"{scene[0]}.npy").resolve()))
        self.occupancy_map = np.load(str(OCCUPANCY_MAPS_DIRECTORY.joinpath(f"{scene[0]}_{layout}.npy").resolve()))
        self._scene_bounds = loads(SCENE_BOUNDS_PATH.read_text())[scene[0]]

        # Load a list of visual materials for target objects.
        target_object_materials = TARGET_OBJECT_MATERIALS_PATH.read_text(encoding="utf-8").split("\n")

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
            target_object_name = random.choice(target_object_names)
            # Set custom object info for the target objects.
            audio = ObjectInfo(name=target_object_name, mass=Transport.TARGET_OBJECT_MASS,
                               material=AudioMaterial.ceramic, resonance=0.6, amp=0.01, library="models_core.json",
                               bounciness=0.5)
            scale = target_objects[target_object_name]
            # Add the object.
            object_id = self._add_object(position={"x": x, "y": 0, "z": z},
                                         rotation={"x": 0, "y": random.uniform(-179, 179), "z": z},
                                         scale={"x": scale, "y": scale, "z": scale},
                                         audio=audio,
                                         model_name=target_object_name)
            self.target_objects.append(object_id)
            # Set a random visual material for each target object.
            visual_material = random.choice(target_object_materials)
            substructure = Transport.__LIBRARIAN.get_record(target_object_name).substructure
            self._object_init_commands[object_id].extend(TDWUtils.set_visual_material(substructure=substructure,
                                                                                      material=visual_material,
                                                                                      object_id=object_id,
                                                                                      c=self))
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
