import numpy as np
from typing import Dict, List, Union
from pathlib import Path
from tdw.tdw_utils import TDWUtils
from tdw.output_data import OutputData, Images
from magnebot import ActionStatus, Arm
from transport_challenge import Transport


class Demo(Transport):
    # This is a pre-calculated path that the Magnebot will use to move between rooms.
    PATH: np.array = np.array([[6.396355, 0, -2.465405],
                               [5.41636, 0, -1.4854207],
                               [4.615, 0, -0.9954208],
                               [3.946356, 0, 0.66],
                               [0.4, 0, 0.66],
                               [0.02635, 0, -1.975]])

    def __init__(self, port: int = 1071, launch_build: bool = True, screen_width: int = 1024, screen_height: int = 1024,
                 images_directory: str = "images", image_pass_only: bool = False, random_seed: int = None):
        super().__init__(port=port, launch_build=launch_build, screen_width=screen_width, screen_height=screen_height,
                         auto_save_images=False, debug=False, images_directory=images_directory,
                         random_seed=random_seed)

        self._image_directories: Dict[str, Path] = dict()
        self._create_images_directory(avatar_id="a")
        self.image_pass_only = image_pass_only

        self._image_count = 0

    def add_camera(self, position: Dict[str, float], roll: float = 0, pitch: float = 0, yaw: float = 0,
                   look_at: bool = True, follow: bool = False, camera_id: str = "c") -> ActionStatus:
        """
        See: `Magnebot.add_camera()`.

        Adds some instructions to render images per-frame.
        """

        self._create_images_directory(avatar_id=camera_id)
        status = super().add_camera(position=position, roll=roll, pitch=pitch, yaw=yaw, look_at=look_at, follow=follow,
                                    camera_id=camera_id)
        # Always save images.
        if not self._debug:
            self._per_frame_commands.extend([{"$type": "enable_image_sensor",
                                              "enable": True},
                                             {"$type": "send_images"}])
        return status

    def communicate(self, commands: Union[dict, List[dict]]) -> List[bytes]:
        """
        See `Magnebot.communicate()`.

        Images are saved per-frame.
        """

        resp = super().communicate(commands=commands)
        if not self._debug:
            # Save all images.
            got_images = False
            for i in range(len(resp) - 1):
                r_id = OutputData.get_data_type_id(resp[i])
                if r_id == "imag":
                    got_images = True
                    images = Images(resp[i])
                    TDWUtils.save_images(filename=TDWUtils.zero_padding(self._image_count, 8),
                                         output_directory=self._image_directories[images.get_avatar_id()],
                                         images=images)
            if got_images:
                self._image_count += 1
        return resp

    def get_nearest_container(self) -> int:
        """
        :return: The ID of the container closest to the avatar.
        """

        min_id = -1
        min_distance = np.inf
        for container_id in self.containers:
            distance = np.linalg.norm(self.state.magnebot_transform.position -
                                      self.state.object_transforms[container_id].position)
            if distance < min_distance:
                min_distance = distance
                min_id = container_id
        return min_id

    def transport(self, object_ids: List[int]) -> None:
        """
        Transport some objects to the other room.

        :param object_ids: The list of target object IDs.
        """
        # Pick up some objects.
        for object_id in object_ids:
            self.move_to(target=object_id, arrived_at=1)
            self.pick_up(target=object_id, arm=Arm.right)

            # Move back just a bit so the arms have enough free space.
            self.move_by(distance=-0.3, arrived_at=0.1)

            self.put_in()

        # Follow the path to the other room.
        path = Demo.PATH[1:]
        for waypoint in path:
            self.move_to(target=TDWUtils.array_to_vector3(waypoint))
        self.pour_out()

    def go_to_start(self) -> None:
        """
        Navigate back to the start position.
        """

        path = np.flip(Demo.PATH[:-1], axis=0)
        for waypoint in path:
            self.move_to(target=TDWUtils.array_to_vector3(waypoint))

    def _get_scene_init_commands(self, magnebot_position: Dict[str, float] = None) -> List[dict]:
        commands = super()._get_scene_init_commands(magnebot_position=magnebot_position)
        # Hide the roof.
        commands.append({"$type": "set_floorplan_roof",
                         "show": False})
        if self.image_pass_only:
            commands.append({"$type": "set_pass_masks",
                             "pass_masks": ["_img"]})
        return commands

    def _create_images_directory(self, avatar_id: str) -> None:
        """
        :param avatar_id: The ID of the avatar.

        :return: An images directory for the avatar.
        """

        # Build the images directory.
        a_dir = self.images_directory.joinpath(avatar_id)
        if not a_dir.exists():
            a_dir.mkdir(parents=True)
        self._image_directories[avatar_id] = a_dir


if __name__ == "__main__":
    # This random seed will spawn the Magnebot, the target objects, and a container all in the same room.
    m = Demo(launch_build=False, random_seed=9, images_directory="D:/transport_challenge_demo", image_pass_only=True)
    m.init_scene(scene="2a", layout=1, room=4)
    # Add an overhead camera.
    m.add_camera(position={"x": -3.6, "y": 8, "z": -0.67}, look_at=True, follow=True)

    # Pick up the nearest container.
    nearest_container = m.get_nearest_container()
    m.move_to(target=nearest_container, arrived_at=1)
    m.pick_up(nearest_container, arm=Arm.left)

    # Pick up some objects and put them in another room.
    m.transport(m.target_objects[:4])

    # Go back to the starting room.
    m.go_to_start()

    # Pick up some more objects and put them in the other room.
    m.transport(m.target_objects[5:])

    m.move_by(-1)
    m.end()
