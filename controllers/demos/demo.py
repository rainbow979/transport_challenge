from os import chdir
from typing import Dict, List, Union
from pathlib import Path
from subprocess import call
import numpy as np
from tdw.tdw_utils import TDWUtils
from tdw.output_data import OutputData, Images
from magnebot import ActionStatus, Arm
from transport_challenge import Transport


class Demo(Transport):
    """
    A demo of a Magnebot using a container to transport target objects to a new room.

    **This is NOT a use-case example.** It should only be used to generate a demo video of the Transport Challenge.

    Key differences:

    - Navigation is pre-calculated (see `PATH`).
    - Only the `img` pass is captured (not `id` or `depth`).
    - The screen is large, there is an overhead camera, and images are saved per-frame instead of per-action. This means that this controller will run *much* slower than a use-case controller.
    - There are some low-level commands to optimize the demo such as teleporting a container and hiding the roof.
    """

    # This is a pre-calculated path that the Magnebot will use to move between rooms.
    PATH: np.array = np.array([[6.396355, 0, -2.465405],
                               [5.41636, 0, -1.4854207],
                               [4.615, 0, -0.9954208],
                               [3.946356, 0, 0.66],
                               [0.4, 0, 0.66],
                               [0.02635, 0, -1.975]])

    def __init__(self, port: int = 1071, screen_width: int = 1024, screen_height: int = 1024,
                 images_directory: str = "images", image_pass_only: bool = False, overhead_camera_only: bool = False):
        super().__init__(port=port, launch_build=False, screen_width=screen_width, screen_height=screen_height,
                         auto_save_images=False, debug=False, images_directory=images_directory, random_seed=9,
                         img_is_png=False, skip_frames=0)

        self.image_directories: Dict[str, Path] = dict()
        self.image_pass_only = image_pass_only
        self.overhead_camera_only = overhead_camera_only
        if not overhead_camera_only:
            self._create_images_directory(avatar_id="a")

        self._image_count = 0
        self._to_transport: List[int] = list()

    def init_scene(self, scene: str, layout: int, room: int = None, goal_room: int = None) -> ActionStatus:
        status = super().init_scene(scene=scene, layout=layout, room=room)

        self._to_transport = self.target_objects[:]
        return status

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
            if self.overhead_camera_only:
                self._per_frame_commands.extend([{"$type": "enable_image_sensor",
                                                  "enable": True},
                                                 {"$type": "send_images",
                                                  "ids": ["c"]}])
            else:
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
                    avatar_id = images.get_avatar_id()
                    if avatar_id in self.image_directories:
                        TDWUtils.save_images(filename=TDWUtils.zero_padding(self._image_count, 8),
                                             output_directory=self.image_directories[avatar_id],
                                             images=images)
            if got_images:
                self._image_count += 1
        return resp

    def transport(self) -> None:
        """
        Transport some objects to the other room.
        """

        for i in range(4):
            # Get the closest object that still needs to be transported.
            self._to_transport = list(sorted(self._to_transport, key=lambda x: np.linalg.norm(
                self.state.object_transforms[x].position - self.state.magnebot_transform.position)))
            object_id = self._to_transport[0]
            # Go to the object and pick it up.
            self.move_to(target=object_id)
            self.pick_up(target=object_id, arm=Arm.right)
            # Put the object in the container.
            self.put_in()
            # Record this object as done.
            self._to_transport = self._to_transport[1:]

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

    def teleport_container(self) -> None:
        """
        Teleport a container into the same room as the Magnebot.
        """

        self._next_frame_commands.append({"$type": "teleport_object",
                                          "id": self.containers[0],
                                          "position": {"x": 7.44, "y": 0, "z": -2.62}})
        self._end_action()

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
        self.image_directories[avatar_id] = a_dir


if __name__ == "__main__":
    m = Demo(images_directory="D:/transport_challenge_demo", image_pass_only=True, overhead_camera_only=True)
    m.init_scene(scene="2a", layout=1, room=4)
    # Add an overhead camera.
    m.add_camera(position={"x": -3.6, "y": 8, "z": -0.67}, look_at=True, follow=True)

    m.teleport_container()

    # Pick up the  container.
    m.move_to(target=m.containers[0])
    m.pick_up(target=m.containers[0], arm=Arm.left)

    # Pick up some objects and put them in another room.
    m.transport()
    # Go back to the starting room.
    m.go_to_start()
    m.end()

    # Create a video.
    if m.overhead_camera_only:
        chdir(str(m.image_directories["c"]))
        call(["ffmpeg.exe",
              "-r", "90",
              "-i", "img_%08d.jpg",
              "-vcodec", "libx264",
              "-pix_fmt", "yuv420p",
              str(m.images_directory.joinpath("transport_challenge_demo.mp4").resolve())])
