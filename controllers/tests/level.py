import numpy as np
from tdw.tdw_utils import TDWUtils
from magnebot import Arm, ActionStatus
from transport_challenge import Transport


class Level(Transport):
    """
    Test whether the Magnebot can level-off the orientation of the container.
    """

    def init_scene(self, scene: str = None, layout: int = None, room: int = None) -> ActionStatus:
        commands = [{"$type": "load_scene",
                     "scene_name": "ProcGenScene"},
                    TDWUtils.create_empty_room(12, 12)]
        # Add containers.
        num_containers = 4
        d_theta = 360 / num_containers
        theta = d_theta / 2
        pos = np.array([2, 0, 0])
        for j in range(num_containers):
            object_position = TDWUtils.rotate_position_around(origin=np.array([0, 0, 0]), position=pos, angle=theta)
            self._add_container("basket_18inx18inx12iin",
                                position=TDWUtils.array_to_vector3(object_position),
                                rotation={"x": 0, "y": float(self._rng.uniform(-179, 179)), "z": 0})
            theta += d_theta
        commands.extend(self._get_scene_init_commands())
        resp = self.communicate(commands)
        self._cache_static_data(resp=resp)
        # Wait for the Magnebot to reset to its neutral position.
        status = self._do_arm_motion()
        self._end_action()
        return status

    def run(self) -> None:
        self.init_scene()

        for container_id in self.containers:
            self.move_to(target=container_id, arrived_at=1)
            self.pick_up(container_id, arm=Arm.left)
            self.drop(target=container_id, arm=Arm.left)
        self.end()


if __name__ == "__main__":
    m = Level(launch_build=False, random_seed=0)
    m.run()
