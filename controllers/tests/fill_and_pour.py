import numpy as np
from tdw.tdw_utils import TDWUtils
from magnebot import Arm, ActionStatus
from transport_challenge import Transport


class FillAndPour(Transport):
    """
    Test whether the Magnebot can put objects in a container, move them around the scene, pour out the container.
    """

    def init_scene(self, scene: str = None, layout: int = None, room: int = None) -> ActionStatus:
        origin = np.array([0, 0, 0])
        commands = [{"$type": "load_scene",
                     "scene_name": "ProcGenScene"},
                    TDWUtils.create_empty_room(12, 12)]
        self._add_container(model_name="basket_18inx18inx12iin",
                            position={"x": 0.354, "y": 0, "z": 0.549},
                            rotation={"x": 0, "y": -70, "z": 0})
        # Add target objects in a circle.
        num_objects = 10
        d_theta = 360 / num_objects
        theta = d_theta / 2
        pos = np.array([2, 0, 0])
        for j in range(num_objects):
            object_position = TDWUtils.rotate_position_around(origin=origin, position=pos, angle=theta)
            self._add_target_object("jug05",
                                    position=TDWUtils.array_to_vector3(object_position))
            theta += d_theta
        commands.extend(self._get_scene_init_commands())
        resp = self.communicate(commands)
        self._cache_static_data(resp=resp)
        # Wait for the Magnebot to reset to its neutral position.
        status = self._do_arm_motion()
        self._end_action()
        return status


if __name__ == "__main__":
    m = FillAndPour(launch_build=False, random_seed=0)
    m.init_scene()
    # Pick up the container.
    m.pick_up(target=m.containers[0], arm=Arm.right)

    i = 0
    for object_id in m.target_objects:
        # Every n objects, go to the center of the room and pour out the container.
        if i > 0 and i % 4 == 0:
            m.move_to({"x": 0, "y": 0, "z": 0})
            s = m.pour_out()
            assert s == ActionStatus.success, s
        m.move_to(target=object_id, arrived_at=1)
        # Pick up the target object.
        m.pick_up(target=object_id, arm=Arm.left)
        # Put the object in the container.
        s = m.put_in()
        assert s == ActionStatus.success, s
        i += 1
    m.move_to({"x": 0, "y": 0, "z": 0})
    s = m.pour_out()
    assert s == ActionStatus.success, s
    m.move_by(1)
    m.end()
