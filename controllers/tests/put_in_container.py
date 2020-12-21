from tdw.tdw_utils import TDWUtils
from magnebot import Arm, ActionStatus
from transport_challenge import Transport


class PutInContainer(Transport):
    def init_scene(self, scene: str = None, layout: int = None, room: int = None) -> ActionStatus:
        commands = [{"$type": "load_scene",
                     "scene_name": "ProcGenScene"},
                    TDWUtils.create_empty_room(12, 12)]
        self._add_container(model_name="basket_18inx18inx12iin",
                            position={"x": 0.354, "y": 0, "z": 0.549},
                            rotation={"x": 0, "y": -70, "z": 0})
        commands.extend(self._get_scene_init_commands(magnebot_position={"x": 0, "y": 0, "z": 0}))
        resp = self.communicate(commands)
        self._cache_static_data(resp=resp)
        # Wait for the Magnebot to reset to its neutral position.
        status = self._do_arm_motion()
        self._end_action()
        return status


if __name__ == "__main__":
    m = PutInContainer(launch_build=False, debug=True)
    m.init_scene()
    status = m.pick_up(target=m.containers[0], arm=Arm.right)
    assert status == ActionStatus.success, status
