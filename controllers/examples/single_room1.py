from pathlib import Path
from tdw.tdw_utils import TDWUtils
from magnebot import Magnebot, ActionStatus, Arm
from transport_challenge import Transport

import time
import os

import numpy as np

import requests
import json
def create_tdw(port):
    url = "http://localhost:5000/get_tdw"
    data = {
        'ip_address': "localhost",
        'port': port
    }
    response = requests.post(url, json=json.dumps(data))
    print(response.status_code, response.reason)
    docker_id = response.json()['docker_id']
    return docker_id


def kill_tdw(docker_id):
    url = "http://localhost:5000/kill_tdw"
    data = {
        "container_id": docker_id
    }
    response = requests.post(url, json=json.dumps(data))
    print(response.status_code, response.reason)

class SingleRoom(Transport):
    """
    This is an example of how to pick up target objects, put them in a container, and transport them to a goal zone.
    This example does _not_ describe how to implement navigation in the Transport Challenge.
    """

    def get_container(self) -> int:
        """
        This is a VERY naive approach to finding a nearby container.
        The Magnebot will rotate around and record the IDs of each container it sees.

        :return: The ID of a container within visual range of the Magnebot.
        """

        d_turn: int = 90
        turn: int = 0
        while turn < 270:
            # Look around you.
            d_cam_theta = 45
            # First pitch the camera and then yaw the camera.
            for axis in ["pitch", "yaw"]:
                if axis == "pitch":
                    s = self.rotate_camera(pitch=-Magnebot.CAMERA_RPY_CONSTRAINTS[1])
                else:
                    s = self.rotate_camera(yaw=-Magnebot.CAMERA_RPY_CONSTRAINTS[2])
                # Rotate the camera until we get an ActionStatus that the angle has been clamped to the RPY constraints.
                # That means that we've rotated the camera as far as it will go.
                while s == ActionStatus.success:
                    visible_objects = self.get_visible_objects()
                    for object_id in visible_objects:
                        # We found a container.
                        if object_id in self.containers:
                            return object_id
                    # Keep rotating the camera.
                    if axis == "pitch":
                        s = self.rotate_camera(pitch=d_cam_theta)
                    else:
                        s = self.rotate_camera(yaw=d_cam_theta)
                self.reset_camera()
            # Turn the Magnebot.
            self.turn_by(d_turn)
            turn += d_turn
        raise Exception("No container found!")

    def pick_up_container(self) -> None:
        """
        Go to a container in the room and pick it up.
        """

        # Find a container in the room.
        container_id = self.get_container()
        # Go to the container and pick it up.
        self.move_to(target=container_id)
        # Try to pick up the container.
        got_container = False
        while not got_container:
            start_time = time.time()
            self.pick_up(target=container_id, arm=Arm.right)
            print('pick up:', time.time() - start_time)
            if container_id in self.state.held[Arm.right]:
                print("Picked up a container.")
                got_container = True
            else:
                self.reset_arm(arm=Arm.right)
                # Back up and try again.
                # This isn't a very robust approach.
                # In an actual use-case, the actions will vary depending on the position of the object,
                # the position of the Magnebot, etc.
                self.move_by(-0.5)
                self.turn_by(15)
                self.move_to(target=container_id)
                print("Failed to pick up a container. Trying again...")

    def put_object_in_container(self, object_id: int) -> bool:
        """
        Go to a target object, pick it up, and put it in the container.

        This is a VERY naive function because:

        - It assumes that the Magnebot is holding a container.
        - It doesn't handle navigation; it assumes that there is a clear path between the Magnebot and the object.
        - If the Magnebot fails to put the object in the container, it won't try again.

        :param object_id: The ID of the target object.

        :return: True if the Magnebot put the object in the container.
        """

        # Try to go to the object and pick it up.
        # This is a VERY naive navigation solution. It doesn't check for obstructions.
        # And, if the Magnebot fails to pick up the object, it will simply skip it.
        status = self.move_to(target=object_id)
        if status != ActionStatus.success:
            print(f"Failed to move to target object {object_id}: {status}")
            return False
        print(f"Moved to target object {object_id}")
        start_time = time.time()
        status = self.pick_up(target=object_id, arm=Arm.left)
        print('pick up:', time.time() - start_time)
        if object_id not in self.state.held[Arm.left]:
            print(f"Failed to pick up target object {object_id}: {status}")
            return False
        print(f"Picked up target object {object_id}")
        status = self.put_in()
        if status != ActionStatus.success:
            print(f"Failed to put target object {object_id} in the container: {status}")
            return False
        print(f"Put target object {object_id} in the container.")
        return True


if __name__ == "__main__":
    try:
        dir = 'pc5'
        os.mkdir(dir)
    except:
        print('')
    port = 10071
    docker_id = create_tdw(port)
    try:
        # Instantiate the controller.
        # For this example, we set the random seed so that we know where the target objects and containers will be.
        m = SingleRoom(port = port, launch_build=False, random_seed=12, auto_save_images=False)
        print(f"Images will be saved to: {Path(m.images_directory).resolve()}")

        # We know that there are target objects and a container in this room because of the random seed in the constructor.
        start_time = time.time()
        m.init_scene(scene="5a", layout=2, room=2, goal_room=2)
        x, y, z = m.state.magnebot_transform.position
        print('scene:', time.time() - start_time)
        
        
        '''for i in range(2):            
            depth = m.state.get_depth_values()
            TDWUtils.get_point_cloud(depth = depth, \
                        camera_matrix = m.state.camera_matrix, \
                        filename = f'{dir}/{i}.txt')
            m.state.save_images(dir)
            np.save(f'{dir}/camera_matrix{i}.npy', m.state.camera_matrix)           
            np.save(f'{dir}/depth{i}.npy', m.state.get_depth_values())
            m.turn_by(90)
        #start_time = time.time()
        print('done')'''
        m.pick_up_container()
        #print('pick up:', time.time() - start_time)
        m.state.save_images(dir)
        depth = m.state.get_depth_values()
        TDWUtils.get_point_cloud(depth = depth, \
                        camera_matrix = m.state.camera_matrix, \
                        filename = f'{dir}/0.txt')
        depth = depth.reshape(-1)
        index = depth[np.where(depth < 99)]
        print(np.min(depth))
        depth = np.sort(depth)
        np.save('1.npy', depth)
        num_in_container = 0
        # Transport objects to the goal position.
        for target_object in m.target_objects[:3]:
            in_container = m.put_object_in_container(object_id=target_object)
            if in_container:
                # Record the object as being in the container.
                num_in_container += 1
        # If the container is mostly full, bring it to the goal position and pour it out.
        if num_in_container == 3:
            print("Bringing target objects to the goal zone.")
            m.move_to(target=TDWUtils.array_to_vector3(m.goal_position))
            m.pour_out()
            print("Poured out objects.")
            num_in_container = 0

        # Not all of the objects that were in the container are in the goal zone.
        # They might have bounced or rolled away.
        print(f"Objects in the goal zone: {m.get_target_objects_in_goal_zone()}")
        print(f"Action cost: {m.action_cost}")
        m.end()
    finally:
        kill_tdw(docker_id)
