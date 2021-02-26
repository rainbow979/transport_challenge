from csv import DictReader
from json import loads
from pathlib import Path
import random
import numpy as np
from pkg_resources import resource_filename
from typing import Dict, List, Union, Optional, Tuple
from tdw.floorplan_controller import FloorplanController
from tdw.tdw_utils import TDWUtils, QuaternionUtils
from tdw.output_data import Bounds, Rigidbodies, SegmentationColors, Raycast, CompositeObjects, Overlap, Transforms,\
    Version
from tdw.py_impact import AudioMaterial, PyImpact, ObjectInfo
from tdw.object_init_data import AudioInitData
from tdw.release.pypi import PyPi

from magnebot.constants import MAGNEBOT_RADIUS, OCCUPANCY_CELL_SIZE
from magnebot.paths import ROOM_MAPS_DIRECTORY, OCCUPANCY_MAPS_DIRECTORY, SCENE_BOUNDS_PATH, SPAWN_POSITIONS_PATH
from transport_challenge.paths import TARGET_OBJECT_MATERIALS_PATH, TARGET_OBJECTS_PATH, CONTAINERS_PATH
'''from sticky_mitten_avatar.avatars import Arm, Baby
from sticky_mitten_avatar.avatars.avatar import Avatar, BodyPartStatic
from sticky_mitten_avatar.util import get_data, OCCUPANCY_CELL_SIZE, \
    TARGET_OBJECT_MASS, CONTAINER_MASS, CONTAINER_SCALE
from sticky_mitten_avatar.paths import SPAWN_POSITIONS_PATH, OCCUPANCY_MAP_DIRECTORY, SCENE_BOUNDS_PATH, \
    ROOM_MAP_DIRECTORY, Y_MAP_DIRECTORY, TARGET_OBJECTS_PATH, COMPOSITE_OBJECT_AUDIO_PATH, SURFACE_MAP_DIRECTORY, \
    TARGET_OBJECT_MATERIALS_PATH, OBJECT_SPAWN_MAP_DIRECTORY
from sticky_mitten_avatar.static_object_info import StaticObjectInfo
from sticky_mitten_avatar.frame_data import FrameData
from sticky_mitten_avatar.task_status import TaskStatus'''

import pickle

def get_occupancy_position(i, j, _scene_bounds):
    x = _scene_bounds["x_min"] + (i * OCCUPANCY_CELL_SIZE)
    z = _scene_bounds["z_min"] + (j * OCCUPANCY_CELL_SIZE)
    return x, z

    
def get_target_object_name():
    _target_objects: Dict[str, float] = dict()
    with open(str(TARGET_OBJECTS_PATH.resolve())) as csvfile:
        reader = DictReader(csvfile)
        for row in reader:
            _target_objects[row["name"]] = float(row["scale"])
    _target_object_names = list(_target_objects.keys())
    return _target_object_names
    
def generate_data(scene, layout, room):
    data = {}
    data['scene'] = {'scene': scene, 'layout': layout, 'room': room}
    

    _scene_bounds = loads(SCENE_BOUNDS_PATH.read_text())[scene[0]]
    room_map = np.load(str(ROOM_MAPS_DIRECTORY.joinpath(f"{scene[0]}.npy").resolve()))
    map_filename = f"{scene[0]}_{layout}.npy"
    occupancy_map = np.load(str(OCCUPANCY_MAPS_DIRECTORY.joinpath(f"{scene[0]}_{layout}.npy").resolve()))
    map = np.zeros_like(occupancy_map)
    map[occupancy_map == 1] = 1
    from scipy.signal import convolve2d
    conv = np.ones((3, 3))
    map = convolve2d(map, conv, mode='same', boundary='fill')
    
    
    all_placeable_positions = []
    rooms: Dict[int, List[Tuple[int, int]]] = dict()
    
    for ix, iy in np.ndindex(room_map.shape):
        room_index = room_map[ix][iy]
        if room_index not in rooms:
            rooms[room_index] = list()
        if map[ix][iy] == 0:
            rooms[room_index].append((ix, iy))
    
    data['target_object'] = []
    
    _target_object_names = get_target_object_name()
    
    target_room_index = random.choice(np.array(list(rooms.keys())))
    target_room_positions: np.array = np.array(rooms[target_room_index])
        
    used_target_object_positions: List[Tuple[int, int]] = list()
    for i in range(random.randint(8, 12)):
        got_position = False
        ix, iy = -1, -1
        # Get a position where there isn't a target object.
        while not got_position:
            target_room_index = random.choice(np.array(list(rooms.keys())))
            target_room_positions = np.array(rooms[target_room_index])
            ix, iy = target_room_positions[random.randint(0, len(target_room_positions) - 1)]
            got_position = True
            for utop in used_target_object_positions:
                if utop[0] == ix and utop[1] == iy:
                    got_position = False
        used_target_object_positions.append((ix, iy))
        x, z = get_occupancy_position(ix, iy, _scene_bounds)
        data['target_object'].append({
            'model_name': random.choice(_target_object_names),
            'position': {"x": x, "y": 0, "z": z}
                })
    data['container'] = []
    containers = CONTAINERS_PATH.read_text(encoding="utf-8").split("\n")
    for room_index in list(rooms.keys()):
        # Maybe don't add a container in this room.
        if random.random() < 0.3:
            continue
        if len(data['container']) == 4:
            break
        # Get a random position in the room.
        room_positions: np.array = np.array(rooms[room_index])
        got_position = False
        ix, iy = -1, -1
        # Get a position where there isn't a target object.
        while not got_position:
            ix, iy = room_positions[random.randint(0, len(room_positions) - 1)]
            got_position = True
            for utop in used_target_object_positions:
                if utop[0] == ix and utop[1] == iy:
                    got_position = False
        # Get the (x, z) coordinates for this position.
        x, z = get_occupancy_position(ix, iy, _scene_bounds)
        #container_name = random.choice(containers)
        container_name = "basket_18inx18inx12iin"
        data['container'].append({
            'model_name': container_name,
            'position': {"x": x, "y": 0, "z": z},
            'rotation': {"x": 0, "y": random.uniform(-179, 179), "z": 0}
        })
    '''
    2a 0 bed
    2a 1 bed
    2a 2 bed
    5a 0 bed
    5a 1 bed
    bed
    '''
    '''_goal_positions = loads(SURFACE_MAP_DIRECTORY.joinpath(f"{scene[0]}_{layout}.json").
                                   read_text(encoding="utf-8"))
    goal_positions = dict()
    goal_objects = []
    for k, v in _goal_positions.items():        
        goal_positions[int(k)] = _goal_positions[k]
        for k1 in v:
            goal_objects.append(k1)
    goal_objects = list(set(goal_objects))
    goal_object = random.choice(goal_objects)
    data['goal_object'] = goal_object'''
    data['goal_object'] = 'bed'
    rooms = loads(SPAWN_POSITIONS_PATH.read_text())[scene[0]][str(layout)]
    if room == -1:
        room = random.randint(0, len(rooms) - 1)
    assert 0 <= room < len(rooms), f"Invalid room: {room}"
    data['scene']['room'] = room
    avatar_position = rooms[str(room)]
    
    data['avatar_position'] = avatar_position
    return data

if __name__ == '__main__':
    import time
    start = time.time()
    scene = '1a'
    layout = 0
    room = -1
    dataset = []
    train = True
    if train:
        scenes = ['2a', '2b', '5a', '5b', '1a']
        layouts = [0, 1]
        l = 100
        path = 'train_dataset.pkl'
    else:   
        scenes = ['5a', '5b', '2a', '2b']
        layouts = [2]        
        l = 10
        path = 'test_dataset.pkl'
    for scene in scenes:
        for layout in layouts:
            for i in range(l):                
                dataset.append(generate_data(scene, layout, room))
    print(len(dataset))
    if train == True:
        random.shuffle(dataset)
    with open(path, 'wb') as f:
        pickle.dump(dataset, f)
    print(time.time() - start)