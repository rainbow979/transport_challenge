from pathlib import Path
from pkg_resources import resource_filename

"""
Paths to data files in this Python module.
"""

# The path to the data files.
DATA_DIRECTORY = Path(resource_filename(__name__, "data"))
# The path to object data.
OBJECT_DATA_DIRECTORY = DATA_DIRECTORY.joinpath("objects")
# The path to the list of target objects.
TARGET_OBJECTS_PATH = OBJECT_DATA_DIRECTORY.joinpath("target_objects.csv")
# The path to the list of target object materials.
TARGET_OBJECT_MATERIALS_PATH = OBJECT_DATA_DIRECTORY.joinpath("target_object_materials.txt")
# The path to the list of containers.
CONTAINERS_PATH = OBJECT_DATA_DIRECTORY.joinpath("containers.txt")
