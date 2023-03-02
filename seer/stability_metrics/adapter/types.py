from typing import Any
from pydantic import BaseModel

from pybullet_multigoal_gym.utils.cube_path import \
    CUBE_PATH

class RobotConfig(BaseModel):
    urdf_path: str = CUBE_PATH
    cube_link_name: str = "cube"

# TODO plug in pybullet types here
RobotState = Any
