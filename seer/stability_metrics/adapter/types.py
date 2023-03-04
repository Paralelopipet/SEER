from typing import Any

from pydantic import BaseModel

from seer.stability_metrics.adapter.stability_metrics import StabilityMetrics
from seer.utility.types import Point3D, Vector3D


class RobotConfig(BaseModel):
    metric: StabilityMetrics = StabilityMetrics.force_angle
    urdf_path: str
    cube_link_name: str

class RobotState(BaseModel):
    centre_of_mass: Point3D
    net_force: Vector3D
    net_moment: Vector3D
    
    class Config:
        arbitrary_types_allowed = True
