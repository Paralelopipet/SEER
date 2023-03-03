from typing import Any

from pydantic import BaseModel

from seer.stability_metrics.adapter.stability_metrics import StabilityMetrics


class RobotConfig(BaseModel):
    metric: StabilityMetrics = StabilityMetrics.force_angle
    urdf_path: str
    cube_link_name: str

# TODO plug in pybullet types here
RobotState = Any
