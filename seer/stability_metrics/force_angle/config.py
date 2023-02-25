from typing import List
from pydantic import BaseModel

from seer.utility.types import Point3D


class ForceAngleConfig(BaseModel):
    clockwise_ground_contact_points: List[Point3D]
