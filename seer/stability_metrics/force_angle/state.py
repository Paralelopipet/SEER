from typing import List

from pydantic import BaseModel
from seer.utility.types import Point3D, Vector3D


class ForceAngleState(BaseModel):
    centre_of_mass: Point3D
    reaction_forces: List[Vector3D]
    reaction_moments: List[Vector3D]
