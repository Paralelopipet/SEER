from typing import Any, List

from pydantic import BaseModel

from seer.utility.types import Point3D


class ForceAngleConfig(BaseModel):
    clockwise_ground_contact_points: List[Point3D]

    class Config:
        arbitrary_types_allowed = True
    
    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, ForceAngleConfig):
            return False
        return self._plain_python_points() == other._plain_python_points()
    
    def _plain_python_points(self) -> List[List[float]]:
        return [list(point) for point in self.clockwise_ground_contact_points]
