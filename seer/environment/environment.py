import  sys
sys.path.append("../")
from abc import ABC, abstractmethod
from typing import List, Union
from seer.stability_metrics.force_angle import ForceAngle

class Environment(ABC):
    @abstractmethod
    def getEndEffectorWorldState(self) -> tuple:
        # returns ee position and velocity
        pass

    @abstractmethod
    def getJacobian(self, linkIndex : Union[int, None] = None) -> List[List[float]]:
        pass

    @abstractmethod
    def getGravityVector(self) -> List[List[float]]:
        pass

    @abstractmethod
    def computeInverseKinematics(self, endEffectorPosition : List[float]) -> List[float]:
        pass
    
    @abstractmethod
    def getEndEffectorHomePosition(self) -> List[float]:
        pass

    @abstractmethod
    def getForceAngleMetric(self) -> float:
        pass