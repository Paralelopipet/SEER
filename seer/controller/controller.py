import  sys
sys.path.append("../")
from abc import ABC, abstractmethod
from seer.trajectory import Trajectory
from typing import List

class Controller(ABC):
    def setTrajectory(self, trajectory : Trajectory):
        self.trajectory = trajectory

    @abstractmethod
    def getNextJointPositions(self) -> List[float]:
        pass

    @abstractmethod
    def getNextJointVelocities(self) -> List[float]:
        pass

    @abstractmethod
    def getNextJointTorques(self) -> List[float]:
        pass

    @abstractmethod
    def getEndEffectorWorldPosition(self) -> List[float]:
        pass


