import  sys
sys.path.append("../")
from abc import ABC, abstractmethod
from typing import List

class Trajectory(ABC):
    @abstractmethod
    def __init__(self, startPosition: List[float], endPosition: List[float]):
        pass

    @abstractmethod
    def getPosition(self, time : float) -> List[float]:
        pass

    @abstractmethod
    def getVelocity(self, time : float) -> List[float]:
        pass

    @abstractmethod
    def getAcceleration(self, time : float) -> List[float]:
        pass