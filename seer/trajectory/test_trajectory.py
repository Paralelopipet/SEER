import  sys
sys.path.append("../")
from seer.trajectory import Trajectory
import math
from typing import List

class TestTrajectory(Trajectory):
    def __init__(self, startPosition: List[float], endPosition: List[float]):
        y_coeff = 0.2 
        z_coeff = 0.2
        z_offset = 0.5
        self.pos = lambda t : [-0.5, 0.2 * math.cos(t), z_offset + 0.2 * math.sin(t)]
        self.vel = lambda t : [0, -0.2 * math.sin(t),  0.2 * math.cos(t)]
        self.acc = lambda t : [0, -0.2 * math.cos(t), -0.2 * math.sin(t)]
    
    def getPosition(self, time : float) -> List[float]:
        return self.pos(time)

    def getVelocity(self, time : float) -> List[float]:
        return self.vel(time)

    def getAcceleration(self, time : float) -> List[float]:
        return self.acc(time)