import  sys
sys.path.append("../")
from seer.trajectory import Trajectory
from typing import List


class SimpleTrajectory(Trajectory):
    def __init__(self, startPosition: List[float], endPosition: List[float], startTime: float):
        self.startPosition = startPosition
        self.endPosition = endPosition
        self.startTime = startTime
        # a = p(-2t+D)
        # v = -pt(t-D)
        # distance travelled = 1/6 * p * D**3
        # p is a coefficient and D is the duration of the movement
        self.direction = [v-u for (u,v) in zip(startPosition, endPosition)]
        distance = sum([v**2 for v in self.direction])
        self.p = 0.1
        self.trajectoryDuration = (distance * 6 / self.p)**(1/3)
        print(f"Duration of movement: {self.trajectoryDuration}")


    def getPosition(self, time : float) -> List[float]:
        dt = time - self.startTime
        assert dt >= 0
        if dt >= self.trajectoryDuration:
            return self.endPosition
        distanceTravelledMagnitude = (-1/3 *self.p* dt**3)+ (0.5*self.trajectoryDuration *self.p* dt**2)
        positionChange = self._normalizeVector(self.direction, distanceTravelledMagnitude)
        return [s + dx for (s,dx) in zip(self.startPosition, positionChange)]

    def getVelocity(self, time : float) -> List[float]:
        dt = time - self.startTime
        assert dt >= 0
        if dt >= self.trajectoryDuration:
            return [0.,0.,0.]
        velocityMagnitude = self.p*dt*(dt-self.trajectoryDuration)
        velocity = self._normalizeVector(self.direction, velocityMagnitude)
        return velocity

    def getAcceleration(self, time : float) -> List[float]:
        dt = time - self.startTime
        assert dt >= 0
        if dt >= self.trajectoryDuration:
            return [0.,0.,0.]
        accelerationMagnitude = -2*self.p*dt + self.p*self.trajectoryDuration
        acceleration = self._normalizeVector(self.direction, accelerationMagnitude)
        return acceleration
    
    def _normalizeVector(self, vec : List[float], magnitude: float) -> List[float]:
        norm = sum([v**2 for v in vec])
        return [v*magnitude/norm for v in vec]