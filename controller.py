import pybullet
from datetime import datetime
import math
import time

class KQueue:
    def __init__(self, k):
        self.queue = []
        self.k = k

    def add(self, item):
        if len(self.queue) >= self.k:
            self.queue = self.queue[1:]
        self.queue.append(item)

    def max(self):
        return max(self.queue)

class ForceAngleController:
    '''Implementation of force angle controller as proposed in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=656414
    
    This maybe needs the robot's geometry as input so as to compute the Jacobian for IK
    '''

    def __init__(self, currentTime, jointIndices, physicsClientId, robotId, endEffectorLinkIndex, verbose=False):
        self.endEffectorLinkIndex = endEffectorLinkIndex
        self.robotId = robotId
        self.jointIndices = jointIndices
        self.physicsClientId = physicsClientId
        self.isTipping = False
        self.currentTime = currentTime
        self.intervals = 0.1
        self.k=100
        self.timesUntilTipoverStat = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        self.timesUntilTipoverDyn = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        self.tipoverPreventionResponseDuration = 3 # duration that tipover response enacted (sec)
        self.timeElapsedTipoverPreventionInitiated = 0
        self.tipoverPreventionReponseThreshold = 0.15 # if tut is below this threshold for >=k steps, initiate tipover prevention
        if verbose:
            self._printSummary()

    def _printSummary(self):
        print(f"Force Angle Controller")
        print(f"Physics client id: {self.physicsClientId}")
        print(f"Robot id: {self.robotId}")
        print(f"End effector link index: {self.endEffectorLinkIndex}")
        print(f"Number of joints: {len(self.jointIndices)}")
        print(f"Intervals in trajectory planning: {self.intervals}s")
        print(f"Size of queue: {self.k}")
        print(f"Tipover Prevention Duration: {self.tipoverPreventionResponseDuration}")
        print(f"Tipover Response Initiation threshold: {self.tipoverPreventionReponseThreshold}")

    def _updateState(self):
        currentTime = time.time()
        # needs geometry of base, COM, resultant force vector on COM
        timeElapsed = currentTime - self.currentTime
        self.currentTime = currentTime
        # compute force angle stability measures
        fa_stat, fa_stat_dt, fa_dyn, fa_dyn_dt = self.computeForceAngleStabilityMeasures()
        self.timesUntilTipoverStat.add(fa_stat / fa_stat_dt)
        self.timesUntilTipoverDyn.add(fa_dyn / fa_dyn_dt)

        # Update internal state
        if self.timeElapsedTipoverPreventionInitiated > self.tipoverPreventionResponseDuration and not self._isTipoverTriggered():
            self.isTipping = False
            self.timeSinceTipoverPreventionInitiated = 0
        if self._isTipoverTriggered():
            if not self.isTipping:
                self.timeSinceTipoverPreventionInitiated = 0
            else:
                self.timeSinceTipoverPreventionInitiated += timeElapsed
            self.isTipping = True

    def getNextJointPoses(self):
        # compute trajectories
        self._updateState()
        if self.isTipping:
            return self._tipoverStateGetNextPose()
        else:
            return self._normalStateGetNextPose()

    def _isTipoverTriggered(self):
        return (self.timesUntilTipoverStat.max() < self.tipoverPreventionReponseThreshold) or (self.timesUntilTipoverDyn.max() < self.tipoverPreventionReponseThreshold)

    def generateTrajectory(self, startTime, startEndEffectorPosition, targetEndEffectorPosition, movementDuration=5):
        self.trajectoryStartTime = startTime
        movementDuration = 10 # secs
         # secs
        self.timesteps = [i*self.intervals for i in range(int(movementDuration / self.intervals))]
        # positions = [[-0.4, 0.02 * math.cos(t), 0. + 0.02 * math.sin(t)] for t in self.timesteps]
        positions = [[0.7,0,1.5] for t in self.timesteps]
        self.timesteps = [i + self.trajectoryStartTime for i in self.timesteps]
        self.positions = positions

    def _nextPosition(self):
        if self.currentTime <= self.timesteps[0]:
            return self.positions[0]
        if self.currentTime >= self.timesteps[-1]:
            return self.positions[-1]
        index = math.ceil((self.currentTime - self.trajectoryStartTime) / self.intervals)
        targetPosition = self.positions[index]
        return targetPosition

    def _normalStateGetNextPose(self):
        nextEndEffectorPosition = self._nextPosition()
        jointPoses = pybullet.calculateInverseKinematics(self.robotId, self.endEffectorLinkIndex, targetPosition=nextEndEffectorPosition, physicsClientId=self.physicsClientId)
        return jointPoses

    def _tipoverStateGetNextPose(self):
        pass

    def computeForceAngleStabilityMeasures(self):
         # will return fa_stat, fa_stat_dt, fa_dyn, fa_dyn_dt
        return 1,1,1,1


if __name__ == "__main__":
    pass