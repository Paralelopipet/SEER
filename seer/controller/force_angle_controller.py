import  sys
sys.path.append("../")
from typing import List
from seer.controller import Controller
from seer.trajectory import TestTrajectory, Trajectory, SimpleTrajectory
import time
import pybullet as p
import seer.utils as utils
import numpy as np 
import scipy.optimize as optimization
from seer.stability_metrics.adapter.stability_metric_adapter import StabilityMetricAdapter
from seer.stability_metrics.adapter import RobotConfig, RobotState
from seer.environment import Environment
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
    
    def toList(self):
        return self.queue
    
    
class ForceAngleController(Controller):
    '''Implementation of force angle controller as proposed in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=656414
    
    This maybe needs the robot's geometry as input so as to compute the Jacobian for IK
    '''
    def __init__(self, robotId: int, jointIndices: List[int], endEffectorLinkIndex : int, homeEndEffectorPosition: List[float], robotConfig : RobotConfig, physicsClientId: int, environment: Environment):
        # self.endEffectorLinkIndex = endEffectorLinkIndex
        self.environment = environment
        # self.homeEndEffectorPosition = homeEndEffectorPosition
        # self.robotId = robotId
        # self.jointIndices = jointIndices
        # self.physicsClientId = physicsClientId
        self.robotConfig = robotConfig
        self.isTipping = False
        self.currentTime = time.time()
        self.k=100
        self.gravity = [0, 0, -9.81]
        self.kp = 0.1
        self.kd = 0.1
        self.timesUntilTipoverStat = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        self.timesUntilTipoverDyn = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        self.alphas = KQueue(100)
        self.tipoverPreventionResponseDuration = 3 # duration that tipover response enacted (sec)
        self.timeElapsedTipoverPreventionInitiated = 0
        self.tipoverPreventionResponseThreshold = 0.15 # if tut is below this threshold for >=k steps, initiate tipover prevention

        # self.forceAngleMetric = StabilityMetricAdapter.instance(self.robotConfig)
        verbose = True
        if verbose:
            self._printSummary()
        print(self.getEndEffectorWorldPosition())

    def _estimateGradient(self, currentTime) -> float:
        alphas = list(zip(*self.alphas.toList()))
        alphas, times = np.array(alphas[0]), np.array(alphas[1])
        initialGuess = np.array([0.,0.,0.,0.])
        def func(x, a, b, c, d):
            return a + b*x + c*x*x + d*(x**3)
        curve, _ = optimization.curve_fit(func, times, alphas, initialGuess)
        def grad(x,b,c,d):
            return b + 2*c*x + 3*d*x*x
        gradient = grad(currentTime, curve[1], curve[2], curve[3])
        return gradient
    
    def _updateState(self):
        currentTime = time.time()
        timeElapsed = currentTime - self.currentTime
        if timeElapsed > 6 and not self.isTipping:
            self._generateTipoverPreventionTrajectory()
            self.isTipping = True

    def _updateState2(self):
        currentTime = time.time()
        timeElapsed = currentTime - self.currentTime
        self.currentTime = currentTime
        # TODO will need updating once RobotState is implemented
        robotState = RobotState()
        forceAngle = self.forceAngleMetric.get(robotState)
        self.alphas.add((forceAngle, currentTime))
        forceAngleGradientEstimate = self._estimateGradient(currentTime)
        self.timesUntilTipoverStat.add(-forceAngle / forceAngleGradientEstimate)

        # Update internal state
        if self.timeElapsedTipoverPreventionInitiated > self.tipoverPreventionResponseDuration and not self._isTipoverTriggered():
            self.isTipping = False
            self.timeSinceTipoverPreventionInitiated = 0
        if self._isTipoverTriggered():
            if not self.isTipping:
                # set new trajectory here
                self._generateTipoverPreventionTrajectory()
                self.timeSinceTipoverPreventionInitiated = 0
            else:
                self.timeSinceTipoverPreventionInitiated += timeElapsed
            self.isTipping = True
    
    def _isTipoverTriggered(self):
        return (self.timesUntilTipoverStat.max() < self.tipoverPreventionResponseThreshold) # or (self.timesUntilTipoverDyn.max() < self.tipoverPreventionReponseThreshold)
    
    def _generateTipoverPreventionTrajectory(self):
        self.homeEndEffectorPosition
        currentEndEffectorPosition = self.getEndEffectorWorldPosition()
        trajectory = SimpleTrajectory(currentEndEffectorPosition, self.homeEndEffectorPosition, time.time())
        self.setTrajectory(trajectory)

    def _printSummary(self):
        print(f"Force Angle Controller")
        print(f"Physics client id: {self.physicsClientId}")
        print(f"Robot id: {self.robotId}")
        print(f"End effector link index: {self.endEffectorLinkIndex}")
        print(f"Number of joints: {len(self.jointIndices)}")
        # print(f"Intervals in trajectory planning: {self.intervals}s")
        print(f"Size of queue: {self.k}")
        print(f"Tipover Prevention Duration: {self.tipoverPreventionResponseDuration}")
        print(f"Tipover Response Initiation threshold: {self.tipoverPreventionResponseThreshold}")

    
    def getEndEffectorWorldState(self) -> tuple:
        '''returns end effector world position and velocity'''
        return self.environment.getEndEffectorWorldState()
    
    def getEndEffectorWorldPosition(self) -> List[float]:
        pos, _ = self.getEndEffectorWorldState()
        return pos
    
    def getNextJointPositions(self) -> List[float]:
        self._updateState()
        endEffectorPosition = self.trajectory.getPosition(time.time())
        jointPoses = self.environment.computeInverseKinematics(endEffectorPosition)
        return jointPoses
    
    def getNextJointVelocities(self) -> List[float]:
        raise Exception("Not Implemented")
    
    def getNextJointTorques(self) -> List[float]:
        self._updateState()
        t = time.time()
        desiredEndEffectorPosition = self.trajectory.getPosition(t)
        desiredEndEffectorVelocity = self.trajectory.getVelocity(t)
        currentEndEffectorPosition, currentEndEffectorVelocity = self.environment.getEndEffectorWorldState() 
        return self.pdVirtualForcesPdControl(currentEndEffectorPosition, desiredEndEffectorPosition, currentEndEffectorVelocity, desiredEndEffectorVelocity)
    
    def pdVirtualForcesPdControl(self, currentEndEffectorPosition, desiredEndEffectorPosition, currentEndEffectorVelocity, desiredEndEffectorVelocity):
        # currentJointPositions, _, _ = self.getJointStates()
        endEffectorForceVector = [(-(self.kp * (currentEndEffectorPosition[i] - desiredEndEffectorPosition[i])) - (self.kd * (currentEndEffectorVelocity[i] - desiredEndEffectorVelocity[i])))  for i in range(len(currentEndEffectorPosition))]

        jacobianTranpose = utils.transpose(self.environment.getJacobian())
        result = np.matmul(jacobianTranpose, endEffectorForceVector)
        gravityVector = self.environment.getGravityVector()
        # print(len(result))
        result = [result[i]+gravityVector[i] for i in range(len(result))]
        return result
    

