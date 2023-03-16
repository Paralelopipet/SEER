import  sys
sys.path.append("../")
from typing import List
from seer.controller import Controller
from seer.trajectory import SimpleTrajectory
import time
import seer.utils as utils
import numpy as np 
import scipy.optimize as optimization
from seer.stability_metrics.adapter.stability_metric_adapter import StabilityMetricAdapter
from seer.stability_metrics.adapter import RobotConfig, RobotState 
from seer.environment import Environment
from seer.stability_metrics.force_angle import ForceAngle

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
    
    def __len__(self):
        return len(self.queue)

    
    
class ForceAngleController(Controller):
    '''Implementation of force angle controller as proposed in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=656414
    
    This maybe needs the robot's geometry as input so as to compute the Jacobian for IK
    '''
    def __init__(self, environment: Environment):
        self.environment = environment
        self.forceAngleMetric = self.environment.getForceAngleMetric()
        self.isTipping = False
        self.currentTime = environment.getTime()# time.time()
        self.k=10
        self.gravity = [0, 0, -9.81]
        self.kp = 0.1
        self.kd = 0.1
        self.timesUntilTipoverStat = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        for _ in range(self.k):
            self.timesUntilTipoverStat.add(100)
        # self.timesUntilTipoverDyn = KQueue(self.k) # needs to be implemented as a queue that records the last k time steps.
        self.alphas = KQueue(100)
        self.tipoverPreventionResponseDuration = 3 # duration that tipover response enacted (sec)
        self.timeElapsedTipoverPreventionInitiated = 0
        self.tipoverPreventionResponseThreshold = 0.15 # if tut is below this threshold for >=k steps, initiate tipover prevention
        self.tipoverReponsesGenerated = 0

        # self.forceAngleMetric = StabilityMetricAdapter.instance(self.robotConfig)
        verbose = False
        if verbose:
            self._printSummary()

    def _estimateGradient2(self, currentTime) -> float:
        if len(self.alphas) < 10:
            return 0
        alphas = list(zip(*self.alphas.toList()))[-10:]
        alphas, times = np.array(alphas[0]), np.array(alphas[1])
        initialGuess = [0.,0.,0.,0.]
        def func(x, a, b, c, d):
            return a + b*x + c*x*x + d*(x**3)
        curve, _ = optimization.curve_fit(func, times, alphas, initialGuess,maxfev=1100)
        def grad(x,b,c,d):
            return b + 2*c*x + 3*d*x*x
        gradient = grad(currentTime, curve[1], curve[2], curve[3])
        return gradient
    
    def _estimateGradient(self) -> float:
        if len(self.alphas) < 2:
            return 1
        prev, t1 = self.alphas.queue[-2]
        current, t2 = self.alphas.queue[-1]
        return (current - prev) / (t2-t1)
    
    def _updateState2(self):
        if self.trajectory is None:
            raise Exception("No trajectory is currently set.")
        currentTime = self.environment.getTime()
        timeElapsed = currentTime - self.currentTime
        if timeElapsed > 6 and not self.isTipping:
            self._generateTipoverPreventionTrajectory()
            self.isTipping = True

    def _updateState(self):
        currentTime = self.environment.getTime()
        timeElapsed = currentTime - self.currentTime
        self.currentTime = currentTime
        forceAngle = self.environment.getForceAngleMetric()
        self.alphas.add((forceAngle, currentTime))
        forceAngleGradientEstimate = self._estimateGradient2(currentTime)
        # forceAngleGradientEstimate = self._estimateGradient()
        tut = -forceAngle / forceAngleGradientEstimate
        if tut < 0:
            # THis means that actually forceAngle value is increasing resulting in negative tut in this case we should be setting tut to some arbitrary large value, anything is ok as long as it is above the threshold
            tut = 1e10
        # print(tut)
        # print(forceAngle)
        # print(forceAngleGradientEstimate)
        # print(tut)
        self.timesUntilTipoverStat.add(tut)

        if self.isTipping:
            if self._isTipoverTriggered() or self.timeElapsedTipoverPreventionInitiated < self.tipoverPreventionResponseDuration:
                # print(self._isTipoverTriggered())
                # print(self.timeElapsedTipoverPreventionInitiated < self.tipoverPreventionResponseDuration)
                print("Recovering from tipping state")
                # stay in tipping state
                self.timeElapsedTipoverPreventionInitiated += timeElapsed
            else:
                print("Reached stable state again")
                # transition to stable state
                self.isTipping = False 
                self.timeElapsedTipoverPreventionInitiated = 0 
        else:
            if self._isTipoverTriggered():
                # transition to tipping state
                print("Generating tipover response")
                self.timeElapsedTipoverPreventionInitiated = 0
                self.isTipping = True
                self._generateTipoverPreventionTrajectory()
            else:
                pass
                # print("Stable")

    def _isTipoverTriggered(self):
        return (self.timesUntilTipoverStat.max() < self.tipoverPreventionResponseThreshold) # or (self.timesUntilTipoverDyn.max() < self.tipoverPreventionReponseThreshold)
    
    def _generateTipoverPreventionTrajectory(self):
        self.tipoverReponsesGenerated +=1
        currentEndEffectorPosition = self.getEndEffectorWorldPosition()
        trajectory = SimpleTrajectory(currentEndEffectorPosition, self.environment.getEndEffectorHomePosition(), self.environment.getTime())
        self.setTrajectory(trajectory)

    def tipoverResponsesSinceLastCheck(self):
        foo = self.tipoverReponsesGenerated
        self.tipoverReponsesGenerated = 0 
        return foo

    def _printSummary(self):
        print(f"Force Angle Controller")
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
    
    def getNextJointPositions(self, t=None) -> List[float]:
        self._updateState()
        if t is None:
            t = self.environment.getTime()
        endEffectorPosition = self.trajectory.getPosition(t)
        jointPoses = self.environment.computeInverseKinematics(endEffectorPosition)
        return jointPoses
    
    def getNextJointVelocities(self, t=None) -> List[float]:
        raise Exception("Not Implemented")
        # endEffectorVelocity
    
    def getNextJointTorques(self, t=None) -> List[float]:
        self._updateState()
        if t is None:
            t = self.environment.getTime()
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
    

