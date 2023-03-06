import  sys
sys.path.append("../")
from abc import ABC, abstractmethod
from typing import List, Union
from pybullet_multigoal_gym.envs.base_envs.kuka_single_3_joint_env import KukaBullet3Env
from seer import utils
import numpy as np
from seer.environment import SimpleEnvironment

class GymEnvironment(SimpleEnvironment):
    def __init__(self, kukaEnv : KukaBullet3Env):
        self.kukaEnv = kukaEnv
        forceAngleMetric = kukaEnv._force_angle_calculator
        robot = kukaEnv.robot
        robotId = robot.kuka_body_index 
        endEffectorLinkIndex = self.kukaEnv.end_effector_tip_joint_index 
        jointIndices = self.kukaEnv.kuka_joint_index 
        homeEndEffectorPosition = self.kukaEnv.end_effector_tip_initial_position
        super().__init__(self.kukaEnv._p, robotId, jointIndices, endEffectorLinkIndex, homeEndEffectorPosition, forceAngleMetric)

    def computeInverseKinematics(self, endEffectorPosition : List[float]) -> List[float]:
        return self.kukaEnv.compute_ik(endEffectorPosition)
    
    def getForceAngleMetric(self) -> float:
        centreOfMass = self.kukaEnv.getCenterOfMass()
        return self.kukaEnv._force_angle(centreOfMass   )
