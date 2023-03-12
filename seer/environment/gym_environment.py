import  sys
sys.path.append("../")
from typing import List
from pybullet_multigoal_gym.envs.base_envs.kuka_single_3_joint_env import KukaBullet3Env
import numpy as np
from seer.environment import SimpleEnvironment

class GymEnvironment(SimpleEnvironment):
    def __init__(self, kukaEnv : KukaBullet3Env):
        print(kukaEnv.__dict__.keys())
        self.kukaEnv = kukaEnv
        # forceAngleMetric = kukaEnv._force_angle_calculator
        robot = kukaEnv.robot
        robotId = robot.kuka_body_index 
        endEffectorLinkIndex = robot.end_effector_tip_joint_index 
        jointIndices = robot.kuka_joint_index 
        homeEndEffectorPosition = robot.end_effector_tip_initial_position
        super().__init__(self.kukaEnv.p, robotId, jointIndices, endEffectorLinkIndex, homeEndEffectorPosition, None)

    def computeInverseKinematics(self, endEffectorPosition : List[float]) -> List[float]:
        return self.kukaEnv.robot.compute_ik2(np.array(endEffectorPosition))
    
    def getForceAngleMetric(self) -> float:
        centreOfMass = self.kukaEnv.get_centre_of_mass()
        return self.kukaEnv.force_angle(centreOfMass)
    
    def getTime(self) -> float:
        return self.kukaEnv.simulation_time()
