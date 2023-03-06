from typing import List, Union
from seer.environment import Environment
from seer import utils
import numpy as np
from seer.stability_metrics.force_angle import ForceAngle, ForceAngleConfigAdapter, ForceAngleStateAdapter
from seer.stability_metrics.adapter import RobotConfig, StabilityMetricAdapter, RobotState

class SimpleEnvironment(Environment):
    def __init__(self, p, robotId : int, jointIndices : List[int], endEffectorLinkIndex : int, homeEndEffectorPosition : List[float], forceAngleCalculator : ForceAngle):
        self.p = p
        self.robotId = robotId
        self.jointIndices = jointIndices 
        self.endEffectorLinkIndex = endEffectorLinkIndex
        self.gravity = [0,0,-9.81]
        self.homeEndEffectorPosition = homeEndEffectorPosition
        self.forceAngleCalculator = forceAngleCalculator

    def getEndEffectorWorldState(self) -> tuple:
        # returns ee position and velocity
        result = self.p.getLinkState(self.robotId, self.endEffectorLinkIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
        linkWorldPosition, link_rot, com_trn, com_rot, frame_pos, frame_rot, worldLinkLinearVelocity, link_vr = result
        return linkWorldPosition, worldLinkLinearVelocity
        

    def getJacobian(self, linkIndex : Union[int, None] = None) -> List[List[float]]:
        if linkIndex is None:
            linkIndex = self.endEffectorLinkIndex
        currentJointPositions,_ = self._getJointStates()
        zero_vec = [0.] * len(currentJointPositions)
        result = self.p.getLinkState(self.robotId, linkIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        jac_t, jac_r = self.p.calculateJacobian(self.robotId, linkIndex, com_trn, currentJointPositions, zero_vec, zero_vec)
        jac_t = [j[:len(currentJointPositions)] for j in jac_t]
        return jac_t

    def getGravityVector(self) -> List[float]:
        currentJointPositions, _ = self._getJointStates()
        gravityVector = [0. for _ in range(len(currentJointPositions))]
        for i in range(len(currentJointPositions)):
            jacobian = self.getJacobian(linkIndex=i)
            jacobian = utils.transpose(jacobian)# hopefully this gives the jacobian for the joints
            massOfLink,_,_,_,_,_,_,_,_,_,_,_ = self.p.getDynamicsInfo(self.robotId, i)
            # jacobian_Transpose should be 7x3
            gravitationalForce = [i*massOfLink for i in self.gravity]
            linkGravity = np.matmul(jacobian, gravitationalForce)
            assert len(linkGravity) == len(gravityVector)
            for j in range(len(linkGravity)):
                gravityVector[j] += linkGravity[j]
        return gravityVector

    def computeInverseKinematics(self, endEffectorPosition : List[float]) -> List[float]:
        jointPoses = self.p.calculateInverseKinematics(self.robotId, self.endEffectorLinkIndex, targetPosition=endEffectorPosition)
        return jointPoses
    
    def getEndEffectorHomePosition(self) -> List[float]:
        return self.homeEndEffectorPosition 
    
    def getForceAngleMetric(self) -> float:
        # TODO this does not compute the correct forces currently.  
        fx, fy, fz, mx, my, mz = self.p.getJointState(self.robotId, 0)[2]
        centreOfMass = self._getCentreOfMass()
        robotState = RobotState(
            centre_of_mass=np.array(centreOfMass),
            net_force=np.array([fx,fy,fz]),
            net_moment=np.array([mx,my,mz]))
        forceAngleState = ForceAngleStateAdapter.convert(robotState)
        return self.forceAngleCalculator.get(forceAngleState)

    def _getJointStates(self) -> tuple:
        joint_states = self.p.getJointStates(self.robotId, range(self.p.getNumJoints(self.robotId)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        # joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities

    def _getCentreOfMass(self) -> List[float]:
        # Calculate the center of mass of the robot_id
        total_mass = 0
        for link_idx in range(self.p.getNumJoints(self.robotId)):
            link_mass = self.p.getDynamicsInfo(self.robotId, link_idx)[0]
            total_mass += link_mass

        com_position: List[float] = [0, 0, 0]
        joint_poses, joint_vels = self._getJointStates()
        for link_idx in range(len(joint_poses)):
            link_mass = list(self.p.getDynamicsInfo(self.robotId, link_idx))[0]
            link_com = self.p.getLinkState(self.robotId, link_idx)[0]
            link_com_position = [link_com[i] for i in range(3)]
            link_com_offset = [(link_mass/total_mass) * link_com_position[i] for i in range(3)]
            com_position = [com_position[i] + link_com_offset[i] for i in range(3)]
        return com_position