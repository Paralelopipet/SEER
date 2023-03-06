from typing import List, Union
from seer.environment import Environment
from seer import utils
import numpy as np
import pybullet as p

class SimpleEnvironment(Environment):
    def __init__(self, robotId, jointIndices, endEffectorLinkIndex, homeEndEffectorPosition, physicsClientId):
        self.physicsClientId = physicsClientId
        self.robotId = robotId
        self.jointIndices = jointIndices 
        self.endEffectorLinkIndex = endEffectorLinkIndex
        self.gravity = [0,0,-9.81]
        self.homeEndEffectorPosition = homeEndEffectorPosition

    def getEndEffectorWorldState(self) -> tuple:
        # returns ee position and velocity
        result = p.getLinkState(self.robotId, self.endEffectorLinkIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
        linkWorldPosition, link_rot, com_trn, com_rot, frame_pos, frame_rot, worldLinkLinearVelocity, link_vr = result
        return linkWorldPosition, worldLinkLinearVelocity
        

    def getJacobian(self, linkIndex : Union[int, None] = None) -> List[List[float]]:
        if linkIndex is None:
            linkIndex = self.endEffectorLinkIndex
        currentJointPositions,_,_ = self._getJointStates()
        zero_vec = [0.] * len(currentJointPositions)
        result = p.getLinkState(self.robotId, linkIndex,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        jac_t, jac_r = p.calculateJacobian(self.robotId, linkIndex, com_trn, currentJointPositions, zero_vec, zero_vec)
        jac_t = [j[:len(currentJointPositions)] for j in jac_t]
        return jac_t

    def getGravityVector(self) -> List[float]:
        currentJointPositions, _, _ = self._getJointStates()
        gravityVector = [0. for _ in range(len(currentJointPositions))]
        for i in range(len(currentJointPositions)):
            jacobian = self.getJacobian(linkIndex=i)
            jacobian = utils.transpose(jacobian)# hopefully this gives the jacobian for the joints
            massOfLink,_,_,_,_,_,_,_,_,_,_,_ = p.getDynamicsInfo(self.robotId, i)
            # jacobian_Transpose should be 7x3
            gravitationalForce = [i*massOfLink for i in self.gravity]
            linkGravity = np.matmul(jacobian, gravitationalForce)
            assert len(linkGravity) == len(gravityVector)
            for j in range(len(linkGravity)):
                gravityVector[j] += linkGravity[j]
        return gravityVector

    def computeInverseKinematics(self, endEffectorPosition : List[float]) -> List[float]:
        jointPoses = p.calculateInverseKinematics(self.robotId, self.endEffectorLinkIndex, targetPosition=endEffectorPosition, physicsClientId=self.physicsClientId)
        return jointPoses

    def _getJointStates(self) -> tuple:
        joint_states = p.getJointStates(self.robotId, range(p.getNumJoints(self.robotId)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques