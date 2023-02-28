import  sys
sys.path.append("../")
import math
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet as p
import pybullet_data
import time
from datetime import datetime
import seer.utils as utils
from seer.controller import Controller, ForceAngleController
from seer.stability_metrics.adapter import RobotConfig
from seer.trajectory import TestTrajectory, Trajectory

config = {
    "box_scale" :       0.5,
    "box_mass_scale":   10.0,
    "box_fixed" :       1, # 0 if free, 1 if fixed
    "timestep" :        0.1# 1/240.
}

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

p.setPhysicsEngineParameter(enableConeFriction=0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0, 0, -0.3])
model_path = "../pybullet_multigoal_gym/pybullet_multigoal_gym/assets/objects/assembling_shape/cube.urdf"

# boxId = p.loadURDF("cube.urdf", useFixedBase=config["box_fixed"], globalScaling=config["box_scale"])
boxId = p.loadURDF(model_path, useFixedBase=config["box_fixed"], globalScaling=config["box_scale"])
# kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf", 0.193749, 0.345564, 0.120208, 0.002327,-0.000988, 0.996491, 0.083659)
kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf")
jointPositions = [3.559609, 0.411182, 0.862129, 1.744441, 0.077299, -1.129685, 0.006001]


cid = p.createConstraint(boxId, -1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., -.26],
                         [0, 0, 0, 1])

baseorn = p.getQuaternionFromEuler([3.1415, 0, 0.3])
baseorn = [0, 0, 0, 1]
#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()
for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])
kuka_joint_indices = list(range(numJoints))
p.setGravity(0, 0, -10)
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)


def inverseKinematicControl(kukaId,joint_indices):
  control_mode = p.POSITION_CONTROL
  # Set the maximum joint torque for the robot (in Nm)
  max_torque = 500
  endEffectorLinkIndex=kukaEndEffectorIndex
  prevActualPos = [0.,0.,0.]
  prevTargetPos = [0.,0.,0.]
  while True:
    if (useRealTimeSimulation):
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
    # targetPos = [0.2 * math.cos(t), 0.3, 0. + 0.2 * math.sin(t) + 0.7]
    targetPos  = [-0.5, 0.2 * math.cos(t), 0.5 + 0.2 * math.sin(t)]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])
    jointPoses = p.calculateInverseKinematics(kukaId, endEffectorLinkIndex, targetPos, orn, ll, ul,jr, rp)
    for i in range(numJoints):
      p.setJointMotorControl2(bodyIndex=kukaId,
                              jointIndex=i,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=jointPoses[i],
                              targetVelocity=0,
                              force=500,
                              positionGain=0.03,
                              velocityGain=1)
    # for i in range(numJoints):
    #   p.resetJointState(kukaId, i, jointPoses[i])
    # print(f"joint pose length = {len(jointPoses)}")
    # p.setJointMotorControlArray(kukaId, joint_indices, control_mode, targetPositions=jointPoses)
    # for i in range(numJoints):
    #   p.setJointMotorControl2(bodyIndex=kukaId,
    #                           jointIndex=i,
    #                           controlMode=p.POSITION_CONTROL,
    #                           targetPosition=jointPoses[i],
    #                           targetVelocity=0,
    #                           force=500,
    #                           positionGain=0.03,
    #                           velocityGain=1)
    actualPos,_,_,_,_,_ = p.getLinkState(kukaId, endEffectorLinkIndex)
    p.addUserDebugLine(prevActualPos, actualPos, [1, 0, 0], 1, 10)
    p.addUserDebugLine(prevTargetPos, targetPos, [0, 0,0], 1, 10)
    prevActualPos = actualPos
    prevTargetPos = targetPos

def testController(controller : Controller, trajectory : Trajectory):
    endEffectorLinkIndex=kukaEndEffectorIndex
    # p.setJointMotorControl2(kukaId, endEffectorLinkIndex, p.VELOCITY_CONTROL, force=0)
    for i in range(numJoints):
        p.setJointMotorControl2(kukaId, i, p.VELOCITY_CONTROL, force=0)
    control_mode = p.TORQUE_CONTROL
    # Set the maximum joint torque for the robot (in Nm)
    max_torque = 500
    prevActualPos = [0.,0.,0.]
    prevTargetPos = [0.,0.,0.]
    while True:
        jointTorques = controller.getNextJointTorques()
        print(utils.formatVec(jointTorques))
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=kukaId,
                                    jointIndex=i,
                                    controlMode=control_mode,
                                    force=jointTorques[i])
        currentPos = controller.getEndEffectorWorldPosition()
        targetPos = trajectory.getPosition(time.time())

        res = p.getJointStates(robotId, jointIndices, physicsClientId)
        print([res[i][-1] for i in range(numJoints)])

    # for i in range(numJoints):
    #   p.resetJointState(kukaId, i, jointPoses[i])
    # print(f"joint pose length = {len(jointPoses)}")
    # p.setJointMotorControlArray(kukaId, joint_indices, control_mode, targetPositions=jointPoses)
    # for i in range(numJoints):
    #   p.setJointMotorControl2(bodyIndex=kukaId,
    #                           jointIndex=i,
    #                           controlMode=p.POSITION_CONTROL,
    #                           targetPosition=jointPoses[i],
    #                           targetVelocity=0,
    #                           force=500,
    #                           positionGain=0.03,
    #                           velocityGain=1)
    # actualPos,_,_,_,_,_ = p.getLinkState(kukaId, endEffectorLinkIndex)
        p.addUserDebugLine(prevActualPos, currentPos, [1, 0, 0], 1, 10)
        p.addUserDebugLine(prevTargetPos, targetPos, [0, 0,0], 1, 10)
        prevActualPos = currentPos
        prevTargetPos = targetPos
        p.stepSimulation() 
        time.sleep(0.1)

if __name__ == "__main__":
    target_positions = [0.2, 0.2, 1.3]  # target positions for the first three joints
    # target_positions = [0.184, 0.048, 1.390]
    # inverseKinematicControl(kukaId, kuka_joint_indices)
    robotId = kukaId
    jointIndices= kuka_joint_indices
    endEffectorLinkIndex = kukaEndEffectorIndex
    physicsClientId = 0
    robotConfig = RobotConfig()
    testTrajectory = TestTrajectory(None, None)
    forceAngleController = ForceAngleController(robotId, jointIndices, endEffectorLinkIndex, [0.,0.,0.],robotConfig, physicsClientId)
    forceAngleController.setTrajectory(testTrajectory)
    testController(forceAngleController, testTrajectory)
    # torque = forceAngleController.getNextJointTorques()
    # print(torque)
