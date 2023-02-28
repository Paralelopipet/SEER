import sys
sys.path.append("../")
import pybullet as p
import time
import math
import pybullet_data
import seer.utils as utils
import pybullet_multigoal_gym as pg
from seer.controller import Controller, ForceAngleController
from seer.trajectory import TestTrajectory, Trajectory
from seer.stability_metrics.adapter import RobotConfig
import measuredTorque

MODE = "TORQUE"
# MODE = "POSITION"

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
# kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf")
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()
kuka_joint_indices = list(range(numJoints))
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

for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

p.setGravity(0, 0, -9.81)
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)

trailDuration = 15

def testController(controller : Controller, trajectory : Trajectory):
    endEffectorLinkIndex=kukaEndEffectorIndex
    for i in range(numJoints):
        p.setJointMotorControl2(kukaId, i, p.VELOCITY_CONTROL, force=0)
        p.enableJointForceTorqueSensor(kukaId, i, True)
    # p.setJointMotorControl2(kukaId, endEffectorLinkIndex, p.VELOCITY_CONTROL, force=0)
    control_mode = p.POSITION_CONTROL
    if MODE == "TORQUE":
        control_mode = p.TORQUE_CONTROL
    print(f"Using {control_mode}")        
    # Set the maximum joint torque for the robot (in Nm)
    max_torque = 500
    prevActualPos = [0.,0.,0.]
    prevTargetPos = [0.,0.,0.]
    p.stepSimulation()
    measuredTorques=  []
    # while True:
    for i in range(100):
        jointTorques = controller.getNextJointTorques()
        jointPositions = controller.getNextJointPositions()
        jointTorques = measuredTorque.MEASURED_TORQUE[i]
        # print(utils.formatVec(jointTorques))
        # jointTorques = [20] * 7
        for i in range(numJoints):
            if MODE == "TORQUE":
                p.setJointMotorControl2(bodyIndex=kukaId,
                                        jointIndex=i,
                                        controlMode=control_mode,
                                        force=jointTorques[i],
                                        targetVelocity=0)
            else:
                p.setJointMotorControl2(bodyIndex=kukaId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPositions[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)
        # p.setJointMotorControlArray(kukaId, jointIndices, control_mode, forces=jointTorques)
        # p.setJointMotorControlArray(kukaId, jointIndices, p.POSITION_CONTROL, targetPositions=jointPositions)
            
        res = p.getJointStates(kukaId, list(range(numJoints)))
        measuredJointTorques = [res[i][-1] for i in range(numJoints)]
        # measuredTorques.append(measuredJointTorques)
        print(utils.formatVec(measuredJointTorques))

        currentPos = controller.getEndEffectorWorldPosition()
        targetPos = trajectory.getPosition(time.time())
        p.addUserDebugLine(prevActualPos, currentPos, [1, 0, 0], 1, 10)
        p.addUserDebugLine(prevTargetPos, targetPos, [0, 0,0], 1, 10)
        prevActualPos = currentPos
        prevTargetPos = targetPos
        time.sleep(0.1)
        p.stepSimulation()
    # print(measuredTorques)

if __name__ == "__main__":
    target_positions = [0.2, 0.2, 1.3]  # target positions for the first three joints
    # target_positions = [0.184, 0.048, 1.390]
    # inverseKinematicControl(kukaId, kuka_joint_indices)
    robotId = kukaId
    jointIndices= kuka_joint_indices
    endEffectorLinkIndex = kukaEndEffectorIndex
    physicsClientId = 0
    robotConfig = RobotConfig()
    forceAngleController = ForceAngleController(robotId, jointIndices, endEffectorLinkIndex, [0.,0.,0.],robotConfig, physicsClientId)
    testTrajectory = TestTrajectory(None, None)
    forceAngleController.setTrajectory(testTrajectory)
    testController(forceAngleController, testTrajectory)
    # torque = forceAngleController.getNextJointTorques()
    # print(torque)
