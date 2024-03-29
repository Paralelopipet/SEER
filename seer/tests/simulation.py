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
from pybullet_multigoal_gym.utils.assets_dir import ASSETS_DIR
from pybullet_multigoal_gym.utils.assets_dir import CUBE_LINK_NAME
from seer.environment import SimpleEnvironment
from seer.stability_metrics import StabilityMetricAdapter
from seer.stability_metrics.force_angle import ForceAngleConfigAdapter, ForceAngle
# MODE = "TORQUE"
MODE = "POSITION"

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])

# model_urdf=str(ASSETS_DIR / 'robots' / 'kuka' / 'iiwa14_parallel_jaw_cube.urdf')
model_urdf = "kuka_iiwa/model.urdf"
kukaId = p.loadURDF(model_urdf, [0, 0, 0])
# kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf")
# p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
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
for i in range(numJoints):
        p.setJointMotorControl2(kukaId, i, p.VELOCITY_CONTROL, force=0)
        p.enableJointForceTorqueSensor(kukaId, i, True)


def testController(controller : Controller):
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
    for i in range(1000):
        jointTorques = controller.getNextJointTorques()
        jointPositions = controller.getNextJointPositions()
        # jointTorques = measuredTorque.MEASURED_TORQUE[i]
        # print(utils.formatVec(jointTorques))
        # jointTorques = [20] * 7
        for i in range(numJoints):
            if MODE == "TORQUE":
                p.setJointMotorControl2(bodyIndex=kukaId,
                                        jointIndex=i,
                                        controlMode=control_mode,
                                        force=jointTorques[i])
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
        # print(utils.formatVec(measuredJointTorques))

        currentPos = controller.getEndEffectorWorldPosition()
        targetPos = controller.trajectory.getPosition(time.time())
        p.addUserDebugLine(prevActualPos, currentPos, [1, 0, 0], 1, 10)
        p.addUserDebugLine(prevTargetPos, targetPos, [0, 0,0], 1, 10)
        prevActualPos = currentPos
        prevTargetPos = targetPos
        time.sleep(0.1)
        p.stepSimulation()
    # print(measuredTorques)

if __name__ == "__main__":
    robotId = kukaId
    jointIndices= kuka_joint_indices
    endEffectorLinkIndex = kukaEndEffectorIndex
    physicsClientId = 0
    # robotConfig = RobotConfig(cube_link_name=CUBE_LINK_NAME, urdf_path=model_urdf)
    robotConfig = None
    homeEndEffectorPosition =[0., 0., 0.9]
    model_urdf2=str(ASSETS_DIR / 'robots' / 'kuka' / 'iiwa14_parallel_jaw_cube.urdf')

    robotConfig = RobotConfig(cube_link_name=CUBE_LINK_NAME, urdf_path=model_urdf2)
    forceAngleConfig = ForceAngleConfigAdapter.convert(robotConfig)
    forceAngleCalculator = ForceAngle(forceAngleConfig)

    environment = SimpleEnvironment(p, robotId, jointIndices, endEffectorLinkIndex, homeEndEffectorPosition, forceAngleCalculator)

    forceAngleController = ForceAngleController(robotConfig, environment)
    testTrajectory = TestTrajectory(None, None)
    forceAngleController.setTrajectory(testTrajectory)
    testController(forceAngleController)
    # torque = forceAngleController.getNextJointTorques()
    # print(torque)
