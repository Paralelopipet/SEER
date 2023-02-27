import math
from pybullet_multigoal_gym.pybullet_multigoal_gym.utils.cube_path import CUBE_PATH
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet
import pybullet_data
import time
from enum import Enum
from controller import ForceAngleController
from measurements import *
class ControlType(Enum):
    FORWARD_KINEMATICS = 1
    INVERSE_KINEMATICS = 2
    TORQUE_CONTROL = 3

config = {
    "box_scale" :       0.5,
    "box_mass_scale":   10.0,
    "box_fixed" :       0, # 0 if free, 1 if fixed
    "timestep" :        1/240.
}


def load_environment():
    p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
    p0.setAdditionalSearchPath(pybullet_data.getDataPath())

    p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
    p1.setAdditionalSearchPath(pybullet_data.getDataPath())

    #can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

    box = p1.loadURDF(CUBE_PATH, useFixedBase=config["box_fixed"], globalScaling=config["box_scale"])
    panda = p0.loadURDF("franka_panda/panda.urdf")
    # panda = p0.loadURDF("kuka_iiwa/model.urdf")

    #check and change dynamics (masses et cetera)
    boxDyn = p1.getDynamicsInfo(box, -1)
    p1.changeDynamics(box, -1, mass=boxDyn[0]*config["box_mass_scale"])

    ed0 = ed.UrdfEditor()
    ed0.initializeFromBulletBody(box, p1._client)
    ed1 = ed.UrdfEditor()
    ed1.initializeFromBulletBody(panda, p0._client)
    #ed1.saveUrdf("combined.urdf")

    parentLinkIndex = 0

    jointPivotXYZInParent = [0, 0, 0.5 * config["box_scale"]]
    jointPivotRPYInParent = [0, 0, 0]

    jointPivotXYZInChild = [0, 0, 0]
    jointPivotRPYInChild = [0, 0, 0]

    newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                            jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
    newjoint.joint_type = p0.JOINT_FIXED

    # ed0.saveUrdf("combined.urdf")

    print(p0._client)
    print(p1._client)
    print("p0.getNumBodies()=", p0.getNumBodies())
    print("p1.getNumBodies()=", p1.getNumBodies())

    pgui = bc.BulletClient(connection_mode=pybullet.GUI)
    # pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 0)

    orn = [0, 0, 0, -1]
    ed0.createMultiBody([0, 0, config["box_scale"]/2], orn, pgui._client)
    plane_id = pgui.loadURDF("plane.urdf")
    pgui.setRealTimeSimulation(0)

    # pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 1)

    # set gravity 
    pgui.setGravity(0.0, 0.0, -9.81)
    return pgui, panda, box, plane_id

def FK_control(pgui, robot_id, joint_indices, target_positions):
    
    # Set up the controller for the first three joints
    joint_positions = [0.0, 0.0, 0.0]  # initial joint positions
    joint_velocities = [0.0, 0.0, 0.0]  # initial joint velocities
    joint_torques = [0.0, 0.0, 0.0]  # initial joint torques

    # Set the position control mode for the joints
    control_mode = pybullet.POSITION_CONTROL

    # Set the maximum joint torque for the robot (in Nm)
    max_torque = 500

    
    # Apply the joint torques to the robot
    pgui.setJointMotorControlArray(robot_id, joint_indices, control_mode, targetPositions=target_positions)

    # Set up a loop to control the robot's joints
    while True:
        # Get the current joint positions
        joint_states = pybullet.getJointStates(robot_id, joint_indices,2)
        joint_positions = [joint_states[i][0] for i in range(len(joint_indices))]

        # Compute the joint torques required to achieve the target positions
        joint_errors = [target_positions[i] - joint_positions[i] for i in range(len(joint_indices))]
        joint_torques = [max_torque * joint_errors[i] for i in range(len(joint_indices))]
        
        # Step the simulation
        pgui.stepSimulation()

        # Sleep to control the simulation rate
        time.sleep(1./240)


def IK_control(pgui, robot_id, box_id, plane_id, joint_indices, target_positions):
    
    # Set up the controller for the first three joints
    joint_positions = [0.0, 0.0, 0.0]  # initial joint positions
    joint_velocities = [0.0, 0.0, 0.0]  # initial joint velocities
    joint_torques = [0.0, 0.0, 0.0]  # initial joint torques

    # limits
    lower_limits = [-1.0 -1.0 -1.0, 0.0, 0.0, 0.0, 0.0]
    upper_limits = [-1.0 -1.0 -1.0, 0.0, 0.0, 0.0, 0.0]
    joint_ranges = [-1.0 -1.0 -1.0, 0.0, 0.0, 0.0, 0.0]
    rest_poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    orn = [0,0,0,1]#pgui.getQuaternionFromEuler([0, -math.pi, 0])

    # Set the position control mode for the joints
    control_mode = pybullet.POSITION_CONTROL

    # Set the maximum joint torque for the robot (in Nm)
    max_torque = 500

    # target_positions_joints = pybullet.calculateInverseKinematics(robot_id, 6, [0.1,0.1,0.2], lowerLimits=lower_limits, upperLimits = upper_limits, jointRanges = joint_ranges, restPoses=rest_poses, physicsClientId=2)
    target_positions_joints = pybullet.calculateInverseKinematics(robot_id, 8, targetPosition=target_positions, physicsClientId=2)
    target_positions_joints = target_positions_joints[:9]
    # Apply the joint torques to the robot
    pgui.setJointMotorControlArray(robot_id, joint_indices, control_mode, targetPositions=target_positions_joints)

    # Simulation time
    t = 0
    # Create Measurement class
    measurements = Measurements(pgui, robot_id, box_id, plane_id)
    
    while True:
        # Get the current joint positions
        joint_states = pybullet.getJointStates(robot_id, joint_indices,2)
        joint_positions = [joint_states[i][0] for i in range(len(joint_indices))]

        # Compute the joint torques required to achieve the target positions
        joint_errors = [target_positions_joints[i] - joint_positions[i] for i in range(len(joint_indices))]
        joint_torques = [max_torque * joint_errors[i] for i in range(len(joint_indices))]
        
        # target_positions_joints = pybullet.calculateInverseKinematics(robot_id, 6, target_positions, orn, physicsClientId=2)
        # target_positions_joints = target_positions_joints[:6]
        pgui.setJointMotorControlArray(robot_id, joint_indices, control_mode, targetPositions=target_positions_joints)

        # Step the simulation
        pgui.stepSimulation()


        measurements.getMeasurements(target_positions, t)
        measurements.getCenterOfMass()
        
        # Sleep to control the simulation rate
        time.sleep(config["timestep"])
        t+=config["timestep"]

def testController(pgui, robot_id, joint_indices, client_id):
    endEffectorLinkIndex=8
    controller = ForceAngleController(time.time(), joint_indices, client_id, robot_id, endEffectorLinkIndex, True)
    controller.generateTrajectory(time.time(), None,None)
    prevPos = [0.,0.,0.]
    prevPredictedPose = [0.,0.,0.]
    pos,_,_,_,_,_ = pybullet.getLinkState(robot_id, endEffectorLinkIndex, physicsClientId=2)
    print(f"Inital end effector pos: {pos}")
    while True:
        nextPose = controller.getNextJointPoses()
        # print("Next Pose")
        # print(nextPose)
        control_mode = pybullet.POSITION_CONTROL
        pgui.setJointMotorControlArray(robot_id, joint_indices, control_mode, targetPositions=nextPose)
        # res = pybullet.getJointStates(robot_id, [0,1,2,3,4,5,6,7,8], 2)
        # currentPose = [r[0] for r in res]
        # print("current pose")
        # print(currentPose)
        pgui.stepSimulation()
        time.sleep(0.1)
        pos,_,_,_,_,_ = pybullet.getLinkState(robot_id, endEffectorLinkIndex, physicsClientId=2)
        pgui.addUserDebugLine(prevPos, pos, [1, 0, 0], 1, 10,physicsClientId=2)
        prevPos = pos
        predictedPos = controller._nextPosition()
        pgui.addUserDebugLine(prevPredictedPose, predictedPos, [0,0,1], 1, 10,physicsClientId=2)
        prevPredictedPose = predictedPos
        # print(pos)


if __name__ == "__main__":
    pgui, robot_id, box_id, plane_id = load_environment()
    
    ################ FORWARD KIMENATICS CONTROL ################
    # joint_indices = [0, 1, 2]  # indices of the first three joints
    # # Set the target joint positions
    # target_positions = [0.2, -0.3, 1.4]  # target positions for the first three joints
    # FK_control(pgui, robot_id, joint_indices, target_positions)

    ################ INVERSE KINEMATICS CONTROL #################
    ################ IN CONSTUCITON - DOES NOT WORK #############
    joint_indices = range(9)  # indices of the first three joints
    # Set the target joint positions
    target_positions = [0.2, 0.2, 1.3]  # target positions for the first three joints
    # IK_control(pgui, robot_id, box_id, plane_id, joint_indices, target_positions)


    joint_indices = list(range(9))
    print(f"Number of joints : {pybullet.getNumJoints(robot_id, 2)}")
    testController(pgui, robot_id, joint_indices, 2)
