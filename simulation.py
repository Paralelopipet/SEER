import math
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet
import pybullet_data
import time
from enum import Enum

class ControlType(Enum):
    FORWARD_KINEMATICS = 1
    INVERSE_KINEMATICS = 2
    TORQUE_CONTROL = 3

config = {
    "box_scale" :   0.5,
    "box_fixed" :   1, # 0 if free, 1 if fixed
    "timestep" :    1/10.
}


def load_environment():
    p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
    p0.setAdditionalSearchPath(pybullet_data.getDataPath())

    p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
    p1.setAdditionalSearchPath(pybullet_data.getDataPath())

    #can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

    box = p1.loadURDF("cube.urdf", useFixedBase=config["box_fixed"], globalScaling=config["box_scale"])
    panda = p0.loadURDF("franka_panda/panda.urdf")
    # panda = p0.loadURDF("kuka_iiwa/model.urdf")

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
    return pgui, panda

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

def IK_control(pgui, robot_id, joint_indices, target_positions):
    
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
    # Set up a loop to control the robot's joints
    prevPos = [0.0,0.0,0.0]
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

        # initiate line for drawings
        pos = [sum(x) for x in zip([0.02 * math.cos(t), 0, 0. + 0.02 * math.sin(t)], target_positions)]  
        # Sleep to control the simulation rate
        time.sleep(config["timestep"])
        t+=config["timestep"]
        pgui.addUserDebugLine(prevPos, pos, [1, 0, 0], 1, 10)
        prevPos = pos
        


if __name__ == "__main__":
    pgui, robot_id = load_environment()
    
    ################ FORWARD KIMENATICS CONTROL ################
    joint_indices = [0, 1, 2]  # indices of the first three joints
    # Set the target joint positions
    target_positions = [0.2, -0.3, 1.4]  # target positions for the first three joints
    FK_control(pgui, robot_id, joint_indices, target_positions)

    ################ INVERSE KINEMATICS CONTROL #################
    ################ IN CONSTUCITON - DOES NOT WORK #############
    # joint_indices = range(9)  # indices of the first three joints
    # # Set the target joint positions
    # target_positions = [0.2, 0.2, 1.3]  # target positions for the first three joints
    # IK_control(pgui, robot_id, joint_indices, target_positions)
