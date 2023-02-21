import pybullet as p
import pybullet_data
import time

#connect physics client

physicsClient = p.connect(p.GUI)

# Load the UR3 robot model
urdf_path = pybullet_data.getDataPath() + "/franka_panda/panda.urdf"
plane_path = pybullet_data.getDataPath() + "/plane.urdf"

plane_id = p.loadURDF(plane_path)
robot_id = p.loadURDF(urdf_path, useFixedBase=1)

# get joint positions
joint_positions = [j[0] for j in p.getJointStates(robot_id, range(7))]
print(joint_positions)

# set gravity 
p.setGravity(0.0, 0.0, -9.81)

p.setRealTimeSimulation(0)
p.setJointMotorControlArray(robot_id, range(7), p.POSITION_CONTROL, targetPositions=[0.2]*7)

for _ in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
input()