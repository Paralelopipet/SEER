import math

class Measurements:
    def __init__(self, pgui, robot_id, box_id, plane_id):
        self.pgui = pgui
        self.robot_id = robot_id
        self.box_id = box_id
        self.plane_id = plane_id
        self.arm_mass = self.calculateArmMass()
        self.total_mass = self.calculateTotalMass()

    def calculateTotalMass(self):
        total_mass = 0
        for link_idx in range(self.pgui.getNumJoints(self.robot_id)):
            link_mass = self.pgui.getDynamicsInfo(self.robot_id, link_idx, physicsClientId=2)[0]
            total_mass += link_mass
        box_mass = self.pgui.getDynamicsInfo(self.box_id,-1,physicsClientId=2)[0]
        total_mass = total_mass + box_mass
        return total_mass
    def calculateArmMass(self):
        total_mass = 0
        for link_idx in range(self.pgui.getNumJoints(self.robot_id)):
            link_mass = self.pgui.getDynamicsInfo(self.robot_id, link_idx, physicsClientId=2)[0]
            total_mass += link_mass
        return total_mass

    def drawPoint(self, pos, color):
        x_val = [0.01, 0.00, 0.00]
        y_val = [0.00, 0.01, 0.00]
        pointA = [x[0] + x[1] for x in zip(pos, x_val)]
        pointB = [x[0] - x[1] for x in zip(pos, x_val)]
        pointC = [x[0] + x[1] for x in zip(pos, y_val)]
        pointD = [x[0] - x[1] for x in zip(pos, y_val)]
        self.pgui.addUserDebugLine(pointA, pointB, color, 1, 5, physicsClientId=2)
        self.pgui.addUserDebugLine(pointC, pointD, color, 1, 5, physicsClientId=2)

    def getMeasurements(self, target_positions, t):
        
        # Get contact points
        points = self.pgui.getContactPoints(self.box_id, self.plane_id, physicsClientId=2)

        # Get

        # Draw Contact Points
        for cp in points:
            cp_pos = list(cp[5])
            add_val = [0.01, 0.01, 0.0]
            self.pgui.addUserDebugLine([sum(x) for x in zip(cp_pos, add_val)], [x[0]-x[1] for x in zip(cp_pos, add_val)], [1, 0, 0], 1, 0.1)

        # initiate line for drawings
        link_com = self.pgui.getLinkState(self.robot_id, 8, physicsClientId=2)[0]
        pos = [sum(x) for x in zip([0.02 * math.cos(t), 0, 0. + 0.02 * math.sin(t)], target_positions)]  
        self.pgui.addUserDebugLine(link_com, pos, [0,1,0],1,1)
            
    def getCenterOfMass(self):
        # Calculate the center of mass of the robot_id
        com_position = [0, 0, 0]
        for link_idx in range(self.pgui.getNumJoints(self.robot_id)):
            link_mass = list(self.pgui.getDynamicsInfo(self.robot_id, link_idx, physicsClientId=2))[0]
            link_com = self.pgui.getLinkState(self.robot_id, link_idx, physicsClientId=2)[0]
            link_position = [0,0,0]
            link_com_position = [link_com[i] + link_position[i] for i in range(3)]
            link_com_offset = [(link_mass/self.total_mass) * link_com_position[i] for i in range(3)]
            com_position = [com_position[i] + link_com_offset[i] for i in range(3)]

        #draw com_position
        self.drawPoint(com_position, [0,0,1])

                
        # Calculate the center of mass of the robot_id
        # com_position = [0, 0, 0]
        # print("Total mass: ", self.total_mass)
        # for link_idx in range(self.pgui.getNumJoints(self.robot_id)):
        #     print("Link mass: ",list(self.pgui.getDynamicsInfo(self.robot_id, link_idx, physicsClientId=2))[0])
        #     link_com = self.pgui.getLinkState(self.robot_id, link_idx, physicsClientId=2)[0]
        #     self.drawPoint(link_com, [0,0,1])


        return com_position