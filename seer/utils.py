from typing import List

def distance(source, target):
    assert len(source) == len(target)
    return (sum([(a-b)**2 for (a,b) in zip(source,target)]))**0.5

def pos(pgui, robot_id, endEffectorLinkIndex, physicsClientId):
    pos,_,_,_,_,_ = pgui.getLinkState(robot_id, endEffectorLinkIndex, physicsClientId=2)
    return pos

def formatVec(vec):
    return "(" + ', '.join([f'{v:.3f}' for v in vec]) + ")"

def transpose(matrix : List[List[float]]):
    height = len(matrix)
    width = len(matrix[0])
    tranpose = [[0. for _ in range(height)] for _ in range(width)]
    for i in range(height):
        for j in range(width):
            tranpose[j][i] = matrix[i][j]
    return tranpose


if __name__ == "__main__":
    matrix = [[1.,2.,3.,4.],[5.,6.,7.,8.]]
    print(transpose(matrix))