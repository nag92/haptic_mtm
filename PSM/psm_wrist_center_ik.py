import numpy as np
from math import atan2, tan, sqrt, pow, asin
from numpy import sin, cos
import sympy as sym


def InverseKinematics(goal):
    theta1 = atan2(-goal[0], -goal[2])
    # if theta1 < 0:
    #     theta1 += np.pi
    theta2 = atan2(-goal[1], -goal[2])
    # if theta2 < 0:
    #     theta2 += np.pi
    d = sqrt(pow(goal[1], 2) + pow(goal[2], 2) + pow(goal[0], 2)) - 0.4318
    # d = sqrt(pow(goal[1], 2) + pow(goal[2] - 0.4318, 2) + pow(goal[0], 2))

    return theta1, theta2, d


def IK_orientation(goal_R, F):
    Rot = goal_R
    R03 = np.matmul(np.matmul(F[0][0:3, 0:3], F[1][0:3, 0:3]), F[2][0:3, 0:3])
    # print("R03 and R03 transpose is ", R03 , R03.transpose())
    # R36 = np.matmul(R03.transpose(), Rot)
    # theta4 = atan2(R36[2, 2], R36[0, 2])
    R36 = np.matmul(R03.transpose(), Rot)
    # print(R34)
    theta4 = atan2(-R36[1, 2], -R36[0, 2])
    # print("theta4 is ", theta4)
    # if theta4 < 0:
    #     theta4 += np.pi
    theta5 = atan2(-(sqrt(pow(R36[0, 2], 2) + pow(R36[1, 2], 2))), R36[2, 2])
    if theta5 < 0:
        theta5 += np.pi/2
    theta6 = atan2(R36[2, 0], R36[2, 1])
    if theta6 < 0:
        theta6 += np.pi/2

    return R36, theta4, theta5, theta6

"""
R =
 
[ cos(theta4)*sin(theta5)*sin(theta6) - 1.0*cos(theta6)*sin(theta4) + 6.1e-17*cos(theta5)*sin(theta4)*sin(theta6) + 3.7e-33*cos(theta6)*sin(theta4)*sin(theta5) - 6.1e-17*cos(theta4)*cos(theta5)*cos(theta6),     sin(theta4)*sin(theta6) - 3.7e-33*sin(theta4)*sin(theta5)*sin(theta6) + 6.1e-17*cos(theta4)*cos(theta5)*sin(theta6) + cos(theta4)*cos(theta6)*sin(theta5) + 6.1e-17*cos(theta5)*cos(theta6)*sin(theta4), cos(theta4)*cos(theta5) - 6.1e-17*sin(theta4) - 6.1e-17*sin(theta4)*sin(theta5), 4.5e-3*sin(theta4 + theta5) - 4.5e-3*sin(theta4 - 1.0*theta5)]
[     cos(theta4)*cos(theta6) + sin(theta4)*sin(theta5)*sin(theta6) - 6.1e-17*cos(theta4)*cos(theta5)*sin(theta6) - 3.7e-33*cos(theta4)*cos(theta6)*sin(theta5) - 6.1e-17*cos(theta5)*cos(theta6)*sin(theta4), 3.7e-33*cos(theta4)*sin(theta5)*sin(theta6) - 1.0*cos(theta4)*sin(theta6) + 6.1e-17*cos(theta5)*sin(theta4)*sin(theta6) + cos(theta6)*sin(theta4)*sin(theta5) - 6.1e-17*cos(theta4)*cos(theta5)*cos(theta6), 6.1e-17*cos(theta4) + 6.1e-17*cos(theta4)*sin(theta5) + cos(theta5)*sin(theta4), 4.5e-3*cos(theta4 - 1.0*theta5) - 4.5e-3*cos(theta4 + theta5)]
[                                                                                                                             6.1e-17*cos(theta6) + cos(theta5)*sin(theta6) + 6.1e-17*cos(theta6)*sin(theta5),                                                                                                                             cos(theta5)*cos(theta6) - 6.1e-17*sin(theta6) - 6.1e-17*sin(theta5)*sin(theta6),                                                       3.7e-33 - 1.0*sin(theta5),                                     9.1e-3*cos(theta5) + 0.42]
[                                                                                                                                                                                                           0,                                                                                                                                                                                                           0,                                                                               0,                                                           1.0
"""

def make_frame(a, alpha, D, theta):

    return np.array([[np.cos(theta), -np.sin(theta), 0, a],
                    [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -D*np.sin(alpha)],
                    [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), D*np.cos(alpha)],
                    [0, 0, 0, 1]])


def ForwardKinematics(q):

    pi = np.pi
    a = [0, 0, 0, 0, 0, 0.0091, 0]
    alpha = [0.5*pi, -0.5*pi, 0.5*pi, 0, -0.5*pi, -0.5*pi, -0.5*pi]
    D = [0, 0, q[2] - 0.4318,  0.4162, 0, 0, 0.0102]
    theta = [q[0] + 0.5*pi, q[1] - 0.5*pi, 0, q[3], q[4] - 0.5*pi, q[5] - 0.5*pi, 0]
    F = []
    lastFrame = np.eye(4)

    for ii in range(len(a)):

        frame = make_frame(a[ii], alpha[ii], D[ii], theta[ii])
        temp = np.matmul(lastFrame, frame)
        F.append(temp)
        lastFrame = temp

    return F


if __name__ == '__main__':

    q = np.array([0, 0, 0, 0, 0., 0.])
    F = ForwardKinematics(q)
    F06 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(F[0], F[1]), F[2]), F[3]), F[4]), F[5])
    print("FK is ", F06)
    goal_R = np.array(F06[0:3, 0:3])
    goal_p = np.array(F06[0:3, 3])
    goal_point = goal_p + (0.4318-0.4162+0.0091)*goal_R[2]

    theta1, theta2, d = InverseKinematics(goal_point)2
    wrist_rot_matrix, theta4, theta5, theta6 = IK_orientation(goal_R, F)
    calc_F = ForwardKinematics([theta1, theta2, d, theta4, theta5, theta6])
    calc_F06 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(calc_F[0], calc_F[1]), calc_F[2]), calc_F[3]), calc_F[4]), calc_F[5])
    print("calculated FK ", calc_F06)
    print("theta 12d456 value ", theta1, theta2, d, theta4, theta5, theta6)

    """
        q = np.array([0, 0, 0, 0, 0., 0., 0])
    # print("q is ", q)
    F = ForwardKinematics(q)
    F06 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(F[0], F[1]), F[2]), F[3]), F[4]), F[5])
    # F05 = np.matmul(np.matmul(np.matmul(np.matmul(F[0], F[1]), F[2]), F[3]), F[4])
    # F04 = np.matmul(np.matmul(np.matmul(F[0], F[1]), F[2]), F[3])
    # F03 = np.matmul(np.matmul(F[0], F[1]), F[2])
    print("FK is ", F06)
    goal_R = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
    # goal_R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    # goal_R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    part_goal_p = np.array([-0.0065, 0, -0.4318])
    # part_goal_p = np.array([0, 0, -0.4162])
    part_goal_point = part_goal_p + (0.4318-0.4162)*goal_R[2]
    # goal_point = np.array([0, 0, -0.0037])
    # goal_R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    # goal_p = goal_point - (0.0102+0.0091)*goal_R[2]
    # # print(goal_p, goal_R[2])
    # print(goal)
    # print("last frame is", F[-1])
    theta1, theta2, d = InverseKinematics(part_goal_point)
    wrist_rot_matrix, theta4, theta5, theta6 = IK_orientation(goal_R, F)
    calc_F = ForwardKinematics([theta1, theta2, d, theta4, 0, 0])
    calc_F06 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(calc_F[0], calc_F[1]), calc_F[2]), calc_F[3]), calc_F[4]), calc_F[5])
    # unit_mat = np.matmul(calc_F04[0:3, 0:3], F04[0:3, 0:3])
    # print("unit mat", unit_mat)
    print("calculated FK ", calc_F06)
    print("theta 12d45 value ", theta1, theta2, d, theta4, theta5, theta6)
    """
