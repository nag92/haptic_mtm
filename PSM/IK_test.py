import numpy as np
from math import atan2, tan, sqrt, pow

from scipy.optimize import fsolve

def make_frame(a, alpha, D, theta):

    return np.array([[np.cos(theta), -np.sin(theta), 0, a],
                    [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -D*np.sin(alpha)],
                    [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), D*np.cos(alpha)],
                    [0, 0, 0, 1]])


def ForwardKinematics(q):

    pi = np.pi
    a = [0, 0, 0]
    alpha = [0.5*pi, -0.5*pi, 0.5*pi]
    D = [0, 0, q[2] - 0.4318]
    theta = [q[0] + 0.5*pi, q[1] - 0.5*pi, 0]
    F = []

    lastFrame = np.eye(4)
    for ii in range(len(a)):

        frame = make_frame(a[ii], alpha[ii], D[ii], theta[ii])
        temp = lastFrame.dot(frame)
        F.append(temp)
        lastFrame = temp

    return F


if __name__ == '__main__':

    q = np.array([0.0, 0.5*3.14, 0.9])
    F = ForwardKinematics(q)
    # x, y, z = solver(goal)
    # zGuess = np.array([0.1, 0.1, 0.1])
    # z = fsolve(myFunction, zGuess)
    # print(z)
    goal = F[-1][:3, -1]
    #goal = np.array([-0.03, 0.03, 0.32])
    x = goal[0]
    y = goal[1]
    z = goal[2]

    print(x)
    print(z)
    q[0] = atan2(-x, z)
    q[1] = atan2(y, z)
    q[2] = sqrt(pow(y, 2) + pow(z-0.4318, 2) + pow(x, 2))
    print(q[0])
    print(q[1])
    print(q[2])

    # print("solution is ", x, y, z)