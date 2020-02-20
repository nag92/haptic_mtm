
import numpy as np
from math import cos, sin
from scipy.optimize import lsq_linear

import copy

def lse(A, b, B, d):
    """
    Equality-contrained least squares.
    The following algorithm minimizes ||Ax - b|| subject to the
    constrain Bx = d.
    Parameters
    ----------
    A : array-like, shape=[m, n]
    B : array-like, shape=[p, n]
    b : array-like, shape=[m]
    d : array-like, shape=[p]
    Reference
    ---------
    Matrix Computations, Golub & van Loan, algorithm 12.1.2
    Examples
    --------
    # >>> A = np.array([[0, 1], [2, 3], [3, 4.5]])
    # >>> b = np.array([1, 1])
    # >>> # equality constrain: ||x|| = 1.
    # >>> B = np.ones((1, 3))
    # >>> d = np.ones(1)
    # >>> lse(A.T, b, B, d)
    array([-0.5,  3.5, -2. ])
    """
    from scipy import linalg
    if not hasattr(linalg, 'solve_triangular'):
        # compatibility for old scipy
        solve_triangular = linalg.solve
    else:
        solve_triangular = linalg.solve_triangular
    A, b, B, d = map(np.asanyarray, (A, b, B, d))
    p = B.shape[0]
    Q, R = linalg.qr(B.T)
    y = solve_triangular(R[:p, :p].T, d)
    A = np.dot(A, Q)
    z = linalg.lstsq(A[:, p:], b - np.dot(A[:, :p], y))[0].ravel()
    return np.dot(Q[:, :p], y) + np.dot(Q[:, p:], z)

def make_frame(a, alpha, D, theta):


    return np.array([ [ cos(theta), - sin(theta), 0, a],
                      [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -D*sin(alpha)],
                      [ sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), D*cos(alpha)],
                      [ 0,0,0, 1 ]])

def ForwardKinematics(q):

    pi = np.pi
    a = [ 0, 0, 0, 0, 0, 0.0091, 0  ]
    alpha = [ 0.5*pi, -0.5*pi, 0.5*pi,0, -0.5*pi, -0.5*pi, -0.5*pi ]
    D = [ 0, 0, q[2] - 0.4318,  0.4162, 0, 0, 0.0102 ]
    theta = [ q[0] + 0.5*pi, q[1] - 0.5*pi, 0, 0, q[3], q[4] - 0.5*pi, q[5] - 0.5*pi, 0]
    F = []

    lastFrame = np.eye(4)
    for ii in xrange(len(a)):

        frame = make_frame(a[ii], alpha[ii],D[ii], theta[ii] )
        temp = lastFrame.dot(frame)
        F.append(temp)
        lastFrame = temp

    return F

def jacob(q):

    pos = ForwardKinematics(q)
    final = pos[-1]
    J = np.zeros((6,7))

    for i in xrange(len(pos)):
        current = pos[i]
        O_prev = np.array([0,0,0])
        z_prev = np.array([0,0,1])
        if i is not 2:
            Jv = z_prev * np.cross(z_prev, final[-1, :3] - O_prev)
            J[:3, i] = Jv
            J[3:, i] = z_prev
            O_prev = current[-1, :3]
            z_prev = current[-2, :3].reshape((-1, 1))
        else:
            J[:3, i] = z_prev
            J[3:, i] = [0,0,0]

    return J



def InverseKinematics(q_, goal):

    tol = 0.001
    max_iter = 500
    step = 0
    e = 100
    q = copy.deepcopy(q_)

    while step < max_iter or abs(e) < tol:
        alpha = 0.01
        pos = ForwardKinematics(q)
        final = pos[-1]

        dx = goal[-1, 0:3] - final[-1, 0:3]

        dr = 0.5 * (final[0, 0:3] % goal[0, 0:3]) + (final[1, 0:3] % goal[1, 0:3]) + (final[2, 0:3] % goal[2, 0:3])

        e = np.array([dx[0], dx[1], dx[2], dr[0], dr[1], dr[2]]).reshape((-1,1))
        J = jacob(q)
        dq = np.dot(np.linalg.pinv(J), e)
        dq = alpha*np.linalg.norm(dq)
        q += dq

    return q
