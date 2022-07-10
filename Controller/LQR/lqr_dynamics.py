import numpy as np

from lqr_constants import *

# dynamics matrix
m_A = np.array([[0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0.5, 0, 0, 0, 0, 0],
                [0, 0.5, 0, 0, 0, 0],
                [0, 0, 0.5, 0, 0, 0]])

# control matrix
m_I_inv = np.linalg.inv(m_I)
# control matrix
m_B = np.concatenate((m_I_inv, np.zeros(3, 3)))


def linear_dynamics(x, u):
    return m_A.dot(x) + m_B.dot(u)


def nonlinear_dynamics(x, u):
    q = x[0:3]
    w = x[3:6]
    q_0 = (1 - np.linalg.norm(q)) ** 0.5
    q_dot = -0.5 * np.cross(w, q) + 0.5 * q_0 * w
    # pi_dot is rate of change of angular momentum
    pi_dot = -1 * np.cross(w, m_I.dot(w)) + u
    w_dot = m_I_inv.dot(pi_dot)
    return np.concatenate((q_dot, w_dot))
