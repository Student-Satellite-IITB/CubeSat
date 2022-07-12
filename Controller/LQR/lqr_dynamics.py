import numpy as np
from lqr_controller import control_law
from lqr_constants import *


def linear_dynamics(t, x):
    u = control_law(x)
    return m_A.dot(x) + m_B.dot(u)


def nonlinear_dynamics(t, x):
    q = x[0:3]
    w = x[3:6]
    q_0 = (1 - np.linalg.norm(q)) ** 0.5
    q_dot = -0.5 * np.cross(w, q) + 0.5 * q_0 * w
    # pi_dot is rate of change of angular momentum
    u = control_law(x)
    pi_dot = -1 * np.cross(w, m_I.dot(w)) + u
    w_dot = m_I_inv.dot(pi_dot)
    return np.concatenate((q_dot, w_dot))
