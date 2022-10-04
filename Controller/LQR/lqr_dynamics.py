import numpy as np
import lqr_controller
import lqr_constants


def nonlinear_dynamics(t, x):
    q = x[0:3]
    w = x[3:6]
    q_0 = (1 - np.linalg.norm(q)) ** 0.5
    q_dot = -0.5 * np.cross(w, q) + 0.5 * q_0 * w
    # pi_dot is rate of change of angular momentum
    m = lqr_controller.control_law(x, t)
    u = np.cross(m, lqr_constants.v_B(t))
    pi_dot = -1 * np.cross(w, lqr_constants.m_I.dot(w)) + u
    w_dot = lqr_constants.m_I_inv.dot(pi_dot)
    return np.concatenate((q_dot, w_dot))
