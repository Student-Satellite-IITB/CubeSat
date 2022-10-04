from scipy.linalg import sqrtm
from scipy.linalg import solve_continuous_are
import numpy as np
import lqr_constants


def initialize_gain():
    m_B = lqr_constants.m_B_tiv
    m_R = lqr_constants.scal_R * lqr_constants.m_Q2
    # m_R = np.eye(3)
    m_F11 = np.dot(lqr_constants.m_I, sqrtm(m_R)).dot(sqrtm(lqr_constants.m_Q2))
    m_F11 += np.dot(sqrtm(lqr_constants.m_Q2), sqrtm(m_R)).dot(lqr_constants.m_I)
    m_F11 = lqr_constants.m_Q1 + 0.5 * m_F11
    m_F11 = np.dot(lqr_constants.m_I, sqrtm(m_R), sqrtm(m_F11))
    m_F12 = np.dot(lqr_constants.m_I, sqrtm(m_R), sqrtm(lqr_constants.m_Q2))
    m_F22 = lqr_constants.m_Q1 + np.dot(lqr_constants.m_I, sqrtm(m_R), sqrtm(lqr_constants.m_Q2))
    m_F22 = 2 * np.dot(sqrtm(lqr_constants.m_Q2), sqrtm(m_F22))
    F11_F12 = np.concatenate((m_F11, m_F12), axis=1)
    F12_F22 = np.concatenate((m_F12.T, m_F22), axis=1)
    m_F = np.concatenate((F11_F12, F12_F22))
    K = -1 * np.dot(np.linalg.inv(m_R), m_B.T).dot(m_F)
    return K


K = initialize_gain()


def control_law(x, t):
    v_b = lqr_constants.v_B(t)
    u = np.dot(K, x)
    m = np.cross(v_b, u) / np.linalg.norm(v_b) ** 2
    return m
