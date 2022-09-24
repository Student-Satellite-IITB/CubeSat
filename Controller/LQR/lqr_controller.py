from scipy.linalg import sqrtm
from scipy.linalg import solve_continuous_are
import numpy as np
import lqr_constants


def initialize_gain(t):
    m_B = lqr_constants.m_B(t)
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


def control_law(x, t):
    B = lqr_constants.m_B(t)
    F = solve_continuous_are(lqr_constants.m_A, B, lqr_constants.m_Q, lqr_constants.m_R)
    gain = -1 * np.linalg.inv(lqr_constants.m_R).dot(B.T).dot(F)
    m_tilde = np.dot(gain, x)
    b = lqr_constants.v_B(t)
    m = np.cross(m_tilde, b) / np.linalg.norm(b)
    return m
