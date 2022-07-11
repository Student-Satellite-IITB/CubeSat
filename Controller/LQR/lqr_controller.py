from scipy.linalg import sqrtm

from lqr_constants import *
from lqr_dynamics import m_B

m_Q1 = np.diag((1, 1, 1))
m_Q2 = np.diag((1, 1, 1))
m_Q = np.diag(np.concatenate((m_Q1, m_Q2)))
scal_R = 1
m_R = scal_R * m_Q2 * m_I
m_F11 = np.dot(m_I, sqrtm(m_R), sqrtm(m_Q2)) + np.dot(sqrtm(m_Q2), sqrtm(m_R), m_I)
m_F11 = m_Q1 + 0.5 * m_F11
m_F11 = np.dot(m_I, sqrtm(m_R), sqrtm(m_F11))
m_F12 = np.dot(m_I, sqrtm(m_R), sqrtm(m_Q2))
m_F22 = m_Q1 + np.dot(m_I, sqrtm(R), sqrtm(m_Q2))
m_F22 = 2 * np.dot(sqrtm(m_Q2), sqrtm(m_F22))
F11_F12 = np.concatenate((m_F11, m_F12), axis=1)
F12_F22 = np.concatenate((m_F12.T, m_F22), axis=1)
m_F = np.concatenate((F11_F12, F12_F22))


def control_law(x):
    gain = np.dot(np.linalg.inv(m_R), m_B.T, m_F)
    return -1 * np.dot(gain, x)
