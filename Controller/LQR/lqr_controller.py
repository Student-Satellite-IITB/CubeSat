
from lqr_constants import *
from scipy.linalg import sqrtm

m_Q1 = np.diag((1, 1, 1))
m_Q2 = np.diag((1, 1, 1))
m_Q = np.diag(np.concatenate((m_Q1, m_Q2)))
scal_R = 1
m_R = scal_R * m_Q2 * m_I
m_F11 = np.dot(m_I, sqrtm(m_R), sqrtm(m_Q2)) + np.dot(sqrtm(m_Q2), sqrtm(m_R), m_I)
m_F11 = m_Q1 + 0.5 * m_F11
m_F11 = np.dot(m_I, sqrtm(m_R), sqrtm(m_F11))
