import numpy as np


# Inertia matrix of satellite
m_I = np.array([[10, 0, 0], [0, 20, 0], [0, 0, 40]])
m_I_inv = np.linalg.inv(m_I)

# dynamics matrix
m_A = np.array([[0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0.5, 0, 0, 0, 0, 0],
                [0, 0.5, 0, 0, 0, 0],
                [0, 0, 0.5, 0, 0, 0]])

# control matrix
m_B = np.concatenate((m_I_inv, np.zeros((3, 3))))

m_Q1 = np.diag((1, 1, 1))
m_Q2 = np.diag((1, 1, 1))
scal_R = 0.0001
