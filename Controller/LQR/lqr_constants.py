import numpy as np


# Inertia matrix of satellite
m_I = np.array([[10, 0, 0], [0, 20, 0], [0, 0, 40]])

# dynamics matrix
m_A = np.array([[0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0.5, 0, 0, 0, 0, 0],
                [0, 0.5, 0, 0, 0, 0],
                [0, 0, 0.5, 0, 0, 0]])
m_I_inv = np.linalg.inv(m_I)

# control matrix
m_B = np.concatenate((m_I_inv, np.zeros((3, 3))))
