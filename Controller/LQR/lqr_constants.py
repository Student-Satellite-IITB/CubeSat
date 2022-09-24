import numpy as np

# Inertia matrix of satellite
m_I = np.array([[10, 0, 0], [0, 20, 0], [0, 0, 40]])
m_I_inv = np.linalg.inv(m_I)

# dynamics matrix
sigma_x = (m_I[1][1] - m_I[2][2]) / m_I[0][0]
sigma_y = (m_I[2][2] - m_I[0][0]) / m_I[1][1]
sigma_z = (m_I[0][0] - m_I[1][1]) / m_I[2][2]
v_orbit_ang_vel = 0.07
m_A = np.array([[0, 0, -1 * sigma_x * v_orbit_ang_vel, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [-1 * sigma_z * v_orbit_ang_vel, 0, 0, 0, 0, 0],
                [0.5, 0, 0, 0, 0, v_orbit_ang_vel],
                [0, 0.5, 0, 0, 0, 0],
                [0, 0, 0.5, -1 * v_orbit_ang_vel, 0, 0]])


# control matrix
# m_B = np.concatenate((m_I_inv, np.zeros((3, 3))))
def m_B(t):
    v_b = v_B(t)
    bx = v_b[0]
    by = v_b[1]
    bz = v_b[2]
    b = np.linalg.norm(v_b)
    m_B1 = np.array([[-1 * (bx ** 2) - (by ** 2), bx * by, bx * bz],
                     [bx * by, -1 * (bz ** 2) - (bx ** 2), by * bz],
                     [bx * bz, by * bz, -1 * (bx ** 2) - (by ** 2)]])
    m_B1 = m_I_inv.dot(m_B1) / b
    m_B2 = np.zeros((3, 3))
    return np.concatenate((m_B1, m_B2))


m_Q1 = np.diag((1, 1, 1))
m_Q2 = np.diag((1, 1, 1))
scal_R = 0.0001
m_Q = np.eye(6)
m_R = np.eye(3)


# v_B = np.array([1, 0, 0])

def v_B(t):
    b1 = np.cos(t) * np.sin(np.pi / 4)
    b2 = -1 * np.cos(np.pi / 4)
    b3 = 2 * np.sin(t) * np.sin(np.pi / 4)
    return np.array([b1, b2, b3])
