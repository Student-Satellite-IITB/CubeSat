import numpy as np

# Inertia matrix of satellite
m_I = np.array([[10, 0, 0], [0, 20, 0], [0, 0, 40]])
m_I_inv = np.linalg.inv(m_I)

# dynamics matrix
sigma_x = (m_I[1][1] - m_I[2][2]) / m_I[0][0]
sigma_y = (m_I[2][2] - m_I[0][0]) / m_I[1][1]
sigma_z = (m_I[0][0] - m_I[1][1]) / m_I[2][2]
v_orbit_ang_vel = 0.7
m_A = np.array([[0, 0, -1 * sigma_x * v_orbit_ang_vel, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [-1 * sigma_z * v_orbit_ang_vel, 0, 0, 0, 0, 0],
                [0.5, 0, 0, 0, 0, v_orbit_ang_vel],
                [0, 0.5, 0, 0, 0, 0],
                [0, 0, 0.5, -1 * v_orbit_ang_vel, 0, 0]])

# control matrix when torque is taken as input
m_B_tiv = np.concatenate((m_I_inv, np.zeros((3, 3))))


m_Q1 = np.diag((1, 1, 1))
m_Q2 = np.diag((1, 1, 1))
scal_R = 0.001
m_Q = np.eye(6)
m_R = np.eye(3)

# some constants used for defining magnetic field
a = 8413 * 1000
mew_f = 7.9 * (10 ** 15)
i_m = np.pi / 4


def v_B(t):
    b1 = np.cos(v_orbit_ang_vel * t) * np.sin(i_m)
    b2 = -1 * np.cos(i_m)
    b3 = 2 * np.sin(v_orbit_ang_vel * t) * np.sin(i_m)
    return np.array([b1, b2, b3]) * (mew_f / a ** 3)
