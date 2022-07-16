import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import lqr_controller
import lqr_dynamics
import solver_rk4method

# initial conditions
initial_ori_euler = Rot.from_euler('xyz', [10, 30, 20], degrees=True)
q_BI = initial_ori_euler.as_quat()[0:3]
w_BIB = np.array([0.1, 0.01, 0.2])
x_init = np.concatenate((q_BI, w_BIB))
time = np.linspace(1, 30, 30000)
sol = solver_rk4method.rk4method(lqr_dynamics.nonlinear_dynamics, x_init, time)
# st_u = lqr_controller.control_law(sol.T).T
K = lqr_controller.initialize_gain()
st_u = K.dot(sol.T).T
q0_BI = (1 - sol[:, 0] ** 2 - sol[:, 1] ** 2 - sol[:, 2] ** 2) ** 0.5
euler_angles = np.zeros([sol.shape[0], 3])
for i in range(sol.shape[0]):
    rotation = Rot.from_quat([sol[i][0], sol[i][1], sol[i][2], q0_BI[i]])
    euler_angles[i] = rotation.as_euler('xyz', degrees=True)


plot4 = plt.figure(4)
plt.plot(time, st_u[:, 0])
plt.plot(time, st_u[:, 1])
plt.plot(time, st_u[:, 2])
plt.title('Control Torque')
plt.ylabel('torque')
plt.xlabel('time')
plt.legend(['x', 'y', 'z'])

plot3 = plt.figure(3)
plt.plot(time, sol[:, 3])
plt.plot(time, sol[:, 4])
plt.plot(time, sol[:, 5])
plt.title('Angular Velocity')
plt.ylabel('angular velocity')
plt.xlabel('time')
plt.legend(['x', 'y', 'z'])

plot2 = plt.figure(2)
plt.plot(time, euler_angles[:, 0])
plt.plot(time, euler_angles[:, 1])
plt.plot(time, euler_angles[:, 2])
plt.title('euler angles in xyz form')
plt.ylabel('degrees')
plt.xlabel('time')
plt.legend(['x', 'y', 'z'])

plot1 = plt.figure(1)
plt.plot(time, sol[:, 0])
plt.plot(time, sol[:, 1])
plt.plot(time, sol[:, 2])
# plt.plot(time, q0_BI)
plt.title('Quaternion')
plt.ylabel('quaternion')
plt.xlabel('time')
plt.legend(['q1', 'q2', 'q3'])

plt.show()
print(euler_angles)



