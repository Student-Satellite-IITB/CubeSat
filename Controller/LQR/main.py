import numpy as np
from scipy.spatial.transform import Rotation as R
from lqr_dynamics import *
from lqr_controller import *
from solver_rk4method import *

# initial conditions
initial_ori_euler = R.from_euler('xyz', [10, 30, 10], degrees=True)
q_BI = initial_ori_euler.as_quat()[0:3]
w_BIB = np.array([0, 0, 0])
x_init = np.concatenate((q_BI, w_BIB))
time = np.linspace(1, 10, 1000)
sol = rk4method(nonlinear_dynamics, x_init, time)
