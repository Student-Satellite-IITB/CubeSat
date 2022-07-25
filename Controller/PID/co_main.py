# Author: Ajay Tak
# Date: 28-06-2022

import numpy as np
import matplotlib.pyplot as plt
from co_propagating_w import *
from co_propagating_q import *
from co_propagate_w_rw import *
from co_control_variable import *
from co_quaternion_to_euler import *
from co_sort import *
import co_constants

omega_n = float(input("enter the value for omega_n: "))
zeta = float(input("enter the value for zeta: "))
T  = float(input("enter the value for T: "))
plot_graphs = input("please enter 1 for Yes, and 0 for no only: ")

w = np.array([0.008726, 0.008726, 0.008726]) # initial angular velocity of the satellite (rad/s)
W = np.array([0, 0, 0]) # initial angular velocity of the reaction wheels (rad/s)
T = np.array([0, 0, 0]) # initial control variable/armature currents (A)
q = np.array([-0.3061862, 0.4355957, -0.6597396, 0.5303301]) # initial attitude
q_command = np.array([1, 0, 0, 0])
w_command = np.array([0, 0, 0])
w_count = []
W_count = []
q_count = []
sigma_integrate = [0.001, 0.001 , 0.001]

for i in range(300):
    w_count.append(w)
    W_count.append(W)
    q_count.append(q)

    T = co_armature_current(q, q_command, w, w_command, co_constants.H, omega_n, zeta, T, [co_constants.I_1, co_constants.I_2, co_constants.I_3], co_constants.KR, sigma_integrate)
    w_new = co_propagating_w_i(w[0], w[1], w[2], co_constants.I_1, co_constants.I_2, co_constants.I_3, T[0], T[1], T[2], W[0], W[1], W[2], co_constants.J_PER, co_constants.J_PAR, co_constants.H)
    W_new = np.array([co_propagating_w_rw_i(co_constants.J_PAR, T[0], W[0], w_new[0], w[0], co_constants.H),
                      co_propagating_w_rw_i(co_constants.J_PAR, T[1], W[1], w_new[1], w[1], co_constants.H),
                      co_propagating_w_rw_i(co_constants.J_PAR, T[2], W[2], w_new[2], w[2], co_constants.H)])
    q_new = co_propagatin_q_IB(q[0], q[1], q[2], q[3], w[0], w[1], w[2], co_constants.H)

    sigma_integrate = sigma_integrate + (co_constants.H*co_state(q, q_command))
    w = w_new
    W = W_new
    q = q_new

if plot_graphs:
    t = np.arange(0, 30, 0.1)
    L = len(q_count)
    euler_plot = []

    for i in range(L):
        euler_plot.append(co_quat_to_euler(q_count[i]))
    
    [w_x_plot, w_y_plot, w_z_plot] = co_sorting(w_count)
    [W_x_plot, W_y_plot, W_z_plot] = co_sorting(W_count)
    [yaw_plot, pitch_plot, roll_plot] = co_sorting(euler_plot)
    plt.figure(1)
    plt.subplot(3, 3, 1)
    plt.xlabel("time")
    plt.ylabel("w_x")
    plt.plot(t, w_x_plot)
    plt.subplot(3, 3, 2)
    plt.xlabel("time")
    plt.ylabel("w_y")
    plt.plot(t, w_y_plot)
    plt.subplot(3, 3, 3)
    plt.xlabel("time")
    plt.ylabel("w_z")
    plt.plot(t, w_z_plot)
    plt.subplot(3, 3, 4)
    plt.xlabel("time")
    plt.ylabel("W_x")
    plt.plot(t, W_x_plot)
    plt.subplot(3, 3, 5)
    plt.xlabel("time")
    plt.ylabel("W_y")
    plt.plot(t, W_y_plot)
    plt.subplot(3, 3, 6)
    plt.xlabel("time")
    plt.ylabel("W_z")
    plt.plot(t, W_z_plot)
    plt.subplot(3, 3, 7)
    plt.xlabel("time")
    plt.ylabel("roll")
    plt.plot(t, roll_plot)
    plt.subplot(3, 3, 8)
    plt.xlabel("time")
    plt.ylabel("pitch")
    plt.plot(t, pitch_plot)
    plt.subplot(3, 3, 9)
    plt.xlabel("time")
    plt.ylabel("yaw")
    plt.plot(t, yaw_plot)
    plt.show()
    print([roll_plot[-1], pitch_plot[-1], yaw_plot[-1]])