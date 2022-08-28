# Author: Ajay Tak
# Date: 28-06-2022

import numpy as np
import matplotlib.pyplot as plt
from co_propagating_w import *
from co_propagating_q import *
from co_magnetorquers import *
from co_control_variable import *
from co_quaternion_to_euler import *
from co_sort import *
from co_frames import *
import co_constants

omega_n = float(input("enter the value for omega_n: "))
zeta = float(input("enter the value for zeta: "))
timec  = float(input("enter the value for time constant: "))
plot_graphs = input("please enter 1 for Yes, and 0 for no only: ")

w = np.array([0.008726, 0.008726, 0.008726]) # initial angular velocity of the satellite (rad/s)
s = np.array([0, 0, 0]) # initial control Torque (A)
q = np.array([-0.3061862, 0.4355957, -0.6597396, 0.5303301]) # initial attitude
q_command = np.array([1, 0, 0, 0])
w_command = np.array([0, 0, 0])
w_count = []
q_count = []
euler_plot = []
sigma_integrate = [0, 0 , 0]
K_p = np.dot(((omega_n**2)+((2*0.015*zeta*omega_n)/timec)), np.diag(np.array([co_constants.I_1, co_constants.I_2, co_constants.I_3])))
K_d = np.dot(((2*zeta*omega_n)+(0.015/timec)), np.diag(np.array([co_constants.I_1, co_constants.I_2, co_constants.I_3])))
K_i = np.dot(0.015*((omega_n**2)/timec), np.diag(np.array([co_constants.I_1, co_constants.I_2, co_constants.I_3])))
mag_field = np.genfromtxt('magfield.csv')


for i in range(270000):
    w_count.append(w)
    q_count.append(q)
    B_orbit = mag_field[i]
    euler_angles = co_quat_to_euler(q)
    euler_plot.append(euler_angles)
    B_body = orbit2body(B_orbit, euler_angles[2], euler_angles[1], euler_angles[0])
    i_control_variable = co_current(q, q_command, w, w_command, co_constants.H, sigma_integrate, K_p, K_i, K_d)*1e-2
    T = co_actuator_model(i_control_variable, B_body)
    w_new = co_propagating_w_i(w[0], w[1], w[2], co_constants.I_1, co_constants.I_2, co_constants.I_3, T[0], T[1], T[2], co_constants.H)
    q_new = co_propagatin_q_IB(q[0], q[1], q[2], q[3], w[0], w[1], w[2], co_constants.H)

    sigma_integrate = sigma_integrate + (co_constants.H*co_state(q, q_command))
    w = w_new
    q = q_new

if plot_graphs:
    t = np.arange(0, 27000, 0.1)   
    [w_x_plot, w_y_plot, w_z_plot] = co_sorting(w_count)
    [yaw_plot, pitch_plot, roll_plot] = co_sorting(euler_plot)
    plt.figure(1)
    plt.subplot(2, 3, 1)
    plt.xlabel("time")
    plt.ylabel("w_x")
    plt.plot(t, w_x_plot)
    plt.subplot(2, 3, 2)
    plt.xlabel("time")
    plt.ylabel("w_y")
    plt.plot(t, w_y_plot)
    plt.subplot(2, 3, 3)
    plt.xlabel("time")
    plt.ylabel("w_z")
    plt.plot(t, w_z_plot)
    plt.subplot(2, 3, 4)
    plt.xlabel("time")
    plt.ylabel("roll")
    plt.plot(t, roll_plot)
    plt.subplot(2, 3, 5)
    plt.xlabel("time")
    plt.ylabel("pitch")
    plt.plot(t, pitch_plot)
    plt.subplot(2, 3, 6)
    plt.xlabel("time")
    plt.ylabel("yaw")
    plt.plot(t, yaw_plot)
    plt.show()
    print([roll_plot[-1], pitch_plot[-1], yaw_plot[-1]])