# Author: Ajay Tak
# Date: 26-06-2022

import numpy as np

def co_state(q_actual, q_command):

    """
    > q_actual: The quaternion that represents a rotation from current/actual body frame to the inertial frame.
    > q_command: The quaternion that represents a rotation from the desired orientation of the  body frame to the inertial frame.
    """

    M = np.matrix([[  q_command[0],  q_command[1],  q_command[2],  q_command[3]],
                    [-q_command[1],  q_command[0], -q_command[3],  q_command[2]],
                    [-q_command[2],  q_command[3],  q_command[0], -q_command[1]],
                    [-q_command[3], -q_command[2],  q_command[1],  q_command[0]]])
    q_e = np.dot(M, q_actual)
    sigma_r = (2*q_e[0, 0])*np.array([q_e[0, 1], q_e[0, 2], q_e[0, 3]])
    return sigma_r

def co_current(q_actual, q_command, w_actual, w_command, h, sigma_integrate_prev, Kp, Ki, Kd):

    """
    > q_actual: The quaternion that represents a rotation from current/actual body frame to the inertial frame.
    > q_command: The quaternion that represents a rotation from the desired orientation of the  body frame to the inertial frame.
    > w_actual: The current/actual angular velocity of the body frame of the cubesat w.r.t the inertial frame.
    > w_command: The required angular velocity of the body frame of cubesat w.r.t to inertial frame.
    > h: time step
    > Kp: The prortional gain matrix
    > Kd: The derivative gain matrix
    > Kd: The integrator gain matrix
    > sigma_integrate_prev: integral of sigma from 0 to prev step
    """

    u = np.array((np.dot(Kp, co_state(q_actual, q_command)))+(np.dot(Kd, w_command-w_actual))+(np.dot(Ki,sigma_integrate_prev + h*co_state(q_actual, q_command))))
    return u

    