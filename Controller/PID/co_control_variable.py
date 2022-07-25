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

def co_armature_current(q_actual, q_command, w_actual, w_command, h, omega_n, zeta, T, I, kr_i, sigma_integrate_prev):

    """
    > q_actual: The quaternion that represents a rotation from current/actual body frame to the inertial frame.
    > q_command: The quaternion that represents a rotation from the desired orientation of the  body frame to the inertial frame.
    > w_actual: The current/actual angular velocity of the body frame of the cubesat w.r.t the inertial frame.
    > w_command: The required angular velocity of the body frame of cubesat w.r.t to inertial frame.
    > h: time step
    > omega_n: It is the linear control bandwidth of the integral controller
    > zeta: It is the damping ratio of the integral controller
    > T: It is the time constant of the integral controller
    > I: It is the moment of inertial matrix of the cubesat
    > kr_i: It the motor torque constant of the i'th motor running the i'th reaction wheel
    """
    
    R_inv = np.matrix([[I[0]/kr_i, 0, 0],
                        [0, I[1]/kr_i, 0],
                        [0, 0, I[2]/kr_i]])
    K_pr = ((omega_n**2)+((2*zeta*omega_n)/T))*R_inv
    K_dr = ((2*zeta*omega_n)+(1/T))*R_inv
    K_ir = ((omega_n**2)/T)*R_inv

    """
    > R_inv: The inverse of a diagonal matrix whose diagonal entries are the motor torque constants
    > K_pr: The prortional gain matrix
    > K_dr: The derivative gain matrix
    > K_ir: The integrator gain matrix
    """

    u = np.array((np.dot(K_pr, co_state(q_actual, q_command)))+(np.dot(K_dr, w_command-w_actual))+(np.dot(K_ir,sigma_integrate_prev + h*co_state(q_actual, q_command))))
    return u

    