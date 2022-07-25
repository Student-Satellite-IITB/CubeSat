# Author: Ajay Tak
# Date: 12-06-2022

import numpy as np

def co_dynamic_function(K, T_i, W_i2, W_i3, w_i2, w_i3, I_i2, I_i3, J_par):
    
    """
    > K is defined in the function below
    > T_i is the torque provided by the i'th reaction wheel 
    > W_i1, W_i2, W_i3, each are the reaction wheel angular velocities
    > w_i2, w_i3 are  2 components of cubesat's angular velocity wrt to earth's inertial frame expressed in body frame
    > I_i2, I_i3 are the two principle elements of the moment of inertia matrix of the cubesat
    > J_par is the moment of inertia of the reaction wheel about the spin axis
    """

    return -((T_i)+(w_i2*w_i3*(I_i3-I_i2))+(J_par*((W_i3*w_i2)-(W_i2*w_i3))))/(K)


def co_propagating_w_i(w_i1, w_i2, w_i3, I_i1, I_i2, I_i3, T_i1, T_i2, T_i3, W_i1, W_i2, W_i3, J_perp, J_par, h):

    """
    > w_i1, w_i2, w_i3 are components of cubesat's angular velocity wrt to earth's inertial frame expressed in body frame
    > I_i1, I_i2, I_i3 are the three principle elements of the moment of inertia matrix
    > T_i1, T_i2, T_i3 are the three components of the control torque provided by the reaction wheels
    > W_i1, W_i2, W_i3, each are the reaction wheel angular velocities
    > J_par is the moment of inertia of the reaction wheel about the spin axis
    > J_perp is the moment of inertia of the reaction wheel about an axis perpendicular to its spin axis
    > h is time step
    """

    Ki1 = I_i1 + (2*J_perp)
    Ki2 = I_i2 + (2*J_perp) 
    Ki3 = I_i3 + (2*J_perp)  
    """
    use of the above constant will be clear in the further steps
    """

    ai1 = co_dynamic_function(Ki1, T_i1, W_i2, W_i3, w_i2, w_i3, I_i2, I_i3, J_par)
    ai2 = co_dynamic_function(Ki2, T_i2, W_i3, W_i1, w_i3, w_i1, I_i3, I_i1, J_par)
    ai3 = co_dynamic_function(Ki3, T_i3, W_i1, W_i2, w_i1, w_i2, I_i1, I_i2, J_par)
    bi1 = co_dynamic_function(Ki1, T_i1, W_i2, W_i3, w_i2 + (0.5*h*ai2), w_i3 + (0.5*h*ai3), I_i2, I_i3, J_par)
    bi2 = co_dynamic_function(Ki2, T_i2, W_i3, W_i1, w_i3 + (0.5*h*ai3), w_i1 + (0.5*h*ai1), I_i3, I_i1, J_par)
    bi3 = co_dynamic_function(Ki3, T_i3, W_i1, W_i2, w_i1 + (0.5*h*ai1), w_i2 + (0.5*h*ai2), I_i1, I_i2, J_par)
    ci1 = co_dynamic_function(Ki1, T_i1, W_i2, W_i3, w_i2 + (0.5*h*bi2), w_i3 + (0.5*h*bi3), I_i2, I_i3, J_par)
    ci2 = co_dynamic_function(Ki2, T_i2, W_i3, W_i1, w_i3 + (0.5*h*bi3), w_i1 + (0.5*h*bi1), I_i3, I_i1, J_par)
    ci3 = co_dynamic_function(Ki3, T_i3, W_i1, W_i2, w_i1 + (0.5*h*bi1), w_i2 + (0.5*h*bi2), I_i1, I_i2, J_par)
    di1 = co_dynamic_function(Ki1, T_i1, W_i2, W_i3, w_i2 + (h*ci2), w_i3 + (h*ci3), I_i2, I_i3, J_par)
    di2 = co_dynamic_function(Ki2, T_i2, W_i3, W_i2, w_i3 + (h*ci3), w_i1 + (h*ci1), I_i3, I_i1, J_par)
    di3 = co_dynamic_function(Ki3, T_i3, W_i1, W_i3, w_i1 + (h*ci1), w_i2 + (h*ci2), I_i1, I_i2, J_par)
    w_i1_next = w_i1 + ((h/6)*(ai1 + (2*bi1) + (2*ci1) + (di1)))
    w_i2_next = w_i2 + ((h/6)*(ai2 + (2*bi2) + (2*ci2) + (di2)))
    w_i3_next = w_i3 + ((h/6)*(ai3 + (2*bi3) + (2*ci3) + (di3)))

    return np.array([w_i1_next, w_i2_next, w_i3_next])