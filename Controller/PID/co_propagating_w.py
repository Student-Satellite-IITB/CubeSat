# Author: Ajay Tak
# Date: 25-07-2022

import numpy as np

def co_dynamic_function(I_i1, I_i2, I_i3, T_i1, w_i2, w_i3):
    
    """
    > K is defined in the function below
    > T_i is the torque provided by the i'th reaction wheel 
    > W_i1, W_i2, W_i3, each are the reaction wheel angular velocities
    > w_i2, w_i3 are  2 components of cubesat's angular velocity wrt to earth's inertial frame expressed in body frame
    > I_i2, I_i3 are the two principle elements of the moment of inertia matrix of the cubesat
    > J_par is the moment of inertia of the reaction wheel about the spin axis
    """

    w_i1_dot = ((w_i2*w_i3*(I_i2-I_i3)) + T_i1)/I_i1

    return w_i1_dot


def co_propagating_w_i(w_1, w_2, w_3, I_1, I_2, I_3, T_1, T_2, T_3, h):

    """
    > w_i1, w_i2, w_i3 are components of cubesat's angular velocity wrt to earth's inertial frame expressed in body frame
    > I_i1, I_i2, I_i3 are the three principle elements of the moment of inertia matrix
    > T_i1, T_i2, T_i3 are the three components of the control torque provided by the reaction wheels
    > W_i1, W_i2, W_i3, each are the reaction wheel angular velocities
    > J_par is the moment of inertia of the reaction wheel about the spin axis
    > J_perp is the moment of inertia of the reaction wheel about an axis perpendicular to its spin axis
    > h is time step
    """
  
    """
    use of the above constant will be clear in the further steps
    """

    a1 = co_dynamic_function(I_1, I_2, I_3, T_1, w_2, w_3)
    a2 = co_dynamic_function(I_2, I_3, I_1, T_2, w_3, w_1)
    a3 = co_dynamic_function(I_3, I_1, I_2, T_3, w_1, w_2)
    b1 = co_dynamic_function(I_1, I_2, I_3, T_1, w_2 + (0.5*h*a2), w_3 + (0.5*h*a3))
    b2 = co_dynamic_function(I_2, I_3, I_1, T_2, w_3 + (0.5*h*a3), w_1 + (0.5*h*a1))
    b3 = co_dynamic_function(I_3, I_1, I_2, T_3, w_1 + (0.5*h*a1), w_2 + (0.5*h*a2))
    c1 = co_dynamic_function(I_1, I_2, I_3, T_1, w_2 + (0.5*h*b2), w_3 + (0.5*h*b3))
    c2 = co_dynamic_function(I_2, I_3, I_1, T_2, w_3 + (0.5*h*b3), w_1 + (0.5*h*b1))
    c3 = co_dynamic_function(I_3, I_1, I_2, T_3, w_1 + (0.5*h*b1), w_2 + (0.5*h*b2))
    d1 = co_dynamic_function(I_1, I_2, I_3, T_1, w_2 + (h*c2), w_3 + (h*c3))
    d2 = co_dynamic_function(I_2, I_3, I_1, T_2, w_3 + (h*c3), w_1 + (h*c1))
    d3 = co_dynamic_function(I_3, I_1, I_2, T_3, w_1 + (h*c1), w_2 + (h*c2))
    w_1_next = w_1 + ((h/6)*(a1 + (2*b1) + (2*c1) + (d1)))
    w_2_next = w_2 + ((h/6)*(a2 + (2*b2) + (2*c2) + (d2)))
    w_3_next = w_3 + ((h/6)*(a3 + (2*b3) + (2*c3) + (d3)))

    return np.array([w_1_next, w_2_next, w_3_next])