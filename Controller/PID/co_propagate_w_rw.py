# Author: Ajay Tak
# Date: 18-06-2022

def co_actuator_model(J_par, T_i, W_i, b_i, w_i_dot):

    """
    > I_dash is the moment of inertia of the reaction wheel about the spin axis
    > Ir_i is our control variable/ armature current of the motor driving the reaction wheel
    > W_i is the reaction wheel angular velocity
    > w_i_dot is the rate of change of angular velocity (one component) of the body frame with respect to inertial frame
    """
    W_i_dot = (T_i-(b_i*W_i)-(J_par*w_i_dot))/J_par

    return W_i_dot


def co_propagating_w_rw_i(J_par, T_i, W_i, b_i, w_i_present, w_i_previous, h):
    
    """
    > J_par is the moment of inertia of the reaction wheel about the spin axis
    > T_i is our control Torque of the motor driving the i'th reaction wheel
    > W_i is the i'th reaction wheel angular velocity
    > w_i_present is the value of w_i (component of angular velocity) at present time step
    > w_i_previous is the value of w_i (component of angular velocity) at previous time step
    > h is time step
    """

    w_i_dot = (w_i_present-w_i_previous)/h
    a = co_actuator_model(J_par, T_i, W_i, b_i, w_i_dot)
    b = co_actuator_model(J_par, T_i, W_i+(0.5*h*a), b_i, w_i_dot)
    c = co_actuator_model(J_par, T_i, W_i+(0.5*h*b), b_i, w_i_dot)
    d = co_actuator_model(J_par, T_i, W_i+(h*c), b_i, w_i_dot)
    W_i_next = W_i + ((h/6)*(a + 2*b + 2*c + d))

    return W_i_next