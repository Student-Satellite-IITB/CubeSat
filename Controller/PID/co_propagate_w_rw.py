# Author: Ajay Tak
# Date: 18-06-2022

def co_actuator_model(I_dash, kr_i, br_i, Ir_i, W_i, w_i_dot):

    """
    > I_dash is the moment of inertia of the reaction wheel about the spin axis
    > kr_i is the motor torque constant
    > br_i is the motor frictional co-efficient
    > Ir_i is our control variable/ armature current of the motor driving the reaction wheel
    > W_i is the reaction wheel angular velocity
    > w_i_dot is the rate of change of angular velocity (one component) of the body frame with respect to inertial frame
    """
    W_i_dot = ((kr_i*Ir_i) - (br_i*W_i) - (I_dash*(w_i_dot)))/I_dash

    return W_i_dot


def co_propagating_w_rw_i(I_dash, kr_i, br_i, Ir_i, W_i, w_i_present, w_i_previous, h):
    
    """
    > I_dash is the moment of inertia of the reaction wheel about the spin axis
    > kr_i is the motor torque constant
    > br_i is the motor frictional co-efficient
    > Ir_i is our control variable/ armature current of the motor driving the reaction wheel
    > W_i is the reaction wheel angular velocity
    > w_i_present is the value of w_i (component of angular velocity) at present time step
    > w_i_previous is the value of w_i (component of angular velocity) at previous time step
    > h is time step
    """

    w_i_dot = (w_i_present-w_i_previous)/h
    a = co_actuator_model(I_dash, kr_i, br_i, Ir_i, W_i, w_i_dot)
    b = co_actuator_model(I_dash, kr_i, br_i, Ir_i, W_i + (0.5*h*a), w_i_dot)
    c = co_actuator_model(I_dash, kr_i, br_i, Ir_i, W_i + (0.5*h*b), w_i_dot)
    d = co_actuator_model(I_dash, kr_i, br_i, Ir_i, W_i + (h*c), w_i_dot)
    W_i_next = W_i + ((h/6)*(a + 2*b + 2*c + d))

    return W_i_next