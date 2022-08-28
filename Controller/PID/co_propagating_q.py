# Author: Ajay Tak
# Date: 14-06-2022

import numpy as np

def co_kinematic_function(q_0, q_1, q_2, q_3, P, Q, R):

    """
    > q_0, q_1, q_2, q_3 are the components of quaternion from body frame to inertial frame
    > P, Q, R are the components of the angular velocity of body frame with respect to inertial frame
    """

    q_IB = np.array([q_0, q_1, q_2, q_3])
    w = np.array([P, Q, R])
    m_Q = 0.5*np.matrix([[-q_IB[1], -q_IB[2], -q_IB[3]],
                         [ q_IB[0], -q_IB[3],  q_IB[2]],
                         [ q_IB[3],  q_IB[0], -q_IB[1]],
                         [-q_IB[2],  q_IB[1],  q_IB[0]]])
    dot = np.dot(m_Q, w)
    q_IB_dot = np.array([dot[0, 0], dot[0, 1], dot[0, 2], dot[0, 3]])

    return q_IB_dot

def co_propagatin_q_IB(q_0, q_1, q_2, q_3, P, Q, R, h):

    """
    > q_0, q_1, q_2, q_3 are the components of quaternion from body frame to inertial frame
    > P, Q, R are the components of the angular velocity of body frame with respect to inertial frame
    > h is time step
    """

    q_IB = np.array([q_0, q_1, q_2, q_3])
    a = co_kinematic_function(q_0, q_1, q_2, q_3, P, Q, R)
    b = co_kinematic_function(q_0+(0.5*h*a[0]), q_1+(0.5*h*a[1]), q_2+(0.5*h*a[2]), q_3+(0.5*h*a[3]), P, Q, R)
    c = co_kinematic_function(q_0+(0.5*h*b[0]), q_1+(0.5*h*b[1]), q_2+(0.5*h*b[2]), q_3+(0.5*h*b[3]), P, Q, R)
    d = co_kinematic_function(q_0+(h*c[0]), q_1+(h*c[1]), q_2+(h*c[2]), q_3+(h*c[3]), P, Q, R)
    q_IB_next = q_IB + ((h/6)*(a + 2*b + 2*c + d))
    '''if q_IB_next[0] > 0:
        q_IB_next[0] = np.sqrt(1-(q_IB[1]**2)-(q_IB[2]**2)-(q_IB[3]**2))
    else:
        q_IB_next[0] = -np.sqrt(1-(q_IB[1]**2)-(q_IB[2]**2)-(q_IB[3]**2))'''
        
    return q_IB_next