# Author: Ajay Tak
# Date: 14-06-2022

import numpy as np

def co_kinematic_function(q_0, q_1, q_2, q_3, P, Q, R):

    """
    > q_0, q_1, q_2, q_3 are the components of quaternion from body frame to inertial frame
    > P, Q, R are the components of the angular velocity of body frame with respect to inertial frame
    """

    q_IB = np.transpose(np.array([q_0, q_1, q_2, q_3]))
    m_Q = 0.5*np.matrix([[0, -P, -Q, -R], 
                        [P,   0,  R, -Q],
                        [Q,  -R,   0, P],
                        [R,   Q, -P,  0]])
    dot = np.dot(m_Q, q_IB)
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

    return q_IB_next

q = np.array([-0.3061862, 0.4355957, -0.6597396, 0.5303301])
P = 0.5
Q = 0.5
R = 0.5
for i in range(2000):
    q_next = co_propagatin_q_IB(q[0], q[1], q[2], q[3], P, Q, R, 1)
    print(np.sqrt(np.sum(q**2)))
    q = q_next
