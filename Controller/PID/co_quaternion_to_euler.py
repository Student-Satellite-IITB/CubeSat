# Author: Ajay Tak
# Date: 30-06-2022

import numpy as np
from scipy.spatial.transform import Rotation

def co_quat_to_euler(q):
    Q = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    return Q.as_euler('zyx', True)
