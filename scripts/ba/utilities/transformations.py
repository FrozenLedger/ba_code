import numpy as np
import quaternion # pip install numpy-quaternion

from tf.transformations import quaternion_from_euler as quat_from_euler
from geometry_msgs.msg import Quaternion

def quaternion_from_euler(euler_angles):
    """Transforms euler angles to the ros specific message type: geometry_msgs/Quaternion."""
    Q = Quaternion()
    q = quat_from_euler(*euler_angles)
    Q.x = q[0]
    Q.y = q[1]
    Q.z = q[2]
    Q.w = q[3]
    return Q

def quaternion_msg_multiply(q1: Quaternion,q2: Quaternion) -> Quaternion:
    q3 = np.quaternion(q1.w,q1.x,q1.y,q1.z)*np.quaternion(q2.w,q2.x,q2.y,q2.z)
    return Quaternion(x=q3.x,y=q3.y,z=q3.z,w=q3.w)

def unwrap_quaternion_msg(q: Quaternion):
    return np.array([q.w,q.x,q.y,q.z],dtype=np.float64)