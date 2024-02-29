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