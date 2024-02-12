import rospy,tf

from ba.utilities.data import Data

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from std_srvs.srv import SetBool

import numpy as np

def main():
    rospy.init_node("simple_wall_explorer")

    tf_listener = tf.TransformListener()
    rospy.Rate(1).sleep()

    base = "base_footprint"
    world = "map"

    wallsrv = "wall_following/enable"
    wall_service = rospy.ServiceProxy(wallsrv,SetBool)
    rospy.wait_for_service(wallsrv)

    tf_listener.waitForTransform(base,world,rospy.Time(),rospy.Duration(5))

    t0 = rospy.Time.now() #start time
    header = Header(stamp=t0,frame_id=base)
    tf_listener.waitForTransform(base,world,t0,rospy.Duration(5))
    p0 = tf_listener.transformPoint(world,PointStamped(header=header)) # start position

    # Start wall following
    wall_service(True)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            t1 = rospy.Time.now()
            header = Header(stamp=t1,frame_id=base)
            tf_listener.waitForTransform(base,world,t1,rospy.Duration(5))
            p1 = tf_listener.transformPoint(world,PointStamped(header=header))
            dist = distance(p0,p1)
            print(dist)
            if dist <= 1 and (t1-t0) > rospy.Duration(10):
                break
        except Exception as e:
            print(e)
        rate.sleep()

    # Stop wall following
    wall_service(False)
    print(f"Start: {p0}\tEnd:{p1}\tdT:{t1-t0}")

def unwrap(msg: PointStamped):
    return np.array((msg.point.x,msg.point.y,msg.point.z))

def distance(p0,p1):
    # based on: https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
    return np.linalg.norm(unwrap(p0)-unwrap(p1))

if __name__ == "__main__":
    main()