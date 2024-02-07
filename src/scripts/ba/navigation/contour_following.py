import rospy,tf

from geometry_msgs.msg import TwistStamped,Twist,PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

import math
from math import radians
import numpy as np

from ba.configs import min_gap,max_gap

class ScanData:
    def __init__(self):
        self.__data = LaserScan()

    @property
    def data(self):
        return self.__data

    def setData(self,msg):
        self.__data = msg

@np.vectorize
def inFront(p):
    return -math.pi/2 <= p <= math.pi/2

@np.vectorize
def calcVectors(V):
    return (math.sin(V),math.cos(V))

def calcVelocity(scan):
    dist_arr = np.array(scan.data.ranges)

    if len(dist_arr) <= 0:
        return (0,0,0)
    
    min_dist = np.min(dist_arr)
    if min_dist == math.inf:
        min_dist = scan.data.range_max

    start_angle = scan.data.angle_min
    incr = scan.data.angle_increment
    MAX_N = len(dist_arr)
    angles = np.array([start_angle + i*incr for i in range(MAX_N)])
    #dist_arr[dist_arr > 12] = 12
        
    #dist_arr[dist_arr > min_dist + 0.25] = 0 #
    indizes = np.logical_and(dist_arr >= min_dist,dist_arr < min_dist + 0.2)
    dist_arr = dist_arr[indizes]

    angles = angles[indizes]

    #indizes = inFront(angles)
    #(X,Y) = calcVectors(angles[indizes])*dist_arr[indizes]
    (X,Y) = calcVectors(angles)*dist_arr
    x = np.sum(X)/len(X)
    y = np.sum(Y)/len(Y)
    
    return x,y,np.average(dist_arr) #min_dist

def main():
    rospy.init_node("motor")

    scan = ScanData()
    vis_pub = rospy.Publisher("/cmd_vel_stamped",TwistStamped,queue_size = 10)
    vis_dir_pub = rospy.Publisher("/cmd_dir_stamped",TwistStamped,queue_size=10)
    
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    #tf_listener = tf.TransformListener()

    running = True
    rate = rospy.Rate(10)
    seq = 0
    while running:
        dir_vel = Twist()
        dir_vel.linear.x = 1

        x,y,min_dist = calcVelocity(scan)
        print(min_dist)

        alpha = math.atan2(x,y)
        try:
            header = Header(frame_id="base_footprint",stamp=rospy.Time.now(),seq=seq)
            seq += 1

            vis_msg = TwistStamped(header=header)
            vis_dir = TwistStamped(header=header)

            if -math.pi <= alpha < 0: # defines the rotation direction depending on the obstacles measured
                rot_dir = 1
            else:
                rot_dir = -1

            vel = Twist()

            rospy.loginfo(f"alpha: {math.degrees(round(alpha,2))}°")
            #rospy.loginfo(f"vel: {vel}")

            vis_msg.twist.linear.x = vel.linear.x
            vis_msg.twist.linear.y = vel.angular.z
            vis_dir.twist.linear.x = dir_vel.linear.x
            vis_dir.twist.linear.y = dir_vel.angular.z

            vis_pub.publish(vis_msg)
            vis_dir_pub.publish(vis_dir)
            #vel_pub.publish(vel)

            rate.sleep()
        except Exception as e:
            print(e)
            running = False

    sub.unregister()

if __name__ == "__main__":
    main()