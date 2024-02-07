import rospy,tf

from geometry_msgs.msg import TwistStamped,Twist,PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

import math
from math import radians
import numpy as np

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
    #dist_arr[dist_arr > 12] = 12
    dist_arr[dist_arr > min_dist + 0.25] = 0
    #dist_arr[::] = dist_arr/12
    #dist_arr[dist_arr > 2] = 2

    start_angle = scan.data.angle_min
    incr = scan.data.angle_increment
    N = len(dist_arr)
    angles = np.array([start_angle + i*incr for i in range(N)])

    #indizes = inFront(angles)
    #(X,Y) = calcVectors(angles[indizes])*dist_arr[indizes]
    (X,Y) = calcVectors(angles)*dist_arr
    x = np.sum(X)/len(X)
    y = np.sum(Y)/len(Y)
    
    return x,y,min_dist

def main():
    rospy.init_node("motor")

    scan = ScanData()
    vis_pub = rospy.Publisher("/cmd_vel_stamped",TwistStamped,queue_size = 10)
    vis_dir_pub = rospy.Publisher("/cmd_dir_stamped",TwistStamped,queue_size=10)
    
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    tf_listener = tf.TransformListener()

    running = True
    rate = rospy.Rate(10)
    seq = 0
    while running:
        dir_vel = Twist()
        dir_vel.linear.x = 1
        #try:
        #    pose = PoseStamped(header=Header(frame_id="map",stamp=rospy.Time.now()-rospy.Duration(0.1)))
        #    pose.pose.position.x,pose.pose.position.y = (0,0)

        #    bf_pose = tf_listener.transformPose("base_footprint",pose)
        #    x = bf_pose.pose.position.x
        #    y = bf_pose.pose.position.y
        #    s = math.sqrt(x**2+y**2)
            #print(x,y,math.atan2(math.sin(y),math.cos(x)))
        #    dir_vel.linear.x = x/s#-min(1,max(-1,x/s))
        #    dir_vel.angular.z = y/s#-min(1,max(-1,y/s))
            #vel_pub.publish(dir_vel)
        #except Exception as e:
        #    rospy.loginfo(e)

        x,y,min_dist = calcVelocity(scan)

        alpha = math.atan2(x,y)
        #print(alpha)
        try:
            header = Header(frame_id="base_footprint",stamp=rospy.Time.now(),seq=seq)
            seq += 1

            vis_msg = TwistStamped(header=header)
            vis_dir = TwistStamped(header=header)
            #print(msg.twist)

            #x_vel = math.cos(alpha)
            #y_vel = math.sin(alpha)

            vel = Twist()
            #vel.linear.x = x_vel
            #vel.angular.z = y_vel

            #beta = math.atan2(math.sin(dir_vel.angular.z),math.cos(dir_vel.linear.x))
            #print(f"beta: {beta}")
            if min_dist > 3:
                vel.linear.x = math.cos(alpha)
                vel.angular.z = math.sin(alpha)
            elif -math.pi <= alpha < 0:
                vel.linear.x = math.cos(alpha +radians(90))
                vel.angular.z = math.sin(alpha +radians(90))
            else:
                vel.linear.x = math.cos(alpha -radians(90))
                vel.angular.z = math.sin(alpha -radians(90))

            #vel.linear.x += dir_vel.linear.x
            #vel.angular.z += dir_vel.angular.z
            #vel.linear.x /= 2
            #vel.angular.z /= 2
                
            vel.linear.x = max(vel.linear.x,0)

            vis_msg.twist.linear.x = vel.linear.x
            vis_msg.twist.linear.y = vel.angular.z
            vis_dir.twist.linear.x = dir_vel.linear.x
            vis_dir.twist.linear.y = dir_vel.angular.z

            vis_pub.publish(vis_msg)
            vis_dir_pub.publish(vis_dir)
            vel_pub.publish(vel)

            rate.sleep()
        except Exception as e:
            print(e)
            running = False

    sub.unregister()

if __name__ == "__main__":
    main()