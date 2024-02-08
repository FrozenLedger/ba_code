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
    x = np.average(X) #np.sum(X)/len(X)
    y = np.average(Y) #np.sum(Y)/len(Y)
    
    return x,y,min_dist

def main():
    rospy.init_node("fuzzy_drive")

    scan = ScanData()
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    max_lin_spd = 0.4
    min_gap = 3

    running = True
    rate = rospy.Rate(10)
    while running and not rospy.is_shutdown():
        dir_vel = Twist()
        dir_vel.linear.x = 1

        x,y,min_dist = calcVelocity(scan)
        alpha = math.atan2(x,y)
        
        if -math.pi <= alpha < 0:
            fdir = radians(90)
        else:
            fdir = -radians(90)
        
        try:
            vel = Twist()
            if min_dist > min_gap:
                vel.linear.x = math.cos(alpha)
                vel.angular.z = math.sin(alpha)
            elif -math.pi <= alpha < 0:
                vel.linear.x = math.cos(alpha +fdir)
                vel.angular.z = math.sin(alpha +fdir)
            else:
                vel.linear.x = math.cos(alpha +fdir)
                vel.angular.z = math.sin(alpha +fdir)

            vel.linear.x = max(min(vel.linear.x,max_lin_spd),-max_lin_spd)
            vel_pub.publish(vel)

            rate.sleep()
        except Exception as e:
            print(e)
            running = False

    sub.unregister()

if __name__ == "__main__":
    main()