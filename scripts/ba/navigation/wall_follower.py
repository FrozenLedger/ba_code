import rospy

from geometry_msgs.msg import TwistStamped,Twist,PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from std_srvs.srv import SetBool,SetBoolResponse

import math
from math import radians
import numpy as np

from ba.utilities.data import Data

@np.vectorize
def inFront(p):
    return -math.pi/2 <= p <= math.pi/2

@np.vectorize
def calcVectors(V):
    return (math.sin(V),math.cos(V))

def calcVelocity(scan,obstacle=False):
    dist_arr = np.array(scan.data.ranges)

    if len(dist_arr) <= 0:
        return (0,0,0)
    
    #min_dist = np.min(dist_arr)
    #if min_dist == math.inf:
    #    min_dist = scan.data.range_max
    
    # indized = np.logical_and(dist_arr <= scan.data.range_max, dist_arr >= scan.data.range_min, not np.nan(dist_arr))
    dist_arr[dist_arr > scan.data.range_max] = scan.data.range_max
    dist_arr[dist_arr < scan.data.range_min] = scan.data.range_max # <---

    min_dist = np.min(dist_arr)
    #print(min_dist)
    if obstacle:
        dist_arr[dist_arr > min_dist + 0.2] = 0
    else:
        dist_arr[dist_arr > scan.data.range_max] = scan.data.range_max

    start_angle = scan.data.angle_min
    incr = scan.data.angle_increment
    N = len(dist_arr)
    angles = np.array([start_angle + i*incr for i in range(N)])

    (Y,X) = calcVectors(angles)*dist_arr
    x = np.average(X) #np.sum(X)/len(X)
    y = np.average(Y) #np.sum(Y)/len(Y)

    return x,y,min_dist

def calcVelocityII(scan,obstacle=False):
    dist_arr = np.array(scan.data.ranges)

    if len(dist_arr) <= 0:
        return (0,0,0)
    
    #min_dist = np.min(dist_arr)
    #if min_dist == math.inf:
    #    min_dist = scan.data.range_max
    
    indizes = np.logical_and(dist_arr <= scan.data.range_max, dist_arr >= scan.data.range_min, ~np.isnan(dist_arr))
    # dist_arr[dist_arr > scan.data.range_max] = scan.data.range_max
    # dist_arr[dist_arr < scan.data.range_min] = scan.data.range_max # <---

    min_dist = np.nanmin(dist_arr)
    #print(min_dist)
    if obstacle:
        dist_arr[dist_arr > min_dist + 0.2] = 0
    else:
        dist_arr[dist_arr > scan.data.range_max] = scan.data.range_max

    start_angle = scan.data.angle_min
    incr = scan.data.angle_increment
    N = len(dist_arr)
    angles = np.array([start_angle + i*incr for i in range(N)])

    (Y,X) = calcVectors(angles[indizes])*dist_arr[indizes]
    x = np.nanmean(X) #average(X) #np.sum(X)/len(X)
    y = np.nanmean(Y) #average(Y) #np.sum(Y)/len(Y)

    return x,y,min_dist

class WallFollower:
    """A node that enables the robot to follow a wall and explore its outline."""
    def __init__(self,max_spd:float = 0.2,min_gap:float = 2,enabled=False):
        self.__scan = Data(LaserScan())
        self.__scan_sub = rospy.Subscriber("/scan",LaserScan,self.__scan.setData)
        self.__vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.__max_spd = max_spd
        self.__min_gap = min_gap
        self.__enabled = enabled
        self.__explore_rate = rospy.Rate(10)
        self.__idle_rate = rospy.Rate(1)
        self.__state = self.__idle

        self.__enable_service = rospy.Service("/wall_follower/enable",SetBool,self.__set_enable)
        rospy.loginfo(f"WallFollower running. Enabled: {self.__enabled}")

    def __set_enable(self,request):
        self.__enabled = request.data
        return SetBoolResponse(success=True)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.__state()
            except Exception as e:
                print(e)
            
    def __idle(self):
        # State change
        if self.__enabled:
            self.__state = self.__follow_wall
            print("State: follow wall")
            return
        # sleep
        self.__idle_rate.sleep()

    def __follow_wall(self):
        # State change
        if not self.__enabled:
            self.__state = self.__idle
            print("State: idle")
            return
        
        # Function
        msg = Twist()
        msg.linear.x = 1

        ox,oy,min_dist = calcVelocityII(self.__scan,obstacle=True)

        alpha = math.atan2(oy,ox) # obstacle
        alpha -= math.pi/2
        print(math.degrees(alpha))

        if -math.pi <= alpha < 0:
            fdir = radians(90)
        else:
            fdir = -radians(90)
        
        angularF = 1.25

        vel = Twist()
        if (min_dist < 0.4) and inFront(alpha):
            vel.linear.x = -0.1
            vel.angular.z = 0.2*(-1)**(math.sin(alpha) > 0)
        elif min_dist > self.__min_gap:
            vel.linear.x = math.cos(alpha)
            vel.angular.z = math.sin(alpha)*angularF
        elif -math.pi <= alpha < 0:
            vel.linear.x = math.cos(alpha +fdir)
            vel.angular.z = math.sin(alpha +fdir)*angularF
        else:
            vel.linear.x = math.cos(alpha +fdir)
            vel.angular.z = math.sin(alpha +fdir)*angularF

        vel.linear.x = max(min(vel.linear.x,self.__max_spd),-self.__max_spd)
        self.__vel_pub.publish(vel)

        # sleep
        self.__explore_rate.sleep()
        
def main():
    rospy.init_node("wall_drive")
    rospy.loginfo("Start node wall_drive.")

    scan = Data(LaserScan())
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    
    max_lin_spd = 0.2 #0.15
    min_gap = 3 #5

    running = True
    rate = rospy.Rate(10)
    while running and not rospy.is_shutdown():
        dir_vel = Twist()
        dir_vel.linear.x = 1

        ox,oy,min_dist = calcVelocityII(scan,obstacle=True)

        alpha = math.atan2(oy,ox) # obstacle
        #alpha += math.pi

        if -math.pi <= alpha < 0:
            fdir = radians(90)
        else:
            fdir = -radians(90)
        
        try:
            vel = Twist()
            if (min_dist < 0.4) and inFront(alpha):
                vel.linear.x = -0.2
                vel.angular.z = 0.2*(-1)**(math.sin(alpha) > 0)
            elif min_dist > min_gap:
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
    rospy.loginfo("/cmv_vel subscriber unregistered.")

def mainII():
    rospy.init_node("wall_follower")
    wall_follower = WallFollower(max_spd=0.2,min_gap=0.75,enabled=False)
    wall_follower.loop()

if __name__ == "__main__":
    mainII()