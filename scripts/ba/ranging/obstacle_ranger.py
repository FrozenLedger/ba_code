import rospy

from geometry_msgs.msg import TwistStamped,Twist,PoseArray,Pose,Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray

import math
from math import radians
import numpy as np

from ba.configs import min_gap,max_gap
from ba.utilities.data import Data
from ba.ranging.scanreader import ScanReader

#@np.vectorize
#def inFront(p):
#    return -math.pi/2 <= p <= math.pi/2

def main():
    rospy.init_node("obstacle_ranger")

    scan = Data(LaserScan())
    #vis_pub = rospy.Publisher("/closest_obstacle",TwistStamped,queue_size = 10)
    vis_pub_arr = [rospy.Publisher(f"/obstacle{i}",TwistStamped,queue_size = 10) for i in range(9)]

    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)

    running = True
    rate = rospy.Rate(10)
    seq = 0
    while running and not rospy.is_shutdown():
        scan_reader = ScanReader(scan.data) # reads distance and angle values from laserscan
        
        #s,c,alpha,min_dist,min_avg,min_med = scan_reader.calcVectorSumNormalized() # calculates the normalized vector of all distance measurements
        #(start_angle,end_angle) = -math.pi,0
        #(start_angle,end_angle) = -radians(30),radians(30)
        incr = (scan_reader.scan.angle_max-scan_reader.scan.angle_min)/8 #(2*math.pi)/8
        start_angle = scan_reader.scan.angle_min #-math.pi
        angles = [(start_angle+incr*i,start_angle+incr*(i+1)) for i in range(8)]
        vsd_arr = [scan_reader.calcVectorSumNormalized(s,e) for (s,e) in angles]#math.pi) # calculates the normalized vector of all distance measurements
        
        #vsd = scan_reader.calcVectorSumNormalized(start_angle,end_angle)#math.pi) # calculates the normalized vector of all distance measurements
        #rospy.loginfo(f"s: {round(vsd.yn,3)}\tc: {round(vsd.xn,3)}\talpha: {round(math.degrees(vsd.alpha),2)}°")
        #rospy.loginfo(f"min_dist: {round(vsd.min_dist,3)}m\tmin_avg: {round(vsd.min_avg,3)}m\tmin_median: {round(vsd.min_median,3)}m")
        
        try:
            header = Header(frame_id="base_footprint",stamp=rospy.Time.now(),seq=seq)
            seq += 1

            #vis_msg = TwistStamped(header=header)
            #vis_msg.twist.linear.x = vsd.xn
            #vis_msg.twist.linear.y = vsd.yn
            vis_msg_arr = [TwistStamped(header=header) for _ in range(9)]
            sx = 0
            sy = 0
            for idx,vsd in enumerate(vsd_arr):
                vis_msg_arr[idx].twist.linear.x = vsd.x
                vis_msg_arr[idx].twist.linear.y = vsd.y
                vis_msg_arr[idx].twist.linear.z = 1
                sx += vsd.x
                sy += vsd.y
            vis_msg_arr[-1].twist.linear.x = sx
            vis_msg_arr[-1].twist.linear.y = sy
            
            for idx,pub in enumerate(vis_pub_arr):
                pub.publish(vis_msg_arr[idx])

            rate.sleep()
        except Exception as e:
            print(e)
            running = False

    sub.unregister()

if __name__ == "__main__":
    main()