import rospy,math

from sensor_msgs.msg import LaserScan
import numpy as np

from geometry_msgs.msg import Polygon,PolygonStamped,Point32

from ba.utilities.data import Data
from ba.configs import min_gap

def build_polygon(dist_arr,angles,seq=0):
    N = len(dist_arr)
    points = [Point32() for _ in range(N)]
    for idx,v in enumerate(dist_arr):
        alpha = angles[idx]#start + incr*idx
        x = round(math.cos(alpha),4) * v
        y = round(math.sin(alpha),4) * v
        points[idx].x = x
        points[idx].y = y

    poly = Polygon()
    poly_stamped = PolygonStamped()
            
    poly_stamped.header.seq = seq
    poly_stamped.header.stamp = rospy.Time.now()
    poly_stamped.header.frame_id = "base_footprint"

    poly.points = points
    poly_stamped.polygon = poly
    return poly_stamped

def main():
    rospy.init_node("cone_visualizer")

    scan = Data(LaserScan())
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    pub = rospy.Publisher("/cone_vis",PolygonStamped,queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = scan.data
        
        if len(data.ranges) > 0:
            dist_arr = np.array(data.ranges)
            dist_arr[dist_arr > min_gap] = min_gap
            min_dist = np.min(dist_arr)
            
            start = data.angle_min
            incr = data.angle_increment
            angles = np.array([start+i*incr for i in range(len(data.ranges))])

            #idx_filter = np.logical_and(-math.radians(45) <= angles, angles <= math.radians(45))
            rospy.loginfo(f"Min_distance: {min_dist}")
            rospy.loginfo(min_gap)

            poly_stamped = build_polygon(dist_arr,angles) #dist_arr[idx_filter],angles[idx_filter])

            pub.publish(poly_stamped)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    sub.unregister()

if __name__ == "__main__":
    main()