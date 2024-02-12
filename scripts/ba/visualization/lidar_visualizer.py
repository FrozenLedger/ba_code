import rospy,math

from sensor_msgs.msg import LaserScan
import numpy as np

from geometry_msgs.msg import Polygon,PolygonStamped,Point32

class LaserScanData:
    def __init__(self):
        self.__msg = LaserScan()

    @property
    def data(self):
        return self.__msg
    
    def setData(self,msg):
        self.__msg = msg

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
    rospy.init_node("lidar_visualizer")

    scan = LaserScanData()
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)

    pub = rospy.Publisher("/poly",PolygonStamped,queue_size=10)

    rate = rospy.Rate(10)
    seq = 0
    while not rospy.is_shutdown():
        data = scan.data
        N = len(data.ranges)
        info = f"min: {data.range_min}\tmax: {data.range_max}\tN: {N}\tincr: {data.angle_increment}\n"
        if N > 0:
            dist_arr = np.array(data.ranges)
            dist_arr[dist_arr > 12] = 12#data.range_max
            indizes = ~np.isnan(dist_arr)
            #dist_arr[dist_arr == np.nan] = 12
            #dist_arr[dist_arr == np.inf] = 12
            
            start = data.angle_min
            incr = data.angle_increment
            angles = np.array([start+i*incr for i in range(N)])

            poly_stamped = build_polygon(dist_arr[indizes],angles[indizes])
            pub.publish(poly_stamped)
        print(info)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    sub.unregister()

if __name__ == "__main__":
    main()