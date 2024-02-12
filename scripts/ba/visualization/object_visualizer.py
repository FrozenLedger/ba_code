import rospy

from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PointStamped

from ba_code.srv import GetTrackedObjects

class ObjectVisualizer:
    def __init__(self):
        self.__pub = rospy.Publisher("/visualization_marker_array",MarkerArray,queue_size=10)
        self.__object_tracker_service = rospy.ServiceProxy("/object_tracker/tracked",GetTrackedObjects)
        self.__rate = rospy.Rate(0.2)

    def __publish_marker(self,pnt_arr:PointStamped):
        arr = []
        mid = 0
        for pnt in pnt_arr:
            marker = Marker()
            #marker.header.frame_id = "base_link"
            #marker.header.stamp = rospy.Time.now()
            marker.header = pnt.header

            #marker.ns = "basic_shape"
            marker.type = 2 #Marker.CUBE
            marker.id = mid
            mid += 1

            marker.pose.position = pnt.point
            #marker.pose.position.x = 1

            #print(pnt.point)
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            #marker.color.r = 1
            marker.color.b = 1
            marker.color.a = 1

            marker.pose.orientation.w = 1
            marker.lifetime = rospy.Duration(5)
            arr.append(marker)

        marker_arr = MarkerArray()
        marker_arr.markers = arr
        #marker.lifetime = rospy.Duration(5)
        self.__pub.publish(marker_arr)

    def __publish(self):
        objects = self.__object_tracker_service().objects

        pnts = []
        for obj in objects:
            pnts.append(obj.point)
            #pnt = PointStamped(header=resp.header)
            #pnt.point = calcPoint(detection,idx)
            #print(f"Transform: {base_pnt}")
            #pnts.append(base_pnt)

        self.__publish_marker(pnts)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.__publish()
            except rospy.ServiceException as rse:
                print(rse)
            except Exception as e:
                print(e)
            self.__rate.sleep()

def main():
    rospy.init_node("tracked_objects_visualizer")

    object_visualizer = ObjectVisualizer()
    object_visualizer.loop()

if __name__ == "__main__":
    main()