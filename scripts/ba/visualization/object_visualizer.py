import rospy

from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PointStamped

import ba_code.srv as basrv

class ObjectVisualizer:
    """A node to visualize the detected objects and trash that are tracked by the ObjectTracker."""
    def __init__(self):
        self.__pub = rospy.Publisher("/object_tracker/visualization",MarkerArray,queue_size=10)
        self.__objects_req = rospy.ServiceProxy("/object_tracker/list",basrv.GetObjectList)
        self.__rate = rospy.Rate(0.2)

    def __publish_marker(self,markers):
        arr = []
        mid = 0
        for idx in range(len(markers)):
            marker = Marker()
            lbl = Marker()

            lbl.text = f"{markers[idx].clsName.data}/{markers[idx].objID}" #f"{markers[idx].note}"
            # obj = markers[idx][0]
            color = (0, markers[idx].clsID ,255)
            alpha = markers[idx].confidence
            #pnt = obj.point.point
            #marker.header.frame_id = "base_link"
            #marker.header.stamp = rospy.Time.now()
            header = markers[idx].point.header
            marker.header = header
            lbl.header = header

            #marker.ns = "basic_shape"
            marker.type = 2 #Marker.CUBE
            marker.id = mid
            
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.id = mid+1000
            
            # increase ID coutner
            mid += 1    

            pos = markers[idx].point.point
            marker.pose.position = pos
            lbl.pose.position = pos
            #marker.pose.position.x = 1

            #print(pnt.point)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 1.50
            lbl.scale.x = 0.05
            lbl.scale.y = 0.05
            lbl.scale.z = 0.05

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = alpha

            lbl.color.r = 1
            lbl.color.g = 0.25
            lbl.color.b = 0.25
            lbl.color.a = alpha
            lbl.pose.orientation.w = 1
            lbl.lifetime = rospy.Duration(5)

            marker.pose.orientation.w = 1
            marker.lifetime = rospy.Duration(5)
            arr.append(marker)
            arr.append(lbl)

        marker_arr = MarkerArray()
        marker_arr.markers = arr
        #marker.lifetime = rospy.Duration(5)
        self.__pub.publish(marker_arr)

    def __publish(self):
        objects = self.__objects_req().objects

        markers = []
        for obj in objects:
            markers.append( obj )
            #pnt = PointStamped(header=resp.header)
            #pnt.point = calcPoint(detection,idx)
            #print(f"Transform: {base_pnt}")
            #pnts.append(base_pnt)

        self.__publish_marker( markers )

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