import rospy
import math

from copy import deepcopy

from geometry_msgs.msg import PoseStamped,Quaternion
from visualization_msgs.msg import MarkerArray

from ba_code.srv import GetPoseStamped, GetPoseStampedResponse

from ba.utilities.markers import VectorMarker

from ba.utilities.transformations import quaternion_from_euler, quaternion_msg_multiply, unwrap_quaternion_msg

from tf.transformations import euler_from_quaternion

STATIONNS = "station"
class StationNode:
    def __init__(self) -> None:
        self._pose = PoseStamped()
        self._pose.header.frame_id = "map"
        self._pose.header.stamp = rospy.Time.now()
        self._pose.pose.orientation.w = 1
        self._pose_visualizer = rospy.Publisher(f"{STATIONNS}/pose/markers",MarkerArray,queue_size=1)
        self._pose_setter = rospy.Subscriber(f"{STATIONNS}/pose",PoseStamped,self._set_pose)
        self._pose_getter = rospy.Service(f"{STATIONNS}/pose",GetPoseStamped,self._get_pose)
        self.update()

    def _set_pose(self,msg):
        self._pose = msg
        #self._pose.pose.orientation = quaternion_from_euler((0,0,180))
        self.update()

    def _get_pose(self,request):
        resp = GetPoseStampedResponse()
        self._pose.header.stamp = rospy.Time.now()
        resp.pose = self._pose
        return resp
    
    def _visualize(self):
        station = VectorMarker(pose=deepcopy(self._pose),mid=3000,rgb=(200,0,200)).data

        marr = MarkerArray()
        marr.markers = [station]
        self._pose_visualizer.publish(marr)

    def update(self):
        self._visualize()

if __name__ == "__main__":
    rospy.init_node(f"{STATIONNS}_node")

    station = StationNode()

    r = 2
    rate = rospy.Rate(r)
    rospy.loginfo("Update rate: " + str(r) + "s")
    while not rospy.is_shutdown():
        try:
            station.update()
            rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)