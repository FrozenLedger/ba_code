from ba.navigation.path_explorer import PathExplorer, PosePath
from ba.navigation.robot import RobotMover

from geometry_msgs.msg import PointStamped, PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

from ba.utilities.ros_conversions import convert_to_ros_posestamped
from ba.utilities.datatypes import Pose, Position, Orientation
from ba.navigation.explorer import EXPLORERNS
from ba.utilities.markers import VectorMarker

from visualization_msgs.msg import MarkerArray

import rospy

WAYPOINTSNS = "waypoints"
WAYPOSESNS = "wayposes"

class PathExplorerNode(PathExplorer):
    def __init__(self, robot: RobotMover, path: PosePath):
        super().__init__(robot, path)

        self._clear_waypoints(None)
        self._clear_wayposes(None)

        self._init_subscribers()
        self._init_services()
        self._init_publishers()

    def _init_subscribers(self):
        rospy.Subscriber(f"/{EXPLORERNS}/{WAYPOINTSNS}/add",PointStamped,self._add_point)
        rospy.Subscriber(f"/{EXPLORERNS}/{WAYPOSESNS}/add",PoseStamped,self._add_pose)

    def _init_services(self):
        rospy.Service(f"/{EXPLORERNS}/{WAYPOINTSNS}/clear",Trigger,self._clear_waypoints)
        rospy.Service(f"/{EXPLORERNS}/{WAYPOINTSNS}/apply",Trigger,self._apply_waypoints)
        rospy.Service(f"/{EXPLORERNS}/{WAYPOSESNS}/clear",Trigger,self._clear_wayposes)
        rospy.Service(f"/{EXPLORERNS}/{WAYPOSESNS}/apply",Trigger,self._apply_wayposes)
        rospy.Service(f"/{EXPLORERNS}/explore",Trigger,self._explore)
        
    def _init_publishers(self):
        self._marker_publisher = rospy.Publisher(f"{EXPLORERNS}/{WAYPOSESNS}/visualization",MarkerArray,queue_size=10)

    def _explore(self, request):
        self.explore()
        return TriggerResponse(success=True)

    def _clear_waypoints(self, request):
        self._waypoints = []
        return TriggerResponse(success=True)
    
    def _clear_wayposes(self, request):
        self._wayposes = []
        return TriggerResponse(success=True)
    
    def _apply_waypoints(self, request):
        points = [(pnt.x,pnt.y) for pnt in self._waypoints]
        poses = [convert_to_ros_posestamped(Pose(position=Position(x=x,y=y,z=0),
                          orientation=Orientation()),"map") for x,y in points]
        
        self.path.set_poses(poses)
        
        return TriggerResponse(success=True)
    
    def _apply_wayposes(self, request):
        ts = rospy.Time.now()
        
        for waypose in self._wayposes:
            waypose.header.stamp = ts
        
        self._path.set_poses(self._wayposes)

        markers = [VectorMarker(pose=pose,mid=2000+idx).data for idx,pose in enumerate(self._wayposes)]
        marker_array = MarkerArray(markers=markers)
        self._marker_publisher.publish(marker_array)
        
        return TriggerResponse(success=True)

    def _add_point(self, msg):
        self._waypoints.append(msg.point)

    def _add_pose(self, msg):
        self._wayposes.append(msg)

if __name__ == "__main__":
    nodename = f"{EXPLORERNS}_node"
    rospy.init_node(nodename)
    points = [(0,0),(1,0),(0,1),(1,1)]
    poses = [convert_to_ros_posestamped(Pose(position=Position(x=x,y=y,z=0),
                          orientation=Orientation()),"map") for x,y in points]
    posepath = PosePath(poses=poses)
    explorer = PathExplorerNode(RobotMover(),posepath)
    rospy.loginfo(nodename + " running.")

    rospy.spin()