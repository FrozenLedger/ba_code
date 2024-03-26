import rospy

from dataclasses import dataclass

from ba.navigation.path_explorer import Pose
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

@dataclass
class Position:
    x: float = 0
    y: float = 0
    z: float = 0

@dataclass
class Orientation:
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 1

@dataclass
class Pose:
    position: Position
    orientation: Orientation

def convert_to_ros_pose(pose: Pose):
    pos: Position = pose.position
    orient: Orientation = pose.orientation

    return ROSPose(
        position=Point(x=pos.x,y=pos.y,z=pos.z),
        orientation=Quaternion(x=orient.x,y=orient.y,z=orient.z,w=orient.w)
    )

def convert_to_ros_posestamped(pose: Pose,frame_id: str):
    pose = convert_to_ros_pose(pose)
    header = Header(frame_id=frame_id,stamp=rospy.Time.now())

    return PoseStamped(header=header,pose=pose)

if __name__ == "__main__":
    rospy.init_node("testnode")
    p = Pose(position=Position(),orientation=Orientation())

    rosp = ROSPose()
    print(p)
    print(rosp)

    merged: ROSPose = convert_to_ros_pose(p)
    print(merged)
    pub = rospy.Publisher("test",ROSPose,queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(merged)