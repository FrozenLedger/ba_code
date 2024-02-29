import rospy,tf,math,actionlib

from geometry_msgs.msg import PoseStamped, Pose,PointStamped

from ba.utilities.transformations import quaternion_from_euler

from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import GoalID

class RobotMover:
    """A class that simplifies the interaction with the 'move_base' package for controlling the robot movement."""
    def __init__(self,world="map",base="base_link"):
        self.__world = world
        self.__base = base
        self.__tf_listener = tf.TransformListener()
        self.__move_base_client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        t0 = rospy.Time.now()
        rospy.sleep(0.25)
        #self.__tf_listener.waitForTransform(world,base,t0,rospy.Duration(1))

    def rotate(self,angle):
        if angle < 0:
            angle = max(angle,-math.pi)
        elif angle > 0:
            angle = min(angle,math.pi)
        else:
            return
        
        pose = Pose()
        pose.orientation = quaternion_from_euler((0,0,angle))
        ps = self.__transform(pose)
        self.move_to_pose(ps)

    def translate(self,distance):
        pose = Pose()
        pose.position.x = distance
        ps = self.__transform(pose)
        self.move_to_pose(ps)

    def move_to_pose(self,pose):
        self.__move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.__move_base_client.send_goal(goal)
        
    def move_to_point(self,point,distance=0):
        t0 = rospy.Time.now()
        point.header.stamp = t0
        target_frame = point.header.frame_id
        self.__tf_listener.waitForTransform(self.__base,target_frame,t0,rospy.Duration(0.25))
        pntst = self.__tf_listener.transformPoint(self.__base,point)

        alpha = math.atan2(pntst.point.y, pntst.point.x)
        x = math.cos(alpha)*distance
        y = math.sin(alpha)*distance

        ps = PoseStamped()
        ps.pose.position = pntst.point
        ps.header.frame_id = self.__base
        ps.header.stamp = t0
        ps.pose.position.x -= x
        ps.pose.position.y -= y
        ps.pose.orientation = quaternion_from_euler((0,0,alpha))

        self.__tf_listener.waitForTransform(self.__base,self.__world,t0,rospy.Duration(0.25))
        ps = self.__tf_listener.transformPose(self.__world,ps)

        self.move_to_pose(ps)

    def __transform(self,pose):
        ps = PoseStamped()
        ps.header.frame_id = self.__base
        ps.pose = pose
        t0 = rospy.Time.now()
        self.__tf_listener.waitForTransform(self.__world,self.__base,t0,rospy.Duration(0.25))
        return self.__tf_listener.transformPose(self.__world,ps)

    def wait_for_result(self):
        return self.__move_base_client.wait_for_result()
    
    def get_actionclient(self):
        return self.__move_base_client
    
    @property
    def pose(self):
        return self.__transform(Pose())

def main():
    rospy.init_node("robot_mover")
    mover = RobotMover()
    mover.rotate(math.pi)
    #rospy.spin()

if __name__ == "__main__":
    main()