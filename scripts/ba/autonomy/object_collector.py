import rospy, actionlib, math

import ba_code.srv as basrv

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point,PoseStamped

from ba.navigation.robot import RobotMover
from ba.utilities.transformations import quaternion_from_euler
#from tf.transformations import quaternion_from_euler

from ba.manipulator.robot_pose_publisher import RobotarmPosePublisher,pickupInstructions,dropInstructions

def move_to_point(pnt:Point):
    """Uses the 'move_base' Node to move to a specified point in the 'map frame' with the orientation set to (0,0,0,1) in quaternions.
Code based on: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/"""
    #based on: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position = pnt
    
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

class ObjectCollector:
    """This node will controll the robot to fullfill the task of collecting objects that have been found in the area."""
    def __init__(self):
        self.__rate = rospy.Rate(0.2) # update every 5s
        self.__mover = RobotMover()
        self.__robotarm = RobotarmPosePublisher()
        self.__init_server_proxies()

    def __init_server_proxies(self):
        self.__pop_req = rospy.ServiceProxy("/object_tracker/pop",basrv.PopObject)

    def follow_plan(self):
        obj = self.__pop_req().object

        if obj.note.data == "empty":
            print("There are no objects to collect...")
        elif obj.clsID not in [1,2,3]:
            print(f"Object of clsID: {obj.clsID} not collectible. Skip.")
        else:
            print("Create plan...")
            self.__collect_object(obj)

    def __collect_object(self,obj):
        #pnt = obj.point.point
        print(f"Move to object at position: {obj.point.point}")
        #move_to_point(pnt)
        self.__mover.move_to_point(obj.point,distance=0.75)
        self.__mover.wait_for_result()
        self.__mover.move_to_point(obj.point,distance=0.5)
        self.__mover.wait_for_result()

        print("Start pickup routine.")
        self.__start_pickup_routine()
        rospy.sleep(.5)
        trasharea = PoseStamped()
        trasharea.header.frame_id = "map"
        trasharea.header.stamp = rospy.Time.now()
        trasharea.pose.orientation = quaternion_from_euler((0,0,math.pi))
        
        print("Move to position infront of trash collection area.")
        trasharea.pose.position.x = 1
        self.__mover.move_to_pose(trasharea)
        
        rospy.sleep(.5)
        print("Move to trash collection area.")
        trasharea.pose.position.x = 0
        self.__mover.move_to_pose(trasharea)
        
        rospy.sleep(.5)
        self.__start_drop_routine()

        rospy.sleep(.5)
        rospy.loginfo("Finished trash delivery.")

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.follow_plan()
            except Exception as e:
                print(e)
            self.__rate.sleep()

    def __start_pickup_routine(self):
        self.__robotarm.publish(pickupInstructions())
    def __start_drop_routine(self):
        self.__robotarm.publish(dropInstructions())

def main():
    rospy.init_node("object_collector")
    collector = ObjectCollector()
    collector.loop()

def main2():
    """For testing purposes."""
    try:
        rospy.init_node('movebase_client_py')
        result = move_to_point(Point())
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

if __name__ == "__main__":
    main()