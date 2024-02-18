import rospy, actionlib

import ba_code.srv as basrv

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

from ba.navigation.robot import RobotMover

#from tf.transformations import quaternion_from_euler

def move_to_point(pnt:Point):
    # based on: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
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
    def __init__(self):
        self.__rate = rospy.Rate(0.2) # update every 5s
        self.__mover = RobotMover()
        self.__init_server_proxies()

    def __init_server_proxies(self):
        self.__pop_req = rospy.ServiceProxy("/object_tracker/pop",basrv.PopObject)

    def __follow_plan(self):
        obj = self.__pop_req().object

        if obj.note.data == "empty":
            print("There are no objects to collect...")
        else:
            print("Create plan...")
            self.__collect_object(obj)

    def __collect_object(self,obj):
        #pnt = obj.point.point
        print(f"Move to object at position: {obj.point.point}")
        #move_to_point(pnt)
        self.__mover.move_to_point(obj.point,distance=1)
        self.__mover.wait_for_result()
        self.__mover.move_to_point(obj.point,distance=0.5)
        self.__mover.wait_for_result()

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.__follow_plan()
            except Exception as e:
                print(e)
            self.__rate.sleep()

def main():
    rospy.init_node("object_collector")

    collector = ObjectCollector()
    collector.loop()

def main2():
    try:
        rospy.init_node('movebase_client_py')
        result = move_to_point(Point())
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

if __name__ == "__main__":
    main()