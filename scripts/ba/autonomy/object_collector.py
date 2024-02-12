import rospy, actionlib

from ba_code.srv import GetTrackedObjects,RemoveObject

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Point

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

        self.__init_server_proxies()

    def __init_server_proxies(self):
        self.__object_tracker_sub = rospy.ServiceProxy("/object_tracker/tracked",GetTrackedObjects)

    def __follow_plan(self):
        obj_srv = self.__object_tracker_sub()
        objects = obj_srv.objects
        keys = obj_srv.keys

        if len(keys) == 0:
            print("There are no objects to collect...")
        else:
            print("Create plan...")
            self.__collect_object(objects[-1],keys[-1])

    def __collect_object(self,obj,key):
        pnt = obj.point.point
        print(f"Move to object at position: {pnt}")
        move_to_point(pnt)
        print(f"Send /object_tracker/remove {key} request...")
        unregister = rospy.ServiceProxy("/object_tracker/remove",RemoveObject)
        res = unregister(key=key)
        print(f"Result: {res}")

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