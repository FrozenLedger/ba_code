import rospy, actionlib, math

import ba_code.srv as basrv

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point,PoseStamped, PointStamped

from ba.navigation.robot import RobotMover
from ba.utilities.transformations import quaternion_from_euler
#from tf.transformations import quaternion_from_euler

from ba.manipulator.robot_pose_publisher import RobotarmPosePublisher,pickupInstructions,dropInstructions
from ba.tracking.object_tracker import TRACKERNAMESPACE

STATIONNAMESPACE = "station"

def move_to_point(pnt: Point):
    """Uses the 'move_base' Node to move to a specified point in the 'map frame' with the orientation set to (0,0,0,1) in quaternions.
Code based on: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/"""
    #based on: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    client: actionlib.SimpleActionClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal: MoveBaseGoal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position = pnt
    
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait: bool = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

class ObjectCollector:
    """This node will controll the robot to fullfill the task of collecting objects that have been found in the area."""
    def __init__(self):
        self.__rate: rospy.Rate = rospy.Rate(0.2) # update every 5s
        self.__mover: RobotMover = RobotMover()
        self.__robotarm: RobotarmPosePublisher = RobotarmPosePublisher()
        self.__init_server_proxies()

        self.__substate = self.__look_for_object

    def __init_server_proxies(self):
        self.__pop_req = rospy.ServiceProxy(f"/{TRACKERNAMESPACE}/pop",basrv.PopObject)
        self.__station_pose_req = rospy.ServiceProxy(f"/{STATIONNAMESPACE}/pose",basrv.GetPoseStamped)

    def follow_plan(self):
        return self.__substate()
    
    def __print_state_execution(self,statename):
        print(f"ObjectCollector state execution: {statename}")
    
    def __look_for_object(self):
        self.__print_state_execution("look for object")
        try:
            obj = self.__pop_req().object
            if obj.note.data == "empty":
                print("There are no objects to collect...")
                return False
            elif obj.clsID not in [1,2,3]:
                print(f"Object of clsID: {obj.clsID} not collectible. Skip.")
            else:
                print("Object found...")
                self.__target_object = obj
                self.__substate = self.__move_to_object # set next substate
            return True
        except Exception as e:
            print(e)
        return False
    
    def __move_to_object(self):
        self.__print_state_execution("move to object")
        try:
            obj = self.__target_object
            actionstate = self.__mover.get_actionclient().get_state()
            print(f"move_base ActionState: {actionstate}")

            if actionstate == 1:
                print("Waiting for robot to reach goal...")
                return True
            elif actionstate in [0,9]:
                print(f"Move to object at position: {obj.point.point}")
                self.__mover.move_to_point(obj.point,distance=0.75)
            elif actionstate == 3:
                print(f"Position reached.")
                self.__substate = self.__approach_object
            else:
                self.__raise_invalid_action_state_exception()
            return True            
        except Exception as e:
            print(e)
        return False
    
    def __approach_object(self):
        self.__print_state_execution("approach object")
        try:
            self.__mover.move_to_point(self.__target_object.point,distance=0.5)
            self.__mover.wait_for_result()

            self.__substate = self.__pickup_object
            return True
        except Exception as e:
            print(e)
        return False

    def __pickup_object(self):
        self.__print_state_execution("pickup object")
        try:
            print("Start pickup routine.")
            self.__start_pickup_routine()
            rospy.sleep(.5)

            self.__substate = self.__move_to_trash_area
            return True
        except Exception as e:
            print(e)
        return False

    def __move_to_trash_area(self):
        self.__print_state_execution("move to trash area")
        try:
            print("Move to position infront of station.")
            #self.__trasharea.header.stamp = rospy.Time.now()
            station_pose = self.__station_pose_req().pose
            station_pose.header.stamp = rospy.Time.now()
            self.__mover.move_to_point(
                point=PointStamped(
                    #header=self.__trasharea.header,
                    #point=self.__trasharea.pose.position
                    header=station_pose.header,
                    point=station_pose.pose.position
                ),
                distance=1
            )
            self.__mover.wait_for_result()
        
            rospy.sleep(.5)
            print("Move to trash collection area.")
            #self.__trasharea.header.stamp = rospy.Time.now()
            station_pose.header.stamp = rospy.Time.now()
            self.__mover.move_to_pose(station_pose) #self.__trasharea)
            self.__mover.wait_for_result()

            self.__substate = self.__drop_trash
            return True
        except Exception as e:
            print(e)
        return False
    
    def __drop_trash(self):
        self.__print_state_execution("drop trash")
        try:
            self.__start_drop_routine()
            rospy.sleep(0.5)

            self.__substate = self.__look_for_object
            return True
        except Exception as e:
            print(e)
        return False

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

    def __raise_invalid_action_state_exception(self):
        raise Exception("The action client reached a state that has not been handled by the ObjectCollector yet.")

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