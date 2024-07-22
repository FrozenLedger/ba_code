import rospy

from ba.autonomy.object_collector import ObjectCollector
from ba.utilities.singletons import get_robot_mover

from geometry_msgs.msg import PointStamped

class RobotRoutine:
    """A routine that controls the autonomous behaviour of the robot to explore a path and track all the objects it is encountering.
The routine will try to collect all found objects after the exploration."""
    def __init__(self):
        self.__object_collector = ObjectCollector()
        self.__mover = get_robot_mover() #RobotMover()
        #self.__scout = ObjectScout()
        #self.__tracker = ObjectTracker()

    def start_routine(self):
        print("Start routine...")
        path = getPath()

        point = PointStamped()
        point.header.frame_id = "map"
        #self.__scout.look_around()
        print("Start exploration...")
        for p in path:
            point.header.stamp = rospy.Time.now()
            point.point.x = p[0]
            point.point.y = p[1]
            print(f"Move to point: {p}")
            self.__mover.move_to_point(point)
            self.__mover.wait_for_result()
        #    self.__scout.look_around()
            self.__object_collector.follow_plan()

        #rospy.sleep(1)
        #print("Start collecting objects...")
        #self.__object_collector.follow_plan() # pickup object if there are any objects to collect

        rospy.sleep(1)
        print("Returning home...")
        point.point.x = 0
        point.point.y = 0
        point.header.stamp = rospy.Time.now()
        self.__mover.move_to_point(point)
        self.__mover.wait_for_result()
        print("End routine...")

def getPath():
    return [(1,0),(1,-1),(0.5,-1),(0.5,0),(0.5,1),(1,1),(1,0),(0,0)]

if __name__ == "__main__":
    rospy.init_node("robot_routine")
    robot = RobotRoutine()
    robot.start_routine()