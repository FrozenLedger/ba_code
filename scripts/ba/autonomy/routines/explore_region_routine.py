import rospy
import ba_code.srv as basrv
from ba_code.srv import GetObjectList

from geometry_msgs.msg import PointStamped, Point

from ba.autonomy.object_collector import ObjectCollector
from ba.navigation.explorer import NodeExplorer
from ba.autonomy.object_collector import STATIONNAMESPACE
from ba.tracking.object_tracker import TRACKERNAMESPACE

import ba.autonomy.routines.iroutine as iroutine

#from ba.utilities.singletons.robot_mover_singleton import get_robot_mover
class ExploreRegionRoutine(iroutine.IRoutine):
    """Routine that will command the robot to explore the area. Transition to CollectGarbageRoutine as soon as trash has been detected."""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__explorer = NodeExplorer()
        self.__object_tracker_request = rospy.ServiceProxy(f"/{TRACKERNAMESPACE}/list",GetObjectList)

    def execute(self):
        garbage_detected = len(self.__object_tracker_request().objects) > 0
        if garbage_detected:
            print("State transition -> CollectGarbageRoutine")
            return iroutine.CollectGarbageRoutine(self.FSM)

        self.__explorer.explore()
        return self