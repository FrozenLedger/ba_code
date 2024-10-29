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
class CollectGarbageRoutine(iroutine.IRoutine):
    """Routine that will collect trash that has been detected. If no trash is available -> transition to ExploreRegionRoutine"""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__object_collector = ObjectCollector()
        self.__storage_full = False

    def execute(self):
        running = self.__object_collector.follow_plan()

        if self.__storage_full:
            print("State transition -> EmptyStorageRoutine")
            return iroutine.EmptyStorageRoutine(self.FSM)
        elif running:
            return self
        else: # there is no object to collect -> transition to the exploration
            print("State transition -> ExploreRegionRoutine")
            return iroutine.ExploreRegionRoutine(self.FSM)