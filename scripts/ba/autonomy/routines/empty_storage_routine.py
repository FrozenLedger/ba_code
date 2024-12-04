import rospy
import ba_code.srv as basrv
from ba_code.srv import GetObjectList

from geometry_msgs.msg import PointStamped, Point

from ba.autonomy.object_collector import ObjectCollector
from ba.navigation.explorer import NodeExplorer
from ba.autonomy.object_collector import STATIONNAMESPACE
from ba.tracking.object_tracker import TRACKERNAMESPACE

import ba.autonomy.routines.iroutine as iroutine
from ba.autonomy.autonomy_logger import AUTONOMYLOGGER as LOGGER

#from ba.utilities.singletons.robot_mover_singleton import get_robot_mover
class EmptyStorageRoutine(iroutine.IRoutine):
    """Routine that will move the robot to the trash collection area and empty its storage.
    !!! Routine and functionalities not implemented yet !!! -> Transition to CollectGarbageRoutine to prevent endless loop.
    !!! trash collection area not yet defined -> robot will move to origin of the map instead. !!!"""
    def __init__(self, FSM):
        super().__init__(FSM)

    def execute(self):
        LOGGER.info("EmptyStorageRoutine not implemented yet.")
        LOGGER.info("State transition -> CollectGarbageRoutine")
        return iroutine.CollectGarbageRoutine()