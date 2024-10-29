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
from ba.navigation.robot import RobotMover
class ShutdownRoutine(iroutine.IRoutine):
    """Routine to move the robot to the starting location and forces the ros node to shutdown.
    !!! starting location not yet defined -> robot will move to origin of the map instead. !!!"""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__mover = RobotMover()

    def execute(self):
        """Handles the ShutdownRoutine. The routine moves the robot to the starting location. After reaching the starting location, the state machine is shutdown.
        Further development neede:
        * all other nodes should be shutdown aswell -> currently not implemented
        * some states of the move_base action client are currently not handled
        !!! starting location not yet defined -> robot will move to the origin of the map instead. !!!"""
        try:
            state = self.__mover.get_actionclient().get_state()
        except:
            state = 9

        if state in [2,9]: # 9 := LOST -> goal has been lost, 2 := ACCEPTED -> another goal was excepted
            get_station_pose = f"/{STATIONNAMESPACE}/pose"
            rospy.wait_for_service(get_station_pose)
            station_pose_request = rospy.ServiceProxy(get_station_pose,basrv.GetPoseStamped)
            station_pose = station_pose_request().pose
            self.__mover.move_to_pose(station_pose)
        elif state in [3,8]: # 3 := RECALLED, 8 := SUCCESS
            LOGGER.info(f"Goal reached. Shutting down now...")
            rospy.signal_shutdown("Shutdown requested.")
        elif state in [1]: # 1 := PENDING
            pass
        else:
            raise Exception("Invalid state recognized in the shutdown routine. A state has been reached by the move_base action server that is currently not handled by the ShutdownRoutine.")

        return self