import rospy
import ba_code.srv as basrv
from ba_code.srv import GetObjectList

from geometry_msgs.msg import PointStamped, Point

from ba.navigation.robot import RobotMover
from ba.autonomy.object_collector import ObjectCollector
from ba.navigation.explorer import NodeExplorer
from ba.autonomy.object_collector import STATIONNAMESPACE
from ba.tracking.object_tracker import TRACKERNAMESPACE

#from ba.utilities.singletons.robot_mover_singleton import get_robot_mover

class IRoutine:
    def __init__(self,FSM):
        self.__FSM = FSM

    @property
    def FSM(self):
        return self.__FSM

    def execute(self):
        return self
    
    #def __del__(self):
    #    try:
    #        get_robot_mover().cancel()
    #    except Exception as e:
    #        print(e)

class CollectGarbageRoutine(IRoutine):
    """Routine that will collect trash that has been detected. If no trash is available -> transition to ExploreRegionRoutine"""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__object_collector = ObjectCollector()
        self.__storage_full = False

    def execute(self):
        running = self.__object_collector.follow_plan()

        if self.__storage_full:
            print("State transition -> EmptyStorageRoutine")
            return EmptyStorageRoutine(self.FSM)
        elif running:
            return self
        else: # there is no object to collect -> transition to the exploration
            print("State transition -> ExploreRegionRoutine")
            return ExploreRegionRoutine(self.FSM)

class EmptyStorageRoutine(IRoutine):
    """Routine that will move the robot to the trash collection area and empty its storage.
    !!! Routine and functionalities not implemented yet !!! -> Transition to CollectGarbageRoutine to prevent endless loop.
    !!! trash collection area not yet defined -> robot will move to origin of the map instead. !!!"""
    def __init__(self, FSM):
        super().__init__(FSM)

    def execute(self):
        print("EmptyStorageRoutine not implemented yet.")
        print("State transition -> CollectGarbageRoutine")
        return CollectGarbageRoutine()

class ExploreRegionRoutine(IRoutine):
    """Routine that will command the robot to explore the area. Transition to CollectGarbageRoutine as soon as trash has been detected."""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__explorer = NodeExplorer()
        self.__object_tracker_request = rospy.ServiceProxy(f"/{TRACKERNAMESPACE}/list",GetObjectList)

    def execute(self):
        garbage_detected = len(self.__object_tracker_request().objects) > 0
        if garbage_detected:
            print("State transition -> CollectGarbageRoutine")
            return CollectGarbageRoutine(self.FSM)

        self.__explorer.explore()
        return self

class ShutdownRoutine(IRoutine):
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
            print(f"Goal reached. Shutting down now...")
            rospy.signal_shutdown("Shutdown requested.")
        elif state in [1]: # 1 := PENDING
            pass
        else:
            raise Exception("Invalid state recognized in the shutdown routine. A state has been reached by the move_base action server that is currently not handled by the ShutdownRoutine.")

        return self

class ChargeBatteryRoutine(IRoutine):
    """Routine that commands the robot to move to the designated battery charging station.
    !!! Not yet implemented -> the robot will move to the origin of the map. !!!"""
    def __init__(self, FSM):
        super().__init__(FSM)
        self.__mover = RobotMover()

    def execute(self):
        """Handles the ChargeBatteryRoutine. This routine moves the robot to the origin of the map, which is expected to be the location of the charging station. After reaching the location, the robot will remain at that station until charged.
        Further development neede:
        * battery readings are not implemented yet"""
        if not self.__battery_low():
            print("State transition -> ExploreRegionRoutine")
            return ExploreRegionRoutine(self.FSM)

        try:
            state = self.__mover.get_actionclient().get_state()
        except:
            state = 9

        print(f"State: {state}")
        if state in [2,9]: # 9 := LOST -> goal has been lost, 2 := ACCEPTED -> another goal was excepted
            pnt = PointStamped(point=Point(0,0,0))
            pnt.header.frame_id = "map"
            print(f"Publish new goal: {pnt.point} of frame {pnt.header.frame_id}")
            self.__mover.move_to_point(pnt)
        elif state in [3,8]: # 3 := RECALLED, 8 := SUCCESS
            print(f"Goal reached... charging not implemented yet... Waiting for further instructions...")
        elif state in [1]: # 1 := PENDING
            print(f"Running: {state}")
        else:
            raise Exception("Invalid state recognized in the shutdown routine. A state has been reached by the move_base action server that is currently not handled by the ShutdownRoutine.")

        return self
    
    def __battery_low(self):
        """Reading battery voltage not imlemented yet."""
        return False