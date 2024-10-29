from ba.navigation.robot import RobotMover

from geometry_msgs.msg import PointStamped, Point

import ba.autonomy.routines.iroutine as iroutine

class ChargeBatteryRoutine(iroutine.IRoutine):
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
            return iroutine.ExploreRegionRoutine(self.FSM)

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