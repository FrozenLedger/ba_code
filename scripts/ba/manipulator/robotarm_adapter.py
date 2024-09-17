import rospy
import time
from magni_dxl_module.robotarm_controller import RobotarmController
from ba.manipulator.robot_pose_publisher import pickupInstructions

class RobotarmAdapter:
    @property
    def controller(self) -> RobotarmController:
        return self._robotarm

    def __init__(self):
        self._robotarm: RobotarmController = RobotarmController()
        self._robotarm.enable_torque(True)

    def run_pickup_instructions(self):
        self.__run_movement(self._robotarm.pickup_instructions)

    def run_drop_instructions(self):
        self.__run_movement(self._robotarm.drop_instructions)

    def __run_movement(self, instructions):
        for inst in instructions:
            self._robotarm.set_positions_deg(inst)
            time.sleep(3)

if __name__ == "__main__":
    import time
    rospy.init_node("robotarm_node")
    robotarm = RobotarmAdapter()
    robotarm.run_pickup_instructions()
    time.sleep(2)
    robotarm.run_drop_instructions()
    robotarm.controller.enable_torque(False)
