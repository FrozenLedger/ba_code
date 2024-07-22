import rospy

from ba.navigation.robot import RobotMover

__ROBOT_NS = "RobotMover"
__ROBOT = None
def get_robot_mover():
    global __ROBOT
    if __ROBOT is None:
        print(f"Initialize {__ROBOT_NS}...")
        try:
            __ROBOT = RobotMover()
            print(f"{__ROBOT_NS} initialized:\t{__ROBOT}")
        except rospy.exceptions.ROSInitException as e:
            print(e)
            print("Initialization failed.")
            return None
    return __ROBOT