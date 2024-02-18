import rospy

from ba_code.manipulator.joints_controller import JointsController

def main():
    rospy.init_node("joints_controller")
    controller = JointsController()

if __name__ == "__main__":
    main()