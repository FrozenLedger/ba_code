# based on: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html 
# and: https://github.com/DougUOW/om_moveit_examples/blob/master/src/execute_trajectory.py

import rospy,sys,time
import moveit_commander
import moveit_msgs.msg
#import geometry_msgs.msg

import math
from ba_code.srv import MoveArm

class JointsController:
    """A class to controll the robotarm."""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm = moveit_commander.MoveGroupCommander("arm")
        self.__gripper = moveit_commander.MoveGroupCommander("gripper")

        self.__sub_arm = rospy.Service("/robotarm/move",MoveArm,self.__move_arm)
        self.__sub_arm_deg = rospy.Service("/robotarm/move_deg",MoveArm,self.__move_by_degrees)

    def move_arm(self,values):
        _move_joints(self.__arm,values)

    def move_gripper(self,values):
        _move_joints(self.__gripper,values)

    def __move_arm(self,req):
        angles = req.joints
        self.move_arm(angles)

    def __move_by_degrees(self,req):
        angles = [math.radians(d) for d in req.joints]
        self.move_arm(angles)

### Testing ###
def __main():
    from sys import argv as args

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial",anonymous=True)
    robotarm = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,)

    planning_frame = arm.get_planning_frame()
    eel_link = arm.get_end_effector_link()
    group_names = robotarm.get_group_names()
    print(robotarm.get_current_state())

    if "--arm" in args:
        p0 = [0,0,0,0]
        p1 = [1,0,0,0]

        _move_joints(arm,p1)
        _move_joints(arm,p0)

    if "--gripper" in args:
        g0 = [0.009]
        g1 = [-0.009]

        _move_joints(gripper,g0)
        time.sleep(1)
        _move_joints(gripper,g1)

def _move_joints(move_group,values):
    joint_goal = move_group.get_current_joint_values()

    for idx,v in enumerate(values):
        joint_goal[idx] = v

    move_group.go(joint_goal,wait=True)
    move_group.stop()

def main():
    #rospy.init_node("robotarm_controller")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial",anonymous=True)
    robotarm = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,)

    planning_frame = arm.get_planning_frame()
    eel_link = arm.get_end_effector_link()
    group_names = robotarm.get_group_names()

    rospy.loginfo("robotarm_controller ready.")
    rospy.spin()

def mainII():
    rospy.init_node("robotarm_controller")
    controller = JointsController()
    rospy.spin()

if __name__ == "__main__":
    mainII()
