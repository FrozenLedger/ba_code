# based on: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html 
# and: https://github.com/DougUOW/om_moveit_examples/blob/master/src/execute_trajectory.py

import rospy,sys,time
import moveit_commander
import moveit_msgs.msg
#import geometry_msgs.msg

class JointsController:
    def __init__(self):
        self.__arm = moveit_commander.MoveGroupCommander("arm")
        self.__gripper = moveit_commander.MoveGroupCommander("gripper")
    
    def move_arm(self,values):
        _move_joints(self.__arm,values)

    def move_gripper(self,values):
        _move_joints(self.__gripper,values)

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

if __name__ == "__main__":
    __main()