#!/usr/bin/env python

from time import sleep
import rospy
from ba_code.msg import JointAngles
from math import radians
import sys

from moveit_msgs.msg import MoveGroupFeedback

def manual_pose_publisher():
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('joint_angles_topic', JointAngles, queue_size=10)

    while not rospy.is_shutdown():
        try:
            angles_input: str = input("Enter joint angles separated by spaces(in degrees): ")
            angles_list: list[float] = [radians(float(angle)) for angle in angles_input.split()]
            
            gripper_input: float = float(input("Enter gripper command (1 for open, 0 for close): "))

            joint_angles_msg: JointAngles = JointAngles()
            joint_angles_msg.angles = angles_list
            joint_angles_msg.gripper_command = gripper_input

            pub.publish(joint_angles_msg)
        except Exception as e:
            rospy.logwarn("Error in input: %s", e)

class RobotarmPosePublisher:
    """A class to control the angles of the joints of the robot arm, based on the 'move_to_pose_package by Mouheb Khairallah'."""
    def __init__(self,init_node: bool=False,delay: float=3):
        if init_node:
            self.__node = rospy.init_node('pose_publisher',anonymous=True)
        self.__publisher = rospy.Publisher('joint_angles_topic', JointAngles, queue_size=10)
        self.__delay = delay

    def publish_unsafe(self,instructions):
        """Takes a list of float values."""
        print("Instructions in degrees:",instructions)
        for inst in instructions:
            if rospy.is_shutdown():
                break
            #input("Press any key to continue...")
            sleep(self.__delay)
            pose = inst[0]
            gripper_command = inst[1]
            print("Pose:",pose)
            print("gripper_command:",gripper_command)

            rads = [radians(float(d)) for d in pose]
            joint_angles_msg = JointAngles()
            joint_angles_msg.angles = rads #angles_list
            joint_angles_msg.gripper_command = float(gripper_command) #gripper_input
            self.__publisher.publish(joint_angles_msg)

    def publish(self,instructions):
        print("Instructions in degrees:",instructions)
        for inst in instructions:
            if rospy.is_shutdown():
                break
            #input("Press any key to continue...")
            rospy.sleep(self.__delay)
            pose = inst[0]
            gripper_command = inst[1]
            print("Pose:",pose)
            print("gripper_command:",gripper_command)

            rads = [radians(float(d)) for d in pose]
            joint_angles_msg = JointAngles()
            joint_angles_msg.angles = rads #angles_list
            joint_angles_msg.gripper_command = float(gripper_command) #gripper_input
            self.__publisher.publish(joint_angles_msg)
            
def manual_publisher():
    try:
        manual_pose_publisher()
    except rospy.ROSInterruptException:
        pass

#def getInstructions():
#    s0_8 = ((0,15,60,-45),0)
#    s1_7 = ((90,15,60,-45),0)
#    s2_6 = ((90,0,0,-60),0)#((90,55,-45,-90),0)
#    s3_5 = (s2_6[0],1)
#    s_4 = ((90,70,-45,-90),1)
    
#    steps = [s0_8,
#             s1_7,
#             s2_6,
#             s3_5,
#             s_4,
#             s3_5,
#             s2_6,
#             s1_7,
#             s0_8]

#    return steps

def pickupInstructions():
    """Returns the path to execute the pickup movement of the robotarm."""
    p0 = (0,15,60,-45)
    p1 = (20,15,60,-45)
    p2 = (0,-45,35,-85)
    p3 = (-90,-45,35,-85)
    p4 = (-90,-45,35,-75)
    p5 = (-90,-30,0,-30)
    p6 = (-90,-15,-45,0)
    #p7 = (-90,-90,0,-70)
    #p7 = (-90,-45,-45,0)

    O = -0.006
    C = 0.006

    steps = [(p0,C),
             (p1,C),
             (p2,C),
             (p3,C),
             (p4,O),
             (p5,O),
             (p6,O),
             (p6,C),
             (p5,C),
             (p4,C),
             #(p7,C),
             #(p7,O),
             #(p4,C),
             (p3,C),
             (p2,C),
             (p1,C),
             (p0,C)]

    return steps    

def dropInstructions():
    """Returns the path to execute the drop movement of the robotarm."""
    p0 = (0,15,60,-45)
    p1 = (20,15,60,-45)
    p2 = (90,0,-45,-45)
    #p2 = (0, -45,35,-85)
    #p3 = (-90,-45,35,-85)
    #p4 = (-90,-65,-45,45)

    O = -0.006
    C = 0.006

    steps = [(p0,C),
             (p1,C),
             (p2,C),
             (p2,O),
             (p1,C),
             (p0,C)]
    
    return steps

def testInstructions():
    """Returns a path to test the robotarm movement."""
    p0 = (0,15,60,-45)
    p1 = (20, 0, 0, 0)

    C = 0.006
    O = -0.006
    return [(p0,C),(p1,C),(p1,O),(p0,C)]

if __name__ == '__main__':
    try:
        opt = sys.argv[1]
    except:
        opt= ""

    if opt == "--help" or opt == "-h":
        print("--publisher or -p to use publisher")
        print("--prog or -r for program")
    elif opt == "--publisher" or opt == "-p":
        manual_publisher()
    else:
        rospy.sleep(1)
        publisher = RobotarmPosePublisher(init_node=True,delay=1)
        #publisher.publish(testInstructions())
        rospy.sleep(1)
        publisher.publish(pickupInstructions())#,rospy.Rate(0.25))
        rospy.sleep(1)
        publisher.publish(dropInstructions())