#!/usr/bin/env python

from time import sleep
import rospy
from ba_code.msg import JointAngles
from math import radians
import sys

def manual_pose_publisher():
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('joint_angles_topic', JointAngles, queue_size=10)

    while not rospy.is_shutdown():
        try:
            angles_input = input("Enter joint angles separated by spaces(in degrees): ")
            angles_list = [radians(float(angle)) for angle in angles_input.split()]
            
            gripper_input = float(input("Enter gripper command (1 for open, 0 for close): "))

            joint_angles_msg = JointAngles()
            joint_angles_msg.angles = angles_list
            joint_angles_msg.gripper_command = gripper_input

            pub.publish(joint_angles_msg)
        except Exception as e:
            rospy.logwarn("Error in input: %s", e)

class RobotarmPosePublisher:
    def __init__(self,init_node=False,delay=0):
        if init_node:
            self.__node = rospy.init_node('pose_publisher',anonymous=True)
        self.__publisher = rospy.Publisher('joint_angles_topic', JointAngles, queue_size=10)
        self.__delay = delay

    def publish(self,instructions):
        print("Instructions:",instructions)
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
            
def manual_publisher():
    try:
        manual_pose_publisher()
    except rospy.ROSInterruptException:
        pass

def getInstructions():
    s0_8 = ((0,15,60,-45),0)
    s1_7 = ((90,15,60,-45),0)
    s2_6 = ((90,0,0,-60),0)#((90,55,-45,-90),0)
    s3_5 = (s2_6[0],1)
    s_4 = ((90,70,-45,-90),1)
    
    steps = [s0_8,
             s1_7,
             s2_6,
             s3_5,
             s_4,
             s3_5,
             s2_6,
             s1_7,
             s0_8]

    return steps

if __name__ == '__main__':
    opt = sys.argv[1]

    if opt == "--help" or opt == "-h":
        print("--publisher or -p to use publisher")
        print("--prog or -r for program")
    elif opt == "--publisher" or opt == "-p":
        manual_publisher()
    elif opt == "--program" or opt == "-r":
        publisher = RobotarmPosePublisher(init_node=True,delay=4)
        publisher.publish(getInstructions())#,rospy.Rate(0.25))
    else:
        manual_publisher()