import rospy
import time

from ba.manipulator.commands.commands import *

rospy.init_node("manipulator_test")

print("Start test...")
time.sleep(1)
print("Enable torque...")
EnableTorqueCommand().execute()
time.sleep(3)
print("Start routine...")
StateTransitionCommand().execute()
time.sleep(5)
print("Reverse routine...")
StateTransitionCommand().execute()
time.sleep(3)
print("Disable torque...")
EnableTorqueCommand().undo()
print("Test finished.")