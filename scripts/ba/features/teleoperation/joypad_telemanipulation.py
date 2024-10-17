import rospy

from sensor_msgs.msg import Joy
from solarswarm2.components.GrabBehaviour import GrabBehaviour
from time import sleep

class JoypadGripper:
    """A node that takes inputs from the /joy topic and sends further control signals to the robotarm to control its grab behaviour."""
    def __init__(self):
        self.grab_behaviour = GrabBehaviour(start_pose=[0,15,60,-45])
        self.subscriber = rospy.Subscriber("/joy",Joy,self.publisher_cb)        
        self.__btns = {"A":True,
                       "B":True,
                       "X":True,
                       "Y":True}

        self.state = 0
        self.gripper_state = 0
        
        sleep(1)
        rospy.loginfo("Start delay...")
        self.grab_behaviour.setState(s0)
        self.grab_behaviour.open_gripper()
        self.grab_behaviour.publish()
        sleep(5)
        rospy.loginfo("JoypadGripper ready.")
        
        #self.loop()
        
    def publisher_cb(self,data):
        #A := buttons[0]
        #B := btns[1]
        #X := btns[3]
        #Y := btns[4]
        #LT = axes[5]
        #axes = data.axes
        #if data.axes[5] < 1-2*self.__DEADZONE:
        #    fact = -1.0
        #else:
        #    fact = 1.0

        buttons = data.buttons

        n = 0
        n += buttons[0]
        n += buttons[1]
        n += buttons[3]
        n += buttons[4]
        if n > 1:
            return

        keys = {"A":0,"B":1,"X":3,"Y":4}
        for k,v in keys.items():
            if self.state == 0 and k != "A":
                continue
            if self.state == 1 and k not in ["A","B"]:
                continue
            if self.state == 2 and k not in ["B","X"]:
                continue

            if buttons[v] and self.__btns[k]:
                #print(f"Test: {k} {v}")
                self.__btns[k] = False
                
                if k == "A":
                    if self.state == 0:
                        self.state = 1
                        trans_0_1(self.grab_behaviour)
                    else:
                        trans_1_0(self.grab_behaviour)
                        self.state = 0

                if k == "B":
                    if self.state == 1:
                        trans_1_2(self.grab_behaviour)
                        self.state = 2
                    else:
                        trans_2_1(self.grab_behaviour)
                        self.state = 1

                if k == "X":
                    if self.gripper_state == 0:
                        self.gripper_state = 1
                        self.grab_behaviour.open_gripper()
                        self.grab_behaviour.publish()
                    else:
                        self.gripper_state = 0
                        self.grab_behaviour.close_gripper()
                        self.grab_behaviour.publish()

            if not buttons[v]:
                self.__btns[k] = True

s0 = (0,15,60,-45)
s1 = (90,15,60,-45)
s2 = (90,0,0,-60)

s = 0
def trans_0_1(gripper:GrabBehaviour):
    gripper.setState(s1)
    gripper.publish()
    rospy.loginfo("Wait for movement to stop...")
    sleep(s)

def trans_1_0(gripper:GrabBehaviour):
    gripper.setState(s0)
    gripper.publish()
    rospy.loginfo("Wait for movement to stop...")
    sleep(s)

def trans_1_2(gripper:GrabBehaviour):
    gripper.setState(s2)
    gripper.publish()
    rospy.loginfo("Wait for movement to stop...")
    sleep(s)

def trans_2_1(gripper:GrabBehaviour):
    gripper.setState(s1)
    gripper.publish()
    rospy.loginfo("Wait for movement to stop...")
    sleep(s)

if __name__ == "__main__":
    rospy.init_node("joypad_manipulator")
    #rospy.init_node("joypad_manipulator_node")
    joypadgripper = JoypadGripper()
    rospy.spin()