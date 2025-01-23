import rospy

from enum import Enum
from sensor_msgs.msg import Joy

from ba.features.teleoperation.teleoperation_logger import TELEOPLOGGER as LOGGER

from ba.manipulator.commands.commands import *

class XBoxButtons(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    Menu = 7
    LA = 9
    RA = 10

DEFAULT_COMMANDS = {
    XBoxButtons.A.value: PrintCommand(msg="A pressed."),
    XBoxButtons.B.value: PrintCommand(msg="B pressed."),
    XBoxButtons.X.value: PrintCommand(msg="X pressed."),
    XBoxButtons.Y.value: PrintCommand(msg="Y pressed."),
    XBoxButtons.LB.value: PrintCommand(msg="LB pressed."),
    XBoxButtons.RB.value: PrintCommand(msg="RB pressed."),
    XBoxButtons.Menu.value: PrintCommand(msg="Menu pressed."),
    XBoxButtons.LA.value: PrintCommand(msg="LA pressed."),
    XBoxButtons.RA.value: PrintCommand(msg="RA pressed.")
}

class JoypadGripperNode:
    #commands = {
    #    XBoxButtons.A.value: LogJointsPositionsCommand(),
    #    XBoxButtons.B.value: StateTransitionCommand(),
    #    XBoxButtons.X.value: PrintCommand(msg="X pressed."),
    #    XBoxButtons.Y.value: ToggleEnableTorqueCommand(),
    #    XBoxButtons.LB.value: PrintCommand(msg="LB pressed."),
    #    XBoxButtons.RB.value: PrintCommand(msg="RB pressed."),
    #    XBoxButtons.Menu.value: PrintCommand(msg="Menu pressed."),
    #    XBoxButtons.LA.value: PrintCommand(msg="LA pressed."),
    #    XBoxButtons.RA.value: PrintCommand(msg="RA pressed.")
    #}

    @property
    def commands(self):
        return self._commands

    """A node that takes inputs from the /joy topic and sends further control signals to the /cmd_vel topic in order to control its move behaviour."""
    def __init__(self,commands=DEFAULT_COMMANDS) -> None:
        self._commands = commands

        self.subscriber: rospy.Subscriber = rospy.Subscriber("/joy",Joy,self._publisher_cb)
        LOGGER.info(f"<{type(self).__name__}> Subscriber initialized for topic: /joy.")
        LOGGER.info(f"{type(self).__name__} was created.")
    
    def _publisher_cb(self,data: Joy) -> None:
        buttons = data.buttons

        for idx, btn in enumerate(buttons):
            if btn == 1:
                self.commands[idx].execute()

    def __del__(self) -> None:
        pass
        #LOGGER.info(f"{type(self).__name__} was destroyed.")

if __name__ == "__main__":
    rospy.init_node("joypad_manipulator")

    progStateTransCMD = ProgressiveStateTransitionCommand() # LoopCommand(ProgressiveStateTransitionCommand())

    commands = {
        XBoxButtons.A.value: UndoCommand(progStateTransCMD), #LogJointsPositionsCommand(),
        XBoxButtons.B.value: StateTransitionCommand(),
        XBoxButtons.X.value: progStateTransCMD, #PrintCommand(msg="X pressed."),
        XBoxButtons.Y.value: ToggleEnableTorqueCommand(),
        XBoxButtons.LB.value: PrintCommand(msg="LB pressed."),
        XBoxButtons.RB.value: PrintCommand(msg="RB pressed."),
        XBoxButtons.Menu.value: PrintCommand(msg="Menu pressed."),
        XBoxButtons.LA.value: PrintCommand(msg="LA pressed."),
        XBoxButtons.RA.value: PrintCommand(msg="RA pressed.")
    }

    joypadgripper = JoypadGripperNode(commands=commands)
    rospy.spin()

"""
class JoypadGripper:
    "A node that takes inputs from the /joy topic and sends further control signals to the robotarm to control its grab behaviour."
    def __init__(self):
        self.grab_behaviour = RobotarmController() #GrabBehaviour(start_pose=[0,15,60,-45])
        self.subscriber = rospy.Subscriber("/joy",Joy,self.publisher_cb)        
        self.__btns = {"A":True,
                       "B":True,
                       "X":True,
                       "Y":True}

        self.state = 0
        self.gripper_state = 0
        
        sleep(1)
        rospy.loginfo("Start delay...")
        #self.grab_behaviour.set_positions_deg(s0) #self.grab_behaviour.setState(s0)
        #self.grab_behaviour.open_gripper()
        #self.grab_behaviour.publish()
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
"""
"""
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
"""