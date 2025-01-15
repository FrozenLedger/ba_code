import rospy
import threading

from abc import ABC, abstractmethod

from magni_dxl_module.robotarm_controller import RobotarmController

class ACommand(ABC):
    @abstractmethod
    def execute(self) -> None: pass
    @abstractmethod
    def undo(self) -> None: pass

class NullCommand(ACommand):
    pass

class UndoCommand(ACommand):
    def __init__(self, target):
        self._target = target
    def execute(self):
        self._target.undo()
    def undo(self):
        self._target.execute()

STATE = 0
LOCK = threading.Lock()
DELAY = 3
class StateTransitionCommand(ACommand):
    def __init__(self):
        self._robotarm = RobotarmController()
        self._positions =   [[178, 191, 240, 101, 306],
                            [266, 178, 238, 92, 306],
                            [267, 175, 180, 89, 306],
                            [266, 179, 89, 86, 439]]
            
    def execute(self):
        global STATE
        global LOCK
        global INPUT
        lock = LOCK.acquire(blocking=False)
        print("Acquired")
        if lock:
            if STATE == 0:
                for position in self._positions:
                    self._robotarm.set_positions_deg(position)
                    time.sleep(DELAY)
                STATE = 1
            elif STATE == 1:
                for position in reversed(self._positions):
                    self._robotarm.set_positions_deg(position)
                    time.sleep(DELAY)
                STATE = 0
            else:
                print("")
            INPUT = True
            LOCK.release()
        else:
            print("Lock couldn't be acquired.")

    def undo(sefl):
        pass

class ToggleEnableTorqueCommand(ACommand):
    def __init__(self):
        self._enabled: bool = False
        self._robotarm = RobotarmController()

    def execute(self):
        self._enabled = not self._enabled
        self._robotarm.enable_torque(self._enabled)

    def undo(self):
        pass

class OpenGripperCommand(ACommand):
    def __init__(self):
        self._robotarm = RobotarmController()

    def execute(self):
        self._robotarm.open_gripper()

    def undo(self):
        self._robotarm.close_gripper()

class EnableTorqueCommand(ACommand):
    def __init__(self):
        self._robotarm = RobotarmController()

    def execute(self):
        self._robotarm.enable_torque(True)

    def undo(self):
        self._robotarm.enable_torque(False)

class PrintCommand(ACommand):
    def __init__(self,msg:str) -> None:
        self._msg:str = msg

    def execute(self):
        print(self._msg)

    def undo(self):
        pass

class LogJointsPositionsCommand(ACommand):
    def __init__(self) -> None:
        self._robotarm = RobotarmController()

    def execute(self):
        positions = self._robotarm.get_positions_deg()
        with open("/tmp/joint_positions.txt","a") as f:
            f.write(str(positions) + "\n")

    def undo(self):
        pass