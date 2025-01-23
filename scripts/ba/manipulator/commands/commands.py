import rospy
import time
import threading
import numpy as np

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

class LoopCommand(ACommand):
    def __init__(self, target: ACommand):
        self._target: ACommand = target
        self._delay: float = 0.02 # s

        self._enabled = False
        self._thread = threading.Thread(target=self._loop)
        self._thread._stop_event = threading.Event()
        self._thread.start()

    def execute(self):
        self._enabled = not self._enabled

    def undo(self):
        self._enabled = not self._enabled

    def _loop(self):
        while True:
            if self._enabled:
                self._target.execute()
                time.sleep(self._delay)

    def __del__(self):
        self._thread._stop_event.set()
        self._thread.join()

STATE = 0
LOCK = threading.Lock()
DELAY = 3
class StateTransitionCommand(ACommand):
    def __init__(self):
        self._robotarm = RobotarmController()
        self._positions =   [[178, 191, 240, 101, 230], # open := 230 # old value was 306
                            [266, 178, 238, 92, 230],
                            [267, 175, 180, 89, 230],
                            [267, 175, 180, 89, 100],
                            [266, 184, 100, 93, 100], # before: [266, 179, 89, 86, 100]
                            ### reverse ###
                            [266, 184, 100, 93, 230], # before: [266, 179, 89, 86, 230]
                            [267, 175, 180, 89, 230],
                            [267, 175, 180, 89, 230],
                            [266, 178, 238, 92, 230],
                            [178, 191, 240, 101, 230],
                            ] # close := 230 # old value was 439
            
    def execute(self):
        global STATE
        global LOCK
        global INPUT
        lock = LOCK.acquire(blocking=False)
        print("Acquired")
        if lock:
            if STATE == 0:
                for position in self._positions[0:5]:
                    self._robotarm.set_positions_deg(position)
                    time.sleep(DELAY)
                STATE = 1
            elif STATE == 1:
                for position in self._positions[5::]: #reversed(self._positions):
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

class ProgressiveStateTransitionCommand(ACommand):
    def __init__(self):
        self._robotarm = RobotarmController()
        #self._positions =   [[178, 191, 240, 101, 230], # open := 230 # old value was 306
        #                    [266, 178, 238, 92, 230],
        #                    [267, 175, 180, 89, 230],
        #                    [267, 175, 180, 89, 100],
        #                    [266, 184, 100, 93, 100], # before: [266, 179, 89, 86, 100]
                            ### reverse ###
        #                    [266, 184, 100, 93, 230], # before: [266, 179, 89, 86, 230]
        #                    [267, 175, 180, 89, 230],
        #                    [267, 175, 180, 89, 230],
        #                    [266, 178, 238, 92, 230],
        #                    [178, 191, 240, 101, 230],
        #                    ] # close := 230 # old value was 439
        self._index = 0

        self._enabled = False

        def __interpolate(start,end,N=7) -> list:
            return [int(np.interp(i/N, [0,1],[start,end])) for i in range(0,N+1)]
        
        vi = __interpolate(start=178,end=266) + __interpolate(266,267) + __interpolate(267,267) + __interpolate(267,266) \
            + __interpolate(266,266) + __interpolate(266,267) + __interpolate(267,267) + __interpolate(267,266) + __interpolate(266,178)
        wi = __interpolate(start=191,end=178) + __interpolate(178,175) + __interpolate(175,175) + __interpolate(175,184) \
            + __interpolate(184,184) + __interpolate(184,175) + __interpolate(175,175) + __interpolate(175,178) + __interpolate(178,191)
        xi = __interpolate(start=240,end=238) + __interpolate(238,180) + __interpolate(180,180) + __interpolate(180,100) \
            + __interpolate(100,100) + __interpolate(100,180) + __interpolate(180,180) + __interpolate(180,238) + __interpolate(238,240)
        yi = __interpolate(start=101,end=92) + __interpolate(92,89) + __interpolate(89,89) + __interpolate(89,93) \
            + __interpolate(93,93) + __interpolate(93,89) + __interpolate(89,89) + __interpolate(89,92) + __interpolate(92,101)
        zi = __interpolate(start=230,end=230) + __interpolate(230,230) + __interpolate(230,100) + __interpolate(100,100) \
            + __interpolate(100,230) + __interpolate(230,230) + __interpolate(230,230) + __interpolate(230,230) + __interpolate(230,230)
        
        self._positions = np.array([[vi[i],wi[i],xi[i],yi[i],zi[i]] for i in range(len(vi))])
        print(self._positions)

    def execute(self):
        print(f"Set positions: {self._positions[self._index]}")
        self._robotarm.set_positions_deg(self._positions[self._index])
        self._index = min(len(self._positions)-1, self._index+1)
        #time.sleep(0.01)

    def undo(self):
        self._index = max(0,self._index-1)
        print(f"Set positions: {self._positions[self._index]}")
        self._robotarm.set_positions_deg(self._positions[self._index])
        #time.sleep(0.01)

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