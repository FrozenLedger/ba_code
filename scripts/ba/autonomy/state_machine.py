import ba.autonomy.routines as routines

import rospy, threading
import argparse

from std_srvs.srv import Trigger,TriggerResponse, TriggerRequest

from ba.utilities.data import Data
from ba.tracking.object_tracker import TRACKERNAMESPACE
from ba.autonomy.object_collector import STATIONNAMESPACE
from ba.autonomy.autonomy_logger import AUTONOMYLOGGER as LOGGER
from ba.autonomy.routines.iroutine import IRoutine
from ba.autonomy.routines.charge_battery_routine import ChargeBatteryRoutine
from ba.autonomy.routines.shutdown_routine import ShutdownRoutine

State = IRoutine
Service = rospy.Service
Rate = rospy.Rate

class StateMachine:
    """A first conecpt of a state machine, that controls the autonomous behaviour of the robot."""
    def __init__(self):
        LOGGER.info("Initialize State Machine...")
        self._active_state: State = ChargeBatteryRoutine(self)
        self._shutdown_signal_server: Service = rospy.Service("/state_machine/shutdown",Trigger,self._shutdown_signal_received)
        self._battery_low_signal_server: Service = rospy.Service("/state_machine/battery_low",Trigger,self._battery_low_signal_received)
        self._lock = threading.Lock()
        LOGGER.info("State Machine running.")

    def _set_active_state(self,new_state: State):
        try:
            self._lock.acquire()
            print("Lock acquired.")
            self._active_state = new_state
            return TriggerResponse(success=True,message="SUCCESS")
        except Exception as e:
            LOGGER.error(str(e))
            return TriggerResponse(success=False,message="FAILED")
        finally:
            print("Lock released.")
            self._lock.release()

    def _battery_low_signal_received(self,request: TriggerRequest):
        return self._set_active_state( ChargeBatteryRoutine(self) )
    
    def _shutdown_signal_received(self,request: TriggerRequest):
        return self._set_active_state( ShutdownRoutine(self) )
    
    def update(self):
        """Executes the behaviour of the active state. The execute methode of the state is expected to return itself or a different state if a state transition is performed.
        Further development:
            * implementation of a 'on_state_enter' method, that will be triggered when a state is entered during a state transition.
            * implementation of a 'on_state_exit' method, that will be triggered when a state is exited during a state transition."""
        self._lock.acquire()
        self._active_state: State = self._active_state.execute()
        self._lock.release()

def main():
    """Creates an object of the state machine and runs the update method in a loop to process the states and enable state transitions.
The loop runs at 10hz, which is !not! in real-time. -> The loop might run slower than 10hz, if a process in the loop is delaying."""
    
    rospy.init_node("finite_state_machine")

    parser = argparse.ArgumentParser()
    parser.add_argument("-o","--origin",metavar="FLOAT",help="Sets the origin of the station. Format: x y",default=[0,0],type=float,nargs="+")
    parser.add_argument("-d","--max-distance",metavar="FLOAT",help="Sets the max distance of a detected object that will be collected.",default=1,type=float)
    #parser.add_argument("-g","--min-gap",metavar="FLOAT",help="Sets the minimum gap [cm] of tracked objects. Objects that are within this range of another tracked object will be ignored.",default=1,type=float)
    args = parser.parse_args()

    rospy.set_param(f"/{TRACKERNAMESPACE}/max_distance",args.max_distance)
    #rospy.set_param(f"/{TRACKERNAMESPACE}/min_gap", args.min_gap)

    # Setting ROS params
    try:
        x = args.origin[0]
        y = args.origin[1]
    except KeyError as e:
        LOGGER.error(str(e))
        x = y = 0
    rospy.set_param(f"/{STATIONNAMESPACE}/origin",{"x":x,"y":y})

    FSM: StateMachine = StateMachine()
    rate: Rate = rospy.Rate(10)
    
    running: bool = True
    while running and not rospy.is_shutdown():
        try:
            FSM.update()
            rate.sleep()
        except Exception as e:
            LOGGER.error(str(e))
            running = False
    
def test():
    LOGGER.info("Running state_machine test.")
    rospy.init_node("test_routine")
    
    FSM: StateMachine = StateMachine()
    routine: State = ShutdownRoutine(FSM)

    rate: Rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            routine.execute()
            rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            LOGGER.error(str(e))
            break
    LOGGER.info("Finished state_machine test.")

if __name__ == "__main__":
    #main()
    test()