import ba.autonomy.routines as routines

import rospy

from std_srvs.srv import Trigger,TriggerResponse, TriggerRequest

from ba.utilities.data import Data
from ba.autonomy.routines import IRoutine

State = IRoutine
Service = rospy.Service
Rate = rospy.Rate
class StateMachine:
    """A first conecpt of a state machine, that controls the autonomous behaviour of the robot."""
    def __init__(self):
        self.__active_state: State = routines.ChargeBatteryRoutine(self)
        self.__shutdown_signal_server: Service = rospy.Service("/state_machine/shutdown",Trigger,self.__shutdown_signal_received)
        self.__battery_low_signal_server: Service = rospy.Service("/state_machine/battery_low",Trigger,self.__battery_low_signal_received)
        print("State Machine running.")

    def __set_active_state(self,new_state: State):
        try:
            self.__active_state = new_state
            return TriggerResponse(success=True,message="SUCCESS")
        except Exception as e:
            print(e)
            return TriggerResponse(success=False,message="FAILED")

    def __battery_low_signal_received(self,request: TriggerRequest):
        return self.__set_active_state( routines.ChargeBatteryRoutine(self) )
    
    def __shutdown_signal_received(self,request: TriggerRequest):
        return self.__set_active_state( routines.ShutdownRoutine(self) )
    
    def update(self):
        """Executes the behaviour of the active state. The execute methode of the state is expected to return itself or a different state if a state transition is performed.
        Further development:
            * implementation of a 'on_state_enter' method, that will be triggered when a state is entered during a state transition.
            * implementation of a 'on_state_exit' method, that will be triggered when a state is exited during a state transition."""
        self.__active_state: State = self.__active_state.execute()

def main():
    """Creates an object of the state machine and runs the update method in a loop to process the states and enable state transitions.
The loop runs at 10hz, which is !not! in real-time. -> The loop might run slower than 10hz, if a process in the loop is delaying."""
    
    rospy.init_node("finite_state_machine")
    
    FSM: StateMachine = StateMachine()
    rate: Rate = rospy.Rate(10)
    
    running: bool = True
    while running and not rospy.is_shutdown():
        try:
            FSM.update()
            rate.sleep()
        except Exception as e:
            print(e)
            running = False

def test():
    rospy.init_node("test_routine")
    
    FSM: StateMachine = StateMachine()
    routine: State = routines.ShutdownRoutine(FSM)

    rate: Rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            routine.execute()
            rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            print(e)
            break

if __name__ == "__main__":
    main()
    #test()