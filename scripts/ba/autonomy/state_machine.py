import ba.autonomy.routines as routines

import rospy

class StateMachine:
    """A first conecpt of a state machine, that controls the autonomous behaviour of the robot."""
    def __init__(self):
        self.__active_state = routines.ChargeBatteryRoutine()

    def __check_battery(self):
        return True
    
    def __check_shutdown(self):
        return False
    
    def update(self):
        """To check for interrupts, the update methode will call the necessary funcionalities to check battery-voltage and shutdown signals.
If no interrupt has been received, then the active state will be executed, wich will set the new active state of the robot.
The state is expected to set itself as the next active state, if there is no state transition."""
        if self.__check_shutdown():
            self.__active_state = routines.ShutdownRoutine(self)
        elif self.__check_battery():
            self.__active_state = routines.ChargeBatteryRoutine(self)
        
        self.__active_state = self.__active_state.execute()

def main():
    """Creates an object of the state machine and runs the update method in a loop to process the states and enable state transitions.
The loop runs at 100hz, which is !not! in real-time. -> The loop might run slower than 100hz, if a process in the loop is delaying."""
    FSM = StateMachine()
    rate = rospy.Rate(100)

    rospy.init_node("finite_state_machine")
    
    running = True
    while running and not rospy.is_shutdown():
        try:
            FSM.update()
            rate.sleep()
        except Exception as e:
            print(e)
            running = False

if __name__ == "__main__":
    main()