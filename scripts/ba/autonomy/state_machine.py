import ba.autonomy.routines as routines

import rospy

class StateMachine:
    def __init__(self):
        self.__active_state = routines.ChargeBatteryRoutine()

    def __check_battery(self):
        return True
    
    def __check_shutdown(self):
        return False
    
    def update(self):
        if self.__check_shutdown():
            self.__active_state = routines.ShutdownRoutine(self)
        elif self.__check_battery():
            self.__active_state = routines.ChargeBatteryRoutine(self)
        
        self.__active_state = self.__active_state.execute()

def main():
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