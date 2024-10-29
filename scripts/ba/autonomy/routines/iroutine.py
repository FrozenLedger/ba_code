class IRoutine:
    def __init__(self,FSM):
        self.__FSM = FSM

    @property
    def FSM(self):
        return self.__FSM

    def execute(self):
        return self
    
    #def __del__(self):
    #    try:
    #        get_robot_mover().cancel()
    #    except Exception as e:
    #        print(e)

from ba.autonomy.routines.charge_battery_routine import ChargeBatteryRoutine
from ba.autonomy.routines.collect_garbage_routine import CollectGarbageRoutine
from ba.autonomy.routines.empty_storage_routine import EmptyStorageRoutine
from ba.autonomy.routines.explore_region_routine import ExploreRegionRoutine
from ba.autonomy.routines.shutdown_routine import ShutdownRoutine