from ba.autonomy.state_machine import StateMachine

class IRoutine:
    def __init__(self,FSM:StateMachine):
        self.__FSM = FSM

    def execute(self):
        return self

class CollectGarbageRoutine(IRoutine):
    def __init__(self, FSM: StateMachine):
        super().__init__(FSM)

    def execute(self):
        return super().execute()

class EmptyStorageRoutine(IRoutine):
    def __init__(self, FSM: StateMachine):
        super().__init__(FSM)

    def execute(self):
        return super().execute()

class ExploreRegionRoutine(IRoutine):
    def __init__(self, FSM: StateMachine):
        super().__init__(FSM)

    def execute(self):
        return super().execute()

class ShutdownRoutine(IRoutine):
    def __init__(self, FSM: StateMachine):
        super().__init__(FSM)

    def execute(self):
        return super().execute()

class ChargeBatteryRoutine(IRoutine):
    def __init__(self, FSM: StateMachine):
        super().__init__(FSM)

    def execute(self):
        return super().execute()