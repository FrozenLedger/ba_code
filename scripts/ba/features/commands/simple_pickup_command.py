from ba_code.scripts.ba.manipulator.robotarm_adapter import RobotarmAdapter

class SimplePickupCommand:
    def __init__(self, robotarm) -> None:
        self.__robotarm: RobotarmAdapter = robotarm

    def execute(self):
        self.__robotarm.run_pickup_instructions()