from dataclasses import dataclass

class Data:
    """A class to simplify the process of storing changes of a published topic."""
    def __init__(self,data):
        self.__data = data

    @property
    def data(self):
        return self.__data
    
    def setData(self,data):
        self.__data = data

@dataclass
class ScanData:
    distance: float
    angle: float