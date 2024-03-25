from collections.abc import Sequence
from typing import Any

class LidarScan(Sequence):
    """Class that takes lidar data."""
    def __init__(self,*data):
        self._data = list(data)
        print(data)

    def __len__(self):
        return len(self._data)
    
    def __getitem__(self,item):
        return self._data.__getitem__(item)