import time

from ba.navigation.robot import RobotMover
from ba.utilities.datatypes import Position, Orientation, Pose

class PosePath:
    def __init__(self,poses):
        self._poses = poses
        self._index = 0

    def __iter__(self):
        while True:
            yield self.__next__()

    def __next__(self):
        pose: Pose = self._poses[self._index]
        self._index += 1
        self._index %= len(self._poses)
        return pose
    
    def set_poses(self, poses):
        self._poses = poses
        self._index = 0

class PathExplorer:
    def __init__(self, robot: RobotMover, path: PosePath):
        self._robot = robot
        self._path = path

    def explore(self):
        self._robot.move_to_pose(next(self._path))
        self._robot.wait_for_result()

    @property
    def path(self):
        return self._path

def main():
    points = [(0,0),(1,0),(0,1),(1,1)]
    path = PosePath([Pose(position=Position(x=x,y=y,z=0),
                          orientation=Orientation()) for x,y in points])

    for p in path:
        print(p)
        time.sleep(1)

if __name__ == "__main__":
    main()