from dataclasses import dataclass

@dataclass
class Position:
    x: float = 0
    y: float = 0
    z: float = 0

@dataclass
class Orientation:
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 1

@dataclass
class Pose:
    position: Position
    orientation: Orientation