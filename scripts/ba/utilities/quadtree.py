from dataclasses import dataclass

@dataclass
class Boundaries:
    width: int
    height: int

    def __contains__(self, point2D: tuple[float,float]) -> float:
        x, y = point2D
        return 0 <= x < self.width and 0 <= y < self.height
    
class Grid:
    def __init__(self,width: float, heigth: float):
        self._boundaries = Boundaries(width=width,height=heigth)

    @property
    def width(self) -> float:
        return self._boundaries.width

    @property
    def height(self) -> float:
        return self._boundaries.height

    def __contains__(self, point2D: tuple[float,float]) -> bool:
        return point2D in self._boundaries
    
class Quadtree:
    pass

if __name__ == "__main__":
    bounds = Boundaries(10,10)

    print(f"(5,5) in bounds: {(5,5) in bounds}")
    print(f"(10,10) in bounds: {(10,10) in bounds}")