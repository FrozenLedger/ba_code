import math

def pose_distance(A,B):
    dx = A.point.x - B.point.x
    dy = A.point.y - B.point.y

    return math.sqrt(dx**2 + dy**2)