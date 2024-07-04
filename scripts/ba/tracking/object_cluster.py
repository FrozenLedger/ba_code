import math

from ba_code.msg import Object
from geometry_msgs.msg import Point

from ba.tracking.object_tracker import ObjectTracker

class ObjectCluster:
    def __init__(self, obj: Object):
        self.__data: list[object] = []
        self.__avg: Object = obj
        self.add(obj)

    def add(self, obj: Object) -> None:
        pnt: Point = obj.point.point
        avg: Object = self.__avg

        N: int = len(self.__data)
        NN: int = N+1
        x_avg: float = (avg.point.point.x*N + pnt.x)/NN
        y_avg: float = (avg.point.point.y*N + pnt.y)/NN
        z_avg: float = (avg.point.point.z*N + pnt.z)/NN
        conf_avg: float = (avg.confidence*N + obj.confidence)/NN

        self.__avg.point.point.x = x_avg
        self.__avg.point.point.y = y_avg
        self.__avg.point.point.z = z_avg
        self.__avg.confidence = conf_avg

        self.__data.append(obj)
        print(f"N: {NN}")

    def distance(self,obj: Object):
        avg = self.avg.point.point
        ax,ay,az = avg.x, avg.y, avg.z
        bx,by,bz = obj.point.point.x, obj.point.point.y, obj.point.point.z

        x = bx-ax
        y = by-ay
        z = bz-az

        return math.sqrt(x**2+y**2+z**2)

    @property
    def point(self):
        return self.__point
    
    @property
    def avg(self) -> Object:
        return self.__avg

class ObjectClusterTracker(ObjectTracker):
    @property
    def clusters(self):
        return self.__clusters
    
    def __init__(self, epsilon=0.05):
        self.__clusters = []
        self.__epsilon = epsilon # min. distance to an object cluster that a object needs to be in 

    def add(self, cluster: ObjectCluster) -> None:
        self.__clusters.append(cluster)

    def eval(self, obj: Object) -> None:
        if len(self.__clusters) == 0:
            print("(1)Create new cluster.")
            cluster: ObjectCluster = ObjectCluster(obj)
            self.add(cluster)
            return
        
        min_dist: float = math.inf
        index = -1
        for idx,cluster in enumerate(self.__clusters):
            if obj.clsID != cluster.avg.clsID:
                continue

            dist: float = cluster.distance(obj)
            if dist < min_dist:
                min_dist = dist
                index = idx

        if min_dist < self.__epsilon and index >= 0:
            print("(2)Add to cluster.")
            self.__clusters[index].add(obj)
        else:
            print(min_dist, index, self.__epsilon)
            print("(3)Create new cluster.")
            cluster: ObjectCluster = ObjectCluster(obj)
            self.add(cluster)

    def pop(self, index: int = 0) -> ObjectCluster:
        return self.__clusters.pop(index)