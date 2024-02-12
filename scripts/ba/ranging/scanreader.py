import numpy as np
import math
from dataclasses import dataclass,field

from sensor_msgs.msg import LaserScan

from ba.configs import min_gap

@np.vectorize
def calcSinCos(angle):
    return (math.sin(angle),math.cos(angle))

@dataclass
class VectorSumData:
    y:float
    x:float
    alpha:float
    min_dist:float
    min_avg:float
    min_median:float

def _calcVectorSum(dist_arr,angles,range_max,deviation=0.05):
    try:
        min_dist = max(min_gap,np.min(dist_arr)) # distance to the obstacle with shortest distance or min_gap if within min_gap distance
        if min_dist == math.inf:
            min_dist = range_max # sets the min_dist to the max allowed value
        #dist_arr[dist_arr == math.inf] = range_max
        indizes = dist_arr < min_dist+deviation #np.logical_and(dist_arr >= min_dist,dist_arr < min_dist + deviation) # considers all values within min_dist + added radius
        dist_arr = dist_arr[indizes] # filters values
        angles = angles[indizes] # filters values

        if len(dist_arr) == 0:
            return VectorSumData(0,0,0,0,0,0)

        (S,C) = calcSinCos(angles)*dist_arr # calculates the sum of all laser scans with consideration to the direction
        y = np.average(S) #np.sum(S)/len(S) # normalizes x value of the vector#
        x = np.average(C) #np.sum(C)/len(C) # normalizes y value of the vector#
        alpha = math.atan2(y,x) # calculates the angle of the vector

        return VectorSumData(y=y, x=x, alpha=alpha, min_dist=min_dist,min_avg=np.average(dist_arr), min_median=np.median(dist_arr))
    except Exception as e:
        print(e)
        return VectorSumData(0,0,0,0,0,0)
    # returns the vector and the average of considered values

class ScanReader():
    def __init__(self,scan: LaserScan):
        self.__scan = scan
        self.__readScan()

    def __readScan(self):
        scan = self.__scan
        self.__distances = np.array(scan.ranges)

        start_angle = scan.angle_min
        incr = scan.angle_increment
        N = len(self.distances)

        self.__angles = np.array([start_angle + i*incr for i in range(N)])
        
    @property
    def scan(self):
        return self.__scan

    @property
    def distances(self):
        return self.__distances
    
    @property
    def angles(self):
        return self.__angles

    def calcVectorSumNormalized(self, min_angle=-math.pi, max_angle=math.pi, deviation=0.05):
        """
Return the closest obstacle in the ROI defined by the min_angle and max_angle.
min_angle(default):= -pi
max_angle(default):= +pi
add_radius(default):=0.05
"""
        indizes = np.logical_and(self.angles >= min_angle,self.angles<=max_angle,~np.isnan(self.distances))

        dist_arr = self.distances[indizes]
        angles = self.angles[indizes]
        range_max = self.scan.range_max
        
        return _calcVectorSum(dist_arr,angles,range_max,deviation)