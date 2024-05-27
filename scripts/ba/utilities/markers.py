import rospy
from visualization_msgs.msg import Marker

class VectorMarker:
    def __init__(self, pose, mid=0, duration=0, rgb=(0,255,0), alpha=0.5, scale=(1.41,0.05,0.05)):
        marker = Marker()

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = alpha
        marker.header = pose.header
        marker.type = 0
        marker.id = mid
        marker.pose = pose.pose
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.lifetime = rospy.Duration(duration)

        self._data = marker
        
    @property
    def data(self):
        return self._data

class ConeMarker:
    def __init__(self, pose, mid=0):
        marker = Marker()

        marker.color.r = 200
        marker.color.b = 200
        marker.color.a = 0.5
        marker.header = pose.header
        marker.type =  2
        marker.id = mid
        marker.pose = pose.pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 1.50

        marker.lifetime = rospy.Duration(0)

        self._data = marker
        
    @property
    def data(self):
        return self._data