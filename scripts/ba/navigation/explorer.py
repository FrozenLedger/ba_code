import rospy

from std_srvs.srv import Trigger

EXPLORERNS = "explorer"
class NodeExplorer:
    def __init__(self):
        service_name = f"{EXPLORERNS}/explore"
        rospy.wait_for_service(service_name)
        self._trigger = rospy.ServiceProxy(service_name,Trigger)
        rospy.loginfo("NodeExplorer initialized")

    def explore(self):
        self._trigger()