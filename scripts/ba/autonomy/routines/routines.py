import rospy
import ba_code.srv as basrv
from ba_code.srv import GetObjectList

from geometry_msgs.msg import PointStamped, Point

from ba.autonomy.object_collector import ObjectCollector
from ba.navigation.explorer import NodeExplorer
from ba.autonomy.object_collector import STATIONNAMESPACE
from ba.tracking.object_tracker import TRACKERNAMESPACE
from ba.autonomy.routines.iroutine import IRoutine

#from ba.utilities.singletons.robot_mover_singleton import get_robot_mover