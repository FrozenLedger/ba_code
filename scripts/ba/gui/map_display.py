import rospy

from solarswarm_gui import Ui_MainWindow
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtGui import QPixmap

class MapDisplay:
    def __init__(self, parent: Ui_MainWindow):
        self._parent: Ui_MainWindow = parent

        #self._map = OccupancyGrid()
        #self._map_listener = rospy.Subscriber("")

    def display(self):
        self._parent.map_image.setPixmap(QPixmap("map.pgm"))