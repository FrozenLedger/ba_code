import rospy

from PyQt5 import QtCore, QtGui, QtWidgets
from solarswarm_gui import Ui_MainWindow
from status_display import StatusDisplay
from map_display import MapDisplay
from controller.sm_controller import StateMachineController

class SolarSwarm_MainWindow(Ui_MainWindow):
    def setupUi(self, MainWindow):
        result = super().setupUi(MainWindow)
        MainWindow.setWindowTitle("SolarSwarm GUI")

        self.velocity_display = StatusDisplay(self)
        self.map_display = MapDisplay(self)
        self.map_display.display()

        self._sm_controller = StateMachineController(self)

        return result

if __name__ == "__main__":
    import sys

    rospy.init_node("solarswarm_gui_node")

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()

    ui = SolarSwarm_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    sys.exit(app.exec_())