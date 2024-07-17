import rospy

from threading import Thread
from solarswarm_gui import Ui_MainWindow
from ba.autonomy.state_machine import StateMachine

class ObjectDetectionController:
    def __init__(self, parent: Ui_MainWindow):
        self._parent = parent
        self._enabled = False

        self._parent.btn_obj_detection.clicked.connect(lambda: self.toggle())

    def exec(self):
        rate: rospy.Rate = rospy.Rate(10)
        self._enabled: bool = True

        print("State machine started.")
        while self._enabled and not rospy.is_shutdown():
            try:
                self._FSM.update()
                rate.sleep()
            except Exception as e:
                print(e)
                self._enabled = False
        print("State machine stopped.")

    def toggle(self):
        if self._enabled:
            self._disable()
        else:
            self._enable()

    def _disable(self):
        self._parent.btn_sm.setText("Start")
        self._enabled = False

        self._thread.join()

    def _enable(self):
        self._thread = Thread(target=self.exec)
        self._parent.btn_sm.setText("Stop")
        self._enabled = True

        self._thread.start()