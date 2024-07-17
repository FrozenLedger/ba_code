import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from PyQt5.QtCore import QTimer
from solarswarm_gui import Ui_MainWindow

class StatusDisplay:
    def __init__(self,parent: Ui_MainWindow,vel_topic: str = "/cmd_vel", pose_topic: str = "/robot/pose"):
        self._parent: Ui_MainWindow = parent

        self._velocity: Twist = Twist()
        self._pose: PoseStamped = PoseStamped()
        
        self._vel_subscriber: rospy.Subscriber = rospy.Subscriber(vel_topic,Twist, self.set_velocity)
        self._pose_subscriber: rospy.Subscriber = rospy.Subscriber(pose_topic, PoseStamped, self.set_pose)
        self._qTimer: QTimer = QTimer()

        self._qTimer.setInterval(100) # 100 ms
        self._qTimer.timeout.connect(self.display)
        self._qTimer.start()

    def display(self):
        linear = self._velocity.linear.x
        angular = self._velocity.angular.z

        # display the robot target velocity (not actual velocity)
        self._parent.lcd_linear_spd.display(linear)
        self._parent.lcd_angular_spd.display(angular)

        # display the robot position on the map frame
        pos = self._pose.pose.position
        self._parent.lcd_position_x.display(pos.x)
        self._parent.lcd_position_y.display(pos.y)
        self._parent.lcd_position_z.display(pos.z)
    
    def set_velocity(self,msg):
        self._velocity = msg

    def set_pose(self,msg):
        self._pose = msg