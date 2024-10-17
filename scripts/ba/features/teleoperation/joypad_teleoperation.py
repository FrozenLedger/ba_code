import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from .teleoperation_logger import TELEOPLOGGER as LOGGER

class JoypadTeleopNode:
    """A node that takes inputs from the /joy topic and sends further control signals to the /cmd_vel topic in order to control its move behaviour."""
    def __init__(self,freq=10) -> None:
        self.vel_msg = Twist()

        self.DEADZONE = 0.05
        LOGGER.info(f"Joypad deadzone/sensibility set to {self.DEADZONE}.")

        self.enabled = False
        LOGGER.info("Movement disabled by default. Press [A4] to enable movement.")

        self.rate = rospy.Rate(freq)
        LOGGER.info(f"Publishing rate set to {freq}/s.")
        
        self.publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        LOGGER.info("Publisher initialized for topic: /cmd_vel")

        self.subscriber = rospy.Subscriber("/joy",Joy,self.publisher_cb)
        LOGGER.info("Subscriber initialized for topic: /joy.")

        LOGGER.info(f"{type(self).__name__} was created.")
        self.loop()
    
    def loop(self) -> None:
        LOGGER.info(f"Publishing loop running.")
        while not rospy.is_shutdown():
            self.publish()
        LOGGER.info("Publishing loop stopped.")
    
    def publisher_cb(self,data: Joy) -> None:
        x_lin = data.axes[1]
        self.vel_msg.linear.x = x_lin

        z_rot: float = data.axes[0]
        if x_lin >= 0:
            self.vel_msg.angular.z = z_rot
        else:
            self.vel_msg.angular.z = -z_rot

        if data.axes[4] < 1-2*self.DEADZONE:
            if not self.enabled:
                self.enabled = True
                LOGGER.debug("Enabled movement.")
        else:
            if self.enabled:
                self.enabled = not self.enabled
                LOGGER.debug("Disabled movement.")

    def publish(self) -> None:
        if self.enabled:
            # publishes the Twist message if movement is enabled
            self.publisher.publish(self.vel_msg)
        self.rate.sleep()

    def __del__(self) -> None:
        LOGGER.info(f"{type(self).__name__} was destroyed.")

if __name__ == "__main__":
    #rospy.init_node()
    teleop = JoypadTeleopNode()
    try:
        teleop.loop()
    except KeyboardInterrupt as e:
        LOGGER.warning("Keyboardinterrupt received.")