import rospy
import threading, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from .teleoperation_logger import TELEOPLOGGER as LOGGER

class JoypadTeleopNode:
    """A node that takes inputs from the /joy topic and sends further control signals to the /cmd_vel topic in order to control its move behaviour."""
    def __init__(self,freq=10) -> None:
        self.vel_msg: Twist = Twist()

        self.DEADZONE: float = 0.05
        LOGGER.info(f"Joypad deadzone/sensibility set to {self.DEADZONE}.")

        self.enabled: bool = False
        LOGGER.info("Movement set to disabled.")# Disabled by default. Press [A4] to enable movement.

        self.rate: rospy.Rate = rospy.Rate(freq)
        LOGGER.info(f"Publishing rate set to {freq}/s.")
        
        self.publisher: rospy.Publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        LOGGER.info("Publisher initialized for topic: /cmd_vel")

        self.subscriber: rospy.Subscriber = rospy.Subscriber("/joy",Joy,self._publisher_cb)
        LOGGER.info("Subscriber initialized for topic: /joy.")

        self.reset()
        LOGGER.info(f"{type(self).__name__} was created.")

    def run(self) -> None:
        self._thread.run()
        self._thread = threading.Thread(target=self._loop)

    def reset(self) -> None:
        self._running = False
        self._thread = threading.Thread(target=self._loop)
        LOGGER.info("Thread object created. Loop function added as target.")

    def stop(self) -> None:
        if self._thread.is_alive():
            self._running = False
            LOGGER.info("Joining thread...")
            self._thread.join()

    def _loop(self) -> None:
        self._running = True
        LOGGER.info(f"Publishing loop running.")
        while self._running and not rospy.is_shutdown():
            try:
                self.publish() #time.sleep(0.1)
            except KeyboardInterrupt:
                self._running = False
                LOGGER.info("Keyboardinterrupt received. Shutting down loop.")
            except SystemExit:
                self._running = False
                LOGGER.info("System exited. Shutting down loop.")
        LOGGER.info("Publishing loop stopped.")
    
    def _publisher_cb(self,data: Joy) -> None:
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
                print("Enabled movement.")
        else:
            if self.enabled:
                self.enabled = not self.enabled
                print("Disabled movement.")

    def publish(self) -> None:
        if self.enabled:
            # publishes the Twist message if movement is enabled
            self.publisher.publish(self.vel_msg)
        self.rate.sleep()

    def __del__(self) -> None:
        if self._thread.is_alive():
            self.stop()
        LOGGER.info(f"{type(self).__name__} was destroyed.")
        
if __name__ == "__main__":
    rospy.init_node("joypad_teleoperator")
    teleop = JoypadTeleopNode()
    teleop.run()