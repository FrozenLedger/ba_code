import rospy

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Trigger

from ba.utilities.roshandler import ROSTriggerHandler

class DummyCamera:
    def take_snapshot(self):
        print("Dummy snapshot taken...")

class ROSImagePublisher:
    """
    \tA simple publisher that starts a thread and publishes images when enabled.\n
    
    Properties:
    prefix: str\n\tA prefix for name resolution of the created services and topics.\n
    Exposed Services:\n
    \t{ prefix }/enable\tPublishes the images taken by the camera.\n
    \t{ prefix }/disable\ŧStops the publishing.\n
    
    Expose Topics:\n
    \t{ prefix }/image\tThe topic to receive the images from.\n
    """
    def __init__(self,imager, enabled: bool = False, prefix="",rate: rospy.Rate=None):
        self._imager = imager
        self._enabled = enabled
        self._rate = rate or rospy.Rate(1)
        self._prefix = prefix
        self.__start_thread()
        self.__init_publisher()
        self.__init_services()

    def __init_services(self):
        self._enable_service = rospy.Service(f"{self._prefix}/enable",Trigger, ROSTriggerHandler(self.enable))
        self._disable_service = rospy.Service(f"{self._prefix}/disable",Trigger, ROSTriggerHandler(self.disable))

    def __init_publisher(self):
        self._publisher = rospy.Publisher(f"{self._prefix}/image",String,queue_size=10)

    def __start_thread(self):
        self._t = Thread(target=self.__publish)
        self._t.start()

    def __publish(self):
        while not rospy.is_shutdown():
            if self._enabled:
                self._imager.take_snapshot()
                self._publisher.publish(f"Hello i am {self._prefix}")
            self._rate.sleep()

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    def __del__(self):
        print("End thread started by ROSImagePublisher...")
        self._t.join()
        print("Thread closed.")

if __name__ == "__main__":
    rospy.init_node("ros_image_publisher_test_node")

    camera = DummyCamera()
    #depthpub = ROSImagePublisher(DummyCamera(), False, prefix="/rs_d435/depth")
    colorpub = ROSImagePublisher(DummyCamera(), False, prefix="/rs_d400/color", rate=rospy.Rate(1))