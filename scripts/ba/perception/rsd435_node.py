import rospy
import pyrealsense2 as rs

from cv_bridge import CvBridge
from ba.perception.rsd435_camera import RealSenseD435

from std_srvs.srv import SetBool,SetBoolResponse
from ba_code.srv import TakeSnapshot

class RealSenseD435Server:
    def __init__(self, width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0):
        self.__camera = RealSenseD435(width=width,heigth=height,format=format,framerate=framerate,delay=delay)

        self.__stream_enable = False
        self.__framerate = rospy.Rate(framerate)
        self.__idle = rospy.Rate(1)
        self.__cvbride:CvBridge = CvBridge()

        self.__init_services()
        self.__init_publisher()

        self.__loop()

    def loop(self):
        running = True
        while running and not rospy.is_shutdown():
            try:
                self.__publish()
            except Exception as e:
                print(e)
                running = False
                self.__stream_enable = False
            self.__idle.sleep()

    def __publish(self):
        while self.__stream_enable and not rospy.is_shutdown():
            data = self.__camera.take_snapshot()

            color_msg = self.__cvbride.cv2_to_imgmsg(data.colorim)
            self.__color_pub.publish(color_msg)
            #self.__depth_pub.publsih(depth_msg)
            #self.__scan_pub.publish(scan_msg)

            self.__framerate.sleep()

    def __init_services(self):
        self.__snapshot_server = rospy.Service("/rs_d435/take_snapshot",TakeSnapshot,self.__take_snapshot)
        self.__stream_enabler = rospy.Service("/rs_d435/stream_enable",SetBool,self.__enable_stream)
        self.__roi_setter = rospy.Service("/rs_d435/set_roi")

    def __init_publisher(self):
        self.__color_pub = rospy.Publisher("/rs_d435/color/image")
        self.__depth_pub = rospy.Publisher("/rs_d435/depth/image")
        self.__scan_pub = rospy.Publsiher("/rs_d435/depth/scan")
        
    def __enable_stream(self,request):
        response = SetBoolResponse()
        try:
            self.__stream_enable = request.data
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def __remove_services(self):
        self.__snapshot_server.unregister()
        self.__stream_enabler.unregister()
        self.__roi_setter.unregister()

    def __del__(self):
        self.__remove_services()

    def take_snapshot(self,msg):
        data = self.__camera.take_snapshot()
        raise NotImplementedError()

def main():
    rospy.init_node("rs_d435")
    rs_server = RealSenseD435Server()
    rospy.spin()

if __name__ == "__main__":
    main()