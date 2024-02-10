import rospy
import pyrealsense2 as rs

from cv_bridge import CvBridge
from ba.perception.rsd435_camera import RealSenseD435

from std_srvs.srv import SetBool,SetBoolResponse

from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2

from ba_code.msg import Distance
from ba_code.srv import TakeSnapshot,TakeSnapshotResponse,GetDistance,GetDistanceResponse

from pathlib import Path

class RealSenseD435Server:
    def __init__(self, outpath="/tmp/rsd435_images/", width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0):
        self.__camera = RealSenseD435(width=width,height=height,format=format,framerate=framerate,delay=delay)

        self.__outpath = outpath
        Path(outpath).mkdir(parents=True, exist_ok=True)

        self.__stream_enable = False
        self.__framerate = rospy.Rate(framerate)
        self.__idle = rospy.Rate(1)
        self.__cvbride:CvBridge = CvBridge()

        self.__init_services()
        self.__init_publisher()

        self.__loop()

    def __loop(self):
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

            header = Header(frame_id="camera_link",stamp=rospy.Time.now())
            
            color_msg = self.__cvbride.cv2_to_imgmsg(data.colorim)
            depth_msg = self.__cvbride.cv2_to_imgmsg(data.depthim)
            
            depth_data_msg = PointCloud2(header=header)
            
            self.__color_pub.publish(color_msg)
            self.__depth_pub.publish(depth_msg)
            #self.__deptharr_pub.publish(depth_data_msg)

            self.__framerate.sleep()

    def __init_services(self):
        self.__snapshot_server = rospy.Service("/rs_d435/take_snapshot",TakeSnapshot,self.__take_snapshot)
        self.__stream_enabler = rospy.Service("/rs_d435/stream_enable",SetBool,self.__enable_stream)
        self.__distance_server = rospy.Service("rs_d435/get_distance",GetDistance,self.__get_distance)
        #self.__roi_setter = rospy.Service("/rs_d435/set_roi")

    def __init_publisher(self):
        self.__color_pub = rospy.Publisher("/rs_d435/color/image",Image,queue_size=1)
        self.__depth_pub = rospy.Publisher("/rs_d435/depth/image",Image,queue_size=1)
        self.__deptharr_pub = rospy.Publisher("/rs_d435/depth/data",PointCloud2,queue_size=1)
        
    def __enable_stream(self,request):
        response = SetBoolResponse()
        try:
            self.__stream_enable = request.data
            response.success = True
            print(f"Streaming enabled: {request.data}")
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def __take_snapshot(self,request):
        data = self.__camera.take_snapshot()
        response = TakeSnapshotResponse()
        if request.roi.do_rectify:
            x_offset = request.roi.x_offset
            y_offset = request.roi.y_offset
            width = request.roi.width
            height = request.roi.height
            color = data.colorim[y_offset:height,x_offset:width,:]
            depth = data.depth[y_offset:height,x_offset:width]
        else:
            color = data.colorim
            depth = data.depth
        if request.return_depth:
            print("Returning depth values not implemented yet.")
        if request.return_color:
            print("Returning color values not implemented yet.")
        return response
    
    def __get_distance(self, request):
        roi = request.roi
        if roi.do_rectify:
            region = (roi.x_offset,roi.y_offset,roi.width,roi.height)
            img = self.__camera.take_snapshot()
            img.write(outpath=self.__outpath,region=region)
            data = img.region_to_depth(*region,center_size=10)
        else:
            img = self.__camera.take_snapshot()
            img.write(outpath=self.__outpath)
            data = img.image_to_depth(center_size=50)
        response = GetDistanceResponse()
        response.distance = Distance(*data)
        return response

def main():
    rospy.init_node("rs_d435")
    rs_server = RealSenseD435Server(delay=0)
    rospy.spin()

if __name__ == "__main__":
    main()