import rospy, cv2
import pyrealsense2 as rs

from cv_bridge import CvBridge
from ba.perception.rsd435_camera import RealSenseD435

from std_srvs.srv import SetBool,SetBoolResponse

from std_msgs.msg import Header
from sensor_msgs.msg import Image

import ba_code.srv as basrv

from pathlib import Path
import numpy as np

class RealSenseD435Server:
    """A server node that exposes a limited set of camera functionalities to the ros network."""
    # Camera link: camera_link
    # Color image frame: camera_color_optical_frame
    # Depth image frame: camera_depth_optical_frame
    def __init__(self, frame_id="camera_link",outpath="/tmp/rsd435_images/", width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0,max_buffer_size=10):
        self.__camera = RealSenseD435(width=width,height=height,format=format,framerate=framerate,delay=delay)

        self.__frame_id = frame_id
        self.__outpath = outpath
        Path(outpath).mkdir(parents=True, exist_ok=True)

        self.__stream_enable = False
        self.__framerate = rospy.Rate(framerate)
        self.__idle = rospy.Rate(1)
        self.__cvbride:CvBridge = CvBridge()

        self.__buffer = {}
        self.__max_buffer_size = max_buffer_size
        self.__buffer_count = 0

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
            t0 = rospy.Time.now()
            header = Header(stamp=t0,frame_id=self.__frame_id)
            data = self.__camera.take_snapshot(header)

            #header = Header(frame_id="camera_link")
            
            color_msg = self.__cvbride.cv2_to_imgmsg(data.colorim)
            color_msg.header.frame_id = self.__frame_id
            depth_msg = self.__cvbride.cv2_to_imgmsg(data.depthim)
            depth_msg.header.frame_id = self.__frame_id

            #depth_data_msg = PointCloud2(header=header)
            
            self.__color_pub.publish(color_msg)
            self.__depth_pub.publish(depth_msg)
            #self.__deptharr_pub.publish(depth_data_msg)

            self.__framerate.sleep()

    def __init_services(self):
        self.__snapshot_server = rospy.Service("/rs_d435/take_snapshot",basrv.TakeSnapshotStamped,self.__take_snapshot)
        self.__stream_enabler = rospy.Service("/rs_d435/stream_enable",SetBool,self.__enable_stream)
        #self.__distance_server = rospy.Service("rs_d435/get_distance",basrv.GetDistance,self.__get_distance)
        self.__clear_frame_server = rospy.Service("/rs_d435/frames/clear",basrv.ClearFrame,self.__clear_frame_request)
        self.__color_crame_server = rospy.Service("rs_d435/frames/color",basrv.SendImage,self.__send_color)
        self.__depth_frame_server = rospy.Service("/rs_d435/frames/depthim",basrv.SendImage,self.__send_depthim)
        self.__pixel_to_point3d_server = rospy.Service("/rs_d435/frames/deproject_pixel_to_point3d",basrv.PixelToPoint3D,self.__deproject_pixel_to_point3d)
        self.__get_distance_metrics_server = rospy.Service("/rs_d435/frames/metrics",basrv.GetMetrics,self.__distance_metrics_from_area)
        #self.__get_distance_metrics_server = rospy.Service("/rs_d435/get_distance_metrics",basrv.GetMetrics,self.__distance_metrics_from_area)

    def __init_publisher(self):
        self.__color_pub = rospy.Publisher("/rs_d435/color/image",Image,queue_size=1)
        self.__depth_pub = rospy.Publisher("/rs_d435/depth/image",Image,queue_size=1)
        #self.__deptharr_pub = rospy.Publisher("/rs_d435/depth/data",PointCloud2,queue_size=1)
        
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
    
    def __send_color(self,request):
        imgID = request.imgID
        if imgID not in self.__buffer:
            return basrv.SendImageResponse()
        return self.__send_image_response(self.__buffer[imgID].colorim,request.roi)

    def __send_depthim(self,request):
        imgID = request.imgID
        if imgID not in self.__buffer:
            return basrv.SendImageResponse()
        return self.__send_image_response(self.__buffer[imgID].depthim,request.roi)
        
    def __send_image_response(self,img,roi):
        if roi.do_rectify:
            img = img[roi.y_offset:roi.y_offset+roi.height, roi.x_offset:roi.x_offset+roi.width]
        img_msg = self.__cvbride.cv2_to_imgmsg(img)
        return basrv.SendImageResponse(data=img_msg)

    def __take_snapshot(self,request):
        t0 = rospy.Time.now()
        header = Header(stamp=t0,frame_id=self.__frame_id)
        frame_buffer = self.__camera.take_snapshot(header)

        imgID = int(str(t0))
        if request.add_buffer:
            try:
                self.__add_frames(frame_buffer,imgID=imgID)
            except MemoryError as e:
                print(e)
        try:
            cv2.imwrite(f"{self.__outpath}/color_{imgID}.jpg",frame_buffer.colorim)
            print(f"[INFO] RGB image saved with imgID:{imgID}")
        except Exception as e:
            print(e)

        response = basrv.TakeSnapshotStampedResponse(header=Header(stamp=rospy.Time.now(),frame_id=self.__frame_id),imgID=imgID)
        return response
    
    def __add_frames(self,frame_buffer,imgID):
        if len(self.__buffer) < self.__max_buffer_size:
            self.__buffer[imgID] = frame_buffer
            self.__buffer_count += 1
            print(f"[INFO] Added frames to buffer with imgID: {imgID} -> buffer[{self.__buffer_count}/{self.__max_buffer_size}]")
            return
        raise MemoryError("[Error] Buffersize exceeded. Please clear some frames from the RealSenseD435.frame_buffer.")

    def __clear_frame_request(self,request):
        response = basrv.ClearFrameResponse()
        response.success = self.__remove_frames(request.imgID)    
        return response
    
    def __remove_frames(self,imgID):
        if imgID in self.__buffer:
            del self.__buffer[imgID]
            self.__buffer_count -= 1
            print(f"[INFO] Removed frames from buffer with imgID: {imgID} -> buffer[{self.__buffer_count}/{self.__max_buffer_size}]")
            return True
        print(f"[Warning] Frame buffer with imgID:{imgID} not in buffer.")
        return False
    
    def __deproject_pixel_to_point3d(self,request):
        imgID = request.imgID
        if imgID in self.__buffer:
            pnt = self.__buffer[imgID].deproject_pixel_to_point3D(request.px,request.py,request.distance)
            return basrv.PixelToPoint3DResponse(point=pnt)
        raise ValueError(f"No frames with imgID: {imgID} in buffer.")
    
    def __distance_metrics_from_area(self,request):
        response = basrv.GetMetricsResponse()
        try:
            metrics = self.__buffer[request.imgID].get_distance_metrics(request.roi)
            response = basrv.GetMetricsResponse(metrics=metrics)
        except Exception as e:
            response = basrv.GetMetricsResponse()
            print(e)
        return response

def get_distance(area,px,py,size=0):
        if size == 0:
            return area[py,px]
        else:
            h,w = area.shape
            subarea = area[max(0,py-size):min(h,py+size+1),max(0,px-size):min(w,px+size+1)]
            #ah,aw = area.shape
            #print(aw,ah,px,py,size)
            return np.median(subarea)

def main():
    rospy.init_node("rs_d435")

    frame_id = "camera_color_optical_frame"
    rs_server = RealSenseD435Server(delay=0,width=1280,height=720,frame_id=frame_id)
    rospy.spin()

if __name__ == "__main__":
    main()