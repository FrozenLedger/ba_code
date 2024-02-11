import rospy, cv2
import pyrealsense2 as rs

from cv_bridge import CvBridge
from ba.perception.rsd435_camera import RealSenseD435

from std_srvs.srv import SetBool,SetBoolResponse

from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2

from ba_code.msg import Distance
from ba_code.srv import TakeSnapshotStamped,TakeSnapshotStampedResponse,GetDistance,GetDistanceResponse

from pathlib import Path
import numpy as np

class RealSenseD435Server:
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
            data = self.__camera.take_snapshot()

            header = Header(frame_id="camera_link")
            
            color_msg = self.__cvbride.cv2_to_imgmsg(data.colorim)
            depth_msg = self.__cvbride.cv2_to_imgmsg(data.depthim)
            
            depth_data_msg = PointCloud2(header=header)
            
            self.__color_pub.publish(color_msg)
            self.__depth_pub.publish(depth_msg)
            #self.__deptharr_pub.publish(depth_data_msg)

            self.__framerate.sleep()

    def __init_services(self):
        self.__snapshot_server = rospy.Service("/rs_d435/take_snapshot",TakeSnapshotStamped,self.__take_snapshot)
        self.__stream_enabler = rospy.Service("/rs_d435/stream_enable",SetBool,self.__enable_stream)
        self.__distance_server = rospy.Service("rs_d435/get_distance",GetDistance,self.__get_distance)
        #self.__roi_setter = rospy.Service("/rs_d435/set_roi")

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

    def __take_snapshot(self,request):
        frame_buffer = self.__camera.take_snapshot()
        #data = self.__camera.take_snapshot()

        imgID = int(str(rospy.Time.now()))
        self.__add_frame_buffer(frame_buffer,imgID=imgID)

        try:
            cv2.imwrite(f"{self.__outpath}/color_{imgID}.jpg",frame_buffer.colorim)
            print(f"[INFO] RGB image saved with imgID:{imgID}")
        except Exception as e:
            print(e)

        response = TakeSnapshotStampedResponse(header=Header(stamp=rospy.Time.now(),frame_id=self.__frame_id),imgID=imgID)
        #print(f"[Header] {response.header}")
        return response
    
    def __get_distance(self, request):
        roi = request.roi
        if roi.do_rectify:
            region = (roi.x_offset,roi.y_offset,roi.width,roi.height)
            img = self.__camera.take_snapshot()
            # debugging: img.write(outpath=self.__outpath,region=region)
            data = img.region_to_depth(*region,center_size=5)
        else:
            img = self.__camera.take_snapshot()
            # debugging: img.write(outpath=self.__outpath)
            data = img.image_to_depth(center_size=5)
        response = GetDistanceResponse()
        response.distance = Distance(*data)
        return response
    
    def __add_frame_buffer(self,frame_buffer,imgID):
        if len(self.__buffer) < self.__max_buffer_size:
            self.__buffer[imgID] = frame_buffer
            self.__buffer_count += 1
            return True
        else:
            print(f"[Warning] Buffer is full. Please clear some frames from the buffer: BufferSize[{len(self.__buffer)}/{self.__max_buffer_size}")
        return False
    
    def __remove_frame_buffer(self,imgID):
        if imgID in self.__buffer:
            del self.__buffer[imgID]
            self.__buffer_count -= 1
            return True
        print(f"[Warning] Frame buffer with imgID:{imgID} not in buffer.")
        return False

def get_distance(area,px,py,size=0):
        if size == 0:
            return area[py,px]
        else:
            h,w = area.shape
            subarea = area[max(0,py-size):min(h,py+size+1),max(0,px-size):min(w,px+size+1)]
            ah,aw = area.shape
            #print(aw,ah,px,py,size)
            return np.median(subarea)

def main():
    rospy.init_node("rs_d435")
    rs_server = RealSenseD435Server(delay=0)
    rospy.spin()

if __name__ == "__main__":
    main()

class DepthImage:
    def __init__(self,colorim=np.zeros([]),depthim=np.zeros([]),depth=np.zeros([])):
        self.__colorim = colorim
        self.__depthim = depthim
        self.__depth = depth

    @property
    def depthim(self):
        return self.__depthim
    @property
    def colorim(self):
        return self.__colorim
    @property
    def depth(self):
        return self.__depth

    def pixel_to_depth(self, px,py) -> int:
        """Returns the distance value of the pixelcoordinate (<x>,<y>) read from the depth data."""
        #return self.__alignment.get_depth_frame().get_distance(x,y)
        dist = self.__depth_data[py,px]
        print(f"Distance data at ({px,py} = {dist} mm")
        return dist #self.__aligned_depth_frames.get_distance(x,y)
    
    def image_to_depth(self,center_size=0):
        return self.__area_to_depth(self.__depth,center_size=center_size)
    
    def region_to_depth(self,x_offset,y_offset,width,height,center_size=0):
        h,w = self.__depth.shape
        area = self.__depth[max(0,y_offset):min(h,y_offset+height),max(0,x_offset):min(w,x_offset+width)]
        return self.__area_to_depth(area,center_size=center_size)
        
    def __area_to_depth(self,area,center_size=0):
        h,w = area.shape
        px = w//2
        py = h//2

        center_dist = get_distance(area,px=px,py=py,size=center_size)
        min_dist = np.min(area)
        max_dist = np.max(area)
        avg_dist = np.average(area)
        median_dist = np.median(area)

        return (center_dist,min_dist,max_dist,avg_dist,median_dist)
        
    def write(self,outpath:str,region = (0,0,0,0),imgID=0,subname=""):
        if region == (0,0,0,0):
            colorim = self.__colorim
            depthim = self.__depthim
            deptharr = self.__depth
        else:
            h,w = self.__depth.shape
            x_offset,y_offset,width,height = region
            x_start = max(0,x_offset)
            x_end = min(w,x_offset+width)
            y_start = max(0,y_offset)
            y_end = min(h,y_offset+height)
            colorim = self.__colorim[y_start:y_end,x_start:x_end]
            depthim = self.__depthim[y_start:y_end,x_start:x_end]
            deptharr = self.__depth[y_start:y_end,x_start:x_end]
        cv2.imwrite(f"{outpath}/color_{imgID}.jpg",colorim)
        cv2.imwrite(f"{outpath}/depth_{imgID}.jpg",depthim)
        with open(f"{outpath}/depth_{imgID}.npy","wb") as npyf:
            np.save(npyf,deptharr)

    def read(self,inpath:str,imgID=0):
        self.__colorim = cv2.imread(f"{inpath}/color_{imgID}.jpg")
        self.__depthim = cv2.imread(f"{inpath}/depth_{imgID}.jpg")
        return self
    
    @classmethod
    def read_depth(cls,inpath:str,imgID):
        with open(f"{inpath}/depth_{imgID}.npy","rb") as npyf:
            depth = np.load(npyf)
        return DepthImage(depth=depth)