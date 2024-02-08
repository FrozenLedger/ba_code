import pyrealsense2 as rs
import rospy
import cv2
import numpy as np

from pathlib import Path
from cv_bridge import CvBridge

from ba_code.srv import TakeSnapshotRequest,TakeSnapshot

from sensor_msgs.msg import Image

class ImageData:
    def __init__(self,rgb_img,depth_img,nparray,imgID):
        self.__rgb_img = rgb_img
        self.__depth_img = depth_img
        self.__nparray = nparray
        self.__imgID = imgID

    @property
    def rgb(self):
        return self.__rgb_img
    
    @property
    def depth(self):
        return self.__depth_img
    
    @property
    def nparray(self):
        return self.__nparray
    
    @property
    def imgID(self):
        return self.__imgID

class RealSenseD435Proxy:
    def __init__(self, inpath, outpath:str = "/tmp/rs_d435_images/", width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0):
        # create the path where the images will be stored, if it doesn't exist
        Path(outpath).mkdir(parents=True, exist_ok=True)

        # variables
        self.__outpath = outpath
        self.__inpath = inpath
        #self.__running = False

        print("Running rs-proxy. This is a simulated camera.")

        self.__delay_amount = delay
        
        # Protected
        self._width = width
        self._height = height

        print("RealSenseD435Proxy running. [INFO] This is not a real camera.")
        
    def take_snapshot(self) -> ImageData:
        imgID = 1702375746

        rgb_img = cv2.imread(f"{self.__inpath}/Images/snapshot_{imgID}/rgb_img_{imgID}.jpg")
        depth_img = cv2.imread(f"{self.__inpath}/Images/snapshot_{imgID}/rgb_img_{imgID}.jpg")
        depth = np.fromfile(f"{self.__inpath}/Images/snapshot_{imgID}/depth_img_{imgID}.npy")

        impath = f"{self.__outpath}/Images/snapshot_{imgID}"
        Path(impath).mkdir(parents=True, exist_ok=True)
        
        try:
            b1 = cv2.imwrite(f"{impath}/rgb_img{imgID}.jpg",rgb_img)
            b2 = cv2.imwrite(f"{impath}/depth_img{imgID}.jpg",depth_img)
            np.save(f"{impath}/depth_img_{imgID}.npy", depth)
            
            print(f"imshape: {rgb_img.shape}",f"imsave: RGB({b1}),DEPTH({b2})")
        except  Exception as e:
            print(e)
        
        img_data = ImageData(rgb_img=rgb_img,depth_img=depth_img,nparray=depth,imgID=imgID)
        return img_data
    
class RealSenseD435ServerProxy:
    def __init__(self,camera,frame_id="rs_camera"):
        self.__camera = camera

        self.__rgb_pub = rospy.Publisher("/rs_rgb",Image,queue_size=1)

        self.__init_services(frame_id)

    def __init_services(self,frame_id):
        self.__bridge = CvBridge()
        
        self.__server = rospy.Service("/take_sapshot",TakeSnapshot,self.take_snapshot)
        self.__seq = 0
        self.__frame_id = frame_id
        rospy.loginfo("/take_snapshot service initialized.")

    #def __publish(self,data:ImageData):
        #header = Header(stamp=rospy.Time.now(),frame_id=self.__frame_id,seq=self.__seq)
        #self.__seq += 1
        
        #Image = Image(header=header)
        #dtype, n_channels = self.__bridge.encoding_as_cvtype2()
        
        #rgb_msg = self.__bridge.cv2_to_compressed_imgmsg(data.rgb)
    #    rgb_msg = self.__bridge.cv2_to_imgmsg(data.rgb)

        #print(type(rgb_msg))
    #    self.__rgb_pub.publish(rgb_msg)

    def take_snapshot(self,msg:TakeSnapshotRequest):
        return self.__camera.take_snapshot().imgID

if __name__ == "__main__":
    from ba.perception.rscamera_proxy import RealSenseD435Proxy

    rospy.init_node("rs435_proxy")
    inpath = "/home/workspace1/rsd435_images/"
    rs_d435_proxy = RealSenseD435Proxy(inpath=inpath)
    rs_d435_server_proxy = RealSenseD435ServerProxy(camera=rs_d435_proxy)
    rospy.spin()