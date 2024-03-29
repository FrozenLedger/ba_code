import pyrealsense2 as rs
import rospy
import cv2
import numpy as np

from pathlib import Path
from cv_bridge import CvBridge

from ba_code.srv import TakeSnapshotRequest,TakeSnapshot

from sensor_msgs.msg import Image

from ba.perception import DepthImage

class RealSenseD435Proxy:
    """A class to simulate a real camera in a simulation environment."""
    def __init__(self, inpath, width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0):
        print("Running rs-proxy. This is a simulated camera.")
        self.__inpath = inpath
        
        # Protected
        self._width = width
        self._height = height
        self._format = format

        print("RealSenseD435Proxy running. [INFO] This is not a real camera.")
    
    ### properties ###
    @property
    def WIDTH(self):
        return self._width
    @property
    def HEIGHT(self):
        return self._height
    @property
    def FORMAT(self):
        return self._format
    ###################

    def take_snapshot(self) -> DepthImage:
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
        
        img_data = DepthImage(colorim=rgb_img,depthim=depth_img,depth=depth)
        return img_data
    
class RealSenseD435ServerProxy:
    """A service node to simulate a real camera service node in a simulation environment."""
    def __init__(self,camera,frame_id="rs_camera"):
        self.__camera = camera

        self.__rgb_pub = rospy.Publisher("/rs_rgb",Image,queue_size=1)

        self.__init_services(frame_id)

    def __init_services(self,frame_id):
        self.__bridge = CvBridge()
        
        self.__server = rospy.Service("/take_snapshot",TakeSnapshot,self.take_snapshot)
        self.__seq = 0
        self.__frame_id = frame_id
        rospy.loginfo("/take_snapshot service initialized.")

    def take_snapshot(self,msg:TakeSnapshotRequest):
        return self.__camera.take_snapshot().imgID

if __name__ == "__main__":
    rospy.init_node("rs435_proxy")
    inpath = "/home/workspace1/rsd435_images/"
    rs_d435_proxy = RealSenseD435Proxy(inpath=inpath)
    rs_d435_server_proxy = RealSenseD435ServerProxy(camera=rs_d435_proxy)
    rospy.spin()