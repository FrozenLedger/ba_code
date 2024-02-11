import rospy,cv2
import numpy as np

from ba.detection.yolov5_interface import Yolov5Model,Trashnet,DetectionAdapter
from ba.perception.rsd435_camera import DepthImage

from ba_code.msg import Distance
from ba_code.srv import Detect,DetectResponse
from ba_code.srv import TakeSnapshotStamped

from std_msgs.msg import Header

class DetectionServer:
    def __init__(self,outpath:str = "/tmp/detection", inpath:str = "/tmp/rsd435_images"):
        self.__outpath = outpath
        self.__inpath = inpath

        self.__yolov5 = Yolov5Model()
        self.__trashnet = Trashnet()

        self.__snapshot_server = rospy.ServiceProxy("/rs_d435/take_snapshot/",TakeSnapshotStamped)

        self.__init_services()
        rospy.loginfo("Detection node ready.")

    def __init_services(self):
        self.__yolov5_server = rospy.Service("/trashnet/detect",Detect,self.__trash_detection)
        self.__trashnet_server = rospy.Service("/yolov5/detect",Detect,self.__object_detection)

    def __trash_detection(self,request):
        return self.__detect(self.__trashnet,request.imgID)
    def __object_detection(self,request):
        return self.__detect(self.__yolov5,request.imgID)

    def __detect(self,cnn,imgID=0):
        if imgID == 0:
            request = self.__snapshot_server()
            imgID = request.imgID
            result = DetectResponse(header=request.header)
        else:
            result = DetectResponse()
        
        depth = DepthImage.read_depth(inpath=self.__inpath,imgID=imgID)
        
        impath = f"{self.__inpath}/color_{imgID}.jpg"
        data = DetectionAdapter(cnn.detect(impath))
        # yolov5/trashnet headers: xmin ymin xmax ymax confidence class name
        df = data.dataframe
        
        dtype = "int16"
        result.detection.xmin = df["xmin"].astype(dtype)
        result.detection.xmax = df["xmax"].astype(dtype)
        result.detection.ymin = df["ymin"].astype(dtype)
        result.detection.ymax = df["ymax"].astype(dtype)
        result.detection.clsID = df["class"]
        result.detection.confidence = df["confidence"]

        distances = []
        det = result.detection
        for idx,_ in enumerate(det.clsID):
            xmin = det.xmin[idx]
            width = det.xmax[idx]-xmin
            ymin = det.ymin[idx]
            height = det.ymax[idx]-ymin
            region = (xmin,ymin,width,height)
            xmin = det.xmin
            d = Distance(*depth.region_to_depth(*region,3))
            distances.append(d)# print("depth", d)
        result.detection.distance = distances
        return result
        
def main():
    rospy.init_node("detection_node")
    server = DetectionServer()
    rospy.spin()

if __name__ == "__main__":
    main()