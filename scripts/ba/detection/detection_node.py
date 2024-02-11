import rospy,cv2
import numpy as np

from ba.detection.yolov5_interface import Yolov5Model,Trashnet,DetectionAdapter
from ba.perception.rsd435_node import DepthImage

from ba_code.srv import Detect,DetectResponse
from ba_code.srv import TakeSnapshotStamped, GetMetrics, ClearFrame

from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest

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
            request = self.__snapshot_server(add_buffer=True)
            imgID = request.imgID
            result = DetectResponse(header=request.header)
        else:
            result = DetectResponse()
        
        #depth = DepthImage.read_depth(inpath=self.__inpath,imgID=imgID)
        
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

        get_metrics = rospy.ServiceProxy("/rs_d435/get_distance_metrics",GetMetrics)
        distance_metrics = []
        det = result.detection
        for idx,_ in enumerate(det.clsID):
            w = det.xmax[idx] - det.xmin[idx]
            h = det.ymax[idx] - det.ymin[idx]
            roi = RegionOfInterest(x_offset=det.xmin[idx],
                                   y_offset=det.ymin[idx],
                                   width=w,
                                   height=h,
                                   do_rectify=True)
            metrics = get_metrics(imgID=imgID,roi=roi).metrics
            #detection = DetectionStamped(*metrics)
            distance_metrics.append(metrics) # print("depth", d)
        result.detection.metrics = distance_metrics

        clear_buffer = rospy.ServiceProxy("/rs_d435/clear_frame",ClearFrame)
        clear_buffer(imgID=imgID)

        return result
        
def main():
    rospy.init_node("detection_node")
    server = DetectionServer()
    rospy.spin()

if __name__ == "__main__":
    main()