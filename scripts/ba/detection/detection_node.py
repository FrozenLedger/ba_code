import rospy

from ba.detection.yolov5_interface import Yolov5Model,Trashnet,DetectionAdapter

from ba_code.srv import Detect,DetectResponse
from ba_code.srv import TakeSnapshotStamped, GetMetrics, ClearFrame

from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import String

from ba.perception.rsd435_node import CAMERA_NS

class DetectionServer:
    """A node to enable the ros network to infere images using CNN's that are trained to detect objects and trash.
Services:
/trashnet/detect: Detects trash in an image. CNN based on: https://stackoverflow.com/questions/67302634/how-do-i-load-a-local-model-with-torch-hub-load
/yolov5/detect: Detects objects in an image. Based on ultralytics:Yolov5 pre-trained model.
"""
    def __init__(self,outpath:str = "/tmp/detection", inpath:str = f"/tmp/{CAMERA_NS}_images"):
        self.__outpath = outpath
        self.__inpath = inpath

        self.__yolov5 = Yolov5Model()
        self.__trashnet = Trashnet()

        snapshot_srv = f"/{CAMERA_NS}/take_snapshot/"
        self.__snapshot_server = rospy.ServiceProxy(snapshot_srv,TakeSnapshotStamped)

        self.__init_services()

        print(f"[INFO] Waiting for /{CAMERA_NS}/take_snapshot service...")
        rospy.wait_for_service(snapshot_srv)
        rospy.loginfo("[INFO] Detection node ready.")

    def __init_services(self):
        self.__yolov5_server = rospy.Service("/trashnet/detect",Detect,self.__trash_detection)
        self.__trashnet_server = rospy.Service("/yolov5/detect",Detect,self.__object_detection)

    def __trash_detection(self,request):
        return self.__detect(self.__trashnet,request.imgID)
    def __object_detection(self,request):
        return self.__detect(self.__yolov5,request.imgID)

    def __detect(self,cnn,imgID=0):
        """Makes a request to the 'take_snapshot' service and processes the image to detect objects and trash,
        as well as calculating distance measurements using the depth information of the depth camera."""
        if imgID == 0:
            request = self.__snapshot_server(add_buffer=True)
            imgID = request.imgID
            result = DetectResponse(header=request.header)
            clear_buffer = True
        else:
            result = DetectResponse()
            clear_buffer = False

        #depth = DepthImage.read_depth(inpath=self.__inpath,imgID=imgID)
        
        impath = f"{self.__inpath}/color_{imgID}.jpg"
        data = DetectionAdapter(cnn.detect(impath))
        # yolov5/trashnet headers: xmin ymin xmax ymax confidence class name
        df = data.dataframe
        
        dtype = "int16"
        xmin = df["xmin"].astype(dtype)
        xmax = df["xmax"].astype(dtype)
        ymin = df["ymin"].astype(dtype)
        ymax = df["ymax"].astype(dtype)
        for idx in range(len(xmin)):
            w = xmax[idx]-xmin[idx]
            h = ymax[idx]-ymin[idx]
            result.detection.roi.append(RegionOfInterest(x_offset=xmin[idx],y_offset=ymin[idx],width=w,height=h,do_rectify=True))
        result.detection.clsID = df["class"]
        result.detection.confidence = df["confidence"]
        result.detection.clsName = [String(data=s) for s in df["name"]]

        get_metrics = rospy.ServiceProxy(f"/{CAMERA_NS}/frames/metrics",GetMetrics)
        distance_metrics = []
        det = result.detection
        for idx,_ in enumerate(det.clsID):
            #w = det.xmax[idx] - det.xmin[idx]
            #h = det.ymax[idx] - det.ymin[idx]
            #roi = RegionOfInterest(x_offset=det.xmin[idx],
            #                       y_offset=det.ymin[idx],
            #                       width=w,
            #                       height=h,
            #                       do_rectify=True)
            #roi = det.roi[idx]
            metrics = get_metrics(imgID=imgID,roi=det.roi[idx]).metrics
            #detection = DetectionStamped(*metrics)
            distance_metrics.append(metrics) # print("depth", d)
        result.detection.metrics = distance_metrics

        if clear_buffer:
            clear_buffer = rospy.ServiceProxy(f"/{CAMERA_NS}/frames/clear",ClearFrame)
            clear_buffer(imgID=imgID)

        return result
        
def main():
    rospy.init_node("detection_node")
    server = DetectionServer()
    rospy.spin()

if __name__ == "__main__":
    main()