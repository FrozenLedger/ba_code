import rospy,cv2

from cv_bridge import CvBridge
from pathlib import Path

from sensor_msgs.msg import RegionOfInterest

import ba_code.srv as basrv

class ObjectDetectionImager:
    """A node that simplifies the process of detecting objects and trash in the scene and saves the images locally for debuggin purposes."""
    def __init__(self):
        self.__take_snapshot = rospy.ServiceProxy("/rs_d435/take_snapshot",basrv.TakeSnapshotStamped)
        self.__clear_frame = rospy.ServiceProxy("/rs_d435/frames/clear",basrv.ClearFrame)
        self.__detect_trash = rospy.ServiceProxy("trashnet/detect",basrv.Detect)
        self.__detect_object = rospy.ServiceProxy("yolov5/detect",basrv.Detect)
        self.__send_color = rospy.ServiceProxy("/rs_d435/frames/color",basrv.SendImage)

        self.__cv_bridge = CvBridge()
        self.__path = "/tmp/imager/"
        Path(self.__path).mkdir(exist_ok=True)
        
    def run(self):
        imgID = self.__take_snapshot(add_buffer=True).imgID
        im_msg = self.__send_color(imgID=imgID).data
        im = self.__cv_bridge.imgmsg_to_cv2(im_msg)

        self.__detect(cnn=self.__detect_trash,subname="trash_",imgID=imgID,im=im)
        self.__detect(cnn=self.__detect_object,subname="object_",imgID=imgID,im=im)
        self.__clear_frame(imgID)

    def __detect(self,cnn,subname,imgID,im):
        data = cnn(imgID=imgID).detection
        for idx in range(len(data.clsID)):
            w = data.xmax[idx]-data.xmin[idx]
            h = data.ymax[idx]-data.ymin[idx]
            roi = RegionOfInterest( x_offset=data.xmin[idx],
                                    y_offset=data.ymin[idx],
                                    width=w,
                                    height=h,
                                    do_rectify=True)
            self.__write_img(im,roi,idx,subname=subname)

    def __write_img(self,im,roi:RegionOfInterest,idx,subname):
        try:
            if roi.do_rectify:
                im = im[    roi.y_offset:roi.y_offset+roi.height,
                            roi.x_offset:roi.x_offset+roi.width]    
            cv2.imwrite(f"{self.__path}/{subname}img_{idx}.jpg",im)
        except Exception as e:
            print(e)

def main():
    rospy.init_node("object_detection_imager")

    imager = ObjectDetectionImager()
    imager.run()

if __name__ == "__main__":
    main()