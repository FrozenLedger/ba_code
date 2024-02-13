import rospy,tf

from ba_code.srv import Detect
from ba_code.msg import ObjectPosition

import ba_code.srv as basrv

import numpy as np

from geometry_msgs.msg import PointStamped,Point
from std_msgs.msg import Header,String

class ObjectPublisherProxy:
    def __init__(self):
        self.__rate = rospy.Rate(.1)
        self.__tf_listener = tf.TransformListener()
        self.__world = "map"
        self.__origin = "base_link"
        self.__trash_pub = rospy.Publisher("/detection/trash",ObjectPosition,queue_size=20)
        self.__object_pub = rospy.Publisher("/detection/object",ObjectPosition,queue_size=20)
        self.__tf_listener = tf.TransformListener()

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.__publish()
                print("Published.")
            except tf.LookupException as lue:
                print(lue)
            except TypeError as te:
                print(te)
            self.__rate.sleep()

    def __publish(self):
        t0 = rospy.Time.now()
        tf.waitForTransform(self.__origin,self.__world,t0,rospy.Duration(1))
        header = Header(stamp=t0,frame_id=self.__origin)
        cur_pnt = PointStamped(header=header)

        map_pnt = self.__tf_listener.transformPoint(self.__world,cur_pnt)
        msg = ObjectPosition(point=map_pnt,note=String("Type:trash"),clsID=42,confidence=0.5)
        self.__trash_pub.publish(msg)

class ObjectPublisher:
    def __init__(self):
        self.__tf_listener = tf.TransformListener()
        self.__world = "map"
        self.__init_subscriptions()
        self.__rate = rospy.Rate(1)
        self.__trash_pub = rospy.Publisher("/detection/trash",ObjectPosition,queue_size=20)
        self.__obj_pub = rospy.Publisher("/detection/objects",ObjectPosition,queue_size=20)

        self.__take_snapshot = rospy.ServiceProxy("/rs_d435/take_snapshot",basrv.TakeSnapshotStamped)
        self.__clear_frame = rospy.ServiceProxy("/rs_d435/frames/clear",basrv.ClearFrame)
        self.__pixel_to_point3D = rospy.ServiceProxy("/rs_d435/pixel_to_point3d",basrv.PixelToPoint3D)

    def __init_subscriptions(self):
        yolov5_service = "/yolov5/detect"
        trashnet_service = "/trashnet/detect"
    
        print(f"Waiting for services: {yolov5_service}, {trashnet_service} ...")
        rospy.wait_for_service(yolov5_service)
        rospy.wait_for_service(trashnet_service)
        self.__detect_object = rospy.ServiceProxy(yolov5_service,Detect)
        self.__detect_trash = rospy.ServiceProxy(trashnet_service,Detect)
        print("Services available.")

    def __calcPoint(self,detection,idx):
        distance = detection.distance
        dist = max(distance[idx].median,distance[idx].center)
        if dist <= 100:
            print("[Warning] Distance to low. Ignore prediction.")
        print(detection.clsID[idx],detection.confidence[idx],distance[idx].median)

        return Point(x = dist)
    
    def __publish_object(self,detection,idx,snap):
        dist = max(detection.metrics[idx].center,detection.metrics[idx].median)
        if dist == 0 or np.isnan(dist):
            print("[Err]")
            return

        w = detection.xmax[idx] - detection.xmin[idx]
        h = detection.ymax[idx] - detection.ymin[idx]
        px = detection.xmin[idx] + w//2
        py = detection.ymax[idx] + h//2
        
        pnt = self.__pixel_to_point3D(imgID=snap.imgID,
                                      px=px,
                                      py=py,
                                      distance=dist).point

        pnt = PointStamped(point=pnt,header=snap.header)
        msg = ObjectPosition(point=pnt,
                            note=String(data="object"),
                            clsID=detection.clsID[idx],
                            confidence=detection.confidence[idx])
        self.__trash_pub.publish(msg)

    def __publish_trash(self,detection,idx,snap):
        print(f"T[{detection.clsID[idx]}]")
        
        #pnt = PointStamped(header=header)

        #msg = ObjectPosition()
        #self.__obj_pub.publish(msg)
        return ObjectPosition()

    def __publish(self):
        snap = self.__take_snapshot(add_buffer=True)
        imgID = snap.imgID
        try:
            object_detection = self.__detect_object(imgID=imgID).detection
            trash_detection = self.__detect_trash(imgID=imgID).detection

            for idx,_ in enumerate(object_detection.clsID):
                self.__publish_object(object_detection,idx,snap)
            for idx,_ in enumerate(trash_detection.clsID):
                pass
                #self.__publish_trash(trash_detection,idx,snap)
        except Exception as e:
            print(e)
        finally:
            self.__clear_frame(imgID)

    def loop(self):
        while not rospy.is_shutdown():
            self.__publish()
            self.__rate.sleep()

def main():
    rospy.init_node("object_publisher")
    object_pub = ObjectPublisher()
    #object_pub = ObjectPublisherProxy()
    object_pub.loop()

if __name__ == "__main__":
    main()