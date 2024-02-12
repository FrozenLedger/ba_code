import rospy,tf

from ba_code.srv import Detect
from ba_code.msg import ObjectPosition

from geometry_msgs.msg import PointStamped,Point
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import Header,String

class ObjectPublisherProxy:
    def __init__(self):
        self.__rate = rospy.Rate(.1)
        self.__tf_listener = tf.TransformListener()
        self.__target_frame = "map"
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
        header = Header(stamp=rospy.Time.now()-rospy.Duration(0.02),frame_id="base_link")
        cur_pnt = PointStamped(header=header)

        map_pnt = self.__tf_listener.transformPoint(self.__target_frame,cur_pnt)
        msg = ObjectPosition(point=map_pnt,note=String("Type:trash"),clsID=42,confidence=0.5)
        self.__trash_pub.publish(msg)

class ObjectPublisher:
    def __init__(self):
        self.__tf_listener = tf.TransformListener()
        self.__target_frame = "map"
        self.__init_subscriptions()
        self.__rate = rospy.Rate(1)
        self.__trash_pub = rospy.Publisher("/detection/trash",ObjectPosition,queue_size=20)
        self.__obj_pub = rospy.Publisher("/detection/objects",ObjectPosition,queue_size=20)

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
    
    def _publish_object(self,detection,idx):
        print(f"T[{detection.clsID[idx]}]")
        self.__trash_pub.publish()

    def _publish_trash(self,detection,idx):
        print(f"T[{detection.clsID[idx]}]")
        self.__obj_pub.publish()

    def __publish(self):
        imgID = self.__take_snapshot(add_buffer=True).imgID
        object_detection = self.__detect_object(imgID=imgID).detection
        trash_detection = self.__detect_trash(imgID=imgID).detection

        for idx,_ in enumerate(object_detection.clsID):
            self.__publish_object(object_detection,idx)
        for idx,_ in enumerate(trash_detection.clsID):
            self.__publish_trash(trash_detection,idx)

    def __transform_point(self):
        pass

    def loop(self):
        while not rospy.is_shutdown():
            self.__publish()
            self.__rate.sleep()

def main():
    rospy.init_node("object_publisher")
    #object_pub = ObjectPublisher()
    object_pub = ObjectPublisherProxy()
    object_pub.loop()

if __name__ == "__main__":
    main()