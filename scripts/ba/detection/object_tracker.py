import rospy,tf

from ba_code.srv import Detect

from geometry_msgs.msg import PointStamped,Point
from visualization_msgs.msg import MarkerArray,Marker

def publish_marker(pnt_arr:PointStamped):
    pub = rospy.Publisher("/visualization_marker_array",MarkerArray,queue_size=10)

    arr = []
    mid = 0
    for pnt in pnt_arr:
        marker = Marker()
        #marker.header.frame_id = "base_link"
        #marker.header.stamp = rospy.Time.now()
        marker.header = pnt.header

        #marker.ns = "basic_shape"
        marker.type = 2 #Marker.CUBE
        marker.id = mid
        mid += 1

        marker.pose.position = pnt.point
        #marker.pose.position.x = 1

        print(pnt.point)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1
        marker.color.a = 1

        marker.pose.orientation.w = 1
        arr.append(marker)

    marker_arr = MarkerArray()
    marker_arr.markers = arr
    #marker.lifetime = rospy.Duration(5)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(marker_arr)
        rate.sleep()#rospy.rostime.wallsleep(1)

def calcPoint(detection,idx):
    distance = detection.distance
    dist = max(distance[idx].median,distance[idx].center)
    if dist <= 100:
        print("[Warning] Distance to low. Ignore prediction.")
    print(detection.clsID[idx],detection.confidence[idx],distance[idx].median)

    return Point(x = dist/1000)

def main():
    rospy.init_node("object_tracker")

    tf_listener = tf.TransformListener()

    service = "/yolov5/detect"
    print(f"Wait for service: {service}")
    rospy.wait_for_service(service)
    print("Ready.")

    server = rospy.ServiceProxy(service,Detect)
    resp = server()

    target_frame = "base_link"
    detection = resp.detection

    pnts = []
    for idx in range(len(detection.clsID)):
        pnt = PointStamped(header=resp.header)
        pnt.point = calcPoint(detection,idx)
        base_pnt = tf_listener.transformPoint(target_frame,pnt)
        print(f"Transform: {base_pnt}")
        pnts.append(base_pnt)
    publish_marker(pnts)

if __name__ == "__main__":
    main()