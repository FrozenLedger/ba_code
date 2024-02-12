import rospy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from ba_code.msg import ObjectPosition

def main():
    rospy.init_node("object_spawner")
    pub = rospy.Publisher("/detection/trash",ObjectPosition,queue_size=10)
    rate = rospy.Rate(10)

    def publish_trash(pnt:PointStamped):
        msg = ObjectPosition()
        msg.point = pnt
        msg.clsID = 42
        msg.confidence = 1.0
        msg.note = String("Test object")

        pub.publish(msg)
        rate.sleep()

    sub = rospy.Subscriber("/clicked_point",PointStamped,publish_trash)
    rospy.spin()

if __name__ == "__main__":
    main()