import rospy
import tf

from geometry_msgs.msg import PoseStamped

from ba.utilities.singletons import get_transform_listener

class PosePublisher:
    def __init__(self,rate: float = 10):
        self._rate: rospy.Rate = rospy.Rate(rate)
        self._publisher: rospy.Publisher = rospy.Publisher("/robot/pose", PoseStamped, queue_size=10)

        print("PosePublisher initialized.")

    def update(self):
        base_frame = "base_link"
        target_frame = "map"

        base_pose: PoseStamped = PoseStamped()
        base_pose.header.frame_id = base_frame

        tf_listener: tf.TransformListener = get_transform_listener()
        
        pose: PoseStamped = PoseStamped()
        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            base_pose.header.stamp = t0
            try:
                tf_listener.waitForTransform(base_frame,target_frame,t0,rospy.Duration(0.25))
                pose = tf_listener.transformPose(target_frame,base_pose)
        
                self._publisher.publish(pose)
            except Exception as e:
                print(e)
            self._rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pose_publisher_node")

    try:
        pp = PosePublisher()
        pp.update()
    except rospy.exceptions.ROSInterruptException as e:
        print(e)