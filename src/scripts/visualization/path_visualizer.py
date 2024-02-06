import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Header

def main():
    rospy.init_node("path_visualizer")

    pub = rospy.Publisher("/simple_path",Path,queue_size=1)
    rate = rospy.Rate(1)

    running = True
    while running:
        try:
            header = Header(frame_id="base_footprint",stamp=rospy.Time.now())
            path = Path(header=header)
            path.poses = [PoseStamped(header=header) for _ in range(10)]

            l = 0
            for p in path.poses:
                p.pose.orientation.w = 1
                p.pose.position.x = l
                p.pose.position.y = l**2
                l += 0.25

            pub.publish(path)
            rate.sleep()
        except Exception as e:
            print(e)
            running = False

if __name__ == "__main__":
    main()