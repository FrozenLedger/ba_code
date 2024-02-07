import rospy

from geometry_msgs.msg import Twist

class Data:
    def __init__(self,data):
        self.__data = data

    @property
    def data(self):
        return self.__data
    
    def setData(self,data):
        self.__data = data

def main():
    rospy.init_node("navigator")

    steer_vel = Data(Twist())
    nav_vel = Data(Twist())

    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    sub1 = rospy.Subscriber("cmd_steer",Twist,steer_vel.setData)
    sub2 = rospy.Subscriber("cmd_nav",Twist,steer_vel.setData)

    rate = rospy.Rate(10)
    running = True
    while running:
        try:
            twist = steer_vel.data
            twist.linear.x += nav_vel.data.linear.x
            twist.angular.z += nav_vel.data.angular.z
            pub.publish(twist)
            rate.sleep()
        except Exception as e:
            print(e)
            running = False

if __name__ == "__main__":
    main()