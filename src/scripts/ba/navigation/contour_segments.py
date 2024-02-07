import rospy,math

from ba.utilities.data import Data
from geometry_msgs.msg import TwistStamped,Twist

def main():
    rospy.init_node("segment_follower")

    obst3 = Data(TwistStamped())
    obst4 = Data(TwistStamped())
    
    sub3 = rospy.Subscriber("/obstacle3",TwistStamped,obst3.setData)
    sub4 = rospy.Subscriber("/obstacle4",TwistStamped,obst4.setData)

    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    steer = Data(Twist())
    sub = rospy.Subscriber("/cmd_vel_steer",Twist,steer.setData)

    rate = rospy.Rate(10)
    running = True
    while running:
        try:
            x3 = obst3.data.twist.linear.x
            x4 = obst4.data.twist.linear.x
            y3 = obst3.data.twist.linear.y
            y4 = obst4.data.twist.linear.y
            d3 = math.sqrt(x3**2+y3**2)
            d4 = math.sqrt(x4**2+y4**2)
            alpha3 = math.atan2(y3,x3)
            alpha4 = math.atan2(y4,x4)

            move_back = -(d3 < 1)*(1-d3) - (d4 < 1)*(1-d4)

            msg = Twist()
            msg.linear.x = move_back + steer.data.linear.x

            pub.publish(msg)

            rate.sleep()
        except Exception as e:
            print(e)
            running = False
    
if __name__ == "__main__":
    main()