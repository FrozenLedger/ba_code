import rospy,math

from sensor_msgs.msg import LaserScan
import numpy as np

from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Header

class LaserScanData:
    def __init__(self):
        self.__msg = LaserScan()

    @property
    def data(self):
        return self.__msg
    
    def setData(self,msg):
        self.__msg = msg

@np.vectorize
def calcVectors(V):
    return (math.sin(V),math.cos(V))

def calcVelocity(scan,max_range=12):
    dist_arr = np.array(scan.data.ranges)

    if len(dist_arr) <= 0:
        return (1,0,0)
    
    min_dist = np.min(dist_arr)
    if min_dist == math.inf:
        min_dist = scan.data.range_max
    dist_arr[dist_arr > max_range] = max_range
    #dist_arr[dist_arr > min_dist + 0.25] = 0
    #dist_arr[::] = dist_arr/12
    #dist_arr[dist_arr > 2] = 2

    start_angle = scan.data.angle_min
    incr = scan.data.angle_increment
    N = len(dist_arr)
    angles = np.array([start_angle + i*incr for i in range(N)])

    #indizes = inFront(angles)
    #(X,Y) = calcVectors(angles[indizes])*dist_arr[indizes]
    (Y,X) = calcVectors(angles)*dist_arr
    x = np.sum(X)/len(X)
    y = np.sum(Y)/len(Y)
    
    return x,y,min_dist

def main():
    rospy.init_node("fuzzy")

    scan = LaserScanData()
    sub = rospy.Subscriber("/scan",LaserScan,scan.setData)
    fuzzy_vis_pub = rospy.Publisher("/fuzzy_visual",TwistStamped,queue_size=10)
    forward_vis_pub = rospy.Publisher("/forward_visual",TwistStamped,queue_size=10)

    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    
    rate = rospy.Rate(10)
    running = True
    while running:
        try:
            data = scan.data
            #info = f"min: {data.range_min}\tmax: {data.range_max}\tN: {len(data.ranges)}\tincr: {data.angle_increment}\n"
            header = Header(frame_id="base_footprint",stamp=rospy.Time.now())

            forward_vel = TwistStamped(header=header)
            forward_vel.twist.linear.x = 1
            forward_vel.twist.angular.z = 0
            fx = forward_vel.twist.linear.x
            fy = forward_vel.twist.angular.z
            beta = math.atan2(fy,fx)

            N = len(data.ranges)
            if N > 0:
                x,y,min_dist = calcVelocity(scan,max_range=3)
                alpha = math.atan2(y,x)
                
                print(round(alpha,3),round(beta,3))
                x_dir = math.cos(alpha)
                z_rot = math.sin(alpha)
                twist = Twist()
                twist.linear.x = x_dir
                twist.linear.y = z_rot
                fuzzy_vis = TwistStamped(header=header)
                fuzzy_vis.twist = twist
                fuzzy_vis_pub.publish(fuzzy_vis)
                forward_vis_pub.publish(forward_vel)

                vel = Twist()
                vel.linear.x = min(1,max(-1,forward_vel.twist.linear.x - fuzzy_vis.twist.linear.x))*0.5
                vel.angular.z = min(1,max(-1,forward_vel.twist.angular.z + fuzzy_vis.twist.linear.y))*0.5
                #vel_pub.publish(vel)

                rate.sleep()
        except Exception as e:
            print(e)
            running = False

    sub.unregister()

if __name__ == "__main__":
    main()