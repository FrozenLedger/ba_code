import rospy,tf

from tf.transformations import euler_from_quaternion,quaternion_from_euler

from geometry_msgs.msg import PoseStamped,Twist,Vector3
from std_msgs.msg import Header,String

import ba_code.srv as basrv

import math

def _unwrap(quaternion):
    return (quaternion.x,quaternion.y,quaternion.z,quaternion.w)

class ShortDistanceController:
    def __init__(self):
        self.__world = "map"
        self.__base = "base_footprint"
        self.__tf_listener = tf.TransformListener()
        self.__rate = rospy.Rate(10)
        self.__vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.__tolerance = math.radians(1)
        self.__rot_spd = 0.2
        rospy.Rate(1).sleep()

    @property
    def pose(self):
        t0 = rospy.Time.now()
        p0 = PoseStamped(header=Header(frame_id=self.__base,stamp=t0))
        self.__tf_listener.waitForTransform(self.__base,self.__world,t0,rospy.Duration(0.2))
        pose = self.__tf_listener.transformPose(self.__world,p0)
        return pose
    
    def __calc_rot(self,pose):
        orient = pose.pose.orientation
        rot = euler_from_quaternion(_unwrap(orient))
        return rot

    def rotate_by(self,angle:float):
        p0 = self.pose
        rz0 = self.__calc_rot(p0)[2]
        rz_goal = rz0 + angle

        print(f"Start: {rz0}\tGoal: {rz_goal}")
        while not rospy.is_shutdown():
            try:
                rz1 = self.__calc_rot(self.pose)[2]
                if rz_goal - self.__tolerance <= rz1 <= rz_goal + self.__tolerance:
                    break
                if rz_goal > rz1:
                    rot = self.__rot_spd
                else:
                    rot = -self.__rot_spd
                self.__vel_pub.publish(Twist(angular=Vector3(0, 0,rot) ))
            except Exception as e:
                print(e)
            self.__rate.sleep()

    def rotate_to(self,frame_id):
        get_dir_req = rospy.ServiceProxy("/direction/get_dir",basrv.GetDirection)
        msg = Twist()

        pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        origin = String("base_footprint")
        target = String(frame_id)

        def rot(msg):
            if alpha > 0.05:
                msg.angular.z = 0.1
            elif alpha < -0.05:
                msg.angular.z = -0.1
            else:
                msg.angular.z = 0

        def lin(mng):
            if dist > 1:
                spd = 0.2
                msg.linear.x = 0.2
            elif dist > 0.01:
                spd = 0.05
                msg.linear.x = 0.05
            else:
                spd = 0

            if  not (-math.pi/2 < alpha < math.pi/2):
                msg.linear.x = -spd
            else:
                msg.linear.x = spd

        def calc():
            pnt = get_dir_req(origin=origin,target=target).vector
            alpha = math.atan2(pnt.y,pnt.x)
            dist = math.sqrt(pnt.y**2 + pnt.x**2)
            return (dist,alpha)

        pnt = get_dir_req(origin=origin,target=target).vector
        alpha = math.atan2(pnt.y,pnt.x)
        dist = math.sqrt(pnt.y**2 + pnt.x**2)
        while not (-0.05 < alpha < 0.05) or (not dist < 0.1):
            print(f"Phase1 - angle: {alpha}\tdist: {dist}")

            rot(msg)
            lin(msg)

            pub.publish(msg)
            self.__rate.sleep()

            dist,alpha = calc()

        msg.angular.z = 0
        while not dist < 0.01:
            print(f"Phase2 - angle: {alpha}\tdist: {dist}")

            lin(msg)
            pub.publish(msg)
            dist,alpha = calc()
            
        print(alpha)
        print("Done.")

def main():
    rospy.init_node("short_distance_navigator")
    motor = ShortDistanceController()
    #motor.rotate_by(math.pi/2)
    motor.rotate_to("map")

if __name__ == "__main__":
    main()