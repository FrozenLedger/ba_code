import rospy,tf

from geometry_msgs.msg import PoseStamped,PointStamped
from std_msgs.msg import Header

from ba.utilities.data import Data

import ba_code.msg as bamsg
import ba_code.srv as basrv

class ApproachPointServer:
    def __init__(self,base="base_link"):
        self.__server = rospy.Service()
        self.__base = base
        self.__enabled = True
        self.__idle_rate = rospy.Rate(1)
        self.__rate = rospy.Rate(10)
        self.__tf_listener = tf.TransformListener()
        self.__goal = PoseStamped()

        rospy.Duration(0.2).sleep()

    def __update(self):
        vec = getOV(self.__tf_listener,self.__world,self.__goal)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                while self.__enabled and not rospy.is_shutdown():
                    self.__update()
                    self.__rate.sleep()
            except Exception as e:
                print(e)
                break
            self.__idle_rate.sleep()

class DirectionServer:
    def __init__(self):
        self.__tf_listener = tf.TransformListener()
        rospy.Rate(1).sleep()

        self.__server = rospy.Service("/direction/get_dir",basrv.GetDirection,self.__get_direction)

    def __get_direction(self,req):
        origin = req.origin.data
        target = req.target.data
        try:
            result = getOV(self.__tf_listener,goal=target,base=origin)
            print(f"Processed request: origin: <{origin}>\ttarget:<{target}>")
            return result
        except Exception as e:
            print(e)

def main():
    rospy.init_node("approach_point")

    #server = ApproachPointServer(base="base_footprint")
    server = DirectionServer()
    rospy.loginfo("[Ready] Approach_point action server available.")
    #server.loop()
    rospy.spin()

def getRobotPose(tf_listener,world,base):
    t0 = rospy.Time.now()
    header = Header(frame_id=base,stamp=t0)
    pose = PoseStamped(header = header)
    tf_listener.waitForTransform(world,base,t0,rospy.Duration(0.02))
    world_pose = tf_listener.transformPose(world,pose)
    return world_pose

def getOV(tf_listener,goal,base):
    t0 = rospy.Time.now()
    header = Header(frame_id=goal,stamp=t0)
    pnt = PointStamped(header=header)
    tf_listener.waitForTransform(goal,base,t0,rospy.Duration(5))
    tpnt = tf_listener.transformPoint(base,pnt)
    return tpnt.point

if __name__ == "__main__":
    main()