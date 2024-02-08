import rospy,cv2
import numpy as np
import tf, time

from splitmap_searchtree import build_tree,build_graph,astar,mark_bounds,show_graph
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

class GridMapper:
    def __init__(self):
        self.__mapmsg = OccupancyGrid()
        self.__sub = rospy.Subscriber("/map",OccupancyGrid,self.set_map)
        self.__tbc = tf.TransformBroadcaster()
        self.__rate = rospy.Rate(10)
        self.__pgm = None
        self.__G = None
        self.__qtree = None
        self.__transform_listener = tf.TransformListener()

    @property
    def pgm(self):
        if type(self.__pgm) == type(None):
            self.__build_pgm()
        return self.__pgm
    
    @property
    def G(self):
        if self.__G == None:
            self.__build_graph()
        return self.__G
    
    @property
    def qtree(self):
        if self.__qtree == None:
            self.__build_qtree()
        return self.__qtree
    
    def __build_pgm(self):
        width = self.__mapmsg.info.width
        height = self.__mapmsg.info.height
        self.__pgm = np.array(self.__mapmsg.data)
        unoccupied = self.__pgm == 0
        occupied = self.__pgm > 0

        self.__pgm[occupied] = 0
        self.__pgm[unoccupied] = 254
        self.__pgm[self.__pgm == -1]= 205
            
        self.__pgm = np.reshape(self.__pgm,(height,width))
        self.__pgm = self.__pgm.astype(np.uint8)

    def __build_graph(self):
        self.__G = build_graph(self.qtree)

    def __build_qtree(self):
        self.__qtree = build_tree(self.pgm,resolution=7)

    def set_map(self,msg):
        self.__mapmsg = msg
        self.__pgm = None
        self.__G = None
        self.__qtree = None

    def save_map(self,path):
        t = rospy.Time.now()

        base_pnt = PointStamped()
        base_pnt.header.frame_id = "base_footprint"
        base_pnt.header.stamp = t - rospy.Duration(0.2)

        im = cv2.flip(self.pgm,0)
        cv2.imwrite(path,im)

    def broadcast_tf(self):
        while not rospy.is_shutdown():
            try:
                t = self.__mapmsg.info.origin.position
                r = self.__mapmsg.info.origin.orientation
                self.__tbc.sendTransform((t.x,t.y,t.z),(r.x,r.y,r.z,r.w),rospy.Time.now(),"gridmap_origin","map")
            except:
                rospy.loginfo("Broadcast error.")
            self.__rate.sleep()

    def __del__(self):
        print("Unregister /map subscriber")
        self.__sub.unregister()

def main():
    rospy.init_node("test_mapper")

    gridmapper = GridMapper()
    gridmapper.broadcast_tf()
    
    rospy.spin()

if __name__ == "__main__":
    main()