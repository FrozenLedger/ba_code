from splitmap_searchtree import build_tree,build_graph,astar,mark_bounds,show_graph
import rospy
import cv2

import numpy as np

from navigation.gridmapper import GridMapper

class GlobalPlanner:
    def __init__(self):
        #self.__mapmsg = None
        #self.__pgm = None
        self.__gridmapper = GridMapper()

    @property
    def pgm(self):
        return self.__pgm

    def setMap(self,msg):
        self.__mapmsg = msg

        #rospy.loginfo(msg.header)
        #rospy.loginfo(msg.info)
        #rospy.loginfo(f"Reveived new map.")

        width = msg.info.width
        height = msg.info.height
        self.__pgm = np.array(self.__mapmsg.data)
        
        unoccupied = self.__pgm == 0
        occupied = self.__pgm > 0

        self.__pgm[occupied] = 0
        self.__pgm[unoccupied] = 254
        self.__pgm[self.__pgm == -1]= 205
        
        self.__pgm = np.reshape(self.__pgm,(height,width))
        self.__pgm = self.__pgm.astype(np.uint8)
        self.__pgm = cv2.flip(self.__pgm,0)

        self.__qtree = build_tree(self.__pgm,resolution=7)
        self.__G = build_graph(self.__qtree)

        #_,_,WIDTH,HEIGHT = self.__qtree.bounds
        #start = self.__qtree.search((0,0)).bounds
        #goal = self.__qtree.search((300,300)).bounds

        #try:
        #    path = astar(self.__G,start,goal)
        #except:
        #    path = [start]
        #mark_bounds(self.__pgm,path)
        #rospy.loginfo(np.unique(self.__pgm))
        #rospy.loginfo(self.__pgm.dtype)
        #cv2.imwrite("map_save.pgm",self.__pgm)

if __name__ == "__main__":
    rospy.init_node("global_planner")

    global_planner = GlobalPlanner()

    #sub = rospy.Subscriber("/map",OccupancyGrid,global_planner.setMap)

    rospy.spin()
    #rate = rospy.Rate(10)
    #while global_planner.pgm is None:
    #    rate.sleep()

    #pgm = global_planner.pgm

    