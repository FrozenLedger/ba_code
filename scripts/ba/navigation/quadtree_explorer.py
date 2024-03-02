import cv2
import numpy as np
import rospy, math

from ba.utilities.data import Data

from slam_toolbox_msgs.srv import SaveMap,SaveMapRequest
from std_msgs.msg import String
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Point,PointStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger,TriggerResponse

from ba.navigation.robot import RobotMover
from ba.utilities.imageprocessing import mark_bounds, split
#from ba.utilities.imageprocessing import display

from cv_bridge import CvBridge

class TileMap:
    def __init__(self,im,boundaries):
        self.__im = im
        self.__boundaries = boundaries
        self.__sorted_scores = []

        count = 0
        for idx,boundarie in enumerate(self.__boundaries):
            top,left,width,height = boundarie

            SCORE = width*height

            if SCORE < 32*32:
                continue
            
            self.__sorted_scores.append((idx,SCORE))
        self.__sorted_scores.sort(key=lambda x: x[1],reverse=True)
        print(count)

    @property
    def boundaries(self):
        return self.__boundaries
    
    @property
    def sorted_scores(self):
        return self.__sorted_scores

    def save_im(self,impath="/tmp/tilemap.pgm"):
        try:
            imcp = self.__im.copy()
            mark_bounds(imcp,self.__boundaries)
            cv2.imwrite(impath,imcp)
        except Exception as e:
            print(e)

class TilemapExplorer:
    def __init__(self,pgmpath="/tmp/exploremap",filterfunction=lambda x: np.min(x) < 50):
        self.__map_saver = rospy.ServiceProxy("/slam_toolbox/save_map",SaveMap)
        self.__pgmpath = pgmpath
        self.__mover = RobotMover()
        self.__mapmetadata = Data(MapMetaData())
        self.__map_meta_listener = rospy.Subscriber("/map_metadata",MapMetaData,self.__mapmetadata.setData)
        
        self.__last_pose = self.__mover.pose
        self.__delay_timer = rospy.Duration(5) # refreshes map after 30s
        self.__min_move_distance = 0.25 # attempt counts as failed if robot doesn't move more than min_move_distance
        self.__min_goal_distance = 0.25 # goal reached if within this distance

        self.__filter = filterfunction

        self.__cv_bridge = CvBridge()
        self.__tilemap_publisher = rospy.Publisher("/tilemap/image",Image,queue_size=1)

        self.__target = 0
        self.__timestamp = 0
        self.__progress = 0
        self.__state = self.__refresh_map
        self.__max_process_delay = rospy.Duration(45)

        rospy.wait_for_service("/slam_toolbox/save_map")
        self.__update_pgm()

        self.__marked_regions = {}
        self.__explored_regions = {}

    def __save_tiled(self,_):
        try:
            self.__tilemap.save_im()
        except Exception as e:
            print(e)
            return TriggerResponse(success=False)
        return TriggerResponse(success=True)

    def __update_pgm(self):
        print(f"\tSave map from /map topic locally: {self.__pgmpath}")
        self.__map_saver(SaveMapRequest(name=String(self.__pgmpath)))    

        self.__im = cv2.imread(f"{self.__pgmpath}.pgm",cv2.IMREAD_GRAYSCALE)
        self.__tilemap = create_tilemap(self.__im,self.__filter)
        #self.__last_update_ts = rospy.Time.now()

        imcp = self.__im.copy()
        mark_bounds(imcp,self.__tilemap.boundaries)
        rosimg = self.__cv_bridge.cv2_to_imgmsg(imcp)
        self.__tilemap_publisher.publish(rosimg)
        self.__marked_regions = {}

    def explore(self):
        try:
            self.__state = self.__state()
        except Exception as e:
            print(e)

    # Explore behaviour FSM
    def __refresh_map(self):
        print("### Refresh map ###")
        self.__update_pgm()
        return self.__set_target

    def __set_target(self):
        print("### Set new target ###")
        for idx in range(len(self.__tilemap.sorted_scores)):
            boundsindex = self.__tilemap.sorted_scores[idx][0]
            max_unexplored = self.__tilemap.boundaries[boundsindex]
            if max_unexplored in self.__marked_regions or self.__target == max_unexplored or self.__target in self.__explored_regions:
                max_unexplored = 0
                continue
            break

        if not max_unexplored:
            print(f"\tNo more unexplored regions. Force reset.")
            self.__explored_regions = {}
            self.__marked_regions = {}
            return self.__refresh_map
        
        print(f"\tSetting new exploration goal: {max_unexplored}")
        self.__target = max_unexplored
        return self.__move_to_target

    def __move_to_target(self):
        print("### Move to target ###")
        HEIGHT = self.__im.shape[0]

        minx,miny,width,height = self.__target #max_unexplored

        # calculate pixel coordinates
        px = minx + width//2
        py = miny + height//2
        res = self.__mapmetadata.data.resolution

        print(f"\tTile Pixelcoordinates: {(px,py)}")

        # transform to map coordinates
        x = px*res + self.__mapmetadata.data.origin.position.x
        y = (HEIGHT - py)*res + self.__mapmetadata.data.origin.position.y

        print(f"\tTile Coordinates: {(x,y)}")

        pnt = PointStamped()
        pnt.point = Point(x=x,y=y)
        pnt.header.frame_id = "map"
        pnt.header.stamp = rospy.Time.now()

        print(f"\tTry moving to unexplored tile:\n\t\t{pnt.point}")
        self.__last_pose = self.__mover.pose
        self.__target_pnt = pnt.point
        self.__mover.move_to_point(pnt)
        self.__start_time = rospy.Time.now()

        return self.__delay

    def __delay(self):
        if not self.__timestamp:
            print("### Delay ###")
            self.__timestamp = rospy.Time.now()
        elif rospy.Time.now() - self.__timestamp > self.__delay_timer:
            self.__timestamp = 0
            return self.__check_progress
        return self.__delay

    def __check_progress(self):
        print("### Check Progress ###")
        pose = self.__mover.pose
        dx = self.__last_pose.pose.position.x - pose.pose.position.x
        dy = self.__last_pose.pose.position.y - pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        self.__last_pose = pose
        if dist < self.__min_move_distance:
            print(f"\tFailed to move by more than {self.__min_move_distance}m.")
            print(f"\t\tMark region: {self.__target}")
            self.__marked_regions[self.__target] = 1
            return self.__set_target        

        if rospy.Time.now() - self.__start_time > self.__max_process_delay:
            print("\tMax exploration time exceeded.")
            return self.__refresh_map
        return self.__check_goal_distance
    
    def __check_goal_distance(self):
        print("### Check goal distance ###")
        pose = self.__mover.pose
        goal = self.__target_pnt
        dx = goal.x - pose.pose.position.x
        dy = goal.y - pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.__min_goal_distance:
            print("\tGoal reached.")
            print("\tAdding region to explored regions.")
            self.__explored_regions[self.__target] = 1
            return self.__refresh_map
        print("\tGoal not reached yet.")
        return self.__delay

def create_tilemap(im,filterfunction=lambda x: np.min(x) < 50):
    def MaxFilter(image,kernel):
        kernel = np.ones((kernel,kernel),np.uint8)
        return cv2.erode(image,kernel)

    im = MaxFilter(im,kernel=5)

    HEIGHT = im.shape[0]
    WIDTH = im.shape[1]

    boundaries = split(im,(0,0,WIDTH,HEIGHT),depth=0,max_depth=4,filter=filterfunction)# ,filter=lambda x: np.min(x) <= 210)

    print(f"\tBounds|Sub-Images: {len(boundaries)}")
    tilemap = TileMap(im,boundaries)
    return tilemap

def test():
    im = cv2.imread("map.pgm",cv2.IMREAD_GRAYSCALE)
    im = im[0:1024,0:1024]

    def MaxFilter(image,kernel):
        kernel = np.ones((kernel,kernel),np.uint8)
        return cv2.erode(image,kernel)

    im = MaxFilter(im,kernel=5)

    HEIGHT,WIDTH = im.shape

    boundaries = split(im,(0,0,WIDTH,HEIGHT),depth=0,max_depth=7)# ,filter=lambda x: np.min(x) <= 210)

    print(f"Bounds|Sub-Images: {len(boundaries)}")
    #display(im,boundaries)
   
    tilemap = TileMap(im,boundaries)
    
    max_unexplored = tilemap.max_unexplored
    print(max_unexplored)
    mark_bounds(im,[max_unexplored],150)
    cv2.imshow("MaxUnexplored",im)
    cv2.waitKey(0)

    print("Test done.")

def main():
    rospy.init_node("splitmap_explorer")

    explorer = TilemapExplorer(pgmpath="/home/workspace1/tilemap")

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        explorer.explore()
        rate.sleep()

if __name__ == "__main__":
    test()
    main()