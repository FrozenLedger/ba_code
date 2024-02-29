import cv2
import numpy as np
import rospy, math

from ba.utilities.data import Data

from slam_toolbox_msgs.srv import SaveMap,SaveMapRequest
from std_msgs.msg import String
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Point,PointStamped
from sensor_msgs.msg import Image

from ba.navigation.robot import RobotMover

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

            if SCORE < 64*64:
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
    def __init__(self,pgmpath="/tmp/exploremap"):
        self.__map_saver = rospy.ServiceProxy("/slam_toolbox/save_map",SaveMap)
        self.__pgmpath = pgmpath
        self.__mover = RobotMover()
        self.__mapmetadata = Data(MapMetaData())
        self.__map_meta_listener = rospy.Subscriber("/map_metadata",MapMetaData,self.__mapmetadata.setData)
        self.__fails = 0
        self.__last_pose = self.__mover.pose
        self.__update_interval = 30 # refreshes map after 30s
        self.__min_move_distance = 0.25 # attempt counts as failed if robot doesn't move more than min_move_distance

        self.__cv_bridge = CvBridge()
        self.__tilemap_publisher = rospy.Publisher("/tilemap/image",Image,queue_size=1)

        rospy.wait_for_service("/slam_toolbox/save_map")
        self.__update_pgm()

    def save_tiled(self):
        try:
            self.__tilemap.save_im()
        except Exception as e:
            print(e)

    def __update_pgm(self):
        print(f"Save map from /map topic locally: {self.__pgmpath}")
        self.__map_saver(SaveMapRequest(name=String(self.__pgmpath)))    

        self.__im = cv2.imread(f"{self.__pgmpath}.pgm",cv2.IMREAD_GRAYSCALE)
        self.__tilemap = create_tilemap(self.__im)
        self.__last_update_ts = rospy.Time.now()

        imcp = self.__im.copy()
        mark_bounds(imcp,self.__tilemap.boundaries)
        rosimg = self.__cv_bridge.cv2_to_imgmsg(imcp)
        self.__tilemap_publisher.publish(rosimg)

    def explore(self):
        try:
            N = len(self.__tilemap.sorted_scores)
            dT = rospy.Time.now() - self.__last_update_ts
            if self.__fails >= N:
                self.__update_pgm()
                self.__fails = 0
            elif dT > rospy.Duration(self.__update_interval):
                self.__update_pgm()
            else:
                print(f"dT since last update: {dT.to_sec()}s")
            
            next_idx = self.__tilemap.sorted_scores[self.__fails][0]
            max_unexplored = self.__tilemap.boundaries[next_idx]
            print(f"Explore region: {max_unexplored}\tTotal Regions: {N}")
            
            HEIGHT = self.__im.shape[0]

            minx,miny,width,height = max_unexplored
            px = minx + width//2
            py = miny + height//2
            res = self.__mapmetadata.data.resolution

            print(f"Tile Pixelcoordinates: {(px,py)}")

            # transform to map frame
            x = px*res + self.__mapmetadata.data.origin.position.x
            y = (HEIGHT - py)*res + self.__mapmetadata.data.origin.position.y

            print(f"Tile Coordinates: {(x,y)}")

            pnt = PointStamped()
            pnt.point = Point(x=x,y=y)
            pnt.header.frame_id = "map"
            pnt.header.stamp = rospy.Time.now()

            print(f"Try moving to unexplored tile: {pnt.point}...")
            self.__mover.move_to_point(pnt)

            pose = self.__mover.pose
            dx = self.__last_pose.pose.position.x - pose.pose.position.x
            dy = self.__last_pose.pose.position.y - pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < self.__min_move_distance:
                self.__fails += 1
                print(f"Failed to move more than 1m... Fails:{self.__fails}")
            self.__last_pose = pose
        except Exception as e:
            print(e)
        
def split(im,boundaries,depth,max_depth=1,bounds=[],filter=lambda x: np.min(x) <= 50):
    if depth > max_depth:
        return
        
    left,top,width,height = boundaries

    subim = im[top:top+height,left:left+width]
    if filter(subim) and depth < max_depth:
        nheight = height//2
        nwidth = width//2
        new_boundaries = [  (left,top,                  nwidth,nheight), # NW
                            (left+nwidth,top,           nwidth,nheight), # NE
                            (left,top+nheight,          nwidth,nheight), # SW
                            (left+nwidth,top+nheight,   nwidth,nheight)] # SE
            
        #print("  "*depth,f"Splitting old: {boundaries}")
        for bound in new_boundaries:
            #print("  -"*depth,f"split into: {bound}")
            split(im,bound,depth=depth+1,max_depth=max_depth,bounds=bounds,filter=filter)
    else:
        bounds.append(boundaries)
    return bounds

def mark_bounds(im,bounds,color=100):
    for bound in bounds:
        left,top,width,height = bound

        im[top,left:left+width] = color
        im[top:top+height,left] = color
        im[top:top+height,left+width-1] = color
        im[top+height-1,left:left+width] = color

def display(im,bounds,duration_ms=0):
    mark_bounds(im,bounds)
    cv2.imshow("Img",im)
    cv2.waitKey(duration_ms)

def create_tilemap(im):
    #im = im[0:1024,0:1024]

    def MaxFilter(image,kernel):
        kernel = np.ones((kernel,kernel),np.uint8)
        return cv2.erode(image,kernel)

    im = MaxFilter(im,kernel=5)

    HEIGHT = im.shape[0]
    WIDTH = im.shape[1]

    boundaries = split(im,(0,0,WIDTH,HEIGHT),depth=0,max_depth=5,filter=lambda x: np.min(x) < 220)# ,filter=lambda x: np.min(x) <= 210)

    print(f"Bounds|Sub-Images: {len(boundaries)}")
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
    #test()
    main()