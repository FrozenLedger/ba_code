import cv2, rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

def mark_bounds(im,bounds,color=100):
    for bound in bounds:
        left,top,width,height = bound

        im[top,left:left+width] = color
        im[top:top+height,left] = color
        im[top:top+height,left+width-1] = color
        im[top+height-1,left:left+width] = color

def split(im,boundaries,depth,bounds,max_depth=1,filter=lambda x: np.min(x) <= 50):
    if depth > max_depth:
        return
        
    left,top,width,height = boundaries

    HEIGHT = im.shape[0]
    WIDTH = im.shape[1]

    h = min(HEIGHT,top+height)
    w = min(WIDTH,left+width)

    subim = im[top:h,left:w]
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

def display(im,bounds,duration_ms=0):
    mark_bounds(im,bounds)
    cv2.imshow("Img",im)
    cv2.waitKey(duration_ms)

def expand_ocg(ocg: OccupancyGrid, size=20):
    H = ocg.info.height
    W = ocg.info.width
    ocgimage = np.array(ocg.data,dtype=np.int8)
    
    ocgimage = ocgimage.reshape((H,W))
    ocgimage = np.pad(ocgimage,((size,size),(size,size)),mode="constant",constant_values=-1)
    
    H = ocgimage.shape[0]
    W = ocgimage.shape[1]

    ocg.data = ocgimage.reshape(H*W)
    ocg.info.width = W
    ocg.info.height = H
    ocg.info.origin.position.x = ocg.info.origin.position.x - size*ocg.info.resolution
    ocg.info.origin.position.y = ocg.info.origin.position.y - size*ocg.info.resolution
    return ocg

if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher("/map_expanded",OccupancyGrid,queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        ocg = rospy.wait_for_message("/map",OccupancyGrid)
        ocg = expand_ocg(ocg,size=100)

        pub.publish(ocg)
        rate.sleep()