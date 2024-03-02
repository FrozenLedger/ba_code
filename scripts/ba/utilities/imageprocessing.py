import cv2

def mark_bounds(im,bounds,color=100):
    for bound in bounds:
        left,top,width,height = bound

        im[top,left:left+width] = color
        im[top:top+height,left] = color
        im[top:top+height,left+width-1] = color
        im[top+height-1,left:left+width] = color

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

def display(im,bounds,duration_ms=0):
    mark_bounds(im,bounds)
    cv2.imshow("Img",im)
    cv2.waitKey(duration_ms)