import cv2
import numpy as np

import networkx as nx
import time,math

import matplotlib.pyplot as plt
from sys import argv as args

class PGMQuadtree:
    def __init__(self,pgm,bounds,depth=0,max_depth=0,filter=lambda x: np.min(x) <= 50):
        self.__pgm = pgm
        self.__bounds = bounds
        
        self.__subtrees = []
        
        left,top,width,height = bounds
        im_height, im_width = pgm.shape
        subim = pgm[max(0,top):min(top+height,im_height),max(0,left):min(left+width,im_width)]
        try:
            self.__occupied = filter(subim)
        except:
            self.__occupied = True

        self.__split(depth,max_depth,filter)

    @property
    def bounds(self):
        return self.__bounds
    @property
    def pos(self): # describes the center position of the cell/node
        return (self.X+self.WIDTH//2,self.Y+self.HEIGHT//2)
    @property
    def origin(self): # describes the origin of the cell -> top-left of the bounding box
        return (self.X,self.Y)

    @property
    def occupied(self):
        return self.__occupied

    def __split(self,depth,max_depth,filter):
        if depth > max_depth:
            return
        
        left,top,width,height = self.__bounds
        if self.__occupied and depth < max_depth:
            nheight = height//2
            nwidth = width//2
            new_boundaries = [  (left,top,                  nwidth,nheight), # NW
                                (left+nwidth,top,           nwidth,nheight), # NE
                                (left,top+nheight,          nwidth,nheight), # SW
                                (left+nwidth,top+nheight,   nwidth,nheight)] # SE
            for bound in new_boundaries:
                subtree = PGMQuadtree(pgm=self.__pgm,bounds=bound,depth=depth+1,max_depth=max_depth,filter=filter)
                self.__subtrees.append(subtree)
    
    def inbounds(self,pos):
        px,py = pos
        x,y,w,h = self.__bounds
        return x <= px < x+w and y <= py < y+h
    
    def search(self,pos):
        if len(self.__subtrees) == 0:
            return self

        for subtree in self.__subtrees:
            if subtree.inbounds(pos):
                return subtree.search(pos)
            
        raise ValueError(f"Pos {pos} not in bounds of the tree.")

    def print(self,msg=""):
        if len(self.__subtrees) > 0:
            for subtree in self.__subtrees:
                subtree.print(msg=msg+" - ")
        else:
            print(msg,self.__bounds)

    @property
    def X(self):
        return self.__bounds[0]
    @property
    def Y(self):
        return self.__bounds[1]
    @property
    def WIDTH(self):
        return self.__bounds[2]
    @property
    def HEIGHT(self):
        return self.__bounds[3]
    
    @property
    def N(self):
        return (self.X, self.Y-1)
    @property
    def E(self):
        return (self.X+self.WIDTH, self.Y)
    @property
    def S(self):
        return (self.X, self.Y+self.HEIGHT)
    @property
    def W(self):
        return (self.X-1, self.Y)
    @property
    def NE(self):
        return (self.X+self.WIDTH, self.Y-1)
    @property
    def SE(self):
        return (self.X+self.WIDTH, self.Y+self.HEIGHT)
    @property
    def SW(self):
        return (self.X-1, self.Y+self.HEIGHT)
    @property
    def NW(self):
        return (self.X-1, self.Y-1)
    @property
    def SURROUNDING(self):
        return [self.N,self.NE,self.E,self.SE,self.S,self.SW,self.W,self.NW]

def mark_bounds(im,bounds):
    im_width,im_height = im.shape
    for bound in bounds:
        left,top,width,height = bound

        im_height, im_width = im.shape
        if left >= im_width or top >= im_height:
            return

        im[top,left:min(left+width,im_width)] = 100
        im[top:min(top+height,im_height),left] = 100
        im[top:min(top+height,im_height),min(left+width,im_width)-1] = 100
        im[min(top+height,im_height)-1,left:min(left+width,im_width)] = 100

def display(im,bounds,duration_ms=0):
    mark_bounds(im,bounds)
    cv2.imshow("Img",im)
    cv2.waitKey(duration_ms)

def show_graph(G):
        nodes_positions = nx.get_node_attributes(G, 'pos')
        node_weights = nx.get_node_attributes(G,'weight')
        nx.draw(G, pos=nodes_positions, with_labels=True, node_size=70, node_color='skyblue', font_size=8, font_color='black', font_weight='bold', font_family='sans-serif', edge_color='black')
        nx.draw_networkx_labels(G,pos=nodes_positions,labels=node_weights)
        plt.show()

def astar(G,start,goal):
    def cost(u,v):
        if G.nodes[u]["occupied"] or G.nodes[v]["occupied"]:
            return math.inf
        
        (ux,uy) = G.nodes[u]["pos"] #return G.nodes[u]["weight"] + G.nodes[v]["weight"]
        (vx,vy) = G.nodes[v]["pos"]
        return math.sqrt((ux-vx)**2+(uy-vy)**2)
    
    try:
        shortest_path = nx.astar_path(G,source=start,target=goal,heuristic=cost)
    except nx.exception.NodeNotFound:
        print(f"No path from {start} to {goal} found.")
        shortest_path = []

    return shortest_path

def calcSize(WIDTH,HEIGHT):
    MINOR = min(WIDTH,HEIGHT)

    n = 1
    i = MINOR/n
    while i >= 1:
        n *= 2
        i = MINOR/n
    return n

def build_tree(im,resolution=7):
    HEIGHT,WIDTH = im.shape
    SIZE = calcSize(WIDTH,HEIGHT)
    return PGMQuadtree(pgm=im,bounds=(0,0,SIZE,SIZE),max_depth=resolution)

def build_graph(quadtree):
    HEIGHT = quadtree.HEIGHT
    G = nx.Graph()
    queue = []
    pos = (0,0)
    visited = set()
    queue.append(quadtree.search(pos))
    while len(queue) > 0:
        tree = queue.pop()
        if tree.bounds in visited:
            continue
        visited.add(tree.bounds)
        G.add_node(tree.bounds,origin=(tree.X,HEIGHT-tree.Y),pos=tree.pos, occupied=tree.occupied)

        for npos in tree.SURROUNDING:
            if quadtree.inbounds(npos):
                subtree = quadtree.search(npos)
                if subtree not in visited:
                    queue.append(subtree)

                    if not tree.occupied and not subtree.occupied:
                        G.add_edge(tree.bounds,subtree.bounds)
    return G

def draw_path(im,path):
    imcp = im.copy()
    mark_bounds(imcp,path)
    return imcp

def main():
    im = cv2.imread("./maps/map.pgm",cv2.IMREAD_GRAYSCALE)

    resolution = 7
    quadtree = build_tree(im,resolution)

    G = build_graph(quadtree)

    start = quadtree.search((0,0)).bounds
    
    import random
    HEIGHT,WIDTH = im.shape
    goals = [quadtree.search((random.randrange(WIDTH),random.randrange(HEIGHT))).bounds for _ in range(100)]
    for goal in goals:
        start = quadtree.search((random.randrange(WIDTH),random.randrange(HEIGHT))).bounds
        try:
            path = astar(G,start,goal)
        except nx.NetworkXNoPath:
            path = [start,goal]

        imcp = draw_path(im,path)
        cv2.imshow("Path",imcp)
        cv2.waitKey(0)

if __name__ == "__main__":
    main()