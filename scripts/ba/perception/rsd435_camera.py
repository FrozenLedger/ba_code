import cv2,time
import pyrealsense2 as rs
import numpy as np

from pathlib import Path

class DepthImage:
    def __init__(self,colorim,depthim,depth):
        self.__colorim = colorim
        self.__depthim = depthim
        self.__depth = depth

    @property
    def depthim(self):
        return self.__depthim
    @property
    def colorim(self):
        return self.__colorim
    @property
    def depth(self):
        return self.__depth

    def pixel_to_depth(self, px,py) -> int:
        """Returns the distance value of the pixelcoordinate (<x>,<y>) read from the depth data."""
        #return self.__alignment.get_depth_frame().get_distance(x,y)
        dist = self.__depth_data[py,px]
        print(f"Distance data at ({px,py} = {dist} mm")
        return dist #self.__aligned_depth_frames.get_distance(x,y)

class RealSenseD435:
    def __init__(self, width: int=640, height: int=480, format: rs.format=rs.format.z16, framerate: int=30, delay: float=5.0):
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__running = False

        try:
            self.__pipeline_profile = self.__config.resolve(rs.pipeline_wrapper(self.__pipeline))
        except:
            print("RuntimeError: No device connected.")
            exit(1)
        
        self.__device = str(self.__pipeline_profile.get_device().get_info(rs.camera_info.product_line))
        self.__delay_amount = delay
        self.__alignment = rs.align(rs.stream.color)
        #self.__alignment = rs.align(rs.stream.depth)

        # Configurations
        self.__config.enable_stream(rs.stream.depth, width, height, rs.format.z16, framerate)
        self.__config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, framerate)

        # Protected
        self._width = width
        self._height = height
        self._format = format

        if self.__device != "D400":
            print(f"WARNING: Found {self.__device}, which is not supported for use with the class RealSenseD435Facet. Please connect a RealSense D400 series device!")
        else:
            try:
                self.__start_pipeline()
                self.__running = True
            except:
                print("Unkown Error: Starting pipeline failed.")

    def __start_pipeline(self) -> None:
        self.__pipeline.start(self.__config)
        self.__starttime: float = time.time()
        #self.__delayed: bool = False
        print("INFO: Pipeline started.")

    ### properties ###
    @property
    def WIDTH(self):
        return self._width
    @property
    def HEIGHT(self):
        return self._height
    @property
    def FORMAT(self):
        return self._format
    ###################

    def __stop_pipeline(self) -> None:
        self.__pipeline.stop()
        print("INFO: Pipeline stopped.")

    # Dunder
    def __del__(self) -> None:
        if self.__running:
            self.__stop_pipeline()

    def take_snapshot(self,sharpen:bool = False) -> DepthImage:
        # based on: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
        """Takes a snapshot with the provided camera and saves the results in the [SolarSwarmAssets] folder.
If sharpen is True, then the image will be sharpened using the cv2 library.
The sharpened image will be saved in the /sharpened subdirectory."""
        
        # Delays the program in order to let the camera adjust to the environment
        dT: float = time.time() - self.__starttime
        if dT <= self.__delay_amount:
            delay: float = self.__delay_amount - dT
            print("Delay program by %.2fs in order to let the camera adjust to the environment..." % delay)
            time.sleep(delay)

        # Get frames
        frames: rs.composite_frame = self.__pipeline.wait_for_frames()
        # Align frames
        aligned_frames = self.__alignment.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return
            
        depth_data = np.asanyarray(aligned_depth_frame.get_data())
        color_img = np.asanyarray(color_frame.get_data())
        depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03),cv2.COLORMAP_JET)

        return DepthImage(colorim=color_img,depthim=depth_img,depth=depth_data)

def cv_view():
    # based on: https://www.youtube.com/watch?v=mFLZkdH1yLE&t=305s

    cam = RealSenseD435()

    pnt = (cam.WIDTH//2,cam.HEIGHT//2)
    while True:
        data = cam.take_snapshot()
        cv2.circle(data.colorim,pnt,4,(0,0,255)) #bgr

        dist = data.depth[pnt[1],pnt[0]]
        print(dist)

        cv2.imshow("depth",data.depthim)
        cv2.imshow("color",data.colorim)

        key = cv2.waitKey(1)
        if key == 27:
            break

if __name__ == "__main__":
    cv_view()