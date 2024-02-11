import cv2,time
import pyrealsense2 as rs
import numpy as np

class FrameBuffer:
    def __init__(self,aligned_frames):
        self.__aligned_frames = aligned_frames
        self.__imgs = {}

    @property
    def colorim(self):
        return self.__get_img("colorim",self.aligned_color_frame)
    @property
    def depthim(self):
        return cv2.applyColorMap(cv2.convertScaleAbs(self.depth_data, alpha=0.03),cv2.COLORMAP_JET)
    @property
    def depth_data(self):
        return self.__get_img("depthim",self.aligned_depth_frame)

    def __get_img(self,key,frame):
        if key not in self.__imgs:
            self.__imgs[key] = np.asanyarray(frame.get_data())
        return self.__imgs[key]

    @property
    def aligned_depth_frame(self):
        return self.__aligned_frames.get_depth_frame()
    
    @property
    def aligned_color_frame(self):
        return self.__aligned_frames.get_color_frame()
    
    @property
    def intrinsics(self):
        return self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    
    @property
    def FOV(self):
        return rs.rs2_fov(self.intrinsics)
    
    def get_distance(self,px,py):
        return self.aligned_depth_frame.get_distance(px,py)
        
    def pixel_to_point3D(self,px,py):
        return rs.rs2_deproject_pixel_to_point(self.intrinsics,[px,py], self.get_distance(px,py))
        

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
                self.start_pipeline()
                self.__running = True
            except:
                print("Unkown Error: Starting pipeline failed.")

    def start_pipeline(self) -> None:
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

    def stop_pipeline(self) -> None:
        self.__pipeline.stop()
        print("INFO: Pipeline stopped.")

    # Dunder
    def __del__(self) -> None:
        if self.__running:
            self.stop_pipeline()

    def take_snapshot(self) -> FrameBuffer:
        # based on: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
        # also based on: https://github.com/InterlRealSense/librealsense/issues/2481
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
        #aligned_depth_frame = aligned_frames.get_depth_frame()
        #color_frame = aligned_frames.get_color_frame()
        frame_buffer = FrameBuffer(aligned_frames)

        #print(aligned_depth_frame.get_profile())
        
        # Validate that both frames are valid
        if not frame_buffer.aligned_depth_frame or not frame_buffer.aligned_color_frame:
            return

        # depth_data = frame_buffer.depth_data #np.asanyarray(frame_buffer.aligned_depth_frame.get_data())
        # color_img = frame_buffer.colorim #color_img = np.asanyarray(frame_buffer.aligned_color_frame.get_data())
        # depth_img = frame_buffer.depthim #cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03),cv2.COLORMAP_JET)

        # h,w = depth_data.shape
        # px = w//2
        # py = h//2
        # dist = frame_buffer.aligned_depth_frame.get_distance(px,py)
        # pnt = frame_buffer.pixel_to_point3D(px,py)
        #print(dist,depth_data[py,px])
        #print(f"Pnt: {pnt}")
        #print(frame_buffer.FOV)

        return frame_buffer #DepthImage(colorim=color_img,depthim=depth_img,depth=depth_data)
    
def cv_view(frame_rate=1,size=4):
    # based on: https://www.youtube.com/watch?v=mFLZkdH1yLE&t=305s

    cam = RealSenseD435()

    (px,py) = (cam.WIDTH//2,cam.HEIGHT//2)

    delay = int((1/frame_rate)*1000)
    while True:
        data = cam.take_snapshot()
        
        cv2.circle(data.colorim,(px,py),max(1,size),(0,0,255)) #bgr

        dist = data.depth[py,px]
        
        region = (px-50,py-50,100,100)
        result = data.region_to_depth(*region)
        print(dist)
        print(result)

        cv2.imshow("depth",data.depthim)
        cv2.imshow("color",data.colorim)

        key = cv2.waitKey(delay)
        if key == 27:
            break

def buffer_test():
    import time
    cam = RealSenseD435()

    width = cam.WIDTH
    height = cam.HEIGHT

    frames = cam.take_snapshot()
    time.sleep(1)
    cam.take_snapshot()
    print("1")
    time.sleep(1)
    cam.take_snapshot()
    print("2")
    time.sleep(1)
    cam.stop_pipeline()
    time.sleep(1)
    cam.start_pipeline()
    time.sleep(1)
    fb2 = cam.take_snapshot()
    print("3")
    del cam

    for i in range(5):
        time.sleep(1)
        print(f"Delay: {i}")

    px = width//2
    py = height//2
    pnt = frames.pixel_to_point3D(px,py)
    print(pnt)

    cv2.imshow("FB2",fb2.colorim)
    cv2.waitKey(0)

if __name__ == "__main__":
    #cv_view(frame_rate=5,size=4)
    buffer_test()