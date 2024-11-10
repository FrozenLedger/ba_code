import cv2, time
import pyrealsense2 as rs
import numpy as np

from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

from ba_code.msg import Metrics
from ba.perception.camera_logger import CAMERALOGGER as LOGGER

from ba.perception.rsd435_resolution import RealSenseD435ColorResolution, RealSenseD435DepthResolution

class FrameBuffer:
    """A class to store information about the depth and color information of an image taken from a scene and calculating the metrics of the depth information of an area in the image."""

    def __init__(self, aligned_frames, header):
        self.__aligned_frames = aligned_frames
        self.__imgs = {}
        self.__header = header

    @property
    def colorim(self):
        return self.__get_img("colorim", self.aligned_color_frame)

    @property
    def depthim(self):
        return cv2.applyColorMap(
            cv2.convertScaleAbs(self.depth_data, alpha=0.03), cv2.COLORMAP_JET
        )

    @property
    def depth_data(self):
        return self.__get_img("depthim", self.aligned_depth_frame)

    def __get_img(self, key, frame):
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
        return (
            self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        )

    @property
    def FOV(self):
        return rs.rs2_fov(self.intrinsics)

    def get_distance(self, px, py):
        return self.aligned_depth_frame.get_distance(px, py)

    def pixel_to_point3D(self, px, py):
        dist = self.get_distance(px, py)
        return self.pixel_with_distance_to_point3D(px, py, dist)

    def deproject_pixel_to_point3D(self, px, py, dist):
        pnt = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], dist)
        return PointStamped(header=self.__header, point=Point(*pnt))

    def get_distance_metrics(self, roi: RegionOfInterest):
        if roi.do_rectify:
            area = self.depth_data[
                roi.y_offset : roi.y_offset + roi.height,
                roi.x_offset : roi.x_offset + roi.width,
            ]
        else:
            area = self.depth_data
        h = area.shape[0]
        w = area.shape[1]

        px = w // 2
        py = h // 2

        c = 2  # center_size
        # center = area[py,px]/1000 #np.nanmedian(area[py-c:py+c+1,px-c:px+c+1])/1000
        center = (
            np.nanmedian(area[py - c : py + c + 1, px - c : px + c + 1]) / 1000
        )
        median = np.nanmedian(area) / 1000
        avg = np.nanmean(area) / 1000
        std = np.nanstd(area) / 1000
        # var = np.nanvar(area)/1000

        return Metrics(
            center=center, median=median, avg=avg, std=std
        )  # returns values in [m]

class RealSenseD435:
    """A class that will handle the communication with the real sense d435 camera and align the depth information with the color information."""

    def __init__(
        self,
        color_resolution: RealSenseD435ColorResolution = RealSenseD435ColorResolution.rs640x480, #width: int = 640,
        depth_resolution: RealSenseD435DepthResolution = RealSenseD435DepthResolution.rs640x480, #height: int = 480,
        format: rs.format = rs.format.z16,
        framerate: int = 30,
        delay: float = 5.0,
    ):
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__running = False

        try:
            self.__pipeline_profile = self.__config.resolve(
                rs.pipeline_wrapper(self.__pipeline)
            )
        except RuntimeError as e:
            #LOGGER.error("RuntimeError: No device connected.")
            LOGGER.error("[Camera] RuntimeError: " + str(e))
            exit(1)

        self.__device = str(
            self.__pipeline_profile.get_device().get_info(
                rs.camera_info.product_line
            )
        )
        self.__delay_amount = delay
        LOGGER.info(f"[Camera] Delay set to: {delay}")
        self.__alignment = rs.align(rs.stream.color)
        LOGGER.info(f"[Camera] Alignment: rs.align(rs.stream.color)")
        # self.__alignment = rs.align(rs.stream.depth)

        # Configurations
        depth_width,depth_height = depth_resolution.value
        color_width,color_height = color_resolution.value
        LOGGER.info(f"[Camera] Depth resolution set to: {depth_resolution.value}")
        LOGGER.info(f"[Camera] Color resolution set to: {color_resolution.value}")

        self.__config.enable_stream(
            rs.stream.depth, depth_width, depth_height, rs.format.z16, framerate
        )
        LOGGER.info("[Camera] Enabled depth stream.")
        self.__config.enable_stream(
            rs.stream.color, color_width, color_height, rs.format.bgr8, framerate
        )
        LOGGER.info("[Camera] Enabled color stream.")

        # Protected
        self._width = color_width
        self._height = color_height
        self._format = format
        LOGGER.info(f"[Camera] Width set to: {self._width}")
        LOGGER.info(f"[Camera] Height set to: {self._height}")
        LOGGER.info(f"[Camera] Format set to: {format}")

        if self.__device != "D400":
            LOGGER.warning(
                f"[Camera] Found {self.__device}, which is not supported for use with the class RealSenseD435Facet. Please connect a RealSense D400 series device!"
            )
        else:
            try:
                self.start_pipeline()
                self.__running = True
            except Exception as e:
                LOGGER.error(f"[Camera] {str(e)}")
                LOGGER.error("[Camera] Unkown Error. Starting pipeline failed.")
                exit(1)

    def start_pipeline(self) -> None:
        self.__pipeline.start(self.__config)
        self.__starttime: float = time.time()
        # self.__delayed: bool = False
        LOGGER.info("[Camera] Pipeline started.")

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
        LOGGER.info("[Camera] Pipeline stopped.")

    # Dunder
    def __del__(self) -> None:
        if self.__running:
            self.stop_pipeline()

    def take_snapshot(self, header: Header = Header()) -> FrameBuffer:
        # based on: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
        # also based on: https://github.com/InterlRealSense/librealsense/issues/2481
        """Takes an image of the scene and returns an object of the class FrameBuffer. Returns a FrameBuffer."""

        # Delays the program in order to let the camera adjust to the environment
        dT: float = time.time() - self.__starttime
        if dT <= self.__delay_amount:
            delay: float = self.__delay_amount - dT
            LOGGER.info(
                "Delay program by %.2fs in order to let the camera adjust to the environment..."
                % delay
            )
            time.sleep(delay)

        # Get frames
        try:
            frames: rs.composite_frame = self.__pipeline.wait_for_frames()
            # Align frames
            aligned_frames = self.__alignment.process(frames)
            # Get aligned frames
            frame_buffer = FrameBuffer(aligned_frames, header=header)
            return frame_buffer

        except RuntimeError as e:
            LOGGER.error(f"[Camera] {str(e)}")
        
        # Validate that both frames are valid
        #if (
        #    not frame_buffer.aligned_depth_frame
        #    or not frame_buffer.aligned_color_frame
        #):
        #    return



def cv_view(frame_rate=1, size=4):
    """A function to visualize the images taken by the real sense camera used for debugging purposes."""
    # based on: https://www.youtube.com/watch?v=mFLZkdH1yLE&t=305s

    cam = RealSenseD435()

    (px, py) = (cam.WIDTH // 2, cam.HEIGHT // 2)

    delay = int((1 / frame_rate) * 1000)
    while True:
        data = cam.take_snapshot()

        cv2.circle(data.colorim, (px, py), max(1, size), (0, 0, 255))  # bgr

        dist = data.depth[py, px]

        region = (px - 50, py - 50, 100, 100)
        result = data.region_to_depth(*region)
        print(dist)
        print(result)

        cv2.imshow("depth", data.depthim)
        cv2.imshow("color", data.colorim)

        key = cv2.waitKey(delay)
        if key == 27:
            break


def buffer_test():
    """A test to ensure the functionalities of the RealSenseD435 buffer."""
    import time

    cam = RealSenseD435(framerate=15)
    width = cam.WIDTH
    height = cam.HEIGHT

    success = False
    LOGGER.info("Start buffer test...")
    try:
        frames = cam.take_snapshot()
        time.sleep(1)
        cam.take_snapshot()
        LOGGER.info("1")
        time.sleep(1)
        cam.take_snapshot()
        LOGGER.info("2")
        time.sleep(1)
        cam.stop_pipeline()
        time.sleep(1)
        cam.start_pipeline()
        time.sleep(1)
        fb2 = cam.take_snapshot()
        LOGGER.info("3")
        del cam

        for i in range(5):
            time.sleep(1)
            LOGGER.info(f"Delay: {i}")

        px = width // 2
        py = height // 2
        pnt = frames.pixel_to_point3D(px, py)
        LOGGER.info(pnt)

        cv2.imshow("FB2", fb2.colorim)
        cv2.waitKey(0)
        success = True
    except Exception as e:
        LOGGER.error(str(e))
        success = False
    finally:
        if success:
            LOGGER.info("Buffer test successful.")
        else:
            LOGGER.info("Buffer test failed")

if __name__ == "__main__":
    # cv_view(frame_rate=5,size=4)
    buffer_test()