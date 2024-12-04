import cv2
import numpy as np

from sensor_msgs.msg import RegionOfInterest

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
