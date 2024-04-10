#!/usr/bin/env python3

import rospy
import tf
import math
from threading import Thread, Event

from geometry_msgs.msg import PointStamped
from ba.utilities.transformations import quaternion_from_euler

def quaternion_from_frames(tf_listener: tf.TransformListener, origin_frame: str, target_frame: str, time: rospy.Time = None):
    t0 = time or rospy.Time.now()
    tf_listener.waitForTransform(origin_frame,target_frame,t0,rospy.Duration(0.25))

    target_point = PointStamped()
    target_point.header.frame_id = target_frame
    target_point.header.stamp = t0

    transformed_point = tf_listener.transformPoint(origin_frame,target_point)
    (x,y) = (transformed_point.point.x,transformed_point.point.y)
    alpha = math.atan2(y,x)
    return quaternion_from_euler((0,0,alpha))

class ROSMapPointer:
    """A frame that points with its x-axis towards a target frame."""
    def __init__(self,origin_frame: str, targte_frame: str, rate: rospy.Rate = None):
        self._frame_id = "arrow"
        self._origin_frame: str = origin_frame
        self._target_frame: str = targte_frame
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self._rate = rate or rospy.Rate(5)
        self._stop_event = Event()
        self._task = Thread(target=self._publish)
        self._task.start()

    def _publish(self):
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                t0 = rospy.Time.now()
                quaternion = quaternion_from_frames(self._tf_listener, self._origin_frame, self._target_frame, time=t0)
                self._tf_broadcaster.sendTransform(
                    (0,0,0),
                    (quaternion.x,quaternion.y,quaternion.z,quaternion.w),
                    t0,
                    self._frame_id,
                    self._origin_frame
                )
                self._rate.sleep()
            except rospy.ROSInterruptException:
                self._stop_event.set()

    def __del__(self):
        self._stop_event.set()
        self._task.join()

if __name__ == "__main__":
    rospy.init_node("base_station_node")
    arrow = ROSMapPointer(origin_frame="map",targte_frame="base_link",rate=rospy.Rate(10))
    rospy.spin()