#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import ba_code.srv as basrv

from ba.tracking.object_tracker import TRACKERNAMESPACE

def spawn_obstacles(get_obstacles_callback, publisher, rate):
    obstacles = get_obstacles_callback().objects
    
    cloud_msg = PointCloud()
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = "map"

    for obj in obstacles:
        for x in range(-1,1):
            for y in range(-1,1):
                point = Point32()
                point.x = obj.point.point.x +x*0.05
                point.y = obj.point.point.y +y*0.05
                point.z = obj.point.point.z
                cloud_msg.points.append(point)

    publisher.publish(cloud_msg)
    
    rate.sleep()

def publish_virtual_obstacles():
    rospy.init_node("virtual_obstacles_node")

    publisher = rospy.Publisher("/obstacles_cloud", PointCloud, queue_size=1)
    
    obstacle_tracker = f"/{TRACKERNAMESPACE}/list"
    rospy.wait_for_service(obstacle_tracker)
    get_obstacles_callback = rospy.ServiceProxy(obstacle_tracker, basrv.GetObjectList)
    
    rate = rospy.Rate(10)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            spawn_obstacles(get_obstacles_callback, publisher, rate)
        except rospy.ServiceException as e:
            print(e)
            rospy.wait_for_service(obstacle_tracker)
            print(f"service [{obstacle_tracker}] available")

if __name__ == '__main__':
    try:
        publish_virtual_obstacles()
    except rospy.ROSInterruptException as e:
        print(e)
