import rospy

import ba_code.srv as basrv

def main():
    rospy.init_node("camera_unit_test")

    def success(srv):
        print(f"{srv}. Success.")

    snap_srv = "/rs_d435/take_snapshot"
    rospy.wait_for_service(snap_srv,rospy.Duration(3))
    __snapshot = rospy.ServiceProxy(snap_srv,basrv.TakeSnapshotStamped)
    imgID = __snapshot(add_buffer=True).imgID
    success(snap_srv)

    color_srv = "/rs_d435/frames/color"
    rospy.wait_for_service(color_srv,rospy.Duration(3))
    __color = rospy.ServiceProxy(color_srv,basrv.SendImage)
    colorim = __color(imgID=imgID).data
    success(color_srv)

    depth_srv = "/rs_d435/frames/depthim"
    rospy.wait_for_service(depth_srv,rospy.Duration(3))
    __depth = rospy.ServiceProxy(depth_srv,basrv.SendImage)
    depthim = __depth(imgID=imgID).data
    success(depth_srv)

    metrics_srv = "/rs_d435/frames/metrics"
    rospy.wait_for_service(metrics_srv,rospy.Duration(3))
    __metrics = rospy.ServiceProxy(metrics_srv,basrv.GetMetrics)
    met = __metrics(imgID=imgID).metrics
    dist = met.center
    success(metrics_srv)

    pixel_to_point3d_srv = "/rs_d435/frames/deproject_pixel_to_point3d"
    rospy.wait_for_service(pixel_to_point3d_srv,rospy.Duration(3))
    __p2p3d = rospy.ServiceProxy(pixel_to_point3d_srv,basrv.PixelToPoint3D)
    pnt = __p2p3d(imgID=imgID,px=0,py=0,distance=dist).point
    success(pixel_to_point3d_srv)

    clear_srv = "/rs_d435/frames/clear"
    rospy.wait_for_service(clear_srv,rospy.Duration(3))
    __clear = rospy.ServiceProxy(clear_srv,basrv.ClearFrame)
    __clear(imgID=imgID)
    success(clear_srv)

if __name__ == "__main__":
    main()