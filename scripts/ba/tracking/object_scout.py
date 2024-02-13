import rospy

import ba_code.srv as basrv
import ba_code.msg as bamsg

class ObjectScout:
    def __init__(self):
        self.__snapshot_req = rospy.ServiceProxy("/rs_d435/take_snapshot",basrv.TakeSnapshotStamped)
        self.__clear_frame_req = rospy.ServiceProxy("/rs_d435/frames/clear",basrv.ClearFrame)
        self.__detect_req = rospy.ServiceProxy("/yolov5/detect",basrv.Detect)
        self.__pixel_to_point3d_req = rospy.ServiceProxy("/rs_d435/frames/deproject_pixel_to_point3d",basrv.PixelToPoint3D)
        self.__add_object_req = rospy.ServiceProxy("/object_tracker/add",basrv.AddObject)
        self.__rate = rospy.Rate(0.5)

    def __look_around(self):
        snap_resp = self.__snapshot_req(add_buffer=True)
        try:
            imgID = snap_resp.imgID
            header = snap_resp.header

            det_resp = self.__detect_req(imgID)
            detection = det_resp.detection

            for idx in range(len(detection.clsID)):
                roi = detection.roi[idx]
                clsID = detection.clsID[idx]
                conf = detection.confidence[idx]
                metrics = detection.metrics[idx]

                dist = max(metrics.center,metrics.median)
                if dist <= 0.2:
                    continue

                px = roi.x_offset + roi.width//2
                py = roi.y_offset + roi.height//2

                p2p3d_resp = self.__pixel_to_point3d_req(imgID=imgID,
                                                px=px,
                                                py=py,
                                                distance=dist)
                obj = bamsg.Object()
                obj.point = p2p3d_resp.point
                obj.note.data = f"ClsID: {clsID}, CNN: yolov5"

                self.__add_object_req(obj)
                print(f"Object found. ClsID:{clsID}\tconf:{conf}")
        except Exception as e:
            print(e)
        finally:
            self.__clear_frame_req(imgID)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.__look_around()
            except Exception as e:
                print(e)
            self.__rate.sleep()

def main():
    rospy.init_node("object_scout")

    scout = ObjectScout()
    scout.loop()

if __name__ == "__main__":
    main()