import rospy,tf

import ba_code.msg as bamsg
import ba_code.srv as basrv

from std_msgs.msg import String
#from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger,TriggerResponse
from sensor_msgs.msg import RegionOfInterest

from geometry_msgs.msg import PointStamped

class ObjectTracker:
    def __init__(self):
        self.__objects = {}
        self.__clear(None)
        #self.__rate = rospy.Rate(0.1) # publishes objects
        self.__tf_listener = tf.TransformListener()
        self.__world = "map"
        rospy.Rate(1).sleep()

        self.__init_services()
        self.__init_service_proxies()

    def __init_services(self):
        self.__register_objects = rospy.Service("/object_tracker/snap",Trigger,self.__snap)
        self.__service = rospy.Service("/object_tracker/tracked",basrv.GetTrackedObjects,self.__get_tracked_objects)
        self.__clear_service = rospy.Service("/object_tracker/clear",Trigger,self.__clear)
        self.__unregister_object_service = rospy.Service("/object_tracker/remove",basrv.RemoveObject,self.__unregister_object)

    def __init_service_proxies(self):
        self.__take_snapshot_req = rospy.ServiceProxy("/rs_d435/take_snapshot",basrv.TakeSnapshotStamped)
        self.__clear_frame_req = rospy.ServiceProxy("/rs_d435/frames/clear",basrv.ClearFrame)
        self.__pixel_to_point3D_req = rospy.ServiceProxy("/rs_d435/frames/deproject_pixel_to_point3d",basrv.PixelToPoint3D)
        self.__get_metrics_req = rospy.ServiceProxy("/rs_d435/frames/metrics",basrv.GetMetrics)
        self.__detect_trash_req = rospy.ServiceProxy("/yolov5/detect",basrv.Detect)
        self.__detect_object_req = rospy.ServiceProxy("/trashnet/detect",basrv.Detect)

    def __clear(self,request):
        self.__objects = {}
        resp = TriggerResponse(success = True)
        return resp

    def __add_object(self,msg):
        t0 = rospy.Time.now()
        origin = msg.point.header.frame_id
        self.__tf_listener.waitForTransform(origin,self.__world,t0,rospy.Duration(0.25))
        point = self.__tf_listener.transformPoint(self.__world,msg.point)
        x = int(point.point.x*100)
        y = int(point.point.y*100)
        z = int(point.point.z*100)
        key = str((x,y,z))
        if key not in self.__objects:
            msg.point = point
            msg.key.data = key
            self.__objects[key] = msg
                                #{"clsID:":msg.clsID,
                                # "confidence":msg.confidence,
                                # "note":msg.note,
                                # "point":msg.point}
            print(self.__objects[key])
            print(f"Add object: {msg} with key: {key}")
        else:
            print("[Warning] Object with same key already registered.")

    def __remove_object(self,key):
        if key in self.__objects:
            del self.__objects[key]
            return True
        return False

    def __unregister_object(self,request):
        key = request.key.data
        response = basrv.RemoveObjectResponse()
        response.success = self.__remove_object(key)
        return response

    def __get_tracked_objects(self,request):
        res = basrv.GetTrackedObjectsResponse()
        for k in self.__objects:
            print("A: ", self.__objects[k],k)
            res.objects.append(self.__objects[k])
        return res
    
    def __snap(self,request):
        imgID = self.__take_snapshot_req(add_buffer=True).imgID
        try:
            #trash_resp = self.__detect_trash(imgID)
            det_obj = self.__detect_object_req(imgID).detection

            for idx in range(len(det_obj.clsID)):
                if idx > 0:
                    continue

                w = det_obj.xmax[idx] - det_obj.xmin[idx]
                h = det_obj.ymax[idx] - det_obj.ymin[idx]
                roi = RegionOfInterest(x_offset=det_obj.xmin[idx],
                                       y_offset=det_obj.ymin[idx],
                                       width=w,
                                       height=h,
                                       do_rectify=True)
                metrics = self.__get_metrics_req(imgID,roi).metrics

                if metrics.center == 0 and metrics.median == 0:
                    print("[Warning] Center and median values are 0.")
                    continue

                px = det_obj.xmin[idx] + w//2
                py = det_obj.ymin[idx] + h//2
                dist = max(metrics.center,metrics.median)

                pnt:PointStamped = self.__pixel_to_point3D_req(imgID=imgID,
                                        px=px,
                                        py=py,
                                        distance=dist).point

                msg = bamsg.ObjectPosition(point=pnt,
                                     note="object",
                                     clsID=det_obj.clsID[idx],
                                     confidence=det_obj.confidence[idx])
                print("Here:",msg)
                self.__add_object(msg)
                return TriggerResponse(success=True)
        except Exception as e:
            print(e)
        finally:
            self.__clear_frame_req(imgID)
        return TriggerResponse(success=False)

def main():
    rospy.init_node("object_tracker")
    object_tracker = ObjectTracker()
    print("Node running.")
    rospy.spin()
    
if __name__ == "__main__":
    main()