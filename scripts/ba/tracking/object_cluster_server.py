import rospy
import tf
import ba_code.srv as basrv

from ba_code.msg import Object
from ba.tracking.object_cluster import ObjectClusterTracker
from ba.tracking.object_tracker import TRACKERNAMESPACE

OBJECTCLUSTERNS = "object_cluster_server"
class ObjectClusterServer:
    def __init__(self, origin="map", epsilon=0.05, filter_cb = lambda obj: True):
        self.__object_cluster_tracker = ObjectClusterTracker(epsilon=epsilon)
        self.__tf_listener = tf.TransformListener()
        self.__origin = origin
        self.__filter_cb = filter_cb
        self.__init_services()

    def __init_services(self):
        self.__add_object_service = rospy.Service(f"/{TRACKERNAMESPACE}/add",basrv.AddObject,self.__add_object)
        self.__pop_object_service = rospy.Service(f"/{TRACKERNAMESPACE}/pop",basrv.PopObject,self.__pop_object)
        self.__get_list_service = rospy.Service(f"/{TRACKERNAMESPACE}/list",basrv.GetObjectList,self.__get_list)

    def __add_object(self, req):
        obj: Object = req.object
        try:
            t0 = obj.point.header.stamp
            self.__tf_listener.waitForTransform(self.__origin,obj.point.header.frame_id,t0,rospy.Duration(1))

            obj.point = self.__tf_listener.transformPoint(self.__origin,obj.point)
            
            if self.__filter_cb(obj):
                self.__object_cluster_tracker.eval(obj)
                return True
            return False
        except Exception as e:
            print(e)
        return False

    def __pop_object(self, req) -> Object:
        return self.__object_cluster_tracker.pop().avg

    def __get_list(self, req):
        lst: list[Object] = list(map(lambda cluster: cluster.avg, self.__object_cluster_tracker.clusters))
        result: basrv.GetObjectListResponse = basrv.GetObjectListResponse()
        result.objects = lst
        #rospy.loginfo(f"Requested list: {lst}")
        return result
    
if __name__ == "__main__":
    rospy.init_node(TRACKERNAMESPACE)
    epsilon = rospy.get_param(f"/{OBJECTCLUSTERNS}/ObjectTracker/epsilon", 0.05)
    max_z: float = rospy.get_param(f"/{OBJECTCLUSTERNS}/ObjectTracker/max_z",0.1)
    object_filter = lambda obj: obj.point.point.z <= max_z
    #print(f"Initializing: {OBJECTCLUSTERNS}",f"\n\tepsilon: {epsilon}",f"\n\tmax_z: {max_z}")
    tracker = ObjectClusterServer(epsilon=epsilon, filter_cb=object_filter)
    rospy.spin()