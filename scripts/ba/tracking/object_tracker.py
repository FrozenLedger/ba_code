import rospy,tf

import ba_code.srv as basrv
import ba_code.msg as bamsg

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

TRACKERNAMESPACE = "object_tracker"
class ObjectTracker:
    """A server node to track objects and trash detected in the scene."""
    def __init__(self,world="map"):
        self.__tf_listener = tf.TransformListener()
        self.__world = world
        self.__init_services()
        self._reset()

    def _reset(self):
        self.__objects = {}
        self.__count = 0
        if rospy.has_param(f"/{TRACKERNAMESPACE}/max_distance"):
            self._max_distance = rospy.get_param(f"/{TRACKERNAMESPACE}/max_distance")
        else:
            self._max_distance = 1200 #cm
        #if rospy.has_param(f"/{TRACKERNAMESPACE}/min_gap"):
        #    self._min_gap = rospy.get_param(f"/{TRACKERNAMESPACE}/min_gap")

    def _reset_request(self,req):
        self._reset()
        return TriggerResponse(success=True)

    def __init_services(self):
        self.__add_object_service = rospy.Service(f"/{TRACKERNAMESPACE}/add",basrv.AddObject,self.__add_object)
        self.__pop_object_service = rospy.Service(f"/{TRACKERNAMESPACE}/pop",basrv.PopObject,self.__pop_object)
        self.__get_list_service = rospy.Service(f"/{TRACKERNAMESPACE}/list",basrv.GetObjectList,self.__get_objects)
        #self.__max_distance_setter = rospy.Service(f"/{TRACKERNAMESPACE}/reset",Trigger,self._reset_request)
        self.__reset_service = rospy.Service(f"/{TRACKERNAMESPACE}/reset",Trigger,self._reset_request)

    def __add_object(self,request):
        obj = request.object
        try:
            t0 = obj.point.header.stamp
            self.__tf_listener.waitForTransform(self.__world,obj.point.header.frame_id,t0,rospy.Duration(1))

            obj.point = self.__tf_listener.transformPoint(self.__world,obj.point)
            if obj.point.point.z > 0.1:
                print("Object is above 10cm from the ground. Object will not be added to the list.")
                return False
            return self.__add(obj)
        except Exception as e:
            print(e)
        return False

    def __add(self,msg:bamsg.Object):
        h = self.__calc_hash(msg)

        if h not in self.__objects:
            self.__objects[h] = msg
            self.__objects[h].objID = self.__count
            print(f"Object added.\nh: {h}\nnote: {msg.note}\nobjID: {self.__objects[h].objID}\nclsID: {self.__objects[h].clsID}\nname: {msg.clsName}\n")
            self.__count += 1
            return True
        return False

    def __calc_hash(self,msg:bamsg.Object):
        h = (int(msg.point.point.x*100),int(msg.point.point.y*100),int(msg.point.point.z*100))
        return h
    
    def __pop_object(self,_):
        if len(self.__objects) > 0:
            for k in self.__objects:
                key = k
                break
            obj = self.__objects.pop(key)
            return obj
        return bamsg.Object(note=String(data="empty"))
    
    def __get_objects(self,_):
        result = basrv.GetObjectListResponse()
        for k,v in self.__objects.items():
            result.objects.append(v)
        return result

def main():
    rospy.init_node(TRACKERNAMESPACE)
    tracker = ObjectTracker()

    rospy.spin()

if __name__ == "__main__":
    main()