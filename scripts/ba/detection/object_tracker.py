import rospy

from ba_code.msg import ObjectPosition
from ba_code.srv import GetTrackedObjects, GetTrackedObjectsResponse
from ba_code.srv import RemoveObject,RemoveObjectResponse

from std_msgs.msg import String
#from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger,TriggerResponse

class ObjectTracker:
    def __init__(self):
        self.__clear(None)
        self.__rate = rospy.Rate(0.1) # publishes objects
        
        self.__init_subscriber()
        self.__init_services()

    def __init_subscriber(self):
        self.__object_sub = rospy.Subscriber("/detection/object",ObjectPosition,self.__add_object)
        self.__trash_sub = rospy.Subscriber("/detection/trash",ObjectPosition,self.__add_object)

    def __init_services(self):
        self.__service = rospy.Service("/object_tracker/tracked",GetTrackedObjects,self.__get_tracked_objects)
        self.__clear_service = rospy.Service("/object_tracker/clear",Trigger,self.__clear)
        self.__unregister_object_service = rospy.Service("/object_tracker/remove",RemoveObject,self.__unregister_object)

    def __clear(self,request):
        self.__objects = {}
        resp = TriggerResponse(success = True)
        return resp

    def __add_object(self,msg):
        point = msg.point.point
        x = int(point.x*100)
        y = int(point.y*100)
        z = int(point.z*100)
        key = str((x,y,z))
        if key not in self.__objects:
            print(f"Add object: {msg}")
            self.__objects[key] = msg
                                #{"clsID:":msg.clsID,
                                # "confidence":msg.confidence,
                                # "note":msg.note,
                                # "point":msg.point}
            print(f"Added new with key: {key}")
        else:
            print("[Warning] Object with same key already registered.")

    def __remove_object(self,key):
        if key in self.__objects:
            del self.__objects[key]
            return True
        return False

    def __unregister_object(self,request):
        key = request.key.data
        response = RemoveObjectResponse()
        response.success = self.__remove_object(key)
        return response

    def __get_tracked_objects(self,request):
        options = request.options
        print("Options: ", options)

        response = GetTrackedObjectsResponse()
        
        values = []
        keys = []
        for k,v in self.__objects.items():
            values.append(v)
            keys.append(String(data=k))
        response.objects = values
        response.keys = keys
        return response
            
if __name__ == "__main__":
    rospy.init_node("object_tracker")

    object_tracker = ObjectTracker()
    rospy.spin()