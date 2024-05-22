import rospy

from geometry_msgs.msg import PointStamped

from ba_code.srv import AddObject
from ba_code.msg import Object

class ObjectSpawner:
    def __init__(self):
        self.__listener = rospy.Subscriber("/object_spawner/spawn",PointStamped,self.__publish)
        self.__add_object = rospy.ServiceProxy("/object_tracker/add",AddObject)
        print("ObjectSpawner initialized.")

    def __publish(self,msg):
        obj = Object()
        obj.point = msg
        obj.clsID = 1
        obj.objID = 0
        obj.clsName.data = f"paper{obj.objID}"
        obj.confidence = 1
        obj.note.data = "test object"
        success = self.__add_object(obj)
        print(f"Try adding test object. Success: {success}")

def main():
    rospy.init_node("object_spawner")

    object_spawner = ObjectSpawner()

    rospy.spin()

if __name__ == "__main__":
    main()