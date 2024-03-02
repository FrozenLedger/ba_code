import rospy

from ba.utilities.imageprocessing import expand_ocg
from ba.utilities.data import Data

from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap

if __name__ == "__main__":
    node = rospy.init_node("mapexpander")

    print("Wait for /map message...")
    ocg = rospy.wait_for_message("/map",OccupancyGrid)
    data = Data(ocg)
    def send_map(req):
        return data.data

    print("Register publisher and subscriber...")    
    #static_map = rospy.Service("/static_map",GetMap,send_map)
    map_pub = rospy.Publisher("/mapexpanded",OccupancyGrid,queue_size=1)
    meta_pub = rospy.Publisher("/mapexpanded_metadata",MapMetaData,queue_size=1)

    print("Publish map.")
    def republish(ocg):
        ocgexpanded = expand_ocg(ocg,size=100)
        data.setData(ocgexpanded)
        map_pub.publish(ocgexpanded)
        meta_pub.publish(ocgexpanded.info)
    republish(data.data)

    sub = rospy.Subscriber("/map", OccupancyGrid, republish)
    rospy.spin()