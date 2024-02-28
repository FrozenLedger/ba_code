import roslaunch,rospy

import ba_code.srv as basrv

class ExploreLiteInterface:
    def __init__(self,path):
        self.__service = rospy.Service("/explorer/explore_for_s",basrv.Explore,self.__explore_enable)
        self.__path = path

        self.__idle = rospy.Rate(1)
        self.__duration = 0

        self.__success = False
        self.__enabled = False

        rospy.loginfo("ExploreLiteInterface running.")

    def __explore_enable(self,req):
        try:
            self.__duration = req.duration
            self.__finished = False
            self.__enabled = True

            while not rospy.is_shutdown() and not self.__finished:
                self.__idle.sleep()

            return self.__success
        except Exception as e:
            print(e)
            return False

    def __explore(self):
        # based on: https://wiki.ros.org/roslaunch/API%20Usage
        # uses: https://wiki.ros.org/explore_lite
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            print(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{self.__path}/explore.launch"])
            launch.start()
            rospy.loginfo("Start exploration.")

            rospy.sleep(self.__duration)

            #launch.shutdown()
        
            self.__success = True
        except Exception as e:
            print(e)
            self.__success = False
        finally:
            self.__enabled = False
            self.__finished = True
            launch.shutdown()
            rospy.loginfo("End exploration.")
    
    def loop(self):
        running = True
        while running and not rospy.is_shutdown():
            try:
                if self.__enabled:
                    self.__explore()
                self.__idle.sleep()
            except:
                running = False
            
if __name__ == "__main__":
    rospy.init_node("exploration_runner")
    explorer = ExploreLiteInterface(path="/home/workspace1/github/catkin_ws/src/ba_code/launch")
    explorer.loop()