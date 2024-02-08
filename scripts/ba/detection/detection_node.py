import rospy

class DetectionServer:
    def __init__(self):
        self.__sub = rospy.Subscriber("/rs_rgb_img",)

        self.__init_services()

    def __init_services(self):
        self.__server = rospy.Service("/predict",)

if __name__ == "__main__":
    pass