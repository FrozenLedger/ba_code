import rospy
import tf

__TF_LISTENER_NS = "TransformListener"
__TF_TRANSFORM_LISTENER = None
def get_transform_listener():
    global __TF_TRANSFORM_LISTENER
    if __TF_TRANSFORM_LISTENER is None:
        print(f"Initialize {__TF_LISTENER_NS}...")
        try:
            __TF_TRANSFORM_LISTENER = tf.TransformListener()
            print(f"{__TF_LISTENER_NS} initialized:\t{__TF_TRANSFORM_LISTENER}")
        except rospy.exceptions.ROSInitException as e:
            print(e)
            print("Initialization failed.")
            return None
    return __TF_TRANSFORM_LISTENER