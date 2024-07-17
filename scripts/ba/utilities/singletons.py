import rospy
import tf

__TF_TRANSFORM_LISTENER = None
def get_transform_listener():
    global __TF_TRANSFORM_LISTENER
    if __TF_TRANSFORM_LISTENER is None:
        print("Initialize TransformListener...")
        try:
            __TF_TRANSFORM_LISTENER = tf.TransformListener()
            print(f"TransformListener initialized:\t{__TF_TRANSFORM_LISTENER}")
        except rospy.exceptions.ROSInitException as e:
            print(e)
            print("Initialization failed.")
            return None
    return __TF_TRANSFORM_LISTENER