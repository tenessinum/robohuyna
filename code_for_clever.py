import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ahuet_kakoy_kod import get_yaw

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

image_pub = rospy.Publisher('~/robo_flow/debug', Image, queue_size=1)
yaw_pub = rospy.Publisher('~yaw', Float64, queue_size=1)


def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    output_frame, yaw = get_yaw(cv_image)
    image_pub.publish(bridge.cv2_to_imgmsg(output_frame, 'bgr8'))
    if yaw:
        yaw_pub.publish(yaw)


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
rospy.spin()
