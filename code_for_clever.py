import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from ahuet_kakoy_kod import get_yaw

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

image_pub = rospy.Publisher('~/robo_flow/debug', Image, queue_size=1)
mask_pub = rospy.Publisher('~/robo_flow/mask/debug', Image, queue_size=1)
yaw_pub = rospy.Publisher('~yaw', Float64MultiArray, queue_size=1)


def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    output_frame, mask, yaw, y = get_yaw(cv_image)
    mask_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))
    image_pub.publish(bridge.cv2_to_imgmsg(output_frame, 'bgr8'))
    if yaw:
        yaw_pub.publish(Float64MultiArray(data=[yaw, y]))

print 'Subscribing to main_camera/image_raw'
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
rospy.spin()
