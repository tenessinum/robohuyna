import time
import math

import rospy

from std_msgs.msg import Float64MultiArray

from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('fly')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry, persistent=True)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
# set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
# set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


max_velocity = 1.2
yaw_p = 0.55
y_p = 0.004
midle_yaw = 0


def get_velocity(_yaw):
    global max_velocity
    return max_velocity * (1 - abs(_yaw) / math.pi)


def yaw_msg_callback(data):
    global current_time
    global midle_yaw, y_p

    #if time.time() < current_time + 0.07:
    #    pass
    #else:
    yaw, y = data.data
    print yaw, y * y_p
    y_vel = -y * y_p
    y_vel = y_vel * (abs(yaw) / math.pi)
    set_velocity(vx=get_velocity(yaw), vy=y_vel, vz=0, yaw=-yaw * yaw_p, frame_id="body")

    current_time = time.time()

    # midle_yaw = midle_yaw * 0.9 + 0.1 * data.data

    return


print navigate(x=0, y=0, z=1.4, speed=0.5, auto_arm=True, frame_id='body')
rospy.sleep(5)

print 'Subscribe'
take_yaw = rospy.Subscriber('computer_vision_sample/yaw', Float64MultiArray, yaw_msg_callback, queue_size=1)

current_time = time.time()

rospy.spin()