import time
import math

import rospy

from std_msgs.msg import Float64

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

yaw_p = 0.55
midle_yaw = 0

def yaw_msg_callback(data):
    global current_time
    global midle_yaw
    
    if time.time() < current_time + 0.07:
        pass
    else:
        if -0.12 < midle_yaw < 0.12:
            set_velocity(vx=1.0, vy=0, vz=0, frame_id='body')
        else:
            set_velocity(vx=0.6, vy=0, vz=0, yaw=-float(data.data) * yaw_p, frame_id="body")
        
        current_time = time.time()
        print "Do yaw control, yaw = ", str(data.data * yaw_p)

    midle_yaw = midle_yaw * 0.9 + 0.1 * data.data

    return


navigate(x=0, y=0, z=1.2, speed=0.75, auto_arm=True, frame_id='body')
rospy.sleep(5)

print 'Subscribe'
take_yaw = rospy.Subscriber('computer_vision_sample/yaw', Float64, yaw_msg_callback, queue_size=1)

current_time = time.time()

rospy.spin()
