import threading

import rospy
from geometry_msgs.msg import Twist

from tr_drive.util.debug import Debugger
from tr_drive.util.geometry import Frame
from tr_drive.simulation.webots import WebotsAckermannController


rospy.init_node('tr_car_wbt_controller_node', anonymous = False)

wac = WebotsAckermannController(
    'left_front_steer_motor',
    'right_front_steer_motor',
    'left_rear_motor',
    'right_rear_motor',
    1.628,
    2.995,
    0.38,
    0.78,
    '/car'
)

cmd_linear = 0.0
cmd_angular = 0.0


def cmd_vel_received(msg):
    cmd_linear = msg.linear.x
    cmd_angular = msg.angular.z
    rospy.loginfo('cmd_vel_received: linear = %f, angular = %f' % (cmd_linear, cmd_angular))
    wac.command(cmd_linear, cmd_angular, try_best_w = True)


sub_cmd_vel = rospy.Subscriber('/car/cmd_vel', Twist, cmd_vel_received)
pub_odom = rospy.Publisher('/car/odom', Twist, queue_size = 1)

spin_thread = threading.Thread(target = rospy.spin)
spin_thread.start()

while not rospy.is_shutdown():
    import time
    time.sleep(1)

