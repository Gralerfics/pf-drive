import threading

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pf_drive.util.debug import Debugger
from pf_drive.util.geometry import Frame, Vec3, Quat
from pf_drive.simulation.webots import WebotsAckermannController


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

cmd_v = 0.0
cmd_R = 1e9
odom_frame = Frame()


def cmd_vel_received(msg):
    global cmd_v, cmd_R
    cmd_v, cmd_R = wac.command(msg.linear.x, msg.angular.z, try_best_w = True)


sub_cmd_vel = rospy.Subscriber('/car/cmd_vel', Twist, cmd_vel_received)
pub_odom = rospy.Publisher('/car/odom', Odometry, queue_size = 1)

tf_broadcaster = tf.TransformBroadcaster()
tf_broadcaster.sendTransform(
    [0, 0, 0],
    [0, 0, 0, 1],
    rospy.Time.now(),
    'base_link',
    'odom'
)

spin_thread = threading.Thread(target = rospy.spin)
spin_thread.start()

last_time = wac.get_time()
while not rospy.is_shutdown():
    # TODO: reset 后指令保持导致累加
    tmp_last_time = last_time

    current_time = wac.get_time()
    last_time = current_time
    if current_time is None or tmp_last_time is None:
        continue
    dt = current_time - tmp_last_time

    dist = cmd_v * dt

    if cmd_R > 1e9: # inf
        odom_frame.translation += odom_frame.q.rotate(Vec3(dist, 0, 0))
    else:
        theta = dist / cmd_R
        rot = Quat.from_rotation_vector(Vec3(0, 0, theta))

        # P: robot, O: origin, R: instantaneous center of rotation, T: target of P
        PR = odom_frame.q.rotate(Vec3(0, cmd_R, 0))
        RT = -rot.rotate(PR)
        PT = PR + RT

        odom_frame.quaternion = rot * odom_frame.q
        odom_frame.translation = odom_frame.t + PT

    # TODO: fix the situation where the values are nan. 看起来问题不在这里，是传入的数据已经 nan.
    if odom_frame.translation.has_nan() or odom_frame.quaternion.has_nan():
        Debugger.error('Nan value detected in odom_frame. Breaking the loop.')
        break

    pub_odom.publish(odom_frame.to_Odometry(frame_id = 'odom', time_stamp = rospy.Time.now()))
    # rospy.loginfo(rospy.Time.now().to_sec())
    tf_broadcaster.sendTransform(
        odom_frame.t.to_list(),
        odom_frame.q.to_list(),
        rospy.Time.now(),
        'base_link',
        'odom'
    )

