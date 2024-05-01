import time
import json
import signal
import argparse
import multiprocessing as mp

import rospy
from nav_msgs.msg import Odometry

# from cv_bridge import CvBridge

from multinodes import Cable

from pf_drive.util import t3d_ext, stamp_str, fetch
from pf_drive.device.ros_camera import ROSCameraWithResizeGrayscaleAndPatchNormalization
from pf_drive.controller.keyboard_ackermann_controller import KeyboardAckermannController
from pf_drive.actuator.webots_ros_ackermann_actuator import WebotsROSAckermannActuator


"""
    Arguments and Configurations
"""
# python src/webots_ros_ackermann_teacher.py --config ./config/webots_ros_ackermann_teach.json
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config', type = str, default = './config/teacher.json')
parser.add_argument('-o', '--output', type = str, default = None)
args = parser.parse_args()

config_path = args.config
config = json.load(open(config_path, 'r'))
if 'save_path' not in config:
    config['save_path'] = args.output if args.output else './output/record_' + stamp_str() + '/'


"""
    Signal Handler
"""
is_shutdown = mp.Event()

def sigint_handler(sig, frame):
    print('Interrupted.')
    is_shutdown.set()

signal.signal(signal.SIGINT, sigint_handler)


"""
    Main
"""
resize = tuple(fetch(config, ['world', 'camera', 'resize'], [150, 50]))

camera = ROSCameraWithResizeGrayscaleAndPatchNormalization('camera', is_shutdown,
    fetch(config, ['world', 'camera', 'image_topic'], '/car/robot/camera'),
    resize,
    fetch(config, ['world', 'camera', 'patch_size'], 5)
)
controller = KeyboardAckermannController('controller', is_shutdown)
actuator = WebotsROSAckermannActuator('actuator', is_shutdown,
    fetch(config, ['world', 'car', 'left_front_steer_motor'], '/car/left_front_steer_motor'),
    fetch(config, ['world', 'car', 'right_front_steer_motor'], '/car/right_front_steer_motor'),
    fetch(config, ['world', 'car', 'left_rear_motor'], '/car/left_rear_motor'),
    fetch(config, ['world', 'car', 'right_rear_motor'], '/car/right_rear_motor'),
    fetch(config, ['world', 'get_time_srv'], '/car/robot/get_time'),
    fetch(config, ['world', 'car', 'track'], 1.628),
    fetch(config, ['world', 'car', 'wheelbase'], 2.995),
    fetch(config, ['world', 'car', 'wheel_radius'], 0.38),
    fetch(config, ['world', 'car', 'max_steering_angle'], 0.6)
)

cable_camera_image = Cable(
    cable_type = 'shared_object',
    size = resize[0] * resize[1] + 300,
    distributees = [
        (camera, 'image')
    ]
)

cable_actuator_command = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (controller, 'actuator_command'),
        (actuator, 'command')
    ]
)

cable_odom = Cable(
#     cable_type = 'shared_object',
#     size = 300,
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (actuator, 'odom')
    ]
)

camera.start()
controller.start()
actuator.start()

rospy.init_node('webots_ros_ackermann_teacher', anonymous = False)
# from sensor_msgs.msg import Image
# bridge = CvBridge()
pub_odom = rospy.Publisher(fetch(config, ['world', 'odometry', 'odom_output_topic'], '/car/odom'), Odometry, queue_size = 1)

while not is_shutdown.is_set():
    if cable_odom.poll():
        odom = cable_odom.read()
        pub_odom.publish(t3d_ext.e2O(odom, frame_id = 'odom', stamp = rospy.Time.now()))
        if cable_camera_image.poll():
            img = cable_camera_image.read()
            # TODO: 不需要传过来，直接告知 camera 节点保存即可；或实现 RPC.

camera.join()
controller.join()
actuator.join()

