import os
import time
import json
import shutil
import signal
import argparse
import multiprocessing as mp

import numpy as np

import rospy
from nav_msgs.msg import Odometry

# from cv_bridge import CvBridge

from multinodes import Cable

from pf_drive.util import t3d_ext, stamp_str, fetch
from pf_drive.actuator.webots_ros_ackermann_actuator import WebotsROSAckermannActuator
from pf_drive.controller.keyboard_ackermann_controller import KeyboardAckermannController
from pf_drive.device.ros_camera import ROSCameraForRecording
from pf_drive.device.webots_odometry import WebotsROSRobotGlobalLocator


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

rotation_threshold = fetch(config, ['rotation_threshold'], 0.12)
translation_threshold_square = fetch(config, ['translation_threshold'], 3.0) ** 2
save_raw_images = fetch(config, ['save_raw_images'], False)
save_gt_poses = fetch(config, ['save_gt_poses'], False)
save_path_overwrite = fetch(config, ['save_path_overwrite'], False)


"""
    Signal Handler
"""
is_shutdown = mp.Event()

def sigint_handler(sig, frame):
    print('Interrupted.')
    is_shutdown.set()

signal.signal(signal.SIGINT, sigint_handler)


"""
    Nodes
"""
camera = ROSCameraForRecording('camera', is_shutdown,
    fetch(config, ['world', 'camera', 'image_topic'], '/car/robot/camera'),
    tuple(fetch(config, ['world', 'camera', 'resize'], [150, 50])),
    fetch(config, ['world', 'camera', 'patch_size'], 5)
)
locator = WebotsROSRobotGlobalLocator('locator', is_shutdown,
    fetch(config, ['world', 'car', 'def'], 'car'),
    fetch(config, ['world', 'supervisor_srv'], '/car/supervisor')
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

cable_camera_image_save = Cable(
    cable_type = 'rpc',
    size = 100,
    distributees = [
        (camera, 'command')
    ]
)

cable_gt_pose = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (locator, 'gt_pose')
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
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (actuator, 'odom')
    ]
)

camera.start()
locator.start()
controller.start()
actuator.start()


"""
    Main
"""
# ROS
rospy.init_node('webots_ros_ackermann_teacher', anonymous = False)
pub_odom = rospy.Publisher(fetch(config, ['world', 'odometry', 'odom_output_topic'], '/car/odom'), Odometry, queue_size = 1)

# 文件目录
if save_path_overwrite:
    assert os.path.dirname(config['save_path']) != '/' # 保险起见
    shutil.rmtree(config['save_path'], ignore_errors = True)
if save_raw_images:
    raw_image_folder = os.path.join(config['save_path'], 'raw_image/')
    os.makedirs(raw_image_folder, exist_ok = True)
proc_image_folder = os.path.join(config['save_path'], 'processed_image/')
os.makedirs(proc_image_folder, exist_ok = True)
odom_folder = os.path.join(config['save_path'], 'odom/')
os.makedirs(odom_folder, exist_ok = True)
if save_gt_poses:
    save_gt_pose_folder = os.path.join(config['save_path'], 'gt_pose/')
    os.makedirs(save_gt_pose_folder, exist_ok = True)

# 主循环
odom = None
last_odom = None
idx = 0
while not is_shutdown.is_set():
    if cable_odom.poll() and cable_gt_pose.poll():
        odom = cable_odom.read()
        gt_pose = cable_gt_pose.read()
        pub_odom.publish(t3d_ext.e2O(odom, frame_id = 'odom', stamp = rospy.Time.now())) # only for rviz
        
        # 起点或超过阈值
        flag = False
        if last_odom is None:
            flag = True
        else:
            delta_odom = np.dot(t3d_ext.einv(last_odom), odom)
            t = t3d_ext.edt(delta_odom)
            if np.dot(t, t) > translation_threshold_square or t3d_ext.R2yaw(t3d_ext.edR(delta_odom)) > rotation_threshold:
                flag = True
        
        if flag:
            cable_camera_image_save.write(('save_image', {
                'raw_img_path': os.path.join(raw_image_folder, str(idx) + '.png') if save_raw_images else None,
                'proc_img_path': os.path.join(proc_image_folder, str(idx) + '.png')
            }))
            with open(os.path.join(odom_folder, str(idx) + '.json'), 'w') as f:
                json.dump(odom.tolist(), f)
            if save_gt_poses:
                with open(os.path.join(save_gt_pose_folder, str(idx) + '.json'), 'w') as f:
                    json.dump(gt_pose.tolist(), f)
            rospy.loginfo('Goal %d saved.' % idx)
            last_odom = odom
            idx += 1


"""
    Wait for Termination
"""
camera.join()
locator.join()
controller.join()
actuator.join()

