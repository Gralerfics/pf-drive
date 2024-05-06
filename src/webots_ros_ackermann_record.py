# python src/webots_ros_ackermann_record.py --config ./config/webots_ros_ackermann_record.json

import os
import time
import json
import shutil
import argparse
import multiprocessing as mp

import numpy as np

from multinodes import Cable

from pf_drive.util import ROSContext, t3d_ext, stamp_str, fetch
from pf_drive.actuator import WebotsROSAckermannActuatorComputer, WebotsROSAckermannActuatorCaller
from pf_drive.controller import KeyboardAckermannController
from pf_drive.device import ROSCameraWithProcessingAndSaving
from pf_drive.device import WebotsROSRobotGlobalLocator


"""
    Arguments and Configurations
"""
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config', type = str, default = './config/record.json')
parser.add_argument('-o', '--output', type = str, default = None)
args = parser.parse_args()

config_path = args.config
config = json.load(open(config_path, 'r'))
if args.output is not None:
    config['save_path'] = args.output
elif 'save_path' not in config:
    config['save_path'] = './output/record_' + stamp_str() + '/'

resize = tuple(fetch(config, ['world', 'camera', 'resize'], [150, 50]))
patch_size = fetch(config, ['world', 'camera', 'patch_size'], 5)
horizontal_fov = fetch(config, ['world', 'camera', 'horizontal_fov'], 44.97)

rotation_threshold = fetch(config, ['rotation_threshold'], 0.12)
translation_threshold = fetch(config, ['translation_threshold'], 3.0)

save_raw_images = fetch(config, ['save_raw_images'], False)
save_gt_poses = fetch(config, ['save_gt_poses'], False)
save_path_overwrite = fetch(config, ['save_path_overwrite'], False)


"""
    Nodes
"""
camera = ROSCameraWithProcessingAndSaving('camera',
    fetch(config, ['world', 'camera', 'image_topic'], '/car/camera/image'),
    resize,
    patch_size
)
locator = WebotsROSRobotGlobalLocator('locator',
    fetch(config, ['world', 'car', 'def'], 'car'),
    fetch(config, ['world', 'supervisor_srv'], '/car/supervisor')
)
controller = KeyboardAckermannController('controller')
actuator_computer = WebotsROSAckermannActuatorComputer('actuator_computer',
    fetch(config, ['world', 'get_time_srv'], '/car/robot/get_time'),
    fetch(config, ['world', 'car', 'track'], 1.628),
    fetch(config, ['world', 'car', 'wheelbase'], 2.995),
    fetch(config, ['world', 'car', 'wheel_radius'], 0.38),
    fetch(config, ['world', 'car', 'max_steering_angle'], 0.6),
    '/car/left_rear_position_sensor/value',
    '/car/right_rear_position_sensor/value'
)
actuator_caller = WebotsROSAckermannActuatorCaller('actuator_caller',
    fetch(config, ['world', 'car', 'left_front_steer_motor'], '/car/left_front_steer_motor'),
    fetch(config, ['world', 'car', 'right_front_steer_motor'], '/car/right_front_steer_motor'),
    fetch(config, ['world', 'car', 'left_rear_motor'], '/car/left_rear_motor'),
    fetch(config, ['world', 'car', 'right_rear_motor'], '/car/right_rear_motor')
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
        (actuator_computer, 'command')
    ]
)

cable_odom = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (actuator_computer, 'odom')
    ]
)

cable_actuator_param = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (actuator_computer, 'param'),
        (actuator_caller, 'param')
    ]
)

camera.start()
locator.start()
controller.start()
actuator_computer.start()
actuator_caller.start()


"""
    Main
"""
# ROS
ros = ROSContext('webots_ros_ackermann_record')
ros.init_node(anonymous = False)
odom_topic = fetch(config, ['world', 'odometry', 'odom_output_topic'], '/car/odom')

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

# 录制参数
with open(os.path.join(config['save_path'], 'parameters.json'), 'w') as f:
    json.dump({
        "image": {
            "resize": resize,
            "patch_size": patch_size,
            "horizontal_fov": horizontal_fov
        },
        "rotation_threshold": rotation_threshold,
        "translation_threshold": translation_threshold,
    }, f)

# 主循环
translation_threshold_square = translation_threshold ** 2
odom, last_odom = None, None
idx = 0
while not ros.is_shutdown():
    if cable_odom.poll() and cable_gt_pose.poll():
        odom = cable_odom.read()
        gt_pose = cable_gt_pose.read()

        ros.publish_topic(odom_topic, t3d_ext.e2O(odom, frame_id = 'odom', stamp = ros.time())) # only for rviz
        T_map_odom = np.dot(gt_pose, t3d_ext.einv(odom))
        ros.publish_tf(T_map_odom, 'map', 'odom')

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
            ros.loginfo('Goal %d saved.' % idx)
            last_odom = odom
            idx += 1

