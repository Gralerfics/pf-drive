# python src/webots_ros_ackermann_repeat.py --config ./config/webots_ros_ackermann_repeat.json --record /home/gralerfics/MyFiles/Workspace/pf_data/car_2

import os
import time
import json
import shutil
import argparse
import multiprocessing as mp

import numpy as np

from multinodes import Cable

from pf_drive.util import t3d_ext, fetch, get_numbered_file_list
from pf_drive.util import ROSContext
from pf_drive.actuator import WebotsROSAckermannActuatorComputer, WebotsROSAckermannActuatorCaller
from pf_drive.controller import BaselineRepeatController
from pf_drive.device import ROSCameraWithProcessingAndSending
from pf_drive.device import WebotsROSRobotGlobalLocator
from pf_drive.storage import RecordLoaderQueued


"""
    Arguments and Configurations
"""
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config', type = str, default = './config/repeat.json')
parser.add_argument('-r', '--record', type = str, default = None) # config['load_path'] 优先
args = parser.parse_args()

# 配置文件参数
config_path = args.config
config = json.load(open(config_path, 'r'))
if 'load_path' not in config:
    if args.record is None:
        raise ValueError('No record path specified.') # TODO
    config['load_path'] = args.record

# record 参数
with open(os.path.join(config['load_path'], 'parameters.json'), 'r') as f:
    parameters = json.load(f)
    config['world']['camera']['resize'] = parameters['image']['resize']
    config['world']['camera']['patch_size'] = parameters['image']['patch_size']
    config['world']['camera']['horizontal_fov'] = parameters['image']['horizontal_fov']
    # config['record'] = {
    #     'rotation_threshold': parameters['rotation_threshold'],
    #     'translation_threshold': parameters['translation_threshold']
    # }

# 常用参数
load_path = config['load_path']
resize = tuple(fetch(config, ['world', 'camera', 'resize'], [150, 50]))
patch_size = fetch(config, ['world', 'camera', 'patch_size'], 5)
horizontal_fov = fetch(config, ['world', 'camera', 'horizontal_fov'], 44.97)

track = fetch(config, ['world', 'car', 'track'], 1.628)
wheelbase = fetch(config, ['world', 'car', 'wheelbase'], 2.995)
wheel_radius = fetch(config, ['world', 'car', 'wheel_radius'], 0.38)
max_steering_angle = fetch(config, ['world', 'car', 'max_steering_angle'], 0.6)
R_min_abs = wheelbase / np.tan(max_steering_angle) + track / 2


"""
    Nodes
"""
camera = ROSCameraWithProcessingAndSending('camera',
    fetch(config, ['world', 'camera', 'image_topic'], '/car/camera/image'),
    resize,
    patch_size
)
locator = WebotsROSRobotGlobalLocator('locator',
    fetch(config, ['world', 'car', 'def'], 'car'),
    fetch(config, ['world', 'supervisor_srv'], '/car/supervisor')
)
loader = RecordLoaderQueued('loader',
    load_path
)
controller = BaselineRepeatController('controller',
    horizontal_fov = horizontal_fov,
    match_offset_radius = fetch(config, ['match_offset_radius'], 90),
    along_path_radius = fetch(config, ['along_path_radius'], 2),
    predict_number = fetch(config, ['predict_number'], 5),
    k_rotation = fetch(config, ['k_rotation'], 0.06),
    k_along_path = fetch(config, ['k_along_path'], 0.1),
    distance_threshold = fetch(config, ['distance_threshold'], 0.2),
    angle_threshold = fetch(config, ['angle_threshold'], 0.1),
    R_min_abs = R_min_abs
)
actuator_computer = WebotsROSAckermannActuatorComputer('actuator_computer',
    fetch(config, ['world', 'get_time_srv'], '/car/robot/get_time'),
    fetch(config, ['world', 'car', 'track'], 1.628),
    fetch(config, ['world', 'car', 'wheelbase'], 2.995),
    fetch(config, ['world', 'car', 'wheel_radius'], 0.38),
    fetch(config, ['world', 'car', 'max_steering_angle'], 0.6)
)
actuator_caller = WebotsROSAckermannActuatorCaller('actuator_caller',
    fetch(config, ['world', 'car', 'left_front_steer_motor'], '/car/left_front_steer_motor'),
    fetch(config, ['world', 'car', 'right_front_steer_motor'], '/car/right_front_steer_motor'),
    fetch(config, ['world', 'car', 'left_rear_motor'], '/car/left_rear_motor'),
    fetch(config, ['world', 'car', 'right_rear_motor'], '/car/right_rear_motor')
)

cable_camera_ctrl_image = Cable(
    cable_type = 'shared_object',
    size = resize[0] * resize[1] + 300,
    distributees = [
        (camera, 'image'),
        (controller, 'processed_image')
    ]
)

cable_locator_main_gt = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (locator, 'gt_pose')
    ]
)

cable_loader_ctrl_record = Cable(
    cable_type = 'queue',
    size = 5,
    distributees = [
        (loader, 'output'),
        (controller, 'record')
    ]
)

cable_ctrl_actuator_command = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (controller, 'actuator_command'),
        (actuator_computer, 'command')
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

cable_actuator_main_odom = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (actuator_computer, 'odom')
    ]
)

cable_main_ctrl_odom = Cable(
    cable_type = 'pipe',
    latest = True,
    distributees = [
        (controller, 'odom')
    ]
)

camera.start()
locator.start()
loader.start()
controller.start()
actuator_computer.start()
actuator_caller.start()


"""
    Main
"""
# ROS
ros = ROSContext('webots_ros_ackermann_recorder')
ros.init_node(anonymous = False)
odom_topic = fetch(config, ['world', 'odometry', 'odom_output_topic'], '/car/odom')

# 发布 /recorded_gts, /recorded_odoms
odom_load_path = os.path.join(load_path, 'odom')
gt_pose_load_path = os.path.join(load_path, 'gt_pose')
record_odoms = []
record_gts = []
for filename in get_numbered_file_list(odom_load_path):
    with open(os.path.join(odom_load_path, filename), 'r') as f:
        record_odoms.append(np.array(json.load(f)))
for filename in get_numbered_file_list(gt_pose_load_path):
    with open(os.path.join(gt_pose_load_path, filename), 'r') as f:
        record_gts.append(np.array(json.load(f)))
ros.publish_topic('/recorded_odoms', t3d_ext.es2P(record_odoms, frame_id = 'odom'))
ros.publish_topic('/recorded_gts', t3d_ext.es2P(record_gts, frame_id = 'map'))

# 主循环
odom, last_odom = None, None
idx = 0
try:
    while not ros.is_shutdown():
        if cable_actuator_main_odom.poll() and cable_locator_main_gt.poll():
            odom = cable_actuator_main_odom.read() # odom: actuator_computer -> main
            cable_main_ctrl_odom.write(odom) # odom: main -> controller
            gt_pose = cable_locator_main_gt.read() # gt_pose: locator -> main

            # 发布 odom 与坐标变换
            ros.publish_topic(odom_topic, t3d_ext.e2O(odom, frame_id = 'odom', stamp = ros.time())) # only for rviz
            T_map_odom = np.dot(gt_pose, t3d_ext.einv(odom))
            ros.publish_tf(T_map_odom, 'map', 'odom')
finally:
    # TODO: 貌似没用
    cable_ctrl_actuator_command.write(('vw', 0, 0))


# camera.join()
# locator.join()
# loader.join()
# controller.join()
# actuator_computer.join()
# actuator_caller.join()

