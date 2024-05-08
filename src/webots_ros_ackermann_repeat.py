# python src/webots_ros_ackermann_repeat.py --config ./config/webots_ros_ackermann_repeat.json --record /home/gralerfics/MyFiles/Workspace/pf_data/car_2
# [--report]

import os
import time
import json
import shutil
import argparse
import multiprocessing as mp

import numpy as np

from multinodes import Cable

from pf_drive.util import ROSContext, t3d_ext, fetch, get_numbered_file_list
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
parser.add_argument('-r', '--record', type = str, default = None)
parser.add_argument('-R', '--report', type = str, default = None)
args = parser.parse_args()

# 配置文件参数
config_path = args.config
config = json.load(open(config_path, 'r'))

if args.record is not None:
    config['load_path'] = args.record
elif 'load_path' not in config:
    raise ValueError('No record path specified.')
load_path = config['load_path']

if args.report is not None:
    config['report_path'] = args.report
elif 'report_path' not in config:
    config['report_path'] = (load_path if load_path[-1] != '/' else load_path[:-1]) + '_report'
report_path = config['report_path']

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
resize = tuple(fetch(config, ['world', 'camera', 'resize'], [150, 50]))
patch_size = fetch(config, ['world', 'camera', 'patch_size'], 5)
horizontal_fov = fetch(config, ['world', 'camera', 'horizontal_fov'], 44.97)

track = fetch(config, ['world', 'car', 'track'], 1.628)
wheelbase = fetch(config, ['world', 'car', 'wheelbase'], 2.995)
wheel_radius = fetch(config, ['world', 'car', 'wheel_radius'], 0.38)
max_steering_angle = fetch(config, ['world', 'car', 'max_steering_angle'], 0.6)
friction_factor = fetch(config, ['world', 'car', 'friction_factor'], 15)

along_path_radius = fetch(config, ['along_path_radius'], 2)
steering_predict_goals = fetch(config, ['steering_predict_goals'], 5)
steering_weights = fetch(config, ['steering_weights'], [0.0, 1.0, 0.6, 0.1, 0.05, 0.03, 0.02, 0.01])
slowing_predict_goals = fetch(config, ['slowing_predict_goals'], 15)
correction_distance_interval = fetch(config, ['correction_distance_interval'], 3.0)
k_rotation = fetch(config, ['k_rotation'], 0.03)
k_along_path = fetch(config, ['k_along_path'], 0.01)
odom_compensation_rotation_update_rate = fetch(config, ['odom_compensation_rotation_update_rate'], 0.0)
odom_compensation_translation_update_rate = fetch(config, ['odom_compensation_translation_update_rate'], 0.2)
odom_compensation_rotation_threshold = fetch(config, ['odom_compensation_rotation_threshold'], 0.15)
odom_compensation_translation_threshold = fetch(config, ['odom_compensation_translation_threshold'], 0.1)
distance_threshold = fetch(config, ['distance_threshold'], 0.2)
reference_velocity = fetch(config, ['reference_velocity'], 10.0)


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
    along_path_radius = along_path_radius,
    steering_predict_goals = steering_predict_goals,
    steering_weights = steering_weights,
    slowing_predict_goals = slowing_predict_goals,
    correction_distance_interval = correction_distance_interval,
    k_rotation = k_rotation,
    k_along_path = k_along_path,
    odom_compensation_rotation_update_rate = odom_compensation_rotation_update_rate,
    odom_compensation_translation_update_rate = odom_compensation_translation_update_rate,
    odom_compensation_rotation_threshold = odom_compensation_rotation_threshold,
    odom_compensation_translation_threshold = odom_compensation_translation_threshold,
    track = track,
    wheelbase = wheelbase,
    wheel_radius = wheel_radius,
    max_steering_angle = max_steering_angle,
    friction_factor = friction_factor,
    distance_threshold = distance_threshold,
    reference_velocity = reference_velocity,
    along_path_debug_image_topic = '/debug_img',
    local_raw_path_debug_topic = '/recorded_odoms',
    odom_a_debug_topic = '/a',
    odom_b_debug_topic = '/b',
    odom_r_debug_topic = '/r'
)
actuator_computer = WebotsROSAckermannActuatorComputer('actuator_computer',
    fetch(config, ['world', 'get_time_srv'], '/car/robot/get_time'),
    track,
    wheelbase,
    wheel_radius,
    max_steering_angle,
    '/car/left_rear_position_sensor/value',
    '/car/right_rear_position_sensor/value'
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

cable_ctrl_main_passed_goal = Cable(
    cable_type = 'queue',
    size = 5,
    distributees = [
        (controller, 'passed_goal')
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
ros = ROSContext('webots_ros_ackermann_repeat')
ros.init_node(anonymous = False)
odom_topic = fetch(config, ['world', 'odometry', 'odom_output_topic'], '/car/odom')

# 读取路线
odom_load_path = os.path.join(load_path, 'odom')
gt_pose_load_path = os.path.join(load_path, 'gt_pose')
record_odoms = []
record_gt_poses = []
for filename in get_numbered_file_list(odom_load_path):
    with open(os.path.join(odom_load_path, filename), 'r') as f:
        record_odoms.append(np.array(json.load(f)))
for filename in get_numbered_file_list(gt_pose_load_path):
    with open(os.path.join(gt_pose_load_path, filename), 'r') as f:
        record_gt_poses.append(np.array(json.load(f)))
record_odom_path = t3d_ext.es2P(record_odoms, frame_id = 'odom')
record_gt_pose_path = t3d_ext.es2P(record_gt_poses, frame_id = 'map')

# 主循环
report_gt_poses = []
odom, last_odom = None, None
idx = 0
while not ros.is_shutdown():
    if cable_actuator_main_odom.poll() and cable_locator_main_gt.poll():
        odom = cable_actuator_main_odom.read() # odom: actuator_computer -> main
        cable_main_ctrl_odom.write(odom) # odom: main -> controller
        gt_pose = cable_locator_main_gt.read() # gt_pose: locator -> main

        # goal passed
        if cable_ctrl_main_passed_goal.poll():
            idx = cable_ctrl_main_passed_goal.read() # passed_goal: main -> controller
            if idx is None:
                # 停车
                cable_ctrl_actuator_command.write(('vw', 0, 0))
                time.sleep(1.0)

                # 保存 report
                os.makedirs(report_path, exist_ok = True)
                with open(os.path.join(report_path, 'record_traj.txt'), 'w') as f:
                    for gt_pose in record_gt_poses:
                        f.write(t3d_ext.e2kitti(gt_pose) + '\n')
                with open(os.path.join(report_path, 'repeat_traj.txt'), 'w') as f:
                    for gt_pose in report_gt_poses:
                        f.write(t3d_ext.e2kitti(gt_pose) + '\n')
                with open(os.path.join(report_path, 'parameters.json'), 'w') as f:
                    json.dump({
                        'car': {
                            'track': track,
                            'wheelbase': wheelbase,
                            'wheel_radius': wheel_radius,
                            'max_steering_angle': max_steering_angle,
                            'friction_factor': friction_factor
                        },
                        'along_path_radius': along_path_radius,
                        'steering_predict_goals': steering_predict_goals,
                        'steering_weights': steering_weights,
                        'slowing_predict_goals': slowing_predict_goals,
                        'k_rotation': k_rotation,
                        'k_along_path': k_along_path,
                        'odom_compensation_rotation_update_rate': odom_compensation_rotation_update_rate,
                        'odom_compensation_translation_update_rate': odom_compensation_translation_update_rate,
                        'odom_compensation_rotation_threshold': odom_compensation_rotation_threshold,
                        'odom_compensation_translation_threshold': odom_compensation_translation_threshold,
                        'distance_threshold': distance_threshold,
                        'reference_velocity': reference_velocity
                    }, f)
                break
            else:
                # 调试话题
                ros.publish_topic('/a_gt', t3d_ext.e2PS(record_gt_poses[idx], frame_id = 'map'))
                if idx + 1 < len(record_gt_poses):
                    ros.publish_topic('/b_gt', t3d_ext.e2PS(record_gt_poses[idx + 1], frame_id = 'map'))

                # 保存 repeat 过程的 gt_pose
                report_gt_poses.append(gt_pose)

        # 发布 odom 与坐标变换
        ros.publish_topic(odom_topic, t3d_ext.e2O(odom, frame_id = 'odom', stamp = ros.time())) # only for rviz
        T_map_odom = np.dot(gt_pose, t3d_ext.einv(odom))
        ros.publish_tf(T_map_odom, 'map', 'odom')
        
        # 路径调试话题
        # ros.publish_topic('/recorded_odoms', record_odom_path)
        ros.publish_topic('/recorded_gts', record_gt_pose_path)

