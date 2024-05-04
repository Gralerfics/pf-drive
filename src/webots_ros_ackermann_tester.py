# python src/webots_ros_ackermann_benchmark.py

import os
import time
import json
import multiprocessing as mp

import numpy as np

from matplotlib import pyplot as plt

from multinodes import Cable

from pf_drive.util import t3d_ext, stamp_str
from pf_drive.util import ROSContext
from pf_drive.actuator import WebotsROSAckermannActuatorComputer, WebotsROSAckermannActuatorCaller
# from pf_drive.controller import KeyboardAckermannController
from pf_drive.device import WebotsROSRobotGlobalLocator


"""
    Nodes
"""
locator = WebotsROSRobotGlobalLocator('locator', 'car', '/car/supervisor')
# controller = KeyboardAckermannController('controller')
actuator_computer = WebotsROSAckermannActuatorComputer('actuator_computer', '/car/robot/get_time', 1.628, 2.995, 0.38, 0.6)
actuator_caller = WebotsROSAckermannActuatorCaller('actuator_caller', '/car/left_front_steer_motor', '/car/right_front_steer_motor', '/car/left_rear_motor', '/car/right_rear_motor')

cable_locator_main_gt_pose = Cable(cable_type = 'pipe', latest = True, distributees = [(locator, 'gt_pose')])
cable_locator_main_gt_twist = Cable(cable_type = 'pipe', latest = True, distributees = [(locator, 'gt_twist')])
# cable_controller_actuator_command = Cable(cable_type = 'pipe', latest = True, distributees = [(controller, 'actuator_command'), (actuator_computer, 'command')])
cable_main_actuator_command = Cable(cable_type = 'pipe', latest = True, distributees = [(actuator_computer, 'command')])
cable_actuator_main_odom = Cable(cable_type = 'pipe', latest = True, distributees = [(actuator_computer, 'odom')])
cable_actuator_param = Cable(cable_type = 'pipe', latest = True, distributees = [(actuator_computer, 'param'), (actuator_caller, 'param')])

locator.start()
# controller.start()
actuator_computer.start()
actuator_caller.start()


"""
    Main
"""
ros = ROSContext('webots_ros_ackermann_tester')
ros.init_node(anonymous = False)

odom, gt_pose, gt_twist = None, None, None
state = 'straight'

v = 30.0
R = -20
turn_x = 30.0
dt = 0.015

straight_mode = True

cable_main_actuator_command.write(('vw', v, 0))

stamp_offset = time.time()
datas = []

# 主循环
while not ros.is_shutdown():
    if cable_locator_main_gt_pose.poll() and cable_locator_main_gt_twist.poll():
        gt_pose = cable_locator_main_gt_pose.read()
        gt_twist = cable_locator_main_gt_twist.read()
    
    if cable_actuator_main_odom.poll():
        odom = cable_actuator_main_odom.read()
        if gt_pose is not None:
            ros.publish_topic('/car/odom', t3d_ext.e2O(odom, frame_id = 'odom', stamp = ros.time()))
            ros.publish_tf(np.dot(gt_pose, t3d_ext.einv(odom)), 'map', 'odom')

    if gt_pose is not None:
        if straight_mode:
            if state == 'straight':
                data = {
                    'stamp': time.time() - stamp_offset,
                    'p': gt_pose[:2, 3].tolist(),
                    'v': gt_twist[:2].tolist(),
                    'w': gt_twist[-1]
                }
                datas.append(data)
                if gt_pose[0, 3] >= turn_x:
                    state = 'stop'
                    cable_main_actuator_command.write(('vw', 0, 0))
                time.sleep(dt)
            elif state == 'stop':
                os.makedirs('output', exist_ok = True)
                with open(f'output/target_v{v}.json', 'w') as f:
                    json.dump(datas, f)
                print('saved.')
                break
        else:
            if state == 'straight':
                if gt_pose[0, 3] >= turn_x:
                    state = 'turning'
                    cable_main_actuator_command.write(('vw', v, v / R))
                    stamp_offset = time.time()
            elif state == 'turning':
                data = {
                    'stamp': time.time() - stamp_offset,
                    'p': gt_pose[:2, 3].tolist(),
                    'v': gt_twist[:2].tolist(),
                    'w': gt_twist[-1]
                }
                datas.append(data)
                if gt_twist[0] <= 0.0:
                    state = 'stop'
                    cable_main_actuator_command.write(('vw', 0, 0))
                time.sleep(dt)
            elif state == 'stop':
                os.makedirs('output', exist_ok = True)
                with open(f'output/v{v}_R{R}.json', 'w') as f:
                    json.dump(datas, f)
                print('saved.')
                break

