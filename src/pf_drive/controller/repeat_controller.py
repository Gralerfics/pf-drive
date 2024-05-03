import time

import numpy as np

import cv2
from cv_bridge import CvBridge

import transforms3d as t3d

from multinodes import Node

from pf_drive.util import t3d_ext, ROSContext, ListQueue
from pf_drive.util import NCC_horizontal_match


"""
    `processed_image`, input (shared_object)
        format: cv2 image (np.array)
    `odom`, input (pipe)
        format: 4x4 np.array
    `record`, input (queue)
        format: (image, odom)
    `actuator_command`, output (pipe)
        format:
            (type = 'vw', v, w)
            (type = 'vphi', v, phi)
"""
class BaselineRepeatController(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name)
        
        # 参数
        self.horizontal_fov = kwargs['horizontal_fov']
        self.match_offset_radius = kwargs['match_offset_radius'] # 暂时没用到
        self.along_path_radius = kwargs['along_path_radius'] # r
        self.predict_number = kwargs['predict_number'] # p
        self.k_rotation = kwargs['k_rotation']
        self.k_along_path = kwargs['k_along_path']
        self.distance_threshold = kwargs['distance_threshold']
        self.angle_threshold = kwargs['angle_threshold']
        self.R_min_abs = kwargs['R_min_abs']

        self.max_rp = max(self.along_path_radius, self.predict_number)

        # 滑动窗口队列
        self.q_r = self.along_path_radius # r (fixed, 相对滑动窗口队列)
        self.q_size = self.along_path_radius + 1 + self.max_rp
        """
            0   1   2   3   4   5   6   7   8   9
            x   x   D0  D1  D2  D3  D4  D5  D6  D7
           |- r -|  ^  |- r -|
                    r  |----------- p -----------|
        """
        self.q = ListQueue(size = self.q_size) # (image, odom)

        # 运行时
        self.goal_distances = [0.0] # distances between goal 0 and i
        self.goal_idx = -self.max_rp - 1 # goal just passed
        self.T_0_odomA = None
        self.T_0_odomB = None
    
    def pass_to_next_goal(self):
        # loader 读完后不断发送 None, 此处不断入队, 直到 q_passed_idx + 1 项也为 None 时即结束
        self.q.push(self.io['record'].read(block = True))

        # 更新 passed_goal
        self.goal_idx += 1

        # rec_A, rec_B 有效则更新 T_0_odomA, T_0_odomB
        if self.q[self.q_r] is not None and self.q[self.q_r + 1] is not None:
            # 若 T_0_odomB 无值则赋当前 odom 值
            if self.T_0_odomB is None:
                self.T_0_odomB = self.io['odom'].read(block = True)
            
            self.T_0_odomA = self.T_0_odomB
            T_odomA_odomB = t3d_ext.einv(self.q[self.q_r][1]) @ self.q[self.q_r + 1][1]
                # T_odomA_odomB = T_{rec_r}_{rec_(r+1)} = inv(T_0_{rec_r}) * T_0_{rec_(r+1)}
            self.T_0_odomB = self.T_0_odomA @ T_odomA_odomB

        # 最新入队两个元素有效则更新 goal_distances
        if self.q[-2] is not None and self.q[-1] is not None:
            t_delta = self.q[-1][1] - self.q[-2][1]
            self.goal_distances.append(self.goal_distances[-1] + t3d_ext.norm(t3d_ext.edt(t_delta)))
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)
        bridge = CvBridge()

        # 检查接口
        while not ros.is_shutdown():
            if 'processed_image' not in self.io or 'odom' not in self.io or 'actuator_command' not in self.io or 'record' not in self.io:
                time.sleep(0.1)
                continue
            break

        # 简写
        r = self.along_path_radius
        p = self.predict_number

        # 凑满 q_size 个数据, 初始皆为 None, 从 passed_idx 为 -self.max_rp - 1 开始逐个入队直到 0
        self.q.q = [None] * self.q_size
        while not self.goal_idx == 0:
            self.pass_to_next_goal()

        # 主循环
        last_time = time.time()
        operation_num = 0
        while not ros.is_shutdown():
            if time.time() - last_time > 2.0:
                print('fps:', operation_num / (time.time() - last_time))
                operation_num = 0
                last_time = time.time()

            # 结束
            if self.q[r] is not None and self.q[r + 1] is None:
                print('Finished.')
                self.io['actuator_command'].write(('vw', 0, 0))
                # TODO: 保存 report
                break
            
            # 运算
            if self.io['processed_image'].poll() and self.io['odom'].poll():
                image = self.io['processed_image'].read()
                T_0_odomR = self.io['odom'].read() # odom, R: robot

                i = self.goal_idx # q_idx - r + i = goal_idx

                T_odomR_odomB = t3d_ext.einv(T_0_odomR) @ self.T_0_odomB

                t_0_odomA = t3d_ext.edt(self.T_0_odomA)
                t_0_odomB = t3d_ext.edt(self.T_0_odomB)
                t_0_odomR = t3d_ext.edt(T_0_odomR)

                t_odomA_odomB = t_0_odomB - t_0_odomA
                t_odomA_odomR = t_0_odomR - t_0_odomA
                t_odomR_odomB = t3d_ext.edt(T_odomR_odomB)

                l_odomA_odomB = t3d_ext.norm(t_odomA_odomB)
                l_odomR_odomB = t3d_ext.norm(t_odomR_odomB)

                turning_goal = l_odomA_odomB < self.distance_threshold
                if not turning_goal:
                    l_proj_odomA_odomR = np.dot(t_odomA_odomR, t_odomA_odomB) / l_odomA_odomB
                    u = l_proj_odomA_odomR / l_odomA_odomB # not turning_goal

                    # along-path correction
                    scan_q_indices = [q_idx for q_idx in range(2 * r + 1) if self.q[q_idx] is not None]
                    scan_q_indices = [scan_q_indices[0]] * (scan_q_indices[0]) + scan_q_indices
                    scan_q_indices = scan_q_indices + [scan_q_indices[-1]] * (2 * r - scan_q_indices[-1])
                    scan_distances = np.array([self.goal_distances[q_idx - r + i] for q_idx in scan_q_indices]) - self.goal_distances[i] - l_proj_odomA_odomR
                    scan_offsets, scan_values = np.zeros((2, len(scan_q_indices)))
                    for k, q_idx in enumerate(scan_q_indices):
                        if q_idx == r:
                            k_r = k
                        img_ref = self.q[q_idx][0]
                        scan_offsets[k], scan_values[k] = NCC_horizontal_match(image, img_ref)
                    scan_values[scan_values < min(0.1, scan_values.min() / 2)] = 0 # TODO, threshold 当前随意设置, 理应表示 NCC 底噪; 如果全都被滤除说明两图差距已经很大, 也许可以作为确认丢失的一种条件; 最小值除二仅为防止崩溃, 无实际意义.
                    delta_p = scan_values / scan_values.sum() @ scan_distances
                    delta_distance = self.k_along_path * delta_p
                    along_path_correction = (l_odomR_odomB - delta_distance) / l_odomR_odomB # TODO: divide 0
                    # TODO: k * dt

                    # print(i)
                    # print(scan_q_indices)
                    # print(scan_values)
                    # print(scan_distances)
                    # print(delta_p)
                    # print(along_path_correction)
                    # print('\n')

                    if u > 1.0 - 1e-2 or l_odomR_odomB < self.distance_threshold:
                        self.pass_to_next_goal()
                        continue

                    # rotation correction
                    theta_A = scan_offsets[k_r] / image.shape[1] * self.horizontal_fov
                    theta_B = scan_offsets[k_r + 1] / image.shape[1] * self.horizontal_fov
                    theta_R = (1 - u) * theta_A + u * theta_B
                    rotation_correction = -self.k_rotation * theta_R
                    # TODO: k * dt

                    print('\n')
                    print('i', i)
                    print('theta_A', theta_A)
                    print('theta_B', theta_B)
                    print('theta_R', theta_R)
                    print('rotation_correction', rotation_correction)

                    # 修正 T_0_odomB
                    correction_offset = t3d_ext.etR([0, 0, 0], t3d.euler.euler2mat(0, 0, rotation_correction)) @ T_odomR_odomB
                    correction_offset[:3, 3] *= along_path_correction
                    self.T_0_odomB = T_0_odomR @ correction_offset
                
                # TODO: 记录 report 数据
                pass
                
                # TODO: delta distance 判断 (required?)
                T_odomR_odomB = t3d_ext.einv(T_0_odomR) @ self.T_0_odomB # 经过校正, 与 pass_to_next_goal 中的 T_odomA_odomB 不同
                
                # 发布调试话题
                ros.publish_topic('/a', t3d_ext.e2PS(self.T_0_odomA, frame_id = 'odom'))
                ros.publish_topic('/b', t3d_ext.e2PS(self.T_0_odomB, frame_id = 'odom'))
                ros.publish_topic('/r', t3d_ext.e2PS(T_0_odomR, frame_id = 'odom'))
                
                # TODO: actuator_command
                # v = 0.1
                v = 7.2

                T_0_qN = [T for T in self.q.q[r:(r + 3)] if T is not None][-1][1]
                T_0_qB = self.q[r + 1][1]
                T_0_odomN = self.T_0_odomB @ t3d_ext.einv(T_0_qB) @ T_0_qN
                T_odomR_odomN = t3d_ext.einv(T_0_odomR) @ T_0_odomN

                dy = T_odomR_odomN[1, 3]
                dx = T_odomR_odomN[0, 3]
                print(T_odomR_odomN[:3, 3])
                ros.publish_topic('/goal', t3d_ext.e2PS(T_0_odomN, frame_id = 'odom'))

                if abs(dy) < 1e-2:
                    w = 0.0
                else:
                    d_square = dx ** 2 + dy ** 2
                    R = d_square / 2 / dy
                    if abs(R) < self.R_min_abs:
                        R = np.sign(R) * self.R_min_abs
                    w = v / R

                self.io['actuator_command'].write(('vw', v, w))
                operation_num += 1
                
                print('vw', v, w)
                # time.sleep(5.0)

