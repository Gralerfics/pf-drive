import time

import numpy as np

import transforms3d as t3d

from multinodes import Node

from pf_drive.util import t3d_ext, ROSContext, ListQueue
from pf_drive.util.img import NCC_horizontal_match, np_to_Image


"""
    `processed_image`, input (shared_object)
        format: cv2 image (np.array)
    `odom`, input (pipe)
        format: 4x4 np.array
    `record`, input (queue)
        format: (image, odom)
    `passed_goal`, output (queue)
        format: int
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
        self.along_path_radius = kwargs['along_path_radius'] # r
        self.predict_number = kwargs['predict_number'] # p
        self.k_rotation = kwargs['k_rotation']
        self.k_along_path = kwargs['k_along_path']
        self.distance_threshold = kwargs['distance_threshold']
        self.R_min_abs = kwargs['R_min_abs']
        self.reference_velocity = kwargs['reference_velocity']

        # 滑动窗口队列
        self.max_rp = max(self.along_path_radius, self.predict_number)
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

        # 更新 passed_goal 并输出到端口
        self.goal_idx += 1
        if self.goal_idx >= 0:
            self.io['passed_goal'].write(self.goal_idx)

        # rec_A, rec_B 有效则更新 T_0_odomA, T_0_odomB
        if self.q[self.q_r] is not None and self.q[self.q_r + 1] is not None:
            # 若 T_0_odomB 无值则赋当前 odom 值
            if self.T_0_odomB is None:
                self.T_0_odomB = self.io['odom'].read(block = True)
            
            self.T_0_odomA = self.T_0_odomB
            T_odomA_odomB = t3d_ext.einv(self.q[self.q_r][1]) @ self.q[self.q_r + 1][1] # T_odomA_odomB = T_{rec_r}_{rec_(r+1)} = inv(T_0_{rec_r}) * T_0_{rec_(r+1)}
            self.T_0_odomB = self.T_0_odomA @ T_odomA_odomB

        # 最新入队两个元素有效则更新 goal_distances
        if self.q[-2] is not None and self.q[-1] is not None:
            t_delta = self.q[-1][1] - self.q[-2][1]
            self.goal_distances.append(self.goal_distances[-1] + t3d_ext.norm(t3d_ext.edt(t_delta)))
    
    def run(self):
        ros = ROSContext(self.name)
        ros.init_node(anonymous = False)

        # 检查接口
        while not ros.is_shutdown():
            if 'processed_image' not in self.io or 'odom' not in self.io or 'actuator_command' not in self.io or 'passed_goal' not in self.io or 'record' not in self.io:
                time.sleep(0.1)
                continue
            break

        # 简写
        r = self.along_path_radius
        p = self.max_rp

        # 凑满 q_size 个数据, 初始皆为 None, 从 passed_idx 为 -self.max_rp - 1 开始逐个入队直到 0
        self.q.q = [None] * self.q_size
        while not self.goal_idx == 0:
            self.pass_to_next_goal()

        # 主循环
        timer_fps = timer_P = time.time()
        operation_num = 0
        while not ros.is_shutdown():
            current_time = time.time()
            if current_time - timer_fps > 2.0:
                print('fps:', operation_num / (time.time() - timer_fps))
                operation_num = 0
                timer_fps = current_time

            # 结束
            if self.q[r] is not None and self.q[r + 1] is None:
                print('Finished.')
                self.io['actuator_command'].write(('vw', 0, 0)) # 停车
                self.io['passed_goal'].write(None) # 结束信号
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

                    dt = current_time - timer_P
                    timer_P = current_time

                    # along-path correction
                    scan_q_indices = [q_idx for q_idx in range(2 * r + 1) if self.q[q_idx] is not None]
                    scan_q_indices = [scan_q_indices[0]] * (scan_q_indices[0]) + scan_q_indices
                    scan_q_indices = scan_q_indices + [scan_q_indices[-1]] * (2 * r - scan_q_indices[-1])
                    scan_distances = np.array([self.goal_distances[q_idx - r + i] for q_idx in scan_q_indices]) - self.goal_distances[i] - l_proj_odomA_odomR
                    scan_offsets, scan_values = np.zeros((2, len(scan_q_indices)))

                    debug_img = None # [debug]
                    dash_img = np.zeros_like(image)[:5, :] # [debug]

                    for k, q_idx in enumerate(scan_q_indices):
                        if q_idx == r:
                            k_r = k
                        img_ref = self.q[q_idx][0]
                        scan_offsets[k], scan_values[k] = NCC_horizontal_match(image, img_ref)

                        if debug_img is None: # [debug]
                            debug_img = img_ref.copy()
                        else:
                            debug_img = np.concatenate((debug_img, dash_img, img_ref), axis = 0)
                            if q_idx == r:
                                debug_img = np.concatenate((debug_img, dash_img, image), axis = 0)

                    # scan_values[abs(np.arange(len(scan_values)) - np.argmax(scan_values)) > 1] = 0 # [Approach 1] 保留最大值附近的值
                    
                    scan_values[scan_values < scan_values[scan_values != scan_values.max()].max()] = 0 # [Approach 2] 低于次高值的全部置零
                    
                    # scan_values[scan_values < min(0.1, scan_values.min() * 0.8)] = 0 # [Approach 3] 随意的设置, 理应表示 NCC 底噪; 如果全都被滤除说明两图差距已经很大, 也许可以作为确认丢失的一种条件; 最小值 * 0.8 仅为防止崩溃, 无实际意义.
                    
                    delta_p_distance = scan_values / scan_values.sum() @ scan_distances
                    along_path_correction = (l_odomR_odomB - self.k_along_path * dt * delta_p_distance) / l_odomR_odomB # TODO: divide 0

                    print('scan_q_indices', scan_q_indices)
                    print('scan_distances', scan_distances)
                    print('scan_values', scan_values)
                    print('delta_p_distance', delta_p_distance)
                    print('along_path_correction', along_path_correction)
                    print('\n')
                    ros.publish_topic('/debug_img', np_to_Image(debug_img)) # [debug]

                    if u > 1.0 - 1e-2 or l_odomR_odomB < self.distance_threshold:
                        self.pass_to_next_goal()
                        continue

                    # rotation correction
                    theta_A = scan_offsets[k_r] / image.shape[1] * self.horizontal_fov
                    theta_B = scan_offsets[k_r + 1] / image.shape[1] * self.horizontal_fov
                    theta_R = (1 - u) * theta_A + u * theta_B
                    rotation_correction = -self.k_rotation * dt * theta_R

                    # 优化 T_0_odomB
                    correction_offset = t3d_ext.etR([0, 0, 0], t3d.euler.euler2mat(0, 0, rotation_correction)) @ T_odomR_odomB
                    correction_offset[:3, 3] *= along_path_correction
                    self.T_0_odomB = T_0_odomR @ correction_offset
                
                # TODO: delta distance 判断 (似乎不需要)
                # T_odomR_odomB = t3d_ext.einv(T_0_odomR) @ self.T_0_odomB # 经过校正, 与 pass_to_next_goal 中的 T_odomA_odomB 不同
                
                # 发布调试话题
                ros.publish_topic('/a', t3d_ext.e2PS(self.T_0_odomA, frame_id = 'odom'))
                ros.publish_topic('/b', t3d_ext.e2PS(self.T_0_odomB, frame_id = 'odom'))
                ros.publish_topic('/r', t3d_ext.e2PS(T_0_odomR, frame_id = 'odom'))
                
                # 执行器 [Approach 1: r + 2 预测]
                # v = self.reference_velocity

                # T_0_qN = [item for item in self.q.q[r:(r + 3)] if item is not None][-1][1]
                # T_0_qB = self.q[r + 1][1]
                # T_0_odomN = self.T_0_odomB @ t3d_ext.einv(T_0_qB) @ T_0_qN
                # T_odomR_odomN = t3d_ext.einv(T_0_odomR) @ T_0_odomN

                # dy = T_odomR_odomN[1, 3]
                # dx = T_odomR_odomN[0, 3]
                # ros.publish_topic('/goal', t3d_ext.e2PS(T_0_odomN, frame_id = 'odom'))

                # if abs(dy) < 1e-2:
                #     w = 0.0
                # else:
                #     d_square = dx ** 2 + dy ** 2
                #     R = d_square / 2 / dy
                #     if abs(R) < self.R_min_abs:
                #         R = np.sign(R) * self.R_min_abs
                #     w = v / R

                # self.io['actuator_command'].write(('vw', v, w))
                # operation_num += 1

                # 执行器 [Approach 2: 加权预测], TODO: velocity control & weights
                # weights = np.array([0.0, 1.0] + [0.0] * (p - 2)) # r + 1 (Qb) ~ (Qi) ~ r + p (Qp)
                weights = np.array([0.0, 1.0, 0.6, 0.2, 0.1, 0.05, 0.03, 0.02, 0.01])
                v = self.reference_velocity

                T_q_indices = np.array([q_idx for q_idx in range(r + 1, r + p + 1) if self.q[q_idx] is not None])
                T_0_Qi = np.array([self.q[q_idx][1] for q_idx in T_q_indices])
                T_odomR_odomQi = (t3d_ext.einv(T_0_odomR) @ self.T_0_odomB @ t3d_ext.einv(T_0_Qi[0])) @ T_0_Qi
                
                xy = np.array([item[:2, 3] for item in T_odomR_odomQi])

                d_square = np.sum(xy * xy, axis = 1)
                with np.errstate(divide = 'ignore', invalid = 'ignore'):
                    R = d_square / 2 / xy[:, 1]
                    flag = abs(R) < self.R_min_abs
                    R[flag] = np.sign(R[flag]) * self.R_min_abs
                    w = v / R
                    w[np.isnan(w)] = 0.0

                    weights_q = weights[T_q_indices - (r + 1)]
                    w_hat = weights_q @ w / np.sum(weights_q)

                    if np.isnan(w_hat):
                        w_hat = 0.0

                self.io['actuator_command'].write(('vw', v, w_hat))
                operation_num += 1

