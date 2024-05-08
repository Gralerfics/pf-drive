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
        self.steering_predict_goals = kwargs['steering_predict_goals'] # p, 注意弯道的相对位置并没有补偿误差, p 过大会导致转向轨迹太贴近 record 中有误差的值.
        self.steering_weights = kwargs['steering_weights']
        self.slowing_predict_goals = kwargs['slowing_predict_goals'] # s, TODO: 预测量的参数似乎应该使用距离而非 goal 个数, 否则有与 record 打点距离耦合的问题

        self.k_rotation = kwargs['k_rotation']
        self.k_along_path = kwargs['k_along_path']

        self.odom_compensation_rotation_update_rate = kwargs['odom_compensation_rotation_update_rate']
        self.odom_compensation_translation_update_rate = kwargs['odom_compensation_translation_update_rate']
        self.odom_compensation_rotation_threshold = kwargs['odom_compensation_rotation_threshold']
        self.odom_compensation_translation_threshold = kwargs['odom_compensation_translation_threshold']

        self.l = kwargs['track']
        self.d = kwargs['wheelbase']
        self.r = kwargs['wheel_radius']
        self.max_phi = kwargs['max_steering_angle']
        self.friction_factor = kwargs['friction_factor']
        self.R_min_abs = self.d / np.tan(self.max_phi) + self.l / 2

        self.distance_threshold = kwargs['distance_threshold']
        self.reference_velocity = kwargs['reference_velocity']

        self.along_path_debug_image_topic = kwargs.get('along_path_debug_image_topic', None)
        self.local_raw_path_debug_topic = kwargs.get('local_raw_path_debug_topic', None)
        self.odom_a_debug_topic = kwargs.get('odom_a_debug_topic', None)
        self.odom_b_debug_topic = kwargs.get('odom_b_debug_topic', None)
        self.odom_r_debug_topic = kwargs.get('odom_r_debug_topic', None)

        # 滑动窗口队列
        self.max_rps = max(self.along_path_radius, self.steering_predict_goals, self.slowing_predict_goals)
        self.max_rp = max(self.along_path_radius, self.steering_predict_goals)
        self.q_r = self.along_path_radius
        self.q_size = self.along_path_radius + 1 + self.max_rps
        """
            0   1   2   3   4   5   6   7   8   9
            x   x   D0  D1  D2  D3  D4  D5  D6  D7
           |- r -|  ^  |- r -|
                    r  |----------- p -----------|
        """
        self.q = ListQueue(size = self.q_size) # (image, odom)

        # 运行时
        self.goal_distances = [0.0] # distances between goal 0 and i
        self.goal_idx = -self.max_rps - 1 # goal just passed

        self.T_0_odomA = None
        self.T_0_odomB = None
        
        self.record_T_odomA_odomB = None
        self.record_T_odomA_odomB_rotation = None
        self.record_T_odomA_odomB_translation = None

        self.odom_compensation_rotation_factor = 1.0
        self.odom_compensation_translation_factor = 1.0
    
    def pass_to_next_goal(self):
        # loader 读完后不断发送 None, 此处不断入队, 直到 q_passed_idx + 1 项也为 None 时即结束
        self.q.push(self.io['record'].read(block = True))

        # 更新 passed_goal 并输出到端口
        self.goal_idx += 1
        if self.goal_idx >= 0:
            self.io['passed_goal'].write(self.goal_idx)

        # rec_A, rec_B 有效则更新 T_0_odomA, T_0_odomB
        if self.q[self.q_r] is not None and self.q[self.q_r + 1] is not None:
            # 若 T_0_odomA 和 T_0_odomB 有值, 即进行了一轮迭代, 记录迭代后校正的程度
            if self.T_0_odomA is not None and self.T_0_odomB is not None:
                corrected_T_odomA_odomB = t3d_ext.einv(self.T_0_odomA) @ self.T_0_odomB
                rotation_factor = t3d.euler.mat2euler(corrected_T_odomA_odomB[:3, :3])[2] / self.record_T_odomA_odomB_rotation if abs(self.record_T_odomA_odomB_rotation) > self.odom_compensation_rotation_threshold else self.odom_compensation_rotation_factor
                translation_factor = t3d_ext.norm(corrected_T_odomA_odomB[:3, 3]) / self.record_T_odomA_odomB_translation if self.record_T_odomA_odomB_translation > self.odom_compensation_translation_threshold else self.odom_compensation_translation_factor
                
                self.odom_compensation_rotation_factor = self.odom_compensation_rotation_update_rate * rotation_factor + (1 - self.odom_compensation_rotation_update_rate) * self.odom_compensation_rotation_factor
                self.odom_compensation_translation_factor = self.odom_compensation_translation_update_rate * translation_factor + (1 - self.odom_compensation_translation_update_rate) * self.odom_compensation_translation_factor

            # 若 T_0_odomB 无值则赋当前 odom 值; 更新 A 为 B
            if self.T_0_odomB is None:
                self.T_0_odomB = self.io['odom'].read(block = True)
            self.T_0_odomA = self.T_0_odomB

            # record 中的 T_odomA_odomB
            self.record_T_odomA_odomB = t3d_ext.einv(self.q[self.q_r][1]) @ self.q[self.q_r + 1][1] # T_odomA_odomB = T_{rec_r}_{rec_(r+1)} = inv(T_0_{rec_r}) * T_0_{rec_(r+1)}
            self.record_T_odomA_odomB_rotation = t3d.euler.mat2euler(self.record_T_odomA_odomB[:3, :3])[2]
            self.record_T_odomA_odomB_translation = t3d_ext.norm(self.record_T_odomA_odomB[:3, 3])

            # 根据先前的误差程度粗调 T_odomA_odomB 作为初始估计
            T_odomA_odomB = self.record_T_odomA_odomB.copy()
            if self.record_T_odomA_odomB_rotation > self.odom_compensation_rotation_threshold:
                T_odomA_odomB[:3, :3] = t3d.euler.euler2mat(0, 0, t3d.euler.mat2euler(T_odomA_odomB[:3, :3])[2] * self.odom_compensation_rotation_factor)
            if self.record_T_odomA_odomB_translation > self.odom_compensation_translation_threshold:
                T_odomA_odomB[:3, 3] *= self.odom_compensation_translation_factor
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

        # 常用量
        r = self.along_path_radius
        rp = self.max_rp
        rps = self.max_rps
        
        v_full = self.reference_velocity
        v_under = np.sqrt(self.friction_factor * self.R_min_abs)
        weights = np.pad(self.steering_weights, (0, rps - len(self.steering_weights)), 'constant', constant_values = 0)

        v_target = v_full

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
                self.io['passed_goal'].write(None) # 结束信号
                # self.io['actuator_command'].write(('vw', 0, 0)) # 由 __main__ 进行停车
                break
            
            # 运算
            if self.io['processed_image'].poll() and self.io['odom'].poll():
                image = self.io['processed_image'].read()
                odom = self.io['odom'].read() # odom, R: robot

                i = self.goal_idx # q_idx - r + i = goal_idx

                T_0_odomR = odom
                T_odomR_odomB = t3d_ext.einv(T_0_odomR) @ self.T_0_odomB

                t_0_odomA = t3d_ext.edt(self.T_0_odomA)
                t_0_odomB = t3d_ext.edt(self.T_0_odomB)
                t_0_odomR = t3d_ext.edt(T_0_odomR)

                t_odomA_odomB = t_0_odomB - t_0_odomA
                t_odomA_odomR = t_0_odomR - t_0_odomA
                t_odomR_odomB = t3d_ext.edt(T_odomR_odomB)

                l_odomA_odomB = t3d_ext.norm(t_odomA_odomB)
                l_odomR_odomB = t3d_ext.norm(t_odomR_odomB)

                turning_goal = l_odomA_odomB < self.distance_threshold # TODO: 需要吗?
                if not turning_goal:
                    l_proj_odomA_odomR = np.dot(t_odomA_odomR, t_odomA_odomB) / l_odomA_odomB
                    u = l_proj_odomA_odomR / l_odomA_odomB # not turning_goal

                    dt = current_time - timer_P
                    timer_P = current_time

                    # 到达下一个目标
                    if u > 1.0 - 1e-2 or l_odomR_odomB < self.distance_threshold:
                        self.pass_to_next_goal()
                        continue

                    # 沿路径校正
                    scan_q_indices = [q_idx for q_idx in range(2 * r + 1) if self.q[q_idx] is not None]
                    scan_q_indices = [scan_q_indices[0]] * (scan_q_indices[0]) + scan_q_indices
                    scan_q_indices = scan_q_indices + [scan_q_indices[-1]] * (2 * r - scan_q_indices[-1])
                    scan_distances = np.array([self.goal_distances[q_idx - r + i] for q_idx in scan_q_indices]) - self.goal_distances[i] - l_proj_odomA_odomR
                    scan_offsets, scan_values, nooffset_values = np.zeros((3, len(scan_q_indices)))

                    if self.along_path_debug_image_topic is not None: # [debug]
                        debug_img = None
                        dash_h = 5
                        label_w = 10
                        dash_img = np.zeros_like(image)[:dash_h, :]
                    
                    for k, q_idx in enumerate(scan_q_indices):
                        if q_idx == r:
                            k_r = k
                        img_ref = self.q[q_idx][0]
                        scan_offsets[k], scan_values[k], nooffset_values[k] = NCC_horizontal_match(image, img_ref)

                        if self.along_path_debug_image_topic is not None: # [debug]
                            if debug_img is None:
                                debug_img = img_ref.copy()
                            else:
                                debug_img = np.concatenate((debug_img, dash_img, img_ref), axis = 0)
                    
                    scan_values[np.abs(np.arange(len(scan_values)) - scan_values.argmax()) > 1] = 0 # 保留最值和相邻项 (保证相邻)
                    scan_values[scan_values < scan_values[scan_values != scan_values.max()].max()] = 0 # 在此基础上再保留最大和次大值
                    delta_p_distance = scan_values / scan_values.sum() @ scan_distances

                    along_path_correction = (l_odomR_odomB - self.k_along_path * dt * delta_p_distance) / l_odomR_odomB
                    
                    if self.along_path_debug_image_topic is not None: # [debug]
                        h, w = image.shape[0] + 5, image.shape[1]
                        
                        image_pad = np.concatenate((dash_img, image, dash_img), axis = 0)
                        image_pad = np.pad(image_pad, ((0, 0), (0, label_w)), mode = 'constant', constant_values = 0)
                        image_pad[[dash_h // 2, -dash_h // 2], :] = 255

                        debug_img = np.pad(debug_img, ((0, 0), (0, label_w)), mode = 'constant', constant_values = 0)
                        for k, q_idx in enumerate(scan_q_indices):
                            debug_img[(k * h):(k * h + h), w:] = int(255 * scan_values[k] / scan_values.max())

                        debug_img = np.concatenate((debug_img[:(r * h - dash_h), :], image_pad, debug_img[(r * h):, :]), axis = 0)

                        ros.publish_topic(self.along_path_debug_image_topic, np_to_Image(debug_img))

                    # print('scan_q_indices', scan_q_indices) # [debug]
                    # print('scan_distances', scan_distances)
                    # print('scan_offsets', scan_offsets)
                    # print('scan_values', scan_values)
                    # print('nooffset_values', nooffset_values)
                    # print('delta_p_distance', delta_p_distance)
                    # print('apc', along_path_correction)
                    # print('\n')

                    # 转向校正
                    theta_A = scan_offsets[k_r] / image.shape[1] * self.horizontal_fov
                    theta_B = scan_offsets[k_r + 1] / image.shape[1] * self.horizontal_fov
                    theta_R = (1 - u) * theta_A + u * theta_B
                    rotation_correction = -self.k_rotation * dt * theta_R

                    # 优化 T_0_odomB
                    correction_offset = t3d_ext.etR([0, 0, 0], t3d.euler.euler2mat(0, 0, rotation_correction)) @ T_odomR_odomB
                    correction_offset[:3, 3] *= along_path_correction
                    self.T_0_odomB = T_0_odomR @ correction_offset
            
                # 执行器
                p_indices = np.array([q_idx for q_idx in range(r + 1, r + rp + 1) if self.q[q_idx] is not None]) # p 范围内有效点索引
                T_0_Qi = np.array([self.q[q_idx][1] for q_idx in p_indices]) # p 范围内有效点
                T_odomR_odomQi = (t3d_ext.einv(T_0_odomR) @ self.T_0_odomB @ t3d_ext.einv(T_0_Qi[0])) @ T_0_Qi # 与 R 的相对位置
                
                xy = np.array([item[:2, 3] for item in T_odomR_odomQi]) # dx, dy
                d_square = np.sum(xy * xy, axis = 1) # d ** 2
                with np.errstate(divide = 'ignore', invalid = 'ignore'):
                    # 旋转半径
                    R = d_square / 2 / xy[:, 1] # x 是前方, y 才是侧向

                    # 限制最小旋转半径
                    flag = abs(R) < self.R_min_abs
                    R[flag] = np.sign(R[flag]) * self.R_min_abs

                    # 旋转半径的倒数 (便于加权平均, 若使用 R 需要考虑 inf 等)
                    w_normal = 1 / R
                    w_normal[np.isnan(w_normal)] = 0.0

                    # 加权平均
                    weights_q = weights[p_indices - (r + 1)] # 有效点对应的权重
                    w_normal_hat = weights_q @ w_normal / np.sum(weights_q)
                    if np.isnan(w_normal_hat):
                        w_normal_hat = 0.0
                
                # 速度控制
                s_indices = np.array([q_idx for q_idx in range(r + rps + 1) if self.q[q_idx] is not None])
                xy_0_Qi = np.array([self.q[q_idx][1][:2, 3] for q_idx in s_indices])
                vs = xy_0_Qi[1:] - xy_0_Qi[:-1]
                ls = np.linalg.norm(vs, axis = 1)
                with np.errstate(divide = 'ignore', invalid = 'ignore'):
                    phis_abs = np.abs(np.arccos(np.clip(np.sum(vs[:-1] * vs[1:], axis = 1) / (ls[:-1] * ls[1:]), -1.0, 1.0)))
                    Rs_abs = self.d / np.tan(phis_abs)
                    R_min = Rs_abs.min()
                
                if np.isnan(R_min):
                    v_target = v_full
                else:
                    v_target = np.clip(np.sqrt(self.friction_factor * R_min), v_under, v_full)
                
                w_target = v_target * w_normal_hat
                if np.isnan(w_target):
                    w_target = 0.0

                self.io['actuator_command'].write(('vw', v_target, w_target))
                operation_num += 1
                
                # 调试信息
                if self.odom_a_debug_topic is not None:
                    ros.publish_topic('/a', t3d_ext.e2PS(self.T_0_odomA, frame_id = 'odom'))
                if self.odom_b_debug_topic is not None:
                    ros.publish_topic('/b', t3d_ext.e2PS(self.T_0_odomB, frame_id = 'odom'))
                if self.odom_r_debug_topic is not None:
                    ros.publish_topic('/r', t3d_ext.e2PS(T_0_odomR, frame_id = 'odom'))
                if self.local_raw_path_debug_topic is not None: # 队内原始 odom 路径调试话题, Qr 与 T_0_odomA 对齐
                    aligned_q_indices = np.array([q_idx for q_idx in range(r, r + rps + 1) if self.q[q_idx] is not None])
                    aligned_q_odoms = np.array([self.q[q_idx][1] for q_idx in aligned_q_indices])
                    aligned_q_odoms = self.T_0_odomA @ t3d_ext.einv(aligned_q_odoms[0]) @ aligned_q_odoms
                    ros.publish_topic(self.local_raw_path_debug_topic, t3d_ext.es2P(aligned_q_odoms, frame_id = 'odom'))

