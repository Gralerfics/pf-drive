import os
import time
import json
import threading

import rospy

import numpy as np

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator, type_from_str, generate_time_str
from tr_drive.util.geometry import Vec3, Frame, FrameList
from tr_drive.util.image import ImageProcessor

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.controller.goal_controller import GoalController
from tr_drive.sensor.global_locator import GlobalLocator

from tr_drive.persistent.recording import Recording


class Repeater:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'repeater_debugger')
        
        self.ready = threading.Event()
        self.launched = threading.Event()
        self.paused = threading.Event()

        # self.new_goal_passed = threading.Event()
        
        # public
        self.recording: Recording = None
        
        self.passed_goal_index_lock = threading.Lock() # 如下, 但对 int 的自增不是原子的
        self.passed_goal_index = 0 # the index of the nearest goal that has been passed
        
        self.goal_intervals: list[float] = [] # the intervals between goal i and goal i + 1
        self.goal_distances: list[float] = [] # the distances between goal 0 and goal i

        self.passed_goal_image_match_offset: int = 0 # for GUI; 关于线程安全: 对 int 仅赋值和读取在 Python 这里似乎是原子的
        self.next_goal_image_match_offset: int = 0 # 同上

        # P.S. 命名上 T 代表 Transfrom, 0 为 biased_odom 原点, a 为刚经过的目标, b 为下一个目标, c 为当前 biased_odom.
        self.T_0a: Frame = None # 故 frame_id 为 'biased_odom'
        self.T_0b: Frame = None # 同上
        
        # parameters (with default camera parameters, will be modified when loading recording)
        self.params = DictRegulator(rospy.get_param('/tr'))

        self.global_locator_used = 'global_locator' in self.params # 是否引入全局定位信息

        self.save_report = not self.params.persistent.report_path.startswith('.none')
        if self.params.persistent.report_path.startswith('.auto'):
            self.params.persistent.report_path = self.params.persistent.recording_path + '_report' # TODO
        self.repeat_ground_truth_odoms: FrameList = FrameList()
        
        # devices
        self.init_devices()
        
        # load recording
        self.load_recording()
        
        # ready
        self.ready.set()
    
    def init_devices(self):
        self.camera = Camera(
            raw_image_topic = self.params.camera.raw_image_topic,
            patch_size = 15, # self.params.camera.patch_size,
            resize = [150, 50], # self.params.camera.resize,
            horizontal_fov = 114.59, # self.params.camera.horizontal_fov,
            processed_image_topic = self.params.camera.processed_image_topic
        )
        self.params.remove('camera') # 参数存储于设备实例后即删除, 以保证唯一性, 避免参数变更时的冗余和冲突
        self.camera.wait_until_ready()
        
        self.odometry = Odom(
            odom_topic = self.params.odometry.odom_topic,
            processed_odom_topic = self.params.odometry.processed_odom_topic
        )
        self.params.remove('odometry')
        self.odometry.register_odom_received_hook(self.odom_received)
        self.odometry.wait_until_ready()
        
        self.controller = GoalController(
            cmd_vel_topic = self.params.controller.cmd_vel_topic,
            k_rho = self.params.controller.k_rho,
            k_alpha = self.params.controller.k_alpha,
            k_beta = self.params.controller.k_beta,
            k_theta = self.params.controller.k_theta,
            velocity_min = self.params.controller.velocity_min,
            velocity_max = self.params.controller.velocity_max,
            omega_min = self.params.controller.omega_min,
            omega_max = self.params.controller.omega_max,
            translation_tolerance = self.params.controller.translation_tolerance,
            rotation_tolerance = self.params.controller.rotation_tolerance
        )
        self.params.remove('controller')
        self.controller.set_odometry(self.odometry)
        self.controller.wait_until_ready()
        
        if self.global_locator_used:
            global_locator_type = self.params.global_locator.type
            if global_locator_type == 'topic':
                self.global_locator = GlobalLocator(
                    locator_type = global_locator_type,
                    odometry = self.odometry,
                    fixed_frame_id = self.params.global_locator.fixed_frame_id,

                    topic = self.params.global_locator.topic,
                    topic_type = type_from_str(self.params.global_locator.topic_type)
                )
            elif global_locator_type == 'tf':
                self.global_locator = GlobalLocator(
                    locator_type = global_locator_type,
                    odometry = self.odometry,
                    fixed_frame_id = self.params.global_locator.fixed_frame
                )
            else: # global_locator_type == 'webots':
                self.global_locator = GlobalLocator(
                    locator_type = global_locator_type,
                    odometry = self.odometry,
                    fixed_frame_id = self.params.global_locator.fixed_frame_id,
                    
                    supervisor_srv = self.params.global_locator.supervisor_srv,
                    robot_def = self.params.global_locator.robot_def
                )
            self.params.remove('global_locator')
            self.global_locator.wait_until_ready()
    
    def load_recording(self):
        # load recording
        self.recording = Recording.from_file(self.params.persistent.recording_path)

        # initialize goal intervals and distances
        self.goal_distances.append(0.0)
        for idx in range(len(self.recording) - 1):
            d = (self.recording.odoms[idx + 1].t - self.recording.odoms[idx].t).norm()
            self.goal_intervals.append(d)
            self.goal_distances.append(self.goal_distances[-1] + d)
        self.goal_intervals.append(0.0)
        
        # modify camera parameters
        self.camera.set_params(**self.recording.get_image_parameters())
    
    def is_ready(self):
        return self.ready.is_set()
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for repeater ...')
            time.sleep(0.2)
        rospy.loginfo('Repeater is ready.')

    def inc_passed_goal_index(self):
        with self.passed_goal_index_lock:
            self.passed_goal_index += 1

    def get_passed_goal_index(self):
        with self.passed_goal_index_lock:
            return self.passed_goal_index
    
    def set_passed_goal_index(self, idx: int):
        with self.passed_goal_index_lock:
            self.passed_goal_index = idx

    def start(self):
        if not self.is_ready() or self.launched.is_set():
            return False
        
        self.odometry.zeroize()
        self.controller.activate()
        
        # TODO: 全局定位; 下面皆暂时以第一点为起点.

        # 初始化 T_0a, T_0b, passed_goal_index
        self.set_passed_goal_index(0)
        self.T_0a = self.recording.odoms[self.get_passed_goal_index()] # OA
        self.T_0b = self.recording.odoms[self.get_passed_goal_index() + 1] # in fact, OA + AB

        # 起点对齐
        if self.global_locator_used and self.recording.is_ground_truths_available():
            self.global_locator.align_biased_odom_with_global(self.recording.odoms[self.get_passed_goal_index()], self.recording.ground_truths[self.get_passed_goal_index()])

        # 发布路径
        biased_odom_frame_id = self.odometry.get_biased_odom_frame_id()
        self.debugger.publish('/recorded_odoms', self.recording.odoms.to_Path(frame_id = biased_odom_frame_id))
        if self.global_locator_used and self.recording.is_ground_truths_available():
            aligned_global_frame_id = self.global_locator.get_aligned_global_frame_id()
            if aligned_global_frame_id is not None:
                self.debugger.publish('/recorded_gts', self.recording.ground_truths.to_Path(frame_id = aligned_global_frame_id))
        
        self.launched.set()
        return True
    
    def pause(self):
        if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
            return False
        
        self.controller.deactivate()
        self.paused.set()
        return True
    
    def resume(self):
        if not self.is_ready() or not self.launched.is_set() or not self.paused.is_set():
            return False
        
        self.controller.activate()
        self.paused.clear()
        return True
    
    def batched_match(self, image, indices):
        offsets = np.zeros(len(indices))
        values = np.zeros(len(indices))
        for i, idx in enumerate(indices):
            offsets[i], values[i] = ImageProcessor.best_match_offset(image, self.recording.processed_images[idx], self.params.repeater.match_offset_radius)
        return offsets.astype(int), values

    def distance_to_goal(self, d: float):
        return np.searchsorted(self.goal_distances[1:], d, side = 'right')

    def pass_to_next_goal(self):
        self.inc_passed_goal_index()
        # self.new_goal_passed.set()
        
        i = self.get_passed_goal_index()
        n = len(self.recording)
        if i >= n - 1:
            rospy.loginfo('Finished.')
            if self.save_report: # TODO
                os.makedirs(self.params.persistent.report_path, exist_ok = True)
                
                # repeated path
                self.repeat_ground_truth_odoms.to_tum_file(self.params.persistent.report_path + '/traj_repeat.txt')

                # taught path
                self.recording.ground_truths.to_tum_file(self.params.persistent.report_path + '/traj_teach.txt')

                # repeating parameters
                with open(self.params.persistent.report_path + '/parameters.json', 'w') as f:
                    json.dump({
                        'recording': self.params.persistent.recording_path,
                        'controller': {
                            'k_rho': self.controller.k_rho,
                            'k_alpha': self.controller.k_alpha,
                            'k_beta': self.controller.k_beta,
                            'k_theta': self.controller.k_theta,
                            'velocity_min': self.controller.velocity_min,
                            'velocity_max': self.controller.velocity_max,
                            'omega_min': self.controller.omega_min,
                            'omega_max': self.controller.omega_max,
                            'translation_tolerance': self.controller.translation_tolerance,
                            'rotation_tolerance': self.controller.rotation_tolerance
                        },
                        'repeater': {
                            'match_offset_radius': self.params.repeater.match_offset_radius,
                            'along_path_radius': self.params.repeater.along_path_radius,
                            'k_rotation': self.params.repeater.k_rotation,
                            'k_along_path': self.params.repeater.k_along_path,
                            'distance_threshold': self.params.repeater.distance_threshold,
                            'angle_threshold': self.params.repeater.angle_threshold,
                            'goal_advance_distance': self.params.repeater.goal_advance_distance
                        }
                    }, f)
            self.pause()
            return

        self.T_0a = self.T_0b
        self.T_0b = self.T_0b * (self.recording.odoms[i].I * self.recording.odoms[i + 1])
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
            return

        if not hasattr(self, 'last_t'):
            self.last_t = time.time()
        if not hasattr(self, 'n'):
            self.n = 0
        else:
            self.n += 1
        t = time.time()
        if t - self.last_t > 2.0:
            print(f'fps: {self.n / (t - self.last_t)}')
            self.last_t = t
            self.n = 0
        
        # only use new odoms
        t_odom = self.odometry.get_odom_msg().header.stamp.to_sec()
        t_current = time.time()
        # print(f'{t_odom} {t_current - t_odom}')
        if t_current - t_odom > 0.1:
            return
        
        i = self.get_passed_goal_index()
        n = len(self.recording)
        r = self.params.repeater.along_path_radius
        
        T_0c: Frame = self.odometry.get_biased_odom()
        T_cb: Frame = T_0c.I * self.T_0b

        t_0a = self.T_0a.t
        t_0b = self.T_0b.t
        t_0c = T_0c.t
        
        t_ab = t_0b - t_0a
        t_ac = t_0c - t_0a
        t_cb = T_cb.t

        d_ab = t_ab.norm() # self.goal_intervals[i]
        d_cb = t_cb.norm()
        
        turning_goal = d_ab < self.params.repeater.distance_threshold
        if not turning_goal:
            d_p_ac = Vec3.dot(t_ac, t_ab) / d_ab # if not turning_goal else 0.0
            u = d_p_ac / d_ab if not turning_goal else 0.0 # 0.5
            # u = np.clip(d_p_ac / d_ab, 0.0, 1.0) # if not turning_goal else 0.0 # 0.5
            
            # along-path correction
            # scan_indices = list(range(max(0, i - r + 1), min(n, i + r + 1)))
            scan_indices = np.clip(list(range(i - r + 1, i + r + 1)), 0, n - 1).tolist()
            scan_distances = np.array([self.goal_distances[idx] for idx in scan_indices]) - self.goal_distances[i] - d_p_ac # c 处为 0
            scan_offsets, scan_values = self.batched_match(self.camera.get_processed_image(), scan_indices)
            scan_values[scan_values < min(0.1, scan_values.min() / 2)] = 0 # TODO, threshold 当前随便设置的, 理应表示 NCC 底噪; 如果全都被滤除说明两图差距已经很大, 也许可以作为确认丢失的一种条件; 最小值除二仅为防止崩溃, 无实际意义.
            delta_p = scan_values / scan_values.sum() @ scan_distances
            delta_distance = self.params.repeater.k_along_path * delta_p # np.clip((d_cb - delta_distance) / d_cb, scan_distances[0], scan_distances[-1])
            along_path_correction = (d_cb - delta_distance) / d_cb

            # if along_path_correction > 1.0: # [理论上这不是到达目标条件, 暂时先如此] delta_distance 为负, 说明已经超过估计
            #     self.pass_to_next_goal()
            #     # print('along_path_correction > 1.0')
            #     return

            # rospy.loginfo(f'{d_p_ac}, {d_ab}, {u}')
            if u > 1.0 - 1e-2: # 投影点到达下个目标
                self.pass_to_next_goal()
                # print('u -> 1.0')
                return

            # rotation correction
            theta_a = self.camera.px_to_rad_horizontal(scan_offsets[r])
            theta_b = self.camera.px_to_rad_horizontal(scan_offsets[r + 1])
            delta_theta = (1 - u) * theta_a + u * theta_b
            rotation_correction = self.params.repeater.k_rotation * delta_theta # 由于逆时针才是正方向，故刚好符号对消.

            self.passed_goal_image_match_offset = scan_offsets[r] # for GUI
            self.next_goal_image_match_offset = scan_offsets[r + 1] # 同上

            # print(f'rot_c: {rotation_correction}; ap_c: {along_path_correction}; u: {u}; delta_d: {delta_distance}; d_ab/2: {d_ab / 2}\n')
        else:
            along_path_correction = 1.0 # ?
            rotation_correction = 0.0

            # print(f'rot_c: {rotation_correction}; ap_c: {along_path_correction}\n')

        # new estimation of T_0b
        if not turning_goal:
            correction_offset = Frame.from_z_rotation(rotation_correction) * T_cb
            correction_offset.translation *= along_path_correction
            self.T_0b = T_0c * correction_offset

        # report, TODO: 参数控制暂略, 此处将保存数据, 最终输出一个 tum 标准的路径文件
        if self.save_report:
            self.repeat_ground_truth_odoms.append(self.global_locator.get_global_frame())
        
        # print(f'rot_c: {rotation_correction}; ap_c: {along_path_correction}\n')

        # goal
        # if not self.new_goal_passed.is_set():
        delta = T_0c.I * self.T_0b
        if delta.t.norm() < self.params.repeater.distance_threshold or turning_goal:
            if abs(delta.q.Euler[2]) < self.params.repeater.angle_threshold:
                self.pass_to_next_goal() # 里程计反馈到达设定的目标
                # print('in tolerance')
                return
            else:
                goal_advanced = self.T_0b
        else:
            goal_advanced = self.T_0b * Frame.from_translation(Vec3(self.params.repeater.goal_advance_distance, 0, 0))
            # goal_advanced = self.T_0b * Frame.from_translation(delta.t.normalize() * self.params.repeater.goal_advance_distance)
        
        biased_odom_frame_id = self.odometry.get_biased_odom_frame_id()
        self.debugger.publish('/a', self.T_0a.to_PoseStamped(frame_id = biased_odom_frame_id))
        self.debugger.publish('/b', self.T_0b.to_PoseStamped(frame_id = biased_odom_frame_id))
        self.debugger.publish('/r', self.odometry.get_biased_odom().to_PoseStamped(frame_id = biased_odom_frame_id))

        # if self.global_locator_used:
        #     aligned_map_frame_id = self.global_locator.get_aligned_global_frame_id()
        #     self.debugger.publish('/a_gt', self.recording.ground_truths[i].to_PoseStamped(frame_id = aligned_map_frame_id))
        #     if i < len(self.recording) - 1:
        #         self.debugger.publish('/b_gt', self.recording.ground_truths[i + 1].to_PoseStamped(frame_id = aligned_map_frame_id))
        
        self.controller.set_goal(goal_advanced)
        # else:
        #     self.new_goal_passed.clear()


""" TODO ideas
Ackermann:
    用里程计
控制器:
    [important] 修改 goal_controller, 减少目标平移对旋转指令的即时影响; 或许也可以增大 advance distance; 或许也有 rotation offset 估计跳变的原因.
控制:
    切换目标后的第一次估计暂时不发布为 goal; (完成, 效果貌似不明显)
    记录累积转向, 在大趋势上提前补偿; (感觉和增加 I 项类似)
    长时间停留 -> pass_to_next_goal; (不卡在原地的备用手段, 目前暂无卡死情况)
    金字塔匹配辅助确认距离;
    互相关加权, 倾向小角度;
    角度校正跳变处理 (例如跨度过大则找其他尖峰等);
    controller 限速和 goal 间距的关系 (低限速则拐大弯, 插值更平滑, 可能可适用于更大 goal 间距)
    [!] 超前预测控制?
问题:
    goal_index 累计后太过超前.
        删去 along_path_correction > 1.0 条件, 恢复 u > 1.0 - eps 条件, 差距有所减小.
        ** 提高 k_rotation from 0.01 to 0.04, 大幅贴近路线, test_3 跑通; 后期还有一定 goal_index 超前, 但程度很小.
            k_along_path 也有待斟酌; 调参可视化 (?), 录制直线用于测试, 单步运算可视化等.
    帧率一高, 调节 k 虽然可以使路径更贴合, 但过程中左右摆动明显.
        把 P 改成 PD 或 PID (?)
    测试稀疏打点.
    修改控制器.
    Repeater 本身参数也提取出来.
    各 device 加一个 get_parameters_as_dict 方法方便存储.
"""

# if self.first_image is None:
#     self.first_image = args['image']
# current_image: DigitalImage = args['image']
# r, c = current_image.height, current_image.width

# offsets = np.arange(1 - c, c).astype(int)
# correlations = ImageProcessor.horizontal_NCC(current_image, self.first_image, offsets)
# values = np.clip((r - (np.array(correlations)) * r).astype(int), 0, r - 1)

# diagram_image = DigitalImage(np.array(np.zeros((r, 2 * c - 1, 1)), dtype = np.uint8))
# diagram_image.data[values, offsets + c - 1] = [255]
# diagram_image.data[:, np.argmax(correlations)] = [255]

# self.debugger.publish('diagram', diagram_image.to_Image(encoding = 'mono8'))