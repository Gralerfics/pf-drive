import time
import threading

import rospy

import numpy as np

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator
from tr_drive.util.conversion import Vec3, Frame, type_from_str
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
        
        # public
        self.recording: Recording = None
        
        self.passed_goal_index = 0 # the index of the nearest goal that has been passed
        self.passed_goal_index_lock = threading.Lock()
        
        self.goal_intervals: list[float] = [] # the intervals between goal i and goal i + 1
        self.goal_distances: list[float] = [] # the distances between goal 0 and goal i
        
        # parameters (with default camera parameters, will be modified when loading recording)
        self.params = DictRegulator(rospy.get_param('/tr'))
        self.global_locator_used = 'global_locator' in self.params # 是否引入全局定位信息.
        self.params.camera.add('patch_size', 15)
        self.params.camera.add('resize', [150, 50])
        self.params.camera.add('horizontal_fov', 114.59)
        
        # devices
        self.init_devices()
        
        # load recording
        self.load_recording()
        
        # ready
        self.ready.set()
    
    def init_devices(self):
        self.camera = Camera(
            raw_image_topic = self.params.camera.raw_image_topic,
            patch_size = self.params.camera.patch_size,
            resize = self.params.camera.resize,
            horizontal_fov = self.params.camera.horizontal_fov,
            processed_image_topic = self.params.camera.processed_image_topic
        )
        # self.camera.register_image_received_hook(self.image_received)
        self.camera.wait_until_ready()
        
        self.odometry = Odom(
            odom_topic = self.params.odometry.odom_topic,
            processed_odom_topic = self.params.odometry.processed_odom_topic
        )
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
        self.controller.set_odometry(self.odometry)
        # self.controller.register_goal_reached_hook(self.goal_reached)
        self.controller.wait_until_ready()
        
        if self.global_locator_used:
            global_locator_type = self.params.global_locator.type
            if global_locator_type == 'tf':
                self.global_locator = GlobalLocator(
                    locator_type = global_locator_type,
                    fixed_frame = self.params.global_locator.fixed_frame,
                    odometry = self.odometry
                )
            else: # global_locator_type == 'topic'
                self.global_locator = GlobalLocator(
                    locator_type = global_locator_type,
                    topic = self.params.global_locator.topic,
                    topic_type = type_from_str(self.params.global_locator.topic_type)
                )
            # self.global_locator.register_global_frame_received_hook(self.global_frame_received)
            self.global_locator.wait_until_ready()
    
    def load_recording(self):
        # load recording
        self.recording = Recording.from_path(self.params.persistent.recording_path)
        
        # initialize goal intervals and distances
        self.goal_distances.append(0.0)
        for idx in range(len(self.recording.odoms) - 1):
            d = (self.recording.odoms[idx + 1].t - self.recording.odoms[idx].t).norm()
            self.goal_intervals.append(d)
            self.goal_distances.append(self.goal_distances[-1] + d)
        self.goal_intervals.append(0.0)
        
        # modify camera parameters
        self.params.camera.patch_size = self.recording.params['image']['patch_size']
        self.params.camera.resize = self.recording.params['image']['resize']
        self.params.camera.horizontal_fov = self.recording.params['image']['horizontal_fov']
        self.camera.set_params(**self.params.camera.to_dict())
        
        # TODO: debug, publish recording
        self.debugger.publish('/recorded_odoms', Frame.to_path(self.recording.odoms, frame_id = 'odom'))
        if self.global_locator_used:
            frame_id = self.params.global_locator.fixed_frame if 'fixed_frame' in self.params.global_locator else 'map'
            self.debugger.publish('/recorded_gts', Frame.to_path(self.recording.ground_truths, frame_id = frame_id))
    
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

    def start_repeating(self):
        if not self.is_ready() or self.launched.is_set():
            return False
        
        self.odometry.zeroize()
        self.controller.activate()
        
        self.set_passed_goal_index(0) # TODO
        
        self.launched.set()
        return True
    
    def pause_repeating(self):
        if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
            return False
        
        self.controller.deactivate()
        self.paused.set()
        return True
    
    def resume_repeating(self):
        if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
            return False
        
        self.controller.activate()
        self.paused.clear()
        return True
    
    def batched_match(self, image, indices):
        offsets = np.zeros(len(indices))
        values = np.zeros(len(indices))
        for i, idx in enumerate(indices):
            offsets[i], values[i] = ImageProcessor.best_match_offset(image, self.recording.processed_images[idx], self.params.repeater.match_offset_radius)
        return offsets, values

    def px_to_rad(self, px: int):
        return px / self.params.camera.resize[0] * self.params.camera.horizontal_fov / 180 * np.pi
    
    def distance_to_goal(self, d: float):
        return np.searchsorted(self.goal_distances[1:], d, side = 'right')
    
    # def image_received(self, **args):
    #     if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
    #         return
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.launched.is_set() or self.paused.is_set():
            return

        # if not hasattr(self, 'last_t'):
        #     self.last_t = time.time()
        # if not hasattr(self, 'n'):
        #     self.n = 0
        # else:
        #     self.n += 1
        # t = time.time()
        # if t - self.last_t > 2.0:
        #     print(f'fps: {self.n / (t - self.last_t)}')
        #     self.last_t = t
        #     self.n = 0
        
        # t_odom = self.odometry.get_odom_msg().header.stamp.to_sec()
        # t_current = time.time()
        # print(f'{t_current - t_odom}')
        
        # if self.get_passed_goal_index() >= len(self.recording.odoms) - 1:
        #     rospy.loginfo('Finished.')
        #     self.pause_repeating() # TODO
        #     return
        
        # i = self.get_passed_goal_index()
        # r = self.params.repeater.along_path_radius
        
        # T_0a: Frame = self.recording.odoms[i]
        # T_0b: Frame = self.recording.odoms[i + 1]
        # T_0c: Frame = self.odometry.get_biased_odom()
        
        # T_ab = T_0a.I * T_0b
        # d_ab = self.goal_intervals[i]
        
        # T_ac = T_0a.I * T_0c
        # d_p_ac = Vec3.dot(T_ac.t, T_ab.t) / d_ab if d_ab > 0 else 0
        # u = np.clip(d_p_ac / d_ab, 0.0, 1.0)
        
        # T_cb = T_ac.I * T_ab
        # d_cb = T_cb.t.norm()
        
        # along-path correction
        # scan_indices = list(range(max(0, i - r + 1), min(len(self.recording.odoms), i + r + 1)))
        # scan_distances = np.array([self.goal_distances[idx] for idx in scan_indices]) - self.goal_distances[i] - d_p_ac
        # scan_offsets, scan_values = self.batched_match(self.camera.get_processed_image(), scan_indices)
        # scan_values[scan_values < 0.1] = 0 # TODO, threshold
        # delta_p = scan_values / scan_values.sum() @ scan_distances
        # delta_distance = self.params.repeater.k_along_path * delta_p
        
        # if delta_distance > self.goal_intervals[i]:
        #     self.inc_passed_goal_index()
        #     return # TODO
        
        # along_path_correction = (d_cb - delta_distance) / d_cb
        
        # # rotation correction
        # theta_a = self.px_to_rad(scan_offsets[r - 1])
        # theta_b = self.px_to_rad(scan_offsets[r])
        # delta_theta = (1 - u) * theta_a + u * theta_b
        # rotation_correction = self.params.repeater.k_rotation * delta_theta
        
        # print(f'rotation_correction: {rotation_correction}; along_path_correction: {along_path_correction}')
        
        # # goal
        # goal_offset = Frame.from_z_rotation(rotation_correction) * T_cb
        # goal_offset.translation *= along_path_correction
        # goal = T_0c * goal_offset
        
        # ADVANCE_DISTANCE = 0.2
        # goal = goal * Frame.from_translation(Vec3(ADVANCE_DISTANCE, 0, 0)) # TODO: turning goal with no advance distance
        
        # ANGLE_THRESHOLD = 0.1
        # delta = T_0c.I * goal
        # if delta.t.norm() < ADVANCE_DISTANCE and abs(delta.q.Euler[2]) < ANGLE_THRESHOLD:
        #     self.inc_passed_goal_index()
        #     return # TODO
        
        # self.controller.set_goal(goal)


""" TODO ideas
结构：
    用 Decorator 重构，把 ready 的判断写成注解。

算法：
    金字塔匹配辅助确认距离;
    互相关加权, 倾向小角度;
    角度校正跳变处理 (例如跨度过大则找其他尖峰等);
    controller 限速和 goal 间距的关系 (低限速则拐大弯, 插值更平滑, 可能可适用于更大 goal 间距);
    多种计算 0~u~1 的方式融合
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