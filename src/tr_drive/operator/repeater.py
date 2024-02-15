import rospy

import numpy as np

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator
from tr_drive.util.conversion import Frame
from tr_drive.util.image import ImageProcessor

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.controller.goal_controller import GoalController

from tr_drive.persistent.recording import Recording


"""
    重复.
    
    is_ready():
        为 True 时方允许: 启动; 暂停; 继续; 处理 image 和 odom 消息.
        要求: recording 和 params 已初始化; devices 已初始化且 ready.
"""
class Repeater:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'repeater_debugger')
        self.recording_and_params_initialized = False
        
        # public
        self.recording: Recording = None
        self.repeating_launched = False
        self.repeating_paused = False
        
        self.passed_goal_index = 0 # the index of the nearest goal that has been passed
        self.goal_intervals: list[float] = [] # the intervals between goal i and goal i + 1
        self.goal_distances: list[float] = [] # the distances between goal 0 and goal i
        
        # parameters
        self.params = DictRegulator(rospy.get_param('/tr'))
        
        self.recording = Recording.from_path(self.params.persistent.recording_path)
        self.goal_distances.append(0.0)
        for idx in range(len(self.recording.odoms) - 1):
            d = (self.recording.odoms[idx + 1].t - self.recording.odoms[idx].t).norm()
            self.goal_intervals.append(d)
            self.goal_distances.append(self.goal_distances[-1] + d)
        
        self.params.camera.add('patch_size', self.recording.params['image']['patch_size'])
        self.params.camera.add('resize', self.recording.params['image']['resize'])
        self.params.camera.add('horizontal_fov', self.recording.params['image']['horizontal_fov'])
        self.recording_and_params_initialized = True
        
        # devices
        self.init_devices()
    
    def init_devices(self):
        self.camera = None
        self.odometry = None
        self.controller = None
        
        self.camera = Camera(
            raw_image_topic = self.params.camera.raw_image_topic,
            patch_size = self.params.camera.patch_size,
            resize = self.params.camera.resize,
            horizontal_fov = self.params.camera.horizontal_fov,
            processed_image_topic = self.params.camera.processed_image_topic
        )
        self.camera.wait_until_ready()
        
        self.odometry = Odom(
            odom_topic = self.params.odometry.odom_topic,
            processed_odom_topic = self.params.odometry.processed_odom_topic
        )
        self.camera.register_image_received_hook(self.image_received)
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
        self.controller.register_goal_reached_hook(self.goal_reached)
        self.controller.wait_until_ready()
    
    def is_ready(self):
        return \
            self.recording_and_params_initialized and \
            self.camera is not None and self.camera.is_ready() and \
            self.odometry is not None and self.odometry.is_ready() and \
            self.controller is not None and self.controller.is_ready()
    
    def start_repeating(self):
        if not self.is_ready() or self.repeating_launched:
            return False
        
        self.odometry.zeroize()
        self.controller.activate()
        
        self.passed_goal_index = 0 # TODO
        
        self.repeating_launched = True
        return True
    
    def pause_repeating(self):
        if not self.is_ready() or not self.repeating_launched or self.repeating_paused:
            return False
        
        self.controller.deactivate()
        self.repeating_paused = True
        return True
    
    def resume_repeating(self):
        if not self.is_ready() or not self.repeating_launched or not self.repeating_paused:
            return False
        
        self.controller.activate()
        self.repeating_paused = False
        return True
    
    def is_running(self):
        return self.repeating_launched and (not self.repeating_paused)
    
    def batched_match(self, image, indices):
        offsets = np.zeros(len(indices))
        values = np.zeros(len(indices))
        for i, idx in enumerate(indices):
            offsets[i], values[i] = ImageProcessor.best_match_offset(image, self.recording.processed_images[idx], self.params.repeater.match_offset_radius)
        return offsets, values

    def px_to_rad(self, px: int):
        return px / self.params.camera.resize[0] * self.params.camera.horizontal_fov / 180 * np.pi
    
    def image_received(self, **args):
        if not self.is_ready() or not self.is_running():
            return
        
        pass
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.is_running():
            return

        if self.passed_goal_index >= len(self.recording.odoms) - 1:
            rospy.loginfo('Finished.')
            self.pause_repeating() # TODO
            return
        
        T_0c: Frame = self.odometry.get_biased_odom() # args['odom']
        T_0a: Frame = self.recording.odoms[self.passed_goal_index]
        T_0b: Frame = self.recording.odoms[self.passed_goal_index + 1]
        T_ac = T_0a.I * T_0c
        d_ac = T_ac.t.norm()
        T_ab = T_0a.I * T_0b
        d_ab = T_ab.t.norm()
        T_cb = T_ac.I * T_ab
        
        i = self.passed_goal_index
        r = self.params.repeater.along_path_radius
        
        # along-path correction
        scan_indices = list(range(max(0, i - r + 1), min(len(self.recording.odoms), i + r + 1)))
        scan_distances = np.array([self.goal_distances[idx] for idx in scan_indices]) - self.goal_distances[i] - d_ac
        
        scan_offsets, scan_values = self.batched_match(self.camera.get_processed_image(), scan_indices)
        scan_values[scan_values < 0.1] = 0 # TODO, threshold
        along_path_error = scan_values / scan_values.sum() @ scan_distances
        
        # rotation correction
        theta_a = self.px_to_rad(scan_offsets[r - 1])
        theta_b = self.px_to_rad(scan_offsets[r])
        u = np.clip(d_ac / d_ab, 0, 1)
        rotation_error = (1 - u) * theta_a + u * theta_b
        
        # goal
        print(along_path_error, rotation_error)
        goal = T_cb * Frame.from_translation(T_ab.t * (-self.params.repeater.k_along_path * along_path_error)) # TODO, interpolation
        goal = Frame.from_z_rotation(self.params.repeater.k_rotation * rotation_error) * goal
        goal = T_0c * goal
        self.controller.set_goal(goal)
    
    def goal_reached(self, **args):
        self.passed_goal_index += 1


""" TODO ideas
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