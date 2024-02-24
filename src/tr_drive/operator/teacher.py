import time
import threading

import rospy

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator
from tr_drive.util.conversion import Frame, type_from_str

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.sensor.global_locator import GlobalLocator

from tr_drive.persistent.recording import Recording


"""
    示教.
    
    is_ready():
        为 True 时方允许: 重置; 启动; 继续; 处理 image 和 odom 消息.
        要求: recording 和 params 已初始化; devices 对象存在, 不为 None 且 ready.
"""
class Teacher:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'teacher_debugger')
        
        self.ready = threading.Event()
        self.launched = threading.Event()
        
        # public
        self.recording: Recording = None
        
        # parameters
        self.params = DictRegulator(rospy.get_param('/tr'))
        self.params.persistent.add('auto_naming', self.params.persistent.recording_name.startswith('.'))
        self.global_locator_used = 'global_locator' in self.params # 是否引入全局定位信息.
        
        # devices
        self.init_devices()
        
        # recording
        self.init_recording()
        
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
                    
                    robot_def = self.params.global_locator.robot_def,
                    robot_name = self.params.global_locator.robot_name
                )
            # self.global_locator.register_global_frame_received_hook(self.global_frame_received)
            self.global_locator.wait_until_ready()
    
    def init_recording(self):
        self.recording = Recording()
        self.recording.set_image_parameters(
            raw_size = [self.camera.get_raw_image().width, self.camera.get_raw_image().height],
            patch_size = self.camera.patch_size,
            resize = self.camera.resize,
            horizontal_fov = self.camera.horizontal_fov
        )
        self.recording.set_teacher_parameters(
            rotation_threshold = self.params.teacher.rotation_threshold,
            translation_threshold = self.params.teacher.translation_threshold
        )
    
    def is_ready(self):
        return self.ready.is_set()
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for teacher ...')
            time.sleep(0.2)
        rospy.loginfo('Teacher is ready.')
    
    def reset_recording(self):
        if not self.is_ready():
            return False

        self.recording.clear()
        self.odometry.zeroize()
        self.launched.clear()
        return True
    
    def start_recording(self):
        if not self.is_ready() or self.launched.is_set():
            return False
        
        self.reset_recording()
        self.launched.set()
    
    def stop_recording(self):
        if not self.is_ready() or self.launched.is_set():
            return False
        
        path = self.params.persistent.recording_folder + '/' + (self.params.persistent.recording_name if not self.params.persistent.auto_naming else time.strftime('%Y-%m-%d_%H:%M:%S'))
        self.recording.to_path(path)
        self.launched.clear()
        return True
    
    def difference_under_threshold(self, frame_1: Frame, frame_2: Frame):
        return \
            frame_1.yaw_difference(frame_2) < self.params.teacher.rotation_threshold and \
            frame_1.translation_difference(frame_2) < self.params.teacher.translation_threshold
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.launched.is_set():
            return
        
        odom = self.odometry.get_biased_odom()
        if len(self.recording.odoms) == 0 or not self.difference_under_threshold(self.recording.odoms[-1], odom):
            # raw_image
            if self.params.teacher.save_raw_images:
                self.recording.raw_images.append(self.camera.get_raw_image())
            
            # processed_image
            self.recording.processed_images.append(self.camera.get_processed_image())
            
            # odom
            self.recording.odoms.append(odom)
            
            # ground_truth
            if self.params.teacher.save_ground_truth_odoms:
                if self.global_locator_used:
                    global_frame = self.global_locator.get_global_frame()
                    if global_frame:
                        self.recording.ground_truths.append(global_frame)

