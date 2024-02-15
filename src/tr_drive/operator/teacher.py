import time

import rospy

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera

from tr_drive.persistent.recording import Recording


"""
    示教.
    
    is_ready():
        为 True 时方允许: 重置; 启动; 继续; 处理 image 和 odom 消息.
        要求: recording 和 params 已初始化; devices 已初始化且 ready.
"""
class Teacher:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'teacher_debugger')
        self.params_initialized = False
        self.recording_initialized = False
        
        # public
        self.recording: Recording = None
        self.recording_launched = False
        
        # parameters
        self.params = DictRegulator(rospy.get_param('/tr'))
        self.params.persistent.add('auto_naming', self.params.persistent.recording_name.startswith('.'))
        self.params_initialized = True
        
        # devices
        self.init_devices()
        
        # recording
        self.init_recording()
        self.recording_initialized = True
    
    def init_devices(self):
        self.camera = None
        self.odometry = None
        
        self.camera = Camera(
            raw_image_topic = self.params.camera.raw_image_topic,
            patch_size = self.params.camera.patch_size,
            resize = self.params.camera.resize,
            horizontal_fov = self.params.camera.horizontal_fov,
            processed_image_topic = self.params.camera.processed_image_topic
        )
        self.camera.register_image_received_hook(self.image_received)
        self.camera.wait_until_ready()
        
        self.odometry = Odom(
            odom_topic = self.params.odometry.odom_topic,
            processed_odom_topic = self.params.odometry.processed_odom_topic
        )
        self.odometry.register_odom_received_hook(self.odom_received)
        self.odometry.wait_until_ready()
    
    def init_recording(self):
        self.recording = Recording()
        self.recording.set_raw_image_folder('/raw_image') # TODO
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
        return \
            self.params_initialized and \
            self.recording_initialized and \
            self.camera is not None and self.camera.is_ready() and \
            self.odometry is not None and self.odometry.is_ready()
    
    def reset_recording(self):
        if not self.is_ready():
            return False

        self.recording.clear()
        self.odometry.zeroize()
        self.recording_launched = False
        return True
    
    def start_recording(self):
        if not self.is_ready() or self.recording_launched:
            return False
        
        self.reset_recording()
        self.recording_launched = True
    
    def stop_recording(self):
        if not self.is_ready() or not self.recording_launched:
            return False
        
        path = self.params.persistent.recording_folder + '/' + (self.params.persistent.recording_name if not self.params.persistent.auto_naming else time.strftime('%Y-%m-%d_%H:%M:%S'))
        self.recording.to_path(path)
        self.recording_launched = False
        return True
    
    def image_received(self, **args):
        if not self.is_ready() or not self.recording_launched:
            return

        pass
    
    def odom_received(self, **args):
        if not self.is_ready() or not self.recording_launched:
            return
        
        current_odom = self.odometry.get_biased_odom()
        if len(self.recording.odoms) == 0 or self.recording.odoms[-1].yaw_difference(current_odom) >= self.params.teacher.rotation_threshold or self.recording.odoms[-1].translation_difference(current_odom) >= self.params.teacher.translation_threshold:
            self.recording.raw_images.append(self.camera.get_raw_image())
            self.recording.processed_images.append(self.camera.get_processed_image())
            self.recording.odoms.append(self.odometry.biased_odom)

