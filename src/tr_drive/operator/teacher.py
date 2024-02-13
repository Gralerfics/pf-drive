import time

import rospy

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera

from tr_drive.persistent.recording import Recording


# ready 表示 __init__ 是否完成
class Teacher:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'teacher_debugger')
        self.ready = False
        
        # public
        self.recording: Recording = None
        self.recording_launched = False
        
        # parameters
        self.params = DictRegulator(rospy.get_param('/tr'))
        self.params.persistent.add('auto_naming', self.params.persistent.recording_name.startswith('.'))
        
        # devices
        self.init_devices()
        
        # recording
        self.init_recording()
        
        self.ready = True
    
    def init_devices(self):
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
    
    def reset_recording(self):
        self.recording.clear()
        self.odometry.zeroize()
        self.recording_launched = False
    
    def start_recording(self):
        if self.recording_launched:
            return # TODO
        self.reset_recording()
        self.recording_launched = True
    
    def stop_recording(self):
        if not self.recording_launched:
            return
        path = self.params.persistent.recording_folder + '/' + (self.params.persistent.recording_name if not self.params.persistent.auto_naming else time.strftime('%Y-%m-%d_%H:%M:%S'))
        self.recording.to_path(path)
        self.recording_launched = False
    
    def image_received(self, **args):
        if self.ready:
            if self.recording_launched:
                pass
    
    def odom_received(self, **args):
        if self.ready:
            if self.recording_launched:
                current_odom = self.odometry.get_biased_odom()
                if len(self.recording.odoms) == 0 or self.recording.odoms[-1].yaw_difference(current_odom) >= self.params.teacher.rotation_threshold or self.recording.odoms[-1].translation_difference(current_odom) >= self.params.teacher.translation_threshold:
                    self.recording.raw_images.append(self.camera.get_raw_image())
                    self.recording.processed_images.append(self.camera.get_processed_image())
                    self.recording.odoms.append(self.odometry.biased_odom)

