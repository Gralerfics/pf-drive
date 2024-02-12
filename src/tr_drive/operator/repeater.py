import rospy

from tr_drive.util.debug import Debugger
from tr_drive.util.namespace import DictRegulator
from tr_drive.util.conversion import Frame

from tr_drive.sensor.odometry import Odom
from tr_drive.sensor.camera import Camera
from tr_drive.controller.goal_controller import GoalController

from tr_drive.persistent.recording import Recording


class Repeater:
    def __init__(self):
        # private
        self.debugger: Debugger = Debugger(name = 'repeater_debugger')
        self.ready = False
        
        # public
        self.recording: Recording = None
        
        # parameters
        self.params = DictRegulator(rospy.get_param('/tr'))
        self.recording = Recording.from_path(self.params.persistent.recording_path)
        self.params.camera.add('patch_size', self.recording.params['image']['patch_size'])
        self.params.camera.add('resize', self.recording.params['image']['resize'])
        self.params.camera.add('horizontal_fov', self.recording.params['image']['horizontal_fov'])
        print(len(self.recording.raw_images))
        print(len(self.recording.processed_images))
        print(len(self.recording.odoms))
        
        # devices
        self.init_devices()
        
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
        
        self.controller = GoalController(
            cmd_vel_topic = self.params.controller.cmd_vel_topic
        )
        self.controller.wait_until_ready()
    
    def image_received(self, **args):
        if self.ready:
            pass
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

    def odom_received(self, **args):
        if self.ready:
            pass


# TODO: idea: 金字塔匹配辅助确认距离; 互相关加权，倾向小角度;