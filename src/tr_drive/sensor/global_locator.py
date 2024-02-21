import time
import threading

import rospy
import tf

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Vec3, Quat, Frame

from tr_drive.sensor.odometry import Odom


"""
    用于获取全局定位 (作为真值参考等).
    两种类型: topic, tf:
        前者从 Odometry, Pose 等类型的话题直接获取, 后者从 tf 树中获取.
        前者还需提供 topic, topic_type; 后者还需提供 fixed_frame, odometry.
    
    register_x_hook():
        注册新的回调函数到列表.
    
    is_ready():
        为 True 时方允许: 获取 global_frame; 执行回调函数.
        要求: global_frame 不为 None.
    
    get_x():
        线程安全地获取成员.
"""
class GlobalLocator:
    def __init__(self,
        locator_type: str,
        **kwargs
    ):
        # private
        self.debugger: Debugger = Debugger(name = 'global_locator_debugger')
        self.global_frame_received_hooks: list = []
        
        # public
        self.global_frame = None
        self.global_frame_lock = threading.Lock()
        
        # parameters
        self.LOCATOR_TYPE_TOPIC = 'topic'
        self.LOCATOR_TYPE_TF = 'tf'
        self.locator_type = locator_type
        self.params = kwargs
        
        # topics
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            self.topic = self.params['topic']
            self.topic_type = self.params['topic_type']
            self.sub_topic = rospy.Subscriber(self.topic, self.topic_type, self.topic_type_cb, queue_size = 1) # TODO: queue_size
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            self.fixed_frame = self.params['fixed_frame']
            self.odometry: Odom = self.params['odometry']
            self.tf_listener = tf.TransformListener()
            self.odometry.register_odom_received_hook(self.tf_type_cb)
        else:
            raise ValueError('Invalid locator type.')
    
    def __del__(self):
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            self.sub_topic.unregister()
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            pass
        else:
            raise ValueError('Invalid locator type.')
    
    def call_hooks(self):
        if self.is_ready(): # 保证自身 ready 后再执行回调函数.
            for hook in self.global_frame_received_hooks:
                hook()
                # hook(global_frame = self.get_global_frame())
    
    def topic_type_cb(self, msg): # topic 模式
        # TODO: check Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Odometry
        with self.global_frame_lock:
            self.global_frame = Frame(msg)
        
        self.call_hooks()
    
    def tf_type_cb(self): # tf 模式
        if not self.odometry.is_ready(): # 保证 odometry 已 ready.
            return
        
        odom_frame_id = self.odometry.get_odom_msg().header.frame_id
        odom_frame = self.odometry.get_odom()
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.fixed_frame, odom_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        frame_map_odom = Frame(Vec3(trans), Quat(rot)) # map -> odom 的变换
        
        with self.global_frame_lock:
            self.global_frame = frame_map_odom * odom_frame
        
        self.call_hooks()
    
    def register_global_frame_received_hook(self, hook):
        self.global_frame_received_hooks.append(hook)
        
    def is_ready(self):
        with self.global_frame_lock:
            return self.global_frame is not None
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for global locator ...')
            time.sleep(0.2)
        rospy.loginfo('Global locator is ready.')
    
    def get_global_frame(self):
        with self.global_frame_lock:
            return self.global_frame

