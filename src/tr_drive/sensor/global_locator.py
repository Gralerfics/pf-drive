import time
import threading

import rospy
import tf

from webots_ros.srv import supervisor_get_from_defRequest, supervisor_get_from_defResponse, supervisor_get_from_def
from webots_ros.srv import node_get_positionRequest, node_get_positionResponse, node_get_position, node_get_orientationRequest, node_get_orientationResponse, node_get_orientation

from tr_drive.util.debug import Debugger
from tr_drive.util.conversion import Vec3, Quat, Frame

from tr_drive.sensor.odometry import Odom


"""
    用于获取全局定位 (作为真值参考等).
    几种类型:
        topic 类型需提供 topic, topic_type, [odometry, fixed_frame];
        tf 类型需提供 fixed_frame, odometry;
        webots 类型需提供 robot_def, robot_name, odometry, [fixed_frame].
    
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
        self.global_frame_lock = threading.Lock()
        self.global_frame: Frame = None # frame_id = map
        
        # parameters
        self.LOCATOR_TYPE_TOPIC = 'topic'
        self.LOCATOR_TYPE_TF = 'tf'
        self.LOCATOR_TYPE_WEBOTS = 'webots'

        self.locator_type = locator_type
        self.params = kwargs
        
        # topics
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            # 参数: topic, topic_type, [odometry, fixed_frame]
            self.topic = self.params['topic']
            self.topic_type = self.params['topic_type']
            self.odometry: Odom = self.params['odometry'] if 'odometry' in self.params.keys() else None
            self.fixed_frame = self.params['fixed_frame'] if 'fixed_frame' in self.params.keys() else None

            # 用于订阅指定话题
            self.sub_topic = rospy.Subscriber(self.topic, self.topic_type, self.topic_type_cb, queue_size = 1) # TODO: queue_size

            # 用于发布 map to odom (optional)
            if self.fixed_frame is not None and self.odometry is not None:
                self.tf_broadcaster = tf.TransformBroadcaster()
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            # 参数: fixed_frame, odometry
            self.fixed_frame = self.params['fixed_frame']
            self.odometry: Odom = self.params['odometry']
            
            # 用于接收 TF 关系
            self.tf_listener = tf.TransformListener()

            # 以 odom 的接收进行回调
            self.odometry.register_odom_received_hook(self.tf_type_cb)
        elif self.locator_type == self.LOCATOR_TYPE_WEBOTS:
            # 参数: robot_def, robot_name, odometry, [fixed_frame]
            self.robot_def = self.params['robot_def']
            self.robot_name = self.params['robot_name']
            self.odometry: Odom = self.params['odometry'] #  if 'odometry' in self.params.keys() else None
            self.fixed_frame = self.params['fixed_frame'] if 'fixed_frame' in self.params.keys() else None
            
            # 定义服务
            SERVICE_GET_FROM_DEF = '/' + self.robot_name + '/supervisor/get_from_def'
            SERVICE_GET_POSITION = '/' + self.robot_name + '/supervisor/node/get_position'
            SERVICE_GET_ORIENTATION = '/' + self.robot_name + '/supervisor/node/get_orientation'
            rospy.wait_for_service(SERVICE_GET_FROM_DEF)
            rospy.wait_for_service(SERVICE_GET_POSITION)
            rospy.wait_for_service(SERVICE_GET_ORIENTATION)
            self.srv_get_from_def = rospy.ServiceProxy(SERVICE_GET_FROM_DEF, supervisor_get_from_def)
            self.srv_get_position = rospy.ServiceProxy(SERVICE_GET_POSITION, node_get_position)
            self.srv_get_orientation = rospy.ServiceProxy(SERVICE_GET_ORIENTATION, node_get_orientation)

            # 获取 robot 节点句柄 (node)
            try:
                request = supervisor_get_from_defRequest(name = self.robot_def, proto = 0)
                response = self.srv_get_from_def(request)
                self.node_handler = response.node # 获取 node 句柄
            except rospy.ServiceException as e: # TODO
                rospy.logerr('Service call get_from_def failed.')

            # 用于发布 map to odom (optional)
            if self.fixed_frame is not None and self.odometry is not None:
                self.tf_broadcaster = tf.TransformBroadcaster()

            # 以 odom 的接收进行回调 (暂定, 由此导致 odometry 必选)
            if self.odometry is not None:
                self.odometry.register_odom_received_hook(self.webots_type_cb)
        else:
            raise ValueError('Invalid locator type.')
    
    def __del__(self):
        if self.locator_type == self.LOCATOR_TYPE_TOPIC:
            self.sub_topic.unregister()
        elif self.locator_type == self.LOCATOR_TYPE_TF:
            pass
        elif self.locator_type == self.LOCATOR_TYPE_WEBOTS:
            pass
        else:
            raise ValueError('Invalid locator type.')
    
    def publish_map_frame(self, odom_frame: Frame, global_frame: Frame, odom_frame_id = 'odom'): # TODO: to be fixed
        # 由里程计信息和全局信息得到 /map 和 /odom 关系并发布; 此处 odom_frame 由 get_odom() 所获, frame_id 为 odom.
        T_map_odom = global_frame * odom_frame.I
        self.tf_broadcaster.sendTransform(
            T_map_odom.t.to_list(),
            T_map_odom.q.to_list(),
            rospy.Time.now(),
            odom_frame_id,
            self.fixed_frame
        )

    def call_hooks(self):
        if self.is_ready(): # 保证自身 ready 后再执行回调函数.
            for hook in self.global_frame_received_hooks:
                hook()
                # hook(global_frame = self.get_global_frame())
    
    def topic_type_cb(self, msg): # topic 模式
        if self.odometry is not None and not self.odometry.is_ready():
            return
        
        # 订阅话题直接获取全局坐标
        with self.global_frame_lock:
            self.global_frame = Frame(msg) # TODO: check msg type
        
        # 计算并发布 map -> odom 变换
        odom_frame_id = self.odometry.get_odom_msg().header.frame_id
        if self.fixed_frame is not None and self.odometry is not None:
            odom_frame = self.odometry.get_odom()
            self.publish_map_frame(odom_frame, self.global_frame, odom_frame_id)
        
        # 调用回调函数
        self.call_hooks()
    
    def tf_type_cb(self): # tf 模式
        if self.odometry is not None and not self.odometry.is_ready():
            return
        
        # 获取 map -> odom 的变换, 计算全局坐标
        odom_frame_id = self.odometry.get_odom_msg().header.frame_id
        odom_frame = self.odometry.get_odom()
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.fixed_frame, odom_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        frame_map_odom = Frame(Vec3(trans), Quat(rot)) # map -> odom
        with self.global_frame_lock:
            self.global_frame = frame_map_odom * odom_frame
        
        # 调用回调函数
        self.call_hooks()
    
    def webots_type_cb(self): # webots 模式
        if self.odometry is not None and not self.odometry.is_ready():
            return
        
        # 通过服务获取全局坐标
        try:
            position_request = node_get_positionRequest(node = self.node_handler)
            position_response = self.srv_get_position(position_request)
            t = Vec3(position_response.position.x, position_response.position.y, position_response.position.z)
            orientation_request = node_get_orientationRequest(node = self.node_handler)
            orientation_response = self.srv_get_orientation(orientation_request)
            q = Quat(orientation_response.orientation.x, orientation_response.orientation.y, orientation_response.orientation.z, orientation_response.orientation.w)
            with self.global_frame_lock:
                self.global_frame = Frame(t, q)
        except rospy.ServiceException as e: # TODO
            rospy.logerr('Service call node_get_position or node_get_orientation failed.')
        
        # 计算并发布 map -> odom 变换
        odom_frame_id = self.odometry.get_odom_msg().header.frame_id
        if self.fixed_frame is not None and self.odometry is not None:
            odom_frame = self.odometry.get_odom()
            self.publish_map_frame(odom_frame, self.global_frame, odom_frame_id)

        # 调用回调函数
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
    
    def get_global_frame_id(self): # 无则 None
        return self.fixed_frame if hasattr(self, 'fixed_frame') else None
    
    def get_global_frame(self):
        with self.global_frame_lock:
            return self.global_frame

