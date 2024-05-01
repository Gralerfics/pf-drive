import rospy

from webots_ros.srv import get_bool, get_boolRequest, get_boolResponse
from webots_ros.srv import get_float, get_floatRequest, get_floatResponse
from webots_ros.srv import set_float, set_floatRequest, set_floatResponse

from multinodes import Node


class WebotsRotationalMotorController:
    INFINITY = float('inf')

    def __init__(self, motor_name, namespace = ''):
        # TODO: service reset when resetting simulation. 不过看起来服务句柄不会丢失.
        self.SERVICE_SET_POSITION = namespace + '/' + motor_name + '/set_position'
        self.SERVICE_SET_VELOCITY = namespace + '/' + motor_name + '/set_velocity'
        self.SERVICE_SET_TORQUE = namespace + '/' + motor_name + '/set_torque'
        rospy.wait_for_service(self.SERVICE_SET_POSITION)
        rospy.wait_for_service(self.SERVICE_SET_VELOCITY)
        rospy.wait_for_service(self.SERVICE_SET_TORQUE)
        self.srv_set_position = rospy.ServiceProxy(self.SERVICE_SET_POSITION, set_float)
        self.srv_set_velocity = rospy.ServiceProxy(self.SERVICE_SET_VELOCITY, set_float)
        self.srv_set_torque = rospy.ServiceProxy(self.SERVICE_SET_TORQUE, set_float)
    
    def set_position(self, position):
        try:
            request = set_floatRequest(value = position)
            response = self.srv_set_position(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set position failed: %s' % e)
    
    def set_velocity(self, velocity):
        try:
            self.set_position(self.INFINITY)
            request = set_floatRequest(value = velocity)
            response = self.srv_set_velocity(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set velocity failed: %s' % e)
    
    def set_torque(self, torque):
        try:
            request = set_floatRequest(value = torque)
            response = self.srv_set_torque(request)
        except rospy.ServiceException as e:
            rospy.logerr('Set torque failed: %s' % e)


# class WebotsROSAckermannActuator(Node):
    

