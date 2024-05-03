import time
import multiprocessing as mp

from nav_msgs.msg import Odometry

from multinodes import Cable

from pf_drive.util import t3d_ext
from pf_drive.util import ROSContext
from pf_drive.controller.keyboard_ackermann_controller import KeyboardAckermannController
from pf_drive.actuator.webots_ros_ackermann_actuator import WebotsROSAckermannActuator


if __name__ == '__main__':
    controller = KeyboardAckermannController('controller')
    actuator = WebotsROSAckermannActuator('actuator',
        '/car/left_front_steer_motor',
        '/car/right_front_steer_motor',
        '/car/left_rear_motor',
        '/car/right_rear_motor',
        '/car/robot/get_time',
        1.628,
        2.995,
        0.38,
        0.78
    )

    cable_actuator_command = Cable(
        cable_type = 'pipe',
        latest = True,
        distributees = [
            (controller, 'actuator_command'),
            (actuator, 'command')
        ]
    )

    cable_odom = Cable(
    #     cable_type = 'shared_object',
    #     size = 300,
        cable_type = 'pipe',
        latest = True,
        distributees = [
            (actuator, 'odom')
        ]
    )

    controller.start()
    actuator.start()

    ros = ROSContext('main_test_ackermann')
    ros.init_node(anonymous = False)
    
    t = time.time()
    while not ros.is_shutdown():
        if cable_odom.poll():
            odom = cable_odom.read()
            ros.publish_topic('/car/odom', t3d_ext.e2O(odom, frame_id = 'odom', stamp = ros.time()), queue_size = 1)
    
    # controller.join()
    # actuator.join()

