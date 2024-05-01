import time
import signal
import multiprocessing as mp

from multinodes import Cable

from pf_drive.controller.keyboard_ackermann_controller import KeyboardAckermannController
from pf_drive.actuator.webots_ros_ackermann_actuator import WebotsROSAckermannActuator


is_shutdown = mp.Event()
def sigint_handler(sig, frame):
    print('Interrupted.')
    is_shutdown.set()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    
    controller = KeyboardAckermannController('controller', is_shutdown)
    actuator = WebotsROSAckermannActuator('actuator', is_shutdown,
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

    cable = Cable(
        cable_type = 'pipe',
        latest = True,
        distributees = [
            (controller, 'actuator_command'),
            (actuator, 'command')
        ]
    )

    controller.start()
    actuator.start()

    controller.join()
    actuator.join()

