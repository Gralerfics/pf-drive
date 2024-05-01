import time
import signal
import multiprocessing as mp

from multinodes import Cable

from pf_drive.device.ros_camera import ROSCameraWithResizeAndGrayscale
from pf_drive.controller.ros_camera_test_controller import ROSCameraTestController


is_shutdown = mp.Event()
def sigint_handler(sig, frame):
    print('Interrupted.')
    is_shutdown.set()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    
    camera = ROSCameraWithResizeAndGrayscale('camera', is_shutdown, '/car/camera/image', (150, 50))
    controller = ROSCameraTestController('controller', is_shutdown)

    cable = Cable(
        cable_type = 'shared_object',
        size = 150 * 50 * 1 + 1000,
        distributees = [
            (camera, 'image'),
            (controller, 'camera_image')
        ]
    )

    # cable = Cable(
    #     cable_type = 'queue',
    #     distributees = [
    #         (camera, 'image'),
    #         (controller, 'camera_image')
    #     ]
    # )

    # cable = Cable(
    #     cable_type = 'pipe',
    #     distributees = [
    #         (camera, 'image'),
    #         (controller, 'camera_image')
    #     ]
    # )

    camera.start()
    controller.start()

    camera.join()
    controller.join()

