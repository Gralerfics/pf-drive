import time
import multiprocessing as mp

from multinodes import Cable

from pf_drive.device.ros_camera import ROSCameraWithResizeAndGrayscale
from pf_drive.controller.ros_camera_test_controller import ROSCameraTestController


if __name__ == '__main__':
    camera = ROSCameraWithResizeAndGrayscale('camera', '/car/camera/image', (150, 50))
    controller = ROSCameraTestController('controller')

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

