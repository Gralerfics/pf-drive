

from multinodes import Cable

from pf_drive.device.ros_camera import ROSCamera
from pf_drive.controller.ros_camera_test_controller import ROSCameraTestController


if __name__ == '__main__':
    camera = ROSCamera('camera', '/car/camera/image')
    controller = ROSCameraTestController('controller')

    cable = Cable(
        cable_type = 'shared_object',
        size = 1280 * 640 * 4 * 2,
        distributees = [
            (camera, 'image'),
            (controller, 'camera_image')
        ]
    )

    camera.start()
    controller.start()

    camera.join()
    controller.join()

