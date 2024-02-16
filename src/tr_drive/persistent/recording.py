import os
import cv2
import json

import rospy

from tr_drive.util.conversion import Frame
from tr_drive.util.image import DigitalImage, ImageProcessor


# recording_name/
#     parameters.json
#     raw_image/
#         000000.jpg
#         000001.jpg
#         ...
#     processed_image/
#         ...
#     odom/
#         000000.json
#         ...
class Recording:
    def __init__(self, params = {}):
        self.params = {
            'folders': {
                'raw_image': '/raw_image',
                'processed_image': '/processed_image',
                'odom': '/odom'
            },
            'image': {
                'raw_size': None, # [width, height]
                'patch_size': None,
                'resize': None, # [width, height]
                'horizontal_fov': None
            },
            'teacher': {
                'rotation_threshold': None,
                'translation_threshold': None
            }
        }
        
        self.raw_images: list[DigitalImage] = []
        self.processed_images: list[DigitalImage] = []
        self.odoms: list[Frame] = []
        
        self.ZERO_FILL_LENGTH = 6 # temporary
    
    @staticmethod
    def from_path(path): # with name, TODO: index check
        recording = Recording()
        with open(path + '/parameters.json', 'r') as f:
            recording.params = json.load(f)
        valid = True
        
        # load processed images
        processed_image_folder = path + recording.params['folders']['processed_image']
        for filename in sorted(os.listdir(processed_image_folder)):
            if filename.endswith('.jpg'):
                img_cv2 = cv2.imread(processed_image_folder + '/' + filename, cv2.IMREAD_GRAYSCALE)
                img = DigitalImage(img_cv2)
                if img.width != recording.params['image']['resize'][0] or img.height != recording.params['image']['resize'][1]:
                    valid = False
                    break
                recording.processed_images.append(img)
        
        # load odometry
        odom_folder = path + recording.params['folders']['odom']
        for filename in sorted(os.listdir(odom_folder)):
            if filename.endswith('.json'):
                with open(odom_folder + '/' + filename, 'r') as f:
                    odom_dict = json.load(f)
                    recording.odoms.append(Frame.from_dict(odom_dict))
        
        # load raw images and reprocess if validation failed
        if not valid:
            rospy.loginfo('Invalid recording data. Reprocessing raw images ...')
            recording.processed_images.clear()
            
            raw_image_folder = path + recording.params['folders']['raw_image']
            resize = recording.params['image']['resize']
            patch_size = recording.params['image']['patch_size']
            
            for filename in sorted(os.listdir(raw_image_folder)):
                if filename.endswith('.jpg'):
                    img_cv2 = cv2.imread(raw_image_folder + '/' + filename, cv2.IMREAD_COLOR)
                    img = DigitalImage(img_cv2)
                    recording.raw_images.append(img) # TODO: can be removed?
                    
                    processed_img = ImageProcessor.kernel_normalize(img.interpolate(*resize).grayscale(), patch_size)
                    recording.processed_images.append(processed_img)
            
            recording.to_path(path) # overwrite
            valid = True

        if len(recording.processed_images) != len(recording.odoms): # TODO
            raise Exception('The number of processed images and odometry data does not match.')
        
        return recording
    
    def to_path(self, path): # with name, TODO: overwrite check
        os.makedirs(path, exist_ok = True)
        
        # save parameters, TODO: exception for None values
        with open(path + '/parameters.json', 'w') as f:
            json.dump(self.params, f)
        
        # save raw images
        if self.params['folders']['raw_image'] is not None:
            if self.params['folders']['raw_image'] is not None:
                raw_image_folder = path + self.params['folders']['raw_image']
                os.makedirs(raw_image_folder, exist_ok = True)
                for i, img in enumerate(self.raw_images):
                    img.to_jpg(raw_image_folder + '/' + str(i).zfill(self.ZERO_FILL_LENGTH) + '.jpg')
        
        # save processed images
        processed_image_folder = path + self.params['folders']['processed_image']
        os.makedirs(processed_image_folder, exist_ok = True)
        for i, img in enumerate(self.processed_images):
            img.to_jpg(processed_image_folder + '/' + str(i).zfill(self.ZERO_FILL_LENGTH) + '.jpg')
        
        # save odometry
        odom_folder = path + self.params['folders']['odom']
        os.makedirs(odom_folder, exist_ok = True)
        for i, odom in enumerate(self.odoms):
            with open(odom_folder + '/' + str(i).zfill(self.ZERO_FILL_LENGTH) + '.json', 'w') as f:
                json.dump(odom.to_dict(), f)
    
    def set_raw_image_folder(self, folder): # pass None to disable
        self.params['folders']['raw_image'] = folder
    
    def set_image_parameters(self, raw_size, patch_size, resize, horizontal_fov):
        self.params['image'] = {
            'raw_size': raw_size,
            'patch_size': patch_size,
            'resize': resize,
            'horizontal_fov': horizontal_fov
        }
    
    def set_teacher_parameters(self, rotation_threshold, translation_threshold):
        self.params['teacher'] = {
            'rotation_threshold': rotation_threshold,
            'translation_threshold': translation_threshold
        }
    
    def clear(self):
        self.raw_images.clear()
        self.processed_images.clear()
        self.odoms.clear()

