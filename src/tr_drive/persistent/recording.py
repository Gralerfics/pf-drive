import os
import cv2
import json

from tr_drive.util.conversion import Frame
from tr_drive.util.image import DigitalImage


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
                'raw_image': None,
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
    def validate(path):
        return True # TODO
    
    @staticmethod
    def from_path(path, read_raw = False): # with name, TODO: index check
        recording = Recording()
        with open(path + '/parameters.json', 'r') as f:
            recording.params = json.load(f)
        
        # load raw images
        if read_raw and recording.params['folders']['raw_image'] is not None:
            # TODO: exception
            raw_image_folder = path + recording.params['folders']['raw_image']
            for filename in sorted(os.listdir(raw_image_folder)):
                if filename.endswith('.jpg'):
                    img_cv2 = cv2.imread(raw_image_folder + '/' + filename, cv2.IMREAD_GRAYSCALE)
                    img = DigitalImage(img_cv2)
                    recording.raw_images.append(img)
        
        # load processed images
        processed_image_folder = path + recording.params['folders']['processed_image']
        for filename in sorted(os.listdir(processed_image_folder)):
            if filename.endswith('.jpg'):
                img_cv2 = cv2.imread(processed_image_folder + '/' + filename, cv2.IMREAD_GRAYSCALE)
                img = DigitalImage(img_cv2)
                recording.processed_images.append(img)
        
        # load odometry
        odom_folder = path + recording.params['folders']['odom']
        for filename in sorted(os.listdir(odom_folder)):
            if filename.endswith('.json'):
                with open(odom_folder + '/' + filename, 'r') as f:
                    odom_dict = json.load(f)
                    recording.odoms.append(Frame.from_dict(odom_dict))
        
        return recording
    
    def to_path(self, path): # with name, TODO: overwrite check
        os.makedirs(path, exist_ok = False)
        
        # save parameters, TODO: exception for None values
        with open(path + '/parameters.json', 'w') as f:
            json.dump(self.params, f)
        
        # save raw images
        if self.params['folders']['raw_image'] is not None:
            if self.params['folders']['raw_image'] is not None:
                raw_image_folder = path + self.params['folders']['raw_image']
                os.makedirs(raw_image_folder, exist_ok = False)
                for i, img in enumerate(self.raw_images):
                    img.to_jpg(raw_image_folder + '/' + str(i).zfill(self.ZERO_FILL_LENGTH) + '.jpg')
        
        # save processed images
        processed_image_folder = path + self.params['folders']['processed_image']
        os.makedirs(processed_image_folder, exist_ok = False)
        for i, img in enumerate(self.processed_images):
            img.to_jpg(processed_image_folder + '/' + str(i).zfill(self.ZERO_FILL_LENGTH) + '.jpg')
        
        # save odometry
        odom_folder = path + self.params['folders']['odom']
        os.makedirs(odom_folder, exist_ok = False)
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
    
    def reprocess(self):
        pass # TODO

