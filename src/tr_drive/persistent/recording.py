import os
import cv2
import json

import rospy

from tr_drive.util.geometry import Frame, FrameList
from tr_drive.util.image import DigitalImage, DigitalImageList, ImageProcessor
from tr_drive.util.namespace import recursive_dict_update, get_filename_number, get_sorted_file_list

"""
结构:
    recording_name/
        parameters.json
        raw_image/
            0.jpg
            1.jpg
            ...
        processed_image/
            ...
        odom/
            0.json
            1.json
            ...
        ground_truth/
            ...
考虑内存占用问题, 要求:
    Teaching 过程中 raw_image 应对文件系统即时读写, 不完整存储在内存中;
    Teaching 过程中 processed_image, odom, ground_truth 可完整存储在内存中;
    Repeating 时不读取 raw_image;
    Repeating 加载时若出现参数不匹配, 重新处理时只需依次读取 raw_image, 不完整存储在内存中;
    Repeating 过程中 processed_image, odom, ground_truth 可完整存储在内存中.
注:
    该 Recording 类是特定于该应用的, 故此处可考虑写死各数据的文件夹名称, 不必再作为参数存取;
    图像参数存于录制数据的根目录下的 parameters.json 文件中而非在 processed_image 文件夹中;
        重新处理也需要用到 raw_image, 仅存参数于 processed_image 文件夹中并不合适.
样例 (录制):
    recording = Recording()
    recording.reset_binding_states(...)
    recording.set_image_parameters(...)
    recording.set_teacher_parameters(...)
    ... # start recording, 得到 path
    recording.bind_folder(path)
    ... # add data
    recording.odoms.append(...)
    ... # stop recording
    recording.to_file() # 将参数, 以及指定为内存模式的数据整体存储
样例 (重复):
    recording = Recording.from_file(path, ...) # 将参数, 以及指定为内存模式的数据读入
    ... # start repeating, use data
    ... xxx = recording.get_image_parameters() ... # 读取读取到的参数
    ... recording.odoms[...] ... # 使用读取到的数据
    ... # stop repeating
"""
class Recording:
    def __init__(self):
        self.params = {
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

        self.RAW_IMAGES_FOLDER = '/raw_image'
        self.PROCESSED_IMAGES_FOLDER = '/processed_image'
        self.ODOMS_FOLDER = '/odom'
        self.GROUND_TRUTHS_FOLDER = '/ground_truth'

        self.raw_images_instant_fileio = True
        self.processed_images_instant_fileio = False
        self.odoms_instant_fileio = False
        self.ground_truths_instant_fileio = False
        
        self.raw_images: DigitalImageList = DigitalImageList()
        self.processed_images: DigitalImageList = DigitalImageList()
        self.odoms: FrameList = FrameList()
        self.ground_truths: FrameList = FrameList()

        self.bound_folder = None
    
    @staticmethod
    def from_file(path, raw_images_ifio = True, processed_images_ifio = False, odoms_ifio = False, ground_truths_ifio = False):
        recording = Recording()
        recording.reset_binding_states(raw_images_ifio, processed_images_ifio, odoms_ifio, ground_truths_ifio)
        
        # 外存模式数据链接
        recording.bind_folder(path)

        # load parameters
        with open(path + '/parameters.json', 'r') as f:
            loaded_params = json.load(f)
        recording.params = recursive_dict_update(recording.params, loaded_params)

        # 内存模式数据读取 (不读取 raw_image, 除非参数不匹配需要重新处理)
        if not processed_images_ifio:
            recording.processed_images = DigitalImageList.from_file(path + recording.PROCESSED_IMAGES_FOLDER)
        if not odoms_ifio:
            recording.odoms = FrameList.from_file(path + recording.ODOMS_FOLDER)
        if not ground_truths_ifio:
            recording.ground_truths = FrameList.from_file(path + recording.GROUND_TRUTHS_FOLDER)
        if not recording.is_valid(): # reprocess if validation failed
            rospy.loginfo('Invalid recording data. Reprocessing raw images ...')
            recording.processed_images.clear()
            
            resize = recording.params['image']['resize']
            patch_size = recording.params['image']['patch_size']
            
            for filename in get_sorted_file_list(path + recording.RAW_IMAGES_FOLDER):
                if DigitalImageList.is_filename_valid(filename):
                    img = recording.raw_images[get_filename_number(filename)]
                    processed_img = ImageProcessor.kernel_normalize(img.interpolate(*resize).grayscale(), patch_size)
                    recording.processed_images.append(processed_img)
            
            recording.to_file()

        # 检查数据完整性 (TODO)
        if len(recording.processed_images) != len(recording.odoms):
            raise Exception('The number of processed images and odometry data does not match.')
        
        return recording

    def to_file(self):
        # create folder
        os.makedirs(self.bound_folder, exist_ok = True)

        # save parameters
        with open(self.bound_folder + '/parameters.json', 'w') as f:
            json.dump(self.params, f)
        
        # 内存模式数据存储 (外存模式数据已链接)
        if not self.raw_images_instant_fileio:
            self.raw_images.to_file(self.bound_folder + self.RAW_IMAGES_FOLDER)
        if not self.processed_images_instant_fileio:
            self.processed_images.to_file(self.bound_folder + self.PROCESSED_IMAGES_FOLDER)
        if not self.odoms_instant_fileio:
            self.odoms.to_file(self.bound_folder + self.ODOMS_FOLDER)
        if not self.ground_truths_instant_fileio:
            self.ground_truths.to_file(self.bound_folder + self.GROUND_TRUTHS_FOLDER)
    
    def reset_binding_states(self, raw_images_ifio = True, processed_images_ifio = False, odoms_ifio = False, ground_truths_ifio = False):
        self.raw_images_instant_fileio = raw_images_ifio
        self.processed_images_instant_fileio = processed_images_ifio
        self.odoms_instant_fileio = odoms_ifio
        self.ground_truths_instant_fileio = ground_truths_ifio
        if self.bound_folder is not None:
            self.bind_folder(self.bound_folder)
    
    def is_folder_bound(self):
        return self.bound_folder is not None

    def bind_folder(self, folder): # 绑定路径, 并绑定指定为外存模式的子数据路径
        self.bound_folder = folder
        self.raw_images.bind_folder(
            (folder + self.RAW_IMAGES_FOLDER) if self.raw_images_instant_fileio else None,
            clear_memory_data = False
        )
        self.processed_images.bind_folder(
            (folder + self.PROCESSED_IMAGES_FOLDER) if self.processed_images_instant_fileio else None,
            clear_memory_data = False
        )
        self.odoms.bind_folder(
            (folder + self.ODOMS_FOLDER) if self.odoms_instant_fileio else None,
            clear_memory_data = False
        )
        self.ground_truths.bind_folder(
            (folder + self.GROUND_TRUTHS_FOLDER) if self.ground_truths_instant_fileio else None,
            clear_memory_data = False
        )

    # def unbind_folder(self):
    #     self.bound_folder = None
    #     pass

    def get_image_parameters(self):
        return self.params['image']
    
    def get_teacher_parameters(self):
        return self.params['teacher']

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
    
    def is_valid(self): # TODO
        img = self.processed_images[0]
        return img.width == self.params['image']['resize'][0] and img.height == self.params['image']['resize'][1]

    def clear(self):
        self.raw_images.clear()
        self.processed_images.clear()
        self.odoms.clear()
        self.ground_truths.clear()

