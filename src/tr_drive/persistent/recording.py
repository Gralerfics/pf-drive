import os
import json

import rospy

from tr_drive.util.geometry import FrameList
from tr_drive.util.image import DigitalImageList, ImageProcessor
from tr_drive.util.namespace import recursive_dict_update

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
    对于 Recording, 如果从文件读取则自动绑定该目录, 主要是为了给 bound_folder 赋值.
    对于 raw_images 和 ground_truths 这种设计上可选的数据, 提供 xxx_available() 方法检查可用性, 并在读取时允许不存在.
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
    recording.to_file() # 将参数, 以及指定为内存模式的数据整体存储; 此时若已绑定目录, to_file 的参数将被无视
    recording.unbind_folder() # optional; 解绑, 清除内存数据
样例 (重复):
    recording = Recording.from_file(path, ...) # 将参数, 以及指定为内存模式的数据读入
    ... # start repeating, use data
    ... xxx = recording.get_image_parameters() ... # 读取读取到的参数
    ... recording.odoms[...] ... # 使用读取到的数据
    ... # stop repeating
"""
class Recording:
    def __init__(self, bound_folder = None):
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
        self.odoms: FrameList = FrameList() # TODO: 相关变量目录等改名 biased_odom
        self.ground_truths: FrameList = FrameList()

        self.bind_folder(bound_folder) # 可传入 None
    
    def __len__(self):
        return len(self.odoms)
    
    @staticmethod
    def from_file(folder_path: str, raw_images_ifio = True, processed_images_ifio = False, odoms_ifio = False, ground_truths_ifio = False):
        if not os.path.exists(folder_path):
            raise FileNotFoundError("Folder not found.")
        
        recording = Recording(bound_folder = folder_path)
        recording.reset_binding_states(raw_images_ifio, processed_images_ifio, odoms_ifio, ground_truths_ifio)
        
        # 读取参数
        with open(folder_path + '/parameters.json', 'r') as f:
            loaded_params = json.load(f)
        recording.params = recursive_dict_update(recording.params, loaded_params)

        # 数据读取 (统一传入模式参数, 不调用 bind_folder 绑定, 而是使用 from_file)
        recording.raw_images = DigitalImageList.from_file(
            folder_path + recording.RAW_IMAGES_FOLDER,
            ifio = recording.raw_images_instant_fileio,
            allow_not_existed = True
        )
        recording.processed_images = DigitalImageList.from_file(
            folder_path + recording.PROCESSED_IMAGES_FOLDER,
            ifio = recording.processed_images_instant_fileio,
            allow_not_existed = False
        )
        recording.odoms = FrameList.from_file(
            folder_path + recording.ODOMS_FOLDER,
            ifio = recording.odoms_instant_fileio,
            allow_not_existed = False
        )
        recording.ground_truths = FrameList.from_file(
            folder_path + recording.GROUND_TRUTHS_FOLDER,
            ifio = recording.ground_truths_instant_fileio,
            allow_not_existed = True
        )

        # 参数不匹配重新处理
        if not recording.is_valid():
            rospy.loginfo('Invalid recording data. Reprocessing raw images ...')
            if not recording.is_raw_images_available():
                raise Exception('Insufficient raw images.')
            
            recording.processed_images.clear()
            
            resize = recording.params['image']['resize']
            patch_size = recording.params['image']['patch_size']
            
            for i in range(len(recording.raw_images)):
                img = recording.raw_images[i]
                processed_img = ImageProcessor.kernel_normalize(img.interpolate(*resize).to_grayscale(), patch_size)
                recording.processed_images.append(processed_img)
            
            recording.processed_images.to_file(folder_path = folder_path + recording.PROCESSED_IMAGES_FOLDER)

        # 检查数据完整性
        if not recording.is_available():
            raise Exception('Invalid data.')
        
        return recording

    def to_file(self, folder_path: str = None):
        # 纯内存模式 (未绑定目录) 下采用提供路径, 若 recording 已绑定目录则无视 folder_path
        if self.is_folder_bound():
            folder_path = self.bound_folder
        assert folder_path is not None
        
        # 存储参数
        with open(folder_path + '/parameters.json', 'w') as f:
            json.dump(self.params, f)
        
        # 数据存储 (两种模式统一, 外存模式下传入的路径会被忽略)
        self.raw_images.to_file(folder_path + self.RAW_IMAGES_FOLDER)
        self.processed_images.to_file(folder_path + self.PROCESSED_IMAGES_FOLDER)
        self.odoms.to_file(folder_path + self.ODOMS_FOLDER)
        self.ground_truths.to_file(folder_path + self.GROUND_TRUTHS_FOLDER)
    
    def reset_binding_states(self, raw_images_ifio = True, processed_images_ifio = False, odoms_ifio = False, ground_truths_ifio = False):
        # 变更子数据目录绑定状态
        self.raw_images_instant_fileio = raw_images_ifio
        self.processed_images_instant_fileio = processed_images_ifio
        self.odoms_instant_fileio = odoms_ifio
        self.ground_truths_instant_fileio = ground_truths_ifio

        # 已 bind_folder 的重新 bind_folder, 以应用新的状态, 或调用本方法后再 bind_folder
        if self.bound_folder is not None:
            self.bind_folder(self.bound_folder)
    
    def is_folder_bound(self):
        return self.bound_folder is not None

    def bind_folder(self, folder_path: str): # 实际进行路径绑定, 并绑定指定为外存模式的子数据路径
        self.bound_folder = folder_path
        if folder_path is not None:
            os.makedirs(folder_path, exist_ok = True) # 保证有绑定即有目录, 下面的子数据目录也是如此

            # 外存模式子数据目录绑定, 若为内存模式则传入 None
            self.raw_images.bind_folder(
                (folder_path + self.RAW_IMAGES_FOLDER) if self.raw_images_instant_fileio else None,
                clear_memory_data = False
            )
            self.processed_images.bind_folder(
                (folder_path + self.PROCESSED_IMAGES_FOLDER) if self.processed_images_instant_fileio else None,
                clear_memory_data = False
            )
            self.odoms.bind_folder(
                (folder_path + self.ODOMS_FOLDER) if self.odoms_instant_fileio else None,
                clear_memory_data = False
            )
            self.ground_truths.bind_folder(
                (folder_path + self.GROUND_TRUTHS_FOLDER) if self.ground_truths_instant_fileio else None,
                clear_memory_data = False
            )
    
    def unbind_folder(self): # 解除路径绑定, 并清除所有数据内存数据
        if self.is_folder_bound():
            self.raw_images.unbind_folder()
            self.processed_images.unbind_folder()
            self.odoms.unbind_folder()
            self.ground_truths.unbind_folder()
            self.clear() # 解除绑定后清除的是内存数据
            self.bound_folder = None

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
    
    def is_available(self):
        return len(self.odoms) > 0 and len(self.odoms) == len(self.processed_images)
    
    def is_raw_images_available(self):
        return len(self.odoms) == len(self.raw_images)
    
    def is_ground_truths_available(self):
        return len(self.odoms) == len(self.ground_truths)

