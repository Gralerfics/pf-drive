import os
import time
import json

import numpy as np

import cv2

from multinodes import Node

from pf_drive.util import get_numbered_file_list


"""
    `output`, output (queue, block!)
        format: (image, odom)
"""
class RecordLoaderQueued(Node):
    def __init__(self, name, record_folder):
        super().__init__(name)
        
        self.odom_folder = os.path.join(record_folder, 'odom')
        self.processed_image_folder = os.path.join(record_folder, 'processed_image')

    def run(self):
        odom_filelist = get_numbered_file_list(self.odom_folder)
        processed_image_filelist = get_numbered_file_list(self.processed_image_folder)

        assert len(odom_filelist) == len(processed_image_filelist)
        N = len(odom_filelist)

        while True:
            if 'output' not in self.io:
                time.sleep(0.1)
                continue

            for idx in range(N):
                image = cv2.imread(os.path.join(self.processed_image_folder, processed_image_filelist[idx]))
                with open(os.path.join(self.odom_folder, odom_filelist[idx]), 'r') as f:
                    odom = np.array(json.load(f))
                self.io['output'].write((image, odom))

