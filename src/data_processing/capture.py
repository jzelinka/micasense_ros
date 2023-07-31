# class for handling one capture of the camera
import imageutils
import image

import cv2
import numpy as np
import os
import glob


class Capture:
    def __init__(self, images) -> None:
        # test if all the images are of class image
        for img in images:
            if not isinstance(img, image.Image):
                raise TypeError("All images must be of class image.Image")
        self.images = images

        warp_mats = imageutils.align_capture(self)
        crop_region = imageutils.get_cropped_region(self)

        self.aligned_image = imageutils.aligned_capture(self, crop_region, warp_mats)
    
    @classmethod
    def from_file_list(cls, file_list):
        images = []
        if len(file_list) == 0:
            raise ValueError("file_list must not be empty")
        
        for fname in file_list:
            if not os.path.isfile(fname):
                raise FileNotFoundError("File not found: " + fname)
            else:
                images.append(image.Image(fname))
        return cls(images)

if __name__ == "__main__":
    glob_list = sorted(glob.glob("data/000/IMG_0006_*.tif"))
    # test the alignment

    capture = Capture.from_file_list(glob_list)

    for img in capture.images:
        print(img)