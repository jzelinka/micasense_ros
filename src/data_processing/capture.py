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

        print("Aligning images")
        warp_mats = imageutils.align_capture(self)
        crop_region = imageutils.get_cropped_region(self, warp_mats)

        self.band_names = self.get_band_names()
        self.aligned_image = imageutils.aligned_capture(self, crop_region, warp_mats)
        self.channels = len(self.images)

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
    
    def get_band_names(self):
        return [img.band_name.lower() for img in self.images]

    def band_index(self, band_name):
        return self.get_band_names().index(band_name.lower())
    
    def is_rgb(self):
        band_names = self.get_band_names()
        return "red".lower() in band_names and "green".lower() in band_names and "blue".lower() in band_names

if __name__ == "__main__":
    glob_list = sorted(glob.glob("data/000/IMG_0006_*.tif"))[:5]
    # glob_list = sorted(glob.glob("data/000/IMG_0006_*.tif"))[:]
    # test the alignment

    capture = Capture.from_file_list(glob_list)

    for img in capture.images:
        print(img)