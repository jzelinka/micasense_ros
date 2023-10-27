import data_processing.imageutils as imageutils
import data_processing.image as image

import cv2
import numpy as np
import os
import glob

POSSIBLE_BAND_NAMES = ['blue', 'green', 'red', 'nir', 'red edge', 'panchro', 'lwir']

class Capture:
    def __init__(self, images) -> None:
        # test if all the images are of class image
        assert len(images) > 0

        for img in images:
            if not isinstance(img, image.Image):
                raise TypeError("All images must be of class image.Image")
        self.images = images

        self.cap_name = get_prefix(self.images[0].fname)

        warp_mats, self.ref_idx = imageutils.align_capture(self)
        self.warp_mats = warp_mats

        self.band_names = self.get_band_names()
        self.aligned_images = imageutils.aligned_capture(self, warp_mats)

        crop_region = imageutils.get_cropped_region(self)
        self.crop_aligned(crop_region)

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
    
    def use_outside_warp(self, warp_mats):
        if len(warp_mats) != len(self.images):
            raise ValueError("warp_mats must be of same length as images")
        self.aligned_images = imageutils.aligned_capture(self, warp_mats)
        crop_region = imageutils.get_cropped_region(self)
        self.crop_aligned(crop_region)
    
    def get_band_names(self):
        return [img.band_name.lower() for img in self.images]

    def band_index(self, band_name):
        return self.get_band_names().index(band_name.lower())
    
    def is_rgb(self):
        band_names = self.get_band_names()
        return "red".lower() in band_names and "green".lower() in band_names and "blue".lower() in band_names
    
    def crop_aligned(self, crop_region):
        x, y, w, h = crop_region
        target_shape = self.images[self.ref_idx].image.shape
        print("Cropping to region:", crop_region)
        for idx, img in enumerate(self.aligned_images):
            self.aligned_images[idx] = img[y:y+h, x:x + w]
            self.aligned_images[idx] = cv2.resize(self.aligned_images[idx], target_shape[::-1])

def get_prefix(fname):
    return "_".join(fname.split("_")[:-1])

def band_name_possible(band_name):
    return band_name.lower() in POSSIBLE_BAND_NAMES


if __name__ == "__main__":
    glob_list = sorted(glob.glob("data/000/IMG_0006_*.tif"))[:7]

    capture = Capture.from_file_list(glob_list)

    for img in capture.images:
        print(img)