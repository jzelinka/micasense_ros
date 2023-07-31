import cv2
import capture
import numpy as np

print("Loading imageutils.py")

def align_capture(capture: capture.Capture, ref_idx=0):
    warp_mats = []

    init_warp = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    n_iters = 1000
    e_thresh = 1e-6
    warp_mode = cv2.MOTION_EUCLIDEAN
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, n_iters, e_thresh)

    for idx, img in enumerate(capture.images):
        if idx == ref_idx:
            warp_mats.append(None)
            continue
        
        cc, warp = cv2.findTransformECC(capture.images[ref_idx], img, init_warp, warp_mode, criteria)
        warp_mats.append(warp)

def get_cropped_region(capture: capture.Capture):
    pass

def aligned_capture(capture: capture.Capture, crop):
    pass