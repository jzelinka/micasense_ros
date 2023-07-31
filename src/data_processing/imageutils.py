import cv2
import capture
import numpy as np

def align_capture(capture, ref_idx=0):
    warp_mats = []

    init_warp = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    n_iters = 100
    e_thresh = 1e-6
    warp_mode = cv2.MOTION_EUCLIDEAN
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, n_iters, e_thresh)

    for idx, img in enumerate(capture.images):
        print('Registering image:', idx)
        if idx == ref_idx:
            warp_mats.append(None)
        else:
            cc, warp = cv2.findTransformECC(capture.images[ref_idx].image, img.image, init_warp, warp_mode, criteria)
            warp_mats.append(warp.copy())

    return warp_mats

def get_cropped_region(capture, warp_mats):
    # TODO find the biggest crop region
    pass

def aligned_capture(capture, crop, warp_mats):
    # TODO add the crop
    h, w = capture.images[0].image.shape
    final = []
    for warp, img in zip(warp_mats, capture.images):
        print(warp)
        if warp is None:
            final.append(img.image)
        else:
            final.append(cv2.warpAffine(img.image, warp, (w, h), flags=cv2.WARP_INVERSE_MAP))
    return final