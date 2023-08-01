import cv2
import capture
import numpy as np
import image

def align_capture(capture, ref_idx=0):
    warp_mats = []

    init_warp = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    # init_warp = np.eye(3, dtype=np.float32)
    n_iters = 100
    e_thresh = 1e-6
    warp_mode = cv2.MOTION_TRANSLATION
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, n_iters, e_thresh)

    for idx, img in enumerate(capture.images):
        print('Registering image:', idx)
        if idx == ref_idx:
            warp_mats.append(None)
        else:
            _, warp = cv2.findTransformECC(capture.images[ref_idx].image, img.image, init_warp, warp_mode, criteria)
            warp_mats.append(warp.copy())

    return warp_mats, ref_idx

def determine_roi(aligned_img):
    h, w  = aligned_img.shape
    x, y = 0, 0
    shape_y, shape_x = aligned_img.shape

    # crop right edge
    right_x = 0
    for pos_y in range(0, shape_y, 400):
        pos_x = shape_x - 1
        while (pos_x > 0) and (aligned_img[pos_y, pos_x] == 0):
            pos_x -= 1

        right_x = max(right_x, pos_x)
    w = right_x

    # crop left edge
    left_x = shape_x - 1
    for pos_y in range(0, shape_y, 400):
        pos_x = 0
        while (pos_x < shape_x - 1) and (aligned_img[pos_y, pos_x] == 0):
            pos_x += 1

        left_x = min(left_x, pos_x)
    
    x = left_x
    w = w - x

    # crop bottom edge
    bottom_y = 0
    for pos_x in range(0, shape_x, 400):
        pos_y = shape_y - 1
        while (pos_y > 0) and (aligned_img[pos_y, pos_x] == 0):
            pos_y -= 1

        bottom_y = max(bottom_y, pos_y)
    h = bottom_y

    # crop top edge
    top_y = shape_y - 1
    for pos_x in range(0, shape_x, 400):
        pos_y = 0
        while (pos_y < shape_y - 1) and (aligned_img[pos_y, pos_x] == 0):
            pos_y += 1

        top_y = min(top_y, pos_y)
    
    y = top_y
    h = h - y

    return x, y, w, h 

def get_cropped_region(capture):
    rois = [(x, y, w, h) for x, y, w, h in map(determine_roi, capture.aligned_images)]
    x = max([roi[0] for roi in rois])
    y = max([roi[1] for roi in rois])
    w = min([roi[2] for roi in rois])
    h = min([roi[3] for roi in rois])
    return x, y, w, h

def aligned_capture(capture, warp_mats):
    h, w = capture.images[capture.ref_idx].image.shape
    final = []
    for warp, img in zip(warp_mats, capture.images):
        if warp is None:
            final.append(img.image)
        else:
            final.append(cv2.warpAffine(img.image, warp, (w, h), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP))
    return final