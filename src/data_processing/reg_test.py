import cv2
import numpy as np
import timeit
import os
import matplotlib.pyplot as plt


"""Inits"""

location = "/home/jz/multi_spec/initial_collect_at_kn/0006SET/000"

img1 = cv2.imread(os.path.join(location, "IMG_0006_1.tif"))
img2 = cv2.imread(os.path.join(location, "IMG_0006_2.tif"))
img3 = cv2.imread(os.path.join(location, "IMG_0006_3.tif"))

h, w = img1.shape[:2]

# ECC params
init_warp = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
n_iters = 1000
e_thresh = 1e-6
warp_mode = cv2.MOTION_EUCLIDEAN
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, n_iters, e_thresh)


"""Full scale ECC algorithm"""
full_scale_start_time = timeit.default_timer()

gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
gray3 = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)
cc, warp = cv2.findTransformECC(gray1, gray2, init_warp, warp_mode, criteria)

# write blended warp and diff
img2_aligned = cv2.warpAffine(img2, warp, (w, h), flags=cv2.WARP_INVERSE_MAP)

cc, warp = cv2.findTransformECC(gray1, gray3, init_warp, warp_mode, criteria)
img3_aligned = cv2.warpAffine(img3, warp, (w, h), flags=cv2.WARP_INVERSE_MAP)

"""Pyramid ECC algorithm"""

pyr_start_time = timeit.default_timer()

print('Total time:', timeit.default_timer() - full_scale_start_time)

test_rgb = np.zeros((img1.shape[0], img1.shape[1], 3), dtype=np.uint8)
fig, axes = plt.subplots(1, 2)
axes[0].set_title("Red-Green-Blue Composite")
axes[1].set_title("Color Infrared (CIR) Composite")

test_rgb[:, :, 0] = img3_aligned[:, :,0]
test_rgb[:, :, 1] = img2_aligned[:, :,0]
test_rgb[:, :, 2] = img1[:, :,0]

axes[0].imshow(test_rgb)
test_rgb[:, :, 0] = img3[:, :,0]
test_rgb[:, :, 1] = img2[:, :,0]
test_rgb[:, :, 2] = img1[:, :,0]

axes[1].imshow(test_rgb)
plt.savefig("before_and_after.pdf", dpi=300)