import capture
import matplotlib.pyplot as plt
import numpy as np
import os
import glob


def visualize_aligned(cap: capture.Capture, show_plot=True, save_plot=False, save_path="output"):
    # TODO just show the images side by side
    fig, axes = plt.subplots(1, cap.channels)
    for idx, img in enumerate(cap.aligned_image):
        axes[idx].imshow(img)
    if save_plot:
        if not os.path.isdir(save_path):
            os.makedirs(save_path)
        plt.savefig(os.path.join(save_path, "aligned_channels.pdf"), dpi=300)
    if show_plot:
        plt.show()
    
def visualize_overlay_rgb(cap: capture.Capture, show_plot=True, save_plot=False, save_path="output"):
    # TODO show the images overlayed
    # check if the capture is rgb
    if not cap.is_rgb():
        raise ValueError("Capture must be rgb to visualize this overlay")

    blue_img = cap.aligned_image[cap.get_band_names().index("blue")]
    red_img = cap.aligned_image[cap.get_band_names().index("red")]
    green_img = cap.aligned_image[cap.get_band_names().index("green")]

    test_rgb = np.zeros((blue_img.shape[0], blue_img.shape[1], 3), dtype=np.uint8)

    fig, axes = plt.subplots(1, 2)
    axes[0].set_title("Aligned")
    axes[1].set_title("Original")

    test_rgb[:, :, 0] = red_img[:, :]
    test_rgb[:, :, 1] = green_img[:, :]
    test_rgb[:, :, 2] = blue_img[:, :]

    axes[0].imshow(test_rgb)

    blue_img = cap.images[cap.get_band_names().index("blue")].image
    red_img = cap.images[cap.get_band_names().index("red")].image
    green_img = cap.images[cap.get_band_names().index("green")].image

    test_rgb[:, :, 0] = red_img[:, :]
    test_rgb[:, :, 1] = green_img[:, :]
    test_rgb[:, :, 2] = blue_img[:, :]

    axes[1].imshow(test_rgb)

    if save_plot:
        if not os.path.isdir(save_path):
            os.makedirs(save_path)
        plt.savefig(os.path.join(save_path, "rgb_overlay_comparison.pdf"), dpi=300)
    if show_plot:
        plt.show()
    

if __name__ == "__main__":
    show_plot = False
    save_plot = True

    glob_list = sorted(glob.glob("data/000/IMG_0006_*.tif"))[:3]
    capture = capture.Capture.from_file_list(glob_list)

    visualize_aligned(capture, show_plot, save_plot)
    visualize_overlay_rgb(capture, show_plot, save_plot)
