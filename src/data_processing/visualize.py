import capture
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import cv2


def visualize_aligned(cap: capture.Capture, show_plot=True, save_plot=False, save_path="output"):
    fig, axes = plt.subplots(1, cap.channels)
    for idx, img in enumerate(cap.aligned_images):
        axes[idx].imshow(img)
    plt.tight_layout()
    if save_plot:
        if not os.path.isdir(save_path):
            os.makedirs(save_path)
        plt.savefig(os.path.join(save_path, "aligned_channels.pdf"), dpi=300)
    if show_plot:
        plt.show()

def visualize_band_alignment(cap: capture.Capture, vis_band_names, show_plot=True, save_plot=False, save_path="output"):
    if len(vis_band_names) != 3:
        raise ValueError("Must visualize exactly 3 bands")
    
    output_shape = cap.aligned_images[cap.get_band_names().index(vis_band_names[0])].shape
    res = np.zeros((output_shape[0], output_shape[1], 3), dtype=np.uint8)
    for idx, band_name in enumerate(vis_band_names):
        res[:, :, idx] = cap.aligned_images[cap.band_index(band_name)][:, :]
    
    fig, axes = plt.subplots(1, 2)
    title = " ".join(vis_band_names)
    plt.tight_layout()
    plt.suptitle(title)
    axes[0].set_title("Aligned")
    axes[1].set_title("Original")

    axes[0].imshow(res)

    output_shape = cap.images[cap.get_band_names().index(vis_band_names[0])].image.shape
    res = np.zeros((output_shape[0], output_shape[1], 3), dtype=np.uint8)
    for idx, band_name in enumerate(vis_band_names):
        res[:, :, idx] = cap.images[cap.band_index(band_name)].image[:, :]

    axes[1].imshow(res)

    if save_plot:
        if not os.path.isdir(save_path):
            os.makedirs(save_path)
        plt.savefig(os.path.join(save_path, title + "_overlay_comparison.pdf"), dpi=300)
    if show_plot:
        plt.show()

def save_all(cap: capture.Capture, save_path="output"):
    for idx, img in enumerate(cap.aligned_images):
        cv2.imwrite("output/aligned_" + cap.band_names[idx] + ".tif", img)
        print("Saved aligned_" + cap.band_names[idx] + ".tif")


def visualize_overlay_rgb(cap: capture.Capture, show_plot=True, save_plot=False, save_path="output"):
    if not cap.is_rgb():
        raise ValueError("Capture must be rgb to visualize this overlay")

    visualize_band_alignment(cap, ["red", "green", "blue"], show_plot, save_plot, save_path)


if __name__ == "__main__":
    show_plot = False
    save_plot = True

    glob_list = sorted(glob.glob("data/000/IMG_0004_*.tif"))[:6]
    capture = capture.Capture.from_file_list(glob_list)

    # visualize_aligned(capture, show_plot, save_plot)
    visualize_overlay_rgb(capture, show_plot, save_plot)
    # visualize_band_alignment(capture, ["nir", "red", "green"], show_plot, save_plot)
    save_all(capture)