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

def compare_band_alignment(cap: capture.Capture, vis_band_names, show_plot=True, save_plot=False, save_path="output"):
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

def visualize_overlay_band_alignment(cap: capture.Capture, vis_band_names, prefix="aligned_overlay_", save_path="output", add_band_names=True):
    if len(vis_band_names) != 3:
        raise ValueError("Must visualize exactly 3 bands")

    bands = [cap.aligned_images[cap.band_index(band_name)] for band_name in vis_band_names]

    assert bands[0].shape == bands[1].shape == bands[2].shape
    
    # Merge the three bands into an RGB image
    rgb_image = cv2.merge(bands)

    # Save the RGB image to a file
    output_filename = save_path + "/" + prefix + cap.cap_name
    if add_band_names:
        output_filename += "_" + "_".join(vis_band_names) 
    output_filename += ".jpg"
    print("Saving to", output_filename)
    cv2.imwrite(output_filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))


def save_all(cap: capture.Capture, prefix="aligned_", save_path="output"):
    for idx, img in enumerate(cap.aligned_images):
        out_fname = save_path + "/" + prefix + cap.images[idx].fname
        cv2.imwrite(out_fname, img)
        print("Saved " + prefix + cap.images[idx].fname)


def compare_overlay_rgb(cap: capture.Capture, show_plot=True, save_plot=False, save_path="output"):
    if not cap.is_rgb():
        raise ValueError("Capture must be rgb to visualize this overlay")

    compare_band_alignment(cap, ["red", "green", "blue"], show_plot, save_plot, save_path)

def visualize_overlay_rgb(cap: capture.Capture, show_plot=True, save_plot=False, prefix="", add_band_names=True, save_path="output"):
    if not cap.is_rgb():
        raise ValueError("Capture must be rgb to visualize this overlay")

    visualize_overlay_band_alignment(cap, ["red", "green", "blue"], save_path=save_path, add_band_names=add_band_names, prefix=prefix)


if __name__ == "__main__":
    show_plot = False
    save_plot = True

    glob_list = sorted(glob.glob("data/000/IMG_0004_*.tif"))[:5]
    capture = capture.Capture.from_file_list(glob_list)

    compare_overlay_rgb(capture, show_plot, save_plot)
    save_all(capture)