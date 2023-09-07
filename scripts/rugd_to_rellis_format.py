import pathlib
import shutil
import cv2
import numpy as np

'''
Script to convert the RUGD dataset structure to RELLIS format.
The modified structure of the dataset helps to use already written dataloaders.
'''

rugd_color_to_class = {(0, 0, 0): 0,
    (108, 64, 20): 1,
    (255, 229, 204): 2,
    (0, 102, 0): 3,
    (0, 255, 0): 4,
    (0, 153, 153): 5,
    (0, 128, 255): 6,
    (0, 0, 255): 7,
    (255, 255, 0): 8,
    (255, 0, 127): 9,
    (64, 64, 64): 10,
    (251, 28, 0): 11,
    (255, 0, 0): 12,
    (153, 76, 0): 13,
    (102, 102, 0): 14,
    (102, 0, 0): 15,
    (0, 255, 128): 16,
    (204, 153, 255): 17,
    (102, 0, 204): 18,
    (255, 153, 204): 19,
    (0, 102, 102): 20,
    (153, 204, 255): 21,
    (102, 255, 255): 22,
    (101, 101, 11): 23,
    (114, 85, 47): 24
}

def map_to_class(r, g, b):
    if (r, g, b) not in rugd_color_to_class.keys():
        pixel = 0
    else:
        pixel = rugd_color_to_class[(r, g, b)]
    return pixel

def rugd_label_to_rellis_label(rugd_label, output_dir):
    img = cv2.imread(str(rugd_label))
    class_label = np.vectorize(map_to_class)(img[:, :, 0], img[:, :, 1], img[:, :, 2])
    cv2.imwrite(str(output_dir / rugd_label.name), class_label)

def rugd_to_rellis_format(rugd_dir_name, annotation_dir_name, output_dest):
    home_dir = pathlib.Path.home()

    rugd_dir = pathlib.Path(str(home_dir) + rugd_dir_name)
    annotation_dir = pathlib.Path(str(home_dir) + annotation_dir_name)
    output_dir = pathlib.Path(str(home_dir) + output_dest)
    output_dir.mkdir(parents=True, exist_ok=True)

    for i, rugd_sequence in enumerate(rugd_dir.iterdir()):
        print(f"Processing {rugd_sequence.name} ({i+1}/{len(list(rugd_dir.iterdir()))})")
        output_image_dir = output_dir / rugd_sequence.name / "pylon_camera_node"
        output_image_dir.mkdir(parents=True, exist_ok=True)
        frames = rugd_sequence.glob("*.png")
        for frame in frames:
            shutil.copy(frame, output_image_dir / frame.name)

        annotation_sequence = annotation_dir / rugd_sequence.name
        output_annotation_dir = output_dir / rugd_sequence.name / "rugd_annotations"
        output_annotation_dir.mkdir(parents=True, exist_ok=True)

        class_label_dir = output_dir / rugd_sequence.name / "pylon_camera_node_label_id"
        class_label_dir.mkdir(parents=True, exist_ok=True)

        for annotation in annotation_sequence.glob("*.png"):
            shutil.copy(annotation, output_annotation_dir / annotation.name)
            rugd_label_to_rellis_label(annotation, class_label_dir)

if __name__ == "__main__":
    rugd_dir_name = "/datasets/RUGD/RUGD_frames-with-annotations"
    annotation_dir_name = "/datasets/RUGD/RUGD_annotations"
    output_dir_name = "/datasets/RUGD/rugd_rellis_format"
    rugd_to_rellis_format(rugd_dir_name, annotation_dir_name, output_dir_name)