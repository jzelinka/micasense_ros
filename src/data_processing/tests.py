import capture
import glob

def test_crop(cap):
    # tests if the crop function works
    # suply path to image with all black edges

    print("Original:", cap.images[0].image.shape)
    print("Cropped:", cap.aligned_images[0].shape)

if __name__=="__main__":
    glob_list = sorted(glob.glob("data/rotated/IMG_0001_*.tif"))[:3]
    capture = capture.Capture.from_file_list(glob_list)

    test_crop(capture)