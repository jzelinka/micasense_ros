import capture
import glob
import pathlib
import visualize
import argparse

if __name__=="__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument("-n", help="Number of bands to use", type=int, default=5)
    argparse.add_argument("input_path", help="Path to input images")
    argparse.add_argument("output_path", help="Path to output images")
    argparse.add_argument("-vis", help="Folder to visualize the alignment to.", type=str, default=None)

    args = argparse.parse_args()

    # TODO allow for band selection
    input_path = args.input_path

    print("Starting")

    capture_names = sorted(glob.glob(input_path + "/*1.tif"))
    output_dir = pathlib.Path(args.output_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    for capture_name in capture_names:
        prefix = capture.get_prefix(capture_name)

        files = sorted(glob.glob(prefix + "*.tif"))[:args.n]

        assert len(files) == args.n

        capture_now = capture.Capture.from_file_list(files)
        visualize.save_all(capture_now, save_path=str(output_dir))
        visualize.visualize_overlay_rgb(capture_now, save_path=str(output_dir))
        visualize.visualize_overlay_band_alignment(capture_now, ["nir", "red edge", "green"])
