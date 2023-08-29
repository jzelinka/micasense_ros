import pathlib
import exiftool

image_path = pathlib.Path("/home/jz/multi_spec/initial_collect_at_kn/0006SET/000")

images = list()
for image in sorted(image_path.iterdir()):
    with exiftool.ExifTool() as exift:
        metadata = exift.get_metadata(str(image))
    print(image.name, metadata["XMP:BandName"])