from pathlib import Path
import shutil

'''
Script to copy every nth file from a source directory to a destination directory.
'''

# Source and destination directories
source_directory = Path("/home/jz/datasets/multispec/filter_test")
destination_directory = Path("/home/jz/datasets/multispec/reg_10s_test")
copy_every = 10

# List all image files in the source directory
extension = ["*.jpg", "*.png", "*.tif"]
image_files = []
for ext in extension:
    image_files.extend(list(source_directory.glob(ext)))

# get the unique in_files
in_files = list(set(str(fname.parent) + '/' + fname.stem[:-2] for fname in image_files))
in_files.sort()

print("Cleaning the destination directory")
if destination_directory.exists():
    shutil.rmtree(destination_directory)
destination_directory.mkdir(parents=True, exist_ok=True)

for index, image_file in enumerate(in_files, start=1):
    if index % copy_every == 0:
        for f in source_directory.glob(Path(image_file).name + "*"):
            shutil.copy2(f, destination_directory)
            print(f"Copying {f.name} to {destination_directory}")
        
print("Copying complete.")
print(f"Count of previous files {len(list(source_directory.iterdir()))}, new folder has {len(list(destination_directory.iterdir()))}")