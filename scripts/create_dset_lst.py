import pathlib
from sklearn.model_selection import train_test_split

'''
Script to generate .lst files for the structure of RELLIS dataset.
The .lst files are needed to specify which files are needed in which part of the training.
'''

FRAME_POS = "pylon_camera_node"
ANNOTATION_POS = "pylon_camera_node_label_id"

def make_lst_file(file_list, out_file):
    with open(out_file, "w") as f:
        for file_to_lst in file_list:
            label_file = file_to_lst.name
            sequence_name = pathlib.Path(file_to_lst).parent.parent.name
            tmp = indir / sequence_name / ANNOTATION_POS / label_file
            label_file = pathlib.Path(tmp).relative_to(indir)
            frame_file = pathlib.Path(file_to_lst).relative_to(indir)
            
            f.write(f"{frame_file} {label_file}\n")

indir = pathlib.Path("/home/jz/datasets/RUGD/rugd_rellis_format")
outdir = pathlib.Path("/home/jz/datasets/RUGD/rugd_rellis_format")

outdir.mkdir(parents=True, exist_ok=True)

dset_dir = indir / FRAME_POS

files = []
for sequence in indir.iterdir():
    files.extend(list((sequence / FRAME_POS).glob("**/*.png")))

train, tmp = train_test_split(files, test_size=0.2)
val, test = train_test_split(tmp, test_size=0.5)

print("Created the splis")

make_lst_file(train, outdir / "train.lst")
make_lst_file(val, outdir / "val.lst")
make_lst_file(test, outdir / "test.lst")

print("Finished.")