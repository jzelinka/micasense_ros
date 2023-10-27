# import exiftool
import cv2
import data_processing.metadata as metadata
import pathlib
import rospy

class Image:
    def __init__(self, fname="", data=None, index=-1, band_name="") -> None:
        if fname == "":
            self.fname = "created_by_passing_file"
            self.image = data
            self.metadata = None

            assert index >= 0
            self.band_index = index
            assert band_name != ""
            self.band_name = band_name
        else:
            self.fname = pathlib.Path(fname).name
            self.image = cv2.cvtColor(cv2.imread(fname), cv2.COLOR_BGR2GRAY)
            self.metadata = metadata.Metadata(fname)

            self.band_index = self.metadata.band_index()
            self.band_name = self.metadata.band_name()

        self.shape = self.image.shape


    
    def __str__(self) -> str:
        return "Loaded image: " + self.fname + "\n with band index: " + str(self.band_index) + ", band name: " + self.band_name + " shape: " + str(self.shape)