import exiftool
import cv2
import metadata
import pathlib

class Image:
    def __init__(self, fname) -> None:
        self.fname = pathlib.Path(fname).name
        self.image = cv2.cvtColor(cv2.imread(fname), cv2.COLOR_BGR2GRAY)
        self.shape = self.image.shape
        self.metadata = metadata.Metadata(fname)

        self.band_index = self.metadata.band_index()
        self.band_name = self.metadata.band_name()


    
    def __str__(self) -> str:
        return "Loaded image: " + self.fname + "\n with band index: " + str(self.band_index) + ", band name: " + self.band_name + " shape: " + str(self.shape)