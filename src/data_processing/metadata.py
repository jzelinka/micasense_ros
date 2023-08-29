import exiftool
import pathlib


class Metadata:
    BAND_NAMES = {
        1: "Blue",
        2: "Green",
        3: "Red",
        4: "NIR",
        5: "Red edge",
        6: "Panchro",
        7: "LWIR",
        8: "FLIR"
    }

    def __init__(self, fname) -> None:
        self.fname = fname
        with exiftool.ExifTool() as exift:
            self.metadata = exift.get_metadata(fname)

    def fname_to_band_idx(self):
        idx = pathlib.Path(self.fname).stem.split("_")[-1]
        return int(idx)
    
    def fname_to_band_name(self):
        return self.BAND_NAMES[self.fname_to_band_idx()]
        
    
    def band_index(self):
        if "XMP:RigCameraIndex" not in self.metadata:
            return self.fname_to_band_idx()
        else:
            return self.metadata["XMP:RigCameraIndex"]

    def band_name(self):
        if "XMP:BandName" not in self.metadata:
            return self.fname_to_band_name()
        return self.metadata["XMP:BandName"]