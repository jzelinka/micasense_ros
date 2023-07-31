import exiftool

class Metadata:
    def __init__(self, fname) -> None:
        with exiftool.ExifTool() as exift:
            self.metadata = exift.get_metadata(fname)
    
    def band_index(self):
        return self.metadata["XMP:RigCameraIndex"]

    def band_name(self):
        return self.metadata["XMP:BandName"]