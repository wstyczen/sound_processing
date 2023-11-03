from enum import Enum

from rospkg import RosPack


class SamplePaths(str, Enum):
    """
    Enum representing paths to sample audio files and related directories.

    Attributes:
        PACKAGE (str): Path to the package.
        DATA_DIR (str): Path to folder containing data files.
        IN_DIR (str): Path to directory for input files.
        SAMPLES_DIR (str): Path to the directory within the package containing sound samples.
        BACKGROUND_NOISE_SAMPLE (str): The path to the background noise sample file.
            This sample does not contain any 'desired' audio and can be used as a 'noise
            profile' for later noise reduction and removal.
        OUT_DIR (str): Directory for storing generated output.
    """

    PACKAGE = RosPack().get_path("sound_processing")
    DATA_DIR = "%s/data" % PACKAGE
    IN_DIR = "%s/in" % DATA_DIR
    SAMPLES_DIR = "%s/samples" % IN_DIR
    BACKGROUND_NOISE_SAMPLE = "%s/background_noise.wav" % IN_DIR
    OUT_DIR = "%s/out" % DATA_DIR
