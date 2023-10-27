from enum import Enum
from rospkg import RosPack


class SamplePaths(str, Enum):
    """
    Enum representing paths to sample audio files and related directories.

    Attributes:
        PACKAGE (str): Path of the package.
        DATA_DIR (str): Path to folder containing data files.
        SAMPLES_DIR (str): Path to the directory within the package containing sound samples.
        BACKGROUND_NOISE_SAMPLE (str): The path to the background noise sample file.
            This sample does not contain any 'desired' audio and can be used as a 'noise
            profile' for later noise reduction and removal.
        OUT_DIR (str): Directory for generated output.
        PLOTS_DIR (str): Path to the directory used to store generated plots.
        PROCESSED_AUDIO_DIR (str): Path to the directory used to store processed audio.
    """

    PACKAGE = RosPack().get_path("sound_processing")
    DATA_DIR = "%s/data" % PACKAGE
    SAMPLES_DIR = "%s/audio/samples" % DATA_DIR
    BACKGROUND_NOISE_SAMPLE = "%s/audio/background_noise.wav" % DATA_DIR
    OUT_DIR = "%s/out" % DATA_DIR
    PLOTS_DIR = "%s/plots/" % OUT_DIR
    PROCESSED_AUDIO_DIR = "%s/audio/" % OUT_DIR
