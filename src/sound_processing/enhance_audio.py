#!/usr/bin/env python
import os
import rospy

from pydub import AudioSegment
from pydub.silence import split_on_silence

from sound_processing.sample_paths import SamplePaths
from sound_processing.audio_metrics import AudioMetrics
from sound_processing.audio_plots import AudioPlotGenerator

from enum import Enum


class AudioEnhancement:
    """
    Enhance the quality of the audio of a given .wav file.

    Attributes:
        _input_path (str): The path to the input audio file.
        _input (AudioSegment): The audio to be processed.
        _background_noise (AudioSegment): Audio of a sample of the background noise.
        _output_path (str): The path under which the processed audio will be saved.
    """

    class AudioState(str, Enum):
        INITIAL = "initial"
        ENHANCED = "enhanced"

    def __init__(self, input_path, overwrite_input_file=True):
        """Initialize AudioEnhancement instance.

        Args:
            input_path (str): The path to the input audio file.
            overwrite_input_file (bool): If True, overwrites the input file with
                processed audio. If False, creates a new file.
        """
        self.log("Initializing with '%s'." % input_path)
        # Load the audio from files.
        self._input_path = input_path
        self._input = AudioSegment.from_wav(input_path)
        self._background_noise = AudioSegment.from_wav(
            SamplePaths.BACKGROUND_NOISE_SAMPLE
        )
        self._should_overwrite_input = overwrite_input_file
        self._output_dir = self.get_output_dir()

    @staticmethod
    def log(msg):
        """
        Add a module prefix and print the message to the console.

        Args:
            msg (str): The message to log.
        """
        rospy.loginfo("[AudioEnhancement] %s" % msg)

    def get_output_dir(self):
        """
        Get the path to the directory to save related files.

        Returns:
            str: The path for saving output.
        """
        # Ensure the output directory exists.
        if not os.path.exists(SamplePaths.OUT_DIR):
            os.makedirs(SamplePaths.OUT_DIR)
        # Save output within output dir in a subdirectory named after the input
        # file.
        file_name, _ = os.path.splitext(os.path.basename(self._input_path))
        save_dir = os.path.join(SamplePaths.OUT_DIR, file_name)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        return save_dir

    def overwrite_input(self):
        """
        Overwrites the input file with the audio in its current state.
        """
        self.log("Overwriting input audio file: '%s'." % self._input_path)
        self._input.export(self._input_path, format="wav")

    def normalize_volume(self, target_dbfs=0):
        """
        Normalize the volume of the audio.

        This function ensures that the audio's volume reaches the desired amplitude
        available without clipping, making it more consistent in terms of loudness
        and clearer overall.
        """
        self.log("Normalizing volume.")
        self._input = self._input.apply_gain(target_dbfs - self._input.max_dBFS)

    def apply_filter(self, lowcut=100, highcut=4000):
        """
        Applies a filter to the audio.

        Args:
            lowcut (int): The low cut-off frequency (Hz).
            highcut (int): The high cut-off frequency (Hz).

        This function applies a filter to the audio according to the given
        cut-off frequency values. A low-pass filter removes high-frequency noise
        (e.g., hissing), while a high-pass filter removes low-frequency noise
        (e.g., humming).

        Human speech is usually within the range of 100 Hz to 4 kHz, hence the
        default values.
        """
        self.log("Filtering.")
        self._input = self._input.low_pass_filter(highcut)
        self._input = self._input.high_pass_filter(lowcut)

    def remove_silence(self, silence_threshold=-50):
        """
        Remove large chunks of silence from the audio.

        Args:
            silence_threshold (int): The silence threshold in dBFS.

        This function removes silence from the audio by splitting it into
        non-silent parts (with a padding to avoid cut-offs) and then merging
        them back together.
        """
        self.log("Removing silent audio segments.")
        # Split audio segment on silent parts.
        non_silent_chunks = split_on_silence(
            audio_segment=self._input,
            min_silence_len=200,
            keep_silence=100,
            silence_thresh=silence_threshold,
        )
        # Merge non-silent back parts together.
        self._input = non_silent_chunks[0]
        for part in non_silent_chunks[1:]:
            self._input += part

    def spectral_subtraction(self, noise_profile=SamplePaths.BACKGROUND_NOISE_SAMPLE):
        self.log("Performing spectral subtraction.")
        # TODO: Implement working version.
        pass

    def save_audio_data(self, state):
        """
        Saves the audio data, plots and metrics within the data/out dir.
        """
        self.log("Saving audio state to '%s'." % self._output_dir)
        # Save audio data.
        audio_path = os.path.join(self._output_dir, "%s_audio.wav" % state)
        self._input.export(audio_path, format="wav")
        # Save audio visualizing plots as png.
        plot_generator = AudioPlotGenerator(audio_path)
        plot_generator.plot()
        plot_generator.save(self._output_dir, state)
        # Save metrics to a file.
        # TODO: Save as json.
        text_file = open(os.path.join(self._output_dir, "%s_metrics.json" % state), "w")
        text_file.write(str(AudioMetrics(audio_path)))
        text_file.close()

    def enhance(self):
        """
        Enhance the audio by applying a series of processing steps.
        """
        # Save initial audio data.
        self.save_audio_data(self.AudioState.INITIAL.value)

        # Process audio.
        self.normalize_volume()

        self.apply_filter()

        self.remove_silence()

        # TODO: Implement working version.
        self.spectral_subtraction()

        # Save processed audio data.
        self.save_audio_data(self.AudioState.ENHANCED.value)
        if self._should_overwrite_input:
            self.overwrite_input()
