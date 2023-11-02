#!/usr/bin/env python
import os

from pydub import AudioSegment
from pydub.silence import split_on_silence

from sample_paths import SamplePaths


class AudioEnhancement:
    """
    Enhance the quality of the audio of a given .wav file.

    Attributes:
        _input_path (str): The path to the input audio file.
        _input (AudioSegment): The audio to be processed.
        _background_noise (AudioSegment): Audio of a sample of the background noise.
        _output_path (str): The path under which the processed audio will be saved.
    """

    def __init__(self, input_path, overwrite_input_file=False):
        """Initialize AudioEnhancement instance.

        Args:
            input_path (str): The path to the input audio file.
            overwrite_input_file (bool): If True, overwrites the input file with
                processed audio. If False, creates a new file.
        """
        # Load the audio from files.
        self._input_path = input_path
        self._input = AudioSegment.from_wav(input_path)
        self._background_noise = AudioSegment.from_wav(
            SamplePaths.BACKGROUND_NOISE_SAMPLE
        )

        if overwrite_input_file:
            self._output_path = self._input_path
        else:
            self._output_path = self.get_output_path()

    def get_output_path(self):
        """
        Get the path for saving the processed audio file.

        Returns:
            str: The path for saving output.
        """
        # Ensure the directory exists.
        if not os.path.exists(SamplePaths.PROCESSED_AUDIO_DIR):
            os.makedirs(SamplePaths.PROCESSED_AUDIO_DIR)
        # Save under the same name as input in output dir.
        file_name, _ = os.path.splitext(os.path.basename(self._input_path))
        return os.path.join(SamplePaths.PROCESSED_AUDIO_DIR, file_name + "_out.wav")

    def save_output(self):
        """
        Exports the processed audio to the specified output path in .wav format.
        """
        self._input.export(self._output_path, format="wav")

    def normalize_volume(self, target_dbfs=-10):
        """
        Normalize the volume of the audio.

        This function ensures that the audio's volume reaches the desired amplitude
        available without clipping, making it more consistent in terms of loudness
        and clearer overall.
        """
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
        # TODO: Implement working version.
        pass

    def enhance(self):
        """
        Enhance the audio by applying a series of processing steps.
        """
        self.normalize_volume()

        self.apply_filter()

        self.remove_silence()

        self.save_output()
