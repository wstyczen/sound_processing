#!/usr/bin/env python3.6
from pydub import AudioSegment
import numpy as np

from sound_processing.sample_paths import SamplePaths


class AudioMetrics:
    """An AudioMetrics class for analyzing audio signals.

    This class provides methods to analyze audio signals, including calculating the
    Signal-to-noise ratio (SNR), spectral flatness, and other metrics.

    Attributes:
        EPS (float): A small value to avoid division  & log operations on values approaching zero.
        _input (AudioSegment): The audio to be analyzed.
        _background_noise (AudioSegment): Background noise sample.
        _snr (float): The Signal-to-noise ratio (SNR) in decibels.
        _spectral_flatness (float): The Spectral Flatness of the audio signal.
    """

    EPS = 1e-12  # Constant applied to avoid division by ~0

    def __init__(self, input_path):
        """
        Initialize AudioMetrics instance.

        Args:
            input_path (str): The path to the input audio file.
        """
        # Load the audio from samples
        self._input = AudioSegment.from_wav(input_path)
        self._background_noise = AudioSegment.from_wav(
            SamplePaths.BACKGROUND_NOISE_SAMPLE
        )

        # Signal-to-noise ratio (dB)
        self._snr = self._get_SNR()

        # Spectral flatness (dB)
        self.spectral_flatness = self._get_spectral_flatness()

        input_arr = AudioMetrics._as_array(self._input)
        self._loudest = AudioMetrics._as_DB(np.max(input_arr))
        self._quietest = AudioMetrics._as_DB(np.min(input_arr))

    def __repr__(self):
        """
        Return a string representation of this object.

        Returns:
            str: A formatted string representation.
        """
        return (
            '{\n\t"SNR": %.3f,\n\t"Spectral Flatness": %.3f,\n\t"Loudest (dB)": %.3f,\n\t"Quietest (dB)": %.3f\n}'
            % (
                self._snr,
                self.spectral_flatness,
                self._loudest,
                self._quietest,
            )
        )

    @staticmethod
    def _as_DB(amplitude):
        """
        Converts an amplitude value to decibels.

        Args:
            amplitude (float): The amplitude value.

        Returns:
            float: The amplitude value in decibels.
        """
        return 10 * np.log10(max(amplitude, AudioMetrics.EPS))

    @staticmethod
    def _as_array(audio_segment):
        """
        Converts a pydub.AudioSegment to a NumPy array representation.

        Args:
            audio_segment (pydub.AudioSegment): The audio segment to convert.

        Returns:
            numpy.ndarray: The audio data as a NumPy array.
        """
        return np.array(audio_segment.get_array_of_samples())

    def _get_SNR(self):
        """
        Returns calculated signal-to-noise ratio.

        SNR is a measure of the ratio of signal amplitude to background noise
        amplitude.

        Returns:
            float: The calculated SNR in decibels.
        """
        # Convert samples to vectors.
        audio = AudioMetrics._as_array(self._input)
        noise_sample = AudioMetrics._as_array(self._background_noise)

        # Ratio of signal amplitude to background noise amplitude (avg).
        SNR = np.sum(audio**2) / np.sum(noise_sample**2)

        # Return in decibels.
        return AudioMetrics._as_DB(SNR)

    def _get_spectral_flatness(self):
        """
        Calculate and return the Spectral Flatness of the audio.

        Spectral flatness is a measure of the noisiness of a signal.

        Returns:
            float: The calculated spectral flatness.
        """
        # Apply Fast Fourier Transform to each frame to get power spectrum in
        # frequnecy domain.
        magnitude_spectrum = np.abs(np.fft.fft(AudioMetrics._as_array(self._input)))

        # Divide the geometric mean of the power spectrum by its arithmetic mean.
        geometric_mean = np.exp(np.mean(np.log(magnitude_spectrum + AudioMetrics.EPS)))
        arithmetic_mean = np.mean(magnitude_spectrum)
        return geometric_mean / (arithmetic_mean + AudioMetrics.EPS)
