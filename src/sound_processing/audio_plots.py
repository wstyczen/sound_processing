#!/usr/bin/env python3.6
import os

import matplotlib.pyplot as plt
from scipy.io import wavfile
import numpy as np


class AudioPlotGenerator:
    """
    Generate audio-related plots for the specified .wav audio file.

    Attributes:
        _file_path (str): Path to the audio file.
        _sample_rate (int): Sample rate of the audio file.
        _audio_data (numpy.ndarray): Raw audio data.
        _mono_audio_data (numpy.ndarray): Averaged mono audio data.
    """

    SUBPLOT_TITLE_SIZE = 18
    SUBPLOT_AXIS_LABEL_SIZE = 15
    SUBPLOT_AXIS_TICKS_SIZE = 12

    def __init__(self, file_path):
        """
        Initialize AudioPlotGenerator.

        Args:
            file_path (str): Path to the audio file.
        """
        self._file_path = file_path
        self._sample_rate, self._audio_data = wavfile.read(file_path)
        # Audio in mono format is necessary for spectrogram (converted by
        # averaging the stereo audio which is typically from two channels).
        if len(self._audio_data.shape) > 1:
            self._mono_audio_data = np.mean(self._audio_data, axis=1)
        else:
            self._mono_audio_data = self._audio_data

    def _plot_waveform(self):
        """
        Generate a plot of audio waveform.
        """
        plt.title("Audio Waveform", fontsize=self.SUBPLOT_TITLE_SIZE)
        plt.plot(self._audio_data, color="black")
        plt.xlim(0, len(self._audio_data))
        plt.xticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.yticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.xlabel("Sample", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)
        plt.ylabel("Amplitude", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)

    def _plot_spectrogram(self):
        """
        Generate a spectrogram plot.
        """
        plt.title("Spectrogram", fontsize=self.SUBPLOT_TITLE_SIZE)
        plt.specgram(self._mono_audio_data, Fs=self._sample_rate)
        plt.xticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.yticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.xlabel("Time (s)", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)
        plt.ylabel("Frequency (Hz)", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)

    def _plot_volume(self):
        """
        Generate a plot of volume (amplitude) over time.
        """
        amplitude = np.abs(self._audio_data)
        time = np.arange(len(self._audio_data)) / self._sample_rate

        plt.title("Audio Volume Over Time", fontsize=self.SUBPLOT_TITLE_SIZE)
        plt.plot(time, amplitude, color="black")
        plt.xlim(time[0], time[-1])
        plt.xticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.yticks(fontsize=self.SUBPLOT_AXIS_TICKS_SIZE)
        plt.xlabel("Time (s)", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)
        plt.ylabel("Amplitude", fontsize=self.SUBPLOT_AXIS_LABEL_SIZE)

    def plot(self):
        """
        Generate avaiable plots for given audio file.
        """
        plt.figure(figsize=(10, 10))

        # Generate plots in vertical layout.
        plt.subplot(3, 1, 1)
        self._plot_waveform()

        plt.subplot(3, 1, 2)
        self._plot_spectrogram()

        plt.subplot(3, 1, 3)
        self._plot_volume()

        # Adjust spacing between subplots.
        plt.subplots_adjust(hspace=3)
        plt.tight_layout()

    def save(self, directory, file_name):
        """
        Save generated plots to a file.

        Args:
            directory (str): Path to a directory in which the file will be saved.
            file_name (str): Name of the file.
        """
        plt.savefig(os.path.join(directory, file_name + ".png"))
