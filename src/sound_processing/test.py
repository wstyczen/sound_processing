#!/usr/bin/env python
import rospy
import os

from audio_metrics import AudioMetrics
from enhance_audio import AudioEnhancement
from audio_plots import AudioPlotGenerator
from sample_paths import SamplePaths


def process_samples():
    print("Processing samples...")
    for file_name in os.listdir(SamplePaths.SAMPLES_DIR):
        if not file_name.endswith(".wav"):
            continue

        print("Sample '%s':" % file_name)
        file_path = os.path.join(SamplePaths.SAMPLES_DIR, file_name)

        # Generate files with enhanced audio quality.
        AudioEnhancement(file_path, overwrite_input_file=False).enhance()

if __name__ == "__main__":
    rospy.init_node("audio_test")

    process_samples()

    # rospy.spin()
