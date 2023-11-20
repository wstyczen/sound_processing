# Sound processing

> A package meant to provide functionality to analyze and enhance the quality of audio files.
>
> Intended to be used as part of [a robot's voice communication system](https://github.com/wstyczen/dialogflow) to process recorded voice commands before they are passed to speech-to-text.

## Python dependencies

To run the scripts additional packages may need to be installed.

```sh
pip3 install audoai-noise-removal noisereduce numpy matplotlib scipy pydub
```

## Usage

### Audio enhancement

> Common steps used to enhance the overall quality of the audio.

- Normalization (_peak_).
- Filters (_low pass_ & _high pass_).
- Cutting out the silent parts of the audio.
- Eliminating background noise:
  - spectral subtraction / speactral gating (default, does not require api requests via internet, faster)
  - [online API](https://docs.audo.ai/) which uses an AI model to remove background noise (gives better results then algorithms mentioned above)

### Audio plots

> Generate plots of given audio to help visualize its state.

- waveform
- spectrogram
- volume over time

### Metrics

> Calculate some commonly used metrics for the audio (ie _Sound Noise Ratio_, _Spectral Flatness_), though they do not seem to be very informative when it comes to assessing audio's quality.
