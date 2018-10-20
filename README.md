# Cosmic_Looper
Arduino based 12-bit Sampler. Includes the following features:
- 4 Samples with variable length between 3-15 seconds depending on sample rate
- Bit Crushing on Sample & Passthrough
- Variable Sample Rate of Samples & Passthrough (Max Sample Rate 16Khz)
- Variable Playback Speed of Samples
- Dual Lowpass & Highpass filter
- Reverse Playback
- Grain Delay

Based on the Audio Hacker Shield and accompanying Header Files from Nootropic Design. Inspiration from DigDugDIY who I believe also used the Nootropic Design audio shield and code as the basis of his "Lofi Dreams" sampler.

Current build requires an interface with 7 Buttons (connected as Digital Inputs) and 6 knobs (connected as analog inputs).

Known issues w/ current build:
- Grain Delay only plays through 1 loop in Reverse mode and then repeats last grain.

Additional Features in the Queue:
- UX improvements
  - Latching, 1-shot, and momentary button response for sample playback
  - Input / Passthrough Mute
  - Three State effect options (Effect on Sample Only, Effect on Passthrough & Sample, Effect Off) - for filter, grain delay, etc.
- Input threshold for recording.  
- Simultaneous Sample Playback
