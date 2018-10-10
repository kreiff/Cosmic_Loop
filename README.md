# Cosmic_Looper
Arduino based 12-bit Sampler. Includes the following features:
- 4 Samples with variable length between 3-15 seconds depending on sample rate
- Bit Crushing on Sample & Passthrough
- Variable Sample Rate of Samples & Passthrough (Max Sample Rate 10.025Khz)
- Variable Playback Speed of Samples
- Dual Lowpass & Highpass filter
- Reverse Playback

Based on the Audio Hacker Shield and accompanying Header Files from Nootropic Design. Inspiration from DigDugDIY who I believe also used the Nootropic Design audio shield and code as the basis of his "Lofi Dreams" sampler.

Eventually hope to add additional features like simultaneous sample playback, granular delay & reverb.

Current build requires an interface with 6 Buttons (connected as Digital Inputs) and 4 knobs (connected as analog inputs).

Known Issues in current build:
- Highpass Filter does not effect Reversed Samples

Additional Features in the Queue:
- Simultaneous Sample Playback
- Granular Delay on Samples / Passthrough
- UX improvements
  - Latching & 1-shot sample playback
  - Reverse playback switching during forward playback
  - Input / Passthrough Mute
  - Three State effect options (Effect on Sample Only, Effect on Passthrough & Sample, Effect Off) - for filter, delay, etc.
- Reverb on Samples / Passthrough
- Traditional Digital BBD Style Delay on Samples / Passthrough
