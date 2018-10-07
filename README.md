# Cosmic_Looper
Arduino based 12-bit Sampler. Includes the following features:
- Bit Crushing on Sample & Passthrough
- Variable Sample Rate of Samples & Passthrough
- Variable Playback Speed of Samples
- Dual Lowpass & Highpass filter
- Reverse Playback

Based on the Audio Hacker Shield and accompanying Header Files from Nootropic Design. Inspiration from DigDugDIY who I believe also used the Nootropic Design audio shield and code as the basis of his "Lofi Dreams" sampler.

Eventually hope to add additional features like Granular Delay & Reverb.

Current build requires an interface with 6 Buttons (connected as Digital Inputs) and 4 knobs (connected as analog inputs).

Known Issues in current build:
- Highpass Filter does not effect Reversed Samples
- Reverse Button needs debouncing / smoothing
- Reverse Playback triggering gets mushy / slow after 1-2 O'clock on the Sample Rate pot.

Additional Features in the Queue:
- Simultaneous Sample Playback
- Granular Delay
- UX improvements
  - Latching & 1-shot sample playback
  - Reverse playback switching during forward playback
  - Input / Passthrough Mute
  - Three State effect options (Effect on Sample Only, Effect on Passthrough & Sample, Effect Off) - for filter, delay, etc.
- Reverb on Samples / Passthrough
- Delay on Samples / Passthrough
