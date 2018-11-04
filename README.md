# Cosmic_Loop
Arduino based 12-bit Sampler. Includes the following features:
- 4 Samples with variable length between 3-15 seconds depending on sample rate
- Recording threshold of about -50db for easy sampling of percussive sounds
- Monophonic playback w/ Passthrough
- Bit Crushing on Sample & Passthrough
- Variable Sample Rate of Samples & Passthrough (Max Sample Rate 16Khz)
- Variable Playback Speed of Samples
- Dual Lowpass & Highpass filter
- Reverse Playback
- Grain Delay
- Freeze Mode

Based on the Audio Hacker Shield and accompanying Header Files from Nootropic Design. Inspiration from DigDugDIY who I believe also used the Nootropic Design audio shield and code as the basis of his "Lofi Dreams" sampler.

Current build requires an interface with 7 Buttons (connected as Digital Inputs) and 6 knobs (connected as analog inputs).

![alt text](https://github.com/kreiff/Cosmic_Loop/blob/master/Hardware/CL_Prototype.jpg)

Additional Features in the Queue:
- Playback envelope for samples
- UX improvements
  - Latching, 1-shot, and momentary button response for sample playback.
  - Three State effect options (Effect on Sample Only, Effect on Passthrough & Sample, Effect Off) - for filter, grain delay, etc.
- Simultaneous Sample Playback / Sample Polyphony

Additional notes:
- If you're using the Audio Hacker shield from Nootropic Design and an Arudino Uno be aware that only 7 Digital pins and 6 Analog pins are available as inputs / outputs. Digital pin 7-13 are used by the shield to communicate with the ADC, DAC, and SRAM chips. If you're adding additional UX improvements be sure to make sure you're only working with digital pins 0-6. You can alternatively use a shift register to add additional digital inputs (See NooTropic's EX-Expander Shield as an example).
- This project uses the TX and RX pins for specific functions. If you're using the Serial Print function to debug your code - you may get odd results / print-outs if the button you're debugging is connected to pins D0 or D1. 
- As noted above only 7 digital pins are available as inputs / outputs, but you'll notice that there are 8 buttons identified in the wiring diagram - The input / passthrough mute function is hardwired and does not depend on the Arduino or any code to function. It simply grounds the input signal at the bypass switch preventing it from going to the output.
