/*
  Audio Hacker Library
  Copyright (C) 2013 nootropic design, LLC
  All rights reserved.

  Cosmic Looper code is largely modified and combined example code
  from the NooTropic Design Audio Hacker Project - though some
  refactors are my own design.

  Special thanks to E. Scott Eastman & David Lowenfels for mentorship
  and guidance.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  A 12-bit sampler to record sampled audio to SRAM.
  The SRAM is divided into 4 equal parts so that 4 samples can be
  recorded.

  This sketch is designed for 6 Arcade Buttons & 4 10k potentiometers, button pinout is:
  Record button = D5
  Sample 0 button = D6
  Sample 1 button = D4
  Sample 2 button = D3
  Sample 3 button = D2
  Reverse button = D8

  To record a sample, press and hold the record button, then hold a sample button
  for the recording duration.
  To play a sample, press and hold the corresponding sample button.
  To play a smaple in reverse tap (don't hold) the reverse button either before playing a sample or while the sample is playing.

  Effect Pot Assignments are:
  Sample Rate: A0
  Bit Crushing: A1
  Low Pass Filter Cutoff: A2
  High Pass Fiter Cutoff: A3

  Both filter Resonances are static at 150         

  Input is sampled at 16 kHz and reproduced on the output.
  Recordings sampled at 16 kHz and stored to SRAM.
 */

#include <EEPROM.h>
#include <AudioHacker.h>

#define longPressed 1
#define holdingPress 2
#define shortPressed 3
#define longRelease 4
#define BUTTON_ON LOW
#define BUTTON_OFF HIGH
#define DEBUG
#define OFF 0
#define PASSTHROUGH 1
#define RECORD 2
#define PLAYBACK 3
#define RECORD_DONE 4
#define RECORD_BUTTON 5
#define SAMPLE0_BUTTON 6
#define SAMPLE1_BUTTON 4
#define SAMPLE2_BUTTON 3
#define SAMPLE3_BUTTON 2
#define REVERSE_BUTTON 8
#define LOW_PASS 7
#define BAND_PASS 8
#define HIGH_PASS 9

//Playback, Sample Rate, and Sample Address Variables
unsigned int playbackBuf = 2048;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int playbackSampleRate;
byte resolution = 12;
unsigned int mask;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
volatile boolean warning = false;
int currentA0Position;
volatile long address = 0;
volatile long endAddress[4];
volatile byte addressChipNumber = 0;

//Button State Variables
int reading[6];
int buttonState[6] = {BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF};
int lastButtonState[6] = {BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON};
int lastShortPressState[6] = {BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF};
int currentShortPressState[6] = {BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF};
unsigned long lastDebounceTime[6];
unsigned long timeOfLastButtonChange[6];
unsigned long timeSinceLastButtonChange[6];
unsigned long debounceDelay = 20;
unsigned long gateThreshold = 300;

int recordButton = BUTTON_OFF;
int sample0Button = BUTTON_OFF;
int sample1Button = BUTTON_OFF;
int sample2Button = BUTTON_OFF;
int sample3Button = BUTTON_OFF;

//Reverse State Variables
int reverseButton = BUTTON_OFF;
int playbackDirection;
volatile long playbackDirectionReset[4];
volatile long playbackDirectionStart[4] = {0, 65535, 0, 65535};

// Filter Variables
int LPfilterCutoff = 255;
int LPfilterResonance = 150;
long LPfeedback;
int LPbuf0 = 0;
int LPbuf1 = 0;

int HPfilterCutoff = 255;
int HPfilterResonance = 150;
long HPfeedback;
int HPbuf0 = 0;
int HPbuf1 = 0;



volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
unsigned int readBuf[2];
unsigned int writeBuf;
boolean evenCycle = true;

// set to true if you are using a battery backup on the
// SRAM and want to keep sample address info in EEPROM
boolean batteryBackup = true;

unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded[4];
byte sample;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  recordingSampleRate = 16000;
  passthroughSampleRate = 16000;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(SAMPLE0_BUTTON, INPUT);
  pinMode(SAMPLE1_BUTTON, INPUT);
  pinMode(SAMPLE2_BUTTON, INPUT);
  pinMode(SAMPLE3_BUTTON, INPUT);
  pinMode(REVERSE_BUTTON, INPUT);

  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(SAMPLE0_BUTTON, HIGH);
  digitalWrite(SAMPLE1_BUTTON, HIGH);
  digitalWrite(SAMPLE2_BUTTON, HIGH);
  digitalWrite(SAMPLE3_BUTTON, HIGH);
  digitalWrite(REVERSE_BUTTON, HIGH);

  AudioHacker.begin();

#ifdef DEBUG
  Serial.print("sample rate = ");
  Serial.print(passthroughSampleRate);
  Serial.print(" Hz, recording sample rate = ");
  Serial.print(recordingSampleRate);
  Serial.print(" Hz");
  Serial.println();
#endif

  sampleRecorded[0] = false;
  sampleRecorded[1] = false;
  sampleRecorded[2] = false;
  sampleRecorded[3] = false;

  if (batteryBackup) {
    // Read endAddress[] values from EEPROM when we have a battery
    // connected to the Audio Hacker to preserve SRAM contents.
    for (byte i=0;i<4;i++) {
      byte a = i*3;
      long b;
      b = (long)EEPROM.read(a);
      endAddress[i] = (b << 16);
      b = (long)EEPROM.read(a+1);
      endAddress[i] |= (b << 8);
      b = (long)EEPROM.read(a+2);
      endAddress[i] |= b;

      if (endAddress[i] > 0) {
	       sampleRecorded[i] = true;
#ifdef DEBUG
	       Serial.print("sample ");
	       Serial.print(i);
	       Serial.print(" endAddress = ");
	       Serial.print(endAddress[i]);
	       Serial.println();
#endif
      }
    }
  }
}

void loop() {
  //Sample Rate Manipulation
  recordingSampleRate = map(analogRead(0), 0, 1023, 1000, 16000);
  recordingSampleRate = recordingSampleRate - (recordingSampleRate % 100);
  passthroughSampleRate = map(analogRead(0), 0, 1023, 1000, 16000);
  passthroughSampleRate = passthroughSampleRate - (passthroughSampleRate % 100);

  //Button State Check
  reading[0] = digitalRead(SAMPLE0_BUTTON);
  reading[1] = digitalRead(SAMPLE1_BUTTON);
  reading[2] = digitalRead(SAMPLE2_BUTTON);
  reading[3] = digitalRead(SAMPLE3_BUTTON);
  reading[4] = digitalRead(RECORD_BUTTON);
  reading[5] = digitalRead(REVERSE_BUTTON);

  //Record Button = [4] reading

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading[4] != lastButtonState[4]) {
    // reset the debouncing timer
    lastDebounceTime[4] = millis();
  }

  if ((millis() - lastDebounceTime[4]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading[4] != buttonState[4]) {
        buttonState[4] = reading[4];
    }
  }

  // set the button state:
  recordButton = buttonState[4];

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState[4] = reading[4];

  //Sample 0 Button = [0] reading

  if (reading[0] != lastButtonState[0]) {
    lastDebounceTime[0] = millis();
  }

  if ((millis() - lastDebounceTime[0]) > debounceDelay) {
    if (reading[0] != buttonState[0]) {
        buttonState[0] = reading[0];
    }
  }

  sample0Button = buttonState[0];
  lastButtonState[0] = reading[0];

  //Sample 1 Button = [1] reading

  if (reading[1] != lastButtonState[1]) {
    lastDebounceTime[1] = millis();
  }

  if ((millis() - lastDebounceTime[1]) > debounceDelay) {
    
    if (reading[1] != buttonState[1]) {
        buttonState[1] = reading[1];
    }
  }

  sample1Button = buttonState[1];
  lastButtonState[1] = reading[1];
  
  //Sample 2 Button = [2] reading
  if (reading[2] != lastButtonState[2]) {

    lastDebounceTime[2] = millis();
  }

  if ((millis() - lastDebounceTime[2]) > debounceDelay) {

    if (reading[2] != buttonState[2]) {
        buttonState[2] = reading[2];
    }
  }

  sample2Button = buttonState[2];
  lastButtonState[2] = reading[2];

   //Sample 3 Button = [3] reading
  if (reading[3] != lastButtonState[3]) {

    lastDebounceTime[3] = millis();
  }

  if ((millis() - lastDebounceTime[3]) > debounceDelay) {

    if (reading[3] != buttonState[3]) {
        buttonState[3] = reading[3];
    }
  }

  sample3Button = buttonState[3];
  lastButtonState[3] = reading[3];

 // Reverse Button = [5] reading

  if (reading[5] != lastButtonState[5]) { // if pin state has changed from the last reading
    timeOfLastButtonChange[5] = millis(); // if pin state different, store the time of the state change
  }
  timeSinceLastButtonChange[5] = millis() - timeOfLastButtonChange[5];

  if (reading[5] == BUTTON_ON && timeSinceLastButtonChange[5] < debounceDelay) {
    // has only been low for less than the debounce time - do nothing
  }
  if (reading[5] == BUTTON_ON && timeSinceLastButtonChange[5] > gateThreshold && buttonState[5] != longPressed) {
      buttonState[5] = longPressed;
      reverseButton = BUTTON_ON;
  }
  if (reading[5] == BUTTON_ON && timeSinceLastButtonChange[5] > debounceDelay && timeSinceLastButtonChange[5] < gateThreshold && buttonState[5] != holdingPress) {
      buttonState[5] = holdingPress;
      reverseButton = BUTTON_ON;
  }
  if (reading[5] == BUTTON_OFF && timeSinceLastButtonChange[5] > debounceDelay && buttonState[5] == holdingPress) {
      buttonState[5] = shortPressed;
      if (currentShortPressState[5] == BUTTON_OFF && lastShortPressState[5] == BUTTON_ON){
      if (reverseButton == BUTTON_OFF) 
      reverseButton = BUTTON_ON;
      else
      reverseButton = BUTTON_OFF;
  }
  lastShortPressState[5] = reverseButton;
  }
  if (reading[5] == BUTTON_OFF && timeSinceLastButtonChange[5] > debounceDelay && buttonState[5] == longPressed) {
      buttonState[5] = longRelease;
      reverseButton = BUTTON_OFF;   
  }
   
  lastButtonState[5] = reading[5];
  
  //Bit Crushing Pot Read
  resolution = map(analogRead(1), 0, 1023, 1, 12);
  mask = 0x0FFF << (12-resolution);

  // Dual Filters Pot State Read - Cutoff only, Resonance is static
  LPfilterCutoff = analogRead(2) >> 2;
  HPfilterCutoff = analogRead(3) >> 2;

  LPfeedback = (long)LPfilterResonance + (long)(((long)LPfilterResonance * ((int)255 - (255-LPfilterCutoff))) >> 8);
  HPfeedback = (long)HPfilterResonance + (long)(((long)HPfilterResonance * ((int)255 - (255-HPfilterCutoff))) >> 8);

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();

    // Print the number of instruction cycles remaining at the end of the ISR.
    // The more work you try to do in the ISR, the lower this number will become.
    // If the number of cycles remaining reaches 0, then the ISR will take up
    // all the CPU time and the code in loop() will not run.

    /*
    Serial.print("even cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndEven);
    Serial.print("   odd cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndOdd);
    Serial.println();
    if (((UINT16_MAX - timer1EndEven) < 20) || (((UINT16_MAX - timer1EndOdd) < 20))) {
      Serial.println("WARNING: ISR execution time is too long. Reduce sample rate or reduce the amount of code in the ISR.");
    }
    */
  }
#endif

  if ((mode == OFF) || (mode == PASSTHROUGH)) {
    if ((recordButton == BUTTON_ON) && ((sample0Button == BUTTON_ON) || (sample1Button == BUTTON_ON) || (sample2Button == BUTTON_ON) || (sample3Button == BUTTON_ON))) {
      // enter RECORD mode
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
        // debounce the record button.
        recordStartTime = 0;
        return;
      }
      if (sample0Button == BUTTON_ON) {
        sample = 0;
        address = 0;
        addressChipNumber = 0;
      }
      if (sample1Button == BUTTON_ON) {
        sample = 1;
        address = 65535;
        addressChipNumber = 0;
      }
      if (sample2Button == BUTTON_ON) {
        sample = 2;
        address = 0;
        addressChipNumber = 1;
      }
      if (sample3Button == BUTTON_ON) {
        sample = 3;
        address = 65535;
        addressChipNumber = 1;
      }
      mode = RECORD;
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
      currentA0Position = analogRead(0);
    } else {
      // enter PLAYBACK mode
      if ((sample0Button == BUTTON_ON) && (sampleRecorded[0])) {
        address = playbackDirectionStart[0];
        addressChipNumber = 0;
        sample = 0;
        mode = PLAYBACK;
      }
      if ((sample1Button == BUTTON_ON) && (sampleRecorded[1])) {
        address = playbackDirectionStart[1];
        addressChipNumber = 0;
        sample = 1;
        mode = PLAYBACK;
      }
      if ((sample2Button == BUTTON_ON) && (sampleRecorded[2])) {
        address = playbackDirectionStart[2];
        addressChipNumber = 1;
        sample = 2;
        mode = PLAYBACK;
      }
      if ((sample3Button == BUTTON_ON) && (sampleRecorded[3])) {
        address = playbackDirectionStart[3];
        addressChipNumber = 1;
        sample = 3;
        mode = PLAYBACK;
      }
    }
  }

  if (mode == PASSTHROUGH) {
    timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
  }

  if (mode == RECORD) {
    if (((sample == 0) && (sample0Button == BUTTON_OFF)) ||
        ((sample == 1) && (sample1Button == BUTTON_OFF)) ||
        ((sample == 2) && (sample2Button == BUTTON_OFF)) ||
        ((sample == 3) && (sample3Button == BUTTON_OFF))) {
          // recording stopped
          recordEndTime = millis();
          if (recordEndTime - recordStartTime < 20) {
            // debounce
            return;
          }
          sampleRecorded[sample] = true;
          endAddress[sample] = address;
#ifdef DEBUG
          Serial.print("sample ");
          Serial.print(sample);
          Serial.print(" recording time = ");
          Serial.print(recordEndTime - recordStartTime);
          Serial.println(" ms");
          Serial.print(" endAddress = ");
          Serial.println(endAddress[sample]);
#endif

          if (batteryBackup) {
            // Write endAddress to EEPROM for battery backup use.
            byte a = sample*3;
            EEPROM.write(a, (endAddress[sample] >> 16) & 0xFF);
            EEPROM.write(a+1, (endAddress[sample] >> 8) & 0xFF);
            EEPROM.write(a+2, endAddress[sample] & 0xFF);
          }
          mode = PASSTHROUGH;
        }
  } else {
  }

  if (mode == RECORD_DONE) {
    if (recordStartTime != 0) {
#ifdef DEBUG
      Serial.print("sample ");
      Serial.print(sample);
      Serial.print(" recording time = ");
      Serial.print(millis() - recordStartTime);
      Serial.println(" ms");
      Serial.print(" endAddress = ");
      Serial.println(endAddress[sample]);
#endif
      sampleRecorded[sample] = true;
      recordStartTime = 0;

      if (batteryBackup) {
        // Write endAddress to EEPROM for battery backup use.
        byte a = sample*3;
        EEPROM.write(a, (endAddress[sample] >> 16) & 0xFF);
        EEPROM.write(a+1, (endAddress[sample] >> 8) & 0xFF);
        EEPROM.write(a+2, endAddress[sample] & 0xFF);
      }
    }
    if (recordButton == BUTTON_OFF) {
      // record button released
      mode = PASSTHROUGH;
    }
  }

  if (mode == PLAYBACK) {
    if (((sample == 0) && (sample0Button == BUTTON_OFF)) ||
        ((sample == 1) && (sample1Button == BUTTON_OFF)) ||
        ((sample == 2) && (sample2Button == BUTTON_OFF)) ||
        ((sample == 3) && (sample3Button == BUTTON_OFF))) {
          // play button released
          mode = PASSTHROUGH;
    } else {
      
      playbackSampleRate = map(analogRead(0), 0, 1023, 1000, 16000);
      
      // compute the start value for counter1 to achieve the chosen playback rate
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);
    }
  }
 //Play Direction Address Update 
 if (reverseButton == BUTTON_ON){
     playbackDirection = -3;
    
     playbackDirectionReset[0] = 0;
     playbackDirectionReset[1] = 65535;
     playbackDirectionReset[2] = 0;
     playbackDirectionReset[3] = 65535;
    
     playbackDirectionStart[0] = endAddress[0] - 3;
     playbackDirectionStart[1] = endAddress[1] - 3;
     playbackDirectionStart[2] = endAddress[2] - 3;
     playbackDirectionStart[3] = endAddress[3] - 3;
     }else{
     playbackDirection = 3;
     playbackDirectionReset[0] = endAddress[0];
     playbackDirectionReset[1] = endAddress[1];
     playbackDirectionReset[2] = endAddress[2];
     playbackDirectionReset[3] = endAddress[3];

     playbackDirectionStart[0] = 0;
     playbackDirectionStart[1] = 65535;
     playbackDirectionStart[2] = 0;
     playbackDirectionStart[3] = 65535;
     }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  int mix;
  unsigned int signal;

  if (mode != RECORD_DONE) {
    AudioHacker.writeDAC(playbackBuf);
  }

  if ((mode != PLAYBACK) && (mode != RECORD_DONE)) {
    // Read ADC
    signal = AudioHacker.readADC(); 
    mix = signal - 2048;
     
  }

  if (mode == RECORD) {
    if (evenCycle) {
      // we only write to memory on odd cycles, so buffer the sampled signal.
      writeBuf = signal;
    } else {
      // Write to SRAM
      AudioHacker.writeSRAMPacked(addressChipNumber, address, writeBuf, signal);

      address += 3;
      if (((sample == 0) && (address > 65532)) ||
          ((sample == 1) && (address > MAX_ADDR)) ||
          ((sample == 2) && (address > 65532)) ||
          ((sample == 3) && (address > MAX_ADDR))) {
            // end of memory, stop recording
            mode = RECORD_DONE;
            endAddress[sample] = address;
      }
    }
  }


  if (mode == PLAYBACK) {
    if (evenCycle) {
      // Read from SRAM
      unsigned int passbuff = AudioHacker.readADC();
      AudioHacker.readSRAMPacked(addressChipNumber, address, readBuf);
      signal = readBuf[0];
      mix = (passbuff - 2048) + (signal - 2048);

      if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }
      
      address += playbackDirection;
      if ((sample == 0) && (address == playbackDirectionReset[0])) {
        address = playbackDirectionStart[0];
        addressChipNumber = 0;
      }
      if ((sample == 1) && (address == playbackDirectionReset[1])) {
        address = playbackDirectionStart[1];
        addressChipNumber = 0;
      }
      if ((sample == 2) && (address == playbackDirectionReset[2])) {
        address = playbackDirectionStart[2];
        addressChipNumber = 1;
      }
      if ((sample == 3) && (address == playbackDirectionReset[3])) {
        address = playbackDirectionStart[3];
        addressChipNumber = 1;
      }
    } else {
      unsigned int passbuff = AudioHacker.readADC();
      signal = readBuf[1];
      mix = (passbuff - 2048) + (signal - 2048);

      //Prevents Playback Clipping
      if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
     }    
    }
   } // PLAYBACK

  //Interupt Filter Implementation
  int highPass1 = mix - LPbuf0;
  int bandPass1 = LPbuf0 - LPbuf1;
  int tmp1 = highPass1 + (LPfeedback * bandPass1 >> 8);
  LPbuf0 += ((long)LPfilterCutoff * tmp1) >> 8;
  LPbuf1 += ((long)LPfilterCutoff * (LPbuf0 - LPbuf1)) >> 8;
  mix = LPbuf1;

  int highPass2 = mix - HPbuf0;
  int bandPass2 = HPbuf0 - HPbuf1;
  int tmp2 = highPass2 + (HPfeedback * bandPass2 >> 8);
  HPbuf0 += ((long)HPfilterCutoff * tmp2) >> 8;
  HPbuf1 += ((long)HPfilterCutoff * (HPbuf0 - HPbuf1)) >> 8;
  mix = highPass2 + 2048;
 
  playbackBuf = mix;
  playbackBuf &= mask;

#ifdef DEBUG
  if (evenCycle) {
    timer1EndEven = TCNT1;
  } else {
    timer1EndOdd = TCNT1;
  }
#endif
  evenCycle = !evenCycle;
}
