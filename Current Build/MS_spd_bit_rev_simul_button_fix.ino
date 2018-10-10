/*
  Audio Hacker Library
  Copyright (C) 2013 nootropic design, LLC
  All rights reserved.

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

  This sketch is designed for use with the DJ Shield but can be used with
  buttons on a breadboard:
  Record button = D5
  Sample 0 button = D6
  Sample 1 button = D4
  Sample 2 button = D3
  Sample 3 button = D2
  Reverse button = D8

  To record a sample, press and hold the record button, then hold a sample button
  for the recording duration.
  To play a sample, press and hold the corresponding sample button.

  Bit Crushing, Sample Rate Manipulation, Dual Low Pass Filter & High Pass Filter, and Reverse Playback added.

  To do: High Pass filter doesn't work on Reverse Playback.
         
         

  Input is sampled at 10.025 kHz and reproduced on the output.
  Recordings sampled at 10.025 kHz and stored to SRAM.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */



#include <EEPROM.h>
#include <AudioHacker.h>

#define longPressed 1
#define holdingPress 2
#define shortPressed 3
#define longRelease 4
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
#define LOW_PASS 7
#define BAND_PASS 8
#define HIGH_PASS 9
#define REVERSE 10
#define REVERSE_BUTTON 8

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
boolean adjustablePlaybackSpeed = true;  // set to true with pot connected to A0
int currentA0Position;
volatile long address = 0;
volatile long endAddress[4];
volatile byte addressChipNumber = 0;

int reading[6];
int buttonState[6];
int lastButtonState[6];
int lastShortPressState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
int currentShortPressState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[6];
unsigned long timeOfLastButtonChange[6];
unsigned long timeSinceLastButtonChange[6];
unsigned long debounceDelay = 50;
unsigned long gateThreshold = 300;

int revButton = HIGH;

int LPfilterCutoff = 255;
int LPfilterResonance = 0;
long LPfeedback;
int LPbuf0 = 0;
int LPbuf1 = 0;

int HPfilterCutoff = 255;
int HPfilterResonance = 0;
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

  recordingSampleRate = 10025;
  passthroughSampleRate = 10025;
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
  recordingSampleRate = map(analogRead(0), 0, 1023, 1000, 10025);
  recordingSampleRate = recordingSampleRate - (recordingSampleRate % 100);
  passthroughSampleRate = map(analogRead(0), 0, 1023, 1000, 10025);
  passthroughSampleRate = passthroughSampleRate - (passthroughSampleRate % 100);

  //Reverse Playback On / Off

  reading[5] = digitalRead(REVERSE_BUTTON);

  if (reading[5] != lastButtonState[5]) { // if pin state has changed from the last reading
    timeOfLastButtonChange[5] = millis(); // if pin state different, store the time of the state change
  }
  timeSinceLastButtonChange[5] = millis() - timeOfLastButtonChange[5];

  if (reading[5] == LOW && timeSinceLastButtonChange[5] < debounceDelay) {
    // has only been low for less than the debounce time - do nothing
  }
  if (reading[5] == LOW && timeSinceLastButtonChange[5] > gateThreshold && buttonState[5] != longPressed) {
      buttonState[5] = longPressed;
      revButton = LOW;
  }
  if (reading[5] == LOW && timeSinceLastButtonChange[5] > debounceDelay && timeSinceLastButtonChange[5] < gateThreshold && buttonState[5] != holdingPress) {
      buttonState[5] = holdingPress;
      revButton = LOW;
  }
  if (reading[5] == HIGH && timeSinceLastButtonChange[5] > debounceDelay && buttonState[5] == holdingPress) {
      buttonState[5] = shortPressed;
      if (currentShortPressState[5] == HIGH && lastShortPressState[5] == LOW){
      if (revButton == HIGH) 
      revButton = LOW;
      else
      revButton = HIGH;
  }
  lastShortPressState[5] = revButton;
  }
  if (reading[5] == HIGH && timeSinceLastButtonChange[5] > debounceDelay && buttonState[5] == longPressed) {
      buttonState[5] = longRelease;
      revButton = HIGH;   
  }
   
  lastButtonState[5] = reading[5];
  
  //Bit Crushing
  resolution = map(analogRead(1), 0, 1023, 1, 12);
  mask = 0x0FFF << (12-resolution);

  // Dual Filters
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
    if ((digitalRead(RECORD_BUTTON) == LOW) && ((digitalRead(SAMPLE0_BUTTON) == LOW) || (digitalRead(SAMPLE1_BUTTON) == LOW) || (digitalRead(SAMPLE2_BUTTON) == LOW) || (digitalRead(SAMPLE3_BUTTON) == LOW))) {
      // enter RECORD mode
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
        // debounce the record button.
        recordStartTime = 0;
        return;
      }
      if (digitalRead(SAMPLE0_BUTTON) == LOW) {
        sample = 0;
        address = 0;
        addressChipNumber = 0;
      }
      if (digitalRead(SAMPLE1_BUTTON) == LOW) {
        sample = 1;
        address = 65535;
        addressChipNumber = 0;
      }
      if (digitalRead(SAMPLE2_BUTTON) == LOW) {
        sample = 2;
        address = 0;
        addressChipNumber = 1;
      }
      if (digitalRead(SAMPLE3_BUTTON) == LOW) {
        sample = 3;
        address = 65535;
        addressChipNumber = 1;
      }
      mode = RECORD;
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
      currentA0Position = analogRead(0);
    } else {
      // enter PLAYBACK mode
      if ((digitalRead(SAMPLE0_BUTTON) == LOW) && (sampleRecorded[0])) {
        address = 0;
        addressChipNumber = 0;
        sample = 0;
        mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE1_BUTTON) == LOW) && (sampleRecorded[1])) {
        address = 65535;
        addressChipNumber = 0;
        sample = 1;
        mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE2_BUTTON) == LOW) && (sampleRecorded[2])) {
        address = 0;
        addressChipNumber = 1;
        sample = 2;
        mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE3_BUTTON) == LOW) && (sampleRecorded[3])) {
        address = 65535;
        addressChipNumber = 1;
        sample = 3;
        mode = PLAYBACK;
      }
 if (revButton == LOW && ((digitalRead(SAMPLE0_BUTTON) == LOW) || (digitalRead(SAMPLE1_BUTTON) == LOW) || (digitalRead(SAMPLE2_BUTTON) == LOW) || (digitalRead(SAMPLE3_BUTTON) == LOW))) {
          if ((digitalRead(SAMPLE0_BUTTON) == LOW) && (sampleRecorded[0])){
          address = endAddress[0] - 3;
          addressChipNumber = 0;
          sample = 0;
          mode = REVERSE;
          }
         if ((digitalRead(SAMPLE1_BUTTON) == LOW) && (sampleRecorded[1])){
         address = endAddress[1] - 3;
         addressChipNumber = 0;
         sample = 1;
         mode = REVERSE;
         }
         if ((digitalRead(SAMPLE2_BUTTON) == LOW) && (sampleRecorded[2])){
         address = endAddress[2] - 3;
         addressChipNumber = 1;
         sample = 2;
         mode = REVERSE;
         }
         if ((digitalRead(SAMPLE3_BUTTON) == LOW) && (sampleRecorded[3])){
         address = endAddress[3] - 3;
         addressChipNumber = 1;
         sample = 3;
         mode = REVERSE;
         }    
      }
    }
  }

  if (mode == PASSTHROUGH) {
    timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
  }

  if (mode == RECORD) {
    if (((sample == 0) && (digitalRead(SAMPLE0_BUTTON) == HIGH)) ||
        ((sample == 1) && (digitalRead(SAMPLE1_BUTTON) == HIGH)) ||
        ((sample == 2) && (digitalRead(SAMPLE2_BUTTON) == HIGH)) ||
        ((sample == 3) && (digitalRead(SAMPLE3_BUTTON) == HIGH))) {
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
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      mode = PASSTHROUGH;
    }
  }

  if (mode == PLAYBACK  || (mode == REVERSE)) {
    if (((sample == 0) && (digitalRead(SAMPLE0_BUTTON) == HIGH)) ||
        ((sample == 1) && (digitalRead(SAMPLE1_BUTTON) == HIGH)) ||
        ((sample == 2) && (digitalRead(SAMPLE2_BUTTON) == HIGH)) ||
        ((sample == 3) && (digitalRead(SAMPLE3_BUTTON) == HIGH))) {
          // play button released
          mode = PASSTHROUGH;
    } else {
      if (adjustablePlaybackSpeed) {
        playbackSampleRate = map(analogRead(0), 0, 1023, 1000, 10025);
      } 
      // compute the start value for counter1 to achieve the chosen playback rate
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);
    }
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  int mix;
  unsigned int signal;

  if (mode != RECORD_DONE) {
    AudioHacker.writeDAC(playbackBuf);
  }

  if ((mode != PLAYBACK) && (mode != REVERSE) && (mode != RECORD_DONE)) {
    // Read ADC
    signal = AudioHacker.readADC() - 2048;
    int highPass1 = signal - LPbuf0;
    int bandPass1 = LPbuf0 - LPbuf1;
    int tmp1 = highPass1 + (LPfeedback * bandPass1 >> 8);
    LPbuf0 += ((long)LPfilterCutoff * tmp1) >> 8;
    LPbuf1 += ((long)LPfilterCutoff * (LPbuf0 - LPbuf1)) >> 8;
    signal = LPbuf1;

    int highPass2 = signal - HPbuf0;
    int bandPass2 = HPbuf0 - HPbuf1;
    int tmp2 = highPass2 + (HPfeedback * bandPass2 >> 8);
    HPbuf0 += ((long)HPfilterCutoff * tmp2) >> 8;
    HPbuf1 += ((long)HPfilterCutoff * (HPbuf0 - HPbuf1)) >> 8;
    signal = highPass2 + 2048;
    
    mix = signal;
     
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
      
      address += 3;
      if ((sample == 0) && (address == endAddress[0])) {
        address = 0;
        addressChipNumber = 0;
      }
      if ((sample == 1) && (address == endAddress[1])) {
        address = 65535;
        addressChipNumber = 0;
      }
      if ((sample == 2) && (address == endAddress[2])) {
        address = 0;
        addressChipNumber = 1;
      }
      if ((sample == 3) && (address == endAddress[3])) {
        address = 65535;
        addressChipNumber = 1;
      }
    } else {
      unsigned int passbuff = AudioHacker.readADC();
      signal = readBuf[1];
      mix = (passbuff - 2048) + (signal - 2048);

      if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }

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
    
    }
  } // PLAYBACK

  if (mode == REVERSE) {
    if (evenCycle) {
      // Read from SRAM
      unsigned int passbuff = AudioHacker.readADC();
      AudioHacker.readSRAMPacked(addressChipNumber, address, readBuf);
      signal = readBuf[1];
      mix = (passbuff - 2048) + (signal - 2048);

      if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }
    
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

      address -= 3;
      if ((sample == 0) && (address == 0)){
        if (addressChipNumber == 0) {
        address = endAddress[0] - 3;
        addressChipNumber = 0;
      }
      }  
      if ((sample == 1) && (address == 65535)){
        if (addressChipNumber == 0) {
        address = endAddress[1] - 3;
        addressChipNumber = 0;
      }
      }     
      if ((sample == 2) && (address == 0)) {
        if (addressChipNumber == 1) {
        address = endAddress[2] - 3;
        addressChipNumber = 1;
      }
      }
      if ((sample == 3) && (address == 65535)) {
        if (addressChipNumber == 1) {
        address = endAddress[3] - 3;
        addressChipNumber = 1;
      }
      }
    } else {
      unsigned int passbuff = AudioHacker.readADC();
      signal = readBuf[0];
      mix = (passbuff - 2048) + (signal - 2048);

      if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }

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
      
      }
    }
 
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
