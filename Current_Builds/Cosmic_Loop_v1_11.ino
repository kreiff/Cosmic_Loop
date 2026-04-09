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

  This sketch is designed for 8 Arcade Buttons & 6 10k potentiometers, button pinout is:
  Record button = D5
  Sample 0 button = D6
  Sample 1 button = D4
  Sample 2 button = D3
  Sample 3 button = D2
  Reverse button = D1
  Grain Delay button = D0
  Input / Passthrough Mute = Hardwired / Soldered to Input pin of Bypass Switch

  To record a sample, press and hold the record button, then hold a sample button
  for the recording duration.
  Recording threshold has been added to this build. Recording will not start until
  the passthrough signal is above about -50dB.
  To play a sample, press and hold the corresponding sample button.
  To play a sample in reverse tap to latch reverse mode, hold for momentary reverse
  playback.
  To turn on Grain Delay tap the button connected to D0, hold for momentary Grain
  playback.

  Filter can be turned off / on by pressing the record and reverse buttons
  simultaneously. This also works in a momentary fashion by holding both buttons and
  then letting go (hold is filter on, let go is filter off).

  The Playback buffer can be frozen by holding the Record button during playback.
  The size of the freeze window can be altered using the Grain Size & Grain Window
  pots.

  Effect Pot Assignments are:
  Sample Rate:            A0
  Bit Crushing:           A1
  Low Pass Filter Cutoff: A2
  High Pass Filter Cutoff:A3
  Grain Delay Window Size:A4
  Grain Size:             A5

  Both filter Resonances are static at 150.

  Input is sampled at 16 kHz and reproduced on the output.
  Recordings sampled at 16 kHz and stored to SRAM.
*/

#include <EEPROM.h>
#include "AudioHacker.h"

#define LONG_PRESSED   1
#define HOLDING_PRESS  2
#define SHORT_PRESSED  3
#define LONG_RELEASE   4
#define BUTTON_ON      LOW
#define BUTTON_OFF     HIGH
#define DEBUG
#define OFF            0
#define PASSTHROUGH    1
#define RECORD         2
#define PLAYBACK       3
#define RECORD_DONE    4
#define RECORD_BUTTON  5
#define SAMPLE0_BUTTON 6
#define SAMPLE1_BUTTON 4
#define SAMPLE2_BUTTON 3
#define SAMPLE3_BUTTON 2
#define REVERSE_BUTTON 1
#define GRAIN_BUTTON   0
#define LOW_PASS       7
#define BAND_PASS      8
#define HIGH_PASS      9

// ─── Playback / sample-rate / address variables ─────────────────────────────
unsigned int playbackBuf = 2048;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int playbackSampleRate;
byte         resolution = 12;

volatile uint16_t mask = 0x0FFF;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
volatile boolean warning = false;
int  currentA0Position;
volatile long address = 0;
volatile long endAddress[4];
volatile byte addressChipNumber = 0;

// ─── Button state variables ──────────────────────────────────────────────────
int pins[]          = { 6, 4, 3, 2, 5, 1, 0 };
int reading[7];
int buttonState[7]          = {BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF};
int lastButtonState[7]      = {BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON, BUTTON_ON};
int lastShortPressState[7]  = {BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF};
int currentShortPressState[7]={BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF,BUTTON_OFF};
unsigned long lastDebounceTime[7];
unsigned long timeOfLastButtonChange[7];
unsigned long timeSinceLastButtonChange[7];
unsigned long debounceDelay  = 20;
unsigned long gateThreshold  = 300;

int recordButton  = BUTTON_OFF;
int sample0Button = BUTTON_OFF;
int sample1Button = BUTTON_OFF;
int sample2Button = BUTTON_OFF;
int sample3Button = BUTTON_OFF;
int grainButton   = BUTTON_OFF;

// ─── Reverse state variables ─────────────────────────────────────────────────

int  reverseButton = BUTTON_OFF;
volatile int  playbackDirection = 3;
// Used only in loop() for the initial address when entering PLAYBACK:
long playbackDirectionStart[4] = {0, 65535, 0, 65535};

// ─── Filter variables ────────────────────────────────────────────────────────
int  LPfilterCutoff    = 255;
int  LPfilterResonance = 150;
long LPfeedback;
int  LPbuf0 = 0;
int  LPbuf1 = 0;

int  HPfilterCutoff    = 255;
int  HPfilterResonance = 150;
long HPfeedback;
int  HPbuf0 = 0;
int  HPbuf1 = 0;

int filterButton         = BUTTON_ON;
int filterState;
int lastFilterState      = BUTTON_ON;
int currentShortFilterState = BUTTON_OFF;
int lastShortFilterState    = BUTTON_OFF;

// ─── Grain delay variables ───────────────────────────────────────────────────
unsigned int nSamplesToPlay  = 2048;
volatile unsigned int nSamplesPlayed = 0;
unsigned int grainRead;
unsigned int grainSize;           // in address units (multiples of 3)
byte stretch = 1;
volatile long grainAddress        = 0;
volatile byte grainAddressChipNumber = 0;

volatile boolean grainBoundaryPending = false;
volatile int     lastGrainSignal = 0;

// ─── Freeze buffer variables ─────────────────────────────────────────────────
int freezeButton = BUTTON_OFF;
int lastFreezeButton = BUTTON_OFF;
volatile long freezeAddressReset = 0;
volatile long freezeBuffer;

// ─── Read / write buffer & mode variables ────────────────────────────────────
volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
#define POT_READ_INTERVAL_MS 50
unsigned long lastPotRead = 0;

// ─── ADC noise gate ───────────────────────────────────────────────────────────
#define NOISE_GATE_THRESHOLD 24
#define INPUT_GATE_OPEN_THRESHOLD  40
#define INPUT_GATE_CLOSE_THRESHOLD 28
#define INPUT_GATE_HOLD_SAMPLES    96
unsigned int  readBuf[2];
unsigned int  writeBuf;
boolean evenCycle = true;
int  inputEnvelope = 0;
byte inputGateHold = 0;
bool inputGateOpen = false;

// Set to true if using battery backup on SRAM
boolean batteryBackup = true;

unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded[4];
byte sample;
byte previousMode = PASSTHROUGH;
int potSmooth[6] = {512, 1023, 1023, 0, 512, 512};
bool potsInitialized = false;
int dcBlockLastInput = 0;
long dcBlockState = 0;

#define POT_SMOOTH_SHIFT 2
#define DC_BLOCK_COEFFICIENT 252

static inline void smoothPotValue(byte index, int rawValue) {
  if (!potsInitialized) {
    potSmooth[index] = rawValue;
  } else {
    potSmooth[index] += (rawValue - potSmooth[index]) >> POT_SMOOTH_SHIFT;
  }
}

static inline void resetToneState() {
  LPbuf0 = 0;
  LPbuf1 = 0;
  HPbuf0 = 0;
  HPbuf1 = 0;
  dcBlockLastInput = 0;
  dcBlockState = 0;
}

static inline int applyDcBlock(int centeredSample) {
  long filtered = (long)centeredSample - (long)dcBlockLastInput +
                  ((dcBlockState * DC_BLOCK_COEFFICIENT) >> 8);
  dcBlockLastInput = centeredSample;
  dcBlockState = filtered;

  if (filtered < -2048) return -2048;
  if (filtered > 2047)  return 2047;
  return (int)filtered;
}

// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  recordingSampleRate  = 16000;
  passthroughSampleRate = 16000;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON,  INPUT);
  pinMode(SAMPLE0_BUTTON, INPUT);
  pinMode(SAMPLE1_BUTTON, INPUT);
  pinMode(SAMPLE2_BUTTON, INPUT);
  pinMode(SAMPLE3_BUTTON, INPUT);
  pinMode(REVERSE_BUTTON, INPUT);
  pinMode(GRAIN_BUTTON,   INPUT);

  digitalWrite(RECORD_BUTTON,  HIGH);
  digitalWrite(SAMPLE0_BUTTON, HIGH);
  digitalWrite(SAMPLE1_BUTTON, HIGH);
  digitalWrite(SAMPLE2_BUTTON, HIGH);
  digitalWrite(SAMPLE3_BUTTON, HIGH);
  digitalWrite(REVERSE_BUTTON, HIGH);
  digitalWrite(GRAIN_BUTTON,   HIGH);

  AudioHacker.begin();

  for (byte i = 0; i < 6; i++) {
    potSmooth[i] = analogRead(i);
  }
  potsInitialized = true;

#ifdef DEBUG
  Serial.print("sample rate = ");        Serial.print(passthroughSampleRate);
  Serial.print(" Hz, recording rate = "); Serial.print(recordingSampleRate);
  Serial.println(" Hz");
#endif

  for (byte i = 0; i < 4; i++) sampleRecorded[i] = false;

  if (batteryBackup) {
    for (byte i = 0; i < 4; i++) {
      byte a = i * 3;
      long b;
      b = (long)EEPROM.read(a);     endAddress[i]  = (b << 16);
      b = (long)EEPROM.read(a + 1); endAddress[i] |= (b << 8);
      b = (long)EEPROM.read(a + 2); endAddress[i] |= b;
      if (endAddress[i] > 0) {
        sampleRecorded[i] = true;
#ifdef DEBUG
        Serial.print("sample "); Serial.print(i);
        Serial.print(" endAddress = "); Serial.println(endAddress[i]);
#endif
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
void loop() {

  // ── Pot reads (rate-limited) ──────────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastPotRead >= POT_READ_INTERVAL_MS) {
    lastPotRead = now;
    for (byte i = 0; i < 6; i++) {
      smoothPotValue(i, analogRead(i));
    }
    potsInitialized = true;

    // ── Sample rate ─────────────────────────────────────────────────────────
    recordingSampleRate  = map(potSmooth[0], 0, 1023, 1000, 16000);
    recordingSampleRate  = recordingSampleRate - (recordingSampleRate % 100);
    passthroughSampleRate = map(potSmooth[0], 0, 1023, 1000, 16000);
    passthroughSampleRate = passthroughSampleRate - (passthroughSampleRate % 100);

    // ── Effect pots ─────────────────────────────────────────────────────────
    resolution = map(potSmooth[1], 0, 1023, 1, 12);
    mask = (uint16_t)(~((1 << (12 - resolution)) - 1));

    LPfilterCutoff = potSmooth[2] >> 2;
    HPfilterCutoff = potSmooth[3] >> 2;
    LPfeedback = (long)LPfilterResonance +
                 (long)(((long)LPfilterResonance * ((int)255 - (255 - LPfilterCutoff))) >> 8);
    HPfeedback = (long)HPfilterResonance +
                 (long)(((long)HPfilterResonance * ((int)255 - (255 - HPfilterCutoff))) >> 8);

    if (mode == PASSTHROUGH) {
      timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
    }
  }

  // ── Button reads ─────────────────────────────────────────────────────────────
  for (int i = 0; i < 7; i++) reading[i] = digitalRead(pins[i]);

  // Sample + record buttons (j = 0..4)
  for (int j = 0; j < 5; j++) {
    if (reading[j] != lastButtonState[j]) lastDebounceTime[j] = millis();
    if ((millis() - lastDebounceTime[j]) > debounceDelay) {
      if (reading[j] != buttonState[j]) {
        buttonState[j] = reading[j];
        if (j == 4) freezeAddressReset = address;
      }
    }
    if (j == 0) sample0Button = buttonState[j];
    if (j == 1) sample1Button = buttonState[j];
    if (j == 2) sample2Button = buttonState[j];
    if (j == 3) sample3Button = buttonState[j];
    if (j == 4) recordButton  = buttonState[j];
    lastButtonState[j] = reading[j];
  }

  // ── Filter combo (record + reverse held together) ────────────────────────────
  if (reading[4] != lastButtonState[5]) timeOfLastButtonChange[4] = millis();
  timeSinceLastButtonChange[4] = millis() - timeOfLastButtonChange[4];
  if (reading[5] != lastButtonState[5]) timeOfLastButtonChange[5] = millis();
  timeSinceLastButtonChange[5] = millis() - timeOfLastButtonChange[5];

  if ((reading[4] == BUTTON_ON && reading[5] == BUTTON_ON) &&
      (timeSinceLastButtonChange[4] < debounceDelay) &&
      (timeSinceLastButtonChange[5] < debounceDelay)) {
    // debounce — do nothing
  }
  if ((reading[4] == BUTTON_ON && timeSinceLastButtonChange[4] > gateThreshold) &&
      (reading[5] == BUTTON_ON && timeSinceLastButtonChange[5] > gateThreshold) &&
      filterState != LONG_PRESSED) {
    filterState  = LONG_PRESSED;
    filterButton = BUTTON_ON;
  }
  if ((reading[4] == BUTTON_ON && timeSinceLastButtonChange[4] > debounceDelay &&
       timeSinceLastButtonChange[4] < gateThreshold) &&
      (reading[5] == BUTTON_ON && timeSinceLastButtonChange[5] > debounceDelay &&
       timeSinceLastButtonChange[5] < gateThreshold) &&
      filterState != HOLDING_PRESS) {
    filterState  = HOLDING_PRESS;
    filterButton = BUTTON_ON;
  }
  if ((reading[4] == BUTTON_OFF && timeSinceLastButtonChange[4] > debounceDelay) &&
      (reading[5] == BUTTON_OFF && timeSinceLastButtonChange[5] > debounceDelay) &&
      filterState == HOLDING_PRESS) {
    filterState = SHORT_PRESSED;
    if (currentShortFilterState == BUTTON_OFF && lastShortFilterState == BUTTON_ON) {
      filterButton = (filterButton == BUTTON_OFF) ? BUTTON_ON : BUTTON_OFF;
    }
    lastShortFilterState = filterButton;
  }
  if ((reading[4] == BUTTON_OFF && timeSinceLastButtonChange[4] > debounceDelay) &&
      (reading[5] == BUTTON_OFF && timeSinceLastButtonChange[5] > debounceDelay) &&
      filterState == LONG_PRESSED) {
    filterState  = LONG_RELEASE;
    filterButton = BUTTON_OFF;
  }
  lastButtonState[4] = reading[4];
  lastButtonState[5] = reading[5];

  // ── Reverse & grain buttons (k = 5, 6) ──────────────────────────────────────
  for (int k = 5; k < 7; k++) {
    if (reading[k] != lastButtonState[k]) timeOfLastButtonChange[k] = millis();
    timeSinceLastButtonChange[k] = millis() - timeOfLastButtonChange[k];

    if (reading[k] == BUTTON_ON && timeSinceLastButtonChange[k] < debounceDelay) {
      // debounce — do nothing
    }
    if (reading[k] == BUTTON_ON && reading[4] == BUTTON_OFF &&
        timeSinceLastButtonChange[k] > gateThreshold && buttonState[k] != LONG_PRESSED) {
      buttonState[k] = LONG_PRESSED;
      if (k == 5) reverseButton = BUTTON_ON;
      if (k == 6) grainButton   = BUTTON_ON;
    }
    if (reading[k] == BUTTON_ON && reading[4] == BUTTON_OFF &&
        timeSinceLastButtonChange[k] > debounceDelay &&
        timeSinceLastButtonChange[k] < gateThreshold &&
        buttonState[k] != HOLDING_PRESS) {
      buttonState[k] = HOLDING_PRESS;
      if (k == 5) reverseButton = BUTTON_ON;
      if (k == 6) grainButton   = BUTTON_ON;
    }
    if (reading[k] == BUTTON_OFF && timeSinceLastButtonChange[k] > debounceDelay &&
        buttonState[k] == HOLDING_PRESS) {
      buttonState[k] = SHORT_PRESSED;
      if (currentShortPressState[k] == BUTTON_OFF && lastShortPressState[k] == BUTTON_ON) {
        if (k == 5) reverseButton = (reverseButton == BUTTON_OFF) ? BUTTON_ON : BUTTON_OFF;
        if (k == 6) grainButton   = (grainButton   == BUTTON_OFF) ? BUTTON_ON : BUTTON_OFF;
      }
      if (k == 5) lastShortPressState[k] = reverseButton;
      if (k == 6) lastShortPressState[k] = grainButton;
    }
    if (reading[k] == BUTTON_OFF && timeSinceLastButtonChange[k] > debounceDelay &&
        buttonState[k] == LONG_PRESSED) {
      buttonState[k] = LONG_RELEASE;
      if (k == 5) reverseButton = BUTTON_OFF;
      if (k == 6) grainButton   = BUTTON_OFF;
    }
    lastButtonState[k] = reading[k];
  }


#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) lastDebugPrint = millis();
#endif

  // ── Playback direction (safe — only used for initial address in loop()) ──────
  if (reverseButton == BUTTON_ON) {
    playbackDirection = -3;
    playbackDirectionStart[0] = endAddress[0] - 3;
    playbackDirectionStart[1] = endAddress[1] - 3;
    playbackDirectionStart[2] = endAddress[2] - 3;
    playbackDirectionStart[3] = endAddress[3] - 3;
  } else {
    playbackDirection = 3;
    playbackDirectionStart[0] = 0;
    playbackDirectionStart[1] = 65535;
    playbackDirectionStart[2] = 0;
    playbackDirectionStart[3] = 65535;
  }

  // ── Mode: OFF / PASSTHROUGH → RECORD or PLAYBACK ────────────────────────────
  if ((mode == OFF) || (mode == PASSTHROUGH)) {
    if (recordButton == BUTTON_ON &&
        (sample0Button == BUTTON_ON || sample1Button == BUTTON_ON ||
         sample2Button == BUTTON_ON || sample3Button == BUTTON_ON)) {
      recordStartTime = millis();
      if (((recordStartTime - recordEndTime) < 20) || (playbackBuf < 2100)) {
        recordStartTime = 0;
        return;
      }
      if (sample0Button == BUTTON_ON) { sample = 0; address = 0;     addressChipNumber = 0; }
      if (sample1Button == BUTTON_ON) { sample = 1; address = 65535; addressChipNumber = 0; }
      if (sample2Button == BUTTON_ON) { sample = 2; address = 0;     addressChipNumber = 1; }
      if (sample3Button == BUTTON_ON) { sample = 3; address = 65535; addressChipNumber = 1; }
      mode = RECORD;
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
      currentA0Position = potSmooth[0];
    } else {
      // PLAYBACK entry
      if ((sample0Button == BUTTON_ON) && sampleRecorded[0]) {
        sample = 0; addressChipNumber = 0;
        address = playbackDirectionStart[0];
        grainAddress = playbackDirectionStart[0]; grainAddressChipNumber = 0;
        nSamplesPlayed = 0; grainBoundaryPending = false; lastGrainSignal = 0;
        mode = PLAYBACK;
      }
      if ((sample1Button == BUTTON_ON) && sampleRecorded[1]) {
        sample = 1; addressChipNumber = 0;
        address = playbackDirectionStart[1];
        grainAddress = playbackDirectionStart[1]; grainAddressChipNumber = 0;
        nSamplesPlayed = 0; grainBoundaryPending = false; lastGrainSignal = 0;
        mode = PLAYBACK;
      }
      if ((sample2Button == BUTTON_ON) && sampleRecorded[2]) {
        sample = 2; addressChipNumber = 1;
        address = playbackDirectionStart[2];
        grainAddress = playbackDirectionStart[2]; grainAddressChipNumber = 1;
        nSamplesPlayed = 0; grainBoundaryPending = false; lastGrainSignal = 0;
        mode = PLAYBACK;
      }
      if ((sample3Button == BUTTON_ON) && sampleRecorded[3]) {
        sample = 3; addressChipNumber = 1;
        address = playbackDirectionStart[3];
        grainAddress = playbackDirectionStart[3]; grainAddressChipNumber = 1;
        nSamplesPlayed = 0; grainBoundaryPending = false; lastGrainSignal = 0;
        mode = PLAYBACK;
      }
    }
  }

  // ── RECORD mode: stop on button release ─────────────────────────────────────
  if (mode == RECORD) {
    if (((sample == 0) && (sample0Button == BUTTON_OFF)) ||
        ((sample == 1) && (sample1Button == BUTTON_OFF)) ||
        ((sample == 2) && (sample2Button == BUTTON_OFF)) ||
        ((sample == 3) && (sample3Button == BUTTON_OFF))) {
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) return;
      sampleRecorded[sample] = true;
      endAddress[sample] = address;
#ifdef DEBUG
      Serial.print("sample "); Serial.print(sample);
      Serial.print(" time = "); Serial.print(recordEndTime - recordStartTime);
      Serial.print(" ms  endAddress = "); Serial.println(endAddress[sample]);
#endif
      if (batteryBackup) {
        byte a = sample * 3;
        EEPROM.write(a,     (endAddress[sample] >> 16) & 0xFF);
        EEPROM.write(a + 1, (endAddress[sample] >> 8)  & 0xFF);
        EEPROM.write(a + 2,  endAddress[sample]        & 0xFF);
      }
      mode = PASSTHROUGH;
    }
  }

  // ── RECORD_DONE (memory full) ────────────────────────────────────────────────
  if (mode == RECORD_DONE) {
    if (recordStartTime != 0) {
#ifdef DEBUG
      Serial.print("sample "); Serial.print(sample);
      Serial.print(" time = "); Serial.print(millis() - recordStartTime);
      Serial.print(" ms  endAddress = "); Serial.println(endAddress[sample]);
#endif
      sampleRecorded[sample] = true;
      recordStartTime = 0;
      if (batteryBackup) {
        byte a = sample * 3;
        EEPROM.write(a,     (endAddress[sample] >> 16) & 0xFF);
        EEPROM.write(a + 1, (endAddress[sample] >> 8)  & 0xFF);
        EEPROM.write(a + 2,  endAddress[sample]        & 0xFF);
      }
    }
    if (recordButton == BUTTON_OFF) mode = PASSTHROUGH;
  }

  // ── PLAYBACK: freeze / grain pots + stop condition ──────────────────────────
  if (mode == PLAYBACK) {
    freezeButton = (recordButton == BUTTON_ON && reading[5] != BUTTON_ON)
                   ? BUTTON_ON : BUTTON_OFF;

    if (freezeButton == BUTTON_ON && lastFreezeButton == BUTTON_OFF) {
      long lowerBound = (sample == 0 || sample == 2) ? 0L : 65535L;
      long upperBound = endAddress[sample] - 3;

      if (reverseButton == BUTTON_ON) {
        freezeAddressReset = address - (long)grainSize + 3;
      } else {
        freezeAddressReset = address;
      }

      if (freezeAddressReset < lowerBound) freezeAddressReset = lowerBound;
      if (freezeAddressReset > upperBound) freezeAddressReset = upperBound;

      grainAddress = freezeAddressReset;
      nSamplesPlayed = 0;
      grainBoundaryPending = false;
      lastGrainSignal = 0;
    }

    if (((sample == 0) && (sample0Button == BUTTON_OFF)) ||
        ((sample == 1) && (sample1Button == BUTTON_OFF)) ||
        ((sample == 2) && (sample2Button == BUTTON_OFF)) ||
        ((sample == 3) && (sample3Button == BUTTON_OFF))) {
      mode = PASSTHROUGH;
    } else {
      // Grain/playback pots share the same rate-limit window as the other
      // pots above — only read when the interval has elapsed.
      // Playback controls reuse the smoothed pot values updated above.
      grainRead = map(potSmooth[4], 0, 1023, 3, 682);
      grainSize = grainRead * 3;
      stretch   = map(potSmooth[5], 0, 1023, 1, 20);
      playbackSampleRate = map(potSmooth[0], 0, 1023, 1000, 16000);
      nSamplesToPlay = stretch * grainSize *
                       ((float)playbackSampleRate / (float)recordingSampleRate);
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);

    }
  }
  lastFreezeButton = freezeButton;

  if (mode != previousMode) {
    resetToneState();
    inputEnvelope = 0;
    inputGateHold = 0;
    inputGateOpen = false;
    previousMode = mode;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ISR
// ═══════════════════════════════════════════════════════════════════════════════
static inline int gateLiveInput(int centeredSample) {
  int magnitude = centeredSample;
  if (magnitude < 0) magnitude = -magnitude;

  inputEnvelope += (magnitude - inputEnvelope) >> 3;

  if (inputGateOpen) {
    if (inputEnvelope < INPUT_GATE_CLOSE_THRESHOLD) {
      if (inputGateHold < INPUT_GATE_HOLD_SAMPLES) inputGateHold++;
      if (inputGateHold >= INPUT_GATE_HOLD_SAMPLES) inputGateOpen = false;
    } else {
      inputGateHold = 0;
    }
  } else if (inputEnvelope > INPUT_GATE_OPEN_THRESHOLD) {
    inputGateOpen = true;
    inputGateHold = 0;
  }

  return inputGateOpen ? centeredSample : 0;
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  int mix;
  unsigned int signal;

  if (mode != RECORD_DONE) AudioHacker.writeDAC(playbackBuf);

  if ((mode != PLAYBACK) && (mode != RECORD_DONE)) {
    signal = AudioHacker.readADC();
    mix    = signal - 2048;
    if (mix > -NOISE_GATE_THRESHOLD && mix < NOISE_GATE_THRESHOLD) mix = 0;
    mix = gateLiveInput(mix);
  }

  // ── RECORD ──────────────────────────────────────────────────────────────────
  if (mode == RECORD) {
    if (evenCycle) {
      writeBuf = signal;
    } else {
      AudioHacker.writeSRAMPacked(addressChipNumber, address, writeBuf, signal);
      address += 3;
      if (((sample == 0) && (address > 65532)) ||
          ((sample == 1) && (address > MAX_ADDR)) ||
          ((sample == 2) && (address > 65532)) ||
          ((sample == 3) && (address > MAX_ADDR))) {
        mode = RECORD_DONE;
        endAddress[sample] = address;
      }
    }
  }

  // ── PLAYBACK ─────────────────────────────────────────────────────────────────
  if (mode == PLAYBACK) {

    if (evenCycle) {
      unsigned int passbuff = AudioHacker.readADC();
      AudioHacker.readSRAMPacked(addressChipNumber, address, readBuf);

      // Gate the live passthrough component so ADC floor noise doesn't
      // add to the SRAM signal when no live input is present.
      int passcentered = (int)passbuff - 2048;
      if (passcentered > -NOISE_GATE_THRESHOLD && passcentered < NOISE_GATE_THRESHOLD)
        passcentered = 0;
      passcentered = gateLiveInput(passcentered);

      signal = (reverseButton == BUTTON_ON) ? readBuf[1] : readBuf[0];
      mix    = passcentered + (signal - 2048);

      // ── Grain delay ──────────────────────────────────────────────────────────
      if (grainButton == BUTTON_ON) {
        nSamplesPlayed++;

        if (nSamplesPlayed >= nSamplesToPlay / 3) {
          nSamplesPlayed = 0;
          grainBoundaryPending = true;
        }

        // Intra-grain loop
        if (reverseButton == BUTTON_ON) {
          if (address <= grainAddress) address = grainAddress + grainSize - 3;
        } else {
          if (address >= grainAddress + grainSize) address = grainAddress;
        }

        // Jump to the next grain only at a zero crossing so the transition
        // is inaudible.
        int signedSignal = (int)signal - 2048;
        boolean atZeroCrossing = (signedSignal == 0) ||
                                 ((signedSignal >= 0) != (lastGrainSignal >= 0));
        lastGrainSignal = signedSignal;

        if (grainBoundaryPending && atZeroCrossing) {
          grainBoundaryPending = false;
          if (reverseButton == BUTTON_ON) {
            grainAddress -= grainSize;
            long lowerBound = (sample == 0 || sample == 2) ? 0L : 65535L;
            if (grainAddress <= lowerBound) grainAddress = endAddress[sample] - grainSize;
          } else {
            grainAddress += grainSize;
            long startAddr = (sample == 0 || sample == 2) ? 0L : 65535L;
            if (grainAddress >= endAddress[sample]) grainAddress = startAddr;
          }
          address = grainAddress;
        }
      }

      // ── Freeze mode ──────────────────────────────────────────────────────────
      if (freezeButton == BUTTON_ON) {
        // Freeze captures one window when engaged, then loops that same
        // section until the button is released.
        grainAddress = freezeAddressReset;

        if (reverseButton == BUTTON_ON) {
          if (address <= grainAddress) address = grainAddress + grainSize - 3;
        } else {
          if (address >= grainAddress + grainSize) address = grainAddress;
        }
      }

      // Clamp
      if (mix < -2048)     mix = -2048;
      else if (mix > 2047) mix =  2047;

      // ── Address advance + wrap ────────────────────────────────────────────────
      address += playbackDirection;

      if (reverseButton == BUTTON_ON) {
        long lowerBound = (sample == 0 || sample == 2) ? 0L : 65535L;
        long startAddr  = endAddress[sample] - 3;
        if (address <= lowerBound) {
          address = startAddr;
          // Restore chip number
          addressChipNumber = (sample == 0 || sample == 1) ? 0 : 1;
        }
      } else {
        long startAddr = (sample == 0 || sample == 2) ? 0L : 65535L;
        if (address >= endAddress[sample]) {
          address = startAddr;
          addressChipNumber = (sample == 0 || sample == 1) ? 0 : 1;
        }
      }

    } else {
      // ── Odd cycle: second packed sample ──────────────────────────────────────
      unsigned int passbuff = AudioHacker.readADC();
      int passcentered = (int)passbuff - 2048;
      if (passcentered > -NOISE_GATE_THRESHOLD && passcentered < NOISE_GATE_THRESHOLD)
        passcentered = 0;
      passcentered = gateLiveInput(passcentered);
      signal = (reverseButton == BUTTON_ON) ? readBuf[0] : readBuf[1];
      mix    = passcentered + (signal - 2048);

      if (mix < -2048)     mix = -2048;
      else if (mix > 2047) mix =  2047;
    }
  } // PLAYBACK

  // ── Filter ───────────────────────────────────────────────────────────────────
  if (!inputGateOpen && mode != PLAYBACK && mix == 0) {
    LPbuf0 = 0;
    LPbuf1 = 0;
    HPbuf0 = 0;
    HPbuf1 = 0;
  }

  if (filterButton == BUTTON_ON) {
    int highPass1 = mix - LPbuf0;
    int bandPass1 = LPbuf0 - LPbuf1;
    int tmp1 = highPass1 + (LPfeedback * bandPass1 >> 8);
    LPbuf0 += ((long)LPfilterCutoff * tmp1)          >> 8;
    LPbuf1 += ((long)LPfilterCutoff * (LPbuf0 - LPbuf1)) >> 8;
    mix = LPbuf1;

    int highPass2 = mix - HPbuf0;
    int bandPass2 = HPbuf0 - HPbuf1;
    int tmp2 = highPass2 + (HPfeedback * bandPass2 >> 8);
    HPbuf0 += ((long)HPfilterCutoff * tmp2)          >> 8;
    HPbuf1 += ((long)HPfilterCutoff * (HPbuf0 - HPbuf1)) >> 8;
    mix = highPass2 + 2048;
  } else {
    mix = mix + 2048;
  }

  mix = applyDcBlock(mix - 2048) + 2048;

  if (mix < 0)         mix = 0;
  else if (mix > 4095) mix = 4095;

  playbackBuf = (unsigned int)mix;

  playbackBuf &= mask;

#ifdef DEBUG
  if (evenCycle) timer1EndEven = TCNT1;
  else           timer1EndOdd  = TCNT1;
#endif

  evenCycle = !evenCycle;
}
