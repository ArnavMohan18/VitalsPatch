#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// --- Beat detection ---
const byte RATE_SIZE = 8;
byte   rates[RATE_SIZE];
byte   rateSpot    = 0;
long   lastBeat    = 0;
byte   ratesFilled = 0;
float  beatsPerMinute;
int    beatAvg;

// --- SpO2 ---
const byte SPO2_SIZE = 8;
float  spo2Readings[SPO2_SIZE];
byte   spo2Spot   = 0;
byte   spo2Filled = 0;
float  spo2Avg;

// --- Rolling sample buffer for AC/DC SpO2 (150 samples ~= 1.5 beat cycles at 85 BPM) ---
const byte SAMPLE_BUF_SIZE = 150;
long  irBuf[SAMPLE_BUF_SIZE];
long  redBuf[SAMPLE_BUF_SIZE];
byte  bufHead  = 0;
byte  bufCount = 0;

// --- 20-second window ---
const unsigned long WINDOW_MS = 20000UL;
unsigned long windowStart;

int   beatCount;
float bpmWindow[60];
byte  bpmWindowCount;

long  irSum, redSum;
long  irMin, irMax;
long  redMin, redMax;
long  sampleCount;

// -------------------------------------------------------
void resetWindow() {
  windowStart    = millis();
  beatCount      = 0;
  bpmWindowCount = 0;
  irSum  = redSum  = sampleCount = 0;
  irMin  = redMin  = 2000000000L;
  irMax  = redMax  = 0;
}

void printWindowSummary(bool fingerPresent) {
  Serial.println(F("\n===== 20s WINDOW ====="));
  Serial.print(F("Finger:        ")); Serial.println(fingerPresent ? F("YES") : F("NO (IR avg < 50k)"));
  Serial.print(F("Samples:       ")); Serial.println(sampleCount);
  Serial.print(F("Beats found:   ")); Serial.println(beatCount);

  if (bpmWindowCount > 0) {
    // Pass 1: raw stats
    float sum = 0, lo = 999, hi = 0;
    for (byte i = 0; i < bpmWindowCount; i++) {
      sum += bpmWindow[i];
      if (bpmWindow[i] < lo) lo = bpmWindow[i];
      if (bpmWindow[i] > hi) hi = bpmWindow[i];
    }
    float mean = sum / bpmWindowCount;
    float var  = 0;
    for (byte i = 0; i < bpmWindowCount; i++) var += (bpmWindow[i] - mean) * (bpmWindow[i] - mean);
    float stddev = sqrt(var / bpmWindowCount);

    // Pass 2: drop outliers outside mean ± 2σ
    float fSum = 0, fLo = 999, fHi = 0;
    byte  fCount = 0;
    for (byte i = 0; i < bpmWindowCount; i++) {
      if (abs(bpmWindow[i] - mean) <= 2.0f * stddev) {
        fSum += bpmWindow[i];
        if (bpmWindow[i] < fLo) fLo = bpmWindow[i];
        if (bpmWindow[i] > fHi) fHi = bpmWindow[i];
        fCount++;
      }
    }
    float fMean = (fCount > 0) ? fSum / fCount : mean;
    float fVar  = 0;
    for (byte i = 0; i < bpmWindowCount; i++)
      if (abs(bpmWindow[i] - mean) <= 2.0f * stddev)
        fVar += (bpmWindow[i] - fMean) * (bpmWindow[i] - fMean);
    float fStddev = (fCount > 1) ? sqrt(fVar / fCount) : 0;

    Serial.print(F("BPM raw mean:  ")); Serial.println(mean,    1);
    Serial.print(F("BPM raw std:   ")); Serial.println(stddev,  1);
    Serial.print(F("BPM raw range: ")); Serial.print(lo, 1); Serial.print(F(" / ")); Serial.println(hi, 1);
    Serial.println();
    Serial.print(F("BPM filt mean: ")); Serial.println(fMean,   1);  // <-- use this value
    Serial.print(F("BPM filt std:  ")); Serial.println(fStddev, 1);
    Serial.print(F("BPM filt range:")); Serial.print(fLo, 1); Serial.print(F(" / ")); Serial.println(fHi, 1);
    Serial.print(F("Beats used:    ")); Serial.print(fCount); Serial.print(F(" / ")); Serial.println(bpmWindowCount);
    Serial.print(F("BPM roll-avg:  ")); Serial.println(beatAvg);
  } else {
    Serial.println(F("BPM:           no valid beats in window"));
  }

  if (spo2Filled > 0 || spo2Spot > 0) {
    Serial.print(F("SpO2 avg:      ")); Serial.print(spo2Avg, 1); Serial.println(F("%"));
  } else {
    Serial.println(F("SpO2:          insufficient data"));
  }

  if (sampleCount > 0) {
    long irAvg  = irSum  / sampleCount;
    long redAvg = redSum / sampleCount;
    Serial.print(F("IR  avg/min/max: ")); Serial.print(irAvg);  Serial.print(F(" / ")); Serial.print(irMin);  Serial.print(F(" / ")); Serial.println(irMax);
    Serial.print(F("Red avg/min/max: ")); Serial.print(redAvg); Serial.print(F(" / ")); Serial.print(redMin); Serial.print(F(" / ")); Serial.println(redMax);
    Serial.print(F("IR  AC swing:    ")); Serial.println(irMax  - irMin);
    Serial.print(F("Red AC swing:    ")); Serial.println(redMax - redMin);
  }

  Serial.println(F("======================\n"));
}

// Proper AC/DC SpO2: R = (redAC/redDC) / (irAC/irDC), SpO2 = 104 - 17*R
float computeSpO2() {
  if (bufCount < 20) return -1;

  long irBufMin = 2000000000L, irBufMax = 0, irBufSum = 0;
  long redBufMin = 2000000000L, redBufMax = 0, redBufSum = 0;
  byte count = (bufCount < SAMPLE_BUF_SIZE) ? bufCount : SAMPLE_BUF_SIZE;

  for (byte i = 0; i < count; i++) {
    irBufSum  += irBuf[i];  redBufSum  += redBuf[i];
    if (irBuf[i]  < irBufMin)  irBufMin  = irBuf[i];
    if (irBuf[i]  > irBufMax)  irBufMax  = irBuf[i];
    if (redBuf[i] < redBufMin) redBufMin = redBuf[i];
    if (redBuf[i] > redBufMax) redBufMax = redBuf[i];
  }

  float irDC  = (float)irBufSum  / count;
  float redDC = (float)redBufSum / count;
  float irAC  = (float)(irBufMax  - irBufMin);
  float redAC = (float)(redBufMax - redBufMin);

  if (irDC == 0 || irAC == 0) return -1;

  float R    = (redAC / redDC) / (irAC / irDC);
  return constrain(104.0f - (17.0f * R), 80.0f, 100.0f);
}

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing MAX30102..."));

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30102 not found. Check wiring/power."));
    while (1);
  }

  particleSensor.setup(60, 1, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x20);
  particleSensor.setPulseAmplitudeIR(0x20);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println(F("Place finger on sensor. Summary every 20 seconds."));
  resetWindow();
}

// -------------------------------------------------------
void loop() {
  particleSensor.check();

  while (particleSensor.available()) {
    long irValue  = particleSensor.getFIFOIR();
    long redValue = particleSensor.getFIFORed();
    particleSensor.nextSample();

    bool fingerPresent = (irValue > 50000L);

    // Push to rolling buffer for SpO2 AC/DC
    irBuf[bufHead]  = irValue;
    redBuf[bufHead] = redValue;
    bufHead = (bufHead + 1) % SAMPLE_BUF_SIZE;
    if (bufCount < SAMPLE_BUF_SIZE) bufCount++;

    if (irValue > 0) {
      irSum  += irValue;  redSum  += redValue;
      if (irValue  < irMin) irMin = irValue;
      if (irValue  > irMax) irMax = irValue;
      if (redValue < redMin) redMin = redValue;
      if (redValue > redMax) redMax = redValue;
      sampleCount++;
    }

    if (fingerPresent && checkForBeat(irValue)) {
      long delta = millis() - lastBeat;

      if (delta < 500) {
        // Refractory: <500ms (~120 BPM max) — discard double-trigger
      } else {
        lastBeat = millis();
        beatsPerMinute = 60.0f / (delta / 1000.0f);

        if (beatsPerMinute >= 40 && beatsPerMinute <= 120) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          if (ratesFilled < RATE_SIZE) ratesFilled++;

          beatAvg = 0;
          for (byte x = 0; x < ratesFilled; x++) beatAvg += rates[x];
          beatAvg /= ratesFilled;

          beatCount++;
          if (bpmWindowCount < 60) bpmWindow[bpmWindowCount++] = beatsPerMinute;

          float spo2 = computeSpO2();
          if (spo2 > 0) {
            spo2Readings[spo2Spot++] = spo2;
            spo2Spot %= SPO2_SIZE;
            if (spo2Filled < SPO2_SIZE) spo2Filled++;

            spo2Avg = 0;
            for (byte x = 0; x < spo2Filled; x++) spo2Avg += spo2Readings[x];
            spo2Avg /= spo2Filled;
          }
        }
      }
    }
  }

  if (millis() - windowStart >= WINDOW_MS) {
    bool fp = (sampleCount > 0 && (irSum / sampleCount) > 50000L);
    printWindowSummary(fp);
    resetWindow();
  }
}