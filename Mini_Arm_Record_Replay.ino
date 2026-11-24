// Mini robotic arm: manual control, step recording and looping playback
// Board: Arduino Uno / Nano / Mega     Library: Servo
//
// Hardware
//   4 potentiometers (A0..A3) -> 4 servos (base, shoulder, wrist, gripper)
//   Record/play button on pin 6 (to GND), pause switch on pin 4 (to GND), status LED on pin 13
//
// Controls
//   Manual mode : each knob drives its servo directly.
//   Single tap  : record the current arm pose as the next step (also stops playback).
//   Double tap  : play the recorded steps in a loop.
//   Pause switch (pin 4 LOW): freeze everything, LED blinks. Release to continue.
//   In playback the gripper knob (A3) sets the wait between loops: 0, 0, 1, 3, 15, 60, 300 s.

#include <Servo.h>

// ---------- Pins ----------
constexpr uint8_t NUM_JOINTS = 4;
enum Joint : uint8_t { BASE, SHOULDER, WRIST, GRIPPER };

const uint8_t SERVO_PIN[NUM_JOINTS] = { 11, 12, 5, 3 };
const uint8_t POT_PIN[NUM_JOINTS]   = { A0, A1, A2, A3 };

constexpr uint8_t PIN_BUTTON = 6;    // active LOW
constexpr uint8_t PIN_PAUSE  = 4;    // active LOW
constexpr uint8_t PIN_LED    = 13;

// ---------- Calibration: raw knob range -> servo pulse width (us) ----------
// A reversed range (rawA > rawB) flips the direction of that knob.
struct Calib { int rawA, rawB, usA, usB; };
const Calib CAL[NUM_JOINTS] = {
  {  150,  900,  600, 2400 },   // base rotation
  { 1000,  100,  550, 2400 },   // shoulder
  {  120,  860,  400, 2500 },   // wrist
  { 1023,    0,  500, 2500 },   // gripper
};

// ---------- Settings ----------
#if defined(__AVR_ATmega328P__)
constexpr uint16_t MAX_STEPS = 150;     // Uno / Nano have only 2 KB of RAM
#else
constexpr uint16_t MAX_STEPS = 400;
#endif
constexpr uint16_t MANUAL_PERIOD_MS = 25;
constexpr uint16_t DEBOUNCE_MS      = 20;
constexpr uint16_t DOUBLE_TAP_MS    = 300;
constexpr uint32_t US_PER_STEP_SMALL = 1200;   // playback speed: us of travel time per 1 us of pulse change
constexpr uint32_t US_PER_STEP_LARGE = 600;    // used when the biggest joint move is 500 us or more
const uint16_t LOOP_PAUSE_S[7] = { 0, 0, 1, 3, 15, 60, 300 };

// ---------- State ----------
enum Mode : uint8_t { MANUAL, PLAY_MOVE, PLAY_WAIT };
enum Tap  : uint8_t { TAP_NONE, TAP_SINGLE, TAP_DOUBLE };

Servo servos[NUM_JOINTS];
Mode  mode = MANUAL;

float    pos[NUM_JOINTS];                // current commanded pulse widths (us)
float    fromPos[NUM_JOINTS];
float    toPos[NUM_JOINTS];
uint16_t steps[MAX_STEPS][NUM_JOINTS];   // recorded poses
uint16_t stepCount = 0;
uint16_t nextStep  = 0;

uint32_t moveStartUs = 0;
uint32_t moveDurUs   = 0;
uint32_t lastManualMs = 0;

uint16_t waitSeconds = 0;
uint32_t waitTickMs  = 0;

// ---------- Helpers ----------
int readJoint(uint8_t i) {
  long us = map(analogRead(POT_PIN[i]), CAL[i].rawA, CAL[i].rawB, CAL[i].usA, CAL[i].usB);
  long lo = min(CAL[i].usA, CAL[i].usB);
  long hi = max(CAL[i].usA, CAL[i].usB);
  return (int)constrain(us, lo, hi);       // never drive a servo outside its calibrated range
}

uint16_t readLoopPauseSeconds() {
  int raw = constrain(analogRead(POT_PIN[GRIPPER]), 0, 680);
  return LOOP_PAUSE_S[map(raw, 0, 680, 0, 6)];
}

void writeServos() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) servos[i].writeMicroseconds((int)pos[i]);
}

void readKnobs() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) pos[i] = readJoint(i);
}

void ledPulse(uint16_t ms) {
  digitalWrite(PIN_LED, HIGH);
  delay(ms);
  digitalWrite(PIN_LED, LOW);
}

// ---------- Button: single tap / double tap ----------
Tap pollButton(uint32_t now) {
  static bool lastRaw = HIGH, stable = HIGH;
  static uint32_t lastChange = 0, firstTap = 0;
  static uint8_t taps = 0;

  bool raw = digitalRead(PIN_BUTTON);
  if (raw != lastRaw) { lastRaw = raw; lastChange = now; }

  if (now - lastChange >= DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) {                    // a press
      if (taps == 0) firstTap = now;
      taps++;
    }
  }
  if (taps >= 2) { taps = 0; return TAP_DOUBLE; }
  if (taps == 1 && now - firstTap >= DOUBLE_TAP_MS) { taps = 0; return TAP_SINGLE; }
  return TAP_NONE;
}

// ---------- Pause switch ----------
// Returns true while paused. Playback timers are shifted so a pause does not skip ahead.
bool handlePause() {
  static bool wasPaused = false;
  static uint32_t pauseBeganUs = 0, pauseBeganMs = 0;

  if (digitalRead(PIN_PAUSE) == LOW) {
    if (!wasPaused) { wasPaused = true; pauseBeganUs = micros(); pauseBeganMs = millis(); }
    digitalWrite(PIN_LED, (millis() / 500) % 2);   // slow blink
    return true;
  }
  if (wasPaused) {
    wasPaused = false;
    moveStartUs += micros() - pauseBeganUs;
    waitTickMs  += millis() - pauseBeganMs;
    digitalWrite(PIN_LED, LOW);
  }
  return false;
}

// ---------- Recording ----------
void recordStep() {
  if (stepCount >= MAX_STEPS) { Serial.println(F("Memory full")); return; }
  for (uint8_t i = 0; i < NUM_JOINTS; i++) steps[stepCount][i] = (uint16_t)pos[i];
  stepCount++;
  Serial.print(F("Record Step: ")); Serial.println(stepCount);
  ledPulse(100);
}

void dumpSteps() {
  const char* names[NUM_JOINTS] = { "Base", "Shoulder", "Wrist", "Gripper" };
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    Serial.print(names[j]); Serial.print(F(": "));
    for (uint16_t s = 0; s < stepCount; s++) { Serial.print(steps[s][j]); Serial.print(F(", ")); }
    Serial.println();
  }
}

// ---------- Playback ----------
// All joints arrive together; the joint with the largest travel sets the duration.
void startMove(const uint16_t* target) {
  float maxDiff = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    fromPos[i] = pos[i];
    toPos[i]   = target[i];
    maxDiff = max(maxDiff, fabs(toPos[i] - fromPos[i]));
  }
  moveDurUs   = (uint32_t)maxDiff * (maxDiff < 500 ? US_PER_STEP_SMALL : US_PER_STEP_LARGE);
  moveStartUs = micros();
}

void startPlayback() {
  if (stepCount == 0) { Serial.println(F("Nothing recorded")); return; }
  Serial.println(F("playmode"));
  dumpSteps();
  nextStep = 0;
  mode = PLAY_MOVE;
  startMove(steps[nextStep]);
}

void stopPlayback() {
  mode = MANUAL;
  digitalWrite(PIN_LED, LOW);
}

void beginLoopWait() {
  waitSeconds = 0;
  waitTickMs  = millis();
  if (readLoopPauseSeconds() == 0) {       // no wait: start the next loop immediately
    nextStep = 0;
    mode = PLAY_MOVE;
    startMove(steps[nextStep]);
  } else {
    mode = PLAY_WAIT;
  }
}

void updateMove() {
  float t = 1.0f;
  if (moveDurUs > 0) {
    t = (float)(micros() - moveStartUs) / (float)moveDurUs;
    if (t > 1.0f) t = 1.0f;
  }
  float s = t * t * (3.0f - 2.0f * t);     // ease in and out
  for (uint8_t i = 0; i < NUM_JOINTS; i++) pos[i] = fromPos[i] + (toPos[i] - fromPos[i]) * s;
  writeServos();

  if (t >= 1.0f) {
    nextStep++;
    if (nextStep >= stepCount) beginLoopWait();
    else startMove(steps[nextStep]);
  }
}

void updateWait(uint32_t now) {
  if (now - waitTickMs < 1000) return;
  waitTickMs += 1000;
  waitSeconds++;
  ledPulse(25);
  if (waitSeconds >= readLoopPauseSeconds()) {   // the knob is re-read each second
    nextStep = 0;
    mode = PLAY_MOVE;
    startMove(steps[nextStep]);
  }
}

// ---------- Arduino entry points ----------
void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_PAUSE,  INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  readKnobs();                             // start at the knob positions
  // Start the servos one at a time so they do not all draw their start-up current together
  // (important when powering from a USB port).
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    servos[i].attach(SERVO_PIN[i]);
    servos[i].writeMicroseconds((int)pos[i]);
    delay(300);
  }

  Serial.begin(115200);
  Serial.println(F("mini robot ready..."));
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  if (handlePause()) return;

  uint32_t now = millis();
  Tap tap = pollButton(now);

  if (tap == TAP_DOUBLE) {
    if (mode == MANUAL) startPlayback();
  } else if (tap == TAP_SINGLE) {
    if (mode == MANUAL) recordStep();
    else stopPlayback();
  }

  switch (mode) {
    case MANUAL:
      if (now - lastManualMs >= MANUAL_PERIOD_MS) {
        lastManualMs = now;
        readKnobs();
        writeServos();
      }
      break;
    case PLAY_MOVE: updateMove();    break;
    case PLAY_WAIT: updateWait(now); break;
  }
}
