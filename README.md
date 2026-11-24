# Mini Robotic Arm

A four-joint desk robotic arm driven by an Arduino Uno. Four knobs move the base, shoulder, wrist and gripper, and the arm can **record a sequence of poses and replay it in a loop** with smooth, coordinated movement.

I printed and assembled the arm from an open mini-arm design, wired the electronics, and wrote the control and record-and-replay firmware in this repository.

**Project page:** open `index.html` (served by GitHub Pages) for the demo video, photos and a write-up.

---

## Contents

1. [Features](#features)
2. [How it works](#how-it-works)
3. [Hardware](#hardware)
4. [Wiring](#wiring)
5. [Powering the servos](#powering-the-servos)
6. [Software setup](#software-setup)
7. [Using the arm](#using-the-arm)
8. [Calibrating each joint](#calibrating-each-joint)
9. [Configuration reference](#configuration-reference)
10. [Serial output](#serial-output)
11. [Troubleshooting](#troubleshooting)
12. [Limitations and future work](#limitations-and-future-work)


---

## Features

- **Direct knob control.** Each of four potentiometers drives one servo: base rotation, shoulder, wrist and gripper.
- **Record and replay.** Single-tap a button to store the current pose. Double-tap to replay all stored poses in a loop.
- **Smooth, coordinated motion.** Every joint starts and stops together, with an ease-in and ease-out profile, so the arm moves like one machine and not four separate servos.
- **Adjustable loop delay.** While replaying, the gripper knob sets the wait between loops: 0, 1, 3, 15, 60 or 300 seconds.
- **Pause switch.** Freezes the arm mid-motion and resumes exactly where it left off.
- **Per-joint safety limits.** Each joint has a calibrated pulse-width range, so a knob cannot drive a servo past its end stop.
- **Gentle start-up.** Servos are enabled one at a time to avoid a large current spike at power-on.
- **Non-blocking firmware.** No long `delay()` calls in the main loop, so buttons and the pause switch stay responsive during playback and loop waits.

---

## How it works

### Control loop

The sketch has three modes.

| Mode | What happens |
|---|---|
| `MANUAL` | Every 25 ms (40 Hz) the four knobs are read, converted to servo pulse widths and written to the servos. |
| `PLAY_MOVE` | The arm moves from its current pose to the next recorded pose, with all joints arriving at the same moment. |
| `PLAY_WAIT` | After the last pose, the arm waits (set by the gripper knob), then starts the loop again. |

### Knob to servo mapping

Each knob is read with the Uno's 10-bit ADC (values 0 to 1023). A per-joint calibration table maps that range to a pulse width in microseconds, and the result is clamped to the joint's allowed range:

```
pulse_us = map(analogRead(knob), rawA, rawB, usA, usB)   // then constrained to [min(usA,usB), max(usA,usB)]
```

Reversing a joint's direction only needs the two raw values swapped, so no wiring change is required.

### Recording and playback

- A **single tap** (debounced at 20 ms, confirmed after a 300 ms double-tap window) stores the current pose of all four joints.
- A **double tap** starts playback from the first stored pose. Tapping once during playback stops it and returns to manual control.
- Poses are held in RAM: **150 poses on an Uno or Nano** (2 KB of RAM) and 400 on boards with more memory.

### Motion profile

For each move, the joint with the largest travel sets the duration:

- Travel time is 1200 µs per µs of pulse change for small moves, or 600 µs per µs when the largest move is 500 µs or more.
- Every joint's position is interpolated with a smoothstep curve, `s = t²(3 − 2t)`, so speed ramps up and down.
- Because all joints share the same time variable `t`, they reach the target pose together.

### Pause switch

When the pause pin is pulled low, the arm freezes and the on-board LED blinks. On release, the playback timers are shifted by the time spent paused, so the motion continues instead of jumping ahead.

---

## Hardware

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino Uno R3 (or Nano) | ATmega328P, 5 V logic |
| 4 | SG90 micro servos | Base, shoulder, wrist, gripper. Rated 4.8 to 6 V |
| 4 | 10 kΩ linear potentiometers | One per joint |
| 2 | Momentary push buttons | Record/play, pause |
| 1 | Breadboard and jumper wires | |
| 1 | 5 V supply for the servos, 2 A or more | A 4 x AA pack (about 6 V) also works |
| 1 | Electrolytic capacitor, 470 µF or more (optional) | Across the servo power rail |
| | 3D-printed arm parts | See [Attribution](#attribution) |

---

## Wiring

### Pin map

| Function | Uno pin |
|---|---|
| Base servo signal | 11 |
| Shoulder servo signal | 12 |
| Wrist servo signal | 5 |
| Gripper servo signal | 3 |
| Base knob (middle pin) | A0 |
| Shoulder knob (middle pin) | A1 |
| Wrist knob (middle pin) | A2 |
| Gripper knob (middle pin) | A3 |
| Record/play button | 6 (other side to GND) |
| Pause switch | 4 (other side to GND) |
| Status LED | 13 (on-board LED) |

Pins 4 and 6 use the Arduino's **internal pull-ups**, so no external resistors are needed.

### Servo wires (SG90)

| Wire | Function | Connect to |
|---|---|---|
| Orange | Signal | The Arduino pin in the table above |
| Red | +5 V | Servo power rail |
| Brown | GND | GND rail |

### Potentiometer pins

A potentiometer has three pins. One outer pin goes to 5 V, the other outer pin to GND, and the **middle pin to the analog input**. Swapping the outer two only reverses the knob's direction.

---

## Powering the servos

Servos draw short bursts of current, and four of them can exceed what a USB port supplies. Symptoms of too little power are the Uno resetting, servos twitching, or the serial monitor printing the start-up message again.

Recommended setup:

1. Power the **Uno from USB**.
2. Power the **servos from a separate 5 V supply** (2 A or more) or a 4 x AA pack.
3. **Join the grounds.** The supply's GND must connect to the Uno's GND.
4. Add a capacitor of 470 µF or more across the servo rail, close to the servos. The stripe marks the negative leg and goes to GND.

Do not connect a 9 V or 12 V source to the servos. SG90s are rated 4.8 to 6 V.

For a short bench test, USB power alone can work: the sketch starts the servos one at a time to reduce the start-up spike.

---

## Software setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Open `Mini_Arm_Record_Replay.ino`.
3. Select **Tools, Board, Arduino Uno** (or Nano) and the correct port.
4. Upload. The `Servo` library is included with the IDE, so nothing else needs installing.
5. Open the Serial Monitor at **115200 baud**. You should see `mini robot ready...`.

---

## Using the arm

| Action | Result |
|---|---|
| Turn a knob | The matching joint moves. |
| Single tap the record button | Stores the current pose (`Record Step: n`). |
| Double tap the record button | Starts looping playback of all stored poses. |
| Single tap during playback | Stops playback and returns to manual control. |
| Gripper knob during playback | Sets the wait between loops. |
| Hold the pause switch low | Freezes everything and blinks the LED. |

Tips:

- Put the knobs near the arm's current pose before leaving playback. The servos move straight to the knob positions when manual mode resumes.
- Record a pose, move the arm, record again. Two or three poses already make a clear loop.
- Stored poses live in RAM and are lost on reset or power loss.

---

## Calibrating each joint

The default pulse ranges are wide. Small servos like the SG90 only travel safely between about 600 and 2300 µs, and some stop earlier. Past the end stop a servo **hums, gets hot and can strip its gears**.

To find a safe range for one joint:

1. Disconnect the other servos and keep only that joint's servo on its pin.
2. Upload a simple sweep sketch that moves the servo slowly between 1000 and 2000 µs, so it cannot reach the end stops.
3. Widen the range in steps of 100 µs. Stop widening as soon as you hear any buzzing, and back off by 100.
4. Put the two safe numbers into that joint's row of the calibration table in `Mini_Arm_Record_Replay.ino`:

```cpp
const Calib CAL[NUM_JOINTS] = {
  //  rawA  rawB   usA   usB
  {  150,  900,  600, 2400 },   // base rotation
  { 1000,  100,  550, 2400 },   // shoulder
  {  120,  860,  400, 2500 },   // wrist
  { 1023,    0,  500, 2500 },   // gripper
};
```

The last two numbers in each row are the pulse range for that joint. If a joint buzzes at an end, bring its range in toward 1500 µs. For example, `700, 2200` for the wrist and gripper is a more conservative starting point for SG90 servos.

Also check that the mechanical arm itself does not touch the base or table at the ends of the range, because a blocked servo draws a lot of current even without reaching its internal stop.

---

## Configuration reference

All settings are constants near the top of the sketch.

| Constant | Default | Meaning |
|---|---|---|
| `SERVO_PIN[]` | 11, 12, 5, 3 | Servo signal pins: base, shoulder, wrist, gripper |
| `POT_PIN[]` | A0 to A3 | Knob inputs, in the same order |
| `PIN_BUTTON` | 6 | Record and replay button |
| `PIN_PAUSE` | 4 | Pause switch |
| `MAX_STEPS` | 150 (Uno/Nano), 400 (others) | Maximum stored poses |
| `MANUAL_PERIOD_MS` | 25 | Knob read and servo update interval |
| `DEBOUNCE_MS` | 20 | Button debounce time |
| `DOUBLE_TAP_MS` | 300 | Window for a second tap |
| `US_PER_STEP_SMALL` | 1200 | Playback time per µs of travel, for moves under 500 µs |
| `US_PER_STEP_LARGE` | 600 | Playback time per µs of travel, for larger moves |
| `LOOP_PAUSE_S[]` | 0, 0, 1, 3, 15, 60, 300 | Loop wait choices, in seconds, selected by the gripper knob |

To slow the playback, raise the two `US_PER_STEP` values. To speed it up, lower them.

---

## Serial output

At 115200 baud the sketch prints:

| Message | When |
|---|---|
| `mini robot ready...` | After start-up. If this prints again unexpectedly, the board reset. |
| `Record Step: n` | A pose was stored. |
| `Memory full` | The pose buffer is full. |
| `Nothing recorded` | A double tap with no stored poses. |
| `playmode` | Playback started, followed by the stored values for each joint. |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Servo hums or buzzes | Pulse range too wide, or the arm is blocked | Narrow that joint's range in `CAL`. Check nothing is touching. |
| Servo jumps around and the knob does nothing | The knob's middle pin is not reaching the analog pin, so the input floats | Check the middle-pin wire. Push the pins firmly into the breadboard. |
| One knob works, then stops | A loose breadboard contact | Reseat the knob's three wires. |
| Arm freezes and the LED blinks slowly | The pause pin reads low | Remove anything connected to pin 4, or release the switch. |
| Arm keeps returning to one pose, or `mini robot ready...` prints again | Power drop is resetting the board | Use a separate 5 V supply, add the capacitor, and join the grounds. |
| A joint moves the wrong way | Knob wired in the opposite direction | Swap that joint's two raw values in `CAL`, or swap the knob's outer wires. |
| A knob moves the wrong joint | Pins in a different order | Swap entries in `SERVO_PIN[]` or `POT_PIN[]`. |
| Recording or replay does nothing | Button not wired | Touch a jumper from pin 6 to GND: once to record, twice quickly to replay. |

Useful checks:

- **Read the knobs directly.** A four-line sketch that prints `analogRead(A0)` to `A3` shows whether each knob sweeps smoothly from near 0 to near 1023.
- **Replace a knob with a wire.** Touch a jumper from 5V to an analog pin, then from GND to the same pin. If the servo swings to opposite ends, the servo and code are fine and the knob wiring is the problem.
- **Test one servo at a time.** It isolates power problems from wiring problems.

---

## Repository contents

```
index.html                       Project page with demo video, photos and write-up
Mini_Arm_Record_Replay.ino       Arduino firmware
README.md                        This file
```

---

## Limitations and future work

- Recorded poses are stored in RAM only and are lost when the board resets. Saving them to the Uno's EEPROM would make them survive a power cycle.
- SG90 servos have limited torque and plastic gears, so the arm is suited to light objects.
- The servos have no position feedback, so the firmware assumes each servo reaches its commanded position.
- Possible upgrades: metal-gear servos, a dedicated servo power board, an LCD or serial display for the recorded steps, and a speed knob for playback.

---

## Attribution

The mechanical design is an open mini robotic arm by its original designer. I printed and assembled my own copy with modifications and files are attaatched. 

The wiring, calibration, and firmware in this repository were written for my build, with AI assistance.

---

## Author

Zohaib Hassan, Mechatronics Engineering, University of Guelph.
