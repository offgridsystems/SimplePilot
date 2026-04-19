<img width="4032" height="1500" alt="2026-04-17 09 53 09" src="https://github.com/user-attachments/assets/50f60895-67e9-492f-9db7-decf5bd7543a" />

[README.md](https://github.com/user-attachments/files/26862656/README.md)

# SimplePilot

**A small-boat autopilot that steers your outboard's tiller.**

SimplePilot bolts onto your outboard motor's tiller and uses a linear actuator to steer for you. It holds a compass heading automatically, lets you steer manually with a joystick, and keeps your throttle steady with cruise control — all from a single ESP32 board.

---

## ✨ Features at a Glance

| Feature | What it does |
|---|---|
| **Autopilot heading hold** | Locks onto a compass heading and steers to keep it. Small corrections are slow and gentle; large corrections are faster. |
| **Manual joystick steering** | Push the joystick left or right for instant, full-speed tiller movement — just like grabbing the tiller yourself. |
| **Cruise control** | Locks the throttle at your current speed. Nudge the joystick forward for more power; pull back to cancel. |
| **9-axis compass** | ICM-20948 sensor with onboard DMP fusion (gyro + accelerometer + magnetometer). Tilt-compensated, auto-calibrating, drift-resistant. |
| **Compass calibration saved** | Once the compass reaches accuracy level 3/3, the calibration is saved to flash memory. Next boot, it's ready immediately — no figure-8s needed. |
| **Autopilot runaway protection** | Five separate safety guards prevent the actuator from driving to full lock if the compass gets disturbed (dropped, shocked, or moved). |
| **Soft motor ramping** | In autopilot mode, the motor ramps up and down smoothly instead of snapping on/off — extends actuator life and reduces mechanical wear. |
| **Direction reversal protection** | The motor always ramps to a full stop before switching direction — protects the H-bridge driver from damage. |

---

## 🎮 Controls

### Joystick

The single 2-axis joystick does three things depending on which axis you move:

| Joystick Direction | Axis | What Happens |
|---|---|---|
| **Left / Right** | Rudder axis | Steers the tiller. Push left → tiller goes left (port). Push right → tiller goes right (starboard). Instant full-speed — no delay or ramp. |
| **Forward** | Throttle axis | Increases engine RPM. If cruise control is on, nudges the locked speed higher. |
| **Pull back** | Throttle axis | **Cancels cruise control.** If cruise is off, does nothing. This is the "panic release" for cruise. |

**In autopilot mode:** The joystick still overrides the autopilot instantly. As long as you're holding the joystick deflected, you're steering manually. When you let go, the autopilot takes back over smoothly — no jolt or pulse.

### Switches

| Switch | Type | What it does |
|---|---|---|
| **Pilot switch** | Toggle (ON/OFF) | Turns autopilot on and off. When you flip it ON, the system locks onto whatever heading you're currently pointing. Flip it OFF to return to full manual control. |
| **Cruise button** | Momentary (press-release) | Toggles cruise control on/off. You need at least ~10% throttle to engage it (prevents accidentally locking at idle). Press again to disengage. |

### LEDs

| LED | Meaning |
|---|---|
| **Pilot LED** | Solid = autopilot is active |
| **Cruise LED** | Solid = cruise control is engaged |

---

## 🔧 How Autopilot Steering Works

The autopilot uses a **PI controller** (Proportional + Integral):

- **Proportional:** The further off heading you are, the faster the actuator moves. A 5° error gives a slow nudge; a 30° error gives a faster correction.
- **Integral:** If wind or current is steadily pushing you off course, the integral term slowly builds up to counteract it. Without this, the boat would slowly drift.

There's also a **hysteresis zone** (3°) — if the heading is within 3° of the target, the motor stops. This prevents the tiller from constantly hunting back and forth.

The motor **ramps** up and down smoothly (~200ms) in autopilot mode, so corrections are gentle. In manual mode, the joystick still gives instant, crisp response — no ramp.

---

## 🛡️ Safety Features

SimplePilot has multiple layers of protection:

1. **Heading timeout** — If the compass stops providing data for 5 seconds, the autopilot stops the motor. Manual joystick steering still works without a compass.

2. **Runaway prevention** — If the IMU is physically moved or shocked while the autopilot is running, five guards kick in:
   - Invalid compass data (NaN) is rejected entirely — not forced to 0° (which could look like 180° of error)
   - Heading jumps >15° per cycle are rejected as physically impossible (boats don't yaw 250°/sec)
   - Quaternion math is normalized to prevent corrupted heading calculation
   - Maximum heading error is capped at 45° — even a genuine 90° error won't command full-speed steering
   - After the IMU settles, the target heading automatically resets to wherever the boat is now pointing (the old target is meaningless after the IMU moved)

3. **Motor stop on mode change** — Switching from autopilot to manual immediately stops the motor. No coasting, no drift.

4. **Instant stop on joystick release** — When you release the joystick in autopilot mode, the motor stops instantly, then the PI controller ramps up smoothly from zero. No trailing pulse.

5. **Direction reversal protection** — The motor always ramps to zero before reversing. No H-bridge stress.

---

## 🖥️ Serial Monitor

Plug into the ESP32's USB port and open a serial monitor (115200 baud) to see live status:

**Autopilot on:**
```
PilotActive: ON  | JOY-PILOT:1900 | TGT:1900 | CUR HDNG:45 | TARGET HDNG:42 | DAC-OUT:41 | ERR:+3 P:+6.0 I:+0.4 PWM:86 DIR:R
```

**Manual / standby:**
```
PilotActive: OFF | JOY-PILOT:1900 | TGT:1900 | CUR HDNG:45 | TARGET HDNG:45 | DAC-OUT:41 | [STANDBY]
```

### What the fields mean

| Field | Meaning |
|---|---|
| `JOY-PILOT` | Raw joystick position (center ≈ 1900, full left ≈ 0, full right ≈ 4095) |
| `CUR HDNG` | Current compass heading (0–360°) |
| `TARGET HDNG` | The heading the autopilot is trying to hold |
| `DAC-OUT` | Throttle DAC output (41 = idle, 215 = wide open) |
| `ERR` | Heading error in degrees (+ = steer right, − = steer left) |
| `P` | Proportional term output |
| `I` | Integral term output |
| `PWM` | Current motor speed (0 = stopped, 255 = full speed) |
| `DIR` | Motor direction: R = right/starboard, L = left/port, − = stopped |
| `RESYNC` | (appears briefly) Compass was disturbed; target heading was reset |

### Serial commands

| Key | Action |
|---|---|
| `D` | Toggle DMP debug output (shows raw quaternion data) |
| `B` | Show current compass calibration biases |
| `S` | Save current biases to flash memory |
| `R` | Reset (erase) saved biases — forces a fresh calibration on next boot |

---

## ⚡ Quick Start

1. **Flash the firmware** to an ESP32 using the Arduino IDE (install the SparkFun ICM-20948 library via Library Manager first)
2. **Wire up the hardware** — see pin definitions in the source code or the pinout diagram
3. **Power on** — the serial monitor will show boot progress. If no saved calibration exists, you'll see the auto-cal prompt
4. **Calibrate the compass** — do figure-8 motions with the board until accuracy reaches 3/3. Once it does, biases are saved automatically and you won't need to do this again
5. **Manual steering** — just use the joystick. Left/port and right/starboard are instant
6. **Autopilot** — point the boat where you want to go, flip the pilot toggle ON. The autopilot locks that heading
7. **Cruise control** — push the throttle joystick forward to your desired speed, press the cruise button. Pull the joystick back to cancel

---

## 📁 Source Files

| File | What it contains |
|---|---|
| `simplepilot_v2.14.ino` | Main file — configuration, setup, and main loop |
| `sensor_init.ino` | ICM-20948 and DMP initialization |
| `heading.ino` | Compass heading from DMP quaternion data + runaway prevention |
| `eeprom_bias.ino` | Save / restore / reset compass calibration to flash |
| `autocal.ino` | Auto-calibration mode (first 5 minutes after boot) |
| `cruise.ino` | Cruise control logic + DAC throttle output |
| `pilot.ino` | Autopilot PI controller + manual joystick override |
| `serial_cmds.ino` | Serial command handler and diagnostics |

For the full technical documentation including tuning guide, DAC calibration, and troubleshooting, see `README_simplepilot_v2.14.txt`.

---

## ⚓ A Note on Tuning

Every boat is different. The default PI settings work well for a small outboard with a linear actuator, but you may need to adjust:

- **Boat oscillates (oversteers)?** → Reduce `KP` (try 1.5 instead of 2.0) or increase `HEADING_HYSTERESIS` (try 5° instead of 3°)
- **Boat drifts off (understeers)?** → Increase `KP` (try 2.5) or increase `KI` (try 0.3)
- **Motor hums but doesn't move?** → Increase `MIN_AUTO_SPEED` (try 100–120 instead of 80)
- **Motor too abrupt?** → Decrease `RAMP_RATE` (try 0.8 instead of 1.275)

See the full tuning guide in `README_simplepilot_v2.14.txt` for details.

---

*SimplePilot — because steering is the boring part.*
