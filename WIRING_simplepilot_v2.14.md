# SimplePilot v2.14 — Wiring Reference

> Companion to the SVG schematic: `schematic_simplepilot_v2.14.svg`

## ESP32 Pin Assignments

| ESP32 GPIO | Function | Direction | Signal Type | Connected To | Notes |
|---|---|---|---|---|---|
| **GPIO 21** | I2C SDA | I/O | I2C (400kHz) | ICM-20948 SDA | ESP32 default I2C SDA. Needs 4.7kΩ pullup to 3V3 (often on module). |
| **GPIO 22** | I2C SCL | Output | I2C (400kHz) | ICM-20948 SCL | ESP32 default I2C SCL. Needs 4.7kΩ pullup to 3V3 (often on module). |
| **GPIO 25** | DAC Throttle | Output | DAC (0–255) | Op-Amp input | ESP32 DAC1. 0=0V, 255≈3.3V. Op-Amp (gain≈1.5×) boosts to 0.8V–4.2V for hall sensor. |
| **GPIO 26** | PWM_A Motor | Output | PWM (0–255) | H-Bridge IN1 / PWM-A | Right/Starboard direction. `analogWrite()` PWM. |
| **GPIO 27** | PWM_B Motor | Output | PWM (0–255) | H-Bridge IN2 / PWM-B | Left/Port direction. `analogWrite()` PWM. |
| **GPIO 32** | Pilot Joystick | Input | Analog (0–4095) | Joystick VRy pot | 12-bit ADC. Center≈1900. Left=0, Right=4095. Rudder axis. |
| **GPIO 33** | Pilot Switch | Input | Digital (PULLUP) | SPST toggle switch | LOW=ON (autopilot engage). Internal pullup — no external resistor needed. |
| **GPIO 34** | Cruise Joystick | Input | Analog (0–4095) | Joystick VRx pot | 12-bit ADC. Center≈1860. **Input-only pin** — no internal pullup available. |
| **GPIO 4** | Cruise Button | Input | Digital (PULLUP) | Momentary N.O. button | LOW=Pressed (toggle cruise). Internal pullup. Debounced in software (50ms). |
| **GPIO 18** | Pilot LED | Output | Digital | LED + 330Ω → GND | HIGH=ON when autopilot active. |
| **GPIO 19** | Cruise LED | Output | Digital | LED + 330Ω → GND | HIGH=ON when cruise control engaged. |

## Power Connections

| Source | Destination | Voltage | Notes |
|---|---|---|---|
| 12V Battery (+) | ESP32 VIN | 5V (via onboard regulator) | ESP32 DevKit has 5V→3.3V LDO. VIN accepts 5V–12V. |
| 12V Battery (+) | H-Bridge VCC | 12V | Motor power. Must be capable of 5–20A for actuator stall current. |
| 12V Battery (+) | Op-Amp VCC | 12V (or 5V) | Depends on op-amp choice. LM358 works on 5V–32V. |
| ESP32 3V3 | ICM-20948 VCC | 3.3V | Both Pimoroni and SparkFun breakouts have onboard regulators. |
| ESP32 3V3 | Joystick VCC | 3.3V | Powers both potentiometers. 3.3V = ADC full scale. |
| **Common GND** | All components | 0V | **CRITICAL**: All grounds must be tied together. Star grounding recommended. |

## Component Details

### ICM-20948 9-Axis IMU

| Pin | Connection | Notes |
|---|---|---|
| VCC | ESP32 3V3 | 3.3V logic. Module has onboard regulator if using 5V. |
| GND | Common GND | |
| SDA | GPIO 21 | I2C data. 4.7kΩ pullup to 3V3. |
| SCL | GPIO 22 | I2C clock. 4.7kΩ pullup to 3V3. |
| AD0 | GND (Pimoroni) or VCC (SparkFun) | Sets I2C address: AD0=GND → 0x68, AD0=VCC → 0x69. Set `AD0_VAL` in code accordingly. |

**I2C Address Configuration in Code:**
```cpp
#define AD0_VAL  0   // 0 = Pimoroni (0x68), 1 = SparkFun (0x69)
```

### 2-Axis Joystick (Potentiometer Module)

| Pin | Connection | Notes |
|---|---|---|
| VCC | ESP32 3V3 | Must be 3.3V for ADC accuracy. |
| GND | Common GND | |
| VRx | GPIO 34 | Cruise/Throttle axis. Center ≈ 1860. |
| VRy | GPIO 32 | Pilot/Rudder axis. Center ≈ 1900. |
| SW | *(not connected)* | Joystick push-button. Not used in SimplePilot. |

**Calibration Values (adjust for your joystick):**
```cpp
#define JOYCRUISE_CENTER  1860   // VRx rest position
#define JOYPILOT_CENTER   1900   // VRy rest position
```

### Switches

**Pilot Toggle (SPST):**
```
  GPIO33 ──┤ SWITCH ├── GND
            (closed = pilot ON)
```
- Uses `INPUT_PULLUP` — switch connects GPIO33 to GND when closed.
- LOW = autopilot engaged, HIGH = manual mode.

**Cruise Button (Momentary N.O.):**
```
  GPIO4 ──┤ BUTTON ├── GND
           (pressed = cruise toggle)
```
- Uses `INPUT_PULLUP` — button connects GPIO4 to GND when pressed.
- Software debounced (50ms delay after press).

### H-Bridge Motor Driver (e.g., BTS7960 or VNH2SP30)

| Pin | Connection | Notes |
|---|---|---|
| VCC | 12V Battery | Motor supply. Use heavy-gauge wire for current capacity. |
| GND | Common GND | Must share ground with ESP32. |
| PWM_A (IN1) | GPIO 26 | Right/Starboard direction PWM. |
| PWM_B (IN2) | GPIO 27 | Left/Port direction PWM. |
| EN (enable) | 12V (or PWM) | Must be HIGH for motor to run. Some drivers tie this to VCC. |
| OUT_A | Actuator M+ | Motor output A. |
| OUT_B | Actuator M− | Motor output B. |

**Motor Control Logic:**
```
moveRight():  PWM_A = motorSpeed, PWM_B = 0    → Starboard
moveLeft():   PWM_A = 0,          PWM_B = motorSpeed → Port
StopActuator: PWM_A = 0,          PWM_B = 0    → Stopped
```

### DAC + Op-Amp Throttle Circuit

```
                 ┌──────────┐
  GPIO25 (DAC) ──┤ +        ├───→ Hall Throttle Sensor
                 │  Op-Amp  │     (Outboard Engine)
  Rf/Ri div  ────┤ -        │    Gain ≈ 1.5×
                 └──────────┘
                      │
                    GND
```

**Voltage Mapping:**
| DAC Value | DAC Output | Op-Amp Output | Engine State |
|---|---|---|---|
| 0 | 0.0V | 0.0V | ⚠ **Sensor fault** — never use! |
| 41 | ~0.53V | ~0.8V | Idle / tickover |
| 215 | ~2.78V | ~4.2V | Wide Open Throttle (WOT) |
| 255 | ~3.3V | ~5.0V | Beyond WOT (don't use) |

**⚠ IMPORTANT:** Never set DAC below 41 (DACLOW). The hall-effect throttle sensor needs ~0.8V minimum to register idle. 0V looks like a disconnected sensor fault to the outboard.

**Calibration Values:**
```cpp
#define DACLOW   41    // Idle: ~0.8V post-Op-Amp
#define DACHIGH  215   // WOT:  ~4.2V post-Op-Amp
```

### LEDs

```
  GPIO18 ──┤ 330Ω ├───┤▶├─── GND    (Pilot LED — YELLOW)
  GPIO19 ──┤ 330Ω ├───┤▶├─── GND    (Cruise LED — GREEN)
```

- 330Ω resistor limits current to ~10mA at 3.3V (standard LED).
- Pilot LED: ON = autopilot active.
- Cruise LED: ON = cruise control engaged.

## Actuator Safety Limits

The linear actuator has physical endstops. The software enforces soft limits:

```cpp
#define LIMIT_LEFT   500    // Actuator left limit (ADC counts)
#define LIMIT_RIGHT  3500   // Actuator right limit (ADC counts)
```

These constrain the joystick input range to prevent driving the actuator past its mechanical limits.

## Key Constants Quick Reference

| Constant | Value | Description |
|---|---|---|
| `SAMPLE_HZ` | 25 | Compass update rate (Hz) |
| `DECLINATION_DEG` | 14.9° | Magnetic declination (Possession Point) |
| `KP` | 2.0 | Proportional gain (autopilot) |
| `KI` | 0.2 | Integral gain (autopilot) |
| `HEADING_HYSTERESIS` | 3° | Don't steer if error < this |
| `MAX_ERROR_DEG` | 45° | Cap heading error for PI |
| `MIN_AUTO_SPEED` | 80 | Minimum PWM for actuator movement |
| `RAMP_RATE` | 1.275 | PWM ramp rate per ms (~200ms full ramp) |
| `DACLOW` | 41 | Throttle idle DAC value |
| `DACHIGH` | 215 | Throttle WOT DAC value |

## Wiring Checklist

- [ ] ESP32 VIN connected to 12V battery (via fuse!)
- [ ] All GND connections tied together (ESP32, H-Bridge, Op-Amp, IMU, Joystick)
- [ ] ICM-20948 SDA → GPIO21, SCL → GPIO22 (check I2C pullups on module)
- [ ] ICM-20948 AD0 pin matches `AD0_VAL` in code (0=Pimoroni/0x68, 1=SparkFun/0x69)
- [ ] Joystick VRx → GPIO34, VRy → GPIO32, VCC → 3V3, GND → GND
- [ ] Pilot toggle switch: GPIO33 ↔ GND (INPUT_PULLUP, LOW=ON)
- [ ] Cruise momentary button: GPIO4 ↔ GND (INPUT_PULLUP, LOW=Pressed)
- [ ] H-Bridge PWM_A ← GPIO26, PWM_B ← GPIO27
- [ ] H-Bridge VCC ← 12V battery, EN ← 12V (or PWM)
- [ ] Actuator M+/M− → H-Bridge motor outputs
- [ ] DAC GPIO25 → Op-Amp input → Hall throttle sensor
- [ ] Pilot LED: GPIO18 → 330Ω → LED → GND
- [ ] Cruise LED: GPIO19 → 330Ω → LED → GND
- [ ] 12V battery fused (recommended: 10A slow-blow for actuator circuit)
- [ ] ESP32 powered via VIN (not 3V3 pin — onboard regulator needed)
