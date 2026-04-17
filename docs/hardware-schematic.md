# MechDog V1.2 Hardware Schematic Analysis

Authoritative circuit-level documentation for the Hiwonder MechDog robot dog
controller, derived from the official schematic released 2024-10-11.

**Companion document:** `docs/stock-firmware-analysis.md` covers what the stock
firmware *does*; this document covers what the board *is*.

## Source

| Field | Value |
|---|---|
| File | `docs/manufacturer/4.SCH_MechDogV1.2.pdf` |
| Drawing number | ESP32-DEVKITC |
| Revision | C / V1.2 |
| Designer | daijun |
| Date | 2024-10-11 |
| Company | Hiwonder Technology Co., Ltd. |
| Sheet | 1 of 1 |

This schematic supersedes the earlier "MechDog Technical Specification.pdf"
(prose parameter sheet only, no circuits). All prior pin assignments in this
project were reverse-engineered from the stock MicroPython firmware and hardware
register scans; the V1.2 schematic is the first authoritative source.

---

## System Overview

```
Battery (2S LiPo, 6–8.4V)
  └── P1 JST (3-pin) ─── SW1 (power switch) ─── DC-IN jack
                                 │
                                VIN  ──────────────────────────────── SERVO1–8 (via fuses F1–F4)
                                 │
                          [RT8289GSP buck]
                          L1=15µH, R6=10K_0.1%/R7=3.16K feedback
                          D1=P6SMB550A TVS, C1=220µF/10V bulk
                                 │
                                5V  ─────────────────────────────── SERVO9–11 (no fuse)
                                 │                                   ESP32 5V pin (pin 19)
                                 │                                   I2C connector pull-ups
                          [ME6209-3.3V LDO]
                          D5 schottky, C17/C16=105 (1µF) caps
                                 │
                                3V3 ─────────────────────────────── ESP32 3V3 pin
                                                                     QMI8658 VDD/VDDIO
                                                                     I2C pull-ups (IMU bus)
                                                                     LED + button pull-ups
```

Key: SERVO1–8 run on the raw battery rail (VIN ≈ 7.4V nominal), giving full
torque but exposing servos to battery voltage variation. SERVO9–11 run on
regulated 5V — relevant to RR_hip (see servo port matrix).

---

## ESP32-DevKitC Pin Map

Module U2. Green-X marks in the schematic indicate no PCB trace connected
to that pin; those are marked NC below. IO0, IO2, and IO12 are ESP32
strapping pins (noted where used).

| Pin | GPIO alias | Net / signal | Direction | Notes |
|-----|-----------|-------------|-----------|-------|
| 1 | 3V3 | 3V3 | Power out | — |
| 2 | EN | — | Active-low reset | — |
| 3 | SENSOR_VP / IO36 | — | NC | Not routed |
| 4 | SENSOR_VN / IO39 | — | NC | Not routed |
| 5 | IO34 / A6 | ADC_BAT | Analog in | Battery voltage via 30K/10K divider |
| 6 | IO35 / A7 | IMU_INT | Digital in | QMI8658 INT2 interrupt |
| 7 | IO32 / A4 | IO32 | Bidirectional | J5 GPIO expansion pin 4 |
| 8 | IO33 / A5 | IO33 | Bidirectional | J5 GPIO expansion pin 3 |
| 9 | IO25 / A18 | SERVO1 | PWM out | FL_hip — firmware index 0 |
| 10 | IO26 / A19 | SERVO2 | PWM out | FL_knee — firmware index 1 |
| 11 | IO27 / A17 | SERVO3 | PWM out | FR_hip — firmware index 2 |
| 12 | IO14 / A16 | SERVO4 | PWM out | FR_knee — firmware index 3 |
| 13 | IO12 / A15 | SERVO11 | PWM out | Strapping pin; **idle** (no leg assigned) |
| 14 | GND | GND | Power | — |
| 15 | IO13 / A14 | SCL2 | I2C clock | I2C bus 2; previously undocumented in memory |
| 16 | SD2 | — | NC | Flash SPI — not exposed |
| 17 | SD3 | — | NC | Flash SPI — not exposed |
| 18 | CMD | — | NC | Flash SPI — not exposed |
| 19 | 5V | 5V | Power in | From RT8289 buck |
| 20 | CLK | — | NC | Flash SPI — not exposed |
| 21 | SD0 | — | NC | Flash SPI — not exposed |
| 22 | SD1 | — | NC | Flash SPI — not exposed |
| 23 | IO15 / A13 | SERVO7 | PWM out | **Idle** (no leg assigned) |
| 24 | IO2 / BOOT1 | SERVO8 | PWM out | RR_knee — firmware index 7; strapping pin |
| 25 | IO0 / BOOT0 | SERVO9 | PWM out | **Idle** (no leg assigned); strapping pin |
| 26 | IO4 / A10 | SERVO10 | PWM out | RR_hip — firmware index 6; **5V rail** |
| 27 | IO16 / RX2 | SERVO5 | PWM out | RL_hip — firmware index 4 |
| 28 | IO17 / TX2 | SERVO6 | PWM out | RL_knee — firmware index 5 |
| 29 | IO5 | KEY | Digital in | User button; active-LOW with 10K pullup |
| 30 | IO18 | LED | Digital out | Onboard blue LED; active-LOW; **unused by firmware** |
| 31 | IO19 | SDA2 | I2C data | I2C bus 2 |
| 32 | GND | GND | Power | — |
| 33 | IO21 | BUZZER | Digital out | NPN transistor base drive |
| 34 | IO3 / RXD0 | — | UART0 RX | Available; not routed to any connector |
| 35 | IO1 / TXD0 | — | UART0 TX | Available; not routed to any connector |
| 36 | IO22 | SDA1 | I2C data | I2C bus 1 |
| 37 | IO23 | SCL1 | I2C clock | I2C bus 1 |
| 38 | GND | GND | Power | — |
| 39 | GND | GND | Power | — |
| 40 | NC | — | NC | — |

---

## Servo Port Matrix

The board exposes 11 servo signal headers. SERVO1–8 are powered from VIN
(battery); SERVO9–11 are powered from 5V (regulated). Each port has a 1K
series resistor on the signal line for short-circuit/cross-drive protection.
Fuses F1–F4 (1A each) protect pairs of VIN-rail servos.

| Port | Ref-des | Power rail | Fuse | ESP32 GPIO | Series R | FW index | Leg | Notes |
|------|---------|-----------|------|-----------|---------|---------|-----|-------|
| SERVO1 | S1 | VIN | F1 | GPIO 25 | R13 1K | 0 | FL_hip | — |
| SERVO2 | S2 | VIN | F1 | GPIO 26 | R14 1K | 1 | FL_knee | — |
| SERVO3 | S3 | VIN | F2 | GPIO 27 | R8 1K | 2 | FR_hip | — |
| SERVO4 | S4 | VIN | F2 | GPIO 14 | R16 1K | 3 | FR_knee | — |
| SERVO5 | S5 | VIN | F3 | GPIO 16 | R17 1K | 4 | RL_hip | — |
| SERVO6 | S6 | VIN | F3 | GPIO 17 | R21 1K | 5 | RL_knee | — |
| SERVO7 | S7 | VIN | F4 | GPIO 15 | R22 1K | — | **idle** | Available for add-on servo |
| SERVO8 | S8 | VIN | F4 | GPIO 2 | R23 1K | 7 | RR_knee | GPIO2 = strapping pin (boot issues if pulled low at power-on) |
| SERVO9 | S9 | **5V** | none | GPIO 0 | R24 1K | — | **idle** | GPIO0 = strapping pin; avoid driving during boot |
| SERVO10 | S10 | **5V** | none | GPIO 4 | R18 1K | 6 | RR_hip | 5V rail — only main-leg servo not on VIN |
| SERVO11 | S11 | **5V** | none | GPIO 12 | R25 1K | — | **idle** | GPIO12 = strapping pin (flash voltage select) |

**Firmware servo index → GPIO mapping** (from `firmware/include/config.h`):

```
SERVO_PINS = {25, 26, 27, 14, 16, 17, 4, 2}
//            FL  FL  FR  FR  RL  RL  RR  RR
//            hip kne hip kne hip kne hip kne
// Board labels: S1  S2  S3  S4  S5  S6  S10 S8
```

Ports SERVO7, SERVO9, SERVO11 are wired on the PCB but carry no leg servo in
the standard MechDog build. Any of these three headers can be used for
expansion servos or accessories.

---

## IMU Circuit

Component U1, part QMI8658 (14-pin LGA). Connected to I2C bus 1.

| QMI8658 pin | Signal | Connection |
|-------------|--------|-----------|
| 1 | SDO / SA0 | R28 (0Ω) → 3V3 |
| 2 | SDX | NC |
| 3 | SCX | NC |
| 4 | INT1 | NC |
| 5 | VDDIO | 3V3 (C18 100nF decoupling) |
| 6 | GND1 | GND |
| 7 | GND2 | GND |
| 8 | VDD | 3V3 (C8 100nF decoupling) |
| 9 | INT2 | IMU_INT → ESP32 GPIO 35 |
| 10 | CS-AUX | NC |
| 11 | SDO-AUX | NC |
| 12 | CS | R27 (0Ω) → 3V3 — forces I2C mode |
| 13 | SCL | SCL1 → ESP32 GPIO 23 |
| 14 | SDA | SDA1 → ESP32 GPIO 22 |

R27 tying CS high selects I2C mode (SPI mode requires CS low). R28 tying
SDO/SA0 high selects I2C address 0x6B per the QMI8658 datasheet.

**Address discrepancy — open question:** Our hardware I2C scan reliably finds
the IMU at address 0x6A, not 0x6B. Possible explanations: (a) SDO/SA0 behaves
inversely in the QMI8658C variant vs the datasheet; (b) R28 is populated with
0Ω in the schematic but not on production boards; (c) there is a QMI8658 mask
revision with different address logic. Recommend checking with a logic analyzer
on the SA0 pin at boot. Our firmware uses address 0x6A, which is empirically
correct — do not change without hardware verification.

INT2 (not INT1) is wired to GPIO 35. INT1 is left floating.

---

## I2C Buses

### I2C Bus 1 — SDA = GPIO 22, SCL = GPIO 23

Onboard devices:
- QMI8658 IMU at 0x6A (see discrepancy note above)
- Hiwonder I2C ultrasonic sonar at 0x77 (confirmed via I2C scan)
  - The sonar module also carries two addressable RGB LEDs controlled via
    `sonar_set_rgb()` in `firmware/src/sensor_task.cpp`. These are the LEDs
    driven by `cmd_led` — **not** the onboard blue LED.

External connectors (5V-powered, 5264-4P 2.5mm pitch):
- J1, J2, J3 — three parallel I2C1 headers for expansion devices (AI camera
  module, additional sensors, etc.)

Pull-up arrangement: 3V3 pull-ups on the bus (diode D4/S4 cluster visible in
the I2C1 section, 3V3 rail). Connector power pin is 5V for peripheral supply.

### I2C Bus 2 — SDA = GPIO 19, SCL = GPIO 13

SCL2 on GPIO 13 was **not previously documented** in our memory or config.h.

External connector:
- J4 — one 5264-4P 2.5mm I2C2 header

Bus protection via D3/S4 (diode cluster on 3V3 side). Connector pinout matches
J1/J2/J3 (5V, GND, SDA, SCL). Our firmware currently does not use I2C bus 2.

---

## Power Supply Chain

### VIN rail (battery voltage, ~7.4V nominal)

- Entry: P1 3-pin JST battery connector (VIN, GND, thermistor/spare)
- Switch: SW1 — SPDT power switch in series with VIN
- Input filter: C4 (100nF), C6 (10µF) ceramic + C1 (220µF/10V electrolytic)
- ESD: C7 (104pF) across 5V TVS D2 near ESP32 5V pin

### 5V rail — RT8289GSP (U3) synchronous buck converter

- Input: VIN
- Output: 5V
- Inductor: L1 15µH
- Feedback divider: R6 10K_0.1% (top) / R7 3.16K (bottom)
- Output bulk: C11 22µF + C10 100nF
- Additional output: C5 22µF + C3 4.7µF
- Input bulk: C1 220µF/10V (shared with VIN filter)
- TVS: D1 P6SMB550A (input transient protection)
- Bootstrap: C2 10nF (BOOT pin)

### 3.3V rail — ME6209-3.3V (U4) LDO

- Input: 5V (with D5 schottky series diode for reverse-polarity protection)
- Output: 3V3
- Decoupling: C17 + C16 (both 105 = 1µF ceramic)

---

## Peripherals

### LED Indicator Circuit

**PWR RED (power indicator):** Always-on when 3V3 is present.
- 3V3 → R12 (3.3K) → RED LED (anode) → GND (cathode)
- No GPIO control; purely passive power indicator.

**LED1 BLUE (user / status LED):** Driven from GPIO 18, **active-LOW**.
- 3V3 → R19 (1K) → LED1 BLUE (anode) → LED net (cathode) → GPIO 18
- To illuminate: drive GPIO 18 LOW. To extinguish: drive HIGH or set Hi-Z.
- **Not used by our custom firmware.** The firmware uses `sonar_set_rgb()` for
  status indication (controls RGB LEDs on the I2C sonar module). GPIO 18 is
  available for future use (simple one-bit status indication).

### User Button

Component K1 (SW-PB_3×6×4.3 momentary pushbutton). **Active-LOW**.
- 3V3 → R20 (10K pullup) → KEY net → GPIO 5
- When pressed: KEY pulled to GND → reads LOW.
- When released: reads HIGH (3V3 via pullup).

### Buzzer

Component BUZ1 (SEA-12085 magnetic buzzer). Driven from GPIO 21 via NPN.
- 5V → BUZ1 (+) → BUZ1 (−) → Q1 collector (S8050 NPN transistor)
- GPIO 21 → R11 (1K) → Q1 base
- R15 (10K) from Q1 base to GND (keeps transistor off when GPIO floating)
- Drive GPIO 21 HIGH to activate buzzer; LOW to silence.
- Buzzer is 5V-supplied (not VIN), so tone is independent of battery level.

### Battery ADC

- VIN → R1 (30K, 1%) → ADC_BAT → R5 (10K, 1%) → GND
- Voltage divider ratio: ADC_BAT = VIN × 10/(30+10) = VIN / **4.0** (exact)
- ESP32 ADC pin: GPIO 34 (input only, no internal pull-up/down)
- Config.h uses `BATTERY_DIVIDER 3.9f` — slightly below the schematic's 4.0.
  This is a calibrated value accounting for real resistor tolerances and ADC
  non-linearity; do not change to 4.0 without recalibrating against a known
  voltage.
- Schematic uses 1% tolerance resistors for ADC accuracy.

---

## Expansion Connectors

### P11 — Servo Expansion (CON2.0×3P, 2.0mm pitch, 3-pin)

| Pin | Signal |
|-----|--------|
| 1 | GND |
| 2 | GND |
| 3 | VIN |

**Note:** In the schematic, pin 3 is labeled VIN but no servo signal is routed
to this connector — it provides battery power only. Likely intended as a power
tap for an external servo controller or accessory. Verify pin 3 signal
continuity with a DMM before assuming this header carries PWM.

### J5 — GPIO Expansion (4-pin, standard pitch)

| Pin | Signal |
|-----|--------|
| 1 | 5V |
| 2 | GND |
| 3 | IO33 (GPIO 33) |
| 4 | IO32 (GPIO 32) |

GPIO 32 and 33 are general-purpose I/O with ADC capability (A4/A5). They are
not used by any MechDog system function and are available for user expansion
(extra sensors, UART, SPI, additional PWM, etc.).

---

## Firmware Cross-Reference

| Signal | GPIO | Config.h | Schematic match | Notes |
|--------|------|---------|----------------|-------|
| SERVO_PINS[0] FL_hip | 25 | ✓ `SERVO_PINS[0]` | SERVO1 / S1 | VIN rail |
| SERVO_PINS[1] FL_knee | 26 | ✓ `SERVO_PINS[1]` | SERVO2 / S2 | VIN rail |
| SERVO_PINS[2] FR_hip | 27 | ✓ `SERVO_PINS[2]` | SERVO3 / S3 | VIN rail |
| SERVO_PINS[3] FR_knee | 14 | ✓ `SERVO_PINS[3]` | SERVO4 / S4 | VIN rail |
| SERVO_PINS[4] RL_hip | 16 | ✓ `SERVO_PINS[4]` | SERVO5 / S5 | VIN rail |
| SERVO_PINS[5] RL_knee | 17 | ✓ `SERVO_PINS[5]` | SERVO6 / S6 | VIN rail |
| SERVO_PINS[6] RR_hip | 4 | ✓ `SERVO_PINS[6]` | SERVO10 / S10 | **5V rail** — only leg servo on 5V |
| SERVO_PINS[7] RR_knee | 2 | ✓ `SERVO_PINS[7]` | SERVO8 / S8 | VIN rail; strapping pin |
| I2C_SDA_PIN | 22 | ✓ | SDA1 | I2C bus 1 |
| I2C_SCL_PIN | 23 | ✓ | SCL1 | I2C bus 1 |
| BATTERY_ADC_PIN | 34 | ✓ | ADC_BAT | 30K/10K divider |
| QMI8658_ADDR | 0x6A | ✓ (empirical) | 0x6B per R28 strap | See discrepancy note |
| SONAR_ADDR | 0x77 | ✓ | I2C1 bus | Scan-confirmed |
| IMU_INT | 35 | (in memory only) | IO35 pin 6 | Schematic-confirmed |
| Buzzer | 21 | (in source only) | BUZZER net | Q1 S8050 driver |
| LED (user blue) | 18 | **not in config** | LED net | Active-LOW; unused by firmware |
| KEY (button) | 5 | (in source only) | KEY net | Active-LOW, 10K pullup |
| SCL2 | 13 | **not in config** | SCL2 net | I2C bus 2; previously undocumented |
| SDA2 | 19 | (in memory only) | SDA2 net | I2C bus 2 |
| IO32 expansion | 32 | **not used** | J5 pin 4 | Available |
| IO33 expansion | 33 | **not used** | J5 pin 3 | Available |
| UART0 RX | 3 | **not used** | IO3/RXD0 | Free on header |
| UART0 TX | 1 | **not used** | IO1/TXD0 | Free on header |

### Memory corrections

Prior memory entries had incorrect V1.2 board silk-screen labels for some
servo ports. The firmware GPIO assignments are correct; only the board-label
annotations were wrong:

| Memory note | Correct per V1.2 |
|-------------|-----------------|
| SERVO5 = GPIO17 | SERVO5 = GPIO **16** (GPIO17 is SERVO6) |
| SERVO6 = GPIO15 | SERVO6 = GPIO **17** (GPIO15 is SERVO7) |
| SERVO7 = GPIO2 | SERVO7 = GPIO **15** (GPIO2 is SERVO8) |
| SERVO9 = GPIO4 | SERVO9 = GPIO **0** (GPIO4 is SERVO10) |
| SERVO10 = GPIO16 | SERVO10 = GPIO **4** |
| SCL2 not in memory | SCL2 = GPIO **13** |

The firmware `SERVO_PINS = {25,26,27,14,16,17,4,2}` is physically correct;
only the human-readable board labels were mislabeled in our notes.

---

## Open Questions and Follow-Ups

### Implementation Status (post-merge)

The following items from the original follow-up list have been implemented in
firmware as part of the v12 hardware peripheral support work:

- **Buzzer (GPIO 21)** — `cmd_buzzer` command; LEDC ch 9; `config.h` `BUZZER_PIN`/`BUZZER_LEDC_CH`.
- **Button K1 (GPIO 5)** — 20ms debounce, 1000ms long-press threshold; emits `telem_button`.
- **Onboard blue LED (GPIO 18, active-LOW)** — `cmd_led` with `led=0`; `config.h` `ONBOARD_LED_PIN`.
- **IMU interrupt (GPIO 35, INT2)** — binary semaphore; 50ms fallback polling in sensor task; `config.h` `IMU_INT_PIN`/`IMU_INT_SLACK_MS`.
- **I2C bus 2 (SDA=19, SCL=13, J4)** — Wire1 instance; `cmd_i2c` with `bus=2`; `config.h` `I2C2_*`.
- **GPIO expansion (pins 32, 33, 1, 3)** — `cmd_gpio`; pins 1 and 3 return `analog: -1` (not ADC-capable).
- **Aux servo ports (GPIO 15, 0, 12)** — firmware indices 8–10, LEDC ch 10–12, `AUX_SERVOS_ENABLED=1`.

Items 4, 5, 6 from the original list are resolved. Items 1–3 remain open (hardware verification tasks).

**Still unresolved — hardware verification required:**
- **QMI8658 I2C address discrepancy (0x6A vs 0x6B)** — see item 1 below. Firmware uses 0x6A (empirically correct); do not change without logic analyzer verification.

1. **QMI8658 I2C address strap (priority: low)** — Schematic shows R28 (0Ω)
   tying SDO/SA0 to 3V3, which should set address 0x6B. Hardware sees 0x6A.
   Verify with logic analyzer or by probing the SA0 pad at boot. Our firmware
   address of 0x6A is empirically verified and should not change until this
   is resolved.

2. **P11 servo expansion signal pin (priority: low)** — DMM continuity test
   to confirm whether P11 pin 3 (labeled VIN in schematic) carries any PWM
   signal. As drawn, it appears to be power only.

3. **RR_hip on 5V rail (priority: informational)** — SERVO10 / GPIO 4 is on
   the regulated 5V supply rather than VIN. Under load, a 5V servo sees more
   voltage drop than a VIN servo. If RR_hip ever shows torque asymmetry vs
   the other hip servos during gait tuning, check this supply rail first.

4. **GPIO 18 (onboard blue LED) — resolved** — Now implemented via `cmd_led led=0`; active-LOW. See Implementation Status above.

5. **Config.h cleanup (separate patch)** — Add inline comments to `SERVO_PINS`
   annotating each entry with its board silkscreen label (SERVO1, SERVO2, …).
   Add `I2C2_SDA_PIN 19` and `I2C2_SCL_PIN 13` if I2C bus 2 is ever used.

6. **Memory update (separate patch)** — Correct `project_hardware_findings.md`
   servo port label table and add SCL2=GPIO13.


---

## Reproducible Extraction Commands

All run from the repo root. Artifacts are ephemeral (not committed).

```bash
# Extract all text with layout preservation
pdftotext -layout docs/manufacturer/4.SCH_MechDogV1.2.pdf /tmp/mechdog_v12_sch.txt

# Render to PNG at 300 DPI for visual inspection
pdftoppm -r 300 docs/manufacturer/4.SCH_MechDogV1.2.pdf /tmp/mechdog_v12 -png
# Output: /tmp/mechdog_v12-1.png (3508×2481 px)

# Crop individual sub-circuits using PIL
python3 - <<'EOF'
from PIL import Image
im = Image.open('/tmp/mechdog_v12-1.png')
crops = {
    'esp32':       (80,   60,  1450, 1200),
    'powersupply': (1900, 60,  3500, 1200),
    'i2c_buzzer':  (1000, 60,  2000,  900),
    'imu':         (80,  1700, 1000, 2481),
    'servos':      (800, 1200, 3508, 2100),
    'expansion':   (900, 1900, 2000, 2481),
}
for name, box in crops.items():
    im.crop(box).save(f'/tmp/mechdog_v12_{name}.png')
EOF
```
