# Stock Firmware Analysis

Analysis of the Hiwonder MechDog stock firmware flash dump at
`firmware/stock_firmware.bin` (1,747,968 bytes, gitignored). Performed April 2026.
All raw artifacts (partitions, disassembly, source extracts) are kept under
`stock_firmware_dump/` which is also gitignored.

## Firmware Identity

| Field | Value |
|---|---|
| MicroPython version | v1.21.0-dirty (custom Hiwonder build) |
| MicroPython build date | 2025-05-05 |
| ESP-IDF version | v5.0.2 |
| libc | newlib 4.1.0 |
| Chip | ESP32 (Chip ID 0) |
| Flash mode | DIO, 40 MHz, 4 MB |
| ELF SHA256 | `9836d007e414f2b45e017924fc13d70d36ee49f84a19163644ca9b2a23abbbae` |

The "dirty" suffix on MicroPython means Hiwonder made local modifications to the
MicroPython source.

## Dump Status

The raw dump is **truncated at 0x1AAC00** (1,747,968 bytes). The full flash is
4 MB. The factory app partition is also truncated: 1,682,432 of 1,966,080 bytes
are present. The LittleFS (vfs) partition is completely absent.

**Action item**: next time stock firmware is flashed for testing, run
`esptool read_flash 0 0x400000 firmware/stock_full.bin` to capture the complete
4 MB image. The action `.rob` files and live user data (WiFi credentials, offsets)
live in LittleFS and are missing from the current dump.

## Partition Layout

Decoded from the raw partition table at 0x8000:

| Name | Type | Start | Size | End |
|---|---|---|---|---|
| nvs | data/nvs | 0x9000 | 24 KB (0x6000) | 0xF000 |
| phy_init | data/phy | 0xF000 | 4 KB | 0x10000 |
| factory | app | 0x10000 | 1,966 KB (0x1F0000) | 0x200000 |
| vfs | data/littlefs | 0x200000 | 2 MB | 0x400000 |

The NVS partition is empty (all pages erased — likely wiped during custom
firmware flashing). No factory per-unit calibration is stored there.

## App Segment Map

From `esptool image-info` on the carved `partitions/factory.bin`:

| Segment | File Offset | Load Address | Size | Type |
|---|---|---|---|---|
| 0 | 0x18 | 0x3F400020 | 303,668 B | DROM (.rodata, frozen bytecode + constants) |
| 1 | 0x4AA54 | 0x3FF80063 | 8 B | RTC_DRAM |
| 2 | 0x4AA64 | 0x3FFBDB60 | 21,448 B | DRAM |
| 3 | 0x4FE34 | 0x40080000 | 476 B | IRAM |
| 4 | 0x50018 | 0x400D0020 | 1,226,828 B | IROM (.text, main code) |
| 5 | 0x17B46C | 0x400801DC | 126,184 B | IRAM |
| 6 | 0x19A35C | 0x400C0000 | 100 B | RTC_IRAM |
| 7 | 0x19A3C8 | 0x500007F8 | 2,056 B | RTC_DATA |

Entry point: `0x40081474`.

The DROM segment (segment 0) is where all frozen MicroPython module bytecode and
string constants live. The frozen module source filenames, qstrs (method names,
attribute names, kwarg names), and some modules' full Python source text are
recoverable from this segment without a disassembler.

## Embedded Python Modules

These modules are frozen into the firmware. Source text is recoverable for
some; others are compiled to bytecode with only their symbol names readable:

### Readable source text (stored as frozen string literals)

- **Main app** (`# Hiwonder MechDog / APP Bluetooth and WIFI control`): full
  Python source extracted to `stock_firmware_dump/source/main_app_source.py`.
  10,984 characters. Covers BLE/WiFi control loop, action dispatch, sonar,
  battery query, balance toggle. Compile date strings confirm 2025-05-05 build.

### Compiled bytecode (symbol names only readable)

All other modules are frozen as compiled `.mpy` v6 bytecode. Their qstr tables
(method names, kwargs, class attributes) are readable from the DROM segment
but their logic requires MPY v6 bytecode disassembly (Phase B).

| Module file | Key classes / functions |
|---|---|
| `HW_MechDog.py` | `MechDog` class — gait, IK, servo offsets, actions, balance |
| `Hiwonder_IIC.py` | `I2CSonar`, `QMI8658`, `WonderCam`, `asr_module` |
| `Hiwonder.py` | `LED`, `Buzzer`, `Button`, `Battery_power()`, `startMain()` |
| `Hiwonder_ADC.py` | `LightSensor`, `SoilSensor`, `Knob`, `Slider` |
| `Hiwonder_BLE.py` | `BLE` — BLE UART protocol client |
| `Hiwonder_WIFI.py` | `WIFI_CL` — WiFi TCP server, `GROUP|MechDog{}|END` identity |
| `__action.py` | Action script name table |
| `_boot_robot.py` | Robot boot config, gait state variables |
| `default_test.py` | Test script with gait/servo state structure |

Standard MicroPython frozen modules also present: `asyncio`, `mip`, `requests`,
`umqtt`, `webrepl`, `neopixel`, `dht`, `ds18x20`, `onewire`, `ntptime`, `upysh`.

## Hardware Unknowns — Phase A Findings

### Action list (complete)

All 15 actions from `action_list` in main app source, plus 2 bonus actions:

| ID | Name |
|---|---|
| 1 | left_foot_kick |
| 2 | right_foot_kick |
| 3 | stand_four_legs |
| 4 | sit_dowm *(typo for sit_down)* |
| 5 | go_prone |
| 6 | stand_two_legs |
| 7 | handshake |
| 8 | scrape_a_bow |
| 9 | nodding_motion |
| 10 | boxing |
| 11 | stretch_oneself |
| 12 | pee |
| 13 | press_up |
| 14 | rotation_pitch |
| 15 | rotation_roll |
| — | two_legs_back *(in __action.py qstrs, not in main list)* |
| — | right_foot_kick_2 *(bonus variant)* |

### Action file format

Action data is stored in `.rob` files (LittleFS — absent from our dump). Format:
`$$>{action_name}<$$` header, then sections keyed `Actions`, `Duration`,
`Servos`, terminated by `$$>end<$$`. The `import json` reference in `HW_MechDog`
suggests the `.rob` data may be JSON-formatted keyframe tables.
**Cannot fully decode without the LittleFS partition.**

### `set_gait_params` parameters

Validated integer parameters confirmed from runtime error strings:
- `stride_height` — integer
- `swing_time` — integer  
- `stand_time` — integer

`strid_width` (typo for stride_width) appears in `_boot_robot.py` qstrs alongside
these and may also be a parameter. The internal gait state dict additionally
tracks `speed_x`, `trasnlation` (typo), `normal_attitude`, `rotation_pitch`,
`rotation_roll`, `stomp_on_the_spot` — these are probably updated by `move()` and
`set_pose()`, not `set_gait_params`.

**Exact function signature requires Phase B bytecode disassembly.**

### `leg_set_ik` signature

From error message: `"Six parameters are required."` — 6 total positional args.
`leg_index` validated as integer in range 1–4. Related functions:
- `leg_set_ik(leg_index, ?, ?, ?, ?, ?)` — 6 params total
- `leg_set_ik_offset(...)` — adds an offset to IK target
- `leg_set_theta(...)` — sets joint angles directly (bypasses IK)
- `leg_theta_to_pulsewidth(theta)` — converts joint angle to PWM pulse width
- `leg_ik(...)` — probably the lower-level IK calculation function

**Remaining 5 args likely: x_mm, y_mm, z_mm + 2 others (roll/pitch? speed?). Requires Phase B.**

### 9th offset in `read_all_offset()`

`read_all_offset()` returns 9 values for 8 servos. The most likely interpretation:
offset slots 1–8 = per-servo pulse-width trim offsets (stored in `__offsetangle`
array on `MechDog`), slot 9 = IMU gyro calibration offset (`_gyro_offset` in
`QMI8658` class, accessed via `_offset_config`). The `save_angleoffset` /
`read_angleoffset` methods handle servo-specific calibration while `_offset_config`
in QMI8658 handles the gyro zero-point.

**Definitively confirming slot 9 identity requires Phase B bytecode disassembly
or reflashing stock and inspecting the return value.**

### QMI8658 driver

The `QMI8658` is a **pure Python class** inside `Hiwonder_IIC.py`. Its interface
closely mirrors the MPU6050 class (same method names, same register-name
conventions: `_PWR_MGMT_1`, `_ACCEL_CONFIG`, `_GYRO_CONFIG`, `_TEMP_OUT0`, etc.).

Key methods and attributes (from qstr table):

| Symbol | Role |
|---|---|
| `Config_apply` | Applies IMU configuration registers |
| `WhoAmI` | Reads WHO_AM_I register |
| `Read_Revision` | Reads revision register |
| `Read_Raw_XYZ` | Reads raw 16-bit XYZ data |
| `Read_XYZ` | Returns scaled XYZ values |
| `getResult` | Returns processed result |
| `read_sensor_data` | High-level read loop |
| `get_accel_range` / `set_accel_range` | Accelerometer range (±2G/4G/8G/16G) |
| `get_gyro_range` / `set_gyro_range` | Gyro range (±250/500/1000/2000 deg/s) |
| `read_accel_data` / `read_accel_abs` | Accelerometer readings |
| `read_gyro_data` | Gyro readings |
| `get_temperature` | Temperature reading |
| `roll` / `pitch` | Computed orientation angles |
| `_gyro_offset` | Gyro bias calibration offset |
| `_offset_config` | Offset configuration structure |
| `_read_byte` / `_write_byte` | Low-level I2C R/W |
| `_read_block` / `_read_u16` | Block reads |

The actual register address constants (e.g. what value `_ACCEL_CONFIG` is, the
QMI8658-specific register map vs MPU6050 map) are embedded in bytecode constant
tables — **Phase B required** to recover these.

### `homeostasis` balance algorithm

`homeostasis(True/False)` is a method on `MechDog`, tracked by `__balancing`
instance variable. `read_homeostasis_status()` returns running state. Uses
`__mpu` (QMI8658 IMU). From main app usage:

```python
doghw.homeostasis(True)
if not doghw.read_homeostasis_status():
    doghw.homeostasis(False)
```

The algorithm internals and any gain constants are in compiled bytecode.
**Phase B required.**

### Battery ADC divider

`Battery_power()` is in `Hiwonder.py`, calls into `Hiwonder_ADC` with `__adcp`
(ADC pin object). The voltage divider constant is embedded as a float literal
in the bytecode. **Phase B required.**

### `move(speed, rotation)` — confirmed

From main app source: `doghw.move(speed_int, rotation_int)`. Range observed in
source: speed from -40 (backward) to 120 (forward), rotation from -50 (right)
to +50 (left). 0 = stop.

## Phase B Status

Phase B (MicroPython frozen bytecode extraction and disassembly) has not been
performed. Tools required:

1. `python3 micropython/tools/mpy-tool.py -d <module.mpy>` — upstream
   MicroPython tool for bytecode disassembly.
2. Ghidra 11 + `ghidra-xtensa` processor module — for the IROM segment where
   any C extensions would live. Based on Phase A findings, the QMI8658, ADC,
   and homeostasis logic appear to be Python (not C extensions), which should
   make Phase B sufficient.
3. Frozen module extraction: the DROM segment at file offset 0x18, 303,668
   bytes. MicroPython frozen module tables are in `.rodata` under
   `mp_module_*` symbols. A community script (`mpy_unfreeze` or similar)
   can reconstruct individual `.mpy` files; expect to patch for MPY v6.

## Reproducible Commands

All commands run from `/Users/gwen/workspace/bark-buddy/stock_firmware_dump/`.

```bash
# Partition decode
python3 -c "
import struct
data = open('../firmware/stock_firmware.bin','rb').read()
MAGIC = 0x50AA; offset = 0x8000
while offset + 32 <= len(data):
    entry = data[offset:offset+32]
    magic,typ,sub,start,size = struct.unpack_from('<HBBII',entry)
    if magic != MAGIC: break
    name = entry[8:24].rstrip(b'\x00').decode('ascii','replace')
    print(f'{name} type={typ} sub={sub} start={start:#x} size={size:#x}')
    offset += 32
"

# App image info
esptool --chip esp32 image-info partitions/factory.bin

# String extraction
strings -o -t x partitions/factory.bin > source/factory.strings
```
