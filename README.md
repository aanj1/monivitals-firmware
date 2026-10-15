# MoniVitals — ESP32-C3 Super Mini + MAX30102

Firmware for **MAX30102** (I²C PPG / SpO₂) on **ESP32-C3 Super Mini**.  
Advertises **`MoniVitals-XXXX`** and speaks the same GATT as iOS MoniVitals app.

Advertises as **MoniVitals-XXXX**. GATT custom UUID base (shipping / TestFlight):

`425400XX-5441-4542-4F58-3B9A78563412`

| Char | UUID | Role |
|---|---|---|
| Service | `42540001-…` | Primary |
| Quality `[PI×0.1%, contact]` | `42540002-…` | Notify — byte0=PI, byte1=finger contact (authoritative) |
| PPG wave (LE int16 IR AC) | `42540006-…` | Notify |
| COMMAND | `42540007-…` | Write `[0x02,0x01]` start stream |
| Heart Rate | `180D` / `2A37` | Notify |
| SpO₂ (PLX) | `1822` / `2A5F` | Notify |

If MAX30102 is missing, BLE still advertises but **no simulated vitals** are sent.

Real-use behaviour:
- AGC keeps firm finger contact out of ADC saturation
- FIFO drained whenever connected (no overflow while app discovers services)
- HR/SpO₂ notified every ~2 s only when algorithm is valid
- No-contact → HR notify with contact bit clear so the app clears sticky tiles

## Signal processing & accuracy

`src/vitals_engine.c`:
- **Band-pass 0.5–5 Hz** (Butterworth HP@0.5 + LP@5, float RBJ biquads) on IR & RED
  before peak detection → robust HR under baseline wander / HF noise.
- **HR:** adaptive-threshold peak detect → median-of-5 intervals → EMA. Band 40–160.
- **SpO₂:** ratio-of-ratios `R = (rms(AC_red)/DC_red) / (rms(AC_ir)/DC_ir)` over a 2 s
  RMS window (stabler than instantaneous peaks) → table lookup.
- **PI:** AC envelope / DC.

**SpO₂ is INDICATIVE, not calibrated.** The `s_lut[]` R→SpO₂ table is a generic
transmissive curve, not fit to this board's LEDs/photodiode. To calibrate: log
`s_r_x100` vs a reference oximeter across 90–100%, refit `s_lut[]`, then trim with
`SPO2_CAL_OFFSET`. Procedure documented at the top of `vitals_engine.c`.

## Wiring (defaults)

| MAX30102 | ESP32-C3 Super Mini |
|---|---|
| VIN / VCC | **3.3 V** (not 5 V) |
| GND | GND |
| SDA | **GPIO 6** |
| SCL | **GPIO 7** |
| INT | GPIO 5 (optional) |

Defaults avoid strapping pins GPIO 8/9 (often labeled SDA/SCL on the board).  
To use 8/9 instead, set in `platformio.ini`:

```ini
-DSB_PIN_I2C_SDA=8
-DSB_PIN_I2C_SCL=9
```

Add 4.7k–10k pull-ups on SDA/SCL if your module doesn’t include them.

## Build & flash

```bash
cd ~/Downloads/monivitals-esp32c3-max30102
pio run
pio run -t upload
pio device monitor -b 115200
```

Hold **BOOT** while resetting if upload fails on Super Mini USB.

Expect: `MAX30102 online` (or `SIMULATE PPG`) then `advertising as MoniVitals-XXXX`.

## Pair with iOS

1. Flash & power the board.  
2. MoniVitals app → Connect → **MoniVitals-XXXX**.  
3. Place finger on the MAX30102 window for HR / SpO₂ / wave.
