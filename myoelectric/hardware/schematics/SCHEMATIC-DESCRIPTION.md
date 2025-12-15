# WIA EMG 2-Channel Hardware Schematic

## Overview

This document describes the circuit design for the WIA low-cost 2-channel EMG acquisition system.

## Block Diagram

```
                                    +3.3V
                                      │
┌─────────┐   ┌─────────┐   ┌─────────┴─────────┐   ┌─────────────┐
│         │   │  INA128 │   │    2nd Order      │   │             │
│Electrode├───┤  Inst   ├───┤   Butterworth     ├───┤  ESP32-S3   │
│  Pair   │   │  Amp    │   │   Bandpass Filter │   │    ADC      │
│         │   │ G=500   │   │   20-450Hz        │   │  (12-bit)   │
└─────────┘   └─────────┘   └───────────────────┘   └──────┬──────┘
                                                           │
                                                    ┌──────┴──────┐
                                                    │   BLE 5.0   │
                                                    │   or USB    │
                                                    └─────────────┘
```

## Channel 1 Circuit

### Instrumentation Amplifier (U2: INA128PA)

```
                    +3.3V
                      │
                    ┌─┴─┐
                    │   │ 100nF
                    └─┬─┘
                      │
    ┌─────────────────┼─────────────────┐
    │                 7                 │
    │    ┌───────────────────────┐      │
  1 │ IN-│                       │OUT 6 │
────┴────┤   U2: INA128PA        ├──────┴────> To Filter
  2 │ IN+│                       │REF 5 │
────┬────┤                       ├──────┬──── VREF (1.65V)
    │    │                       │      │
    │    └───────────────────────┘      │
    │                 4                 │
    │                 │                 │
    │               ┌─┴─┐               │
    │               │   │ 100nF         │
    │               └─┬─┘               │
    │                 │                 │
    │                GND                │
    │                                   │
    │           Rg = 100Ω               │
    │    ┌─────────────────────┐        │
    └────┤ 1              8    ├────────┘
         └─────────────────────┘

Gain = 1 + (50kΩ / Rg) = 1 + (50k / 100) = 501
```

### 2nd Order Bandpass Filter (20-450Hz)

```
High-Pass Section (fc = 20Hz):

IN ──┬── C1 ──┬── R1 ──┬───> To Low-Pass
     │  10nF  │   10k  │
     │        │        │
    1M       GND      GND
     │
    GND

Low-Pass Section (fc = 450Hz):

IN ────┬── R2 ──┬────> OUT
       │  3.5k  │
       │        │
      C2       C3
     10nF     10nF
       │        │
      GND      GND
```

## Notch Filter (50/60Hz)

Optional Twin-T notch filter for power line rejection:

```
           R      R
IN ──┬────/\/\/──┬──/\/\/──┬── OUT
     │           │         │
     C           │         C
     │         2R/R        │
    GND         │        GND
               2C
                │
               GND

For 50Hz: R = 3.2kΩ, C = 1µF
For 60Hz: R = 2.7kΩ, C = 1µF
```

## Power Supply

### Battery Charging (USB-C)

```
USB-C ────┬──── MCP73831 ────┬──── LiPo 3.7V
VBUS      │     Charger      │     500mAh
          │                  │
         GND               BATT+
```

### 3.3V Regulation

```
BATT+ ──── MCP1700-3302E ────┬──── +3.3V
              │              │
             GND           100µF
                             │
                            GND
```

### Reference Voltage (VREF = 1.65V)

```
+3.3V ──── R ────┬──── VREF (1.65V)
                 │
                 R
                 │
                GND

(R = R = 10kΩ for voltage divider)
Buffer with REF3030 for low impedance output
```

## ESP32-S3 Connections

| ESP32 Pin | Function | Connection |
|-----------|----------|------------|
| GPIO1 | ADC1_CH0 | EMG Channel 1 Output |
| GPIO2 | ADC1_CH1 | EMG Channel 2 Output |
| GPIO3 | Battery Monitor | Voltage Divider from BATT+ |
| GPIO4 | LED Status | Blue LED (BLE) |
| GPIO5 | LED Power | Red LED (Power) |
| GPIO6 | Button | Mode/Reset Switch |
| GPIO19 | USB D- | USB Data |
| GPIO20 | USB D+ | USB Data |

## PCB Layout Guidelines

1. **Ground Plane**: Use solid ground plane on bottom layer
2. **Analog/Digital Separation**: Keep analog and digital sections separate
3. **Trace Width**:
   - Power traces: 0.5mm minimum
   - Signal traces: 0.25mm
4. **Component Placement**: Place amplifiers close to input connectors
5. **Decoupling**: Place 100nF caps within 3mm of IC power pins
6. **Shield**: Consider shielded enclosure for noise rejection

## Component Specifications

### INA128PA Key Specs
- Input Offset Voltage: 50µV max
- CMRR: 120dB min
- Input Bias Current: 5nA
- Bandwidth: 1.3MHz (G=10)
- Noise: 8nV/√Hz

### ESP32-S3 ADC Specs
- Resolution: 12-bit (4096 levels)
- Sampling Rate: Up to 2 MSPS
- Input Range: 0-3.3V (with attenuation)
- DNL: ±1 LSB typical

## Signal Levels

| Stage | Min | Typical | Max |
|-------|-----|---------|-----|
| Electrode Input | -5mV | 0 | +5mV |
| After Amp (G=500) | -2.5V | 0 | +2.5V |
| After Filter + Offset | 0V | 1.65V | 3.3V |
| ADC Input | 0 | 2048 | 4095 |

## Testing Points

1. **TP1**: VREF (1.65V)
2. **TP2**: +3.3V Rail
3. **TP3**: Ch1 Amplifier Output
4. **TP4**: Ch2 Amplifier Output
5. **TP5**: Battery Voltage

---

*WIA Myoelectric Hardware - Schematic Description v1.0*
