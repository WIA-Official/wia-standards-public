# WIA EMG 2-Channel Hardware Specification

**Version:** 1.0.0
**Status:** Draft
**Target Cost:** < $50 USD

## 1. Overview

This specification defines a low-cost, open-source 2-channel EMG acquisition system for myoelectric prosthetic control. The design prioritizes accessibility, ease of construction, and compatibility with the WIA EMG Signal Standard.

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA EMG 2-Channel System                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐    ┌─────────────┐    ┌──────────────────────┐    │
│  │Electrodes│───▶│Instrumentation│───▶│   Analog Filters    │    │
│  │  (2 ch)  │    │  Amplifiers  │    │ HP: 20Hz  LP: 450Hz │    │
│  └──────────┘    │   INA128     │    └──────────┬───────────┘    │
│                  │   G = 500    │               │                │
│                  └─────────────┘               ▼                │
│                                         ┌──────────────┐        │
│  ┌──────────┐    ┌─────────────┐       │   ESP32-S3   │        │
│  │  LiPo    │───▶│   LDO       │───────│   12-bit ADC │        │
│  │ Battery  │    │  3.3V       │       │   BLE 5.0    │        │
│  └──────────┘    └─────────────┘       └──────────────┘        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 3. Performance Specifications

### 3.1 EMG Acquisition

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| Channels | 2 | Expandable to 8 |
| Sample Rate | 1000 Hz | Configurable 100-2000 Hz |
| Resolution | 12-bit | 4096 levels |
| Input Range | ±5 mV | At electrode |
| Gain | 500 V/V | Adjustable |
| Bandwidth | 20-450 Hz | 2nd order Butterworth |
| CMRR | > 100 dB | With INA128 |
| Input Impedance | > 100 MΩ | High impedance inputs |
| Noise | < 2 µV RMS | Referred to input |

### 3.2 Wireless Communication

| Parameter | Specification |
|-----------|---------------|
| Protocol | Bluetooth Low Energy 5.0 |
| Range | > 10 m |
| Latency | < 20 ms |
| Data Rate | 40 samples/packet @ 50 Hz |
| Connection Interval | 7.5-15 ms |

### 3.3 Power

| Parameter | Specification |
|-----------|---------------|
| Battery | LiPo 3.7V 500mAh |
| Operating Time | > 8 hours |
| Charging | USB-C, 500mA |
| Charge Time | ~2 hours |
| Standby Current | < 50 µA |
| Active Current | < 50 mA |

### 3.4 Physical

| Parameter | Specification |
|-----------|---------------|
| Dimensions | 70 × 45 × 25 mm |
| Weight | < 50 g (with battery) |
| Enclosure | 3D printed PLA/PETG |
| Mounting | Elastic arm strap |
| Connectors | 3.5mm audio jacks |
| IP Rating | IP20 (indoor use) |

## 4. Component Selection

### 4.1 Microcontroller: ESP32-S3-DevKitC-1

**Rationale:**
- Built-in BLE 5.0 and WiFi
- Dual-core 240 MHz processor
- 12-bit ADC with 2 MSPS
- USB-C programming
- Low cost (~$7)
- Arduino and PlatformIO support

**Key Features:**
- 512KB SRAM
- 8MB Flash
- Native USB
- RTC and deep sleep

### 4.2 Instrumentation Amplifier: INA128PA

**Rationale:**
- High CMRR (120 dB)
- Low noise (8 nV/√Hz)
- Wide supply range (±2.25V to ±18V)
- Single resistor gain set
- Industry standard for biosignal acquisition

**Configuration:**
- Gain = 1 + 50kΩ/Rg
- Rg = 100Ω for G = 501
- Single 3.3V supply with virtual ground

### 4.3 Voltage Reference: REF3030

**Rationale:**
- 3.0V output (stable ADC reference)
- Low noise (15 µVp-p)
- Low current (< 50 µA)
- SOT-23 package

### 4.4 Power Management

**LDO Regulator: MCP1700-3302E**
- 3.3V output
- 250mA max current
- Low dropout (178 mV @ 250 mA)
- Low quiescent current (1.6 µA)

**Battery Charger: MCP73831**
- Single-cell LiPo charging
- 500mA charge current
- Thermal regulation
- Charge status LED

## 5. Analog Front-End Design

### 5.1 Input Stage

```
Electrode (+)  ────┬──── 1MΩ ────┬──── INA128 IN+
                   │             │
                  GND          10nF
                                 │
                                GND

Electrode (-)  ────┬──── 1MΩ ────┬──── INA128 IN-
                   │             │
                  GND          10nF
                                 │
                                GND
```

- 1MΩ input bias resistors
- 10nF capacitors for RF filtering
- Input protection with TVS diodes (optional)

### 5.2 Gain Stage

The INA128 is configured for G = 500:
- Rg = 50kΩ / (G - 1) = 50kΩ / 499 ≈ 100Ω

### 5.3 Highpass Filter (fc = 20 Hz)

First-order RC highpass:
- R = 10kΩ
- C = 820nF (use 1µF for 16 Hz cutoff)
- fc = 1 / (2πRC) ≈ 19.4 Hz

### 5.4 Lowpass Filter (fc = 450 Hz)

Second-order Sallen-Key:
- R1 = R2 = 3.5kΩ
- C1 = C2 = 100nF
- Q = 0.707 (Butterworth)
- fc = 1 / (2πRC) ≈ 455 Hz

### 5.5 DC Offset

Virtual ground at 1.65V (mid-rail):
- Voltage divider: 2 × 10kΩ
- Buffered with REF3030 for low impedance

## 6. Firmware Architecture

### 6.1 Main Components

```
┌─────────────────────────────────────────────────────────────┐
│                      Firmware Architecture                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │  ADC Driver  │  │  BLE Stack   │  │   USB CDC    │       │
│  │  Timer ISR   │  │  NimBLE      │  │   Serial     │       │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │
│         │                 │                 │               │
│         ▼                 ▼                 ▼               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Sample Buffer Manager                    │   │
│  │              (Ring Buffer, 20 samples)               │   │
│  └──────────────────────────────────────────────────────┘   │
│         │                                                   │
│         ▼                                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                 Packet Formatter                      │   │
│  │            (WIA EMG Binary Protocol)                  │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Sampling

- Hardware timer at 1 kHz
- ISR reads both ADC channels
- Double buffering for continuous streaming
- DMA transfer (optional for higher rates)

### 6.3 BLE Protocol

**Service UUID:** `12345678-1234-5678-1234-56789abcdef0`

| Characteristic | UUID Suffix | Properties | Description |
|---------------|-------------|------------|-------------|
| EMG Data | def1 | Read, Notify | EMG sample packets |
| Command | def2 | Write | Control commands |
| Status | def3 | Read, Notify | Device status |

**Data Packet Format:**
```
Offset  Size  Description
0       4     Timestamp (ms)
4       80    Samples (20 × 2 channels × 2 bytes)
84      1     Sample count
```

## 7. PCB Design Guidelines

### 7.1 Layer Stack

- 2-layer PCB
- Top: Signal and components
- Bottom: Ground plane

### 7.2 Layout Rules

1. **Analog/Digital Separation**: Keep analog section isolated from digital
2. **Ground Plane**: Continuous ground plane under analog components
3. **Decoupling**: 100nF caps within 3mm of IC power pins
4. **Trace Width**: 0.25mm signal, 0.5mm power
5. **Via Stitching**: Ground vias around analog section

### 7.3 Design Files

- Format: KiCad 7.0+
- Gerber: RS-274X
- Drill: Excellon

## 8. Assembly Guide

### 8.1 Required Tools

- Soldering iron (temperature controlled)
- Solder (0.5mm diameter, lead-free)
- Flux
- Tweezers
- Multimeter
- USB-C cable
- Computer with PlatformIO

### 8.2 Assembly Order

1. SMD components (ICs, passives)
2. Through-hole components (connectors)
3. ESP32 module
4. Battery connector
5. Enclosure assembly

### 8.3 Testing Procedure

1. **Power Test**: Verify 3.3V rail
2. **Reference Test**: Check 1.65V mid-rail
3. **Amplifier Test**: Input sine wave, verify gain
4. **ADC Test**: Upload firmware, verify readings
5. **BLE Test**: Connect with phone app

## 9. Calibration

### 9.1 Zero Offset Calibration

1. Short electrode inputs
2. Record ADC values for 10 seconds
3. Calculate mean offset
4. Store in flash memory

### 9.2 Gain Calibration

1. Apply known 1 mV signal
2. Verify output = 1 mV × Gain
3. Calculate correction factor
4. Store in flash memory

## 10. Safety Considerations

### 10.1 Electrical Safety

- Battery protection circuit (over-discharge, over-charge)
- Current limiting on all outputs
- ESD protection on connectors
- No direct connection to mains power

### 10.2 Biocompatibility

- Use medical-grade electrodes
- Follow electrode manufacturer guidelines
- Replace electrodes as recommended

### 10.3 Regulatory Notes

This is a research/educational device. For clinical use:
- Consult local medical device regulations
- Consider IEC 60601-1 compliance
- Implement appropriate risk management

## 11. Bill of Materials Summary

| Category | Cost (USD) |
|----------|------------|
| MCU (ESP32-S3) | $7 |
| Amplifiers (2× INA128) | $9 |
| Passive Components | $3 |
| Power Management | $3 |
| Connectors | $3 |
| PCB | $2 |
| Electrodes (6×) | $3 |
| Cables (2×) | $5 |
| Battery | $4 |
| Enclosure | $5 |
| Strap | $2 |
| **Total** | **~$46** |

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

*WIA Myoelectric Hardware Specification v1.0.0*
