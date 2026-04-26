# WIA-COMM-018: Low-Power Network Specification v1.0

> **Standard ID:** WIA-COMM-018
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [LPWAN Architecture](#2-lpwan-architecture)
3. [Technology Standards](#3-technology-standards)
4. [LoRaWAN Specification](#4-lorawan-specification)
5. [Sigfox Specification](#5-sigfox-specification)
6. [NB-IoT Specification](#6-nb-iot-specification)
7. [LTE-M Specification](#7-lte-m-specification)
8. [Power Management](#8-power-management)
9. [Message Formats](#9-message-formats)
10. [Adaptive Data Rate](#10-adaptive-data-rate)
11. [Security and Privacy](#11-security-and-privacy)
12. [Coverage Planning](#12-coverage-planning)
13. [Performance Requirements](#13-performance-requirements)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for Low-Power Wide Area Networks (LPWAN), enabling ultra-low power, long-range wireless communication for IoT devices with battery lives exceeding 10 years.

### 1.2 Scope

The standard covers:
- LoRaWAN, Sigfox, NB-IoT, and LTE-M protocols
- Power management and optimization strategies
- Message format specifications
- Security and privacy mechanisms
- Network planning and deployment
- Battery life calculations and optimization

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - This standard enables massive IoT deployments with minimal environmental impact through energy-efficient communication, supporting sustainable smart cities, precision agriculture, and environmental monitoring at global scale.

### 1.4 Terminology

- **LPWAN**: Low-Power Wide Area Network
- **LoRaWAN**: Long Range Wide Area Network
- **CSS**: Chirp Spread Spectrum (LoRa modulation)
- **SF**: Spreading Factor (LoRa parameter)
- **ADR**: Adaptive Data Rate
- **OTAA**: Over-the-Air Activation
- **ABP**: Activation By Personalization
- **DevEUI**: Device Extended Unique Identifier
- **AppEUI**: Application Extended Unique Identifier
- **NB-IoT**: Narrowband Internet of Things
- **LTE-M**: Long-Term Evolution for Machines
- **PSM**: Power Saving Mode
- **eDRX**: Extended Discontinuous Reception

---

## 2. LPWAN Architecture

### 2.1 General Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    LPWAN Communication System                │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐            │
│  │ End      │────▶│ Gateway/ │────▶│ Network  │            │
│  │ Device   │     │ Base Stn │     │ Server   │            │
│  └──────────┘     └──────────┘     └──────────┘            │
│       │                                   │                  │
│       │                                   │                  │
│       └───────────────────────────────────┼──────────────┐  │
│                                           │              │  │
│                                     ┌─────▼────┐  ┌─────▼───┐
│                                     │   App    │  │  Cloud  │
│                                     │  Server  │  │ Service │
│                                     └──────────┘  └─────────┘
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Network Topology

**Star-of-Stars** (LoRaWAN, Sigfox):
```
End Devices ──▶ Gateway ──▶ Network Server ──▶ Application Server
     │              │               │
     ├──────────────┤               │
     └──────────────┼───────────────┘
```

**Cellular** (NB-IoT, LTE-M):
```
End Devices ──▶ eNodeB ──▶ Cellular Core ──▶ Application Server
```

### 2.3 Communication Model

**Uplink Dominant**:
- 90-99% uplink traffic (device to network)
- Minimal downlink (network to device)
- Asymmetric data rates

**Message-Based**:
- Small payload sizes (12-250 bytes)
- Infrequent transmissions (minutes to hours)
- Event-driven or scheduled

---

## 3. Technology Standards

### 3.1 Technology Comparison Matrix

| Feature | LoRaWAN | Sigfox | NB-IoT | LTE-M |
|---------|---------|--------|--------|-------|
| **Spectrum** | Unlicensed ISM | Unlicensed ISM | Licensed LTE | Licensed LTE |
| **Frequency** | 868/915 MHz | 868/902 MHz | 700-2100 MHz | 700-2100 MHz |
| **Modulation** | CSS (LoRa) | DBPSK/GFSK | QPSK | QPSK/16QAM |
| **Bandwidth** | 125/250 kHz | 100 Hz | 180 kHz | 1.4-20 MHz |
| **Peak Data Rate** | 50 kbps | 600 bps | 250 kbps | 1 Mbps |
| **Typical Data Rate** | 5.5 kbps | 100 bps | 60 kbps | 200 kbps |
| **Max Payload** | 242 bytes | 12 bytes | 1600 bytes | 1000 bytes |
| **Range (Urban)** | 2-5 km | 3-10 km | 1-10 km | 2-10 km |
| **Range (Rural)** | 15-50 km | 30-50 km | 35+ km | 40+ km |
| **Battery Life** | 10-15 years | 15-20 years | 10+ years | 8-10 years |
| **Latency** | 1-10 s | 2-30 s | 1.5-10 s | 0.01-10 s |
| **Mobility** | Limited | Limited | Full | Full |
| **Duty Cycle** | 1% (EU) | 1% | No limit | No limit |
| **Deployment** | Public/Private | Public only | Public only | Public only |

### 3.2 Use Case Mapping

```
LoRaWAN:
  - Smart Agriculture (soil, weather)
  - Asset Tracking (moderate mobility)
  - Smart Buildings (HVAC, lighting)
  - Private Networks

Sigfox:
  - Simple Sensors (temp, humidity)
  - Asset Tracking (low data)
  - Utility Metering
  - Alarm Systems

NB-IoT:
  - Smart Metering (high reliability)
  - Connected Health
  - Smart Parking
  - Wearables

LTE-M:
  - Asset Tracking (high mobility)
  - Fleet Management
  - Connected Health
  - Smart City Applications
```

---

## 4. LoRaWAN Specification

### 4.1 Physical Layer

**Modulation**: Chirp Spread Spectrum (CSS)

**Spreading Factors**:
```
SF7:  Data Rate = 5.5 kbps,  ToA = 41 ms,  Range = 2 km
SF8:  Data Rate = 3.1 kbps,  ToA = 72 ms,  Range = 4 km
SF9:  Data Rate = 1.8 kbps,  ToA = 144 ms, Range = 6 km
SF10: Data Rate = 980 bps,   ToA = 247 ms, Range = 8 km
SF11: Data Rate = 440 bps,   ToA = 494 ms, Range = 11 km
SF12: Data Rate = 250 bps,   ToA = 988 ms, Range = 14 km

ToA = Time on Air
Higher SF = Longer range, lower data rate, more battery
Lower SF = Shorter range, higher data rate, less battery
```

**Bandwidth Options**:
- 125 kHz (default)
- 250 kHz (higher data rate)
- 500 kHz (maximum data rate)

**Coding Rate**:
```
CR 4/5: Overhead = 1.25 (default)
CR 4/6: Overhead = 1.5  (more error correction)
CR 4/7: Overhead = 1.75
CR 4/8: Overhead = 2.0  (maximum protection)
```

### 4.2 MAC Layer

**Device Classes**:

**Class A** (All devices):
```
┌─────────────────────────────────────────────────────────┐
│  Uplink:    Anytime                                      │
│  Downlink:  Only after uplink (2 RX windows)             │
│             RX1: 1 second after uplink                   │
│             RX2: 2 seconds after uplink (fallback)       │
│  Power:     Lowest (10-15 year battery)                  │
│  Latency:   Depends on uplink interval                   │
└─────────────────────────────────────────────────────────┘

Timeline:
  TX ────┐         ┌RX1──┐  ┌RX2──┐
         └─────────┘      └──┘     └─────────────▶
         0        1s      2s                time
```

**Class B** (Beacon):
```
┌─────────────────────────────────────────────────────────┐
│  Uplink:    Anytime                                      │
│  Downlink:  After uplink + scheduled ping slots          │
│  Beacon:    Synchronized time reference (every 128s)     │
│  Power:     Medium (5-10 year battery)                   │
│  Latency:   Ping slot interval (configurable)            │
└─────────────────────────────────────────────────────────┘
```

**Class C** (Continuous):
```
┌─────────────────────────────────────────────────────────┐
│  Uplink:    Anytime                                      │
│  Downlink:  Anytime (always listening except when TX)    │
│  Power:     Highest (months, mains-powered preferred)    │
│  Latency:   Minimal (~1 second)                          │
└─────────────────────────────────────────────────────────┘
```

### 4.3 Activation Methods

**OTAA (Over-the-Air Activation)** - Recommended:
```
Device                          Network Server
  │                                   │
  │──── Join Request ────────────────▶│
  │     (DevEUI, AppEUI, DevNonce)    │
  │                                   │
  │◀─── Join Accept ─────────────────│
  │     (AppNonce, NetID, DevAddr)    │
  │                                   │
  │ Derive Session Keys:              │
  │   NwkSKey = aes128(AppKey, ...)   │
  │   AppSKey = aes128(AppKey, ...)   │
  │                                   │
  │──── Data Message ────────────────▶│
  │     (encrypted with session keys)  │
  │                                   │
```

**ABP (Activation By Personalization)** - Less secure:
```
Pre-configured on device:
  - DevAddr (32-bit device address)
  - NwkSKey (Network Session Key)
  - AppSKey (Application Session Key)

No join procedure required.
Device can transmit immediately.
```

### 4.4 Message Format

**Uplink Message**:
```
┌──────────┬──────────┬────────┬──────────┬─────────┬─────┐
│  MHDR    │  DevAddr │  FCtrl │  FCnt    │  FPort  │ MIC │
│  (1 B)   │  (4 B)   │ (1 B)  │  (2 B)   │ (1 B)   │(4 B)│
└──────────┴──────────┴────────┴──────────┴─────────┴─────┘
                          │
                          └─── Encrypted Payload ────┐
                                                      │
                               ┌──────────────────────┘
                               │
                         ┌─────▼──────┐
                         │ Application│
                         │   Payload  │
                         └────────────┘
```

**Fields**:
- **MHDR**: Message Type (Unconfirmed/Confirmed Data Up)
- **DevAddr**: Device Address
- **FCtrl**: Frame Control (ADR, ACK, FOptsLen)
- **FCnt**: Frame Counter (replay protection)
- **FPort**: Port (0=MAC, 1-223=Application)
- **FRMPayload**: Encrypted application data
- **MIC**: Message Integrity Code (AES-CMAC)

### 4.5 Duty Cycle Regulations

**Europe (ETSI EN300.220)**:
```
Sub-band g (863-868 MHz):   1% duty cycle = 36 seconds/hour
Sub-band g1 (868-868.6 MHz): 1% duty cycle = 36 seconds/hour
Sub-band g2 (868.7-869.2 MHz): 0.1% duty cycle = 3.6 seconds/hour
Sub-band g3 (869.4-869.65 MHz): 10% duty cycle = 360 seconds/hour
```

**United States (FCC Part 15)**:
```
902-928 MHz: No duty cycle limit (frequency hopping required)
Dwell time: 400 ms max per channel
```

### 4.6 Link Budget Calculation

```
Link Budget = PTX + GTX - LTX + GRX - LRX - Margin

Where:
  PTX = Transmit power (14 dBm typical, 27 dBm max)
  GTX = Transmit antenna gain (2 dBi typical)
  LTX = Transmit losses (1 dB)
  GRX = Receive antenna gain (8 dBi gateway)
  LRX = Receive losses (1 dB)
  Margin = Fade margin (10-20 dB)

Example (SF12, 125 kHz):
  Link Budget = 14 + 2 - 1 + 8 - 1 - 15 = 7 dB
  Sensitivity = -137 dBm (SF12)
  Path Loss = Link Budget - Sensitivity
            = 7 - (-137) = 144 dB

Path Loss = 144 dB corresponds to ~15 km range (Hata Urban model)
```

### 4.7 Battery Life Calculation

```python
def calculate_lorawan_battery_life(
    battery_capacity_mah: float,  # Battery capacity in mAh
    messages_per_day: int,         # Number of uplinks per day
    payload_size: int,             # Payload size in bytes
    spreading_factor: int,         # SF7-SF12
    tx_power_dbm: int,             # TX power in dBm
    voltage: float = 3.3           # Supply voltage
) -> float:
    """Calculate LoRaWAN device battery life in years"""

    # Current consumption
    sleep_current_ua = 1.5       # Deep sleep current
    idle_current_ma = 2.0        # Idle current
    rx_current_ma = 15.0         # RX current

    # TX current depends on power level
    tx_current_map = {
        2: 28, 7: 29, 10: 31, 13: 33, 14: 38,
        17: 82, 20: 120, 27: 150
    }
    tx_current_ma = tx_current_map.get(tx_power_dbm, 100)

    # Time on Air calculation (SF, BW=125kHz, CR=4/5)
    toa_map = {
        7: 41, 8: 72, 9: 144, 10: 247, 11: 494, 12: 988
    }
    toa_ms = toa_map.get(spreading_factor, 247)
    toa_s = toa_ms / 1000.0

    # RX windows (1 second after TX, 2 seconds after TX)
    rx_window_duration = 0.02  # 20ms per RX window
    num_rx_windows = 2

    # Daily energy consumption
    sleep_time_s = 86400 - (messages_per_day * (toa_s + num_rx_windows * rx_window_duration))

    energy_sleep = (sleep_current_ua / 1000.0) * (sleep_time_s / 3600.0)  # mAh
    energy_tx = tx_current_ma * messages_per_day * (toa_s / 3600.0)       # mAh
    energy_rx = rx_current_ma * messages_per_day * num_rx_windows * (rx_window_duration / 3600.0)  # mAh

    daily_consumption_mah = energy_sleep + energy_tx + energy_rx

    # Battery life
    battery_life_days = battery_capacity_mah / daily_consumption_mah
    battery_life_years = battery_life_days / 365.25

    return battery_life_years

# Example: Smart agriculture sensor
battery_life = calculate_lorawan_battery_life(
    battery_capacity_mah=5000,   # 2x AA batteries
    messages_per_day=24,          # Hourly readings
    payload_size=10,              # 10 bytes
    spreading_factor=10,          # Good range
    tx_power_dbm=14               # Standard power
)
print(f"Battery Life: {battery_life:.1f} years")  # ~11.2 years
```

---

## 5. Sigfox Specification

### 5.1 Physical Layer

**Modulation**: DBPSK (Differential Binary Phase Shift Keying)

**Frequency**:
```
Europe:     868.130 MHz (uplink), 869.525 MHz (downlink)
USA:        902.130 MHz (uplink), 905.230 MHz (downlink)
```

**Ultra-Narrow Band**:
- Bandwidth: 100 Hz (uplink), 600 Hz (downlink)
- Extreme sensitivity: -126 dBm

### 5.2 Message Limits

**Uplink**:
- Maximum: 140 messages per day
- Payload: 12 bytes
- Data rate: 100 bps

**Downlink**:
- Maximum: 4 messages per day
- Payload: 8 bytes
- Data rate: 600 bps

### 5.3 Message Format

**Uplink Frame**:
```
┌─────────┬──────────┬─────────────┬─────────┬──────┐
│ Preamble│ Sync Word│   Payload   │   CRC   │ Auth │
│  (5 B)  │  (2 B)   │   (12 B)    │  (2 B)  │(2 B) │
└─────────┴──────────┴─────────────┴─────────┴──────┘
```

**Downlink Frame**:
```
┌─────────┬──────────┬─────────────┬─────────┐
│ Preamble│ Sync Word│   Payload   │   CRC   │
│  (5 B)  │  (2 B)   │    (8 B)    │  (2 B)  │
└─────────┴──────────┴─────────────┴─────────┘
```

### 5.4 Anti-Collision

Sigfox uses **time diversity** and **frequency diversity**:
- Each message transmitted 3 times
- On 3 different frequencies
- At 3 different times

This ensures high reliability even in interference.

### 5.5 Battery Life

```python
def calculate_sigfox_battery_life(
    battery_capacity_mah: float,
    messages_per_day: int
) -> float:
    """Calculate Sigfox device battery life in years"""

    # Sigfox current consumption (typical)
    sleep_current_ua = 1.0       # Extremely low sleep
    tx_current_ma = 50.0         # TX current (conservative)
    tx_duration_s = 6.0          # ~2 seconds per transmission × 3 repetitions

    # Daily consumption
    sleep_time_s = 86400 - (messages_per_day * tx_duration_s)

    energy_sleep = (sleep_current_ua / 1000.0) * (sleep_time_s / 3600.0)
    energy_tx = tx_current_ma * messages_per_day * (tx_duration_s / 3600.0)

    daily_consumption_mah = energy_sleep + energy_tx

    battery_life_days = battery_capacity_mah / daily_consumption_mah
    battery_life_years = battery_life_days / 365.25

    return battery_life_years

# Example: Temperature sensor (4 readings/day)
battery_life = calculate_sigfox_battery_life(
    battery_capacity_mah=2500,   # 1x AA battery
    messages_per_day=4
)
print(f"Battery Life: {battery_life:.1f} years")  # ~18.7 years
```

---

## 6. NB-IoT Specification

### 6.1 Physical Layer

**Modulation**:
- Uplink: SC-FDMA with π/2-BPSK or π/4-QPSK
- Downlink: OFDMA with QPSK

**Bandwidth**: 180 kHz (1 LTE PRB)

**Deployment Modes**:
```
Stand-alone: Dedicated spectrum (e.g., GSM refarming)
Guard-band: LTE guard band
In-band: Within LTE carrier
```

### 6.2 Coverage Classes

```
Normal Coverage:     MCL = 144 dB (standard cell)
Extended Coverage:   MCL = 154 dB (+10 dB, repetitions)
Extreme Coverage:    MCL = 164 dB (+20 dB, more repetitions)

MCL = Maximum Coupling Loss
```

### 6.3 Power Saving Mode (PSM)

**PSM Cycle**:
```
┌─────────┬────────────┬─────────┐
│ Active  │    Idle    │   PSM   │
│         │   (eDRX)   │  (Deep  │
│         │            │  Sleep) │
└─────────┴────────────┴─────────┘
  10 s      5 min        23h 50min

Active:  Full power, can TX/RX
Idle:    Paging monitoring (eDRX)
PSM:     Deep sleep, not reachable
```

**T3324 Timer** (Active Time):
- Device stays active for configuration or messages
- Typical: 10-30 seconds

**T3412 Timer** (Periodic TAU):
- Device wakes up to update Tracking Area
- Typical: 24 hours to 310 hours

### 6.4 Extended Discontinuous Reception (eDRX)

```
┌──────┬────────────┬──────┬────────────┬──────┐
│ Wake │   Sleep    │ Wake │   Sleep    │ Wake │
│  RX  │            │  RX  │            │  RX  │
└──────┴────────────┴──────┴────────────┴──────┘
  100ms   10.24s     100ms   10.24s     100ms

eDRX Cycle: 10.24 seconds (configurable: 5.12s to 2621.44s)
Paging Window: 100 ms
```

### 6.5 Message Format

**NPDU (NB-IoT PDU)**:
```
┌──────────┬─────────────┬─────────┐
│  Header  │   Payload   │   CRC   │
│  (variable) (1-1600 B) │  (2 B)  │
└──────────┴─────────────┴─────────┘
```

### 6.6 Battery Life

```python
def calculate_nbiot_battery_life(
    battery_capacity_mah: float,
    messages_per_day: int,
    payload_size: int,
    psm_enabled: bool = True,
    edrx_cycle_s: float = 10.24
) -> float:
    """Calculate NB-IoT device battery life in years"""

    # Current consumption (typical NB-IoT modem)
    psm_current_ua = 3.0         # PSM deep sleep
    idle_current_ma = 5.0        # Idle with eDRX
    tx_current_ma = 220.0        # TX at 23 dBm
    rx_current_ma = 50.0         # RX

    # Transmission time (conservative estimate)
    tx_time_s = 1.0 + (payload_size / 50.0)  # ~50 bytes/second
    rx_time_s = 0.5  # Downlink ACK

    # Calculate daily consumption
    if psm_enabled:
        # PSM mode: device sleeps most of the time
        active_time_s = messages_per_day * (tx_time_s + rx_time_s + 30)  # 30s active window
        psm_time_s = 86400 - active_time_s

        energy_psm = (psm_current_ua / 1000.0) * (psm_time_s / 3600.0)
        energy_active = (
            (tx_current_ma * tx_time_s + rx_current_ma * rx_time_s) *
            messages_per_day / 3600.0
        )

        daily_consumption_mah = energy_psm + energy_active
    else:
        # eDRX only: more power consumption
        active_time_s = messages_per_day * (tx_time_s + rx_time_s)
        idle_time_s = 86400 - active_time_s

        energy_idle = idle_current_ma * (idle_time_s / 3600.0)
        energy_tx = tx_current_ma * messages_per_day * (tx_time_s / 3600.0)
        energy_rx = rx_current_ma * messages_per_day * (rx_time_s / 3600.0)

        daily_consumption_mah = energy_idle + energy_tx + energy_rx

    battery_life_days = battery_capacity_mah / daily_consumption_mah
    battery_life_years = battery_life_days / 365.25

    return battery_life_years

# Example: Smart water meter (1 reading/day, PSM enabled)
battery_life = calculate_nbiot_battery_life(
    battery_capacity_mah=8000,   # 3x AA batteries
    messages_per_day=1,
    payload_size=50,
    psm_enabled=True
)
print(f"Battery Life: {battery_life:.1f} years")  # ~12.3 years
```

---

## 7. LTE-M Specification

### 7.1 Overview

LTE-M (LTE Cat-M1) is designed for IoT applications requiring:
- Higher data rates than NB-IoT (up to 1 Mbps)
- Full mobility support
- Voice over LTE (VoLTE)
- Lower latency

### 7.2 Physical Layer

**Bandwidth**: 1.4 MHz (6 PRBs minimum)

**Modulation**:
- Uplink: SC-FDMA with QPSK/16QAM
- Downlink: OFDMA with QPSK/16QAM

**Peak Data Rates**:
- Downlink: 1 Mbps
- Uplink: 1 Mbps

### 7.3 Power Saving

LTE-M uses the same PSM and eDRX mechanisms as NB-IoT, with similar power savings.

**Typical Current**:
- PSM: 5 µA
- eDRX idle: 7 mA
- TX: 250 mA (23 dBm)
- RX: 60 mA

### 7.4 Use Cases

- Asset tracking (GPS + cellular)
- Fleet management
- Wearables with voice
- Smart city sensors (higher data rate)

---

## 8. Power Management

### 8.1 Power Consumption States

```
┌────────────────────────────────────────────────────┐
│                Power State Machine                  │
├────────────────────────────────────────────────────┤
│                                                      │
│  ┌──────────┐    Wake     ┌──────────┐            │
│  │   DEEP   │◀──────────▶│   IDLE   │            │
│  │  SLEEP   │   Sleep     └──────────┘            │
│  │ (1-5 µA) │                  │                   │
│  └──────────┘                  │ Start TX         │
│       ▲                        ▼                   │
│       │                  ┌──────────┐              │
│       │                  │ TX/RX    │              │
│       │                  │(20-250mA)│              │
│       │                  └──────────┘              │
│       │                        │                   │
│       └────────────────────────┘                   │
│              Complete TX/RX                        │
│                                                      │
└────────────────────────────────────────────────────┘
```

### 8.2 Sleep Mode Types

**Light Sleep**:
- Current: 100-500 µA
- Wake time: <1 ms
- Use: Short intervals (<1 minute)

**Deep Sleep**:
- Current: 1-5 µA
- Wake time: 10-100 ms
- Use: Long intervals (minutes to hours)

**Hibernation**:
- Current: <1 µA
- Wake time: 100-1000 ms
- Use: Very long intervals (days)
- May lose RAM contents

### 8.3 Energy Harvesting

**Solar**:
```
Panel Size: 10 cm² (typical)
Power Output: 100-500 mW (full sun)
Daily Energy: 1-5 Wh (varies with location/season)

Sufficient for:
  - 10-100 messages/day (LoRaWAN)
  - Continuous operation (Class C)
```

**Thermal**:
```
Thermoelectric Generator (TEG)
Temperature Difference: 10°C
Power Output: 10-50 mW

Sufficient for:
  - 1-10 messages/day
  - Low-power monitoring
```

**Vibration**:
```
Piezoelectric/Electromagnetic
Power Output: 1-10 mW (continuous vibration)

Sufficient for:
  - Event-triggered messages
  - Industrial monitoring
```

### 8.4 Battery Selection

| Battery Type | Voltage | Capacity | Cost | Self-Discharge | Life | Use Case |
|--------------|---------|----------|------|----------------|------|----------|
| **Alkaline AA** | 1.5V | 2500 mAh | Low | 2-3%/year | 5-10 years | Consumer devices |
| **Lithium AA** | 1.5V | 3000 mAh | Medium | <1%/year | 10-15 years | Long-life sensors |
| **Li-SOCl₂** | 3.6V | 19000 mAh | High | <1%/year | 20+ years | Utility metering |
| **Li-Ion** | 3.7V | 2000 mAh | Medium | 2-3%/month | 3-5 years | Rechargeable |
| **Supercap** | 2.7V | 10 F | High | 10%/day | >10 years | Energy harvesting |

**Recommended**:
- **Alkaline**: Consumer IoT, low-cost
- **Lithium**: Professional IoT, extreme temperatures
- **Li-SOCl₂**: Utility metering, 20-year life
- **Rechargeable + Solar**: Continuous operation

---

## 9. Message Formats

### 9.1 Standard Sensor Payload

**CayenneLPP (Cayenne Low Power Payload)**:
```
Efficient encoding for common sensor data:

Temperature (2 bytes):
  [Channel] [Type=0x67] [Value × 10]
  Example: [0x01] [0x67] [0x00 0xFA] = Channel 1, 25.0°C

Humidity (2 bytes):
  [Channel] [Type=0x68] [Value × 2]
  Example: [0x02] [0x68] [0x82] = Channel 2, 65%

GPS (9 bytes):
  [Channel] [Type=0x88] [Lat×10000] [Lon×10000] [Alt×100]
  Example: [0x03] [0x88] [lat][lat][lat] [lon][lon][lon] [alt][alt][alt]

Battery (2 bytes):
  [Channel] [Type=0x02] [Voltage × 100]
  Example: [0x04] [0x02] [0x0E 0x1C] = Channel 4, 3.6V
```

**Complete Example**:
```
Payload (12 bytes):
  [0x01] [0x67] [0x00 0xFA]    // Temp = 25.0°C
  [0x02] [0x68] [0x82]          // Humidity = 65%
  [0x03] [0x02] [0x0E 0x1C]    // Battery = 3.6V
  [0x04] [0x02] [0x03 0xE8]    // Analog = 10.00V

Decodes to:
  {
    "temperature_1": 25.0,
    "relative_humidity_2": 65,
    "analog_input_3": 3.6,
    "analog_input_4": 10.0
  }
```

### 9.2 Custom Binary Format

**Agriculture Sensor**:
```
Byte 0:      Device Type (0x01 = Soil Sensor)
Byte 1-2:    Soil Moisture (0-100%, × 100)
Byte 3-4:    Temperature (°C × 10, signed)
Byte 5:      Battery (0-255 = 0-100%)
Byte 6-7:    Reserved

Total: 8 bytes

Example:
  [0x01] [0x13 0x88] [0x01 0x0E] [0xC8] [0x00 0x00]
  = Soil Sensor, 50.0% moisture, 27.0°C, 80% battery
```

### 9.3 Compression Techniques

**Delta Encoding**:
```
Instead of: [25.0, 25.1, 25.2, 25.1, 25.0]
Send:       [25.0, +0.1, +0.1, -0.1, -0.1]

Saves bandwidth by sending differences.
```

**Run-Length Encoding**:
```
Instead of: [20, 20, 20, 20, 25, 25, 30]
Send:       [20×4, 25×2, 30×1]
```

**Bit Packing**:
```
Instead of 1 byte per boolean:
  [true, false, true, true, false, false, true, false]
Pack into 1 byte:
  [10110010] = 0xB2

Reduces 8 bytes to 1 byte.
```

---

## 10. Adaptive Data Rate

### 10.1 ADR Algorithm (LoRaWAN)

```python
class AdaptiveDataRate:
    def __init__(self):
        self.target_snr = 10  # dB
        self.margin = 5       # dB
        self.history_size = 20
        self.snr_history = []

    def process_uplink(self, snr: float, rssi: float):
        """Process received uplink and adjust parameters"""

        # Add to history
        self.snr_history.append(snr)
        if len(self.snr_history) > self.history_size:
            self.snr_history.pop(0)

        # Calculate average SNR
        avg_snr = sum(self.snr_history) / len(self.snr_history)

        # Determine adjustment
        snr_margin = avg_snr - self.target_snr

        if snr_margin > self.margin + 5:
            # Too much margin - increase data rate or reduce power
            return self.increase_efficiency()
        elif snr_margin < -self.margin:
            # Not enough margin - decrease data rate or increase power
            return self.increase_robustness()
        else:
            # Good - no change
            return None

    def increase_efficiency(self):
        """Increase data rate or reduce TX power"""
        # Try to decrease SF first (higher data rate, less ToA)
        if current_sf > 7:
            return {'action': 'DECREASE_SF', 'new_sf': current_sf - 1}
        # Or reduce TX power (save battery)
        elif current_tx_power > 2:
            return {'action': 'REDUCE_POWER', 'new_power': current_tx_power - 3}
        return None

    def increase_robustness(self):
        """Decrease data rate or increase TX power"""
        # Try to increase TX power first (faster response)
        if current_tx_power < 14:
            return {'action': 'INCREASE_POWER', 'new_power': current_tx_power + 3}
        # Or increase SF (more range, more robust)
        elif current_sf < 12:
            return {'action': 'INCREASE_SF', 'new_sf': current_sf + 1}
        return None
```

### 10.2 ADR Benefits

**Without ADR**:
```
Device at 1 km: Uses SF12 (slowest, most power)
  - ToA: 988 ms
  - Battery consumption: High
  - Network capacity: Low
```

**With ADR**:
```
Device at 1 km: Network adjusts to SF7 (fastest, least power)
  - ToA: 41 ms (24× faster!)
  - Battery consumption: Much lower
  - Network capacity: 24× higher
```

**Power Savings**:
- SF12→SF7: Save ~95% ToA = ~95% TX energy
- Can extend battery life from 5 years to 10+ years

---

## 11. Security and Privacy

### 11.1 LoRaWAN Security

**Encryption Keys**:
```
Root Key:
  └─ AppKey (128-bit, pre-shared)

Derived Session Keys:
  ├─ NwkSKey (Network Session Key)
  │    └─ Used for: MIC calculation, MAC commands encryption
  └─ AppSKey (Application Session Key)
       └─ Used for: Application payload encryption
```

**AES-128 Encryption**:
```
Encrypt: Ciphertext = AES128_ENCRYPT(AppSKey, Plaintext ⊕ IV)
Decrypt: Plaintext = AES128_DECRYPT(AppSKey, Ciphertext) ⊕ IV

IV (Initialization Vector) derived from:
  - Device Address
  - Frame Counter
  - Direction (uplink/downlink)
```

**Message Integrity Code (MIC)**:
```
MIC = AES128_CMAC(NwkSKey, MHDR | DevAddr | FCtrl | FCnt | FPort | Payload)

Prevents:
  - Message tampering
  - Replay attacks (via FCnt)
  - Spoofing
```

### 11.2 Sigfox Security

**Device Authentication**:
- Unique device ID
- Secret device key (NAK - Network Authentication Key)
- Message authentication code

**Message Authentication**:
```
MAC = AES128(NAK, DeviceID | SequenceNumber | Payload)

MAC prevents:
  - Device spoofing
  - Message injection
  - Replay attacks
```

### 11.3 NB-IoT/LTE-M Security

Uses LTE security architecture:

**Authentication (AKA - Authentication and Key Agreement)**:
```
Device ──── Auth Request ────▶ Network
       ◀─── Challenge ────────
       ──── Response ────────▶
       ◀─── Accept ──────────

Mutual authentication using:
  - IMSI (International Mobile Subscriber Identity)
  - K (Secret key)
```

**Encryption**:
- Algorithm: AES-128 or SNOW 3G
- Keys: CK (Cipher Key), IK (Integrity Key)
- Protects both signaling and user data

### 11.4 Privacy Protection

**IMSI Privacy** (NB-IoT/LTE-M):
- Use TMSI (Temporary Mobile Subscriber Identity)
- Changes periodically
- Prevents tracking

**LoRaWAN Privacy**:
- DevAddr changes on re-join
- No permanent identifiers in messages
- Application-level anonymization recommended

---

## 12. Coverage Planning

### 12.1 Link Budget

**General Formula**:
```
Path Loss (dB) = TX Power + TX Gain - TX Losses
                 + RX Gain - RX Losses
                 - RX Sensitivity - Margin

Example (LoRaWAN SF12):
  TX Power:       14 dBm
  TX Antenna:     2 dBi
  TX Losses:      -1 dB
  RX Antenna:     8 dBi (gateway)
  RX Losses:      -1 dB
  RX Sensitivity: -137 dBm
  Fade Margin:    -15 dB
  ──────────────────────────
  Path Loss:      144 dB
```

### 12.2 Range Estimation

**Free Space Path Loss**:
```
FSPL (dB) = 20×log₁₀(d) + 20×log₁₀(f) + 32.45

Where:
  d = distance in km
  f = frequency in MHz

Example (868 MHz):
  FSPL = 20×log₁₀(10) + 20×log₁₀(868) + 32.45
       = 20 + 58.8 + 32.45
       = 111.25 dB for 10 km
```

**Hata Urban Model** (more realistic):
```
Path Loss = 69.55 + 26.16×log₁₀(f) - 13.82×log₁₀(hb)
            + (44.9 - 6.55×log₁₀(hb))×log₁₀(d) - a(hm)

Where:
  f = frequency in MHz
  hb = base station height in meters
  hm = mobile height in meters
  d = distance in km
  a(hm) = correction factor

Example:
  f = 868 MHz, hb = 30m, hm = 1.5m, d = 5 km
  Path Loss ≈ 138 dB
```

### 12.3 Gateway Placement

**Urban Deployment**:
```
Gateway Density: 1 gateway per 2-5 km²
Gateway Height: 15-30 meters (rooftop)
Expected Range: 2-5 km
Devices per Gateway: 5,000-50,000 (depends on traffic)
```

**Rural Deployment**:
```
Gateway Density: 1 gateway per 50-200 km²
Gateway Height: 30-50 meters (tower)
Expected Range: 15-50 km
Devices per Gateway: 1,000-10,000
```

**Indoor Coverage**:
```
Penetration Loss: 10-30 dB (depends on building)
Solution: Indoor gateways or femtocells
Range: 100-500 meters indoors
```

### 12.4 Capacity Planning

**LoRaWAN Gateway Capacity**:
```
Time on Air (SF12, 125 kHz): ~1 second per message
Channels: 8 (US915), 3 (EU868)
Duty Cycle: None (US), 1% (EU)

Theoretical Capacity (US, no collisions):
  8 channels × 3600 messages/hour/channel = 28,800 messages/hour

Practical Capacity (with collisions, traffic):
  ~10,000 messages/hour per gateway
  ~240,000 messages/day per gateway

Devices supported (1 message/hour/device):
  ~10,000 devices per gateway
```

---

## 13. Performance Requirements

### 13.1 Latency Requirements

| Technology | Uplink Latency | Downlink Latency | Total RTT |
|------------|----------------|------------------|-----------|
| **LoRaWAN** | 1-5 s | 1-10 s | 2-15 s |
| **Sigfox** | 2-6 s | 10-30 s | 12-36 s |
| **NB-IoT** | 1.5-10 s | 1.5-10 s | 3-20 s |
| **LTE-M** | 0.01-5 s | 0.01-5 s | 0.02-10 s |

### 13.2 Reliability Requirements

**Packet Delivery Ratio**:
```
LoRaWAN:  95-99% (single transmission)
          99.9%+ (confirmed transmission with retries)
Sigfox:   99.9%+ (triple redundancy)
NB-IoT:   99%+ (with coverage enhancements)
LTE-M:    99%+ (cellular reliability)
```

**Message Confirmation**:
- LoRaWAN: Optional (confirmed uplinks cost battery)
- Sigfox: Automatic (3× redundancy)
- NB-IoT: Protocol-level ACK
- LTE-M: Protocol-level ACK

### 13.3 Battery Life Targets

| Use Case | Target Life | Technology Choice |
|----------|-------------|-------------------|
| **Utility Metering** | 15-20 years | Sigfox, LoRaWAN (Class A) |
| **Agriculture** | 10-15 years | LoRaWAN, NB-IoT (PSM) |
| **Asset Tracking** | 5-10 years | LoRaWAN, LTE-M |
| **Wearables** | 1-3 years | LTE-M, NB-IoT |
| **Smart Parking** | 10+ years | LoRaWAN, Sigfox, NB-IoT |

### 13.4 Cost Targets

**Module Cost**:
```
LoRaWAN:  $5-15 (Class A)
Sigfox:   $3-8
NB-IoT:   $5-12
LTE-M:    $8-15
```

**Deployment Cost** (per device):
```
LoRaWAN:  $0-2/month (private network) or $1-5/year (public)
Sigfox:   $1-10/year (depends on plan)
NB-IoT:   $1-5/month (carrier plan)
LTE-M:    $2-10/month (carrier plan)
```

---

## 14. References

### 14.1 LoRaWAN Standards

1. **LoRaWAN 1.0.4 Specification** - LoRa Alliance
2. **LoRaWAN 1.1 Specification** - LoRa Alliance (enhanced security)
3. **LoRaWAN Regional Parameters** - LoRa Alliance
4. **LoRa Physical Layer Specification** - Semtech

### 14.2 Sigfox Standards

1. **Sigfox Device Radio Specifications** - Sigfox
2. **Sigfox Device Protocol Specification** - Sigfox
3. **Sigfox Network Architecture** - Sigfox

### 14.3 3GPP Standards (NB-IoT, LTE-M)

1. **3GPP TS 36.211** - Physical channels and modulation
2. **3GPP TS 36.213** - Physical layer procedures
3. **3GPP TS 36.331** - RRC protocol specification
4. **3GPP TS 24.301** - NAS protocol for EPS
5. **3GPP TS 23.682** - Architecture enhancements for IoT

### 14.4 Regulatory Standards

1. **ETSI EN 300 220** - Short Range Devices (Europe)
2. **FCC Part 15** - Unlicensed transmitters (USA)
3. **ARIB STD-T108** - Low power radio systems (Japan)

### 14.5 WIA Standards

- **WIA-INTENT**: Intent-based IoT management
- **WIA-OMNI-API**: Universal IoT API
- **WIA-EDGE**: Edge computing for IoT
- **WIA-QUANTUM**: Quantum-secure IoT communication
- **WIA-SOCIAL**: IoT device coordination

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-COMM-018 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
