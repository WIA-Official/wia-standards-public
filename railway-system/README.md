# 🚄 WIA-AUTO-018: Railway System Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-AUTO-018
> **Version:** 1.0.0
> **Status:** Active
> **Category:** AUTO / Mobility
> **Color:** Orange (#F97316)

---

## 🌟 Overview

The WIA-AUTO-018 standard defines a comprehensive framework for railway system technologies, including signaling systems, train control, communication protocols, passenger information systems, and safety mechanisms.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to establish a unified, safe, and efficient railway infrastructure that connects communities and enables sustainable transportation for all humanity.

## 🎯 Key Features

- **Signaling Systems**: ETCS (European Train Control System), CBTC (Communications-Based Train Control), PTC (Positive Train Control)
- **Train Control**: Automatic Train Operation (ATO), Automatic Train Protection (ATP), Automatic Train Supervision (ATS)
- **Communication**: GSM-R (GSM for Railways), LTE-R (LTE for Railways), TETRA
- **Passenger Information**: Real-time arrival/departure, station displays, mobile integration
- **Platform Safety**: Platform Screen Doors (PSD), automatic gap detection
- **Safety Protocols**: SIL (Safety Integrity Level) 4 compliance, fail-safe mechanisms

## 📊 Core Concepts

### 1. Braking Distance Calculation

```
d = (v² / (2 × a)) + (v × t_r) + s_m
```

Where:
- `d` = Total braking distance (meters)
- `v` = Initial velocity (m/s)
- `a` = Deceleration rate (m/s²)
- `t_r` = Reaction time (seconds)
- `s_m` = Safety margin (meters)

### 2. Train Separation (Headway)

```
H = d_b + L_t + d_s
```

Where:
- `H` = Minimum headway (meters)
- `d_b` = Braking distance (meters)
- `L_t` = Train length (meters)
- `d_s` = Safety distance (meters)

### 3. Line Capacity

```
C = 3600 / (H/v + t_d)
```

Where:
- `C` = Line capacity (trains per hour)
- `H` = Headway (meters)
- `v` = Average velocity (m/s)
- `t_d` = Dwell time at stations (seconds)

## 🔧 Components

### TypeScript SDK

```typescript
import {
  calculateBrakingDistance,
  validateSignalAspect,
  createTrainControl,
  monitorPlatformSafety
} from '@wia/auto-018';

// Calculate braking distance for high-speed train
const braking = calculateBrakingDistance({
  initialVelocity: 83.33, // 300 km/h in m/s
  decelerationRate: 0.8,  // m/s²
  reactionTime: 2,        // seconds
  safetyMargin: 50        // meters
});

// Validate signal aspect
const signal = validateSignalAspect({
  aspect: 'GREEN',
  trainSpeed: 250,        // km/h
  nextSignalDistance: 2000, // meters
  speedLimit: 300         // km/h
});

console.log(signal.isValid, signal.requiredAction);
```

### CLI Tool

```bash
# Calculate braking distance
wia-auto-018 calc-braking --speed 300 --deceleration 0.8

# Validate train separation
wia-auto-018 validate-headway --speed 200 --braking-distance 3000

# Generate signaling configuration
wia-auto-018 generate-signal --type ETCS --level 2

# Monitor platform safety
wia-auto-018 monitor-platform --station "Central Station" --platform 3
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-AUTO-018-v1.0.md](./spec/WIA-AUTO-018-v1.0.md) | Complete specification with railway engineering |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-auto-018.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/railway-system

# Run installation script
./install.sh

# Verify installation
wia-auto-018 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/auto-018

# Or yarn
yarn add @wia/auto-018
```

```typescript
import { RailwaySDK } from '@wia/auto-018';

const sdk = new RailwaySDK();

// Calculate safe train separation
const separation = sdk.calculateHeadway({
  trainSpeed: 200,        // km/h
  trainLength: 400,       // meters
  decelerationRate: 0.7,  // m/s²
  safetyDistance: 100     // meters
});

console.log(`Minimum headway: ${separation.headway} meters`);
console.log(`Line capacity: ${separation.capacity} trains/hour`);
```

## 🔬 Railway Constants

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Standard Gauge | G | 1435 | mm |
| Maximum Deceleration | a_max | 1.2 | m/s² |
| Service Deceleration | a_svc | 0.7 | m/s² |
| Emergency Deceleration | a_emg | 1.5 | m/s² |
| Platform Height (Standard) | h_p | 1100 | mm |
| GSM-R Frequency | f_gsm | 876-880 / 921-925 | MHz |

## ⚠️ Safety Considerations

1. **SIL 4 Compliance**: All safety-critical functions must meet SIL 4 requirements
2. **Fail-Safe Design**: System defaults to safe state on any failure
3. **Redundancy**: Critical components have N+2 redundancy
4. **ATP Override**: Automatic Train Protection cannot be overridden without authorization
5. **Platform Safety**: Platform Screen Doors (PSD) must interlock with train doors
6. **Emergency Braking**: Emergency brake application must occur within 1 second

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based passenger journey planning
- **WIA-OMNI-API**: Universal railway API gateway
- **WIA-SOCIAL**: Social coordination for commuters
- **WIA-IOT**: IoT sensors for infrastructure monitoring
- **WIA-ENERGY**: Energy optimization for electric trains

## 📖 Use Cases

1. **High-Speed Rail**: ETCS Level 2/3 for trains operating above 200 km/h
2. **Urban Metro**: CBTC for high-frequency metro operations
3. **Freight Operations**: PTC for freight train safety in North America
4. **Regional Rail**: GSM-R based signaling for regional services
5. **Future Systems**: 5G-R and autonomous train operation

## 🚦 Signaling Systems

### ETCS (European Train Control System)

- **Level 1**: Point-based transmission with track-side equipment
- **Level 2**: Continuous transmission via GSM-R, no track-side signals
- **Level 3**: Moving block operation, satellite-based positioning

### CBTC (Communications-Based Train Control)

- **Grade of Automation (GoA)**:
  - GoA 1: Non-automated (manual driving)
  - GoA 2: Semi-automated (ATO with driver supervision)
  - GoA 3: Driverless (automated train operation, attendant on board)
  - GoA 4: Unattended (fully automated, no staff on train)

### PTC (Positive Train Control)

- Prevents train-to-train collisions
- Enforces speed restrictions
- Protects roadway workers
- Prevents movement through misaligned switches

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
