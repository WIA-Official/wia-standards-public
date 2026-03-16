# WIA-SPACE-017: Drone (UAV) Technical Specification v1.0

**Standard ID**: WIA-SPACE-017
**Title**: Drone (Unmanned Aerial Vehicle) Standard
**Version**: 1.0
**Status**: Active
**Published**: 2025-01-15
**Organization**: WIA (World Certification Industry Association)
**License**: MIT License

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [System Architecture](#5-system-architecture)
6. [Hardware Specifications](#6-hardware-specifications)
7. [Software and Firmware](#7-software-and-firmware)
8. [Communication Protocols](#8-communication-protocols)
9. [Safety Requirements](#9-safety-requirements)
10. [Testing and Certification](#10-testing-and-certification)
11. [Compliance](#11-compliance)

---

## 1. Introduction

### 1.1 Purpose

WIA-SPACE-017 establishes a comprehensive technical standard for Unmanned Aerial Vehicles (UAVs), commonly known as drones. This standard aims to ensure:

- **Safety**: Protecting people, property, and other aircraft
- **Interoperability**: Enabling seamless integration across systems
- **Reliability**: Ensuring consistent performance under specified conditions
- **Compliance**: Meeting regulatory requirements worldwide

### 1.2 Philosophy

This standard embodies the principle of **弘益人間 (Benefit All Humanity)**, ensuring drone technology serves the greater good while maintaining safety and ethical standards.

### 1.3 Applicability

This standard applies to:
- Commercial drones (aerial photography, surveying, inspection)
- Industrial drones (agriculture, delivery, infrastructure)
- Research and development platforms
- Educational drone systems

Military and defense applications may reference but are not bound by this standard.

---

## 2. Scope

### 2.1 Covered Systems

This standard covers:
- Multicopter systems (tri, quad, hexa, octocopter)
- Fixed-wing UAVs
- Hybrid VTOL (Vertical Takeoff and Landing) systems
- Tethered drone systems
- Payload systems and sensors

### 2.2 Weight Classes

- **Micro**: < 250g
- **Mini**: 250g - 2kg
- **Small**: 2kg - 25kg
- **Medium**: 25kg - 150kg
- **Large**: > 150kg

### 2.3 Out of Scope

- Manned aircraft
- Model rockets and balloons
- Tethered kites
- Indoor-only toy drones < 50g

---

## 3. Normative References

The following standards are referenced in this specification:

- **ISO 21384-1:2019**: Unmanned aircraft systems — General requirements
- **ISO 21384-2:2019**: UAS — Product manufacturing requirements
- **ISO 21384-3:2021**: UAS — Operational procedures
- **ASTM F3411-22**: Remote ID and tracking
- **ASTM F3322-18**: Small UAS parachute systems
- **MAVLink v2.0**: Micro Air Vehicle communication protocol
- **FAA Part 107**: Commercial drone regulations (USA)
- **EASA Regulations (EU) 2019/947**: Drone operation rules (Europe)
- **IEEE 1873**: Robot ethics standards

---

## 4. Terms and Definitions

### 4.1 Core Terms

**UAV (Unmanned Aerial Vehicle)**: An aircraft without a human pilot aboard, controlled remotely or autonomously.

**UAS (Unmanned Aircraft System)**: Complete system including UAV, ground control station, communication links, and supporting equipment.

**VLOS (Visual Line of Sight)**: Operation where pilot maintains direct visual contact with aircraft.

**BVLOS (Beyond Visual Line of Sight)**: Operation beyond pilot's visual range, requiring additional safety measures.

**Failsafe**: Automatic safety action when abnormal conditions occur.

**Geofence**: Virtual geographic boundary to restrict drone operations.

**MTOW (Maximum Takeoff Weight)**: Maximum weight at which drone is certified to operate.

### 4.2 Technical Terms

**FC (Flight Controller)**: Central processing unit controlling drone stability and navigation.

**ESC (Electronic Speed Controller)**: Device regulating motor speed.

**IMU (Inertial Measurement Unit)**: Sensor package measuring acceleration and rotation.

**GNSS**: Global Navigation Satellite System (GPS, GLONASS, Galileo, BeiDou).

**RTK (Real-Time Kinematic)**: High-precision GPS technique achieving cm-level accuracy.

**SLAM (Simultaneous Localization and Mapping)**: Algorithm for mapping unknown environments.

---

## 5. System Architecture

### 5.1 Layered Architecture

```
┌─────────────────────────────────────┐
│     Application Layer               │
│  (Mission Planning, Payload Control)│
├─────────────────────────────────────┤
│     Autonomy Layer                  │
│  (Path Planning, Obstacle Avoidance)│
├─────────────────────────────────────┤
│     Control Layer                   │
│  (Flight Controller, PID Loops)     │
├─────────────────────────────────────┤
│     Sensor Layer                    │
│  (IMU, GPS, Cameras, LiDAR)         │
├─────────────────────────────────────┤
│     Actuation Layer                 │
│  (Motors, ESCs, Servos)             │
└─────────────────────────────────────┘
```

### 5.2 Core Subsystems

1. **Propulsion System**: Motors, ESCs, propellers, power distribution
2. **Power System**: Battery, voltage regulation, power monitoring
3. **Flight Control System**: FC, IMU, sensors, control algorithms
4. **Navigation System**: GPS/GNSS, compass, barometer
5. **Communication System**: RC receiver, telemetry, video transmission
6. **Payload System**: Cameras, sensors, mission-specific equipment
7. **Safety System**: Failsafe, geofence, collision avoidance

---

## 6. Hardware Specifications

### 6.1 Frame Requirements

**Materials**: Approved materials include:
- Carbon fiber (preferred for strength-to-weight ratio)
- Aluminum alloy (6061-T6 or equivalent)
- Engineering plastics (ABS, PC, nylon)
- Composite materials (fiberglass, aramid)

**Structural Requirements**:
- Minimum safety factor: 2.0 for static loads
- Vibration resistance: 5-2000 Hz
- Operating temperature: -20°C to +50°C
- Crash resistance: Withstand 5G impact

### 6.2 Propulsion System

**Motors**:
- Type: Brushless DC motors (BLDC) required for drones > 250g
- Efficiency: Minimum 80% at rated load
- Temperature rating: -20°C to +100°C
- Lifetime: Minimum 500 flight hours

**ESCs**:
- Protocol support: DShot300/600 minimum
- Current rating: 125% of maximum motor current
- Thermal protection: Auto-shutdown at 100°C
- Firmware: User-updateable (BLHeli_S/BLHeli_32 or equivalent)

**Propellers**:
- Material: Reinforced polymer or carbon fiber
- Balance: < 0.5 gram-cm imbalance
- Inspection: Visual inspection every 50 flight hours

### 6.3 Battery System

**Battery Type**: LiPo (Lithium Polymer) or LiHV (High Voltage) approved

**Requirements**:
- Cell voltage monitoring per cell
- Over-discharge protection (< 3.0V/cell)
- Over-charge protection (> 4.2V/cell for standard LiPo)
- Short-circuit protection
- Temperature monitoring
- Fire-resistant storage bag for transport

**C-Rating**: Minimum discharge rate to support maximum current draw + 20% margin

### 6.4 Flight Controller

**Processor Requirements**:
- Minimum: 32-bit ARM processor, 168MHz (e.g., STM32 F4)
- Recommended: 480MHz (e.g., STM32 H7)

**Sensors**:
- 6-axis IMU (3-axis gyro + 3-axis accelerometer) minimum
- 9-axis IMU (+ magnetometer) recommended
- Barometer for altitude estimation
- Optional: Dual IMU for redundancy

**Update Rate**:
- Gyro sampling: Minimum 4kHz, recommended 8kHz+
- PID loop frequency: Minimum 2kHz, recommended 4kHz+

---

## 7. Software and Firmware

### 7.1 Flight Controller Firmware

**Approved Firmware**:
- Betaflight 4.3+
- iNav 5.0+
- ArduPilot 4.2+
- PX4 1.13+
- Proprietary firmware meeting equivalent standards

**Required Features**:
- Failsafe mechanisms (RC loss, battery low, GPS loss)
- Flight mode management (Manual, Stabilized, Auto)
- Blackbox logging capability
- Over-the-air (OTA) firmware updates with rollback

### 7.2 Ground Control Software

**Compatibility**:
- MAVLink v2.0 protocol support
- QGroundControl, Mission Planner, or equivalent

**Features**:
- Mission planning and upload
- Real-time telemetry monitoring
- Parameter configuration
- Log analysis tools

---

## 8. Communication Protocols

### 8.1 Remote Control

**Frequency Bands**:
- 2.4 GHz: FHSS (Frequency Hopping Spread Spectrum) required
- 900 MHz: For long-range operations (where permitted)
- 5.8 GHz: Not recommended for control (video only)

**Protocols**:
- Minimum latency: < 20ms
- Range: Minimum 500m VLOS
- Failsafe: Automatic triggered within 2 seconds of signal loss

### 8.2 Telemetry

**Data Link**:
- MAVLink v2.0 recommended
- LTM (Lightweight Telemetry) acceptable
- Proprietary protocols must provide equivalent functionality

**Transmitted Data** (minimum):
- GPS position (latitude, longitude, altitude)
- Battery voltage and current
- Flight mode
- Satellite count and HDOP
- Attitude (roll, pitch, yaw)

**Update Rate**: Minimum 1 Hz, recommended 5 Hz+

### 8.3 Video Transmission

**Analog FPV**:
- Frequency: 5.8 GHz (where permitted)
- Power: Comply with local regulations (typically 25mW-1W)
- Channels: Use standard band allocations (Raceband, Boscam)

**Digital FPV**:
- Latency: < 40ms glass-to-glass
- Resolution: Minimum 720p
- Interference mitigation required

---

## 9. Safety Requirements

### 9.1 Pre-Flight Safety

**Mandatory Checks**:
1. Visual inspection of frame, propellers, motors
2. Battery voltage and capacity verification
3. Control surface and motor response test
4. GPS satellite lock (for GPS-dependent modes)
5. Compass calibration verification
6. Failsafe function test
7. Geofence activation (if applicable)

### 9.2 In-Flight Safety

**Failsafe Behaviors** (configurable priority):
1. **RC Signal Loss**:
   - Option A: Return to Home (RTH)
   - Option B: Land immediately
   - Option C: Hover (GPS required)

2. **Low Battery**:
   - Warning threshold: 30% remaining (configurable)
   - Critical threshold: 15% remaining
   - Action: Automatic RTH or land

3. **GPS Loss** (in GPS mode):
   - Transition to ATTI (attitude) mode
   - Alert operator
   - Hover or descend

### 9.3 Geofencing

**Requirements**:
- Maximum altitude limit (default: 120m AGL, configurable)
- Maximum distance limit (default: 500m, configurable)
- No-fly zone database integration (optional)
- Automatic action on breach: Stop, RTH, or hover

### 9.4 Collision Avoidance

**For drones > 2kg** (recommended):
- Forward obstacle detection (minimum range: 15m)
- Automatic braking or avoidance maneuver
- Visual and/or audible warning to operator

---

## 10. Testing and Certification

### 10.1 Bench Testing

**Required Tests**:
1. **Power System Test**:
   - Current draw at hover and maximum throttle
   - Battery discharge curve
   - Voltage sag under load

2. **Communication Test**:
   - RC range test
   - Telemetry link quality
   - Video transmission quality (if applicable)

3. **Sensor Calibration**:
   - IMU calibration verification
   - Compass calibration (eight-figure pattern)
   - ESC calibration

### 10.2 Ground Testing

**Procedures**:
1. Motor direction verification
2. Propeller security check
3. Arming/disarming sequence
4. Failsafe trigger test (RC off)
5. GPS lock time measurement

### 10.3 Flight Testing

**Test Sequence**:
1. **Hover Test**: 5-minute stable hover
2. **Control Response Test**: Roll, pitch, yaw inputs
3. **Altitude Hold Test**: Maintain altitude ±2m
4. **Position Hold Test**: Maintain position ±5m (GPS required)
5. **RTH Test**: Automatic return to launch point
6. **Failsafe Test**: RC signal loss simulation

**Pass Criteria**:
- All tests completed without crashes
- Failsafe functions as configured
- No unexpected behavior or oscillations

### 10.4 Documentation

**Required Documentation**:
- Build manifest (all components and versions)
- Calibration records
- Test results and logs
- Operating procedures (SOP)
- Maintenance schedule

---

## 11. Compliance

### 11.1 Regulatory Compliance

**Operators must comply with**:
- Local aviation authority regulations (FAA, EASA, CAAC, etc.)
- Radio frequency regulations
- Privacy and data protection laws
- Airspace restrictions and NOTAMs

### 11.2 Marking and Identification

**Required Markings**:
- Registration number (where required, typically > 250g)
- Manufacturer name and model
- Maximum takeoff weight (MTOW)
- Serial number

**Remote ID** (where required):
- Broadcast drone ID, location, altitude, speed
- Operator location
- Timestamp
- Compliance with ASTM F3411 or equivalent

### 11.3 Maintenance

**Periodic Inspection** (recommended intervals):
- **After each flight**: Visual inspection
- **Every 10 flight hours**: Propeller replacement consideration
- **Every 25 flight hours**: Motor and ESC inspection
- **Every 50 flight hours**: Full system inspection and calibration

**Component Replacement**:
- Propellers: At first sign of damage or every 100 flight hours
- Motors: After 500 flight hours or performance degradation
- Batteries: After 300 cycles or capacity < 80%

---

## Appendix A: Acronyms

- **AGL**: Above Ground Level
- **BVLOS**: Beyond Visual Line of Sight
- **ESC**: Electronic Speed Controller
- **FC**: Flight Controller
- **FPV**: First Person View
- **GNSS**: Global Navigation Satellite System
- **GPS**: Global Positioning System
- **IMU**: Inertial Measurement Unit
- **LiPo**: Lithium Polymer
- **MTOW**: Maximum Takeoff Weight
- **PID**: Proportional-Integral-Derivative
- **RC**: Remote Control
- **RF**: Radio Frequency
- **RTH**: Return to Home
- **RTK**: Real-Time Kinematic
- **SLAM**: Simultaneous Localization and Mapping
- **UAV**: Unmanned Aerial Vehicle
- **UAS**: Unmanned Aircraft System
- **VLOS**: Visual Line of Sight
- **VTOL**: Vertical Takeoff and Landing

---

## Appendix B: Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial release |

---

## Contact

**WIA (World Certification Industry Association)**
SmileStory Inc.
Website: https://github.com/WIA-Official/wia-standards
Email: standards@wia.org

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
**Licensed under MIT License**
