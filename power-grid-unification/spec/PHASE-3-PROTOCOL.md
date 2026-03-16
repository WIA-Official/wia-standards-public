# WIA-UNI-007 - Phase 3: Protocol

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 3 defines communication protocols for power grid control, SCADA integration, and cybersecurity.

## 2. IEC 61850 Compliance

### 2.1 Substation Communication

All substations must implement IEC 61850 for:

- GOOSE messaging (Generic Object-Oriented Substation Events)
- MMS (Manufacturing Message Specification)
- ROCOM (Rugged Optical Communication Module)

### 2.2 Logical Nodes

Standard logical nodes for power system devices:
- `XCBR` - Circuit Breaker
- `XSWI` - Switch Controller
- `MMXU` - Measurement
- `CSWI` - Controlled Switch

## 3. DNP3 Protocol

### 3.1 SCADA Communication

DNP3 (Distributed Network Protocol) for SCADA master-RTU communication:

- DNP3 Level 3 compliance required
- Secure Authentication (DNP3-SA) mandatory
- Unsolicited response support
- Event buffering: minimum 1000 events

### 3.2 Data Objects

Standard DNP3 objects:
- Analog Inputs (Group 30): Voltage, current, power
- Binary Inputs (Group 1): Breaker status, alarms
- Controls (Group 12): Trip/close commands

## 4. HVDC Control Protocols

### 4.1 Power Flow Control

HVDC terminals support:

```json
{
  "command": "adjust_power_flow",
  "target_mw": 1800,
  "ramp_rate": 50,
  "direction": "north_to_south",
  "priority": "normal|emergency"
}
```

Response time: <100ms

### 4.2 Frequency Regulation

Automatic frequency control with ±0.1 Hz deadband.

## 5. MQTT for IoT Devices

### 5.1 Topic Structure

```
wia/grid/{region}/{node_id}/{metric}
```

Example: `wia/grid/kr-south/HVDC-001/voltage`

### 5.2 QoS Levels

- QoS 0: Sensor data
- QoS 1: Control commands
- QoS 2: Critical alarms

## 6. Cybersecurity

### 6.1 Encryption

- TLS 1.3 for all communications
- Certificate-based authentication
- Perfect Forward Secrecy (PFS)

### 6.2 Network Segmentation

- Control network isolated from corporate network
- DMZ for external connections
- IDS/IPS at all boundaries

### 6.3 Compliance

- NERC CIP (North American Electric Reliability Corporation)
- IEC 62351 (Power systems management and associated information exchange)

