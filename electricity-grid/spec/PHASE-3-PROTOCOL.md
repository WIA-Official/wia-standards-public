# PHASE 3: Communication Protocol Specification

**WIA-SOC-010 Electricity Grid Standard**
Version: 1.0
Status: Draft
Last Updated: 2025-12-26

---

## 1. Overview

This document specifies communication protocols for the WIA-SOC-010 Electricity Grid Standard, covering SCADA protocols, IoT messaging, and security requirements.

## 2. Protocol Stack

### 2.1 Application Layer
- HTTPS/REST (primary)
- WebSocket (real-time)
- MQTT (IoT devices)
- IEC 61850 (substations)
- DNP3 (SCADA)
- Modbus TCP (field devices)

### 2.2 Transport Layer
- TLS 1.3 (required for all connections)
- TCP (primary)
- UDP (time-critical applications)

### 2.3 Network Layer
- IPv4 and IPv6 (dual-stack support)
- IPsec (optional, for VPNs)

## 3. MQTT for IoT Devices

### 3.1 Broker Configuration
- Protocol: MQTT 5.0
- Port: 8883 (TLS), 1883 (non-TLS, discouraged)
- QoS Levels: 0, 1, 2 (all supported)

### 3.2 Topic Structure
```
wia-soc-010/{region}/{zone}/{device-type}/{device-id}/{metric}
```

**Examples:**
```
wia-soc-010/northeast/zone-a/solar/pv-001/generation
wia-soc-010/northeast/zone-a/battery/ess-001/soc
wia-soc-010/northeast/zone-a/meter/meter-123/consumption
```

### 3.3 Message Format
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 580,
  "unit": "MW",
  "quality": "good"
}
```

### 3.4 Security
- Username/password authentication (minimum)
- Client certificates (recommended)
- TLS 1.3 encryption (required)

## 4. IEC 61850 for Substations

### 4.1 Profile Support
- IEC 61850-7-2: Abstract communication service interface (ACSI)
- IEC 61850-8-1: MMS mapping
- IEC 61850-9-2: Sampled values
- IEC 61850-90-5: Synchrophasors

### 4.2 GOOSE Messaging
Generic Object Oriented Substation Event (GOOSE) for peer-to-peer communication:
- Protocol: IEC 61850-8-1
- Transport: Ethernet Layer 2 (no IP)
- Latency: < 4 milliseconds

### 4.3 Sampled Values
- Protocol: IEC 61850-9-2LE
- Sampling rate: 80 samples/cycle (4800 Hz for 60 Hz systems)
- Precision: 16-bit minimum

## 5. DNP3 for SCADA

### 5.1 Protocol Version
- DNP3: IEEE 1815-2012
- Secure Authentication: DNP3 SAv5

### 5.2 Object Groups
- Binary Input (Group 1)
- Binary Output (Group 10)
- Analog Input (Group 30)
- Analog Output (Group 40)
- Time and Date (Group 50)

### 5.3 Security
- Challenge-response authentication
- Session key management
- Message authentication codes (MAC)

## 6. Modbus TCP

### 6.1 Configuration
- Port: 502
- Unit ID: 1-247
- Timeout: 5 seconds

### 6.2 Function Codes
- 01: Read Coils
- 02: Read Discrete Inputs
- 03: Read Holding Registers
- 04: Read Input Registers
- 05: Write Single Coil
- 06: Write Single Register
- 15: Write Multiple Coils
- 16: Write Multiple Registers

## 7. Security Requirements

### 7.1 Encryption
- TLS 1.3 (required)
- AES-256-GCM (symmetric)
- RSA-4096 or ECC P-384 (asymmetric)

### 7.2 Authentication
- OAuth 2.0 + JWT (APIs)
- X.509 certificates (device authentication)
- Multi-factor authentication (administrative access)

### 7.3 Authorization
- Role-based access control (RBAC)
- Principle of least privilege
- Audit logging

### 7.4 Network Security
- Firewalls and DMZ architecture
- Intrusion detection/prevention systems
- Network segmentation (IT/OT separation)

## 8. Time Synchronization

### 8.1 Protocol
- NTP (Network Time Protocol)
- PTP (Precision Time Protocol) for synchrophasors

### 8.2 Accuracy Requirements
- SCADA: ±1 second
- Synchrophasors: ±1 microsecond
- Smart meters: ±10 seconds

## 9. Quality of Service

### 9.1 Latency Targets
- Critical control: < 10 ms
- SCADA telemetry: < 100 ms
- Smart meter data: < 1 second
- Historical data: Best effort

### 9.2 Availability
- Control systems: 99.99%
- Data collection: 99.9%
- Historical retrieval: 99%

---

**End of PHASE 3 Specification**
