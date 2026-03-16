# WIA-SOC-011: Gas Supply Standard
## PHASE 3: Communication Protocol Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 3 specifies communication protocols for various network environments including SCADA systems, IoT sensor networks, and cloud integrations.

---

## 2. SCADA Protocol Integration

### 2.1 Modbus TCP/IP

**Profile**: WIA-SOC-011-Modbus

Function Codes Supported:
- FC03: Read Holding Registers (measurements, setpoints)
- FC04: Read Input Registers (sensor readings)
- FC06: Write Single Register (setpoint adjustments)
- FC16: Write Multiple Registers (configuration)

Register Mapping:
```
Address Range | Data Type         | Description
40001-40100   | Pressure (bar)    | Pipeline pressure measurements
40101-40200   | Flow (m³/h)       | Flow rate measurements
40201-40300   | Temperature (°C)  | Temperature sensors
40301-40400   | Status Bits       | Equipment status flags
```

### 2.2 DNP3

**Profile**: WIA-SOC-011-DNP3

Object Groups:
- Group 1: Binary Input (valve positions, alarm states)
- Group 30: Analog Input (pressure, flow, temperature)
- Group 40: Analog Output Status (setpoints)
- Group 41: Analog Output (control commands)

### 2.3 OPC UA

**Namespace**: `http://wiastandards.com/UA/GasSupply/`

Information Model:
```
GasSupplySystem (ObjectType)
├── Pipelines (FolderType)
│   ├── Pipeline_001 (PipelineType)
│   │   ├── Pressure (AnalogItemType)
│   │   ├── FlowRate (AnalogItemType)
│   │   └── Temperature (AnalogItemType)
├── Compressors (FolderType)
└── MeterStations (FolderType)
```

---

## 3. IoT Sensor Protocols

### 3.1 MQTT

**Topics Structure**:
```
gas/{region}/{pipelineId}/{sensorType}/{metricName}
gas/kr/PL-KR-001-2025/pressure/value
gas/kr/PL-KR-001-2025/leak-detector/status
```

**QoS Levels**:
- QoS 0: Non-critical monitoring data
- QoS 1: Standard operational measurements
- QoS 2: Critical alarms and control commands

**Payload Format**:
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 75.5,
  "unit": "bar",
  "quality": "good",
  "sensorId": "SENS-001"
}
```

### 3.2 CoAP

**Resource Structure**:
```
coap://sensor.local/gas/pressure
coap://sensor.local/gas/temperature
coap://sensor.local/gas/battery-level
```

**Observe Pattern**: Sensors support CoAP Observe for continuous updates

### 3.3 LoRaWAN

**Port Assignments**:
- Port 1: Leak detection alerts
- Port 2: Pressure measurements
- Port 3: Device diagnostics

**Payload Encoding**: Compact binary format for bandwidth efficiency
```
Byte 0-3: Timestamp (Unix epoch)
Byte 4-5: Pressure (uint16, 0.1 bar resolution)
Byte 6: Battery level (%)
Byte 7: Status flags
```

---

## 4. Security Protocols

### 4.1 TLS Configuration

**Minimum Version**: TLS 1.3

**Cipher Suites** (in order of preference):
1. TLS_AES_256_GCM_SHA384
2. TLS_CHACHA20_POLY1305_SHA256
3. TLS_AES_128_GCM_SHA256

**Certificate Requirements**:
- Key Size: 2048-bit RSA or 256-bit ECC minimum
- Signature Algorithm: SHA-256 or stronger
- Validity Period: Maximum 825 days
- Certificate Transparency: Required for public endpoints

### 4.2 Message Signing

**Algorithm**: EdDSA (Ed25519)

**Signed Message Format**:
```json
{
  "payload": "base64-encoded-data",
  "signature": "base64-signature",
  "keyId": "key-identifier",
  "algorithm": "Ed25519",
  "timestamp": "2025-12-26T14:30:00Z"
}
```

---

## 5. Network Architecture Patterns

### 5.1 DMZ Configuration

```
Internet
    ↓
[Firewall]
    ↓
[DMZ - API Servers, Web Interfaces]
    ↓
[Internal Firewall]
    ↓
[Control Network - SCADA, PLCs]
    ↓
[Process Network - Sensors, Actuators]
```

### 5.2 VPN for Site-to-Site

**Protocol**: IPsec IKEv2  
**Encryption**: AES-256-GCM  
**Authentication**: Certificate-based  

---

## 6. Quality of Service

### 6.1 Traffic Prioritization

```
Priority 1 (Critical):  Emergency shutdown commands, critical alarms
Priority 2 (High):      Control commands, high-severity alarms
Priority 3 (Medium):    Real-time measurements, status updates
Priority 4 (Low):       Historical data queries, reports
```

### 6.2 Message Persistence

**MQTT Retained Messages**: Last known good values  
**Queue Depth**: Minimum 1000 messages per topic  
**TTL**: Configurable, default 24 hours  

---

## 7. Protocol Performance

### 7.1 Latency Requirements

- Emergency commands: <100ms
- Control commands: <500ms
- Measurements: <1s
- Historical queries: <5s

### 7.2 Bandwidth Optimization

**Compression**: gzip or brotli for HTTP APIs  
**Delta Encoding**: Transmit only changed values  
**Batch Updates**: Aggregate multiple measurements  

---

## 8. Interoperability Testing

### 8.1 Conformance Tests

Test suites available for:
- Modbus TCP/IP client/server
- DNP3 outstation/master
- OPC UA client/server
- MQTT publisher/subscriber

### 8.2 Certification Process

1. Submit implementation details
2. Execute conformance test suite
3. Interoperability testing with reference implementations
4. Security assessment
5. Issue compliance certificate

---

**Document Control**
- Version: 1.0
- Test Suites: https://github.com/WIA-Official/wia-standards/tree/main/gas-supply/tests
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License
