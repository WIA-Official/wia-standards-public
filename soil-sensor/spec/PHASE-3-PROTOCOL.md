# WIA-AGRI-005 PHASE 3: Communication Protocol

## Overview
Phase 3 defines the communication protocols for WIA-AGRI-005 Soil Sensor Standard, enabling reliable data transmission from sensors in agricultural fields to cloud platforms through LoRaWAN, MQTT, HTTP/HTTPS, and other protocols.

## Protocol Stack

### Supported Protocols

1. **LoRaWAN** - Long-range, low-power wireless for remote fields
2. **MQTT** - Lightweight pub/sub messaging for connected sensors
3. **HTTP/HTTPS** - RESTful communication for standard deployments
4. **CoAP** - Constrained Application Protocol for IoT devices
5. **Modbus RTU/TCP** - Industrial automation protocol
6. **NB-IoT** - Narrowband IoT cellular communication

## 1. LoRaWAN Protocol

### Overview
LoRaWAN is ideal for soil sensors in remote agricultural fields where traditional connectivity is unavailable. Provides long-range (2-15km) with low power consumption.

### Network Architecture
```
[Soil Sensor] --LoRa--> [Gateway] --IP--> [Network Server] --HTTPS--> [Application Server]
```

### Device Configuration

#### Device Profile
```json
{
  "deviceEUI": "70B3D57ED0000001",
  "appEUI": "70B3D57ED0001000",
  "appKey": "2B7E151628AED2A6ABF7158809CF4F3C",
  "deviceClass": "A",
  "region": "EU868",
  "activationType": "OTAA"
}
```

#### LoRaWAN Classes
- **Class A (Required)**: Bi-directional, lowest power
  - Uplink: Sensor sends data when needed
  - Downlink: Only after uplink (2 receive windows)
  - Battery life: Up to 10 years

- **Class B (Optional)**: Scheduled receive windows
  - Periodic downlink opportunities
  - Time-synchronized beacons
  - Battery life: 2-5 years

- **Class C (Optional)**: Continuous listening
  - Always ready for downlink
  - Highest power consumption
  - Battery life: Requires mains power

### Data Encoding

#### Uplink Message Format (Soil Sensor)
```
Byte 0: Message Type (0x01 = Sensor Reading)
Byte 1-2: Nitrogen (mg/kg) - uint16, Big Endian
Byte 3-4: Phosphorus (mg/kg) - uint16, Big Endian
Byte 5-6: Potassium (mg/kg) - uint16, Big Endian
Byte 7: pH (scaled: pH * 10) - uint8
Byte 8: EC (scaled: EC * 10) - uint8
Byte 9: Moisture (%) - uint8
Byte 10: Temperature (°C + 50) - uint8
Byte 11: Battery (%) - uint8
```

**Example**:
```
01 0055 0023 0096 44 0C 23 48 57
```
Decoded:
- Nitrogen: 85 mg/kg
- Phosphorus: 35 mg/kg
- Potassium: 150 mg/kg
- pH: 6.8
- EC: 1.2 dS/m
- Moisture: 35%
- Temperature: 22°C
- Battery: 87%

#### Downlink Message Format (Configuration)
```
Byte 0: Command Type
  0x01 = Set Interval
  0x02 = Request Calibration
  0x03 = Reset Device
Byte 1-N: Command Parameters
```

### Transmission Strategy

#### Adaptive Data Rate (ADR)
- Start with SF12 (longest range, lowest data rate)
- Network server adjusts SF based on link quality
- Target: SF7-SF9 for optimal battery life

#### Transmission Schedule
```json
{
  "normalMode": {
    "interval": 900,
    "spreadingFactor": "SF7",
    "transmitPower": 14
  },
  "powerSaveMode": {
    "interval": 3600,
    "spreadingFactor": "SF9",
    "transmitPower": 10
  },
  "alertMode": {
    "interval": 300,
    "spreadingFactor": "SF7",
    "transmitPower": 14
  }
}
```

### Security
- **AES-128 Encryption**: All LoRaWAN payloads
- **Network Session Key (NwkSKey)**: Network layer security
- **Application Session Key (AppSKey)**: Application layer security
- **Over-The-Air Activation (OTAA)**: Secure device joining

## 2. MQTT Protocol

### Overview
MQTT provides lightweight publish/subscribe messaging for soil sensors with reliable connectivity (Wi-Fi, Ethernet, 4G).

### Broker Configuration
```json
{
  "broker": "mqtt.wia-agri.org",
  "port": 8883,
  "protocol": "mqtts",
  "clientId": "SOIL-001",
  "username": "soil-sensor",
  "password": "***",
  "keepAlive": 60,
  "cleanSession": false,
  "qos": 1,
  "tlsVersion": "TLSv1.3"
}
```

### Topic Structure

#### Publishing (Sensor → Cloud)
```
wia/agri/soil/{sensorId}/data
wia/agri/soil/{sensorId}/status
wia/agri/soil/{sensorId}/alert
wia/agri/soil/{sensorId}/calibration
```

#### Subscribing (Cloud → Sensor)
```
wia/agri/soil/{sensorId}/command
wia/agri/soil/{sensorId}/config
wia/agri/soil/{sensorId}/firmware
```

### Message Format

#### Data Topic
```json
Topic: wia/agri/soil/SOIL-001/data
QoS: 1
Retain: false

Payload:
{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "npk": {
    "nitrogen": 85,
    "phosphorus": 35,
    "potassium": 150
  },
  "ph": 6.8,
  "ec": 1.2,
  "moisture": 35,
  "temperature": 22,
  "battery": 87,
  "standard": "WIA-AGRI-005"
}
```

#### Status Topic
```json
Topic: wia/agri/soil/SOIL-001/status
QoS: 1
Retain: true

Payload:
{
  "sensorId": "SOIL-001",
  "status": "online",
  "lastSeen": "2025-12-26T10:00:00Z",
  "uptime": 86400,
  "firmware": "1.2.3",
  "signalStrength": -75
}
```

#### Alert Topic
```json
Topic: wia/agri/soil/SOIL-001/alert
QoS: 2
Retain: false

Payload:
{
  "sensorId": "SOIL-001",
  "alertType": "moisture_low",
  "severity": "warning",
  "value": 25,
  "threshold": 30,
  "timestamp": "2025-12-26T10:00:00Z"
}
```

### Last Will and Testament (LWT)
```json
{
  "topic": "wia/agri/soil/SOIL-001/status",
  "payload": {
    "sensorId": "SOIL-001",
    "status": "offline",
    "reason": "connection_lost"
  },
  "qos": 1,
  "retain": true
}
```

## 3. HTTP/HTTPS Protocol

### RESTful Endpoints

#### Submit Reading
```http
POST https://api.wia-agri.org/v1/soil-sensor/readings
Content-Type: application/json
Authorization: Bearer <token>

{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "npk": { "nitrogen": 85, "phosphorus": 35, "potassium": 150 },
  "ph": 6.8,
  "ec": 1.2,
  "moisture": 35,
  "temperature": 22
}
```

#### Batch Submission
```http
POST https://api.wia-agri.org/v1/soil-sensor/readings/batch
Content-Type: application/json
Authorization: Bearer <token>

{
  "readings": [
    { "sensorId": "SOIL-001", ... },
    { "sensorId": "SOIL-002", ... }
  ]
}
```

### Compression
- **Gzip**: Recommended for batch submissions
- **Header**: `Content-Encoding: gzip`

### Retry Strategy
```javascript
const retryConfig = {
  maxRetries: 3,
  backoff: 'exponential',
  initialDelay: 1000,  // 1 second
  maxDelay: 30000,     // 30 seconds
  retryOn: [408, 429, 500, 502, 503, 504]
};
```

## 4. CoAP Protocol

### Overview
Constrained Application Protocol for resource-limited devices.

### Endpoint
```
coaps://coap.wia-agri.org:5684/soil-sensor/readings
```

### Message Format
```
POST coaps://coap.wia-agri.org:5684/soil-sensor/readings
Content-Format: application/json (50)
DTLS 1.2 Encryption

{
  "sensorId": "SOIL-001",
  "n": 85, "p": 35, "k": 150,
  "ph": 6.8, "m": 35, "t": 22
}
```

### Observe Pattern
```
GET coaps://coap.wia-agri.org:5684/soil-sensor/SOIL-001?observe=1
```

## 5. Modbus Protocol

### RTU Configuration
```
Baud Rate: 9600 bps
Data Bits: 8
Parity: None
Stop Bits: 1
Slave Address: 1-247
```

### Register Map

| Address | Register Type | Description | Unit | Scale |
|---------|--------------|-------------|------|-------|
| 0x0000 | Holding | Nitrogen | mg/kg | 1 |
| 0x0001 | Holding | Phosphorus | mg/kg | 1 |
| 0x0002 | Holding | Potassium | mg/kg | 1 |
| 0x0003 | Holding | pH | pH | 0.1 |
| 0x0004 | Holding | EC | dS/m | 0.1 |
| 0x0005 | Holding | Moisture | % | 1 |
| 0x0006 | Holding | Temperature | °C | 0.1 |
| 0x0007 | Input | Battery | % | 1 |
| 0x0008 | Input | Status | bitmap | - |

### Read Command
```
Request: [Slave][0x03][Start Addr Hi][Start Addr Lo][Num Regs Hi][Num Regs Lo][CRC Lo][CRC Hi]
Example: 01 03 00 00 00 07 04 08

Response: [Slave][0x03][Byte Count][Data...][CRC Lo][CRC Hi]
Example: 01 03 0E 00 55 00 23 00 96 00 44 00 0C 00 23 00 48 F2 A1
```

## Security Considerations

### Transport Layer Security
- **TLS 1.3**: Required for HTTPS and MQTTS
- **DTLS 1.2**: Required for CoAP
- **AES-128**: LoRaWAN encryption

### Authentication
```json
{
  "method": "oauth2",
  "tokenEndpoint": "https://auth.wia-agri.org/token",
  "clientId": "soil-sensor-001",
  "clientSecret": "***",
  "scope": "soil-sensor:write"
}
```

### Certificate Pinning
```json
{
  "pinnedCertificates": [
    "sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
  ],
  "validationMode": "strict"
}
```

## Network Resilience

### Offline Buffering
```json
{
  "bufferSize": 1000,
  "storageType": "flash",
  "retentionPolicy": "fifo",
  "syncStrategy": "batch"
}
```

### Connection Management
```javascript
// Automatic reconnection
const connectionConfig = {
  autoReconnect: true,
  reconnectInterval: 5000,
  maxReconnectAttempts: 10,
  connectionTimeout: 30000
};
```

## Performance Optimization

### Payload Compression
- **LoRaWAN**: Binary encoding (12 bytes)
- **MQTT**: JSON with optional gzip
- **HTTP**: JSON with gzip
- **CoAP**: CBOR encoding

### Batching Strategy
```json
{
  "batchSize": 10,
  "batchInterval": 3600,
  "flushOnAlert": true,
  "maxBatchAge": 86400
}
```

## 弘益人間 (Benefit All Humanity)
Phase 3 ensures reliable, secure, and efficient communication from soil sensors to agricultural systems worldwide, enabling farmers to make data-driven decisions regardless of connectivity constraints.

---
© 2025 SmileStory Inc. / WIA

---

## Z.1 Audit transport and observability hooks (Phase 3)

Every Phase 3 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `soil-sensor` and `wia.standard.phase` =
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 3)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-soil-sensor-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3)

Phase 3 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 3)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
