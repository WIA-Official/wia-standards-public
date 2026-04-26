# WIA-COMM-002: IoT (M2M) Specification v1.0

> **Standard ID:** WIA-COMM-002
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA IoT and M2M Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Communication Protocols](#2-communication-protocols)
3. [Device Provisioning](#3-device-provisioning)
4. [Edge-to-Cloud Architecture](#4-edge-to-cloud-architecture)
5. [LPWAN Technologies](#5-lpwan-technologies)
6. [Device Management](#6-device-management)
7. [Security and Authentication](#7-security-and-authentication)
8. [Data Management](#8-data-management)
9. [Digital Twin](#9-digital-twin)
10. [Interoperability](#10-interoperability)
11. [Implementation Guidelines](#11-implementation-guidelines)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for Internet of Things (IoT) and Machine-to-Machine (M2M) communication, enabling billions of connected devices to communicate securely and efficiently across diverse networks and applications.

### 1.2 Scope

The standard covers:
- IoT communication protocols (MQTT, CoAP, AMQP, HTTP)
- Low-Power Wide-Area Networks (LPWAN)
- Device provisioning and lifecycle management
- Edge computing and fog architectures
- Security, authentication, and encryption
- Data aggregation and analytics
- Digital twin synchronization
- Interoperability and standards compliance

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize IoT technology, making it accessible and beneficial for applications in smart cities, sustainable agriculture, healthcare, environmental monitoring, and industrial automation for the betterment of all humanity.

### 1.4 Terminology

- **IoT**: Internet of Things - network of physical devices with sensors and connectivity
- **M2M**: Machine-to-Machine - direct communication between devices
- **MQTT**: Message Queuing Telemetry Transport
- **CoAP**: Constrained Application Protocol
- **LPWAN**: Low-Power Wide-Area Network
- **Edge Computing**: Data processing near the source of data
- **Digital Twin**: Virtual representation of physical device
- **QoS**: Quality of Service

---

## 2. Communication Protocols

### 2.1 MQTT (Message Queuing Telemetry Transport)

#### 2.1.1 Protocol Overview

MQTT is a lightweight publish/subscribe messaging protocol designed for constrained devices and low-bandwidth, high-latency networks.

**Key Characteristics**:
- Protocol Version: MQTT 3.1.1 (RFC) and MQTT 5.0
- Transport: TCP/IP (default port 1883, TLS port 8883)
- Architecture: Publish/Subscribe with central broker
- Overhead: Minimal (2-byte header minimum)

#### 2.1.2 MQTT Message Structure

```
Fixed Header (2-5 bytes):
┌──────────┬──────────┬──────────────────┐
│ Byte 1   │ Byte 2+  │ Payload          │
├──────────┼──────────┼──────────────────┤
│ Type+Flg │ Length   │ Variable Header  │
│          │          │ + Payload        │
└──────────┴──────────┴──────────────────┘

Byte 1 bits:
7-4: Message Type (CONNECT, PUBLISH, SUBSCRIBE, etc.)
3: DUP flag
2-1: QoS level (0, 1, 2)
0: RETAIN flag
```

#### 2.1.3 Quality of Service Levels

**QoS 0: At most once**
```
Publisher → Broker → Subscriber
   PUBLISH →
```
- No acknowledgment
- Message may be lost
- Fastest, lowest overhead
- Use case: Non-critical telemetry

**QoS 1: At least once**
```
Publisher → Broker → Subscriber
   PUBLISH →
         ← PUBACK
               PUBLISH →
                     ← PUBACK
```
- Acknowledged delivery
- Possible duplicates
- Use case: Important telemetry, commands

**QoS 2: Exactly once**
```
Publisher → Broker → Subscriber
   PUBLISH →
         ← PUBREC
   PUBREL →
         ← PUBCOMP
                PUBLISH →
                      ← PUBREC
                PUBREL →
                      ← PUBCOMP
```
- Guaranteed delivery, no duplicates
- Highest overhead
- Use case: Critical commands, billing data

#### 2.1.4 Topic Structure

**Hierarchical topic format**:
```
{location}/{device-type}/{device-id}/{measurement}

Examples:
building-a/floor-2/temp-sensor-01/temperature
factory/line-1/plc-03/status
home/bedroom/motion/detected
city/downtown/parking/lot-A/occupancy
```

**Wildcards**:
- `+` : Single level wildcard
  - `building-a/+/temp-sensor-01/temperature` matches all floors
- `#` : Multi-level wildcard
  - `building-a/#` matches all topics under building-a

#### 2.1.5 MQTT 5.0 Enhancements

**New Features**:
1. **Request/Response Pattern**: Built-in request-response support
2. **Shared Subscriptions**: Load balancing across subscribers
3. **Message Expiry**: Automatic message expiration
4. **User Properties**: Custom key-value pairs in messages
5. **Topic Aliases**: Reduce topic name overhead
6. **Flow Control**: Receive maximum for flow control

**Example - Shared Subscription**:
```
Topic: $share/group-1/sensors/temperature
- Multiple subscribers share message load
- Round-robin or load-balanced delivery
```

### 2.2 CoAP (Constrained Application Protocol)

#### 2.2.1 Protocol Overview

CoAP is a specialized web transfer protocol for constrained devices and networks, designed as a RESTful protocol for IoT.

**Key Characteristics**:
- Protocol: RFC 7252
- Transport: UDP (default port 5683, DTLS port 5684)
- Architecture: Client/Server (REST-based)
- Message Size: Optimized for small packets
- Methods: GET, POST, PUT, DELETE (like HTTP)

#### 2.2.2 CoAP Message Format

```
CoAP Message (4+ bytes):
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|Ver| T |  TKL  |      Code     |          Message ID           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   Token (if any, TKL bytes) ...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   Options (if any) ...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|1 1 1 1 1 1 1 1|    Payload (if any) ...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

Ver: Version (2 bits, currently 01)
T: Type (2 bits - CON, NON, ACK, RST)
TKL: Token Length (4 bits)
Code: Method or Response Code (8 bits)
```

#### 2.2.3 Message Types

**Confirmable (CON)**:
```
Client → Server
  CON [GET] →
          ← ACK [2.05 Content]
```
- Requires acknowledgment
- Reliable delivery

**Non-Confirmable (NON)**:
```
Client → Server
  NON [GET] →
          ← NON [2.05 Content]
```
- No acknowledgment required
- Unreliable but efficient

#### 2.2.4 Observe Pattern (RFC 7641)

Allows clients to observe resource changes:

```
Client → Server
  CON [GET] Observe:0 →
                    ← ACK [2.05 Content] Observe:12
  (resource changes)
                    ← NON [2.05 Content] Observe:13
  (resource changes)
                    ← NON [2.05 Content] Observe:14
```

**Use cases**:
- Temperature monitoring
- Motion detection
- Sensor state changes

#### 2.2.5 Block-Wise Transfer (RFC 7959)

For large payloads exceeding MTU:

```
Client request with Block2 option:
  GET /large-resource Block2:0/0/64
  → Fetch first 64-byte block

Server response:
  ← 2.05 Content Block2:0/1/64 [64 bytes]

Client continues:
  GET /large-resource Block2:1/0/64
  ← 2.05 Content Block2:1/1/64 [64 bytes]

Final block:
  GET /large-resource Block2:2/0/64
  ← 2.05 Content Block2:2/0/64 [remaining bytes]
```

### 2.3 AMQP (Advanced Message Queuing Protocol)

#### 2.3.1 Protocol Overview

AMQP is a robust, enterprise-grade messaging protocol with advanced routing and reliability features.

**Key Characteristics**:
- Protocol Version: AMQP 1.0 (ISO/IEC 19464:2014)
- Transport: TCP (default port 5672, TLS port 5671)
- Architecture: Message broker with exchanges and queues
- Features: Transactions, security, reliability

#### 2.3.2 AMQP Architecture

```
Publisher → Exchange → Queue → Consumer

Exchange Types:
1. Direct: Route by routing key (exact match)
2. Topic: Route by pattern matching
3. Fanout: Broadcast to all queues
4. Headers: Route by header attributes
```

**Example Routing**:
```
Publisher publishes to exchange "sensor-data"
Routing key: "temperature.building-a.floor-2"

Queue bindings:
- Queue 1: "temperature.#" → receives message
- Queue 2: "temperature.building-a.*" → receives message
- Queue 3: "humidity.#" → does not receive
```

#### 2.3.3 Message Properties

```json
{
  "properties": {
    "message-id": "msg-12345",
    "correlation-id": "req-67890",
    "content-type": "application/json",
    "content-encoding": "utf-8",
    "timestamp": 1735200000000,
    "expiration": "60000",
    "priority": 5,
    "delivery-mode": 2,
    "user-id": "sensor-device-01",
    "app-id": "iot-gateway"
  },
  "headers": {
    "device-type": "temperature-sensor",
    "location": "building-a",
    "firmware-version": "2.1.0"
  },
  "body": {
    "temperature": 23.5,
    "unit": "celsius",
    "timestamp": "2025-12-26T10:30:00Z"
  }
}
```

### 2.4 HTTP/REST for IoT

#### 2.4.1 RESTful API Design

**Resource-Oriented URLs**:
```
GET    /devices                    # List all devices
GET    /devices/{id}               # Get device details
POST   /devices                    # Register new device
PUT    /devices/{id}               # Update device
DELETE /devices/{id}               # Remove device

GET    /devices/{id}/telemetry     # Get device telemetry
POST   /devices/{id}/telemetry     # Post telemetry data
GET    /devices/{id}/commands      # Get pending commands
POST   /devices/{id}/commands      # Send command
```

#### 2.4.2 Webhook Callbacks

For asynchronous notifications:

```
Device registers webhook:
POST /devices/sensor-01/webhooks
{
  "url": "https://app.example.com/iot/callback",
  "events": ["telemetry", "alarm", "offline"],
  "secret": "webhook-signing-secret"
}

Server calls webhook when event occurs:
POST https://app.example.com/iot/callback
{
  "event": "alarm",
  "device_id": "sensor-01",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    "type": "temperature-high",
    "value": 85.5,
    "threshold": 80.0
  },
  "signature": "sha256=abc123..."
}
```

---

## 3. Device Provisioning

### 3.1 Zero-Touch Provisioning

#### 3.1.1 Bootstrap Process

**Step 1: Device Discovery**
```
Device → Discovery Service
  DHCP/DNS-SD/mDNS/Bluetooth LE
  → Discover provisioning server
```

**Step 2: Initial Authentication**
```
Device → Provisioning Server
  Device Certificate or TPM attestation
  → Verify device authenticity
```

**Step 3: Credential Exchange**
```
Provisioning Server → Device
  {
    "device_id": "d123456",
    "mqtt_broker": "mqtt://iot.example.com:8883",
    "credentials": {
      "type": "certificate",
      "cert": "-----BEGIN CERTIFICATE-----...",
      "private_key": "-----BEGIN PRIVATE KEY-----..."
    },
    "ca_cert": "-----BEGIN CERTIFICATE-----...",
    "topics": {
      "telemetry": "devices/d123456/telemetry",
      "commands": "devices/d123456/commands",
      "status": "devices/d123456/status"
    }
  }
```

**Step 4: Service Activation**
```
Device connects to IoT platform using provisioned credentials
Device publishes initial status message
Platform confirms device is online and operational
```

### 3.2 Device Identity

#### 3.2.1 Unique Identifiers

**Hardware-based IDs**:
- **MAC Address**: Ethernet/WiFi MAC (48-bit)
- **IMEI**: International Mobile Equipment Identity (15 digits)
- **Serial Number**: Manufacturer-assigned unique ID
- **UUID**: Universally Unique Identifier (128-bit)

**Software-based IDs**:
- **Device ID**: Platform-assigned identifier
- **Thing Name**: Human-readable name
- **Shadow ID**: Digital twin identifier

#### 3.2.2 Certificate-Based Identity

**X.509 Device Certificates**:
```
Subject:
  CN=device-sensor-001
  O=SmartFactory Inc.
  OU=Temperature Sensors
  C=US

Subject Alternative Name:
  URI:urn:device:sensor-001
  DNS:sensor-001.devices.example.com

Extensions:
  Key Usage: Digital Signature, Key Encipherment
  Extended Key Usage: Client Authentication
  Device Type: 1.3.6.1.4.1.12345.1.2 (Temperature Sensor)
```

### 3.3 Trusted Platform Module (TPM)

#### 3.3.1 Hardware Root of Trust

**TPM Functions**:
1. **Endorsement Key (EK)**: Immutable device identity
2. **Storage Root Key (SRK)**: Secure key storage
3. **Attestation Identity Key (AIK)**: Privacy-preserving attestation
4. **Platform Configuration Registers (PCR)**: Boot integrity

**Device Attestation Flow**:
```
1. Device boots, TPM measures boot components
2. Device generates attestation quote signed by AIK
3. Server validates quote against known good values
4. Server provisions credentials if attestation passes
```

---

## 4. Edge-to-Cloud Architecture

### 4.1 Architecture Layers

```
┌─────────────────────────────────────────────┐
│           CLOUD LAYER                        │
│  - Data Lake                                 │
│  - Analytics & ML                            │
│  - Device Management                         │
│  - Long-term Storage                         │
└──────────────────┬──────────────────────────┘
                   │ Internet
┌──────────────────┴──────────────────────────┐
│           FOG/EDGE LAYER                     │
│  - Local Processing                          │
│  - Data Filtering                            │
│  - Real-time Analytics                       │
│  - Edge ML Inference                         │
└──────────────────┬──────────────────────────┘
                   │ Local Network
┌──────────────────┴──────────────────────────┐
│           DEVICE LAYER                       │
│  - Sensors                                   │
│  - Actuators                                 │
│  - Controllers                               │
└─────────────────────────────────────────────┘
```

### 4.2 Edge Processing

#### 4.2.1 Data Filtering

**Threshold-based filtering**:
```javascript
// Only send data when significant change occurs
const CHANGE_THRESHOLD = 0.5; // degrees

let lastSentTemp = null;

function processTemperature(temp) {
  if (lastSentTemp === null ||
      Math.abs(temp - lastSentTemp) >= CHANGE_THRESHOLD) {
    sendToCloud({ temperature: temp });
    lastSentTemp = temp;
  }
  // Local storage for all readings
  storeLocally({ temperature: temp, timestamp: Date.now() });
}
```

#### 4.2.2 Data Aggregation

**Time-window aggregation**:
```javascript
const WINDOW_SIZE = 300000; // 5 minutes
let readings = [];

function aggregateData() {
  const avg = readings.reduce((a, b) => a + b) / readings.length;
  const min = Math.min(...readings);
  const max = Math.max(...readings);

  sendToCloud({
    temperature: {
      avg, min, max,
      samples: readings.length
    },
    window: WINDOW_SIZE
  });

  readings = [];
}

setInterval(aggregateData, WINDOW_SIZE);
```

### 4.3 Edge Computing Platforms

#### 4.3.1 Platform Comparison

| Platform | Runtime | Languages | Use Case |
|----------|---------|-----------|----------|
| AWS IoT Greengrass | Docker | Python, Node.js, Java | Enterprise edge computing |
| Azure IoT Edge | Docker | C, C#, Python, Node.js | Industrial IoT |
| Google Edge TPU | TensorFlow Lite | Python, C++ | Edge ML inference |
| EdgeX Foundry | Microservices | Go, C | Open-source IoT framework |
| KubeEdge | Kubernetes | Any containerized | Edge orchestration |

---

## 5. LPWAN Technologies

### 5.1 LoRaWAN

#### 5.1.1 Network Architecture

```
End Devices ←→ Gateways ←→ Network Server ←→ Application Server

Device Classes:
- Class A: Bi-directional, lowest power (battery years)
- Class B: Scheduled receive windows (battery months)
- Class C: Continuous listening (mains powered)
```

#### 5.1.2 LoRaWAN Protocol

**Uplink Frame**:
```
┌──────────┬──────────┬─────────┬──────────┬─────┐
│ PHYPayld │ MHDR(1)  │ DevAddr │ FCtrl(1) │ ... │
│          │          │  (4)    │          │     │
└──────────┴──────────┴─────────┴──────────┴─────┘

MHDR: Message Header
  - Message Type (Unconfirmed/Confirmed Data Up/Down)
DevAddr: Device Address (32-bit)
FCtrl: Frame Control
FCnt: Frame Counter (anti-replay)
FOpts: Frame Options
FPort: Port (application identifier)
FRMPayload: Encrypted payload
MIC: Message Integrity Code (4 bytes)
```

**Adaptive Data Rate (ADR)**:
```
Network optimizes:
- Spreading Factor (SF7-SF12)
- Transmission Power
- Data Rate

Goal: Maximize battery life while maintaining connectivity
```

#### 5.1.3 Security

**Keys**:
- **AppKey**: Application key (128-bit AES)
- **NwkSKey**: Network session key (derived)
- **AppSKey**: Application session key (derived)

**Join Procedure (OTAA)**:
```
Device → Gateway: Join Request (DevEUI, AppEUI, DevNonce)
Network Server validates
Network Server → Device: Join Accept (AppNonce, NetID, DevAddr, NwkSKey, AppSKey)
Device derives session keys
```

### 5.2 NB-IoT (Narrowband IoT)

#### 5.2.1 Characteristics

**Network Parameters**:
- **Frequency**: Licensed LTE bands
- **Bandwidth**: 200 kHz
- **Data Rate**: Up to 250 kbps downlink, 20-250 kbps uplink
- **Range**: 10 km (urban), 35 km (rural)
- **Power Saving Mode (PSM)**: Years of battery life
- **Extended Discontinuous Reception (eDRX)**: Optimized power

#### 5.2.2 Use Cases

**Ideal for**:
- Smart metering (water, gas, electricity)
- Asset tracking
- Smart parking
- Environmental monitoring

**Coverage Enhancement**:
```
Repetition levels: 1, 2, 4, 8, 16, 32, 64, 128
Higher repetition = better coverage but lower data rate
```

### 5.3 Sigfox

#### 5.3.1 Ultra-Narrowband Protocol

**Characteristics**:
- **Frequency**: ISM bands (868 MHz EU, 902 MHz US)
- **Bandwidth**: 100 Hz
- **Data Rate**: 100 bps uplink, 600 bps downlink
- **Message Size**: 12 bytes uplink, 8 bytes downlink
- **Messages**: 140 uplink/day, 4 downlink/day
- **Range**: Up to 40 km

#### 5.3.2 Message Format

```
Uplink Message (12 bytes max):
┌──────────┬──────────┬──────────┐
│ Device ID│ Sequence │ Payload  │
│  (4 B)   │  (2 B)   │ (0-12 B) │
└──────────┴──────────┴──────────┘

Encoding example:
Temperature: 2 bytes (signed int16, 0.1°C precision)
Humidity: 1 byte (uint8, 0.5% precision)
Battery: 1 byte (uint8, 0.01V precision)
Status: 1 byte (flags)
```

---

## 6. Device Management

### 6.1 Lifecycle Management

#### 6.1.1 Device States

```
      [Manufacturing]
            ↓
      [Provisioned] ←→ [Inventory]
            ↓
       [Activated] ←→ [Suspended]
            ↓
       [Operating] ←→ [Maintenance]
            ↓
    [Decommissioned]
```

#### 6.1.2 State Transitions

**Provisioning → Activation**:
```json
POST /devices/{id}/activate
{
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "description": "Building A, Floor 2, Room 205"
  },
  "configuration": {
    "reporting_interval": 300,
    "sensors": ["temperature", "humidity"],
    "thresholds": {
      "temperature_high": 30.0,
      "temperature_low": 15.0
    }
  }
}
```

### 6.2 Firmware Over-The-Air (FOTA)

#### 6.2.1 Update Process

**Step 1: Version Check**
```json
Device → Server: Current firmware version
{
  "device_id": "sensor-001",
  "current_version": "1.5.3",
  "hardware_version": "2.0"
}

Server → Device: Update available
{
  "update_available": true,
  "new_version": "1.6.0",
  "url": "https://cdn.example.com/firmware/sensor-v1.6.0.bin",
  "checksum": "sha256:abc123...",
  "size": 245760,
  "critical": false,
  "changelog": "Bug fixes and performance improvements"
}
```

**Step 2: Download**
```
Device downloads firmware in chunks:
GET https://cdn.example.com/firmware/sensor-v1.6.0.bin
Range: bytes=0-4095 (first 4KB chunk)

Device stores in secondary flash/partition
Device verifies checksum incrementally
```

**Step 3: Verification**
```
Verify checksum:
sha256(downloaded_firmware) === expected_checksum

Verify signature (if signed):
verify_signature(firmware, manufacturer_public_key)
```

**Step 4: Installation**
```
1. Mark new firmware as pending
2. Reboot device
3. Bootloader checks pending firmware
4. Bootloader validates and flashes new firmware
5. Boot new firmware
6. New firmware marks itself as good
7. Old firmware kept as fallback
```

**Step 5: Rollback (if needed)**
```
If new firmware fails to boot or crashes:
1. Watchdog timer expires
2. Bootloader detects failure
3. Bootloader reverts to old firmware
4. Device reports update failed
```

### 6.3 Remote Configuration

#### 6.3.1 Configuration Update

```json
MQTT Topic: devices/{id}/config/update

Payload:
{
  "version": 2,
  "config": {
    "reporting_interval": 600,
    "sensors": {
      "temperature": {
        "enabled": true,
        "calibration_offset": -0.5,
        "precision": 0.1
      },
      "humidity": {
        "enabled": true,
        "calibration_offset": 2.0
      }
    },
    "network": {
      "mqtt_keepalive": 60,
      "reconnect_backoff": [1, 2, 5, 10, 30]
    },
    "power": {
      "sleep_mode": "deep",
      "wakeup_interval": 300
    }
  }
}

Device acknowledges:
Topic: devices/{id}/config/ack
{
  "version": 2,
  "applied": true,
  "timestamp": "2025-12-26T10:30:00Z"
}
```

---

## 7. Security and Authentication

### 7.1 Transport Layer Security

#### 7.1.1 TLS/DTLS Configuration

**Minimum Requirements**:
- TLS 1.2 or higher for TCP-based protocols
- DTLS 1.2 or higher for UDP-based protocols (CoAP)
- Strong cipher suites only

**Recommended Cipher Suites**:
```
TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305_SHA256
TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256
```

**Avoid**:
- TLS 1.0, 1.1 (deprecated)
- SSL 3.0 and earlier (insecure)
- RC4, DES, 3DES ciphers (weak)
- Anonymous or NULL cipher suites

### 7.2 Device Authentication

#### 7.2.1 Certificate-Based Authentication

**Mutual TLS (mTLS)**:
```
Client (Device) → Server (Broker)
  ClientHello
                ← ServerHello, Certificate, CertificateRequest
  Certificate, CertificateVerify, Finished
                ← Finished

Both parties verify peer certificates against trusted CAs
```

**Certificate Pinning**:
```c
// Device pins server certificate or CA cert
const char* server_cert_fingerprint =
  "A1:B2:C3:D4:E5:F6:...:FF";

if (verify_cert_fingerprint(server_cert, server_cert_fingerprint)) {
  // Proceed with connection
} else {
  // Reject connection - possible MITM attack
}
```

#### 7.2.2 Token-Based Authentication

**JWT (JSON Web Token)**:
```json
Header:
{
  "alg": "ES256",
  "typ": "JWT"
}

Payload:
{
  "sub": "device-sensor-001",
  "aud": "iot-platform",
  "iat": 1735200000,
  "exp": 1735286400,
  "device_type": "temperature-sensor",
  "permissions": ["telemetry.publish", "commands.subscribe"]
}

Signature:
ECDSA_SHA256(
  base64UrlEncode(header) + "." + base64UrlEncode(payload),
  device_private_key
)
```

**OAuth 2.0 Device Flow**:
```
1. Device requests device code:
   POST /oauth/device/code
   → { "device_code": "abc123", "user_code": "WXYZ-1234",
       "verification_uri": "https://example.com/activate" }

2. User visits verification_uri and enters user_code

3. Device polls for authorization:
   POST /oauth/token
   { "grant_type": "device_code", "device_code": "abc123" }

4. After user approval:
   → { "access_token": "...", "refresh_token": "...", "expires_in": 3600 }
```

### 7.3 Data Encryption

#### 7.3.1 End-to-End Encryption

**Message-Level Encryption**:
```json
// Original message
{
  "temperature": 23.5,
  "humidity": 60.2
}

// Encrypted message
{
  "encrypted": true,
  "algorithm": "AES-256-GCM",
  "key_id": "key-2025-001",
  "iv": "base64-encoded-iv",
  "ciphertext": "base64-encoded-encrypted-data",
  "tag": "base64-encoded-auth-tag"
}

// Decryption at application server (not broker)
plaintext = AES_GCM_decrypt(
  ciphertext,
  shared_key[key_id],
  iv,
  tag
)
```

#### 7.3.2 Secure Boot

**Boot Chain of Trust**:
```
1. Hardware Root of Trust (ROM Bootloader)
   - Immutable code in device ROM
   - Verifies Stage 1 bootloader signature

2. Stage 1 Bootloader (Signed)
   - Verifies Stage 2 bootloader signature
   - Verified by ROM bootloader

3. Stage 2 Bootloader (Signed)
   - Verifies OS/Application signature
   - Verified by Stage 1

4. Application (Signed)
   - Normal device operation
   - Verified by Stage 2
```

---

## 8. Data Management

### 8.1 Time-Series Data

#### 8.1.1 Data Model

```json
{
  "device_id": "sensor-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "measurements": [
    {
      "metric": "temperature",
      "value": 23.5,
      "unit": "celsius"
    },
    {
      "metric": "humidity",
      "value": 60.2,
      "unit": "percent"
    },
    {
      "metric": "battery",
      "value": 3.7,
      "unit": "volts"
    }
  ],
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "quality": {
    "signal_strength": -65,
    "snr": 12
  }
}
```

#### 8.1.2 Data Retention

**Retention Policies**:
```
Raw data:      7 days (high resolution)
1-min average: 30 days
5-min average: 90 days
1-hour average: 1 year
1-day average: 5 years
```

**Downsampling**:
```sql
-- Aggregate raw data to 5-minute averages
SELECT
  device_id,
  time_bucket('5 minutes', timestamp) AS time,
  AVG(temperature) AS avg_temp,
  MIN(temperature) AS min_temp,
  MAX(temperature) AS max_temp,
  COUNT(*) AS sample_count
FROM telemetry
GROUP BY device_id, time_bucket('5 minutes', timestamp);
```

### 8.2 Data Formats

#### 8.2.1 Payload Encoding

**JSON (Human-readable)**:
```json
{"temp": 23.5, "hum": 60.2, "bat": 3.7}
Size: ~45 bytes
```

**CBOR (Compact Binary)**:
```
a3                      # map(3)
   64                   # text(4)
      74656d70          # "temp"
   f9 48f0              # float16(23.5)
   63                   # text(3)
      68756d            # "hum"
   f9 4f1a              # float16(60.2)
   63                   # text(3)
      626174            # "bat"
   f9 4230              # float16(3.7)

Size: ~24 bytes (47% smaller)
```

**Protocol Buffers**:
```protobuf
message Telemetry {
  float temperature = 1;
  float humidity = 2;
  float battery = 3;
}

Size: ~15 bytes (67% smaller)
```

**Custom Binary (Ultra-compact)**:
```
Temperature: int16 (0.1°C precision) = 235 (0x00EB)
Humidity: uint8 (0.5% precision) = 120 (0x78)
Battery: uint8 (0.01V precision) = 370 (0x172 → 0x72, 1 bit in flags)

Binary: [0x00, 0xEB, 0x78, 0x72] + 1 flag bit
Size: 4 bytes (91% smaller)
```

---

## 9. Digital Twin

### 9.1 Digital Twin Architecture

```
┌─────────────────────────────────┐
│      Physical Device            │
│  - Sensors                      │
│  - Actuators                    │
│  - State                        │
└────────────┬────────────────────┘
             │ Telemetry
             │ State Updates
             ↓
┌─────────────────────────────────┐
│      Digital Twin               │
│  - Reported State               │
│  - Desired State                │
│  - Metadata                     │
│  - Historical Data              │
└────────────┬────────────────────┘
             │ Commands
             │ Config Updates
             ↓
┌─────────────────────────────────┐
│      Applications               │
│  - Monitoring                   │
│  - Analytics                    │
│  - Control                      │
└─────────────────────────────────┘
```

### 9.2 Device Shadow

#### 9.2.1 Shadow Document

```json
{
  "state": {
    "reported": {
      "temperature": 23.5,
      "humidity": 60.2,
      "power_mode": "normal",
      "firmware_version": "1.5.3",
      "uptime": 86400,
      "last_boot": "2025-12-25T10:30:00Z"
    },
    "desired": {
      "power_mode": "eco",
      "reporting_interval": 600
    }
  },
  "metadata": {
    "reported": {
      "temperature": {
        "timestamp": "2025-12-26T10:30:00Z"
      },
      "humidity": {
        "timestamp": "2025-12-26T10:30:00Z"
      }
    },
    "desired": {
      "power_mode": {
        "timestamp": "2025-12-26T10:25:00Z"
      }
    }
  },
  "version": 127,
  "timestamp": "2025-12-26T10:30:05Z"
}
```

#### 9.2.2 Shadow Synchronization

**Device publishes state update**:
```json
Topic: $aws/things/{thingName}/shadow/update

{
  "state": {
    "reported": {
      "temperature": 23.5,
      "humidity": 60.2
    }
  }
}
```

**Server publishes desired state**:
```json
Topic: $aws/things/{thingName}/shadow/update

{
  "state": {
    "desired": {
      "power_mode": "eco"
    }
  }
}
```

**Device receives delta**:
```json
Topic: $aws/things/{thingName}/shadow/update/delta

{
  "state": {
    "power_mode": "eco"
  },
  "metadata": {
    "power_mode": {
      "timestamp": "2025-12-26T10:25:00Z"
    }
  },
  "version": 128
}
```

**Device applies change and reports**:
```json
Topic: $aws/things/{thingName}/shadow/update

{
  "state": {
    "reported": {
      "power_mode": "eco"
    }
  }
}
```

---

## 10. Interoperability

### 10.1 Data Models

#### 10.1.1 IPSO Smart Objects

**Standard Object IDs**:
```
3300: Temperature Sensor
3301: Humidity Sensor
3302: Light Control
3303: Presence Sensor
3304: Accelerometer
3305: Barometer
3306: Actuation
3311: Light Sensor
3315: Voltmeter
3320: Magnetometer
3321: Push Button
```

**Example - Temperature Sensor (3300)**:
```json
{
  "/3300/0": {
    "5700": 23.5,        // Sensor Value
    "5701": "Cel",       // Sensor Units
    "5601": 15.0,        // Min Measured Value
    "5602": 30.0,        // Max Measured Value
    "5603": -10.0,       // Min Range Value
    "5604": 50.0,        // Max Range Value
    "5750": "Temperature" // Application Type
  }
}
```

### 10.2 Protocol Translation

#### 10.2.1 MQTT to CoAP Gateway

```
MQTT Topic: sensors/building-a/temp-01
  → CoAP Resource: coap://gateway/sensors/building-a/temp-01

MQTT Publish QoS 0
  → CoAP NON (Non-confirmable)

MQTT Publish QoS 1
  → CoAP CON (Confirmable)

MQTT Subscribe
  → CoAP Observe
```

---

## 11. Implementation Guidelines

### 11.1 Device Implementation

#### 11.1.1 Power Management

**Sleep Modes**:
```c
// Deep sleep with periodic wake
void enter_deep_sleep(uint32_t duration_sec) {
  // Disconnect from network
  mqtt_disconnect();
  wifi_disconnect();

  // Configure wake source
  esp_sleep_enable_timer_wakeup(duration_sec * 1000000ULL);

  // Optional: Wake on external interrupt (motion sensor)
  esp_sleep_enable_ext0_wakeup(GPIO_MOTION, 1);

  // Enter deep sleep
  esp_deep_sleep_start();
}

void loop() {
  // Read sensors
  float temp = read_temperature();
  float hum = read_humidity();

  // Publish data
  publish_telemetry(temp, hum);

  // Sleep for 5 minutes
  enter_deep_sleep(300);
}
```

#### 11.1.2 Error Handling

**Retry Logic with Exponential Backoff**:
```javascript
async function publishWithRetry(topic, message, maxRetries = 5) {
  let attempt = 0;
  let delay = 1000; // Start with 1 second

  while (attempt < maxRetries) {
    try {
      await mqtt.publish(topic, message);
      return; // Success
    } catch (error) {
      attempt++;
      if (attempt >= maxRetries) {
        console.error('Max retries exceeded', error);
        // Store locally for later transmission
        localStorage.save(topic, message);
        return;
      }

      // Exponential backoff: 1s, 2s, 4s, 8s, 16s
      await sleep(delay);
      delay *= 2;
    }
  }
}
```

### 11.2 Gateway Implementation

#### 11.2.1 Protocol Bridging

```python
# MQTT-to-HTTP bridge
import paho.mqtt.client as mqtt
import requests

def on_message(client, userdata, message):
    topic = message.topic
    payload = message.payload.decode()

    # Extract device ID from topic
    # Topic format: devices/{device_id}/telemetry
    device_id = topic.split('/')[1]

    # Forward to HTTP REST API
    try:
        response = requests.post(
            f'https://api.example.com/devices/{device_id}/telemetry',
            json=json.loads(payload),
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        response.raise_for_status()
    except Exception as e:
        print(f"Error forwarding message: {e}")
        # Implement retry queue

client = mqtt.Client()
client.on_message = on_message
client.connect("mqtt-broker.local", 1883)
client.subscribe("devices/+/telemetry")
client.loop_forever()
```

---

## 12. References

### 12.1 Protocol Standards

1. **MQTT**: OASIS MQTT Version 5.0 (2019)
2. **CoAP**: RFC 7252 - Constrained Application Protocol (2014)
3. **AMQP**: ISO/IEC 19464:2014 - Advanced Message Queuing Protocol 1.0
4. **HTTP/2**: RFC 7540 - Hypertext Transfer Protocol Version 2 (2015)
5. **WebSocket**: RFC 6455 - The WebSocket Protocol (2011)

### 12.2 Security Standards

6. **TLS 1.3**: RFC 8446 - Transport Layer Security (2018)
7. **DTLS 1.2**: RFC 6347 - Datagram Transport Layer Security (2012)
8. **X.509**: RFC 5280 - Internet X.509 Public Key Infrastructure (2008)
9. **JWT**: RFC 7519 - JSON Web Token (2015)
10. **OAuth 2.0**: RFC 6749 - OAuth 2.0 Authorization Framework (2012)

### 12.3 IoT Standards

11. **LwM2M**: OMA LightweightM2M v1.2 - Device Management Protocol
12. **oneM2M**: ETSI TS 118 101 - IoT Standards and Architecture
13. **OCF**: Open Connectivity Foundation - IoT Interoperability
14. **Thread**: Thread 1.3.0 - IPv6-based Mesh Networking
15. **Zigbee**: Zigbee 3.0 - Wireless Mesh Networking

### 12.4 LPWAN Standards

16. **LoRaWAN**: LoRa Alliance - LoRaWAN 1.0.4 Specification
17. **NB-IoT**: 3GPP Release 13 - Narrowband IoT
18. **LTE-M**: 3GPP Release 13 - LTE for Machines
19. **Sigfox**: Sigfox Technical Overview v1.3

### 12.5 Data Formats

20. **JSON**: RFC 8259 - JavaScript Object Notation (2017)
21. **CBOR**: RFC 8949 - Concise Binary Object Representation (2020)
22. **Protocol Buffers**: Google Protocol Buffers v3
23. **MessagePack**: MessagePack Specification

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
