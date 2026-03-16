# WIA-SOC-008 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines communication protocols, security standards, device discovery, and firmware update mechanisms for water supply IoT infrastructure.

## 2. Supported Protocols

### 2.1 MQTT

**Broker Requirements:**
- MQTT 3.1.1 or 5.0
- TLS 1.3 encryption
- Authentication required
- QoS levels: 0, 1, 2 supported

**Topic Structure:**
```
wia/soc-008/{systemId}/{deviceType}/{deviceId}/{dataType}

Examples:
wia/soc-008/WS-2025-001/sensor/SENSOR-042/water-quality
wia/soc-008/WS-2025-001/meter/METER-123/consumption
wia/soc-008/WS-2025-001/pump/PUMP-07/status
```

### 2.2 CoAP

For low-power devices:
- CoAP RFC 7252
- DTLS 1.3 for security
- Observable resources
- Block-wise transfer

### 2.3 OPC UA

For industrial SCADA integration:
- OPC UA specification Part 1-14
- UA Binary or UA JSON encoding
- Certificate-based security
- Historical data access

## 3. Security

### 3.1 Encryption

**Transport Security:**
- TLS 1.3 (REQUIRED for internet)
- DTLS 1.3 (REQUIRED for UDP protocols)
- Minimum cipher suite: TLS_AES_128_GCM_SHA256
- Perfect forward secrecy (PFS) enabled

**Application Security:**
- End-to-end encryption for sensitive data
- Payload encryption: AES-256-GCM
- Key rotation: Every 90 days

### 3.2 Authentication

**Device Authentication:**
- X.509 certificates (RECOMMENDED)
- Pre-shared keys (PSK) for constrained devices
- OAuth 2.0 for cloud services

**User Authentication:**
- Multi-factor authentication (MFA) for operators
- Single sign-on (SSO) integration
- Role-based access control (RBAC)

### 3.3 Network Security

**Segmentation:**
- Isolated OT (Operational Technology) network
- DMZ for external access
- VLAN separation for different device types

**Firewall Rules:**
- Default deny all
- Whitelist approach for allowed traffic
- Intrusion detection systems (IDS)

## 4. Device Discovery

### 4.1 mDNS/DNS-SD

**Service Type:**
```
_wia-water._tcp.local
```

**TXT Records:**
```
txtvers=1
systemId=WS-2025-001
deviceType=sensor
capabilities=water-quality,pressure,flow
certLevel=STANDARD
version=1.0.0
```

### 4.2 DDS Discovery

For real-time systems:
- OMG DDS specification
- Quality of Service (QoS) policies
- Content-filtered topics

## 5. Data Transmission

### 5.1 Message Format

**MQTT Payload (JSON):**
```json
{
  "v": "1.0.0",
  "ts": "2025-12-26T14:32:15Z",
  "id": "SENSOR-042",
  "type": "water-quality",
  "data": {
    "pH": 7.3,
    "turbidity": 0.8,
    "chlorine": 0.5
  },
  "sig": "base64-signature"
}
```

### 5.2 Compression

- Gzip for large payloads (> 1KB)
- Protocol Buffers for high-frequency data
- CBOR for constrained devices

### 5.3 Batching

- Multiple readings in single message
- Maximum batch size: 100 readings or 1MB
- Maximum batch interval: 5 minutes

## 6. Firmware Updates

### 6.1 OTA (Over-The-Air) Updates

**Update Process:**
1. Version check
2. Download firmware package
3. Verify digital signature
4. Install during maintenance window
5. Rollback on failure

**Security:**
- Code signing (RSA 4096 or ECC P-384)
- Encrypted firmware images
- Secure boot verification

### 6.2 Update Scheduling

- Configurable maintenance windows
- Staggered rollout (10% → 50% → 100%)
- Automatic rollback threshold: 5% failure rate

## 7. Reliability

### 7.1 Quality of Service

| Data Type | QoS Level | Reliability |
|-----------|-----------|-------------|
| Critical Alerts | QoS 2 | Exactly once |
| Water Quality | QoS 1 | At least once |
| Telemetry | QoS 0 | At most once |

### 7.2 Offline Operation

- Local data buffering (minimum 24 hours)
- Automatic reconnection with exponential backoff
- Data synchronization on reconnection

### 7.3 Redundancy

- Dual-path communication (primary + backup)
- Automatic failover (< 30 seconds)
- Health monitoring and heartbeats

## 8. Performance

| Metric | Requirement |
|--------|-------------|
| Message Latency | < 500ms (p95) |
| Throughput | > 10,000 msg/sec per broker |
| Concurrent Connections | > 100,000 devices |
| Message Size | < 256KB |
| Bandwidth | Optimized for 2G/3G/4G/5G/LoRaWAN |

## 9. Compliance

- IEC 62443 (Industrial cybersecurity)
- NIST Cybersecurity Framework
- ISO/IEC 27001 (Information security)
- Local regulatory requirements

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc. · MIT License
