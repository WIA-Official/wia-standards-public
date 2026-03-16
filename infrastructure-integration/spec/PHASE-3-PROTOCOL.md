# WIA-UNI-005 - Phase 3: Protocol

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 3 specifies communication protocols for real-time coordination and control across infrastructure systems. These protocols enable IoT integration, SCADA connectivity, and emergency response coordination.

## 2. IoT Sensor Networks

### 2.1 MQTT Protocol

**Broker Configuration:**
- MQTT v5.0 with TLS 1.3 encryption
- QoS levels: 0 (at most once), 1 (at least once), 2 (exactly once)
- Clean session: false (persistent sessions)
- Keep-alive: 60 seconds

**Topic Structure:**
```
wia/uni/{region}/{assetType}/{assetId}/{dataType}
```

**Example:**
```
wia/uni/seoul/railway/track-001/sensors/temperature
```

### 2.2 CoAP for Constrained Devices

- CoAP over DTLS for security
- Confirmable messages for critical data
- Observe option for real-time updates
- Block-wise transfer for large payloads

## 3. SCADA Integration

### 3.1 IEC 60870-5-104 (Power Systems)

**Connection Parameters:**
- Port: 2404 (default)
- ASDU address: 16-bit
- Common address: 16-bit
- Information object address: 24-bit

**Message Types:**
| Type ID | Description | Usage |
|---------|-------------|-------|
| M_SP_TB_1 | Single point with time tag | Status monitoring |
| M_ME_TF_1 | Measured value, short float | Sensor readings |
| C_SC_TA_1 | Single command with time | Control commands |

### 3.2 DNP3 Protocol

- DNP3 over TCP/IP
- Secure authentication (SAv5)
- Unsolicited responses enabled
- Event buffer size: 1000 events

## 4. Railway Signaling

### 4.1 ERTMS/ETCS Integration

**Level 2 Specification:**
- Eurobalise for track-to-train data transfer
- GSM-R for train-to-control center
- Movement authority via radio block center
- Automatic train protection (ATP)

**Message Format:**
- Packets: 255 types defined in SUBSET-026
- Transmission: Binary coded
- Error detection: CRC-16

## 5. Emergency Communication

### 5.1 Priority Messaging

**Priority Levels:**
1. Emergency (immediate delivery)
2. Critical (< 1 second)
3. High (< 5 seconds)
4. Normal (< 30 seconds)
5. Low (best effort)

### 5.2 Alert Distribution Protocol

```json
{
  "alertId": "wia:alert:2025-001",
  "priority": "emergency",
  "type": "infrastructure-failure",
  "location": {
    "assetId": "wia:uni:asset:bridge-001",
    "coordinates": [37.5665, 126.9780]
  },
  "timestamp": "2025-12-25T10:30:00Z",
  "message": "Structural integrity alert",
  "actions": ["evacuate", "stop-traffic"],
  "recipients": ["emergency-services", "traffic-control"]
}
```

## 6. Security Protocols

### 6.1 TLS 1.3 Requirements

- Cipher suites: TLS_AES_256_GCM_SHA384 (mandatory)
- Certificate validation: X.509v3
- Perfect forward secrecy (PFS)
- Session resumption with PSK

### 6.2 Certificate Management

- Certificate authority: WIA Infrastructure PKI
- Certificate lifetime: 2 years (max)
- Automated renewal: 30 days before expiry
- Revocation: OCSP stapling

## 7. Time Synchronization

### 7.1 NTP/PTP Requirements

- Primary: PTPv2 (IEEE 1588-2019)
- Fallback: NTPv4 with authentication
- Accuracy: ±1 microsecond (PTP), ±1 millisecond (NTP)
- Stratum: 1 or 2 for critical systems

## 8. Data Compression

- Algorithm: Zstandard (zstd)
- Compression level: 3 (default), 9 (archival)
- Use cases: Historical data, bulk transfers
- Minimum size: 1KB (don't compress smaller)

## 9. Quality of Service

### 9.1 Network Requirements

| Service Type | Bandwidth | Latency | Jitter | Packet Loss |
|-------------|-----------|---------|---------|-------------|
| Real-time control | 100 Kbps | < 10ms | < 2ms | < 0.01% |
| Sensor data | 10 Kbps | < 100ms | < 10ms | < 0.1% |
| Video surveillance | 5 Mbps | < 500ms | < 50ms | < 1% |
| Bulk data transfer | Best effort | N/A | N/A | < 5% |

## 10. Protocol Testing

Certification requires:
- Protocol conformance testing
- Interoperability testing with reference implementation
- Security penetration testing
- Performance benchmarking
- Failover and recovery testing

---

**© 2025 SmileStory Inc. / WIA**  
**弘益人間 (홍익인간) · Benefit All Humanity**
