# WIA-MED-003 Phase 3: Communication Protocol Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 3 defines the communication protocols for transmitting vital sign data between devices and applications.

## Supported Protocols

1. **Bluetooth Low Energy (BLE)**
2. **WiFi/TCP**
3. **MQTT**
4. **WebSocket**

## Bluetooth LE Protocol

### GATT Service Definition

**Service UUID:** `00001820-0000-1000-8000-00805f9b34fb`

**Characteristics:**

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Device Info | `00002a1e-...` | Read | Manufacturer, model, firmware |
| Signal Config | `00002a1f-...` | Read/Write | Sampling rate, filters |
| Data Stream | `00002a20-...` | Notify | Real-time vital sign data |
| Control | `00002a21-...` | Write | START, STOP, PAUSE commands |
| Status | `00002a22-...` | Notify | Battery, quality, errors |

### Connection Parameters

- **Connection Interval:** 20-50 ms (real-time streaming)
- **Slave Latency:** 0-2
- **Supervision Timeout:** 2000 ms

## WiFi/TCP Protocol

### TLS Socket Connection

- **Port:** 8883 (standard MQTT-TLS port)
- **Encryption:** TLS 1.3
- **Certificate:** Device certificate for authentication

### Data Format

Binary packets with header, payload, and CRC:
```
[Header 4 bytes][Payload N bytes][CRC 2 bytes]
```

## MQTT Protocol

### Topic Structure

```
wia-med-003/
  ├─ {deviceId}/
  │   ├─ info                 # Device info (retained)
  │   ├─ status               # Status updates
  │   ├─ data/
  │   │   ├─ ecg             # ECG data
  │   │   ├─ spo2            # SpO2 data
  │   │   └─ bp              # Blood pressure data
  │   ├─ alerts              # Alerts
  │   └─ control             # Control commands
  └─ broadcast/
      └─ discovery           # Device discovery
```

### QoS Levels

- **data/*:** QoS 0 (real-time, next sample coming soon)
- **alerts:** QoS 1 (important but duplicates acceptable)
- **control:** QoS 2 (exactly once execution required)
- **info:** QoS 1 + retained (new subscribers need info)

## WebSocket Protocol

### Connection

```
wss://api.example.com/stream?deviceId={deviceId}&token={jwt}
```

### Message Format

JSON messages for commands and data:

```json
{
  "type": "command|data|ping|pong",
  "payload": { ... }
}
```

### Keep-Alive

Ping/Pong messages every 30 seconds to maintain connection.

## Security Requirements

### Encryption

- **Transport Layer:** TLS 1.3 minimum
- **Data Layer:** AES-256-GCM for sensitive data
- **Key Exchange:** ECDHE (Elliptic Curve Diffie-Hellman Ephemeral)

### Authentication

- **Device:** X.509 certificates or pre-shared keys
- **User:** OAuth 2.0 or JWT tokens
- **Session:** Time-limited session tokens with refresh

### Compliance

- **HIPAA:** Encryption at rest and in transit
- **GDPR:** User consent, data portability, right to deletion
- **PIPEDA:** Canadian privacy requirements

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
