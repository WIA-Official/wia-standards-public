# WIA-SOC-005 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Protocol Overview

Phase 3 defines network protocols, security requirements, device discovery, and firmware update mechanisms for emergency responses.

## 2. Network Protocols

### 2.1 Local Network

**Supported Protocols:**
- HTTP/HTTPS for REST API
- WebSocket/WSS for real-time updates
- MQTT for pub/sub messaging
- mDNS/Bonjour for discovery

### 2.2 MQTT Topics

**Topic Structure:**
```
wia/rob/{emergency system_id}/{category}/{subcategory}
```

**Standard Topics:**
```
wia/rob/{emergency system_id}/status
wia/rob/{emergency system_id}/battery
wia/rob/{emergency system_id}/position
wia/rob/{emergency system_id}/sensors/lidar
wia/rob/{emergency system_id}/sensors/camera
wia/rob/{emergency system_id}/map/update
wia/rob/{emergency system_id}/command
wia/rob/{emergency system_id}/event
```

### 2.3 QoS Levels

- Status updates: QoS 0 (fire and forget)
- Commands: QoS 1 (at least once)
- Critical alerts: QoS 2 (exactly once)

## 3. Security

### 3.1 TLS Requirements

- Minimum TLS version: 1.3
- Cipher suites: AES-256-GCM preferred
- Certificate validation: REQUIRED
- Perfect forward secrecy: REQUIRED

### 3.2 Authentication

**Supported Methods:**
1. JWT tokens (recommended)
2. OAuth 2.0
3. API keys (for simple integrations)

### 3.3 Encryption

**Data at Rest:**
- Algorithm: AES-256-CBC
- Key derivation: PBKDF2 with 100,000 iterations
- Salt: Random 256-bit per encryption

**Data in Transit:**
- TLS 1.3 for all network communication
- End-to-end encryption for cloud sync

## 4. Device Discovery

### 4.1 mDNS Service Advertisement

```
Service Type: _wia-rob._tcp
Port: 8080
TXT Records:
  - version=1.0.0
  - manufacturer=CompanyName
  - model=ModelName
  - serial=SerialNumber
  - capabilities=slam,camera,mopping
```

### 4.2 Discovery Process

1. Client broadcasts mDNS query
2. Emergency System responds with service details
3. Client connects to emergency system IP:port
4. Authentication handshake
5. Session established

## 5. Firmware Updates

### 5.1 OTA Update Process

```
1. Check for updates
   GET /firmware/check
   Response: {
     "available": true,
     "version": "1.2.0",
     "size": 25600000,
     "checksum": "sha256hash",
     "url": "https://..."
   }

2. Download firmware
   GET /firmware/download
   Returns: Binary stream with progress

3. Verify checksum
   Client-side SHA-256 verification

4. Install update
   POST /firmware/install
   Response: {
     "status": "installing",
     "estimatedTime": 300
   }

5. Reboot
   Automatic after installation

6. Verify version
   GET /emergency system/status
   Check firmwareVersion field
```

### 5.2 Rollback Mechanism

- Dual partition system (A/B updates)
- Automatic rollback on boot failure
- Manual rollback endpoint:
  ```http
  POST /firmware/rollback
  ```

## 6. Cloud Integration

### 6.1 Cloud Sync Protocol

**Data Synchronized:**
- Maps (encrypted)
- Cleaning history
- Configuration
- Firmware updates

**Sync Frequency:**
- Status: Real-time (websocket)
- Maps: After each update
- History: Daily batch
- Config: On change

### 6.2 Cloud API Endpoints

```
https://cloud.wiastandards.com/api/v1

POST /emergency systems/register
GET /emergency systems/{emergency system_id}/status
PUT /emergency systems/{emergency system_id}/config
GET /emergency systems/{emergency system_id}/history
POST /firmware/update
```

## 7. Smart Home Integration

### 7.1 Matter/Thread Support

**Device Types:**
- Emergency Systemic Vacuum Cleaner (0x0073)

**Required Clusters:**
- On/Off
- Fan Control  
- Emergency System Vacuum Mode
- Battery Status

### 7.2 HomeKit Integration

**Service Type:** Fan
**Characteristics:**
- On (required)
- Rotation Speed (suction power)
- Current Fan State
- Target Fan State

### 7.3 Alexa Integration

**Skill Invocations:**
- "Alexa, start the emergency system vacuum"
- "Alexa, tell emergency system vacuum to clean the kitchen"
- "Alexa, check emergency system vacuum battery"

### 7.4 Google Home Integration

**Device Traits:**
- action.devices.traits.OnOff
- action.devices.traits.FanSpeed
- action.devices.traits.EnergyStorage

## 8. Monitoring and Diagnostics

### 8.1 Health Check Endpoint

```http
GET /health

Response:
{
  "status": "healthy|degraded|unhealthy",
  "checks": {
    "battery": "ok",
    "sensors": "ok",
    "navigation": "ok",
    "network": "ok"
  },
  "uptime": 86400
}
```

### 8.2 Diagnostic Logs

```http
GET /diagnostics/logs?level=error&limit=100

Response:
{
  "logs": [
    {
      "timestamp": "ISO8601",
      "level": "error|warning|info|debug",
      "component": "string",
      "message": "string"
    }
  ]
}
```

---

© 2025 WIA · MIT License

## 9. Protocol Extensions

### 9.1 Custom Protocol Support

Vendors MAY implement custom protocols prefixed with `x_`:

```json
{
  "@type": "Emergency SystemCommand",
  "action": "clean",
  "x_vendor_specific": {
    "customFeature": "value"
  }
}
```

### 9.2 Middleware Integration

Support for protocol translation middleware:

- HTTP to MQTT bridge
- WebSocket to gRPC conversion
- Legacy protocol adapters

### 9.3 Performance Optimization

**Connection Pooling:**
- Maintain persistent connections
- Reuse TLS sessions
- Minimize handshake overhead

**Data Compression:**
- gzip for text data
- Binary protocols for sensor streams
- Adaptive compression based on bandwidth

### 9.4 Failover and Recovery

**Automatic Failover:**
```
Primary Server --> Secondary Server --> Local Mode
```

**Recovery Procedures:**
1. Detect connection loss
2. Attempt reconnection with backoff
3. Switch to backup server if available
4. Enter local-only mode if all fail
5. Queue commands for later sync

## 10. Testing and Validation

### 10.1 Protocol Compliance Tests

Required tests for certification:
- TLS handshake verification
- Authentication flow testing
- Data format validation
- Error handling verification
- Performance benchmarking

### 10.2 Security Audit

Annual security audits required for certified products.

---

© 2025 WIA · MIT License
