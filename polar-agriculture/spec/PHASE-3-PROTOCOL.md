# WIA-AGRI-022 Polar Agriculture - Phase 3: Protocol

> **Standard ID:** WIA-AGRI-022
> **Phase:** 3 - Communication Protocol Specification
> **Version:** 1.0.0
> **Status:** ✅ Complete
> **Last Updated:** 2025-12-26

---

## 1. Overview

The WIA-AGRI-022 Protocol defines the communication standards for polar agriculture systems, ensuring reliable data transmission in extreme cold environments with limited connectivity.

### 1.1 Protocol Design Goals

1. **Resilience:** Operate reliably in harsh polar conditions
2. **Efficiency:** Minimize bandwidth usage for satellite/radio links
3. **Security:** Encrypt sensitive operational data
4. **Failsafe:** Continue operation during communication outages
5. **Interoperability:** Compatible with WIA ecosystem standards

### 1.2 Transport Layers Supported

| Layer | Use Case | Reliability | Bandwidth |
|-------|----------|-------------|-----------|
| **HTTPS/REST** | Primary internet connectivity | High | High |
| **MQTT** | IoT sensors, real-time updates | Medium | Low |
| **LoRaWAN** | Long-range sensors, outdoor | Low | Very Low |
| **Satellite** | Remote polar stations | Medium | Limited |
| **Local Storage** | Offline operation | N/A | N/A |

---

## 2. Message Format

### 2.1 Standard Message Structure

All protocol messages follow this envelope format:

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageId": "uuid-v4",
    "messageType": "CLIMATE_UPDATE | SENSOR_READING | ALERT | COMMAND | RESPONSE",
    "timestamp": "ISO8601 datetime",
    "priority": "LOW | MEDIUM | HIGH | EMERGENCY",
    "farmId": "string",
    "senderId": "string",
    "encryption": "none | AES-256 | RSA-2048"
  },
  "payload": {
    // Message-specific data
  },
  "checksum": "SHA-256 hash of payload"
}
```

### 2.2 Message Types

| Type | Direction | Description | Priority |
|------|-----------|-------------|----------|
| `CLIMATE_UPDATE` | Farm → Server | Climate sensor readings | MEDIUM |
| `SENSOR_READING` | Farm → Server | Individual sensor data | LOW |
| `ALERT` | Farm → Server | Alert/warning notifications | HIGH/EMERGENCY |
| `COMMAND` | Server → Farm | Control commands | HIGH |
| `RESPONSE` | Farm → Server | Command acknowledgment | MEDIUM |
| `HEARTBEAT` | Bidirectional | System alive signal | LOW |
| `HARVEST_EVENT` | Farm → Server | Harvest completion | MEDIUM |

---

## 3. Climate Update Protocol

### 3.1 Message Format

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageId": "550e8400-e29b-41d4-a716-446655440000",
    "messageType": "CLIMATE_UPDATE",
    "timestamp": "2025-12-26T15:30:00Z",
    "priority": "MEDIUM",
    "farmId": "POLAR-FARM-SVB-001",
    "senderId": "climate-controller-01",
    "encryption": "AES-256"
  },
  "payload": {
    "externalTemp": -42.3,
    "internalTemp": 22.1,
    "targetTemp": 22.0,
    "humidity": 65,
    "co2Level": 1200,
    "airFlow": 1500.0,
    "pressure": 101.2,
    "dewPoint": -44.8
  },
  "checksum": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
}
```

### 3.2 Update Frequency

| Condition | Frequency | Reasoning |
|-----------|-----------|-----------|
| Normal operation | Every 5 minutes | Balance data freshness and bandwidth |
| Temperature deviation > 2°C | Every 1 minute | Close monitoring required |
| Emergency alert active | Every 30 seconds | Real-time monitoring critical |
| Offline mode | Store locally, sync when online | Resilience |

---

## 4. Alert Protocol

### 4.1 Alert Message Format

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageId": "alert-e29b-41d4-a716-446655440001",
    "messageType": "ALERT",
    "timestamp": "2025-12-26T15:35:00Z",
    "priority": "EMERGENCY",
    "farmId": "POLAR-FARM-SVB-001",
    "senderId": "safety-monitor-01",
    "encryption": "AES-256"
  },
  "payload": {
    "alertType": "TEMPERATURE_CRITICAL",
    "severity": "EMERGENCY",
    "description": "Internal temperature dropped below safe threshold",
    "data": {
      "currentTemp": 12.5,
      "targetTemp": 22.0,
      "externalTemp": -55.0,
      "heatingSystemStatus": "FAILURE"
    },
    "recommendedAction": "Activate backup heating immediately",
    "cropRisk": "HIGH - Lettuce may suffer cold damage within 30 minutes",
    "autoResponse": {
      "activated": true,
      "action": "BACKUP_HEATING_ENGAGED",
      "timestamp": "2025-12-26T15:35:02Z"
    }
  },
  "checksum": "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592"
}
```

### 4.2 Alert Severity Escalation

```
LOW (Info) → MEDIUM (Warning) → HIGH (Critical) → EMERGENCY
     ↓              ↓                  ↓                ↓
  Log only      Email alert      SMS alert      Auto-response +
                                               Multiple channels
```

### 4.3 Alert Types

| Alert Type | Trigger Condition | Auto-Response Available |
|------------|------------------|------------------------|
| `TEMPERATURE_CRITICAL` | Internal temp < 15°C or > 30°C | Yes (backup heating/cooling) |
| `POWER_FAILURE` | Main power loss detected | Yes (switch to battery) |
| `CO2_OUT_OF_RANGE` | CO₂ < 400ppm or > 2000ppm | Yes (ventilation adjustment) |
| `HUMIDITY_EXTREME` | Humidity < 30% or > 90% | Yes (humidifier/dehumidifier) |
| `CROP_DISEASE` | ML detection of disease | No (operator review required) |
| `WATER_LEAK` | Water sensor triggered | Yes (emergency shutdown) |

---

## 5. Command Protocol

### 5.1 Command Message Format

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageId": "cmd-a716-446655440002",
    "messageType": "COMMAND",
    "timestamp": "2025-12-26T15:40:00Z",
    "priority": "HIGH",
    "farmId": "POLAR-FARM-SVB-001",
    "senderId": "operator-dashboard",
    "encryption": "RSA-2048"
  },
  "payload": {
    "commandType": "SET_CLIMATE",
    "parameters": {
      "targetTemp": 23.0,
      "humidity": 70,
      "co2Level": 1300
    },
    "validUntil": "2025-12-26T16:40:00Z",
    "requiresAck": true
  },
  "checksum": "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"
}
```

### 5.2 Command Types

| Command | Parameters | Response Expected |
|---------|-----------|------------------|
| `SET_CLIMATE` | targetTemp, humidity, co2Level | ACK + status |
| `EMERGENCY_SHUTDOWN` | reason | ACK + shutdown confirmation |
| `HARVEST_READY` | cropId, estimatedYield | ACK |
| `UPDATE_PHOTOPERIOD` | hours, startTime | ACK |
| `ACTIVATE_BACKUP` | system (heating/power) | ACK + activation status |
| `RUN_DIAGNOSTICS` | systemComponent | ACK + diagnostic report |

### 5.3 Command Acknowledgment

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageId": "resp-446655440003",
    "messageType": "RESPONSE",
    "timestamp": "2025-12-26T15:40:05Z",
    "priority": "MEDIUM",
    "farmId": "POLAR-FARM-SVB-001",
    "senderId": "climate-controller-01",
    "encryption": "AES-256"
  },
  "payload": {
    "commandId": "cmd-a716-446655440002",
    "status": "SUCCESS",
    "message": "Climate parameters updated successfully",
    "data": {
      "targetTemp": 23.0,
      "currentTemp": 22.1,
      "estimatedAdjustmentTime": "12 minutes"
    }
  },
  "checksum": "..."
}
```

---

## 6. MQTT Topics

### 6.1 Topic Hierarchy

```
wia/polar-agriculture/
├── {farmId}/
│   ├── climate/update       (Sensor readings)
│   ├── climate/alert        (Climate alerts)
│   ├── crops/status         (Crop updates)
│   ├── energy/stats         (Energy consumption)
│   ├── commands/incoming    (Commands to farm)
│   ├── commands/response    (Command responses)
│   └── heartbeat            (System alive)
└── broadcast/
    ├── emergency            (Emergency broadcasts)
    └── updates              (System updates)
```

### 6.2 MQTT QoS Levels

| Topic | QoS | Reason |
|-------|-----|--------|
| `climate/update` | 1 (At least once) | Important but tolerable duplicates |
| `climate/alert` | 2 (Exactly once) | Critical, no duplicates allowed |
| `commands/incoming` | 2 (Exactly once) | Commands must not duplicate |
| `heartbeat` | 0 (At most once) | Frequent, loss acceptable |

### 6.3 MQTT Retained Messages

```javascript
// Last known good state retained for reconnection
{
  "topic": "wia/polar-agriculture/POLAR-FARM-SVB-001/climate/update",
  "retain": true,
  "payload": {
    "externalTemp": -42.3,
    "internalTemp": 22.1,
    "timestamp": "2025-12-26T15:30:00Z"
  }
}
```

---

## 7. Offline Operation

### 7.1 Local Data Storage

When internet connectivity is lost, the system must:

1. ✅ Continue autonomous operation
2. ✅ Store all sensor readings locally (SQLite/TimescaleDB)
3. ✅ Buffer alerts for later transmission
4. ✅ Execute critical auto-responses
5. ✅ Sync data upon reconnection

### 7.2 Data Synchronization Protocol

```json
{
  "header": {
    "messageType": "BULK_SYNC",
    "priority": "MEDIUM",
    "farmId": "POLAR-FARM-SVB-001"
  },
  "payload": {
    "offlinePeriod": {
      "start": "2025-12-25T10:00:00Z",
      "end": "2025-12-26T08:00:00Z"
    },
    "recordCount": 1440,
    "dataPoints": [
      {
        "timestamp": "2025-12-25T10:00:00Z",
        "type": "CLIMATE_UPDATE",
        "data": { "externalTemp": -40.2, "internalTemp": 22.0 }
      },
      // ... compressed batch data
    ],
    "compression": "gzip",
    "checksum": "..."
  }
}
```

---

## 8. Security Protocol

### 8.1 Encryption Standards

| Data Sensitivity | Encryption Method | Key Size |
|-----------------|------------------|----------|
| Public data (location, crops) | None | N/A |
| Operational data | AES-256-GCM | 256-bit |
| Commands | RSA-2048 + AES-256 | 2048-bit |
| Authentication tokens | HMAC-SHA256 | 256-bit |

### 8.2 Message Signing

```javascript
// Calculate checksum
const payload = JSON.stringify(message.payload);
const checksum = crypto
  .createHash('sha256')
  .update(payload + SECRET_KEY)
  .digest('hex');

message.checksum = checksum;
```

### 8.3 Certificate Management

```
Root CA: WIA-AGRI-ROOT-CA
    ↓
Intermediate CA: WIA-AGRI-022-CA
    ↓
Farm Certificate: POLAR-FARM-SVB-001
    - Valid: 2 years
    - Auto-renewal: 30 days before expiry
    - Revocation: CRL + OCSP
```

---

## 9. Failsafe Mechanisms

### 9.1 Communication Timeout Handling

```javascript
const TIMEOUT_THRESHOLDS = {
  CLIMATE_UPDATE: 10 * 60 * 1000,  // 10 minutes
  HEARTBEAT: 5 * 60 * 1000,         // 5 minutes
  COMMAND_RESPONSE: 30 * 1000       // 30 seconds
};

// If no climate update received in 10 minutes
if (timeSinceLastUpdate > TIMEOUT_THRESHOLDS.CLIMATE_UPDATE) {
  // Switch to autonomous mode
  activateLocalControl();
  logAlert({
    type: 'COMMUNICATION_TIMEOUT',
    severity: 'WARNING',
    action: 'Switched to local autonomous control'
  });
}
```

### 9.2 Autonomous Operation Rules

When in autonomous mode (communication lost):

1. ✅ Maintain temperature within ±2°C of last target
2. ✅ Continue photoperiod schedule
3. ✅ Execute scheduled nutrient adjustments
4. ✅ Activate emergency protocols if thresholds exceeded
5. ✅ Log all events for later sync

---

## 10. Protocol Versioning

### 10.1 Version Compatibility Matrix

| Client Version | Server Version | Compatibility |
|----------------|----------------|---------------|
| v1.0.x | v1.0.x | ✅ Full |
| v1.0.x | v1.1.x | ✅ Backward compatible |
| v1.1.x | v1.0.x | ⚠️ Limited (new features unavailable) |
| v2.0.x | v1.x.x | ❌ Incompatible |

### 10.2 Version Negotiation

```json
{
  "header": {
    "messageType": "VERSION_HANDSHAKE",
    "clientVersion": "WIA-AGRI-022-v1.0.5",
    "supportedVersions": ["v1.0", "v1.1"]
  },
  "payload": {
    "requestedVersion": "v1.1"
  }
}
```

---

## 11. Performance Metrics

### 11.1 Protocol Performance Requirements

| Metric | Target | Critical Threshold |
|--------|--------|-------------------|
| Message latency | < 1 second | < 5 seconds |
| Packet loss | < 0.1% | < 1% |
| Command response time | < 2 seconds | < 10 seconds |
| Sync after reconnect | < 5 minutes | < 30 minutes |
| Bandwidth usage | < 10 MB/day | < 50 MB/day |

---

## 12. Integration with WIA Ecosystem

### 12.1 Cross-Standard Messaging

```json
{
  "header": {
    "version": "WIA-AGRI-022-v1.0",
    "messageType": "CROSS_STANDARD_EVENT",
    "farmId": "POLAR-FARM-SVB-001"
  },
  "payload": {
    "targetStandard": "WIA-ENERGY-003",
    "eventType": "POWER_CONSUMPTION_UPDATE",
    "data": {
      "totalConsumption": 87.5,
      "breakdown": {
        "heating": 42.0,
        "lighting": 28.5
      }
    }
  }
}
```

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA-AGRI-022 Polar Agriculture Standard - Communication Protocol*
*© 2025 WIA - World Certification Industry Association*
*MIT License*
