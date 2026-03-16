# WIA-AGRI-035: Space Agriculture Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines communication protocols, operational procedures, and safety standards for space agriculture systems operating in microgravity, lunar, and Martian environments.

### 1.1 Protocol Objectives

- **Mission Safety**: Ensure crew safety and life support system integrity
- **Data Reliability**: Guaranteed delivery of critical telemetry
- **Bandwidth Efficiency**: Minimize communication overhead
- **Autonomous Operation**: Enable independent operation during communication blackouts
- **Interoperability**: Support multiple space agencies and commercial operators

---

## 2. Communication Protocol Stack

### 2.1 Protocol Layers

```
┌─────────────────────────────────────┐
│     Application Layer               │
│  (WIA-AGRI-035 Data Format)        │
├─────────────────────────────────────┤
│     Message Layer                   │
│  (MQTT 5.0, HTTP/2, WebSocket)     │
├─────────────────────────────────────┤
│     Transport Layer                 │
│  (TLS 1.3, DTLS 1.3)               │
├─────────────────────────────────────┤
│     Network Layer                   │
│  (SpaceIP, DTN, TCP/IP)            │
├─────────────────────────────────────┤
│     Physical Layer                  │
│  (S-Band, Ka-Band, Laser Comm)     │
└─────────────────────────────────────┘
```

### 2.2 Communication Modes

#### 2.2.1 Real-Time Mode (Low Latency)
- **Use Case**: Critical alerts, crew commands
- **Protocol**: MQTT over TLS
- **Latency**: <5 seconds (LEO), <3 seconds (ISS)
- **Reliability**: QoS 2 (exactly once delivery)
- **Priority**: HIGHEST

#### 2.2.2 Near-Real-Time Mode (Standard Telemetry)
- **Use Case**: Environmental monitoring, sensor data
- **Protocol**: MQTT over TLS
- **Latency**: <60 seconds
- **Reliability**: QoS 1 (at least once delivery)
- **Priority**: HIGH

#### 2.2.3 Batch Mode (Historical Data)
- **Use Case**: Daily summaries, research data
- **Protocol**: HTTP/2 REST API
- **Latency**: <24 hours
- **Reliability**: Retry with exponential backoff
- **Priority**: MEDIUM

#### 2.2.4 Store-and-Forward Mode (Communication Blackout)
- **Use Case**: Communication gaps, deep space
- **Protocol**: DTN (Delay/Disruption Tolerant Networking)
- **Latency**: Minutes to hours
- **Reliability**: Bundle protocol with custody transfer
- **Priority**: VARIABLE

---

## 3. MQTT Protocol Specification

### 3.1 Broker Configuration

```yaml
Broker: mqtt.wia.space
Port: 8883 (TLS), 443 (WebSocket over TLS)
Protocol: MQTT 5.0
Max Packet Size: 10 MB
Session Expiry: 24 hours
Keep-Alive: 60 seconds
Clean Start: false (persistent sessions)
```

### 3.2 Topic Hierarchy

```
wia/space-agri/
├── {moduleId}/
│   ├── environment          # Environmental sensors
│   ├── sensors              # All sensor readings
│   ├── crop                 # Crop status and growth
│   ├── alerts               # Alerts and warnings
│   ├── eclss                # Life support integration
│   ├── harvest              # Harvest events
│   ├── command              # Ground control commands
│   ├── status               # Module health status
│   └── telemetry            # General telemetry
├── mission/
│   ├── {missionId}/summary  # Mission-wide summaries
│   └── {missionId}/crew     # Crew interactions
└── system/
    ├── heartbeat            # System health checks
    └── diagnostics          # System diagnostics
```

### 3.3 Message Format

```json
{
  "header": {
    "version": "WIA-AGRI-035-v1.0",
    "messageId": "MSG-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "moduleId": "ISS-VEGGIE-001",
    "priority": "HIGH"
  },
  "payload": {
    "dataType": "environment",
    "data": {
      "temperature": 22.5,
      "humidity": 65,
      "co2": 800
    }
  },
  "metadata": {
    "source": "SENSOR-ARRAY-01",
    "checksum": "CRC32:A3B5C7D9",
    "compression": "gzip"
  }
}
```

### 3.4 QoS Levels

| Data Type | QoS | Retain | Reason |
|-----------|-----|--------|--------|
| Critical Alerts | 2 | false | Guaranteed delivery, no duplicates |
| Environmental Data | 1 | true | At least once, latest value stored |
| Harvest Events | 2 | false | Must not lose harvest data |
| Status Updates | 1 | true | Latest status always available |
| Diagnostics | 0 | false | Best effort, high frequency |

### 3.5 Last Will and Testament

```json
{
  "topic": "wia/space-agri/ISS-VEGGIE-001/status",
  "qos": 2,
  "retain": true,
  "payload": {
    "status": "OFFLINE",
    "lastSeen": "2025-12-26T10:30:00Z",
    "reason": "CONNECTION_LOST",
    "action": "INITIATE_BACKUP_SYSTEMS"
  }
}
```

---

## 4. HTTP/REST Protocol Specification

### 4.1 HTTP/2 Configuration

```
Protocol: HTTP/2 over TLS 1.3
Compression: Brotli, GZIP
Multiplexing: Enabled
Server Push: Disabled (space-to-ground bandwidth constraints)
```

### 4.2 Request/Response Cycle

```
Client (Space Module) → Ground Station → API Server
                      ← Response ←
```

### 4.3 Retry Logic

```python
def submit_with_retry(data, max_retries=5):
    for attempt in range(max_retries):
        try:
            response = http_post(url, data)
            if response.status == 200:
                return response
        except Exception as e:
            wait_time = min(2 ** attempt, 300)  # Exponential backoff, max 5 min
            log_error(f"Attempt {attempt + 1} failed, retrying in {wait_time}s")
            time.sleep(wait_time)

    # Store locally for batch upload
    store_for_later(data)
```

### 4.4 Idempotency

All POST/PUT requests must include `X-Idempotency-Key` header to prevent duplicate processing during retries.

```http
POST /api/v1/space-agri/submit
X-Idempotency-Key: SUBMIT-ISS-VEGGIE-001-20251226-001
Content-Type: application/json
```

---

## 5. Data Integrity and Security

### 5.1 Encryption

#### 5.1.1 In-Transit Encryption
- **Protocol**: TLS 1.3 with AES-256-GCM
- **Cipher Suites**:
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
- **Certificate**: ECDSA P-384
- **Perfect Forward Secrecy**: Required

#### 5.1.2 At-Rest Encryption
- **Algorithm**: AES-256-CBC
- **Key Management**: HSM (Hardware Security Module)
- **Key Rotation**: Every 90 days

### 5.2 Authentication

#### 5.2.1 Module Authentication
```http
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <MODULE_SPECIFIC_KEY>
X-Module-ID: ISS-VEGGIE-001
X-Certificate-Thumbprint: <SHA256_THUMBPRINT>
```

#### 5.2.2 JWT Token Structure
```json
{
  "iss": "wia-space-agri",
  "sub": "ISS-VEGGIE-001",
  "aud": "api.wia.space",
  "exp": 1735228800,
  "iat": 1735142400,
  "jti": "JWT-20251226-001",
  "permissions": [
    "submit_data",
    "receive_commands",
    "publish_alerts"
  ],
  "mission": "ISS-Expedition-72"
}
```

### 5.3 Data Integrity Checks

#### 5.3.1 Checksums
- **Algorithm**: CRC32 for real-time telemetry, SHA-256 for critical data
- **Validation**: Server-side verification on all submissions
- **Corruption Handling**: Automatic re-request on checksum mismatch

#### 5.3.2 Sequence Numbers
```json
{
  "sequenceNumber": 12345,
  "previousChecksum": "SHA256:A1B2C3D4...",
  "blockHeight": 678
}
```

---

## 6. Operational Procedures

### 6.1 Module Initialization

```
1. Power On → Self-Test → Sensor Calibration
2. Establish Network Connection (S-Band/Ka-Band)
3. Authenticate with API Server (JWT + API Key)
4. Subscribe to MQTT Command Topic
5. Publish Initial Status Message
6. Begin Environmental Monitoring Loop
7. Notify Mission Control: "MODULE ONLINE"
```

### 6.2 Normal Operation Cycle

```
Every 5 minutes:
  1. Read all sensors
  2. Validate readings (range checks)
  3. Publish to MQTT (wia/space-agri/{moduleId}/environment)
  4. Store locally for redundancy
  5. Check for alerts/warnings
  6. Update module status

Every 1 hour:
  1. Aggregate sensor data
  2. Submit batch via REST API
  3. Request command queue from ground
  4. Execute scheduled maintenance tasks

Every 24 hours:
  1. Generate daily summary report
  2. Perform sensor calibration checks
  3. Submit ECLSS integration report
  4. Backup local data to redundant storage
```

### 6.3 Communication Blackout Protocol

```
1. Detect Communication Loss (3 consecutive heartbeat failures)
2. Switch to Autonomous Mode
3. Continue environmental monitoring
4. Store all data in local buffer (7-day capacity)
5. Activate fail-safe protocols:
   - Maintain environmental setpoints
   - Continue light/dark cycles
   - Monitor critical thresholds
   - Alert crew via local display
6. Upon Reconnection:
   - Re-authenticate
   - Upload buffered data (oldest first)
   - Synchronize time
   - Resume normal operations
```

### 6.4 Emergency Shutdown Procedure

```
Trigger Conditions:
  - Critical system failure (pump, fan, LED array)
  - Life support conflict (O2/CO2 imbalance)
  - Crew safety alert
  - Ground control command

Actions:
  1. Publish CRITICAL alert (QoS 2)
  2. Save all data to non-volatile storage
  3. Execute safe shutdown sequence:
     - Turn off LED array
     - Stop water pumps
     - Close nutrient injection
     - Maintain air circulation (if safe)
  4. Preserve crop status for recovery
  5. Notify crew with audible/visual alarm
  6. Await ground control guidance
```

---

## 7. Time Synchronization

### 7.1 Time Protocol
- **Primary**: NTP (Network Time Protocol) via ground stations
- **Backup**: GPS time (for LEO), onboard atomic clock (deep space)
- **Accuracy**: ±100 milliseconds (LEO), ±1 second (deep space)

### 7.2 Timestamp Format
- **Standard**: ISO 8601 UTC
- **Example**: `2025-12-26T10:30:00.000Z`
- **Leap Seconds**: Handled automatically

### 7.3 Orbit Considerations
```json
{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "orbitContext": {
    "orbitNumber": 12345,
    "phase": "DAYLIGHT",
    "nextDarkness": "2025-12-26T11:00:00Z",
    "nextSunrise": "2025-12-26T11:45:00Z"
  }
}
```

---

## 8. Alert and Notification Protocol

### 8.1 Alert Severity Levels

| Level | Description | Response Time | Notification |
|-------|-------------|---------------|--------------|
| CRITICAL | Life support failure, crew safety risk | Immediate | Ground Control + Crew Audio Alarm |
| HIGH | System malfunction, crop health risk | <5 minutes | Ground Control + Crew Visual Alert |
| MEDIUM | Performance degradation | <1 hour | Ground Control Email |
| LOW | Maintenance reminder | <24 hours | Daily Report |
| INFO | Status update | None | Logged only |

### 8.2 Alert Message Format

```json
{
  "alert": {
    "alertId": "ALERT-20251226-001",
    "severity": "CRITICAL",
    "type": "LED_ARRAY_FAILURE",
    "timestamp": "2025-12-26T10:35:00.000Z",
    "moduleId": "ISS-VEGGIE-001",
    "message": "LED array power failure detected. Backup lights activated.",
    "affectedSystems": ["lighting", "photosynthesis"],
    "currentStatus": {
      "ledArray": "FAILED",
      "backupLights": "ACTIVE",
      "cropHealth": "STABLE"
    },
    "recommendedActions": [
      "Inspect LED power supply",
      "Check circuit breakers",
      "Contact ground control for guidance"
    ],
    "estimatedImpact": {
      "cropGrowth": "10% reduction if not resolved in 24h",
      "harvestDelay": "2-3 days"
    },
    "acknowledgment": {
      "required": true,
      "acknowledgedBy": null,
      "acknowledgedAt": null
    }
  }
}
```

### 8.3 Alert Escalation

```
1. Alert Triggered → Publish to MQTT (QoS 2)
2. If not acknowledged in 5 minutes:
   - Send to backup communication channel
   - Activate crew alarm (CRITICAL only)
3. If not acknowledged in 15 minutes:
   - Escalate to mission director
   - Initiate contingency protocol
4. If not acknowledged in 60 minutes:
   - Autonomous safe mode activation
```

---

## 9. Command and Control Protocol

### 9.1 Command Types

| Command | Description | Authorization | Execution |
|---------|-------------|---------------|-----------|
| SET_LIGHT_CYCLE | Change photoperiod | Mission Control | Immediate |
| ADJUST_TEMPERATURE | Modify temperature setpoint | Mission Control | Gradual (1°C/hour) |
| TRIGGER_HARVEST | Initiate harvest mode | Crew + Ground | Manual |
| CALIBRATE_SENSORS | Run sensor calibration | Mission Control | Scheduled |
| EMERGENCY_SHUTDOWN | Shut down module | Crew OR Ground | Immediate |
| UPDATE_FIRMWARE | Software update | Ground Control | Scheduled downtime |

### 9.2 Command Message Format

```json
{
  "command": {
    "commandId": "CMD-20251226-001",
    "timestamp": "2025-12-26T10:40:00.000Z",
    "issuedBy": "MissionControl-Houston",
    "authorization": "FLIGHT-DIRECTOR-SMITH",
    "moduleId": "ISS-VEGGIE-001",
    "commandType": "SET_LIGHT_CYCLE",
    "parameters": {
      "photoperiod": "18h-on-6h-off",
      "startTime": "2025-12-27T00:00:00Z",
      "transition": "gradual"
    },
    "validation": {
      "crewApproval": false,
      "safetyCheck": true,
      "conflictCheck": true
    },
    "execution": {
      "priority": "NORMAL",
      "retryOnFailure": true,
      "maxRetries": 3
    }
  }
}
```

### 9.3 Command Acknowledgment

```json
{
  "acknowledgment": {
    "commandId": "CMD-20251226-001",
    "status": "ACCEPTED",
    "receivedAt": "2025-12-26T10:40:05.000Z",
    "executedAt": "2025-12-27T00:00:00.000Z",
    "result": "SUCCESS",
    "message": "Light cycle changed to 18h-on-6h-off",
    "newState": {
      "lightCycle": "18h-on-6h-off",
      "nextLightChange": "2025-12-27T18:00:00Z"
    }
  }
}
```

---

## 10. Bandwidth Management

### 10.1 Data Prioritization

```
Priority Queue:
1. CRITICAL ALERTS (immediate transmission)
2. CREW COMMANDS (immediate transmission)
3. REAL-TIME TELEMETRY (every 5 minutes)
4. STATUS UPDATES (every 1 hour)
5. HISTORICAL DATA (batch, once per day)
6. RESEARCH DATA (batch, once per week)
```

### 10.2 Compression Strategies

| Data Type | Compression | Ratio | Use Case |
|-----------|-------------|-------|----------|
| JSON Telemetry | GZIP | 60-70% | Default |
| Time-Series Data | Delta Encoding | 80-90% | Historical |
| Images/Video | JPEG, H.265 | 90-95% | Crop monitoring |
| Logs | LZMA | 70-80% | Archive |

### 10.3 Bandwidth Allocation

```
Total Available: 10 Mbps (Ka-Band downlink)

Allocation:
- Critical Alerts: 1 Mbps (10%)
- Real-Time Telemetry: 3 Mbps (30%)
- Batch Data: 4 Mbps (40%)
- Video/Images: 1 Mbps (10%)
- Reserve: 1 Mbps (10%)
```

---

## 11. Compliance and Certification

### 11.1 Space Agency Standards
- **NASA**: NPR 7150.2D (Software Engineering Requirements)
- **ESA**: ECSS-E-ST-70-11C (Space Segment Operability)
- **JAXA**: JERG-2-320A (Software Quality Assurance)

### 11.2 WIA Certification Requirements
1. **Protocol Compliance**: 100% adherence to WIA-AGRI-035
2. **Security Audit**: Annual penetration testing
3. **Reliability Testing**: 99.9% uptime over 90 days
4. **Interoperability**: Tested with ISS, Gateway, and commercial platforms
5. **Documentation**: Complete API documentation and operational manuals

---

**Document Version:** 1.0.0
**Effective Date:** 2025-12-26
**Next Review:** 2026-06-26

---

© 2025 WIA - World Certification Industry Association
弘益人間 (Benefit All Humanity)
