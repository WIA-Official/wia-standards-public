# WIA-SEMI-001: Phase 3 - Protocol Implementation

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 3 establishes communication protocols enabling chips to interact with each other and coordinate system-wide behavior. This includes inter-chip communication, power coordination, thermal management, and security protocols.

## WIA-SemiLink Protocol

### Overview

WIA-SemiLink is a lightweight protocol for chip-to-chip communication over standard interfaces (I2C, SPI, UART, or custom serial).

### Packet Structure

```c
struct wia_semilink_packet {
    // Header (8 bytes)
    uint16_t magic;           // 0x5749 ('WI' in ASCII)
    uint16_t source_addr;     // Source chip address
    uint16_t dest_addr;       // Destination chip address
    uint8_t  protocol_ver;    // Protocol version (0x10 for v1.0)
    uint8_t  flags;           // Control flags

    // Payload metadata (4 bytes)
    uint16_t payload_len;     // Payload length (0-1024 bytes)
    uint8_t  message_type;    // Message type code
    uint8_t  sequence_num;    // Sequence number

    // Payload (variable, max 1024 bytes)
    uint8_t  payload[payload_len];

    // Footer (2 bytes)
    uint16_t crc16;           // CRC-16/CCITT checksum
};
```

### Message Types

| Code | Type | Description |
|------|------|-------------|
| 0x01 | COMMAND | Command to execute action |
| 0x02 | RESPONSE | Response to command |
| 0x03 | EVENT | Asynchronous event notification |
| 0x04 | TELEMETRY | Periodic telemetry data |
| 0x05 | HEARTBEAT | Keep-alive message |

### Standard Commands

#### IDENTIFY (0x10)

Request chip identification and capabilities.

**Request:**
```json
{
  "command": "IDENTIFY"
}
```

**Response:**
```json
{
  "chipId": "WIA-SOC-2025-001",
  "manufacturer": "Example Corp",
  "capabilities": ["power-mgmt", "thermal-mgmt", "telemetry"]
}
```

#### GET_POWER_STATE (0x20)

Query current power state.

**Response:**
```json
{
  "powerConsumption": {"value": 8.5, "unit": "W"},
  "powerMode": "balanced",
  "powerBudget": {"value": 10, "unit": "W"}
}
```

#### SET_POWER_LIMIT (0x21)

Set power consumption limit.

**Request:**
```json
{
  "command": "SET_POWER_LIMIT",
  "limit": {"value": 10, "unit": "W"},
  "enforcement": "hard"
}
```

#### GET_TEMPERATURE (0x30)

Query temperature sensors.

**Response:**
```json
{
  "sensors": [
    {"location": "CPU", "temp": 72.5, "unit": "°C"},
    {"location": "GPU", "temp": 68.2, "unit": "°C"}
  ]
}
```

## Power Management Coordination

### System-Wide Power Budgeting

Chips coordinate to stay within system power budget.

#### Power Budget Broadcast

Power manager broadcasts available budget:

```json
{
  "type": "POWER_BUDGET_UPDATE",
  "totalBudget": {"value": 45, "unit": "W"},
  "allocated": {"value": 32, "unit": "W"},
  "available": {"value": 13, "unit": "W"},
  "priority": "normal"
}
```

#### Power Request Protocol

1. Chip requests additional power:
```json
{
  "type": "POWER_REQUEST",
  "chipId": "WIA-SOC-2025-001",
  "currentPower": {"value": 8, "unit": "W"},
  "requestedPower": {"value": 12, "unit": "W"},
  "duration": 5000,
  "reason": "performance-boost"
}
```

2. Power manager responds:
```json
{
  "type": "POWER_GRANT",
  "requestId": "req_123",
  "grantedPower": {"value": 10, "unit": "W"},
  "duration": 5000,
  "mustRelease": "on-thermal-event"
}
```

### Dynamic Power Reallocation

Triggers for power reallocation:
- Workload shift between components
- Thermal emergency
- Battery critical
- AC power connect/disconnect

## Thermal Management Protocols

### Thermal Event Propagation

```json
{
  "type": "THERMAL_EVENT",
  "severity": "warning",
  "source": {
    "chipId": "WIA-SOC-2025-001",
    "location": "CPU-Core0"
  },
  "temperature": {"value": 92, "unit": "°C"},
  "threshold": {"value": 95, "unit": "°C"},
  "trend": "rising",
  "estimatedTimeToThreshold": 2.5,
  "recommendedActions": [
    "reduce-frequency",
    "migrate-workload"
  ]
}
```

### Severity Levels

| Level | Temp Range | Response | Response Time |
|-------|-----------|----------|---------------|
| Info | <80°C | Normal monitoring | N/A |
| Warning | 80-90°C | Prepare to reduce power | 5 sec |
| Critical | 90-100°C | Reduce frequency 10-25% | 1 sec |
| Emergency | >100°C | Emergency power reduction | 100 ms |

### Coordinated Thermal Response

All chips in system respond to thermal events:
1. Receive thermal event notification
2. Assess own contribution to thermal load
3. Reduce power proportionally
4. Report new state to thermal manager
5. Monitor temperature trend
6. Gradually restore performance when safe

## Security Protocols

### Secure Boot Protocol

#### Challenge-Response Handshake

1. Boot initiator challenges peripheral:
```json
{
  "type": "SECURE_BOOT_CHALLENGE",
  "nonce": "a1b2c3d4e5f6g7h8",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

2. Peripheral responds with signed attestation:
```json
{
  "type": "SECURE_BOOT_RESPONSE",
  "chipId": "WIA-GPU-2025-001",
  "firmwareVersion": "2.1.5",
  "firmwareHash": "sha256:8f43e8...",
  "signature": "RSA-2048:a5b2c1...",
  "certificate": "X.509 cert...",
  "nonce": "a1b2c3d4e5f6g7h8"
}
```

3. Initiator verifies and allows boot:
```json
{
  "type": "SECURE_BOOT_DECISION",
  "allowed": true,
  "trustLevel": "full",
  "restrictions": []
}
```

### Runtime Security

- **Encryption**: AES-256-GCM for all inter-chip messages
- **Authentication**: HMAC-SHA256 message authentication
- **Key Rotation**: Every 24 hours
- **Replay Protection**: Sequence numbers
- **Anomaly Detection**: Monitor communication patterns

### Security Event Protocol

```json
{
  "type": "SECURITY_EVENT",
  "severity": "critical",
  "category": "authentication-failure",
  "details": {
    "sourceChip": "UNKNOWN",
    "failureReason": "invalid-signature",
    "attemptCount": 3,
    "blocked": true
  },
  "recommendedAction": "isolate-device"
}
```

## Performance Coordination

### Workload Distribution

#### Workload Submission

```json
{
  "type": "WORKLOAD_SUBMIT",
  "workloadId": "wl_001",
  "characteristics": {
    "type": "ai-inference",
    "model": "ResNet-50",
    "batchSize": 32,
    "precision": "INT8"
  },
  "constraints": {
    "maxLatency": 50,
    "powerBudget": {"value": 5, "unit": "W"}
  }
}
```

#### Capability Analysis

```json
{
  "type": "WORKLOAD_ANALYSIS",
  "candidates": [
    {
      "chipId": "WIA-NPU-2025-001",
      "score": 95,
      "estimatedLatency": 25,
      "estimatedPower": {"value": 4.2, "unit": "W"}
    }
  ],
  "recommendation": "WIA-NPU-2025-001"
}
```

### Cache Coherency Protocols

For shared-memory systems:

#### WIA-MESI Protocol

- **Modified**: Cache line modified, exclusive to this cache
- **Exclusive**: Clean line, exclusive to this cache
- **Shared**: Clean line, may be in other caches
- **Invalid**: Line not valid

State transitions follow standard MESI protocol with optimizations for system-on-chip architectures.

## Real-Time Monitoring

### Telemetry Streaming

```json
{
  "type": "TELEMETRY_BATCH",
  "chipId": "WIA-SOC-2025-001",
  "startTime": "2025-01-15T10:30:00.000Z",
  "sampleInterval": 10,
  "samples": [
    {
      "offset": 0,
      "power": {"value": 8.2, "unit": "W"},
      "temperature": {"value": 72, "unit": "°C"},
      "frequency": {"cpu": 3200, "gpu": 950, "unit": "MHz"}
    }
  ]
}
```

### Event Categories

- **Power Events**: Mode changes, throttling
- **Thermal Events**: Temperature thresholds
- **Performance Events**: Frequency changes
- **Error Events**: Hardware errors
- **Security Events**: Authentication failures

## Transport Layer

### Supported Physical Interfaces

| Interface | Speed | Use Case |
|-----------|-------|----------|
| I2C | Up to 3.4 Mbps | Low-bandwidth control |
| SPI | Up to 50 Mbps | Medium-bandwidth data |
| UART | Up to 12 Mbps | Debug and diagnostics |
| Custom Serial | Varies | Specialized applications |

### Flow Control

- Automatic flow control using RTS/CTS
- Software flow control using XON/XOFF
- Packet-level acknowledgments

### Error Detection and Recovery

- CRC-16 checksum on all packets
- Automatic retransmission on checksum failure
- Maximum 3 retries before reporting error
- Exponential backoff on retry

## Compliance Requirements

To be Phase 3 compliant:

1. ✓ Implement WIA-SemiLink protocol
2. ✓ Support power coordination protocol
3. ✓ Generate thermal events
4. ✓ Implement telemetry streaming
5. ✓ (Optional) Secure boot support
6. ✓ (Optional) Workload coordination

## Testing and Validation

```bash
# Protocol conformance test
wia-semi-test protocol --target soc.local

# Multi-chip coordination test
wia-semi-test coordination --chips chip1.local,chip2.local

# Thermal management test
wia-semi-test thermal --stress high
```

---

**Previous**: [Phase 2 - API Interface](PHASE-2-API-INTERFACE.md)
**Next**: [Phase 4 - System Integration](PHASE-4-INTEGRATION.md)

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity
