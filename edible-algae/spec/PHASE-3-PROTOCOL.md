# WIA Edible Algae Communication Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Protocol Architecture](#protocol-architecture)
3. [Message Format](#message-format)
4. [Photobioreactor Communication](#photobioreactor-communication)
5. [Sensor Network Protocol](#sensor-network-protocol)
6. [Control Loop Protocol](#control-loop-protocol)
7. [Data Logging Protocol](#data-logging-protocol)
8. [Alert & Notification Protocol](#alert--notification-protocol)
9. [Security](#security)
10. [Reliability](#reliability)

---

## Overview

### 1.1 Purpose

The WIA Edible Algae Communication Protocol defines standardized data exchange between photobioreactors, sensors, controllers, SCADA systems, and cloud platforms for real-time algae cultivation monitoring and automation.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (JSON/CBOR)    │
├─────────────────────────────────────┤
│   Transport (MQTT/WebSocket/HTTP)  │
├─────────────────────────────────────┤
│   Network (TCP/IP, 6LoWPAN)        │
├─────────────────────────────────────┤
│   Physical (Ethernet, WiFi, LoRa)  │
└─────────────────────────────────────┘
```

### 1.3 Supported Transports

| Protocol | Use Case | Latency | Bandwidth |
|----------|----------|---------|-----------|
| MQTT | Real-time sensor data | <100ms | Low |
| WebSocket | Live dashboard updates | <50ms | Medium |
| HTTP/REST | API requests | <200ms | High |
| Modbus TCP | Industrial PLC | <10ms | Low |
| OPC UA | SCADA integration | <50ms | Medium |

---

## Protocol Architecture

### 2.1 Topic Hierarchy (MQTT)

```
wia/algae/{facility_id}/{reactor_id}/{sensor_type}
```

**Examples**:
```
wia/algae/FAC-OCEAN-01/PBR-001/temperature
wia/algae/FAC-OCEAN-01/PBR-001/biomass_density
wia/algae/FAC-OCEAN-01/PBR-002/ph
wia/algae/FAC-OCEAN-01/alerts
wia/algae/FAC-OCEAN-01/commands
```

### 2.2 Quality of Service

| QoS Level | Guarantee | Use Case |
|-----------|-----------|----------|
| QoS 0 | At most once | Non-critical logs |
| QoS 1 | At least once | Sensor readings |
| QoS 2 | Exactly once | Control commands, alerts |

---

## Message Format

### 3.1 Standard Message Structure

```json
{
  "header": {
    "version": "1.0",
    "message_id": "msg-123e4567-e89b-12d3",
    "timestamp": "2025-01-15T10:30:45.123Z",
    "sender_id": "PBR-001",
    "message_type": "SENSOR_DATA"
  },
  "payload": {
    "sensor_type": "OPTICAL_DENSITY",
    "value": 1.25,
    "unit": "OD680",
    "quality": "GOOD"
  },
  "metadata": {
    "calibration_date": "2025-01-10",
    "sensor_serial": "OD-SENS-2024-042"
  }
}
```

### 3.2 Binary Format (CBOR)

For low-bandwidth sensors (LoRa, Zigbee):

```cbor
{
  1: "1.0",           // version
  2: 1642249845,      // timestamp (Unix)
  3: "PBR-001",       // sender_id
  4: 0x02,            // message_type (SENSOR_DATA)
  5: {
    "type": 0x10,     // OPTICAL_DENSITY
    "val": 1.25,
    "unit": "OD680"
  }
}
```

---

## Photobioreactor Communication

### 4.1 Reactor Status Update

**Topic**: `wia/algae/{facility_id}/{reactor_id}/status`

**Payload**:
```json
{
  "header": {
    "message_type": "REACTOR_STATUS",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "payload": {
    "reactor_id": "PBR-001",
    "state": "ACTIVE_GROWTH",
    "cultivation_id": "CULT-2025-ALG-001",
    "uptime_hours": 432,
    "operational_parameters": {
      "temperature_celsius": 25.2,
      "ph": 7.82,
      "light_intensity_umol": 405,
      "dissolved_oxygen_mg_per_L": 7.6,
      "biomass_density_g_per_L": 3.85
    },
    "system_health": {
      "pumps": "OPERATIONAL",
      "lights": "OPERATIONAL",
      "temperature_control": "OPERATIONAL",
      "co2_injection": "OPERATIONAL",
      "agitation": "OPERATIONAL"
    }
  }
}
```

### 4.2 Control Command

**Topic**: `wia/algae/{facility_id}/{reactor_id}/commands`

**Payload**:
```json
{
  "header": {
    "message_type": "CONTROL_COMMAND",
    "timestamp": "2025-01-15T10:31:00Z",
    "command_id": "cmd-abc123"
  },
  "payload": {
    "action": "SET_PARAMETER",
    "parameters": {
      "temperature_setpoint": 26.0,
      "light_intensity_setpoint": 450,
      "co2_flow_rate": 2.8
    },
    "priority": "NORMAL",
    "execute_at": "IMMEDIATE"
  }
}
```

**Response**:
```json
{
  "header": {
    "message_type": "COMMAND_ACK",
    "timestamp": "2025-01-15T10:31:01Z",
    "command_id": "cmd-abc123"
  },
  "payload": {
    "status": "EXECUTED",
    "execution_time_ms": 250,
    "new_state": {
      "temperature_setpoint": 26.0,
      "light_intensity_setpoint": 450
    }
  }
}
```

---

## Sensor Network Protocol

### 5.1 Sensor Registration

```json
{
  "header": {
    "message_type": "SENSOR_REGISTRATION",
    "timestamp": "2025-01-15T08:00:00Z"
  },
  "payload": {
    "sensor_id": "TEMP-SENS-042",
    "sensor_type": "TEMPERATURE",
    "manufacturer": "Atlas Scientific",
    "model": "PT-1000",
    "calibration_date": "2025-01-10",
    "measurement_range": {
      "min": -10,
      "max": 50,
      "unit": "celsius"
    },
    "accuracy": 0.1,
    "sampling_rate_hz": 0.1
  }
}
```

### 5.2 Sensor Data Stream

**Topic**: `wia/algae/{facility_id}/{reactor_id}/{sensor_type}/data`

**Payload (Time-Series)**:
```json
{
  "header": {
    "message_type": "SENSOR_DATA_BATCH",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "payload": {
    "sensor_id": "TEMP-SENS-042",
    "readings": [
      {"t": "2025-01-15T10:30:00Z", "v": 25.2, "q": "GOOD"},
      {"t": "2025-01-15T10:30:10Z", "v": 25.3, "q": "GOOD"},
      {"t": "2025-01-15T10:30:20Z", "v": 25.2, "q": "GOOD"}
    ],
    "statistics": {
      "avg": 25.23,
      "min": 25.2,
      "max": 25.3,
      "stddev": 0.05
    }
  }
}
```

### 5.3 Sensor Calibration

```json
{
  "header": {
    "message_type": "SENSOR_CALIBRATION",
    "timestamp": "2025-01-15T09:00:00Z"
  },
  "payload": {
    "sensor_id": "PH-SENS-015",
    "calibration_type": "TWO_POINT",
    "calibration_points": [
      {"reference": 7.0, "measured": 7.02},
      {"reference": 10.0, "measured": 9.98}
    ],
    "calibration_curve": {
      "slope": 1.001,
      "offset": -0.01
    },
    "next_calibration_due": "2025-02-15"
  }
}
```

---

## Control Loop Protocol

### 6.1 PID Controller State

```json
{
  "header": {
    "message_type": "CONTROLLER_STATE",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "payload": {
    "controller_id": "PID-TEMP-001",
    "controlled_variable": "TEMPERATURE",
    "setpoint": 25.0,
    "process_variable": 25.2,
    "output": 45.0,
    "pid_parameters": {
      "Kp": 2.0,
      "Ki": 0.5,
      "Kd": 0.1
    },
    "error": 0.2,
    "integral": 1.5,
    "derivative": 0.05,
    "mode": "AUTOMATIC",
    "limits": {
      "output_min": 0,
      "output_max": 100
    }
  }
}
```

### 6.2 Feedforward Control

```json
{
  "header": {
    "message_type": "FEEDFORWARD_ADJUSTMENT",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "payload": {
    "disturbance_type": "AMBIENT_TEMPERATURE_CHANGE",
    "measured_disturbance": 5.0,
    "feedforward_output": 12.0,
    "reasoning": "Ambient temp increased 5°C, pre-emptively increase cooling"
  }
}
```

---

## Data Logging Protocol

### 7.1 High-Frequency Data Log

**Topic**: `wia/algae/{facility_id}/logs/high_freq`

**Payload**:
```json
{
  "header": {
    "message_type": "DATA_LOG",
    "timestamp": "2025-01-15T10:30:00Z",
    "log_level": "INFO"
  },
  "payload": {
    "reactor_id": "PBR-001",
    "log_interval_seconds": 10,
    "data_points": [
      {
        "timestamp": "2025-01-15T10:30:00Z",
        "temperature": 25.2,
        "ph": 7.82,
        "od680": 1.25,
        "do": 7.6
      },
      {
        "timestamp": "2025-01-15T10:30:10Z",
        "temperature": 25.3,
        "ph": 7.81,
        "od680": 1.26,
        "do": 7.5
      }
    ]
  }
}
```

### 7.2 Event Log

```json
{
  "header": {
    "message_type": "EVENT_LOG",
    "timestamp": "2025-01-15T10:35:00Z",
    "log_level": "WARNING"
  },
  "payload": {
    "event_type": "THRESHOLD_EXCEEDED",
    "reactor_id": "PBR-001",
    "parameter": "TEMPERATURE",
    "threshold": 30.0,
    "actual_value": 30.5,
    "duration_seconds": 120,
    "action_taken": "Activated cooling system"
  }
}
```

---

## Alert & Notification Protocol

### 8.1 Alert Message

**Topic**: `wia/algae/{facility_id}/alerts`

**Payload**:
```json
{
  "header": {
    "message_type": "ALERT",
    "timestamp": "2025-01-15T10:40:00Z",
    "alert_id": "ALT-2025-001"
  },
  "payload": {
    "severity": "WARNING",
    "alert_type": "CONTAMINATION_SUSPECTED",
    "reactor_id": "PBR-001",
    "cultivation_id": "CULT-2025-ALG-001",
    "description": "Bacterial count elevated: 950 CFU/g (threshold: 500)",
    "recommended_actions": [
      "Perform microscopic examination",
      "Increase UV sterilization",
      "Consider batch termination if confirmed"
    ],
    "auto_actions_taken": [
      "Increased UV lamp intensity to 100%",
      "Isolated reactor circulation"
    ],
    "escalation": {
      "notify_roles": ["FACILITY_MANAGER", "QC_SUPERVISOR"],
      "escalate_if_not_acked_minutes": 15
    }
  }
}
```

### 8.2 Alert Acknowledgment

```json
{
  "header": {
    "message_type": "ALERT_ACK",
    "timestamp": "2025-01-15T10:42:00Z",
    "alert_id": "ALT-2025-001"
  },
  "payload": {
    "acknowledged_by": "user@oceanfarms.com",
    "ack_timestamp": "2025-01-15T10:42:00Z",
    "notes": "Confirmed contamination. Terminating batch.",
    "resolution_action": "BATCH_TERMINATION"
  }
}
```

---

## Security

### 9.1 TLS/SSL

All MQTT, WebSocket, and HTTP communication MUST use TLS 1.3 or higher.

```
MQTT Broker: mqtts://mqtt.wia-edible-algae.org:8883
WebSocket: wss://api.wia-edible-algae.org/ws
```

### 9.2 Authentication

**MQTT Client Authentication**:
```json
{
  "client_id": "PBR-001",
  "username": "reactor_pbr001",
  "password": "hashed_password_or_token"
}
```

**Certificate-Based Authentication** (for production):
```
Client Certificate: reactor-pbr001.crt
Client Key: reactor-pbr001.key
CA Certificate: wia-ca.crt
```

### 9.3 Message Signing

```json
{
  "header": { ... },
  "payload": { ... },
  "signature": {
    "algorithm": "Ed25519",
    "public_key_id": "key-2025-001",
    "signature_value": "base64_encoded_signature"
  }
}
```

---

## Reliability

### 10.1 Message Acknowledgment

```json
{
  "header": {
    "message_type": "ACK",
    "ack_for_message_id": "msg-123e4567",
    "timestamp": "2025-01-15T10:30:01Z"
  },
  "payload": {
    "status": "RECEIVED_AND_PROCESSED",
    "processing_time_ms": 45
  }
}
```

### 10.2 Heartbeat

**Topic**: `wia/algae/{facility_id}/{reactor_id}/heartbeat`

**Interval**: Every 30 seconds

**Payload**:
```json
{
  "header": {
    "message_type": "HEARTBEAT",
    "timestamp": "2025-01-15T10:30:00Z"
  },
  "payload": {
    "reactor_id": "PBR-001",
    "status": "OPERATIONAL",
    "uptime_seconds": 1555200,
    "last_measurement_timestamp": "2025-01-15T10:29:50Z"
  }
}
```

### 10.3 Offline Buffering

Reactors MUST buffer up to 1 hour of data when network is unavailable, then transmit on reconnection.

---

**弘益人間 (Hongik Ingan)** · Benefit All Humanity

© 2025 WIA Standards - MIT License
