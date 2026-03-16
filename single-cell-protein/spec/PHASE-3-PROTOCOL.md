# WIA-AGRI-034: Phase 3 - Communication Protocol Specification

**Standard ID:** WIA-AGRI-034
**Title:** Single Cell Protein Production Communication Protocol
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the real-time communication protocols for WIA-AGRI-034, enabling bidirectional data exchange between SCP production systems, IoT sensors, control systems, and cloud platforms.

### 1.1 Supported Transport Protocols

- **WebSocket**: Real-time bidirectional communication
- **MQTT**: Lightweight IoT messaging
- **HTTP/2 Server-Sent Events (SSE)**: Server-to-client streaming
- **gRPC**: High-performance RPC for system integration
- **CoAP**: Constrained application protocol for edge devices

### 1.2 Protocol Selection Guide

| Use Case | Recommended Protocol | Rationale |
|----------|---------------------|-----------|
| Real-time sensor streaming | MQTT | Low bandwidth, reliable delivery |
| Dashboard updates | WebSocket | Bidirectional, low latency |
| Control system integration | gRPC | Type-safe, high performance |
| IoT edge devices | CoAP | Resource-constrained environments |
| Web client updates | SSE | Simple, unidirectional streaming |

---

## 2. WebSocket Protocol

### 2.1 Connection Establishment

**Endpoint:** `wss://api.wia.org/agri-034/v1/ws`

**Connection Request:**
```json
{
  "action": "connect",
  "version": "1.0",
  "protocol": "WIA-AGRI-034-WS",
  "auth": {
    "type": "bearer",
    "token": "YOUR_API_TOKEN"
  },
  "client_info": {
    "name": "SCP Control Dashboard",
    "version": "2.1.0",
    "capabilities": ["batch_monitoring", "control", "alerts"]
  }
}
```

**Connection Response:**
```json
{
  "action": "connected",
  "session_id": "sess_abc123xyz",
  "server_version": "1.0",
  "heartbeat_interval_seconds": 30,
  "max_message_size_bytes": 1048576,
  "supported_features": [
    "real_time_streaming",
    "batch_control",
    "multi_batch_monitoring",
    "historical_replay"
  ]
}
```

### 2.2 Message Format

**Base Message Structure:**
```json
{
  "message_id": "msg_uuid_v4",
  "timestamp": "ISO8601 datetime",
  "version": "1.0",
  "type": "data | command | response | event | error",
  "payload": { /* type-specific payload */ }
}
```

### 2.3 Data Streaming

**Subscribe to Batch Stream:**
```json
{
  "message_id": "msg_001",
  "type": "command",
  "payload": {
    "action": "subscribe",
    "resource": "batch",
    "batch_id": "BATCH-20250115-001",
    "parameters": [
      "temperature",
      "pH",
      "dissolved_oxygen",
      "biomass",
      "protein_yield"
    ],
    "sampling_interval_seconds": 5,
    "aggregation": "none | avg | min_max | full_stats"
  }
}
```

**Streaming Data Message:**
```json
{
  "message_id": "msg_stream_001",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "type": "data",
  "payload": {
    "batch_id": "BATCH-20250115-001",
    "sequence": 12345,
    "data": {
      "temperature": {
        "value": 37.2,
        "unit": "C",
        "setpoint": 37.0,
        "deviation": 0.2,
        "status": "normal"
      },
      "pH": {
        "value": 6.48,
        "unit": "pH",
        "setpoint": 6.50,
        "deviation": -0.02,
        "status": "normal"
      },
      "dissolved_oxygen": {
        "value": 42.5,
        "unit": "percent",
        "setpoint": 40.0,
        "deviation": 2.5,
        "status": "normal"
      },
      "biomass": {
        "optical_density": 4.25,
        "dry_cell_weight_g_L": 38.7,
        "growth_rate_h": 0.42
      },
      "protein_yield": {
        "current_g_L": 20.1,
        "rate_g_L_h": 0.85,
        "projected_final_g_L": 24.5
      }
    },
    "quality": {
      "data_completeness_percent": 100,
      "sensor_health": "good",
      "last_calibration_hours_ago": 72
    }
  }
}
```

### 2.4 Control Commands

**Update Process Parameters:**
```json
{
  "message_id": "msg_cmd_001",
  "type": "command",
  "payload": {
    "action": "update_parameters",
    "batch_id": "BATCH-20250115-001",
    "parameters": {
      "temperature": {
        "setpoint_C": 38.0,
        "ramp_rate_C_per_minute": 0.1
      },
      "dissolved_oxygen": {
        "setpoint_percent": 45.0
      }
    },
    "authorization": {
      "operator_id": "OP-001",
      "approval_code": "AUTH-12345"
    }
  }
}
```

**Command Response:**
```json
{
  "message_id": "msg_resp_001",
  "timestamp": "2025-01-15T14:31:00.000Z",
  "type": "response",
  "in_reply_to": "msg_cmd_001",
  "payload": {
    "status": "success | pending | failed",
    "message": "Parameters updated successfully",
    "updated_parameters": {
      "temperature": {
        "old_setpoint": 37.0,
        "new_setpoint": 38.0,
        "estimated_completion": "2025-01-15T15:01:00Z"
      },
      "dissolved_oxygen": {
        "old_setpoint": 40.0,
        "new_setpoint": 45.0,
        "status": "applied"
      }
    },
    "audit_log_id": "AUDIT-20250115-001"
  }
}
```

### 2.5 Event Notifications

**Alert Event:**
```json
{
  "message_id": "msg_evt_001",
  "timestamp": "2025-01-15T14:32:00.000Z",
  "type": "event",
  "payload": {
    "event_type": "alert",
    "severity": "info | warning | critical",
    "batch_id": "BATCH-20250115-001",
    "alert": {
      "code": "TEMP_DEVIATION",
      "title": "Temperature deviation detected",
      "description": "Temperature has deviated from setpoint by more than 1°C",
      "current_value": 38.5,
      "setpoint": 37.0,
      "threshold": 1.0,
      "recommended_action": "Check heating system and increase cooling capacity"
    },
    "requires_acknowledgment": true,
    "auto_resolve_enabled": true
  }
}
```

**Batch Phase Change:**
```json
{
  "message_id": "msg_evt_002",
  "timestamp": "2025-01-15T14:00:00.000Z",
  "type": "event",
  "payload": {
    "event_type": "phase_change",
    "batch_id": "BATCH-20250115-001",
    "previous_phase": "lag",
    "current_phase": "exponential",
    "characteristics": {
      "specific_growth_rate_h": 0.42,
      "doubling_time_hours": 1.65,
      "estimated_phase_duration_hours": 18
    },
    "recommendations": [
      "Monitor oxygen transfer rate",
      "Prepare for fed-batch substrate addition",
      "Increase sampling frequency"
    ]
  }
}
```

### 2.6 Heartbeat & Keep-Alive

**Ping:**
```json
{
  "message_id": "msg_ping_001",
  "type": "ping",
  "timestamp": "2025-01-15T14:33:00.000Z"
}
```

**Pong:**
```json
{
  "message_id": "msg_pong_001",
  "type": "pong",
  "timestamp": "2025-01-15T14:33:00.100Z",
  "in_reply_to": "msg_ping_001",
  "latency_ms": 100,
  "server_load_percent": 42
}
```

---

## 3. MQTT Protocol

### 3.1 Topic Structure

**Hierarchical Topic Naming:**
```
wia/agri-034/{version}/{facility_id}/{reactor_id}/{data_type}/{parameter}
```

**Examples:**
```
wia/agri-034/v1/FAC-001/REACTOR-A/sensor/temperature
wia/agri-034/v1/FAC-001/REACTOR-A/sensor/pH
wia/agri-034/v1/FAC-001/REACTOR-A/sensor/dissolved_oxygen
wia/agri-034/v1/FAC-001/REACTOR-A/production/biomass
wia/agri-034/v1/FAC-001/REACTOR-A/control/setpoint
wia/agri-034/v1/FAC-001/REACTOR-A/alert/threshold_exceeded
wia/agri-034/v1/FAC-001/REACTOR-A/status/batch_phase
```

### 3.2 QoS Levels

| QoS | Level | Use Case |
|-----|-------|----------|
| 0 | At most once | Non-critical sensor readings |
| 1 | At least once | Important events, alerts |
| 2 | Exactly once | Control commands, critical data |

### 3.3 Message Payloads

**Sensor Data (QoS 0):**
```json
{
  "timestamp": "2025-01-15T14:34:00.000Z",
  "sensor_id": "TEMP-001",
  "value": 37.2,
  "unit": "C",
  "quality": "good"
}
```

**Control Setpoint (QoS 2):**
```json
{
  "timestamp": "2025-01-15T14:35:00.000Z",
  "parameter": "temperature",
  "setpoint": 38.0,
  "ramp_rate": 0.1,
  "operator_id": "OP-001",
  "command_id": "CMD-001"
}
```

**Alert Message (QoS 1):**
```json
{
  "timestamp": "2025-01-15T14:36:00.000Z",
  "alert_code": "DO_LOW",
  "severity": "warning",
  "current_value": 32.5,
  "threshold": 35.0,
  "message": "Dissolved oxygen below critical threshold"
}
```

### 3.4 Retained Messages

**Batch Status (Retained):**
```
Topic: wia/agri-034/v1/FAC-001/REACTOR-A/status/current_batch

Payload:
{
  "batch_id": "BATCH-20250115-001",
  "status": "active",
  "start_time": "2025-01-15T08:00:00Z",
  "elapsed_hours": 6.5,
  "current_phase": "exponential"
}
```

### 3.5 Last Will & Testament

**Configuration:**
```json
{
  "topic": "wia/agri-034/v1/FAC-001/REACTOR-A/status/connection",
  "payload": {
    "status": "offline",
    "timestamp": "ISO8601",
    "reason": "unexpected_disconnect"
  },
  "qos": 1,
  "retain": true
}
```

---

## 4. gRPC Protocol

### 4.1 Service Definition (Proto3)

```protobuf
syntax = "proto3";

package wia.agri034.v1;

service SCPProductionService {
  // Batch management
  rpc CreateBatch(CreateBatchRequest) returns (BatchResponse);
  rpc GetBatch(GetBatchRequest) returns (BatchResponse);
  rpc UpdateBatchParameters(UpdateParametersRequest) returns (UpdateResponse);

  // Real-time streaming
  rpc StreamSensorData(StreamRequest) returns (stream SensorDataMessage);
  rpc StreamProductionMetrics(StreamRequest) returns (stream ProductionMetricsMessage);

  // Bidirectional control
  rpc BatchControl(stream ControlCommand) returns (stream ControlResponse);

  // Quality analysis
  rpc SubmitQualitySample(QualitySampleRequest) returns (QualityResponse);
  rpc GetQualityResults(GetQualityRequest) returns (QualityAnalysis);
}

message BatchResponse {
  string batch_id = 1;
  string status = 2;
  int64 timestamp = 3;
  BatchDetails details = 4;
}

message SensorDataMessage {
  string batch_id = 1;
  int64 timestamp = 2;
  repeated SensorReading readings = 3;
}

message SensorReading {
  string sensor_id = 1;
  string parameter = 2;
  double value = 3;
  string unit = 4;
  SensorQuality quality = 5;
}

enum SensorQuality {
  GOOD = 0;
  FAIR = 1;
  POOR = 2;
  FAILED = 3;
}

message ControlCommand {
  string batch_id = 1;
  string command_type = 2;
  map<string, double> parameters = 3;
  string operator_id = 4;
}

message ControlResponse {
  string command_id = 1;
  CommandStatus status = 2;
  string message = 3;
  int64 timestamp = 4;
}

enum CommandStatus {
  SUCCESS = 0;
  PENDING = 1;
  FAILED = 2;
  REJECTED = 3;
}
```

### 4.2 Example Usage (Go)

```go
package main

import (
    "context"
    "log"
    pb "wia.org/agri-034/v1"
    "google.golang.org/grpc"
)

func main() {
    conn, err := grpc.Dial("api.wia.org:443", grpc.WithTransportCredentials(creds))
    if err != nil {
        log.Fatalf("Failed to connect: %v", err)
    }
    defer conn.Close()

    client := pb.NewSCPProductionServiceClient(conn)

    // Create batch
    batch, err := client.CreateBatch(context.Background(), &pb.CreateBatchRequest{
        FacilityId: "FAC-001",
        StrainId:   "STRAIN-001",
        Volume:     1000,
    })

    // Stream sensor data
    stream, err := client.StreamSensorData(context.Background(), &pb.StreamRequest{
        BatchId: batch.BatchId,
    })

    for {
        data, err := stream.Recv()
        if err != nil {
            break
        }
        log.Printf("Temperature: %.2f°C", data.Readings[0].Value)
    }
}
```

---

## 5. Server-Sent Events (SSE)

### 5.1 Connection Setup

**Endpoint:** `https://api.wia.org/agri-034/v1/batches/{batch_id}/events`

**Request:**
```http
GET /api/v1/batches/BATCH-20250115-001/events HTTP/1.1
Host: api.wia.org
Accept: text/event-stream
Authorization: Bearer YOUR_API_TOKEN
Cache-Control: no-cache
```

### 5.2 Event Stream Format

```
event: sensor_update
id: 1
data: {"timestamp":"2025-01-15T14:40:00Z","temperature":37.2,"pH":6.48}

event: phase_change
id: 2
data: {"phase":"exponential","timestamp":"2025-01-15T14:00:00Z"}

event: alert
id: 3
data: {"severity":"warning","code":"TEMP_HIGH","value":38.5}

: heartbeat comment

event: batch_completed
id: 4
data: {"batch_id":"BATCH-20250115-001","final_yield":23.6}
```

### 5.3 Client Reconnection

```javascript
const eventSource = new EventSource(
  'https://api.wia.org/agri-034/v1/batches/BATCH-20250115-001/events',
  {
    headers: {
      'Authorization': 'Bearer YOUR_API_TOKEN'
    }
  }
);

eventSource.addEventListener('sensor_update', (e) => {
  const data = JSON.parse(e.data);
  console.log('Temperature:', data.temperature);
});

eventSource.addEventListener('error', (e) => {
  if (eventSource.readyState === EventSource.CLOSED) {
    // Automatic reconnection will occur
    console.log('Connection closed, reconnecting...');
  }
});
```

---

## 6. CoAP for Edge Devices

### 6.1 Endpoint Structure

```
coap://edge.wia.org:5683/agri-034/v1/sensor
```

### 6.2 Sensor Data POST

**Request:**
```
POST coap://edge.wia.org:5683/agri-034/v1/sensor
Content-Format: application/json

{
  "sensor_id": "TEMP-EDGE-001",
  "batch_id": "BATCH-20250115-001",
  "value": 37.2,
  "timestamp": 1642253400
}
```

**Response:**
```
2.01 Created
Location-Path: /sensor/reading/12345
```

### 6.3 Observable Resources

```
GET coap://edge.wia.org:5683/agri-034/v1/batch/BATCH-20250115-001/status
Observe: 0

Response:
2.05 Content
Observe: 42
Content-Format: application/json

{
  "status": "active",
  "phase": "exponential",
  "updated": 1642253400
}
```

---

## 7. Security Considerations

### 7.1 Transport Security

- **TLS/SSL**: Required for all WebSocket and HTTPS connections
- **DTLS**: Required for CoAP over UDP
- **MQTT over TLS**: Port 8883 for encrypted MQTT
- **gRPC Security**: mTLS for mutual authentication

### 7.2 Message Authentication

```json
{
  "message": { /* payload */ },
  "signature": {
    "algorithm": "HMAC-SHA256",
    "key_id": "key_001",
    "value": "hex_encoded_signature"
  }
}
```

### 7.3 Authorization Scopes

| Scope | Permissions |
|-------|-------------|
| `batch:read` | Read batch data |
| `batch:write` | Create and modify batches |
| `sensor:read` | Read sensor data streams |
| `control:write` | Send control commands |
| `quality:read` | Access quality analysis results |
| `admin` | Full system access |

---

## 8. Performance Specifications

### 8.1 Latency Requirements

| Protocol | Maximum Latency | Typical Latency |
|----------|----------------|-----------------|
| WebSocket | 200ms | 50ms |
| MQTT | 500ms | 100ms |
| gRPC | 100ms | 30ms |
| SSE | 1000ms | 200ms |
| CoAP | 2000ms | 500ms |

### 8.2 Throughput

- **Sensor data rate**: Up to 1000 messages/second per batch
- **WebSocket concurrent connections**: 10,000+
- **MQTT topics per broker**: 100,000+
- **gRPC streaming**: 1GB/s aggregate throughput

### 8.3 Reliability

- **Message delivery guarantee**: 99.9% for QoS 1 and 2
- **Connection uptime**: 99.95% SLA
- **Data loss**: <0.01% under normal conditions

---

## 9. Implementation Examples

### 9.1 Python WebSocket Client

```python
import asyncio
import websockets
import json

async def monitor_batch(batch_id):
    uri = "wss://api.wia.org/agri-034/v1/ws"

    async with websockets.connect(uri) as ws:
        # Authenticate
        await ws.send(json.dumps({
            "action": "connect",
            "auth": {"type": "bearer", "token": "YOUR_TOKEN"}
        }))

        # Subscribe
        await ws.send(json.dumps({
            "type": "command",
            "payload": {
                "action": "subscribe",
                "batch_id": batch_id,
                "parameters": ["temperature", "pH", "biomass"]
            }
        }))

        # Receive data
        while True:
            message = await ws.recv()
            data = json.loads(message)
            if data["type"] == "data":
                print(f"Temperature: {data['payload']['data']['temperature']['value']}")

asyncio.run(monitor_batch("BATCH-20250115-001"))
```

### 9.2 Node.js MQTT Publisher

```javascript
const mqtt = require('mqtt');

const client = mqtt.connect('mqtts://mqtt.wia.org:8883', {
  clientId: 'sensor-001',
  username: 'your-username',
  password: 'your-password'
});

client.on('connect', () => {
  setInterval(() => {
    const data = {
      timestamp: new Date().toISOString(),
      sensor_id: 'TEMP-001',
      value: 37.0 + Math.random(),
      unit: 'C'
    };

    client.publish(
      'wia/agri-034/v1/FAC-001/REACTOR-A/sensor/temperature',
      JSON.stringify(data),
      { qos: 0 }
    );
  }, 5000);
});
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
