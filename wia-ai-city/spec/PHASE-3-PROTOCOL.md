# WIA AI-City - Phase 3: AIDC-Link Protocol

> **Version:** 1.0.0
> **Status:** Complete
> **Last Updated:** 2025-12-24

---

## 1. Overview

AIDC-Link (AI Data Communication Link) is a 4-layer protocol designed for AI-City communications, enabling:
- Real-time sensor data streaming
- Distributed AI inference coordination
- Secure citizen data transmission
- City-wide service orchestration

---

## 2. Protocol Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Layer 4: Application Layer                              │
│  - City Services API                                     │
│  - Citizen Portal                                        │
│  - Developer SDK                                         │
├─────────────────────────────────────────────────────────┤
│  Layer 3: Intelligence Layer                             │
│  - Edge AI Processing                                    │
│  - Central AI Coordination                               │
│  - Federated Learning                                    │
├─────────────────────────────────────────────────────────┤
│  Layer 2: Data Layer                                     │
│  - JSON-LD Semantic Format                               │
│  - Zstd Compression                                      │
│  - AES-256-GCM Encryption                                │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Physical Layer                                 │
│  - Fiber Backbone (100 Gbps)                             │
│  - 5G/6G Mesh Network                                    │
│  - Quantum Links (Future)                                │
└─────────────────────────────────────────────────────────┘
```

---

## 3. Layer 1: Physical Layer

### 3.1 Fiber Backbone

| Specification | Value |
|---------------|-------|
| Bandwidth | 100 Gbps per link |
| Latency | < 1 ms (intra-city) |
| Redundancy | Dual-path minimum |
| Protocol | Ethernet over Fiber |

### 3.2 5G/6G Mesh Network

| Specification | Value |
|---------------|-------|
| Coverage | 99.9% city area |
| Bandwidth | 10 Gbps peak |
| Latency | < 5 ms |
| Device Density | 1M devices/km² |

### 3.3 Quantum Links (Future-Ready)

```
┌──────────────┐     QKD Channel     ┌──────────────┐
│   Node A     │◄───────────────────►│   Node B     │
│              │   Quantum Key       │              │
│   QKD Chip   │   Distribution      │   QKD Chip   │
└──────────────┘                     └──────────────┘
```

- Quantum Key Distribution (QKD) ready
- Post-quantum cryptography support
- BB84 protocol compatible

---

## 4. Layer 2: Data Layer

### 4.1 Message Encoding

#### JSON-LD Base Format

```json
{
  "@context": "https://schema.wia.org/ai-city/v1",
  "@type": "AIDCMessage",
  "aidc_version": "1.0",
  "layer": 2,
  "message_id": "MSG-2024-XYZ789",
  "message_type": "SENSOR_DATA",
  "source": {
    "@type": "Node",
    "node_id": "AIDC-NODE-001",
    "type": "sensor",
    "location": {
      "@type": "GeoCoordinates",
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "payload": { ... },
  "timestamp": "2024-12-24T10:30:00Z"
}
```

### 4.2 Compression

**Zstd Configuration:**
```
Compression Level: 3 (balanced)
Dictionary: City-specific trained dictionary
Estimated Ratio: 4:1 average
```

### 4.3 Encryption

| Field | Algorithm | Key Size |
|-------|-----------|----------|
| Data at Rest | AES-256-GCM | 256-bit |
| Data in Transit | TLS 1.3 | 256-bit |
| Key Exchange | X25519 | 256-bit |
| Signatures | Ed25519 | 256-bit |

### 4.4 Message Integrity

```
┌─────────────────────────────────────────┐
│  Message Header                          │
│  ├── Version (1 byte)                    │
│  ├── Type (1 byte)                       │
│  ├── Length (4 bytes)                    │
│  └── Flags (2 bytes)                     │
├─────────────────────────────────────────┤
│  Payload (variable)                      │
│  └── Zstd compressed JSON-LD             │
├─────────────────────────────────────────┤
│  Trailer                                 │
│  ├── HMAC-SHA256 (32 bytes)              │
│  └── Ed25519 Signature (64 bytes)        │
└─────────────────────────────────────────┘
```

---

## 5. Layer 3: Intelligence Layer

### 5.1 Edge AI Processing

```
                    Central AI Hub
                         │
          ┌──────────────┼──────────────┐
          │              │              │
     ┌────┴────┐    ┌────┴────┐    ┌────┴────┐
     │ Edge AI │    │ Edge AI │    │ Edge AI │
     │ Zone A  │    │ Zone B  │    │ Zone C  │
     └────┬────┘    └────┬────┘    └────┬────┘
          │              │              │
     ┌────┴────┐    ┌────┴────┐    ┌────┴────┐
     │ Sensors │    │ Sensors │    │ Sensors │
     └─────────┘    └─────────┘    └─────────┘
```

**Edge Node Capabilities:**
- Local inference (< 50ms latency)
- Data pre-processing and filtering
- Anomaly detection
- Privacy-preserving aggregation

### 5.2 Central AI Coordination

```json
{
  "coordination_task": {
    "type": "model_update",
    "target_nodes": ["zone-a", "zone-b"],
    "model_id": "traffic-pred-v3",
    "strategy": "federated_averaging",
    "schedule": "*/5 * * * *"
  }
}
```

### 5.3 Federated Learning Protocol

1. **Model Distribution**
   - Central server distributes base model
   - Each edge node receives model weights

2. **Local Training**
   - Nodes train on local data
   - Citizen data never leaves the zone

3. **Gradient Aggregation**
   - Nodes send gradient updates only
   - Differential privacy applied
   - Central server aggregates

4. **Model Update**
   - New global model computed
   - Distributed back to nodes

```python
# Federated Learning Pseudocode
class FederatedNode:
    def local_train(self, global_model, local_data):
        model = global_model.copy()
        for epoch in range(local_epochs):
            model.train(local_data)
        gradients = model.weights - global_model.weights
        gradients = add_noise(gradients, epsilon=1.0)  # DP
        return gradients

class CentralCoordinator:
    def aggregate(self, all_gradients):
        avg_gradient = sum(all_gradients) / len(all_gradients)
        self.global_model.weights += avg_gradient
        return self.global_model
```

---

## 6. Layer 4: Application Layer

### 6.1 City Services API

| Service | Endpoint | Description |
|---------|----------|-------------|
| Traffic | `/services/traffic` | Traffic management |
| Energy | `/services/energy` | Power grid control |
| Safety | `/services/safety` | Emergency services |
| Environment | `/services/environment` | Air/water quality |
| Transport | `/services/transport` | Public transit |

### 6.2 Service Discovery

```json
{
  "@type": "ServiceRegistry",
  "services": [
    {
      "name": "traffic-management",
      "version": "2.1.0",
      "endpoint": "https://traffic.ai-city.wia.org",
      "capabilities": ["signal_control", "prediction", "routing"],
      "sla": {
        "availability": 99.99,
        "latency_p99_ms": 100
      }
    }
  ]
}
```

### 6.3 Citizen Portal Integration

```
┌────────────────────────────────────────────┐
│           Citizen Mobile App               │
│                                            │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐      │
│  │ Consent │ │ Privacy │ │Services │      │
│  │ Manager │ │ Dashboard│ │  Access │      │
│  └────┬────┘ └────┬────┘ └────┬────┘      │
│       │          │           │            │
│       └──────────┼───────────┘            │
│                  ▼                         │
│         AIDC-Link Layer 4                  │
└────────────────────────────────────────────┘
```

---

## 7. Message Types

### 7.1 SENSOR_DATA

```json
{
  "message_type": "SENSOR_DATA",
  "payload": {
    "sensor_type": "traffic_camera",
    "readings": {
      "vehicle_count": 1247,
      "average_speed": 35.5,
      "congestion_level": "moderate"
    },
    "confidence": 0.95
  }
}
```

### 7.2 CONTROL_COMMAND

```json
{
  "message_type": "CONTROL_COMMAND",
  "payload": {
    "target": "signal-A7-001",
    "action": "adjust_timing",
    "parameters": {
      "green_duration": 45,
      "yellow_duration": 5,
      "cycle_time": 120
    },
    "priority": "normal",
    "require_ack": true
  }
}
```

### 7.3 ALERT

```json
{
  "message_type": "ALERT",
  "payload": {
    "level": "critical",
    "type": "emergency",
    "title": "Traffic Accident Detected",
    "location": {
      "zone": "A7",
      "intersection": "Main St & 5th Ave"
    },
    "affected_services": ["traffic", "emergency"],
    "recommended_actions": [
      "dispatch_emergency",
      "reroute_traffic"
    ]
  }
}
```

### 7.4 ANALYTICS

```json
{
  "message_type": "ANALYTICS",
  "payload": {
    "analysis_type": "prediction",
    "model_id": "traffic-pred-v3",
    "result": {
      "peak_traffic_expected": "17:30",
      "affected_zones": ["A5", "A6", "A7"],
      "confidence": 0.92
    },
    "recommendations": [
      {
        "action": "preemptive_signal_adjustment",
        "target_zones": ["A5", "A6"],
        "start_time": "17:15"
      }
    ]
  }
}
```

### 7.5 HANDSHAKE

```json
{
  "message_type": "HANDSHAKE",
  "payload": {
    "node_id": "AIDC-NODE-001",
    "capabilities": ["sensor", "edge_ai"],
    "supported_versions": ["1.0", "1.1"],
    "public_key": "Ed25519:...",
    "certificate": "X509:..."
  }
}
```

### 7.6 HEARTBEAT

```json
{
  "message_type": "HEARTBEAT",
  "payload": {
    "node_id": "AIDC-NODE-001",
    "status": "healthy",
    "uptime_seconds": 86400,
    "last_activity": "2024-12-24T10:29:55Z",
    "metrics": {
      "cpu_usage": 45,
      "memory_usage": 62,
      "network_throughput_mbps": 850
    }
  }
}
```

---

## 8. Security

### 8.1 Node Authentication

```
┌─────────────┐                     ┌─────────────┐
│   Node A    │                     │   Node B    │
└──────┬──────┘                     └──────┬──────┘
       │                                   │
       │  1. HANDSHAKE (public_key, cert)  │
       │──────────────────────────────────>│
       │                                   │
       │  2. CHALLENGE (nonce)             │
       │<──────────────────────────────────│
       │                                   │
       │  3. RESPONSE (signed nonce)       │
       │──────────────────────────────────>│
       │                                   │
       │  4. SESSION_KEY (encrypted)       │
       │<──────────────────────────────────│
       │                                   │
       │  5. Encrypted Communication       │
       │<─────────────────────────────────>│
```

### 8.2 Certificate Chain

```
WIA Root CA
    │
    └── AI-City Intermediate CA
            │
            ├── Node Certificate (AIDC-NODE-001)
            ├── Node Certificate (AIDC-NODE-002)
            └── ...
```

### 8.3 Access Control

| Role | Permissions |
|------|-------------|
| Sensor | Send SENSOR_DATA, HEARTBEAT |
| Controller | Send/Receive CONTROL_COMMAND |
| Gateway | All Layer 2 messages |
| Central | All messages, ANALYTICS |
| Admin | Full access + configuration |

---

## 9. Error Handling

### 9.1 Error Codes

| Code | Description | Recovery |
|------|-------------|----------|
| E001 | Invalid message format | Reject, request resend |
| E002 | Authentication failed | Re-authenticate |
| E003 | Encryption error | Check keys |
| E004 | Node unreachable | Retry with backoff |
| E005 | Rate limit exceeded | Wait, then retry |
| E006 | Version mismatch | Negotiate version |
| E007 | Signature invalid | Reject message |

### 9.2 Retry Strategy

```
Attempt 1: Immediate
Attempt 2: Wait 100ms
Attempt 3: Wait 500ms
Attempt 4: Wait 2000ms
Attempt 5: Wait 5000ms
Max Attempts: 5
Circuit Breaker: Open after 10 consecutive failures
```

---

## 10. Performance Requirements

| Metric | Requirement | Target |
|--------|-------------|--------|
| Message Latency (intra-zone) | < 50ms | 20ms |
| Message Latency (inter-zone) | < 200ms | 100ms |
| Throughput (per node) | > 10,000 msg/s | 50,000 msg/s |
| Availability | 99.99% | 99.999% |
| Message Loss Rate | < 0.001% | 0.0001% |

---

## 11. Implementation Examples

### 11.1 Node Implementation (Rust)

```rust
use aidc_link::{Node, Message, MessageType};

struct SensorNode {
    node_id: String,
    client: AIDCClient,
}

impl Node for SensorNode {
    async fn send_reading(&self, data: SensorData) -> Result<()> {
        let message = Message::new(MessageType::SensorData)
            .source(&self.node_id)
            .payload(data.to_json_ld())
            .sign(&self.private_key)?;

        self.client.send(message).await
    }

    async fn on_control_command(&self, cmd: ControlCommand) -> Result<()> {
        match cmd.action.as_str() {
            "calibrate" => self.calibrate().await,
            "restart" => self.restart().await,
            _ => Err(Error::UnknownAction)
        }
    }
}
```

### 11.2 Central Hub (Python)

```python
from aidc_link import CentralHub, MessageHandler

class TrafficCoordinator(MessageHandler):
    async def on_sensor_data(self, message):
        zone = message.source.location.zone
        readings = message.payload.readings

        # Run AI prediction
        prediction = await self.ai_model.predict(zone, readings)

        if prediction.congestion_expected:
            # Send control command
            await self.send_command(
                target=f"signals-{zone}",
                action="preemptive_adjustment",
                parameters=prediction.recommended_timing
            )

hub = CentralHub()
hub.register_handler("traffic", TrafficCoordinator())
hub.start()
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
