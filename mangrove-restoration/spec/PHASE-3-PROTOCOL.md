# WIA Mangrove Restoration Protocol Standard
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
2. [Communication Protocol](#communication-protocol)
3. [Data Synchronization](#data-synchronization)
4. [Real-time Monitoring](#real-time-monitoring)
5. [Mesh Networking](#mesh-networking)
6. [Security](#security)
7. [Performance](#performance)

---

## Overview

### 1.1 Purpose

The WIA Mangrove Restoration Protocol defines communication standards for distributed monitoring networks, enabling real-time data exchange between remote coastal sites, research stations, and central coordination systems.

### 1.2 Protocol Stack

```
┌─────────────────────────────────┐
│   Application Layer             │
│   (Mangrove Data Exchange)      │
├─────────────────────────────────┤
│   Transport Layer               │
│   (MQTT/CoAP/WebSocket)         │
├─────────────────────────────────┤
│   Network Layer                 │
│   (IPv6/LoRaWAN)                │
├─────────────────────────────────┤
│   Link Layer                    │
│   (WiFi/Cellular/Satellite)     │
└─────────────────────────────────┘
```

---

## Communication Protocol

### 2.1 MQTT Topics

**Topic Structure:**

```
wia/mangrove/{region}/{siteId}/{dataType}
```

**Examples:**

```
wia/mangrove/asia/MANG-2025-001/waterquality
wia/mangrove/asia/MANG-2025-001/carbon
wia/mangrove/asia/MANG-2025-001/biodiversity
wia/mangrove/asia/MANG-2025-001/alert
```

### 2.2 Message Format

**Water Quality Update:**

```json
{
  "protocol": "wia-mangrove-v1",
  "timestamp": "2025-01-15T10:30:00Z",
  "siteId": "MANG-2025-000001",
  "type": "waterquality",
  "data": {
    "salinity": {"value": 15.5, "unit": "ppt"},
    "temperature": {"value": 28.5, "unit": "celsius"},
    "pH": 8.1,
    "tidalRange": {"value": 2.5, "unit": "m"}
  },
  "qos": 1,
  "retain": false
}
```

### 2.3 QoS Levels

| Level | Description | Use Case |
|-------|-------------|----------|
| 0 | At most once | Non-critical telemetry |
| 1 | At least once | Water quality data |
| 2 | Exactly once | Carbon credits, alerts |

---

## Data Synchronization

### 3.1 Event-Driven Sync

**Site Status Change:**

```json
{
  "event": "site.status.changed",
  "timestamp": "2025-01-15T10:30:00Z",
  "siteId": "MANG-2025-000001",
  "changes": {
    "status": {
      "from": "planning",
      "to": "active"
    }
  },
  "metadata": {
    "triggeredBy": "user-123",
    "reason": "Initial planting completed"
  }
}
```

### 3.2 Periodic Sync

**Daily Summary:**

```json
{
  "protocol": "wia-mangrove-sync-v1",
  "syncType": "daily-summary",
  "period": {
    "start": "2025-01-15T00:00:00Z",
    "end": "2025-01-15T23:59:59Z"
  },
  "sites": [
    {
      "siteId": "MANG-2025-000001",
      "dataPoints": 288,
      "alerts": 0,
      "status": "healthy"
    }
  ],
  "hash": "sha256:a1b2c3..."
}
```

### 3.3 Conflict Resolution

**Strategy: Last-Write-Wins with Vector Clocks**

```json
{
  "siteId": "MANG-2025-000001",
  "field": "coverage.total.value",
  "value": 26.0,
  "vectorClock": {
    "server-1": 15,
    "sensor-node-42": 8
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Real-time Monitoring

### 4.1 WebSocket Protocol

**Connection:**

```javascript
const ws = new WebSocket('wss://stream.wia.live/mangrove/v1');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    topics: ['wia/mangrove/asia/+/alert']
  }));
});

ws.on('message', (data) => {
  const alert = JSON.parse(data);
  handleMangroveAlert(alert);
});
```

**Alert Message:**

```json
{
  "type": "alert",
  "severity": "high",
  "siteId": "MANG-2025-000001",
  "message": "Salinity spike detected",
  "data": {
    "current": {"value": 42.0, "unit": "ppt"},
    "threshold": {"value": 35.0, "unit": "ppt"},
    "deviation": "+20%"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.2 Server-Sent Events (SSE)

```http
GET /api/v1/mangrove/sites/MANG-2025-001/stream
Accept: text/event-stream
Authorization: Bearer {token}
```

**Response:**

```
data: {"type":"waterquality","salinity":15.5,"temp":28.5}

data: {"type":"coverage","value":26.0,"unit":"hectares"}

data: {"type":"heartbeat","timestamp":"2025-01-15T10:30:00Z"}
```

---

## Mesh Networking

### 4.1 LoRaWAN Integration

**Device Configuration:**

```json
{
  "deviceId": "LORA-MANG-001",
  "siteId": "MANG-2025-000001",
  "lorawan": {
    "devEUI": "0004A30B001A2B3C",
    "appEUI": "70B3D57ED0000001",
    "appKey": "01020304050607080910111213141516",
    "region": "AS923",
    "dataRate": "SF7BW125"
  },
  "sensors": ["salinity", "temperature", "water_level"]
}
```

**Uplink Payload (Cayenne LPP):**

```
01 67 0F 5A    // Channel 1: Temperature = 39.46°C
02 73 3E 80    // Channel 2: Salinity = 16.0 ppt
03 02 00 FA    // Channel 3: Water Level = 250 cm
```

### 4.2 Multi-hop Routing

**Route Discovery:**

```json
{
  "source": "MANG-2025-001",
  "destination": "GATEWAY-01",
  "route": [
    {"hop": 1, "node": "MANG-2025-002", "rssi": -85},
    {"hop": 2, "node": "MANG-2025-003", "rssi": -75},
    {"hop": 3, "node": "GATEWAY-01", "rssi": -65}
  ],
  "totalHops": 3,
  "quality": "good"
}
```

---

## Security

### 5.1 End-to-End Encryption

**Message Encryption (AES-256-GCM):**

```json
{
  "encrypted": "base64encodedciphertext",
  "nonce": "base64encodednonce",
  "tag": "base64encodedtag",
  "keyId": "key-2025-001"
}
```

### 5.2 Device Authentication

**DTLS Handshake:**

```
Client                          Server
  |                               |
  |-- ClientHello --------------->|
  |                               |
  |<-- ServerHello, Certificate --|
  |                               |
  |-- ClientKeyExchange --------->|
  |-- Finished ------------------->|
  |                               |
  |<-- Finished -------------------|
  |                               |
```

### 5.3 Data Integrity

**Blockchain Anchoring:**

```json
{
  "siteId": "MANG-2025-000001",
  "dataHash": "sha256:a1b2c3d4...",
  "blockchainAnchor": {
    "network": "ethereum",
    "txHash": "0xabc123...",
    "blockNumber": 18500000,
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

---

## Performance

### 6.1 Latency Requirements

| Data Type | Max Latency | Priority |
|-----------|-------------|----------|
| Alerts | 5 seconds | Critical |
| Water Quality | 1 minute | High |
| Carbon Data | 1 hour | Medium |
| Reports | 24 hours | Low |

### 6.2 Bandwidth Optimization

**Data Compression (CBOR):**

```javascript
// JSON: 156 bytes
{
  "siteId": "MANG-2025-000001",
  "salinity": {"value": 15.5, "unit": "ppt"},
  "temp": {"value": 28.5, "unit": "celsius"}
}

// CBOR: 48 bytes (69% reduction)
A3 67 73 69 74 65 49 64 70 4D 41 4E 47 2D 32 30 32 35 2D 30 30 30 30 30 31 ...
```

### 6.3 Edge Computing

**On-Device Processing:**

```python
# Process locally to reduce transmission
def calculate_daily_average(sensor_data):
    avg_salinity = np.mean([d['salinity'] for d in sensor_data])
    avg_temp = np.mean([d['temp'] for d in sensor_data])

    # Only send summary instead of 288 raw readings
    return {
        'date': '2025-01-15',
        'avg_salinity': avg_salinity,
        'avg_temp': avg_temp,
        'samples': len(sensor_data)
    }
```

---

## Protocol Examples

### 7.1 Complete MQTT Session

```bash
# Subscribe to site alerts
mosquitto_sub -h broker.wia.live \
  -t 'wia/mangrove/asia/+/alert' \
  -u user123 \
  -P password \
  --cafile ca.crt

# Publish water quality data
mosquitto_pub -h broker.wia.live \
  -t 'wia/mangrove/asia/MANG-2025-001/waterquality' \
  -m '{"salinity":15.5,"temp":28.5}' \
  -q 1 \
  -u user123 \
  -P password \
  --cafile ca.crt
```

### 7.2 CoAP Resource Discovery

```bash
# Discover available resources
coap-client -m get coap://sensor.wia.live/.well-known/core

# Response
</mangrove/salinity>;rt="sensor.salinity";if="sensor",
</mangrove/temperature>;rt="sensor.temperature";if="sensor",
</mangrove/water-level>;rt="sensor.level";if="sensor"
```

---

<div align="center">

**WIA Mangrove Restoration Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
