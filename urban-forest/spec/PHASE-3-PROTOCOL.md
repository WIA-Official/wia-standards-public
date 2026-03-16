# WIA Urban Forest Creation Protocol Standard
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
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange](#data-exchange)
4. [IoT Integration](#iot-integration)
5. [Real-time Monitoring](#real-time-monitoring)
6. [Security](#security)

---

## Overview

### 1.1 Purpose

The WIA Urban Forest Protocol defines communication standards for real-time forest monitoring, IoT sensor integration, and data exchange between stakeholders.

### 1.2 Protocol Stack

```
┌─────────────────────────────────┐
│   Application Layer (HTTP/WS)  │
├─────────────────────────────────┤
│   Security (TLS 1.3, OAuth)    │
├─────────────────────────────────┤
│   Transport (TCP/UDP/MQTT)     │
├─────────────────────────────────┤
│   Network (IPv4/IPv6)          │
└─────────────────────────────────┘
```

---

## Communication Protocols

### 2.1 REST API (HTTP/HTTPS)

**Use Case**: Standard CRUD operations

```http
POST /api/v1/forest HTTP/1.1
Host: api.wia.live
Authorization: Bearer {token}
Content-Type: application/json

{
  "city": "Seoul",
  "trees": [...]
}
```

### 2.2 WebSocket (Real-time Updates)

**Use Case**: Live monitoring dashboard

```javascript
const ws = new WebSocket('wss://api.wia.live/ws/forest/FOREST-2025-SEOUL-001');

ws.on('message', (data) => {
  const update = JSON.parse(data);
  console.log('Tree health update:', update);
});
```

**Message Format:**
```json
{
  "event": "tree_health_update",
  "forestId": "FOREST-2025-SEOUL-001",
  "treeId": "TREE-051",
  "data": {
    "health": "healthy",
    "soilMoisture": 45,
    "temperature": 22
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 2.3 MQTT (IoT Sensors)

**Use Case**: Sensor data collection

```
Topic: wia/forest/{forestId}/sensor/{sensorId}
QoS: 1 (At least once delivery)
Retain: false
```

**Payload:**
```json
{
  "sensorId": "SOIL-001",
  "forestId": "FOREST-2025-SEOUL-001",
  "type": "soil_moisture",
  "value": 45.2,
  "unit": "percentage",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Data Exchange

### 3.1 Batch Upload Protocol

**Endpoint:** `POST /api/v1/forest/batch`

```json
{
  "batchId": "BATCH-2025-001",
  "forests": [
    {
      "forestId": "FOREST-2025-SEOUL-001",
      "trees": [...]
    },
    {
      "forestId": "FOREST-2025-SEOUL-002",
      "trees": [...]
    }
  ]
}
```

**Response:**
```json
{
  "status": "success",
  "processed": 2,
  "failed": 0,
  "results": [
    {"forestId": "FOREST-2025-SEOUL-001", "status": "created"},
    {"forestId": "FOREST-2025-SEOUL-002", "status": "created"}
  ]
}
```

### 3.2 Data Synchronization

**Pull-based sync:**
```http
GET /api/v1/forest/sync?since=2025-01-15T00:00:00Z
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "updates": [
    {
      "forestId": "FOREST-2025-SEOUL-001",
      "changes": ["tree_added", "health_updated"],
      "timestamp": "2025-01-15T10:30:00Z"
    }
  ],
  "nextSyncToken": "sync-token-12345"
}
```

---

## IoT Integration

### 4.1 Sensor Types

| Sensor Type | Protocol | Update Frequency | Data Format |
|------------|----------|------------------|-------------|
| Soil Moisture | MQTT | Every 1 hour | JSON |
| Temperature | MQTT | Every 30 min | JSON |
| Air Quality | MQTT | Every 15 min | JSON |
| Tree Growth | HTTP | Weekly | JSON |
| Camera | HTTP | On-demand | Base64 Image |

### 4.2 Soil Moisture Sensor

**MQTT Topic:** `wia/forest/{forestId}/sensor/soil`

**Payload:**
```json
{
  "sensorId": "SOIL-001",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "data": {
    "moisture": 45.2,
    "temperature": 18.5,
    "pH": 6.8
  },
  "battery": 85,
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.3 Air Quality Sensor

**MQTT Topic:** `wia/forest/{forestId}/sensor/air`

**Payload:**
```json
{
  "sensorId": "AIR-001",
  "data": {
    "pm25": 12.5,
    "pm10": 18.3,
    "co2": 410,
    "humidity": 65
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.4 Growth Camera

**Endpoint:** `POST /api/v1/forest/{forestId}/tree/{treeId}/image`

**Request:**
```json
{
  "treeId": "TREE-051",
  "imageType": "growth_measurement",
  "image": "data:image/jpeg;base64,/9j/4AAQSkZJRg...",
  "metadata": {
    "capturedBy": "camera-001",
    "angle": "front",
    "distance": "2m"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Real-time Monitoring

### 5.1 Event Stream (Server-Sent Events)

**Endpoint:** `GET /api/v1/forest/{forestId}/events`

```javascript
const eventSource = new EventSource(
  'https://api.wia.live/forest/FOREST-2025-SEOUL-001/events'
);

eventSource.addEventListener('health_alert', (e) => {
  const alert = JSON.parse(e.data);
  console.log('Health alert:', alert);
});
```

**Event Types:**
```json
{
  "event": "health_alert",
  "data": {
    "treeId": "TREE-051",
    "severity": "warning",
    "message": "Soil moisture below threshold",
    "value": 15,
    "threshold": 20
  }
}
```

### 5.2 Alert Protocol

**WebSocket:** `wss://api.wia.live/ws/alerts`

```json
{
  "type": "alert",
  "severity": "critical",
  "forestId": "FOREST-2025-SEOUL-001",
  "treeId": "TREE-051",
  "alert": "tree_health_critical",
  "message": "Tree health status changed to critical",
  "recommendations": [
    "Schedule immediate inspection",
    "Check for pest infestation",
    "Verify irrigation system"
  ],
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Security

### 6.1 TLS Configuration

**Minimum TLS Version:** 1.3

**Cipher Suites:**
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256
- TLS_AES_128_GCM_SHA256

### 6.2 API Authentication

**OAuth 2.0 Scopes:**

| Scope | Description |
|-------|-------------|
| `forest:read` | Read forest data |
| `forest:write` | Create/update forests |
| `forest:delete` | Delete forests |
| `sensor:read` | Read sensor data |
| `sensor:write` | Write sensor data |
| `carbon:read` | Read carbon data |
| `analytics:read` | Access analytics |

### 6.3 Data Encryption

**At Rest:**
- AES-256-GCM encryption
- Encrypted database fields
- Secure key management (AWS KMS, Azure Key Vault)

**In Transit:**
- TLS 1.3 for all HTTP traffic
- MQTT over TLS
- WebSocket Secure (WSS)

### 6.4 Sensor Authentication

**MQTT Client Certificates:**
```
Topic: wia/forest/{forestId}/sensor/{sensorId}
Client Certificate: Required
Certificate Validation: CN must match sensorId
```

---

<div align="center">

**WIA Urban Forest Creation Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
