# WIA-CITY-001: Smart City Standard - Phase 3 Protocol

**Version:** 1.0.0
**Status:** Active
**Category:** CITY (Smart City & Urban Systems)

---

## 1. Protocol Overview

### 1.1 Purpose
Define standardized communication protocols for urban IoT integration, data exchange, and system interoperability across all smart city infrastructure components.

### 1.2 Protocol Stack

```
┌────────────────────────────────────────┐
│     Application Layer (Services)      │
│  Digital Twin, Analytics, Dashboards  │
└────────────────────────────────────────┘
                  ↕
┌────────────────────────────────────────┐
│         API Layer (REST/GraphQL)       │
│      WIA-CITY API Specifications      │
└────────────────────────────────────────┘
                  ↕
┌────────────────────────────────────────┐
│      Message Layer (MQTT/CoAP/WS)     │
│        Real-time Data Streaming       │
└────────────────────────────────────────┘
                  ↕
┌────────────────────────────────────────┐
│      Transport Layer (TCP/UDP/TLS)    │
│        Secure Communication           │
└────────────────────────────────────────┘
                  ↕
┌────────────────────────────────────────┐
│        Network Layer (IPv6/LoRa)      │
│         Device Connectivity           │
└────────────────────────────────────────┘
```

---

## 2. Communication Protocols

### 2.1 MQTT (Message Queue Telemetry Transport)

#### 2.1.1 Overview
Primary protocol for real-time sensor data streaming and IoT device communication.

**Specifications:**
- Protocol Version: MQTT 5.0
- QoS Levels: 0 (at most once), 1 (at least once), 2 (exactly once)
- Retained Messages: Supported
- Session Persistence: Enabled

#### 2.1.2 Topic Structure

```
city/{city-id}/{zone}/{domain}/{type}/{action}
```

**Examples:**
```
city/seoul/gangnam/traffic/camera/telemetry
city/seoul/downtown/environment/air-quality/alert
city/seoul/global/energy/grid/command
city/seoul/seocho/safety/streetlight/status
```

#### 2.1.3 Topic Patterns

**Sensor Data:**
```
city/{city-id}/{zone}/{domain}/{device-type}/data
```

**Commands & Control:**
```
city/{city-id}/{zone}/{domain}/{device-type}/command
```

**Alerts & Events:**
```
city/{city-id}/{zone}/{domain}/{device-type}/alert
```

**Status Updates:**
```
city/{city-id}/{zone}/{domain}/{device-type}/status
```

#### 2.1.4 MQTT Message Format

```json
{
  "header": {
    "messageId": "uuid-v4",
    "timestamp": "2025-12-25T10:30:00Z",
    "protocol": "WIA-CITY-MQTT/1.0",
    "qos": 1,
    "retain": false
  },
  "source": {
    "deviceId": "ENV-AIR-001",
    "deviceType": "AirQualitySensor",
    "zone": "gangnam"
  },
  "payload": {
    /* Sensor-specific data */
  },
  "metadata": {
    "encoding": "json",
    "compression": "none",
    "schema": "wia-city-air-quality-v1.0"
  }
}
```

#### 2.1.5 Connection Parameters

```json
{
  "broker": {
    "host": "mqtt.city.wia.org",
    "port": 8883,
    "protocol": "mqtts",
    "tls": "1.3"
  },
  "authentication": {
    "method": "did-auth",
    "clientId": "did:wia:city:device:001",
    "credentials": "device-certificate"
  },
  "session": {
    "cleanStart": false,
    "sessionExpiry": 3600,
    "keepAlive": 60
  },
  "features": {
    "maximumPacketSize": 268435456,
    "receiveMaximum": 65535,
    "topicAliasMaximum": 100
  }
}
```

### 2.2 CoAP (Constrained Application Protocol)

#### 2.2.1 Overview
Lightweight protocol for resource-constrained IoT devices with limited power and bandwidth.

**Specifications:**
- Protocol Version: CoAP (RFC 7252)
- Transport: UDP (default), TCP (optional)
- Security: DTLS 1.3
- Content Format: CBOR, JSON

#### 2.2.2 Resource Structure

```
coap://city.wia.org/{domain}/{zone}/{resource}
```

**Examples:**
```
coap://city.wia.org/sensor/gangnam/air-quality
coap://city.wia.org/traffic/downtown/camera-042
coap://city.wia.org/energy/seocho/meter-52103
```

#### 2.2.3 Methods

- **GET**: Retrieve sensor data or device status
- **POST**: Submit new sensor readings
- **PUT**: Update device configuration
- **DELETE**: Remove device registration
- **OBSERVE**: Subscribe to resource updates

#### 2.2.4 CoAP Request Example

```
GET coap://city.wia.org/sensor/gangnam/air-quality
Observe: 0
Accept: application/cbor
```

**Response:**
```
2.05 Content
Content-Format: application/cbor
Observe: 42

{
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "pm25": 35.2,
  "aqi": 42
}
```

### 2.3 HTTP/REST API

#### 2.3.1 API Endpoints

**Base URL:** `https://api.city.wia.org/v1`

**Core Endpoints:**

```
# Sensor Data
POST   /sensors/data                    # Submit sensor data
GET    /sensors/{sensorId}/data         # Retrieve sensor history
GET    /sensors/{sensorId}/latest       # Get latest reading

# Device Management
POST   /devices/register                # Register new device
GET    /devices/{deviceId}              # Get device info
PUT    /devices/{deviceId}/config       # Update device config
DELETE /devices/{deviceId}              # Deregister device

# Query & Analytics
GET    /query/sensors                   # Query sensor data
POST   /analytics/aggregate             # Run aggregation
GET    /analytics/metrics               # Get city metrics

# Alerts & Events
GET    /alerts                          # List active alerts
POST   /alerts                          # Create alert rule
GET    /events                          # Event stream

# Digital Twin
GET    /twin/model                      # Get digital twin model
POST   /twin/simulate                   # Run simulation
GET    /twin/zones/{zoneId}            # Get zone data
```

#### 2.3.2 Request Format

```http
POST /v1/sensors/data HTTP/1.1
Host: api.city.wia.org
Authorization: Bearer {jwt-token}
Content-Type: application/json
X-WIA-Protocol: WIA-CITY-001
X-Device-ID: ENV-AIR-001

{
  "@context": "https://wia.org/city/v1",
  "@type": "AirQualitySensorData",
  "timestamp": "2025-12-25T10:30:00Z",
  "measurements": [
    {
      "type": "PM2.5",
      "value": 35.2,
      "unit": "μg/m³"
    }
  ]
}
```

#### 2.3.3 Response Format

```http
HTTP/1.1 201 Created
Content-Type: application/json
X-Request-ID: req-12345-67890
X-Processing-Time: 45ms

{
  "status": "success",
  "dataId": "data-20251225-103000-001",
  "timestamp": "2025-12-25T10:30:00.123Z",
  "received": true,
  "validated": true,
  "stored": true
}
```

#### 2.3.4 Error Handling

```http
HTTP/1.1 400 Bad Request
Content-Type: application/json

{
  "status": "error",
  "code": "VALIDATION_FAILED",
  "message": "Invalid sensor data format",
  "errors": [
    {
      "field": "measurements[0].value",
      "issue": "Value out of acceptable range",
      "expected": "0-500",
      "received": "999"
    }
  ],
  "documentation": "https://docs.wia.org/city/errors/validation"
}
```

### 2.4 WebSocket

#### 2.4.1 Overview
Real-time bidirectional communication for dashboards, alerts, and live monitoring.

**Connection:**
```javascript
ws://stream.city.wia.org/v1/live
```

#### 2.4.2 Subscription Message

```json
{
  "action": "subscribe",
  "subscriptions": [
    {
      "type": "sensor-data",
      "zone": "gangnam",
      "domain": "traffic",
      "filters": {
        "deviceType": "camera",
        "updateInterval": 5000
      }
    },
    {
      "type": "alerts",
      "severity": ["high", "critical"]
    }
  ],
  "format": "json",
  "compression": "gzip"
}
```

#### 2.4.3 Data Stream

```json
{
  "type": "data-update",
  "subscriptionId": "sub-001",
  "timestamp": "2025-12-25T10:30:05Z",
  "data": {
    "deviceId": "TRF-CAM-042",
    "vehicleCount": 145,
    "averageSpeed": 27.5
  }
}
```

---

## 3. Data Exchange Formats

### 3.1 JSON-LD (Linked Data)

Primary format for semantic data representation:

```json
{
  "@context": {
    "@vocab": "https://wia.org/city/v1/",
    "geo": "http://www.w3.org/2003/01/geo/wgs84_pos#",
    "time": "http://www.w3.org/2006/time#",
    "sosa": "http://www.w3.org/ns/sosa/"
  },
  "@type": "SensorObservation",
  "@id": "urn:wia:city:observation:001",
  "observedProperty": "AirQuality",
  "madeBySensor": {
    "@id": "urn:wia:city:sensor:ENV-AIR-001",
    "@type": "AirQualitySensor"
  },
  "hasResult": {
    "@type": "QuantitativeValue",
    "value": 35.2,
    "unit": "μg/m³"
  },
  "resultTime": "2025-12-25T10:30:00Z",
  "hasFeatureOfInterest": {
    "@type": "geo:Point",
    "geo:lat": 37.5665,
    "geo:long": 126.9780
  }
}
```

### 3.2 CBOR (Concise Binary Object Representation)

Efficient binary format for bandwidth-constrained devices:

```javascript
// Example encoding
{
  1: "ENV-AIR-001",           // deviceId (key 1)
  2: 1735122600,              // timestamp (UNIX, key 2)
  3: {                        // measurements (key 3)
    1: 35.2,                  // PM2.5 (sub-key 1)
    2: 58.1                   // PM10 (sub-key 2)
  }
}

// CBOR hex: A3 01 6C 45 4E 56 2D 41 49 52 2D 30 30 31 ...
// Size reduction: ~40% vs JSON
```

### 3.3 Protocol Buffers

For high-performance, type-safe data serialization:

```protobuf
syntax = "proto3";

package wia.city.v1;

message SensorData {
  string device_id = 1;
  int64 timestamp = 2;
  Location location = 3;
  repeated Measurement measurements = 4;
}

message Measurement {
  string type = 1;
  double value = 2;
  string unit = 3;
}

message Location {
  double latitude = 1;
  double longitude = 2;
  string zone = 3;
}
```

---

## 4. Security Protocols

### 4.1 Authentication

#### 4.1.1 Device Authentication (DID-based)

```json
{
  "authentication": {
    "method": "did-auth",
    "did": "did:wia:city:device:ENV-AIR-001",
    "challenge": "nonce-12345",
    "proof": {
      "type": "Ed25519Signature2020",
      "created": "2025-12-25T10:30:00Z",
      "verificationMethod": "did:wia:city:device:ENV-AIR-001#key-1",
      "proofValue": "z58DAdFfa9SkqZMVPxAQp...3EysE"
    }
  }
}
```

#### 4.1.2 User Authentication (OAuth 2.0 / OIDC)

```http
POST /oauth/token HTTP/1.1
Host: auth.city.wia.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=city-dashboard
&client_secret={secret}
&scope=city.read city.analytics
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "city.read city.analytics"
}
```

### 4.2 Authorization

#### 4.2.1 Role-Based Access Control (RBAC)

**Roles:**
- `city:admin` - Full system access
- `city:operator` - Operational control
- `city:analyst` - Read analytics data
- `city:citizen` - Public data access
- `city:device` - Device-specific permissions

**Permission Matrix:**
```json
{
  "roles": {
    "city:operator": {
      "permissions": [
        "sensors:read",
        "sensors:write",
        "devices:manage",
        "alerts:create",
        "analytics:read"
      ],
      "restrictions": {
        "zones": ["gangnam", "seocho"],
        "domains": ["traffic", "environment"]
      }
    }
  }
}
```

### 4.3 Encryption

#### 4.3.1 Transport Layer Security

- **TLS Version**: 1.3 (minimum)
- **Cipher Suites**:
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256
- **Certificate**: X.509 with ECDSA (P-256)

#### 4.3.2 Data Encryption

**At Rest:**
- Algorithm: AES-256-GCM
- Key Management: HSM (Hardware Security Module)
- Key Rotation: Every 90 days

**In Transit:**
- All communications over TLS 1.3
- Perfect Forward Secrecy (PFS)
- Certificate pinning for devices

### 4.4 Data Integrity

**Message Signing:**
```json
{
  "data": { /* sensor data */ },
  "signature": {
    "algorithm": "Ed25519",
    "publicKey": "did:wia:city:device:001#key-1",
    "signedAt": "2025-12-25T10:30:00Z",
    "value": "base64-encoded-signature"
  }
}
```

**Verification:**
1. Extract public key from DID document
2. Canonicalize data payload
3. Verify signature using public key
4. Check timestamp freshness (< 5 minutes)

---

## 5. API Specifications

### 5.1 OpenAPI 3.0 Specification

```yaml
openapi: 3.0.3
info:
  title: WIA-CITY-001 Smart City API
  version: 1.0.0
  description: Smart City Data Integration API

servers:
  - url: https://api.city.wia.org/v1
    description: Production server

paths:
  /sensors/data:
    post:
      summary: Submit sensor data
      security:
        - bearerAuth: []
      requestBody:
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SensorData'
      responses:
        '201':
          description: Data accepted
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DataResponse'

components:
  schemas:
    SensorData:
      type: object
      required:
        - deviceId
        - timestamp
        - measurements
      properties:
        deviceId:
          type: string
        timestamp:
          type: string
          format: date-time
        measurements:
          type: array
          items:
            $ref: '#/components/schemas/Measurement'

    Measurement:
      type: object
      properties:
        type:
          type: string
        value:
          type: number
        unit:
          type: string
```

### 5.2 GraphQL API

```graphql
type Query {
  # Sensor queries
  sensor(id: ID!): Sensor
  sensors(zone: String, type: String): [Sensor!]!

  # Data queries
  sensorData(
    sensorId: ID!,
    from: DateTime!,
    to: DateTime!,
    interval: AggregationInterval
  ): [SensorDataPoint!]!

  # City metrics
  cityMetrics(zone: String): CityMetrics!

  # Alerts
  alerts(severity: [AlertSeverity!]): [Alert!]!
}

type Mutation {
  # Submit data
  submitSensorData(input: SensorDataInput!): DataResponse!

  # Device management
  registerDevice(input: DeviceInput!): Device!

  # Alerts
  createAlertRule(input: AlertRuleInput!): AlertRule!
}

type Subscription {
  # Real-time data
  sensorDataUpdates(sensorId: ID!): SensorDataPoint!

  # Alerts
  newAlerts(severity: [AlertSeverity!]): Alert!
}

type Sensor {
  id: ID!
  type: String!
  location: Location!
  status: DeviceStatus!
  latestData: SensorDataPoint
}

type SensorDataPoint {
  timestamp: DateTime!
  measurements: [Measurement!]!
}

type CityMetrics {
  airQuality: AirQualityMetrics!
  traffic: TrafficMetrics!
  energy: EnergyMetrics!
  safety: SafetyMetrics!
}
```

---

## 6. Rate Limiting & Quotas

### 6.1 API Rate Limits

```json
{
  "rateLimits": {
    "citizen": {
      "requestsPerMinute": 60,
      "requestsPerHour": 1000,
      "requestsPerDay": 10000
    },
    "operator": {
      "requestsPerMinute": 300,
      "requestsPerHour": 10000,
      "requestsPerDay": 100000
    },
    "device": {
      "requestsPerMinute": 10,
      "requestsPerHour": 600,
      "dataPointsPerDay": 86400
    }
  }
}
```

### 6.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735122600
```

---

## 7. Protocol Compliance

### 7.1 Standards Conformance

- **MQTT**: OASIS MQTT 5.0
- **CoAP**: RFC 7252, RFC 7959 (Block-Wise Transfer)
- **HTTP**: RFC 9110, RFC 9111
- **WebSocket**: RFC 6455
- **TLS**: RFC 8446 (TLS 1.3)
- **OAuth 2.0**: RFC 6749, RFC 8252
- **DID**: W3C Decentralized Identifiers

### 7.2 Interoperability

Compatible with:
- FIWARE Context Broker (NGSI-LD)
- Eclipse Ditto (Digital Twin)
- AWS IoT Core
- Azure IoT Hub
- Google Cloud IoT

---

**Document Version:** 1.0.0
**Last Updated:** 2025-12-25

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
