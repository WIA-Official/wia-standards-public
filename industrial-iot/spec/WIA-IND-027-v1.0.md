# WIA-IND-027: Industrial IoT Specification v1.0

> **Standard ID:** WIA-IND-027
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Industrial IoT Architecture](#2-industrial-iot-architecture)
3. [IIoT Device Protocols](#3-iiot-device-protocols)
4. [Edge Computing](#4-edge-computing)
5. [Time-Series Data Management](#5-time-series-data-management)
6. [Real-Time Monitoring](#6-real-time-monitoring)
7. [Alert and Notification Systems](#7-alert-and-notification-systems)
8. [Device Management](#8-device-management)
9. [Industrial Security](#9-industrial-security)
10. [Data Aggregation](#10-data-aggregation)
11. [MES/ERP Integration](#11-meserp-integration)
12. [Digital Twin](#12-digital-twin)
13. [Performance Optimization](#13-performance-optimization)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for Industrial Internet of Things (IIoT) systems, enabling seamless connectivity, real-time monitoring, and intelligent automation across manufacturing and industrial environments.

### 1.2 Scope

The standard covers:
- IIoT device communication protocols (OPC-UA, MQTT, Modbus, PROFINET)
- Edge computing architectures for real-time processing
- Time-series database design for industrial data
- Manufacturing dashboards and visualization
- Predictive maintenance and anomaly detection
- Secure device management and firmware updates
- Integration with Manufacturing Execution Systems (MES) and Enterprise Resource Planning (ERP)
- Digital twin technology for virtual asset modeling
- Industrial cybersecurity best practices

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize industrial automation technology, making advanced manufacturing capabilities accessible to factories of all sizes, improving worker safety, and accelerating sustainable industrial practices globally.

### 1.4 Terminology

- **IIoT (Industrial IoT)**: Internet of Things applied to industrial settings
- **PLC (Programmable Logic Controller)**: Industrial computer for automation
- **SCADA (Supervisory Control and Data Acquisition)**: Control system architecture
- **OPC-UA**: Open Platform Communications - Unified Architecture
- **Edge Gateway**: Local processing node between devices and cloud
- **Time-Series Data**: Sequential data points indexed by timestamp
- **OEE (Overall Equipment Effectiveness)**: Manufacturing productivity metric
- **MES**: Manufacturing Execution System
- **ERP**: Enterprise Resource Planning system
- **Digital Twin**: Virtual representation of physical asset
- **RUL (Remaining Useful Life)**: Predicted time until failure

---

## 2. Industrial IoT Architecture

### 2.1 Three-Layer Architecture

The IIoT architecture consists of three primary layers:

```
┌─────────────────────────────────────────────────────────────┐
│                    CLOUD / ENTERPRISE LAYER                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │   MES    │  │   ERP    │  │ Analytics│  │  Digital │   │
│  │          │  │          │  │   & AI   │  │   Twin   │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↕ (Internet/WAN)
┌─────────────────────────────────────────────────────────────┐
│                      EDGE COMPUTING LAYER                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Edge Gateway                                        │  │
│  │  • Data Aggregation    • Local Analytics            │  │
│  │  • Protocol Translation • Real-time Processing      │  │
│  │  • Filtering & Buffering • Offline Capability       │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕ (Industrial Network)
┌─────────────────────────────────────────────────────────────┐
│                    DEVICE / FIELD LAYER                      │
│  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐    │
│  │ PLC │  │Robot│  │Sensor│ │Motor│  │CNC  │  │ AGV │    │
│  └─────┘  └─────┘  └─────┘  └─────┘  └─────┘  └─────┘    │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Layer Responsibilities

#### 2.2.1 Device/Field Layer
- Direct interaction with physical equipment
- Data acquisition from sensors
- Control of actuators and motors
- Local control loops (PID controllers)
- Safety-critical real-time operations

**Typical devices**:
- PLCs (Siemens S7, Allen-Bradley, Mitsubishi)
- Industrial robots (ABB, FANUC, KUKA)
- Temperature, pressure, vibration sensors
- Motor drives and inverters
- CNC machines
- Automated Guided Vehicles (AGVs)

#### 2.2.2 Edge Computing Layer
- Protocol translation (OPC-UA, MQTT, Modbus)
- Data aggregation from multiple sources
- Local analytics and decision making
- Buffering for network interruptions
- Filtering to reduce cloud traffic
- Real-time alerting

**Typical hardware**:
- Industrial PCs (Beckhoff, Advantech)
- Edge servers (Dell Edge Gateway 3000)
- IoT gateways (Cisco, Siemens)
- Raspberry Pi for lightweight applications

#### 2.2.3 Cloud/Enterprise Layer
- Long-term data storage
- Advanced analytics and machine learning
- Digital twin simulation
- MES and ERP integration
- Business intelligence dashboards
- Multi-site aggregation

**Typical services**:
- AWS IoT Core, Azure IoT Hub, Google Cloud IoT
- InfluxDB Cloud, TimescaleDB
- Grafana, Tableau, Power BI
- SAP MES, Siemens Opcenter
- Oracle ERP, SAP S/4HANA

### 2.3 Network Topologies

#### 2.3.1 Star Topology
All devices connect to central gateway:

```
        ┌────────────┐
        │   Gateway  │
        └────────────┘
         ↙   ↓   ↓   ↘
    [PLC] [Sensor] [Robot] [CNC]
```

**Advantages**: Simple, centralized management
**Disadvantages**: Single point of failure

#### 2.3.2 Hierarchical Topology
Multi-level gateway structure:

```
           ┌──────────────┐
           │ Master Gateway│
           └──────────────┘
              ↙          ↘
    ┌──────────┐      ┌──────────┐
    │ Gateway A│      │ Gateway B│
    └──────────┘      └──────────┘
       ↙    ↘            ↙    ↘
   [PLC] [Sensor]   [Robot] [CNC]
```

**Advantages**: Scalable, segmented
**Disadvantages**: More complex

#### 2.3.3 Mesh Topology
Peer-to-peer device connectivity:

```
    [PLC] ←→ [Sensor]
      ↕  ×    ×   ↕
    [Robot] ←→ [CNC]
```

**Advantages**: Redundant, self-healing
**Disadvantages**: Higher bandwidth, complex routing

### 2.4 Communication Patterns

#### 2.4.1 Request/Response
Synchronous communication for critical operations:

```
Client → [Request] → Server
Client ← [Response] ← Server
```

**Use cases**: PLC register reads, configuration updates

#### 2.4.2 Publish/Subscribe
Asynchronous event-driven communication:

```
Publisher → [Topic: sensors/temp] → Broker
                                       ↓
Subscriber ← [Topic: sensors/temp] ← Broker
```

**Use cases**: Sensor telemetry, status updates

#### 2.4.3 Streaming
Continuous data flow:

```
Device → [Stream] → Gateway → [Stream] → Cloud
```

**Use cases**: Video feeds, high-frequency sensor data

---

## 3. IIoT Device Protocols

### 3.1 OPC-UA (Unified Architecture)

#### 3.1.1 Overview
OPC-UA is the modern standard for industrial communication, providing:
- Platform-independent communication
- Service-oriented architecture
- Built-in security (encryption, authentication)
- Rich information modeling
- Scalability from embedded to cloud

#### 3.1.2 Architecture

**Client/Server Model**:
```
┌──────────┐                 ┌──────────┐
│ OPC-UA   │    TCP/IP       │ OPC-UA   │
│ Client   │ ←────────────→  │ Server   │
└──────────┘                 └──────────┘
```

**Publish/Subscribe Model** (OPC-UA Part 14):
```
┌──────────┐
│Publisher │ →
└──────────┘   ↘
                 ↘  ┌────────┐
┌──────────┐    → │ Broker │
│Publisher │ →  ↗  └────────┘
└──────────┘   ↗        ↓
              ↗         ↓
                    ┌────────────┐
                    │ Subscriber │
                    └────────────┘
```

#### 3.1.3 Information Model

**Address Space**:
Every data point is a node with unique NodeId:

```
NodeId format: ns=<namespace>;i=<identifier>
Example: ns=2;s=Temperature_Sensor_01
```

**Node Classes**:
- **Object**: Represents a physical or logical object
- **Variable**: Contains data values
- **Method**: Executable function
- **ObjectType**: Template for objects
- **DataType**: Defines value type

**Example Information Model**:
```
[Server]
  └─ [Objects]
       └─ [DeviceSet]
            └─ [Machine_001]
                 ├─ [Temperature] (Variable, Double)
                 ├─ [Pressure] (Variable, Double)
                 ├─ [Status] (Variable, String)
                 └─ [StartMachine] (Method)
```

#### 3.1.4 Services

**Read Service**:
```
Request:
  NodeId: ns=2;s=Temperature
  AttributeId: Value

Response:
  Value: 45.2
  StatusCode: Good
  SourceTimestamp: 2025-01-15T10:30:00Z
```

**Write Service**:
```
Request:
  NodeId: ns=2;s=Setpoint
  Value: 50.0

Response:
  StatusCode: Good
```

**Subscription Service**:
```
CreateSubscription:
  PublishingInterval: 1000ms
  LifetimeCount: 10000
  MaxKeepAliveCount: 10

CreateMonitoredItems:
  NodeId: ns=2;s=Temperature
  SamplingInterval: 500ms
  QueueSize: 10

Notification:
  Value: 45.5
  SourceTimestamp: 2025-01-15T10:30:01Z
```

#### 3.1.5 Security

**Security Modes**:
1. **None**: No encryption (not recommended for production)
2. **Sign**: Message signing for integrity
3. **SignAndEncrypt**: Full encryption and signing

**Security Policies**:
- **Basic256Sha256**: AES-256, SHA-256
- **Aes128Sha256RsaOaep**: AES-128, SHA-256, RSA-OAEP
- **Aes256Sha256RsaPss**: AES-256, SHA-256, RSA-PSS (recommended)

**Authentication**:
- Anonymous (no credentials)
- Username/Password
- X.509 Certificate
- Issued Token (JWT, SAML)

**Example Secure Connection**:
```javascript
const client = OPCUAClient.create({
  securityMode: MessageSecurityMode.SignAndEncrypt,
  securityPolicy: SecurityPolicy.Aes256Sha256RsaPss,
  certificateFile: "client_cert.pem",
  privateKeyFile: "client_key.pem",
  serverCertificate: "server_cert.der"
});

await client.connect("opc.tcp://192.168.1.100:4840");
const session = await client.createSession({
  type: UserTokenType.UserName,
  userName: "operator",
  password: "secure_password"
});
```

### 3.2 MQTT (Message Queuing Telemetry Transport)

#### 3.2.1 Overview
MQTT is a lightweight publish/subscribe protocol ideal for:
- Low bandwidth environments
- Unreliable networks
- Resource-constrained devices
- One-to-many message distribution

#### 3.2.2 Architecture

```
┌──────────┐      ┌──────────┐      ┌──────────┐
│Publisher │ ───→ │  Broker  │ ───→ │Subscriber│
└──────────┘      └──────────┘      └──────────┘
                       ↑
                       │ (Topics)
                       ├─ sensors/temperature
                       ├─ sensors/pressure
                       ├─ machines/+/status
                       └─ alerts/#
```

#### 3.2.3 Topic Design

**Best Practices**:
```
# Good: Hierarchical, specific
sensors/factory-001/line-A/temperature
sensors/factory-001/line-A/pressure
machines/cnc-001/status
machines/cnc-001/alarms

# Bad: Flat, generic
temperature
sensor_data
machine_status
```

**Wildcards**:
- **+** (single level): `sensors/+/temperature` matches `sensors/line-A/temperature`
- **#** (multi-level): `sensors/#` matches all under `sensors/`

#### 3.2.4 Quality of Service (QoS)

**QoS Levels**:

| Level | Name | Guarantee | Use Case |
|-------|------|-----------|----------|
| 0 | At most once | Best effort, no ACK | Non-critical telemetry |
| 1 | At least once | Guaranteed delivery, may duplicate | Important events |
| 2 | Exactly once | No duplicates, highest overhead | Critical commands |

**Example**:
```javascript
// Publish with QoS 1
client.publish('sensors/temp-001', JSON.stringify({
  value: 45.2,
  unit: 'celsius',
  timestamp: Date.now()
}), { qos: 1 });

// Subscribe with QoS 2
client.subscribe('machines/+/alarms', { qos: 2 });
```

#### 3.2.5 Retained Messages

The broker stores the last message for new subscribers:

```javascript
// Publish retained status
client.publish('machines/cnc-001/status', 'running', {
  qos: 1,
  retain: true
});

// New subscriber immediately receives last status
client.subscribe('machines/cnc-001/status');
// → Receives: 'running'
```

#### 3.2.6 Last Will and Testament (LWT)

Automatic notification when client disconnects:

```javascript
client.connect('mqtt://broker.factory.com', {
  will: {
    topic: 'devices/sensor-001/status',
    payload: 'offline',
    qos: 1,
    retain: true
  }
});
```

### 3.3 Modbus

#### 3.3.1 Overview
Modbus is a legacy protocol still widely used in industrial environments:
- **Modbus RTU**: Serial communication (RS-232, RS-485)
- **Modbus TCP**: Ethernet/IP based
- Simple master/slave architecture
- Register-based data model

#### 3.3.2 Function Codes

| Code | Function | Description |
|------|----------|-------------|
| 01 | Read Coils | Read 1-2000 contiguous coils |
| 02 | Read Discrete Inputs | Read 1-2000 discrete inputs |
| 03 | Read Holding Registers | Read 1-125 holding registers |
| 04 | Read Input Registers | Read 1-125 input registers |
| 05 | Write Single Coil | Write single coil |
| 06 | Write Single Register | Write single holding register |
| 15 | Write Multiple Coils | Write multiple coils |
| 16 | Write Multiple Registers | Write multiple holding registers |

#### 3.3.3 Data Model

```
Coils (0x):           00001 - 09999 (Read/Write, Boolean)
Discrete Inputs (1x): 10001 - 19999 (Read-Only, Boolean)
Input Registers (3x): 30001 - 39999 (Read-Only, 16-bit)
Holding Registers (4x): 40001 - 49999 (Read/Write, 16-bit)
```

**Example Read**:
```
Request (Read Holding Registers):
  Slave Address: 1
  Function Code: 03
  Start Address: 40001 (Temperature)
  Quantity: 2 (2 registers = 32-bit float)

Response:
  Slave Address: 1
  Function Code: 03
  Byte Count: 4
  Data: [0x42, 0x34, 0xCC, 0xCD] (45.2°C as IEEE 754 float)
```

#### 3.3.4 Modbus TCP Frame

```
┌───────────────┬──────────┬─────────────┬──────────────┐
│ MBAP Header   │ Function │ Data        │ CRC (RTU)    │
│ (7 bytes)     │ Code     │             │ or none (TCP)│
├───────────────┼──────────┼─────────────┼──────────────┤
│ Transaction ID│    03    │ Start: 0000 │              │
│ Protocol ID   │          │ Qty: 0002   │              │
│ Length        │          │             │              │
│ Unit ID       │          │             │              │
└───────────────┴──────────┴─────────────┴──────────────┘
```

### 3.4 PROFINET

#### 3.4.1 Overview
PROFINET is a real-time Industrial Ethernet protocol by Siemens:
- Deterministic communication
- Hard real-time support (Isochronous Real-Time)
- Suitable for motion control
- Integration with TIA Portal

#### 3.4.2 Communication Classes

| Class | Latency | Jitter | Use Case |
|-------|---------|--------|----------|
| TCP/IP | 100ms | High | Non-critical data |
| RT (Real-Time) | 1-10ms | Medium | Automation |
| IRT (Isochronous RT) | <1ms | <1µs | Motion control |

#### 3.4.3 Topology

```
┌──────────────┐
│ PROFINET IO  │
│ Controller   │ (PLC)
└──────────────┘
       │
       ├─── [Switch] ──┬─── [IO Device 1] (Sensor module)
       │               ├─── [IO Device 2] (Actuator)
       │               └─── [IO Device 3] (Drive)
       │
       └─── [Switch] ──┬─── [IO Device 4]
                       └─── [IO Device 5]
```

#### 3.4.4 Device Configuration (GSD File)

PROFINET devices are configured using GSD (General Station Description) XML files:

```xml
<ISO15745Profile>
  <ProfileBody>
    <DeviceIdentity VendorId="0x002A" DeviceID="0x0001">
      <InfoText TextId="DeviceName">
        Temperature Sensor Module
      </InfoText>
    </DeviceIdentity>
    <ModuleInfo>
      <Module ID="1">
        <Input Length="4" /> <!-- 32-bit temperature -->
      </Module>
    </ModuleInfo>
  </ProfileBody>
</ISO15745Profile>
```

---

## 4. Edge Computing

### 4.1 Edge Architecture

#### 4.1.1 Edge Gateway Components

```
┌────────────────────────────────────────────────────────┐
│              Edge Gateway                              │
├────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────┐ │
│  │  Protocol Adapters                               │ │
│  │  [OPC-UA] [MQTT] [Modbus] [PROFINET]            │ │
│  └──────────────────────────────────────────────────┘ │
│                        ↓                                │
│  ┌──────────────────────────────────────────────────┐ │
│  │  Data Normalization & Transformation             │ │
│  │  • Unit conversion  • Timestamp synchronization  │ │
│  └──────────────────────────────────────────────────┘ │
│                        ↓                                │
│  ┌──────────────────────────────────────────────────┐ │
│  │  Local Processing & Analytics                    │ │
│  │  • Filtering  • Aggregation  • Edge ML          │ │
│  └──────────────────────────────────────────────────┘ │
│                        ↓                                │
│  ┌──────────────────────────────────────────────────┐ │
│  │  Data Router                                     │ │
│  │  • Local storage  • Cloud sync  • Alert engine  │ │
│  └──────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────┘
```

### 4.2 Data Filtering

#### 4.2.1 Dead-band Filtering

Only transmit data when value changes beyond threshold:

```javascript
function deadbandFilter(value, lastValue, deadband) {
  const change = Math.abs(value - lastValue);
  return change >= deadband;
}

// Example: Temperature with ±0.5°C deadband
let lastTemp = 45.0;
const newTemp = 45.3;

if (deadbandFilter(newTemp, lastTemp, 0.5)) {
  transmit(newTemp); // No transmission, within deadband
}

const newTemp2 = 45.6;
if (deadbandFilter(newTemp2, lastTemp, 0.5)) {
  transmit(newTemp2); // Transmit: 0.6°C change > 0.5°C threshold
  lastTemp = newTemp2;
}
```

**Benefits**:
- Reduce network traffic by 50-90%
- Lower cloud storage costs
- Preserve signal fidelity

#### 4.2.2 Statistical Sampling

Transmit statistical summary instead of raw data:

```javascript
// Collect 100 samples over 10 seconds
const samples = [45.1, 45.2, 45.0, ..., 45.3]; // 100 values

// Transmit summary
transmit({
  timestamp: Date.now(),
  mean: 45.15,
  min: 44.8,
  max: 45.5,
  stddev: 0.18,
  count: 100
});
```

**Benefits**:
- 100x data reduction
- Preserve statistical properties
- Suitable for trending analysis

### 4.3 Edge Analytics

#### 4.3.1 Anomaly Detection

**Z-Score Method**:
```javascript
function calculateZScore(value, mean, stddev) {
  return (value - mean) / stddev;
}

const historicalMean = 45.0;
const historicalStddev = 2.0;
const currentValue = 52.0;

const zScore = calculateZScore(currentValue, historicalMean, historicalStddev);
// zScore = 3.5

if (Math.abs(zScore) > 3) {
  raiseAlert('Temperature anomaly detected: ' + currentValue);
}
```

**Moving Average Deviation**:
```javascript
function movingAverage(values, window) {
  const sum = values.slice(-window).reduce((a, b) => a + b, 0);
  return sum / window;
}

const recentValues = [45, 46, 45, 44, 46, 52]; // Latest 6 samples
const ma = movingAverage(recentValues, 5); // 45.2
const latest = 52;

if (Math.abs(latest - ma) > 5) {
  raiseAlert('Sudden spike detected');
}
```

#### 4.3.2 Predictive Maintenance

**Vibration Analysis**:
```javascript
// FFT (Fast Fourier Transform) for frequency analysis
const vibrationData = collectVibrationSamples(1024); // 1024 samples
const spectrum = fft(vibrationData);

// Check for bearing fault frequencies
const shaftSpeed = 1800; // RPM
const bearingFTF = shaftSpeed * 0.4; // Fundamental Train Frequency

if (spectrum[bearingFTF] > threshold) {
  raiseAlert('Bearing fault detected at ' + bearingFTF + ' Hz');
}
```

**Remaining Useful Life (RUL)**:
```javascript
// Linear degradation model
function estimateRUL(currentHealth, degradationRate) {
  const failureThreshold = 0.3; // 30% health
  const remainingHealth = currentHealth - failureThreshold;
  const rulHours = remainingHealth / degradationRate;
  return rulHours;
}

const health = 0.75; // 75% health
const degradation = 0.001; // 0.1% per hour
const rul = estimateRUL(health, degradation);
console.log(`Estimated RUL: ${rul} hours (${rul/24} days)`);
```

### 4.4 Offline Capability

#### 4.4.1 Store-and-Forward

Buffer data during network outages:

```javascript
class StoreAndForward {
  constructor(maxSize = 10000) {
    this.buffer = [];
    this.maxSize = maxSize;
    this.isOnline = true;
  }

  async send(data) {
    if (this.isOnline) {
      try {
        await cloudAPI.send(data);
      } catch (error) {
        this.isOnline = false;
        this.buffer.push(data);
      }
    } else {
      this.buffer.push(data);
      if (this.buffer.length > this.maxSize) {
        this.buffer.shift(); // Remove oldest
      }
      this.tryReconnect();
    }
  }

  async tryReconnect() {
    try {
      await cloudAPI.ping();
      this.isOnline = true;
      await this.flushBuffer();
    } catch (error) {
      // Remain offline
    }
  }

  async flushBuffer() {
    while (this.buffer.length > 0 && this.isOnline) {
      const data = this.buffer.shift();
      await cloudAPI.send(data);
    }
  }
}
```

---

## 5. Time-Series Data Management

### 5.1 Time-Series Database (TSDB) Design

#### 5.1.1 Schema Design

**InfluxDB Schema**:
```
measurement: sensor_data
tags:
  - sensor_id (indexed)
  - location (indexed)
  - equipment_type (indexed)
fields:
  - temperature (float)
  - pressure (float)
  - vibration (float)
timestamp: nanosecond precision
```

**Example Write**:
```javascript
const point = {
  measurement: 'sensor_data',
  tags: {
    sensor_id: 'temp-001',
    location: 'line-A',
    equipment_type: 'furnace'
  },
  fields: {
    temperature: 850.5,
    pressure: 1.2
  },
  timestamp: new Date()
};

await influx.writePoints([point]);
```

#### 5.1.2 Indexing Strategy

**Tag Indexing** (for filtering):
- sensor_id: High cardinality (1,000-100,000 unique values)
- location: Low cardinality (10-100 locations)
- equipment_type: Low cardinality (10-50 types)

**Best Practices**:
- Use tags for dimensions you filter/group by
- Use fields for metrics you aggregate
- Limit tag cardinality to prevent index bloat
- Avoid high-cardinality tags (e.g., timestamp strings, UUIDs)

### 5.2 Query Optimization

#### 5.2.1 Time Range Queries

```sql
-- Efficient: Specific time range
SELECT mean("temperature")
FROM "sensor_data"
WHERE time >= '2025-01-01T00:00:00Z'
  AND time <= '2025-01-02T00:00:00Z'
  AND "sensor_id" = 'temp-001'
GROUP BY time(5m)

-- Inefficient: No time filter
SELECT mean("temperature")
FROM "sensor_data"
WHERE "sensor_id" = 'temp-001'
GROUP BY time(5m)
```

#### 5.2.2 Downsampling

Pre-aggregate data for long-term storage:

```sql
-- Create continuous query for 1-hour averages
CREATE CONTINUOUS QUERY "cq_1h_avg" ON "factory_db"
BEGIN
  SELECT mean("temperature") AS "temperature",
         mean("pressure") AS "pressure"
  INTO "sensor_data_1h"
  FROM "sensor_data"
  GROUP BY time(1h), *
END
```

**Retention Policies**:
```sql
-- Raw data: 7 days
CREATE RETENTION POLICY "raw_7d" ON "factory_db"
  DURATION 7d REPLICATION 1 DEFAULT

-- 1-minute aggregates: 90 days
CREATE RETENTION POLICY "agg_1m_90d" ON "factory_db"
  DURATION 90d REPLICATION 1

-- 1-hour aggregates: 2 years
CREATE RETENTION POLICY "agg_1h_2y" ON "factory_db"
  DURATION 104w REPLICATION 1
```

### 5.3 Data Compression

#### 5.3.1 Gorilla Compression

Facebook's Gorilla algorithm for time-series:
- XOR-based compression for floating-point values
- Delta encoding for timestamps
- Typical compression ratio: 10:1 to 30:1

```
Original: [45.1, 45.2, 45.1, 45.3, 45.2] (40 bytes as doubles)
Compressed: [45.1, +0.1, -0.1, +0.2, -0.1] (12 bytes)
Compression: 70% reduction
```

#### 5.3.2 Dead-band Compression

Store only significant changes:

```javascript
function compressDeadband(values, threshold) {
  const compressed = [values[0]]; // Always store first
  let last = values[0];

  for (let i = 1; i < values.length; i++) {
    if (Math.abs(values[i] - last) >= threshold) {
      compressed.push(values[i]);
      last = values[i];
    }
  }

  return compressed;
}

const raw = [45.1, 45.15, 45.12, 45.18, 45.6, 45.58];
const compressed = compressDeadband(raw, 0.2);
// Result: [45.1, 45.6] (67% reduction)
```

---

## 6. Real-Time Monitoring

### 6.1 Manufacturing Metrics

#### 6.1.1 OEE (Overall Equipment Effectiveness)

```
OEE = Availability × Performance × Quality

Where:
  Availability = (Operating Time / Planned Production Time) × 100%
  Performance = (Actual Output / Target Output) × 100%
  Quality = (Good Units / Total Units) × 100%
```

**Example Calculation**:
```javascript
function calculateOEE(data) {
  const plannedTime = 480; // 8 hours in minutes
  const downtime = 45; // minutes
  const operatingTime = plannedTime - downtime;
  const availability = operatingTime / plannedTime;

  const targetCycleTime = 1.0; // minutes per unit
  const targetOutput = operatingTime / targetCycleTime;
  const actualOutput = 400; // units
  const performance = actualOutput / targetOutput;

  const goodUnits = 380; // units
  const quality = goodUnits / actualOutput;

  const oee = availability * performance * quality;

  return {
    availability: (availability * 100).toFixed(1) + '%',
    performance: (performance * 100).toFixed(1) + '%',
    quality: (quality * 100).toFixed(1) + '%',
    oee: (oee * 100).toFixed(1) + '%'
  };
}

const result = calculateOEE();
console.log(result);
// {
//   availability: '90.6%',
//   performance: '92.0%',
//   quality: '95.0%',
//   oee: '79.2%'
// }
```

**World-Class OEE Benchmarks**:
- OEE > 85%: World-class
- OEE 60-85%: Good
- OEE < 60%: Needs improvement

#### 6.1.2 MTBF & MTTR

**MTBF (Mean Time Between Failures)**:
```
MTBF = Total Operating Time / Number of Failures

Example:
  Operating Time: 720 hours (30 days)
  Failures: 3
  MTBF = 720 / 3 = 240 hours
```

**MTTR (Mean Time To Repair)**:
```
MTTR = Total Downtime / Number of Failures

Example:
  Downtime: 12 hours
  Failures: 3
  MTTR = 12 / 3 = 4 hours
```

**Availability**:
```
Availability = MTBF / (MTBF + MTTR)

Example:
  MTBF: 240 hours
  MTTR: 4 hours
  Availability = 240 / (240 + 4) = 98.4%
```

### 6.2 Dashboard Design

#### 6.2.1 Grafana Dashboard Configuration

```json
{
  "dashboard": {
    "title": "Production Line Monitoring",
    "panels": [
      {
        "id": 1,
        "title": "OEE Trend",
        "type": "graph",
        "datasource": "InfluxDB",
        "targets": [
          {
            "query": "SELECT mean(\"oee\") FROM \"production_metrics\" WHERE $timeFilter GROUP BY time(1h)"
          }
        ]
      },
      {
        "id": 2,
        "title": "Machine Status",
        "type": "stat",
        "datasource": "InfluxDB",
        "targets": [
          {
            "query": "SELECT last(\"status\") FROM \"machines\" GROUP BY \"machine_id\""
          }
        ],
        "options": {
          "colorMode": "background",
          "graphMode": "none",
          "mappings": [
            {"value": "running", "color": "green"},
            {"value": "stopped", "color": "red"},
            {"value": "idle", "color": "yellow"}
          ]
        }
      },
      {
        "id": 3,
        "title": "Temperature Trend",
        "type": "timeseries",
        "datasource": "InfluxDB",
        "targets": [
          {
            "query": "SELECT mean(\"temperature\") FROM \"sensor_data\" WHERE \"sensor_id\" = 'temp-001' AND $timeFilter GROUP BY time(1m)"
          }
        ],
        "fieldConfig": {
          "defaults": {
            "thresholds": {
              "steps": [
                {"value": 0, "color": "green"},
                {"value": 70, "color": "yellow"},
                {"value": 85, "color": "red"}
              ]
            }
          }
        }
      }
    ],
    "refresh": "5s",
    "time": {
      "from": "now-1h",
      "to": "now"
    }
  }
}
```

---

## 7. Alert and Notification Systems

### 7.1 Alert Rules

#### 7.1.1 Threshold-Based Alerts

```javascript
const alertRules = [
  {
    metric: 'temperature',
    operator: '>',
    threshold: 80,
    severity: 'critical',
    duration: 60, // seconds
    cooldown: 300 // 5 minutes
  },
  {
    metric: 'vibration',
    operator: '>',
    threshold: 0.5,
    severity: 'warning',
    duration: 120
  }
];

function evaluateAlert(metric, value, rule) {
  const triggered = eval(`${value} ${rule.operator} ${rule.threshold}`);

  if (triggered) {
    if (!rule.startTime) {
      rule.startTime = Date.now();
    }

    const duration = (Date.now() - rule.startTime) / 1000;

    if (duration >= rule.duration) {
      return {
        triggered: true,
        severity: rule.severity,
        message: `${metric} is ${value}, exceeds threshold of ${rule.threshold}`
      };
    }
  } else {
    rule.startTime = null;
  }

  return { triggered: false };
}
```

#### 7.1.2 Anomaly-Based Alerts

```javascript
function detectAnomaly(value, history) {
  const mean = history.reduce((a, b) => a + b) / history.length;
  const variance = history.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / history.length;
  const stddev = Math.sqrt(variance);

  const zScore = (value - mean) / stddev;

  if (Math.abs(zScore) > 3) {
    return {
      triggered: true,
      severity: 'warning',
      message: `Anomaly detected: value ${value} is ${zScore.toFixed(2)} standard deviations from mean ${mean.toFixed(2)}`
    };
  }

  return { triggered: false };
}
```

### 7.2 Notification Channels

#### 7.2.1 Multi-Channel Delivery

```javascript
class NotificationManager {
  async sendAlert(alert) {
    const channels = this.getChannelsForSeverity(alert.severity);

    await Promise.all([
      channels.includes('sms') && this.sendSMS(alert),
      channels.includes('email') && this.sendEmail(alert),
      channels.includes('slack') && this.sendSlack(alert),
      channels.includes('webhook') && this.sendWebhook(alert)
    ]);
  }

  getChannelsForSeverity(severity) {
    const config = {
      'critical': ['sms', 'email', 'slack', 'webhook'],
      'warning': ['email', 'slack'],
      'info': ['slack']
    };
    return config[severity] || [];
  }

  async sendSMS(alert) {
    await twilioClient.messages.create({
      to: '+1234567890',
      from: '+0987654321',
      body: `[${alert.severity.toUpperCase()}] ${alert.message}`
    });
  }

  async sendEmail(alert) {
    await emailService.send({
      to: 'operations@factory.com',
      subject: `[${alert.severity.toUpperCase()}] Industrial Alert`,
      body: alert.message
    });
  }

  async sendSlack(alert) {
    await slackClient.chat.postMessage({
      channel: '#production-alerts',
      text: `*${alert.severity.toUpperCase()}*: ${alert.message}`
    });
  }
}
```

---

## 8. Device Management

### 8.1 Device Provisioning

#### 8.1.1 Zero-Touch Provisioning

```javascript
// Device registration flow
async function provisionDevice(deviceInfo) {
  // 1. Validate device certificate
  const isValid = await validateCertificate(deviceInfo.certificate);
  if (!isValid) throw new Error('Invalid certificate');

  // 2. Generate device credentials
  const credentials = await generateCredentials({
    deviceId: deviceInfo.serialNumber,
    model: deviceInfo.model
  });

  // 3. Register in device registry
  await deviceRegistry.register({
    deviceId: deviceInfo.serialNumber,
    model: deviceInfo.model,
    location: deviceInfo.location,
    credentials: credentials,
    status: 'active'
  });

  // 4. Send configuration to device
  await sendConfiguration(deviceInfo.serialNumber, {
    mqttBroker: 'mqtt://broker.factory.com',
    topics: [`devices/${deviceInfo.serialNumber}/telemetry`],
    reportInterval: 5000
  });

  return credentials;
}
```

### 8.2 Firmware Updates

#### 8.2.1 OTA (Over-The-Air) Update Flow

```javascript
class FirmwareUpdateManager {
  async updateDevice(deviceId, firmwareVersion) {
    // 1. Check current version
    const device = await deviceRegistry.get(deviceId);
    if (device.firmwareVersion === firmwareVersion) {
      return { status: 'already_updated' };
    }

    // 2. Download firmware image
    const firmwareUrl = `https://cdn.factory.com/firmware/${firmwareVersion}.bin`;
    const firmwareBuffer = await downloadFirmware(firmwareUrl);
    const checksum = calculateSHA256(firmwareBuffer);

    // 3. Notify device of pending update
    await mqttClient.publish(`devices/${deviceId}/ota/request`, JSON.stringify({
      version: firmwareVersion,
      url: firmwareUrl,
      checksum: checksum,
      size: firmwareBuffer.length
    }));

    // 4. Monitor update progress
    return new Promise((resolve) => {
      mqttClient.subscribe(`devices/${deviceId}/ota/progress`);
      mqttClient.on('message', (topic, message) => {
        const progress = JSON.parse(message.toString());

        if (progress.status === 'completed') {
          resolve({ status: 'success', version: firmwareVersion });
        } else if (progress.status === 'failed') {
          resolve({ status: 'failed', error: progress.error });
        }
      });
    });
  }

  // Rollback to previous version
  async rollback(deviceId) {
    const device = await deviceRegistry.get(deviceId);
    const previousVersion = device.previousFirmwareVersion;

    await mqttClient.publish(`devices/${deviceId}/ota/rollback`, JSON.stringify({
      version: previousVersion
    }));
  }
}
```

---

## 9. Industrial Security

### 9.1 Defense in Depth

```
┌─────────────────────────────────────────────────┐
│ Layer 1: Physical Security                      │
│ • Locked server rooms • Access badges           │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│ Layer 2: Network Segmentation                   │
│ • Firewalls • VLANs • DMZ                       │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│ Layer 3: Authentication & Authorization         │
│ • Multi-factor auth • RBAC • Certificates       │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│ Layer 4: Encryption                             │
│ • TLS 1.3 • AES-256 • Secure key management     │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│ Layer 5: Intrusion Detection                    │
│ • IDS/IPS • SIEM • Behavioral analysis          │
└─────────────────────────────────────────────────┘
```

### 9.2 IEC 62443 Compliance

**Security Levels (SL)**:
- **SL 1**: Protection against casual/accidental violation
- **SL 2**: Protection against intentional violation using simple means
- **SL 3**: Protection against intentional violation using sophisticated means
- **SL 4**: Protection against intentional violation using sophisticated means with extended resources

**Foundational Requirements (FR)**:
1. **FR1 - Identification and Authentication Control**
2. **FR2 - Use Control**
3. **FR3 - System Integrity**
4. **FR4 - Data Confidentiality**
5. **FR5 - Restricted Data Flow**
6. **FR6 - Timely Response to Events**
7. **FR7 - Resource Availability**

---

## 10. Data Aggregation

### 10.1 Multi-Source Integration

```javascript
class DataAggregator {
  constructor() {
    this.sources = [];
    this.buffer = [];
  }

  addSource(source) {
    this.sources.push(source);
  }

  async aggregate() {
    const data = await Promise.all(
      this.sources.map(source => source.read())
    );

    return this.normalize(data);
  }

  normalize(data) {
    return data.map(point => ({
      timestamp: new Date(point.timestamp).toISOString(),
      value: parseFloat(point.value),
      unit: this.standardizeUnit(point.unit),
      source: point.source
    }));
  }

  standardizeUnit(unit) {
    const mapping = {
      'C': 'celsius',
      'F': 'fahrenheit',
      'PSI': 'psi',
      'bar': 'bar',
      'Pa': 'pascal'
    };
    return mapping[unit] || unit;
  }
}
```

---

## 11. MES/ERP Integration

### 11.1 Production Order Sync

```javascript
async function syncProductionOrder(orderId) {
  // 1. Fetch from ERP
  const erpOrder = await erpAPI.getOrder(orderId);

  // 2. Create in MES
  const mesOrder = await mesAPI.createOrder({
    orderId: erpOrder.id,
    product: erpOrder.productCode,
    quantity: erpOrder.quantity,
    dueDate: erpOrder.dueDate,
    priority: erpOrder.priority
  });

  // 3. Schedule on line
  const schedule = await mesAPI.schedule({
    orderId: mesOrder.id,
    lineId: 'line-A',
    startTime: calculateStartTime(erpOrder.dueDate, erpOrder.quantity)
  });

  return schedule;
}
```

---

## 12. Digital Twin

### 12.1 Virtual Asset Model

```javascript
class DigitalTwin {
  constructor(assetId, model) {
    this.assetId = assetId;
    this.model = model;
    this.state = {};
    this.sensors = [];
  }

  async sync() {
    const sensorData = await Promise.all(
      this.sensors.map(s => s.read())
    );

    this.updateState(sensorData);
    this.runSimulation();
  }

  updateState(sensorData) {
    sensorData.forEach(data => {
      this.state[data.sensor] = data.value;
    });
  }

  runSimulation() {
    // Physics-based simulation
    const temperature = this.state.temperature;
    const power = this.state.power;

    // Predict thermal expansion
    const expansion = this.model.thermalCoefficient * temperature;

    // Estimate remaining life
    const stress = this.calculateStress(temperature, power);
    const rul = this.estimateRUL(stress);

    this.state.predictedExpansion = expansion;
    this.state.estimatedRUL = rul;
  }
}
```

---

## 13. Performance Optimization

### 13.1 Latency Targets

| Application | Target Latency | Typical Latency |
|-------------|----------------|-----------------|
| Motion control | <1ms | 0.1-0.5ms |
| Safety systems | <10ms | 1-5ms |
| Process control | <100ms | 10-50ms |
| SCADA | <1s | 100-500ms |
| Analytics | <10s | 1-5s |

---

## 14. Implementation Guidelines

### 14.1 Best Practices

1. **Start with Edge**: Process data locally before cloud
2. **Use Time-Series DB**: InfluxDB, TimescaleDB for industrial data
3. **Implement Buffering**: Handle network outages gracefully
4. **Security First**: Always encrypt, authenticate, authorize
5. **Monitor Everything**: Metrics, logs, traces
6. **Plan for Scale**: Design for 10x current capacity
7. **Test Failover**: Regular disaster recovery drills
8. **Document**: Comprehensive system documentation

---

## 15. References

1. OPC Foundation. "OPC Unified Architecture Specification"
2. MQTT.org. "MQTT Version 5.0"
3. Modbus Organization. "Modbus Application Protocol Specification"
4. IEC 62443. "Industrial communication networks – Network and system security"
5. ISA-95. "Enterprise-Control System Integration"
6. InfluxData. "InfluxDB Time-Series Database Documentation"
7. Grafana Labs. "Grafana Dashboard Best Practices"

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
