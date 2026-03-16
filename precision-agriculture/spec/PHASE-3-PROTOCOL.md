# WIA Precision Agriculture Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [ISOBUS Protocol](#isobus-protocol)
3. [GPS/GNSS Communication](#gps-gnss-communication)
4. [CAN Bus Protocol](#can-bus-protocol)
5. [NMEA 0183 Protocol](#nmea-0183-protocol)
6. [Task Controller Protocol](#task-controller-protocol)
7. [Telemetry Protocol](#telemetry-protocol)
8. [Data Exchange Formats](#data-exchange-formats)
9. [Security & Encryption](#security-encryption)
10. [Network Communication](#network-communication)
11. [Protocol Examples](#protocol-examples)

---

## Overview

### 1.1 Purpose

The WIA Precision Agriculture Protocol Standard defines communication protocols for agricultural equipment, GPS guidance systems, ISOBUS task controllers, and farm management platforms, ensuring interoperability and real-time data exchange.

**Supported Protocols**:
- **ISOBUS (ISO 11783)**: Agricultural equipment communication
- **GPS/GNSS**: RTK, DGPS, SBAS positioning
- **CAN Bus**: Controller Area Network for vehicle systems
- **NMEA 0183/2000**: Marine GPS protocol adapted for agriculture
- **MQTT**: IoT sensor communication
- **WebSocket**: Real-time telemetry streaming
- **gRPC**: High-performance data exchange

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (Farm Software) │
├─────────────────────────────────────┤
│   API Layer (REST, gRPC, WebSocket) │
├─────────────────────────────────────┤
│   Protocol Layer (ISOBUS, NMEA)     │
├─────────────────────────────────────┤
│   Transport Layer (CAN, TCP/IP)     │
├─────────────────────────────────────┤
│   Physical Layer (RS-232, Ethernet) │
└─────────────────────────────────────┘
```

---

## ISOBUS Protocol

### 2.1 ISOBUS Overview

**ISOBUS (ISO 11783)** is the international standard for agricultural equipment communication, enabling tractors, implements, and task controllers to exchange data.

**Key Components**:
- **Task Controller (TC)**: Central unit managing field operations
- **Working Set Master (WSM)**: Tractor or implement master controller
- **Data Logger (DL)**: Records operational data
- **Virtual Terminal (VT)**: Display and user interface

### 2.2 ISOBUS Task Data (ISO-XML)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ISO11783_TaskData VersionMajor="4" VersionMinor="3" DataTransferOrigin="1">
  <Task TaskId="TSK1" TaskDesignator="Fertilizer Application" TaskStatus="1">
    <PartField PartfieldId="PFD1" PartfieldDesignator="North Field" PartfieldArea="505000">
      <LineString>
        <Point PointType="1" PointNorth="37.566" PointEast="126.977"/>
        <Point PointType="1" PointNorth="37.568" PointEast="126.980"/>
      </LineString>
    </PartField>
    <Grid GridId="GRD1" GridDesignator="Fertilizer Grid" GridCellNorthSize="10.0" GridCellEastSize="10.0">
      <GridCell Row="0" Column="0" Value="180"/>
      <GridCell Row="0" Column="1" Value="175"/>
      <GridCell Row="0" Column="2" Value="150"/>
    </Grid>
    <TreatmentZone TreatmentZoneId="TZN1" TreatmentZoneDesignator="High Zone">
      <ProcessDataVariable ProcessDataDDI="0x0001" ProcessDataValue="180"/>
    </TreatmentZone>
  </Task>
</ISO11783_TaskData>
```

### 2.3 ISOBUS CAN Messages

**PGN (Parameter Group Number) for VRT Application**:

```
PGN: 61440 (0xF000) - Work State
Data: [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Byte 0: Work State (0x01 = Active)
Byte 1: Section State (0x02 = Section 1 ON)
```

**PGN: 61441 (0xF001) - Rate Control**:

```
PGN: 61441
Data: [0xB4, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]
Byte 0-1: Application Rate = 180 (0x00B4) in 0.1 kg/ha
Byte 4: Section Control
```

### 2.4 ISOBUS Device Description

```xml
<DeviceElement DeviceElementId="DET1" DeviceElementType="1" DeviceElementDesignator="Fertilizer Spreader">
  <DeviceProcessData DeviceProcessDataDDI="0x0001" DeviceProcessDataDesignator="Application Rate">
    <DeviceProperty DevicePropertyId="DPT1" DevicePropertyValue="0" DevicePropertyDDI="0x0001"/>
  </DeviceProcessData>
</DeviceElement>
```

---

## GPS/GNSS Communication

### 3.1 RTK (Real-Time Kinematic) Protocol

**RTK Correction Message (RTCM 3.3)**:

```
Message 1005: Stationary RTK Reference Station
Message 1077: GPS MSM7 (Multi-Signal Message)
Message 1087: GLONASS MSM7
Message 1097: Galileo MSM7
Message 1127: BeiDou MSM7
```

**RTCM Frame Structure**:

```
Preamble: 0xD3
Reserved: 6 bits (0)
Message Length: 10 bits
Message Type: 12 bits
Payload: Variable length
CRC-24Q: 24 bits
```

### 3.2 NTRIP (Networked Transport of RTCM via Internet Protocol)

**NTRIP Request**:

```http
GET /RTCM3_BASE HTTP/1.1
Host: rtk.precision-ag.com
User-Agent: NTRIP WIA-Client/1.0
Authorization: Basic dXNlcjpwYXNz
```

**NTRIP Response**:

```http
HTTP/1.1 200 OK
Content-Type: application/octet-stream

[RTCM Binary Data Stream...]
```

### 3.3 GPS Position Message

```json
{
  "message_type": "GPS_POSITION",
  "timestamp": "2025-04-15T10:30:45.123Z",
  "position": {
    "lat": 37.566535,
    "lng": 126.977969,
    "altitude_m": 125.5,
    "accuracy_cm": 2.0
  },
  "fix_type": "RTK_FIXED",
  "satellites": {
    "gps": 12,
    "glonass": 8,
    "galileo": 6,
    "total": 26
  },
  "dop": {
    "hdop": 0.8,
    "vdop": 1.2,
    "pdop": 1.4
  },
  "speed_km_h": 8.5,
  "heading_degrees": 45.5
}
```

---

## CAN Bus Protocol

### 4.1 CAN Frame Structure

```
┌────────────┬───────────┬────────┬─────────┬────────┬────────┬─────┐
│ Identifier │ RTR │ IDE │ DLC    │ Data   │ CRC    │ ACK │
│ 11/29 bits │ 1   │ 1   │ 4 bits │ 0-8 B  │ 15 bits│ 2   │
└────────────┴───────────┴────────┴─────────┴────────┴────────┴─────┘
```

**Example: VRT Rate Control Message**:

```
CAN ID: 0x18EF0001 (Extended Frame)
DLC: 8
Data: [0xB4, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF]

Bytes 0-1: Application Rate (180 kg/ha = 0x00B4)
Byte 4: Section Control (0x01 = Section 1 Active)
Bytes 5-7: Reserved (0xFF)
```

### 4.2 J1939 Protocol (Agricultural Adaptation)

**PGN 65265 (0xFEF1) - Cruise Control/Vehicle Speed**:

```c
struct PGN_FEF1 {
  uint16_t wheel_based_speed;  // 0.00390625 km/h per bit
  uint16_t cruise_control_speed;
  uint8_t clutch_switch;
  uint8_t brake_switch;
  uint8_t accelerator_pedal;
  uint8_t road_speed_limit;
};
```

---

## NMEA 0183 Protocol

### 5.1 NMEA Sentence Structure

```
$GPGGA,103045.123,3733.9921,N,12658.6781,E,4,24,0.8,125.5,M,0.0,M,1.0,0000*4A

$GPGGA - Global Positioning System Fix Data
103045.123 - UTC Time (10:30:45.123)
3733.9921,N - Latitude (37°33.9921' N)
12658.6781,E - Longitude (126°58.6781' E)
4 - Fix Quality (4 = RTK Fixed)
24 - Number of Satellites
0.8 - HDOP
125.5,M - Altitude (meters)
0.0,M - Geoidal Separation
1.0 - Age of DGPS Data
0000 - DGPS Station ID
*4A - Checksum
```

### 5.2 Common NMEA Sentences

| Sentence | Description |
|----------|-------------|
| **$GPGGA** | GPS Fix Data (position, time, quality) |
| **$GPGSA** | GPS DOP and Active Satellites |
| **$GPGSV** | GPS Satellites in View |
| **$GPRMC** | Recommended Minimum Data (position, speed, course) |
| **$GPVTG** | Track Made Good and Ground Speed |
| **$GPZDA** | UTC Date and Time |

### 5.3 NMEA VTG (Speed and Course)

```
$GPVTG,45.5,T,43.0,M,8.5,N,15.7,K,D*2E

45.5,T - True Track (45.5 degrees)
43.0,M - Magnetic Track
8.5,N - Ground Speed (8.5 knots)
15.7,K - Ground Speed (15.7 km/h)
D - Mode (D = Differential)
```

---

## Task Controller Protocol

### 6.1 Task Controller Messages

**TC Client → Task Controller: Request Current Position**

```json
{
  "message_type": "TC_REQUEST_POSITION",
  "message_id": "MSG-001",
  "timestamp": "2025-04-15T10:30:45Z"
}
```

**Task Controller → TC Client: Position Response**

```json
{
  "message_type": "TC_POSITION_RESPONSE",
  "message_id": "MSG-001",
  "timestamp": "2025-04-15T10:30:45Z",
  "position": {
    "lat": 37.566535,
    "lng": 126.977969,
    "accuracy_cm": 2.0
  },
  "field_id": "FLD-2025-001",
  "zone_id": "Z1",
  "current_rate_kg_ha": 180.0
}
```

### 6.2 Section Control Message

```json
{
  "message_type": "TC_SECTION_CONTROL",
  "timestamp": "2025-04-15T10:30:50Z",
  "sections": [
    { "section_id": 1, "state": "ON", "rate_kg_ha": 180.0 },
    { "section_id": 2, "state": "ON", "rate_kg_ha": 180.0 },
    { "section_id": 3, "state": "OFF", "rate_kg_ha": 0.0 }
  ]
}
```

---

## Telemetry Protocol

### 7.1 Real-Time Telemetry (WebSocket)

**Connection Handshake**:

```
GET /v1/telemetry/stream HTTP/1.1
Host: api.precision-ag.wiastandards.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
```

**Subscribe to Telemetry**:

```json
{
  "action": "subscribe",
  "vehicle_id": "TRACTOR-001",
  "data_types": ["gps", "speed", "fuel", "application_rate"]
}
```

**Telemetry Stream**:

```json
{
  "timestamp": "2025-04-15T10:30:45.123Z",
  "vehicle_id": "TRACTOR-001",
  "gps": {
    "lat": 37.566535,
    "lng": 126.977969,
    "accuracy_cm": 2.0,
    "fix_type": "RTK_FIXED"
  },
  "speed_km_h": 8.5,
  "heading_degrees": 45.5,
  "fuel_level_percent": 75.5,
  "application_rate_kg_ha": 180.0,
  "implement_status": "ACTIVE",
  "section_states": [1, 1, 1, 1, 0]
}
```

### 7.2 MQTT Telemetry

**Topic Structure**:

```
wia/precision-ag/farm/{FARM_ID}/field/{FIELD_ID}/vehicle/{VEHICLE_ID}/telemetry
```

**Payload**:

```json
{
  "timestamp": "2025-04-15T10:30:45Z",
  "vehicle_id": "TRACTOR-001",
  "gps": { "lat": 37.566535, "lng": 126.977969 },
  "speed_km_h": 8.5,
  "application_rate_kg_ha": 180.0
}
```

**QoS Levels**:
- QoS 0: Best effort (GPS updates)
- QoS 1: At least once (Application rate changes)
- QoS 2: Exactly once (Critical events)

---

## Data Exchange Formats

### 8.1 Shapefile Format

**Prescription Shapefile Structure**:

```
prescription.shp - Geometry (polygons)
prescription.shx - Shape index
prescription.dbf - Attribute table
prescription.prj - Projection (WGS84)
```

**Attribute Table (.dbf)**:

| ZONE_ID | RATE_KG_HA | PRODUCT    |
|---------|------------|------------|
| Z1      | 180.0      | Urea 46-0-0|
| Z2      | 150.0      | Urea 46-0-0|
| Z3      | 120.0      | Urea 46-0-0|

### 8.2 GeoJSON Format

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [[[126.977, 37.566], [126.978, 37.566], [126.978, 37.567], [126.977, 37.567], [126.977, 37.566]]]
      },
      "properties": {
        "zone_id": "Z1",
        "rate_kg_ha": 180.0,
        "product": "Urea 46-0-0"
      }
    }
  ]
}
```

### 8.3 ADAPT Format

**AgGateway ADAPT (Agricultural Data Application Programming Toolkit)**:

```json
{
  "ApplicationDataModel": {
    "Catalog": {
      "Growers": [ /* Grower data */ ],
      "Farms": [ /* Farm data */ ],
      "Fields": [ /* Field data */ ],
      "Products": [ /* Product data */ ]
    },
    "Documents": {
      "WorkItems": [ /* Work operations */ ],
      "LoggedData": [ /* Operational data */ ]
    }
  }
}
```

---

## Security & Encryption

### 9.1 TLS/SSL Encryption

**Minimum TLS Version**: TLS 1.3

**Cipher Suites**:
- `TLS_AES_256_GCM_SHA384`
- `TLS_CHACHA20_POLY1305_SHA256`

### 9.2 Message Authentication

**HMAC-SHA256 Signature**:

```
Message: {"vehicle_id": "TRACTOR-001", "rate_kg_ha": 180.0}
Secret: "wia-precision-ag-secret-key"
HMAC: HMAC-SHA256(Message, Secret)
```

**Authenticated Message**:

```json
{
  "message": {"vehicle_id": "TRACTOR-001", "rate_kg_ha": 180.0},
  "signature": "a8f5e2c1d3b4f6e7a9c8b5d4e3f2a1b0c9d8e7f6a5b4c3d2e1f0a9b8c7d6e5f4",
  "timestamp": "2025-04-15T10:30:45Z"
}
```

---

## Network Communication

### 10.1 Network Topology

```
┌─────────────┐
│   Cloud     │
│  Platform   │
└─────┬───────┘
      │ 4G/5G/WiFi
┌─────▼───────┐     CAN Bus    ┌──────────────┐
│   Tractor   ├───────────────►│  Implement   │
│  Terminal   │                │  Controller  │
└─────────────┘                └──────────────┘
      │
      │ RS-232/USB
      │
┌─────▼───────┐
│   GPS RTK   │
│   Receiver  │
└─────────────┘
```

### 10.2 Communication Modes

| Mode | Protocol | Use Case |
|------|----------|----------|
| **Local** | CAN Bus, RS-232 | Tractor-implement communication |
| **Field** | WiFi, Bluetooth | Tablet-tractor communication |
| **Cloud** | 4G/5G, Satellite | FMS-cloud synchronization |
| **Broadcast** | LoRaWAN | Soil sensor networks |

---

## Protocol Examples

### 11.1 Complete VRT Application Flow

**Step 1: Upload Prescription (ISOXML)**

```xml
<Task TaskId="TSK1" TaskDesignator="Fertilizer VRT">
  <TreatmentZone TreatmentZoneId="TZN1">
    <ProcessDataVariable ProcessDataDDI="0x0001" ProcessDataValue="180"/>
  </TreatmentZone>
</Task>
```

**Step 2: Task Controller Loads Prescription**

```
TC → Implement: Load Task TSK1
Implement → TC: ACK, Task Loaded
```

**Step 3: GPS Updates Position**

```
$GPGGA,103045,3733.9921,N,12658.6781,E,4,24,0.8,125.5,M,0.0,M,1.0,0000*4A
```

**Step 4: Task Controller Adjusts Rate**

```
TC → Implement: Set Rate 180 kg/ha (Zone Z1)
Implement → TC: ACK, Rate Applied
```

**Step 5: Log Application Data**

```json
{
  "timestamp": "2025-04-15T10:30:45Z",
  "gps": { "lat": 37.566535, "lng": 126.977969 },
  "zone_id": "Z1",
  "rate_kg_ha": 180.0,
  "application_amount_kg": 0.045
}
```

---

**© 2025 WIA Standards - MIT License**
**弘益人間 (Hongik Ingan) - Benefit All Humanity**
