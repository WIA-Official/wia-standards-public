# WIA-CITY-003: Smart Building Standard
## PHASE 3 - PROTOCOL SPECIFICATION

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Protocol Overview

### 1.1 Supported Protocols

WIA-CITY-003 supports the following industry-standard building automation protocols:

1. **BACnet** (Building Automation and Control Network)
2. **KNX** (Konnex)
3. **Modbus** (RTU and TCP)
4. **LonWorks** (Legacy support)
5. **MQTT** (IoT device integration)
6. **OPC UA** (Industrial integration)

### 1.2 Protocol Selection Guide

| Use Case | Recommended Protocol | Rationale |
|----------|---------------------|-----------|
| HVAC Systems | BACnet/IP | Native support, rich object model |
| Lighting Control | KNX | European standard, robust |
| Energy Meters | Modbus TCP | Widely supported, simple |
| IoT Sensors | MQTT | Lightweight, publish-subscribe |
| Industrial Equipment | OPC UA | Secure, standardized |

---

## 2. BACnet Protocol

### 2.1 BACnet Overview

**Standard:** ASHRAE 135, ISO 16484-5
**Transport:** BACnet/IP (UDP port 47808)
**Version:** BACnet 2020 or later

### 2.2 BACnet Device Profile

```
Device Object Properties:
├── Object_Identifier: Unique device ID
├── Object_Name: Human-readable name
├── System_Status: Operational status
├── Vendor_Name: "WIA-CITY"
├── Model_Name: Device model
├── Firmware_Revision: Version string
├── Application_Software_Version: Software version
├── Protocol_Version: BACnet protocol version
├── Protocol_Conformance_Class: 2 (minimum)
├── BBMD_Broadcast_Distribution_Table: For routing
└── Segmentation_Supported: Yes
```

### 2.3 BACnet Object Types

**Analog Input (AI):**
- Temperature sensors
- Humidity sensors
- Pressure sensors
- Flow meters
- Energy meters

**Analog Output (AO):**
- Valve positions
- Damper positions
- Variable speed drives

**Binary Input (BI):**
- Switch status
- Alarm contacts
- Occupancy sensors
- Door contacts

**Binary Output (BO):**
- Relay outputs
- On/off controls
- Fan enables

**Multi-State Input (MI):**
- HVAC mode feedback
- Alarm priorities

**Multi-State Output (MO):**
- HVAC mode commands
- Fan speed selection

### 2.4 BACnet Services

**Required Services:**
- ReadProperty
- WriteProperty
- SubscribeCOV (Change of Value)
- I-Am / Who-Is (device discovery)
- I-Have / Who-Has (object discovery)

**Optional Services:**
- ReadPropertyMultiple
- WritePropertyMultiple
- ConfirmedEventNotification
- AlarmAcknowledgment
- GetEventInformation

### 2.5 BACnet Message Examples

**Read Temperature:**
```
BACnet-Confirmed-Request: ReadProperty
  Invoke ID: 5
  Object Identifier: analog-input, 1
  Property Identifier: present-value

BACnet-Complex-ACK: ReadProperty
  Invoke ID: 5
  Property Value: 22.5 (Real)
```

**Subscribe to COV:**
```
BACnet-Confirmed-Request: SubscribeCOV
  Subscriber Process ID: 123
  Monitored Object: analog-input, 1
  Issue Confirmed Notifications: TRUE
  Lifetime: 3600 seconds

BACnet-Simple-ACK
```

**COV Notification:**
```
BACnet-Unconfirmed-Request: UnconfirmedCOVNotification
  Subscriber Process ID: 123
  Initiating Device: device, 123456
  Monitored Object: analog-input, 1
  Time Remaining: 3500 seconds
  List of Values:
    present-value: 23.0
    status-flags: {in-alarm, fault, overridden, out-of-service} = {0,0,0,0}
```

### 2.6 BACnet Network Architecture

```
┌─────────────────────────────────────────┐
│   BACnet/IP Network (10.0.0.0/24)       │
├─────────────────────────────────────────┤
│                                         │
│  ┌──────────┐  ┌──────────┐           │
│  │  BBMD    │  │  BBMD    │           │
│  │ (Router) │  │ (Router) │           │
│  └────┬─────┘  └────┬─────┘           │
│       │             │                  │
│  ┌────┴─────────────┴────┐            │
│  │                        │            │
│  │  BACnet Devices:       │            │
│  │  - Controllers         │            │
│  │  - Sensors             │            │
│  │  - Actuators           │            │
│  │  - HMI/SCADA           │            │
│  │                        │            │
│  └────────────────────────┘            │
└─────────────────────────────────────────┘
```

---

## 3. KNX Protocol

### 3.1 KNX Overview

**Standard:** ISO/IEC 14543-3
**Transport:** KNX/IP (UDP port 3671)
**Topology:** Bus, Line, Star

### 3.2 KNX Addressing

**Physical Address Format:** Area.Line.Device (e.g., 1.2.3)
- Area: 0-15
- Line: 0-15
- Device: 0-255

**Group Address Format:** Main/Middle/Sub (e.g., 1/2/3)
- Main Group: 0-31
- Middle Group: 0-7
- Sub Group: 0-255

### 3.3 KNX Datapoint Types (DPT)

| DPT | Description | Example |
|-----|-------------|---------|
| DPT 1 | 1-bit Boolean | Switch On/Off |
| DPT 5 | 8-bit Unsigned | Dimming (0-255) |
| DPT 9 | 2-byte Float | Temperature |
| DPT 14 | 4-byte Float | Energy (kWh) |
| DPT 16 | String | Status text |

### 3.4 KNX Telegram Structure

```
┌──────────────────────────────────────────┐
│ Control Field (1 byte)                    │
├──────────────────────────────────────────┤
│ Source Address (2 bytes)                  │
├──────────────────────────────────────────┤
│ Destination Address (2 bytes)             │
├──────────────────────────────────────────┤
│ Data Length (1 byte)                      │
├──────────────────────────────────────────┤
│ APDU (Application Protocol Data Unit)     │
├──────────────────────────────────────────┤
│ Checksum (1 byte)                         │
└──────────────────────────────────────────┘
```

### 3.5 KNX Communication Examples

**Switch Light On:**
```
Source: 1.1.1 (Wall Switch)
Destination: 2/3/4 (Lighting Group)
Command: GroupValueWrite
Data: 0x01 (ON)
```

**Read Temperature:**
```
Source: 1.1.5 (Thermostat)
Destination: 3/4/5 (Temperature Sensor)
Command: GroupValueRead

Response:
Source: 3/4/5 (Temperature Sensor)
Destination: 1.1.5 (Thermostat)
Command: GroupValueResponse
Data: 22.5°C (DPT 9)
```

### 3.6 KNX Functions in Smart Buildings

**Lighting Control:**
- On/Off switching (1/x/x)
- Dimming (2/x/x)
- Scene control (3/x/x)
- Status feedback (4/x/x)

**HVAC Control:**
- Temperature setpoints (5/x/x)
- Fan speed control (6/x/x)
- Mode selection (7/x/x)

**Shading Control:**
- Blind up/down (8/x/x)
- Slat angle (9/x/x)
- Position feedback (10/x/x)

---

## 4. Modbus Protocol

### 4.1 Modbus Overview

**Variants:**
- Modbus RTU (Serial, RS-485)
- Modbus TCP/IP (Ethernet, port 502)

### 4.2 Modbus Function Codes

| Code | Function | Description |
|------|----------|-------------|
| 01 | Read Coils | Read 1-bit outputs |
| 02 | Read Discrete Inputs | Read 1-bit inputs |
| 03 | Read Holding Registers | Read 16-bit registers |
| 04 | Read Input Registers | Read 16-bit input registers |
| 05 | Write Single Coil | Write 1-bit output |
| 06 | Write Single Register | Write 16-bit register |
| 15 | Write Multiple Coils | Write multiple 1-bit outputs |
| 16 | Write Multiple Registers | Write multiple 16-bit registers |

### 4.3 Modbus TCP Frame Structure

```
┌──────────────────────────────────────────┐
│ Transaction ID (2 bytes)                  │
├──────────────────────────────────────────┤
│ Protocol ID (2 bytes) = 0x0000            │
├──────────────────────────────────────────┤
│ Length (2 bytes)                          │
├──────────────────────────────────────────┤
│ Unit ID (1 byte)                          │
├──────────────────────────────────────────┤
│ Function Code (1 byte)                    │
├──────────────────────────────────────────┤
│ Data (N bytes)                            │
└──────────────────────────────────────────┘
```

### 4.4 Modbus RTU Frame Structure

```
┌──────────────────────────────────────────┐
│ Slave Address (1 byte)                    │
├──────────────────────────────────────────┤
│ Function Code (1 byte)                    │
├──────────────────────────────────────────┤
│ Data (N bytes)                            │
├──────────────────────────────────────────┤
│ CRC (2 bytes)                             │
└──────────────────────────────────────────┘
```

### 4.5 Modbus Examples

**Read Energy Meter (Function 03):**
```
Request:
  Slave Address: 01
  Function Code: 03 (Read Holding Registers)
  Starting Address: 0x0000
  Quantity: 0x0002 (2 registers)
  CRC: 0xXXXX

Response:
  Slave Address: 01
  Function Code: 03
  Byte Count: 04
  Register 0x0000: 0x1234 (High word of energy)
  Register 0x0001: 0x5678 (Low word of energy)
  CRC: 0xXXXX

Energy Value: 0x12345678 = 305,419,896 Wh
```

**Write Setpoint (Function 06):**
```
Request:
  Slave Address: 01
  Function Code: 06 (Write Single Register)
  Register Address: 0x0010 (Temperature setpoint)
  Value: 0x00E6 (230 = 23.0°C scaled by 10)
  CRC: 0xXXXX

Response:
  (Echo of request confirms success)
```

### 4.6 Modbus Register Mapping

**Typical Energy Meter Mapping:**
```
Address | Register | Type | Unit | Description
--------|----------|------|------|-------------
0x0000  | 40001    | UINT32 | Wh   | Total Energy
0x0002  | 40003    | UINT32 | W    | Active Power
0x0004  | 40005    | UINT16 | V    | Voltage L1
0x0005  | 40006    | UINT16 | V    | Voltage L2
0x0006  | 40007    | UINT16 | V    | Voltage L3
0x0007  | 40008    | UINT16 | A*10 | Current L1
0x0008  | 40009    | UINT16 | A*10 | Current L2
0x0009  | 40010    | UINT16 | A*10 | Current L3
0x000A  | 40011    | UINT16 | 1/100| Power Factor
```

---

## 5. MQTT Protocol

### 5.1 MQTT Overview

**Standard:** OASIS MQTT Version 5.0
**Transport:** TCP (port 1883), TLS (port 8883)
**Pattern:** Publish-Subscribe

### 5.2 MQTT Topic Naming Convention

```
{organization}/buildings/{building-id}/{floor}/{zone}/{device-type}/{device-id}/{metric}

Examples:
wia/buildings/building-001/floor-03/zone-a/temperature/sensor-001/value
wia/buildings/building-001/floor-03/zone-a/temperature/sensor-001/status
wia/buildings/building-001/floor-03/zone-a/hvac/ahu-001/command
```

### 5.3 MQTT QoS Levels

| QoS | Description | Use Case |
|-----|-------------|----------|
| 0 | At most once | Non-critical sensor data |
| 1 | At least once | Standard telemetry |
| 2 | Exactly once | Critical commands, alarms |

### 5.4 MQTT Message Formats

**Sensor Reading:**
```json
Topic: wia/buildings/bld-001/floor-03/zone-a/temperature/sensor-001/value
Payload: {
  "timestamp": "2025-12-25T10:30:00Z",
  "value": 22.5,
  "unit": "celsius",
  "quality": "good"
}
QoS: 1
Retain: true
```

**Command:**
```json
Topic: wia/buildings/bld-001/floor-03/zone-a/hvac/ahu-001/command
Payload: {
  "command": "set-temperature",
  "value": 23.0,
  "requestId": "cmd-12345",
  "timestamp": "2025-12-25T10:30:00Z"
}
QoS: 2
Retain: false
```

**Alarm:**
```json
Topic: wia/buildings/bld-001/alarms/critical
Payload: {
  "alarmId": "alarm-67890",
  "severity": "critical",
  "source": "chiller-01",
  "message": "Low refrigerant pressure",
  "timestamp": "2025-12-25T10:35:00Z"
}
QoS: 2
Retain: false
```

### 5.5 MQTT Last Will and Testament (LWT)

```json
Topic: wia/buildings/bld-001/devices/sensor-001/status
Payload: {
  "status": "offline",
  "timestamp": "2025-12-25T10:30:00Z"
}
QoS: 1
Retain: true
```

---

## 6. OPC UA Protocol

### 6.1 OPC UA Overview

**Standard:** IEC 62541
**Transport:** TCP (port 4840), HTTPS (port 443)
**Security:** X.509 certificates, encryption

### 6.2 OPC UA Information Model

```
Objects/
├── Server/
│   ├── ServerCapabilities
│   ├── ServerDiagnostics
│   └── ServerStatus
├── Buildings/
│   ├── Building-001/
│   │   ├── Floors/
│   │   │   ├── Floor-03/
│   │   │   │   ├── Zones/
│   │   │   │   │   ├── Zone-A/
│   │   │   │   │   │   ├── Sensors/
│   │   │   │   │   │   │   ├── Temperature-001
│   │   │   │   │   │   │   └── Humidity-001
│   │   │   │   │   │   └── HVAC/
│   │   │   │   │   │       └── AHU-001
```

### 6.3 OPC UA Node Classes

- **Object**: Physical or logical objects
- **Variable**: Data values
- **Method**: Callable functions
- **ObjectType**: Object templates
- **VariableType**: Variable templates
- **DataType**: Data type definitions

### 6.4 OPC UA Services

**Read/Write:**
- Read (single or multiple nodes)
- Write (single or multiple nodes)

**Subscription:**
- CreateSubscription
- CreateMonitoredItems
- Publish (server pushes data)

**Method Call:**
- Call (invoke server-side methods)

### 6.5 OPC UA Example

**Read Temperature:**
```
Request:
  NodeId: ns=2;s=Building-001.Floor-03.Zone-A.Temperature-001
  Attribute: Value

Response:
  Value: 22.5
  DataType: Double
  StatusCode: Good (0x00000000)
  SourceTimestamp: 2025-12-25T10:30:00Z
  ServerTimestamp: 2025-12-25T10:30:00.123Z
```

---

## 7. Protocol Gateway Architecture

### 7.1 Multi-Protocol Gateway

```
┌────────────────────────────────────────────┐
│           WIA-CITY Integration Layer       │
│              (REST API, GraphQL)           │
└────────────────┬───────────────────────────┘
                 │
┌────────────────┴───────────────────────────┐
│          Protocol Gateway Layer            │
├────────────────────────────────────────────┤
│                                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐│
│  │ BACnet   │  │   KNX    │  │ Modbus   ││
│  │ Gateway  │  │ Gateway  │  │ Gateway  ││
│  └────┬─────┘  └────┬─────┘  └────┬─────┘│
│       │             │              │      │
└───────┼─────────────┼──────────────┼──────┘
        │             │              │
   ┌────┴─────┐  ┌───┴────┐    ┌───┴────┐
   │ BACnet   │  │  KNX   │    │ Modbus │
   │ Devices  │  │ Devices│    │ Devices│
   └──────────┘  └────────┘    └────────┘
```

### 7.2 Data Normalization

All protocols are normalized to a common data model:

```json
{
  "sourceProtocol": "BACnet|KNX|Modbus|MQTT|OPC-UA",
  "deviceId": "string",
  "pointId": "string",
  "value": "number|boolean|string",
  "unit": "string",
  "quality": "good|uncertain|bad",
  "timestamp": "ISO 8601",
  "metadata": {
    "protocol-specific": "object"
  }
}
```

---

## 8. Security Requirements

### 8.1 Network Security

- **Segmentation**: Separate OT (Operational Technology) and IT networks
- **Firewall**: Protocol-specific rules
- **VPN**: Encrypted remote access
- **VLAN**: Logical network separation

### 8.2 Protocol-Specific Security

**BACnet:**
- BACnet Secure Connect (BACnet/SC)
- Device authentication
- Encrypted communication

**KNX:**
- KNX Secure (authentication and encryption)
- Device certificates

**Modbus:**
- Modbus Security (TLS encryption)
- Role-based access control

**MQTT:**
- TLS/SSL encryption
- Username/password authentication
- Client certificates

**OPC UA:**
- X.509 certificates
- User authentication
- Message signing and encryption
- Security modes: Sign, SignAndEncrypt

### 8.3 Authentication & Authorization

```
User → Authentication → Authorization → Protocol Access
       (Username/Pass,     (RBAC,         (BACnet, KNX,
        Certificate,        ACL)           Modbus, MQTT,
        API Key)                           OPC UA)
```

---

## 9. Performance Requirements

### 9.1 Latency Requirements

| Operation | Max Latency | Target Latency |
|-----------|-------------|----------------|
| Sensor Read | 1 second | 100 ms |
| Actuator Write | 2 seconds | 500 ms |
| Alarm Notification | 1 second | 100 ms |
| COV Update | 5 seconds | 1 second |

### 9.2 Throughput Requirements

- **Minimum**: 1,000 points/second
- **Recommended**: 10,000 points/second
- **Enterprise**: 100,000+ points/second

### 9.3 Reliability

- **Uptime**: 99.9% minimum
- **Message Delivery**: 99.99% for critical data
- **Data Retention**: No data loss for QoS 2 messages

---

## 10. Testing & Validation

### 10.1 Conformance Testing

**BACnet:**
- BACnet Testing Laboratories (BTL) certification
- Protocol Implementation Conformance Statement (PICS)

**KNX:**
- KNX certification (konnex.org)
- ETS (Engineering Tool Software) compatibility

**Modbus:**
- Modbus Organization compliance testing

### 10.2 Interoperability Testing

- Multi-vendor device integration
- Protocol gateway functionality
- Failover and redundancy
- Performance under load

### 10.3 Security Testing

- Penetration testing
- Vulnerability scanning
- Encryption validation
- Authentication testing

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
