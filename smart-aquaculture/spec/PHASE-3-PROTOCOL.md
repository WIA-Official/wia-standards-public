# WIA-AGRI-012: Smart Aquaculture Standard
## Phase 3 - Communication Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols optimized for underwater and marine aquaculture environments, including acoustic telemetry, optical communication, and surface wireless networks.

---

## 2. Underwater Acoustic Communication

### 2.1 Protocol Overview

Acoustic modems enable wireless data transmission through water, essential for submerged sensors in marine cages and offshore farms.

#### Technical Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency Range | 18-36 kHz | Lower freq = longer range, higher = faster data |
| Data Rate | 10-40 kbps | Depends on distance and conditions |
| Range | 1-5 km | Varies with salinity, temperature, turbidity |
| Modulation | FSK, PSK, OFDM | Frequency/Phase Shift Keying, OFDM for multipath |
| Power Consumption | 1-5W TX, <0.1W RX | Typical values |
| Latency | 100-500 ms | Speed of sound in water ≈ 1500 m/s |

### 2.2 Frame Structure

```
┌─────────────────────────────────────────────────────────┐
│ PREAMBLE │ HEADER │ PAYLOAD │ CRC │ ECC │
├──────────┼────────┼─────────┼─────┼─────┤
│  16 bits │ 32 bits│ N bytes │ 16  │ 32  │
└─────────────────────────────────────────────────────────┘
```

#### Header Format

```json
{
  "version": 1,
  "sourceId": "sensor-12345",
  "destinationId": "gateway-001",
  "messageType": "WATER_QUALITY_DATA",
  "sequenceNumber": 4567,
  "payloadLength": 128,
  "priority": 2,
  "timestamp": 1642291200
}
```

#### Payload Example

```json
{
  "sensorId": "aq-sensor-12345",
  "timestamp": "2025-01-15T14:30:00Z",
  "depth": 5.2,
  "temperature": 18.5,
  "dissolvedOxygen": 7.8,
  "salinity": 32.5,
  "batteryLevel": 87
}
```

### 2.3 Error Correction

- **Reed-Solomon Coding**: RS(255, 223) for forward error correction
- **ARQ (Automatic Repeat Request)**: For critical data
- **Interleaving**: To combat burst errors from multipath fading

### 2.4 Medium Access Control (MAC)

#### TDMA (Time Division Multiple Access)

```
Time Slot Allocation:
┌────────┬────────┬────────┬────────┬────────┐
│ Slot 0 │ Slot 1 │ Slot 2 │ Slot 3 │ Slot 4 │
│ Gateway│ Sensor1│ Sensor2│ Sensor3│ Sensor4│
└────────┴────────┴────────┴────────┴────────┘
   500ms    500ms    500ms    500ms    500ms

Total Frame: 2.5 seconds
```

#### CSMA/CA (Carrier Sense Multiple Access with Collision Avoidance)

For ad-hoc sensor networks where TDMA synchronization is difficult.

### 2.5 Acoustic Modem Commands

#### Initialize Sensor

```
Command: INIT
Payload: {
  "sensorId": "aq-sensor-12345",
  "samplingRate": 60,
  "reportingInterval": 300,
  "parameters": ["temperature", "do", "salinity"]
}
```

#### Query Sensor Status

```
Command: STATUS
Response: {
  "sensorId": "aq-sensor-12345",
  "uptime": 345600,
  "batteryLevel": 87,
  "lastSample": "2025-01-15T14:30:00Z",
  "memoryUsed": 42
}
```

#### Remote Configuration

```
Command: CONFIG
Payload: {
  "samplingRate": 30,
  "alarmThresholds": {
    "temperature": {"min": 15, "max": 25},
    "dissolvedOxygen": {"min": 6.0}
  }
}
```

---

## 3. Optical Underwater Communication

### 3.1 Overview

High-speed, short-range communication using LED/laser light through water.

#### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wavelength | 450-550 nm (blue-green) | Best penetration in seawater |
| Data Rate | 1-100 Mbps | Much faster than acoustic |
| Range | 10-100 m | Limited by turbidity |
| Power | <1W | Very low power |
| Latency | 1-10 ms | Near-instantaneous |

### 3.2 Use Cases

- **High-bandwidth data**: Video streaming from underwater cameras
- **ROV communication**: Real-time control of underwater robots
- **Short-range sensor networks**: Dense sensor arrays in tanks

### 3.3 Protocol

Based on IEEE 802.15.7 (Visible Light Communication) with aquaculture-specific extensions.

```
Physical Layer: On-Off Keying (OOK), Pulse Position Modulation (PPM)
MAC Layer: CSMA/CA
Link Layer: Point-to-point or broadcast
```

---

## 4. Surface Wireless Networks

### 4.1 LoRaWAN for Aquaculture

Long-range, low-power wireless for surface buoys, floating sensors, and edge gateways.

#### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 915 MHz (US), 868 MHz (EU), 920 MHz (KR) | ISM bands |
| Data Rate | 0.3-50 kbps | Adaptive data rate |
| Range | 2-15 km | Line-of-sight over water |
| Power | <50mA TX, <10mA RX | Battery life: 2-10 years |
| Latency | 50-200 ms | Class A devices |

#### Network Topology

```
┌──────────────────────────────────────────────────────┐
│                 LoRaWAN Network                      │
│                                                      │
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐            │
│  │ Buoy │  │ Buoy │  │ Buoy │  │ Buoy │            │
│  │  1   │  │  2   │  │  3   │  │  4   │            │
│  └───┬──┘  └───┬──┘  └───┬──┘  └───┬──┘            │
│      │         │         │         │                │
│      └─────────┴─────────┴─────────┘                │
│                  │                                   │
│           ┌──────▼──────┐                            │
│           │   Gateway   │                            │
│           └──────┬──────┘                            │
│                  │                                   │
│           ┌──────▼──────┐                            │
│           │ Network     │                            │
│           │ Server      │                            │
│           └──────┬──────┘                            │
│                  │                                   │
│           ┌──────▼──────┐                            │
│           │ Application │                            │
│           │ Server      │                            │
│           └─────────────┘                            │
└──────────────────────────────────────────────────────┘
```

#### LoRaWAN Message Format

```json
{
  "devEUI": "70B3D57ED0012345",
  "appEUI": "0000000000000001",
  "fPort": 1,
  "fCnt": 1234,
  "data": "base64_encoded_payload",
  "rssi": -85,
  "snr": 7.5,
  "gatewayId": "gateway-001"
}
```

#### Decoded Payload

```json
{
  "temperature": 18.5,
  "dissolvedOxygen": 7.8,
  "salinity": 32.5,
  "batteryVoltage": 3.7,
  "gps": {
    "latitude": 34.8547,
    "longitude": 128.4333
  }
}
```

### 4.2 Cellular (4G/5G/NB-IoT)

For coastal farms with cellular coverage.

#### Comparison

| Technology | Data Rate | Power | Range | Cost |
|------------|-----------|-------|-------|------|
| 4G LTE | 1-100 Mbps | High | <10 km | Medium |
| 5G | 100 Mbps - 1 Gbps | High | <5 km | High |
| NB-IoT | 20-100 kbps | Low | <15 km | Low |
| LTE-M | 200 kbps - 1 Mbps | Medium | <10 km | Low |

#### Use Cases

- **4G/5G**: Video surveillance, remote monitoring dashboards
- **NB-IoT**: Low-power sensors, water quality monitoring
- **LTE-M**: Mobile assets (boats, feed delivery trucks)

---

## 5. Wired Communication

### 5.1 RS-485 / Modbus RTU

For tank-based aquaculture with wired sensor networks.

#### Specifications

- **Bus Topology**: Multi-drop, up to 32 devices per segment
- **Distance**: Up to 1200 meters
- **Data Rate**: 9600-115200 baud
- **Protocol**: Modbus RTU, Modbus ASCII

#### Modbus RTU Frame

```
┌──────────────────────────────────────────────────┐
│ Address │ Function │ Data │ CRC │
│ 1 byte  │ 1 byte   │ N    │ 2   │
└──────────────────────────────────────────────────┘
```

#### Example: Read Temperature Sensor

```
Request:
  Address: 0x01 (Sensor 1)
  Function: 0x03 (Read Holding Registers)
  Start Register: 0x0000 (Temperature)
  Register Count: 0x0002 (2 registers = 32-bit float)
  CRC: 0xC40B

Response:
  Address: 0x01
  Function: 0x03
  Byte Count: 0x04
  Temperature: 0x41940000 (18.5 °C as IEEE 754 float)
  CRC: 0x9D34
```

### 5.2 Ethernet / Modbus TCP

For large-scale RAS (Recirculating Aquaculture Systems).

- **Speed**: 10/100/1000 Mbps
- **Protocol**: Modbus TCP/IP, MQTT over TCP, HTTP/HTTPS
- **Topology**: Star, with managed switches
- **PoE (Power over Ethernet)**: For powering sensors

---

## 6. Hybrid Network Architecture

### 6.1 Multi-Tier Communication

```
┌─────────────────────────────────────────────────────────┐
│                  Cloud / Application Layer              │
│         (AWS IoT, Azure IoT Hub, Google Cloud)          │
└───────────────────────┬─────────────────────────────────┘
                        │
                   Internet / 4G
                        │
┌───────────────────────▼─────────────────────────────────┐
│                   Edge Gateway                          │
│       (Aggregation, Edge AI, Local Storage)             │
└─────┬─────────────┬─────────────┬─────────────┬─────────┘
      │             │             │             │
   LoRaWAN       Ethernet      Cellular    Satellite
      │             │             │             │
┌─────▼──┐    ┌─────▼──┐    ┌─────▼──┐    ┌─────▼──┐
│ Buoys  │    │  RAS   │    │ Mobile │    │ Offshore│
│Acoustic│    │ Tanks  │    │ Assets │    │  Cages  │
└────────┘    └────────┘    └────────┘    └────────┘
```

### 6.2 Data Flow

1. **Underwater Sensors** → Acoustic Modem → Surface Buoy
2. **Surface Buoy** → LoRaWAN → Edge Gateway
3. **Edge Gateway** → 4G/Ethernet → Cloud Platform
4. **Cloud Platform** → WebSocket/MQTT → User Dashboard

---

## 7. Security Protocols

### 7.1 Encryption

- **TLS 1.3**: For HTTPS, MQTTS, WebSocket Secure
- **DTLS**: For UDP-based protocols (CoAP)
- **AES-256**: For payload encryption in acoustic/LoRa
- **End-to-End Encryption**: Critical data (credentials, financial)

### 7.2 Authentication

- **JWT (JSON Web Tokens)**: For API authentication
- **Device Certificates**: X.509 certificates for IoT devices
- **LoRaWAN Security**: AES-128 encryption with unique device keys
- **MQTT Authentication**: Username/password + client certificates

### 7.3 Key Management

```json
{
  "deviceId": "aq-sensor-12345",
  "appKey": "00112233445566778899AABBCCDDEEFF",
  "nwkSKey": "...",
  "appSKey": "...",
  "certificateARN": "arn:aws:iot:...",
  "expiryDate": "2026-01-01T00:00:00Z"
}
```

---

## 8. Quality of Service (QoS)

### 8.1 MQTT QoS Levels

- **QoS 0 (At most once)**: Non-critical sensor data (temperature, every minute)
- **QoS 1 (At least once)**: Important data (water quality alerts)
- **QoS 2 (Exactly once)**: Critical commands (feeding triggers, aeration control)

### 8.2 Priority Levels

| Priority | Use Case | Latency Target |
|----------|----------|----------------|
| 0 (Critical) | Dissolved oxygen alerts, disease outbreak | <5 seconds |
| 1 (High) | Water quality alarms, system failures | <30 seconds |
| 2 (Medium) | Sensor data, feeding events | <5 minutes |
| 3 (Low) | Historical data sync, reports | <1 hour |

---

## 9. Bandwidth Optimization

### 9.1 Data Compression

```javascript
// Before compression: 512 bytes
{
  "sensorId": "aq-sensor-12345",
  "timestamp": "2025-01-15T14:30:00Z",
  "temperature": 18.5,
  "dissolvedOxygen": 7.8,
  "pH": 8.1,
  "salinity": 32.5
}

// After compression (CBOR): 85 bytes
// 83% reduction
```

### 9.2 Sampling & Aggregation

```javascript
// High-frequency sampling: 10-second intervals
// Transmit: 5-minute aggregates

{
  "timestamp": "2025-01-15T14:30:00Z",
  "interval": 300,
  "temperature": {
    "avg": 18.5,
    "min": 18.2,
    "max": 18.7,
    "samples": 30
  }
}
```

---

## 10. Korean Aquaculture Protocol Extensions

### 10.1 NIFS (국립수산과학원) Data Protocol

```json
{
  "reportType": "monthly_production",
  "farmLicense": "KR-AQ-12345",
  "reportPeriod": "2025-01",
  "species": "넙치 (Paralichthys olivaceus)",
  "data": {
    "productionVolume": 15000,
    "mortalityRate": 2.3,
    "feedConversionRatio": 1.15,
    "waterQuality": {
      "avgTemperature": 18.2,
      "avgDO": 7.5,
      "avgPH": 8.0
    }
  },
  "certifications": ["HACCP"],
  "submittedBy": "did:wia:farm:blue-ocean",
  "timestamp": "2025-02-01T09:00:00+09:00"
}
```

### 10.2 Seafood Traceability Protocol

```json
{
  "traceabilityCode": "TR-2025-001-넙치-12345",
  "harvestId": "harvest-2025-001",
  "farmInfo": {
    "license": "KR-AQ-12345",
    "name": "블루오션 양식장",
    "location": "경남 통영시"
  },
  "productInfo": {
    "species": "넙치",
    "harvestDate": "2025-01-10",
    "quantity": 15000,
    "sizeGrade": "대 (Large)"
  },
  "qualityData": {
    "antibioticFree": true,
    "heavyMetals": "ND (Not Detected)",
    "microplastics": "ND"
  },
  "blockchain": {
    "network": "Klaytn",
    "txHash": "0x7f8d3c2a..."
  }
}
```

---

## 11. Protocol Testing & Validation

### 11.1 Acoustic Channel Simulation

```python
# Test acoustic modem performance under different conditions

test_conditions = [
    {"salinity": 35, "temperature": 20, "turbidity": 5, "distance": 1000},
    {"salinity": 30, "temperature": 15, "turbidity": 15, "distance": 2000},
    {"salinity": 32, "temperature": 18, "turbidity": 10, "distance": 3000}
]

for condition in test_conditions:
    snr = calculate_snr(condition)
    ber = calculate_bit_error_rate(snr)
    throughput = calculate_throughput(ber, data_rate=20000)
    print(f"Condition: {condition}")
    print(f"  SNR: {snr:.2f} dB")
    print(f"  BER: {ber:.2e}")
    print(f"  Throughput: {throughput/1000:.2f} kbps\n")
```

### 11.2 End-to-End Latency Testing

```javascript
// Measure total latency from sensor to cloud

const latencyTest = {
  sensor_to_modem: 50,      // ms
  acoustic_propagation: 200, // ms (300m @ 1500 m/s)
  modem_to_buoy: 10,        // ms
  lora_transmission: 150,    // ms
  gateway_processing: 20,    // ms
  internet_upload: 100,      // ms
  cloud_processing: 50,      // ms
  total: 580                 // ms
};

console.log(`Total latency: ${latencyTest.total} ms`);
// Acceptable for non-critical data
```

---

**Document Status**: ✅ Phase 3 Complete
**Next Phase**: [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

---

© 2025 WIA (World Certification Industry Association)
**License**: MIT
**Philosophy**: 弘益人間 (Benefit All Humanity)
