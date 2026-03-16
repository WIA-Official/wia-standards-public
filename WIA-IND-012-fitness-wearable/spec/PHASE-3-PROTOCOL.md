# WIA-IND-012 PHASE 3: PROTOCOL SPECIFICATION
## Fitness Wearable Standard - 弘益人間 (Benefit All Humanity)

**Version:** 1.0
**Status:** Active
**Category:** IND (Industrial)
**Last Updated:** 2025-01-15

---

## 1. Introduction

Phase 3 of WIA-IND-012 defines communication protocols, data compression, battery optimization, and real-time streaming for fitness wearable devices. The **弘益人間** principle ensures protocols are efficient, secure, and accessible to all device classes regardless of computational capabilities.

### 1.1 Scope

- Bluetooth Low Energy (BLE) protocols
- ANT+ communication standards
- Data compression algorithms
- Battery optimization strategies
- Real-time streaming protocols
- Network communication standards

---

## 2. Bluetooth Low Energy (BLE) Protocol

### 2.1 BLE Version Requirements

- **Minimum:** Bluetooth 4.2
- **Recommended:** Bluetooth 5.0+
- **Features:** Extended range, 2× speed, 8× broadcast capacity

### 2.2 GATT Profile Structure

```
WIA Fitness Service
UUID: 0x181D (Fitness Service - Standard UUID)

Characteristics:
- Heart Rate Measurement (0x2A37)
  - Notify: Real-time HR data
  - Properties: Notify
  - Value Format: UINT8 (BPM)

- Steps Count (Custom UUID: 0xFFA1)
  - Read/Notify
  - Properties: Read, Notify
  - Value Format: UINT32

- Battery Level (0x2A19)
  - Read/Notify
  - Properties: Read, Notify
  - Value Format: UINT8 (0-100%)

- Device Control (Custom UUID: 0xFFA2)
  - Write/Indicate
  - Properties: Write, Indicate
  - Commands: Sync, Reset, Config
```

### 2.3 Connection Parameters

**Connection Interval:**
- Minimum: 7.5ms (high throughput)
- Maximum: 4000ms (low power)
- Recommended: 100ms (balance)

**Slave Latency:**
- Range: 0-499
- Recommended: 4 (allows skipping 4 events for power saving)

**Supervision Timeout:**
- Minimum: 100ms
- Maximum: 32s
- Recommended: 10s

### 2.4 Data Packet Structure

```
[Header (2 bytes)] [Data Length (1 byte)] [Data (N bytes)] [CRC (2 bytes)]

Header:
- Byte 0: Protocol Version (0x01)
- Byte 1: Message Type
  - 0x01: Heart Rate
  - 0x02: Steps
  - 0x03: Sleep
  - 0x04: Activity
  - 0x05: GPS
  - 0xFF: Control

Data Format (Heart Rate Example):
[0x01] [0x01] [0x04] [BPM] [Confidence] [Quality] [Reserved] [CRC-16]
```

---

## 3. ANT+ Protocol

### 3.1 ANT+ Device Profiles

**Heart Rate Monitor Profile:**
- Channel Type: Slave
- Device Type: 0x78 (120)
- Transmission Type: 0x05
- Channel Period: 8070 (4 Hz)
- RF Frequency: 2457 MHz (Channel 57)

**Speed and Cadence Profile:**
- Device Type: 0x79 (121)
- Channel Period: 8086 (4 Hz)

### 3.2 ANT+ Data Page Format

```
Heart Rate Data Page (Page 0):
Byte 0: Page Number (0x00)
Byte 1: Reserved
Byte 2: Reserved
Byte 3: Reserved
Byte 4: Heart Rate Event Time LSB
Byte 5: Heart Rate Event Time MSB
Byte 6: Heart Beat Count
Byte 7: Computed Heart Rate (BPM)
```

### 3.3 ANT+ Network Key

WIA Standard ANT+ Network Key (Public):
```
0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45
```

---

## 4. Data Compression

### 4.1 Compression Algorithms

**LZ4 Compression:**
- Use Case: Real-time data streams
- Compression Ratio: ~2-3×
- Speed: 300-500 MB/s compression
- CPU Overhead: Minimal

**GZIP Compression:**
- Use Case: Historical data export
- Compression Ratio: ~3-5×
- Speed: 50-100 MB/s compression
- CPU Overhead: Moderate

**Custom WIA Compression:**
- Use Case: Highly structured wearable data
- Compression Ratio: ~4-7×
- Features: Schema-aware, predictive encoding

### 4.2 WIA Custom Compression Algorithm

**Principle:** Delta encoding + Run-length encoding + Huffman coding

**Example (Heart Rate Compression):**
```
Original: [72, 73, 73, 74, 74, 74, 75, 75, 76, 77]
Delta:    [72, +1, 0, +1, 0, 0, +1, 0, +1, +1]
RLE:      [72, +1×1, 0×1, +1×1, 0×2, +1×1, 0×1, +1×2]
Huffman:  [Encoded binary stream]

Compression: 10 bytes → 4 bytes (60% reduction)
```

### 4.3 Compression Selection Matrix

| Data Type | Real-Time | Historical | Export |
|-----------|-----------|------------|--------|
| Heart Rate | LZ4 | GZIP | Custom WIA |
| Steps | None | GZIP | GZIP |
| GPS Track | LZ4 | GZIP | Custom WIA |
| Sleep | None | GZIP | GZIP |

---

## 5. Battery Optimization

### 5.1 Power Consumption Profile

**Typical Component Power Draw:**
- ARM Cortex-M4 MCU (Active): 15 mW
- PPG Sensor (Continuous): 8 mW
- Accelerometer (25 Hz): 0.5 mW
- GPS (Active): 120 mW
- BLE (Connected): 12 mW
- BLE (Advertising): 0.3 mW
- Display (Active): 25 mW

### 5.2 Adaptive Sampling Strategy

**Context-Aware Sampling:**

```python
def determine_sampling_rate(activity_level, battery_level):
    if battery_level < 0.15:  # Low battery
        return {
            'hr_interval': 60,      # 1 minute
            'accel_rate': 10,       # 10 Hz
            'gps_enabled': False
        }
    elif activity_level == 'stationary':
        return {
            'hr_interval': 30,      # 30 seconds
            'accel_rate': 10,
            'gps_enabled': False
        }
    elif activity_level == 'light_activity':
        return {
            'hr_interval': 5,       # 5 seconds
            'accel_rate': 25,
            'gps_enabled': False
        }
    else:  # Intense activity
        return {
            'hr_interval': 1,       # 1 second
            'accel_rate': 100,
            'gps_enabled': True,
            'gps_interval': 1       # 1 second
        }
```

### 5.3 Power Modes

**Mode 1: Always-On (Default)**
- HR: Continuous (1 Hz)
- Accel: 25 Hz
- Battery Life: 5-7 days
- Use Case: Normal daily use

**Mode 2: Power Saver**
- HR: 0.1 Hz (every 10 seconds)
- Accel: 10 Hz
- Battery Life: 10-14 days
- Use Case: Minimal tracking

**Mode 3: Workout Mode**
- HR: 1 Hz
- Accel: 100 Hz
- GPS: 1 Hz
- Battery Life: 10-20 hours
- Use Case: Active exercise tracking

**Mode 4: Ultra Low Power**
- HR: On-demand only
- Accel: 1 Hz
- Battery Life: 20-30 days
- Use Case: Basic step counting only

---

## 6. Real-Time Streaming Protocol

### 6.1 WebSocket Protocol

**Connection Establishment:**
```
Client → Server: WebSocket Upgrade Request
GET /v1/stream HTTP/1.1
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==
Sec-WebSocket-Protocol: wia-fitness-v1
Sec-WebSocket-Version: 13
Authorization: Bearer {token}

Server → Client: WebSocket Upgrade Response
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: HSmrc0sMlYUkAGmm5OPpG2HaGWk=
Sec-WebSocket-Protocol: wia-fitness-v1
```

### 6.2 Message Frame Format

```json
{
  "protocol": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "message_id": "msg-123456",
  "timestamp": "2025-01-15T10:30:45.123Z",
  "type": "heart_rate_update",
  "data": {
    "bpm": 75,
    "confidence": 0.95
  }
}
```

### 6.3 Heartbeat/Ping Protocol

```
Client → Server (every 30s):
{"type": "ping", "timestamp": "2025-01-15T10:30:00.000Z"}

Server → Client:
{"type": "pong", "timestamp": "2025-01-15T10:30:00.050Z"}
```

---

## 7. Data Synchronization Protocol

### 7.1 Sync Strategy

**Incremental Sync:**
```
1. Device stores last_sync_timestamp
2. On connection, device sends: GET /sync?since={last_sync_timestamp}
3. Server responds with delta changes
4. Device applies changes and updates last_sync_timestamp
5. Device sends new data created since last sync
```

**Conflict Resolution:**
- Server wins for profile data
- Latest timestamp wins for measurement data
- Merge strategy for aggregated data (steps, calories)

### 7.2 Sync Packet Format

```json
{
  "sync_id": "sync-789012",
  "device_id": "WIA-DEVICE-001",
  "last_sync": "2025-01-15T00:00:00.000Z",
  "current_time": "2025-01-15T23:59:59.999Z",
  "data_batches": [
    {
      "type": "heart_rate",
      "count": 1440,
      "compressed": true,
      "compression_algorithm": "LZ4",
      "data": "base64_encoded_compressed_data"
    },
    {
      "type": "steps",
      "count": 24,
      "compressed": false,
      "data": [...]
    }
  ],
  "philosophy": "弘益人間"
}
```

---

## 8. Network Communication Standards

### 8.1 HTTP/2 Protocol

- Multiplexing: Multiple streams over single connection
- Server Push: Proactive data delivery
- Header Compression: HPACK algorithm
- Binary Framing: Improved efficiency

### 8.2 TLS Requirements

- **Minimum:** TLS 1.2
- **Recommended:** TLS 1.3
- **Cipher Suites:**
  - TLS_AES_128_GCM_SHA256
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256

### 8.3 Connection Pooling

- Maintain persistent connections
- Maximum 6 connections per domain
- Connection timeout: 60 seconds
- Keep-alive: Enabled

---

## 9. Quality of Service (QoS)

### 9.1 Priority Levels

**Priority 0 (Highest):** Real-time heart rate during workout
**Priority 1:** Live GPS tracking
**Priority 2:** Activity session data
**Priority 3:** Historical data sync
**Priority 4 (Lowest):** Bulk data export

### 9.2 Bandwidth Management

**Adaptive Bitrate:**
```
Network Speed → Data Quality
> 5 Mbps:     Full resolution, uncompressed
1-5 Mbps:     Full resolution, LZ4 compressed
0.5-1 Mbps:   Reduced sampling, GZIP compressed
< 0.5 Mbps:   Minimal data, high compression
```

---

## 10. Error Handling and Retry Logic

### 10.1 Exponential Backoff

```
Attempt 1: Wait 1 second
Attempt 2: Wait 2 seconds
Attempt 3: Wait 4 seconds
Attempt 4: Wait 8 seconds
...
Max Wait: 60 seconds
Max Attempts: 5
```

### 10.2 Error Recovery

**Transient Errors:** Retry with exponential backoff
**Network Errors:** Queue data locally, retry on reconnection
**Authentication Errors:** Refresh token, retry once
**Server Errors (5xx):** Exponential backoff, alert user after 5 attempts

---

## 11. Security Protocols

### 11.1 End-to-End Encryption

```
1. Client generates ephemeral key pair
2. Server provides public key
3. Establish shared secret via ECDH
4. Derive encryption key via HKDF
5. Encrypt data with AES-256-GCM
6. Include HMAC for integrity
```

### 11.2 Certificate Pinning

- Pin SHA-256 hash of server certificate
- Prevent man-in-the-middle attacks
- Update pins during app updates

---

## 12. Offline Operation

### 12.1 Local Data Storage

- SQLite database for structured data
- LevelDB for key-value pairs
- Maximum local storage: 500 MB
- Auto-cleanup: Delete data >90 days old

### 12.2 Offline-First Architecture

```
1. All operations work offline first
2. Write to local database immediately
3. Queue sync operations
4. Background sync when network available
5. Conflict resolution on merge
```

---

## 13. Testing and Validation

### 13.1 Protocol Testing

- Unit tests for compression algorithms
- Integration tests for BLE/ANT+ communication
- Load testing for concurrent connections
- Latency testing for real-time streams

### 13.2 Compliance Testing

- BLE SIG qualification
- ANT+ certification
- Energy consumption validation
- Interoperability testing

---

## 14. Implementation Guidelines

### 14.1 Firmware Requirements

```c
// BLE Connection Example
void ble_init() {
    ble_cfg_t ble_cfg;
    ble_cfg.conn_interval_min = 100;  // 100ms
    ble_cfg.conn_interval_max = 200;  // 200ms
    ble_cfg.slave_latency = 4;
    ble_cfg.supervision_timeout = 10000;  // 10s

    ble_stack_init(&ble_cfg);
    gatt_service_init();
    advertising_start();
}
```

### 14.2 App Requirements

```javascript
// WebSocket Connection Example
const ws = new WebSocket('wss://stream.wia-fitness.org/v1/live');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    channel: 'heart_rate',
    user_id: userId,
    philosophy: '弘益人間'
  }));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  updateUI(message);
});
```

---

## 15. Conclusion

Phase 3 of WIA-IND-012 establishes efficient, secure communication protocols enabling real-time fitness tracking while optimizing battery life and ensuring data integrity. These protocols support the **弘益人間** vision by ensuring accessible, reliable wearable technology for all humanity.

---

**Document Control**

- Version: 1.0
- Effective Date: 2025-01-15
- Next Review: 2026-01-15
- Contact: standards@wia.org

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
