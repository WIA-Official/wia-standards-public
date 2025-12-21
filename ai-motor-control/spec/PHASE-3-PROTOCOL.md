# WIA AI Motor Control - Phase 3: Communication Protocol Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

This specification defines the real-time communication protocol for AI motor control systems. The protocol ensures low-latency, deterministic communication between AI controllers and motor hardware with built-in safety mechanisms and fault tolerance.

### 1.2 Design Goals

1. **Low Latency**: Sub-millisecond command-to-execution delay
2. **Deterministic**: Predictable timing for real-time control
3. **Safety-Critical**: Built-in safety validation and emergency protocols
4. **Fault Tolerant**: Automatic recovery and graceful degradation
5. **Multi-Transport**: Support for multiple physical layers

### 1.3 Protocol Stack

```mermaid
graph TB
    subgraph "Application Layer"
        A[Motor Commands]
        B[Feedback Data]
        C[Safety Messages]
    end

    subgraph "Session Layer"
        D[Connection Management]
        E[Authentication]
        F[Heartbeat]
    end

    subgraph "Transport Layer"
        G[Reliable Transport]
        H[Unreliable Transport]
        I[Priority Queues]
    end

    subgraph "Network Layer"
        J[UDP/IP]
        K[TCP/IP]
        L[EtherCAT]
        M[CAN]
    end

    subgraph "Physical Layer"
        N[Ethernet]
        O[WiFi]
        P[CAN Bus]
        Q[EtherCAT Bus]
    end

    A --> D
    B --> D
    C --> D
    D --> E
    E --> F
    F --> G
    F --> H
    G --> J
    G --> K
    H --> J
    I --> G
    J --> N
    J --> O
    K --> N
    L --> Q
    M --> P
```

---

## 2. Transport Protocols

### 2.1 Supported Transports

| Transport | Latency | Reliability | Use Case |
|-----------|---------|-------------|----------|
| **UDP** | < 1ms | Unreliable | Real-time feedback streaming |
| **TCP** | 1-5ms | Reliable | Configuration, trajectories |
| **WebSocket** | 2-10ms | Reliable | Browser-based control |
| **EtherCAT** | < 100μs | Deterministic | Industrial robotics |
| **CAN** | < 1ms | Reliable | Embedded systems |
| **Shared Memory** | < 10μs | N/A | Same-host IPC |

### 2.2 Transport Selection

```typescript
enum TransportType {
  UDP = 'udp',
  TCP = 'tcp',
  WEBSOCKET = 'websocket',
  ETHERCAT = 'ethercat',
  CAN = 'can',
  SHARED_MEMORY = 'shared_memory'
}

interface TransportConfig {
  type: TransportType;
  address: string;
  port?: number;
  priority?: number;
  qos?: QoSConfig;
}
```

---

## 3. Message Format

### 3.1 Common Message Header

All protocol messages share a common header format:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|    Version    |  Message Type |         Sequence Number       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          Timestamp (64-bit)                   |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|         Payload Length        |    CRC16      |     Flags     |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          Payload Data                         |
|                             ...                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

**Header Fields**:

| Field | Size | Description |
|-------|------|-------------|
| Version | 8 bits | Protocol version (current: 1) |
| Message Type | 8 bits | Message type identifier |
| Sequence Number | 16 bits | Monotonically increasing counter |
| Timestamp | 64 bits | Unix timestamp in microseconds |
| Payload Length | 16 bits | Length of payload in bytes |
| CRC16 | 16 bits | CRC-16 checksum of payload |
| Flags | 8 bits | Message flags (priority, ack, etc.) |

### 3.2 Message Types

```c
// Message Type Definitions
#define MSG_TYPE_COMMAND          0x01
#define MSG_TYPE_FEEDBACK         0x02
#define MSG_TYPE_TRAJECTORY       0x03
#define MSG_TYPE_CONFIG           0x04
#define MSG_TYPE_STATUS           0x05
#define MSG_TYPE_HEARTBEAT        0x06
#define MSG_TYPE_EMERGENCY_STOP   0x07
#define MSG_TYPE_ACK              0x08
#define MSG_TYPE_NACK             0x09
#define MSG_TYPE_ERROR            0x0A

// Flag Bits
#define FLAG_PRIORITY_HIGH        0x80
#define FLAG_REQUIRE_ACK          0x40
#define FLAG_BROADCAST            0x20
#define FLAG_ENCRYPTED            0x10
```

---

## 4. Command Protocol

### 4.1 Motor Command Message

**Message Type**: `0x01` (COMMAND)

**Payload Format**:

```c
struct MotorCommandPayload {
    uint32_t motor_id_hash;      // CRC32 hash of motor_id string
    uint8_t  control_mode;       // 0=disabled, 1=position, 2=velocity, 3=torque
    uint8_t  flags;              // Command flags
    uint16_t reserved;
    float    setpoint;           // Target value
    float    velocity_limit;     // Max velocity
    float    acceleration_limit; // Max acceleration
    float    torque_limit;       // Max torque
    float    kp;                 // PID proportional gain
    float    ki;                 // PID integral gain
    float    kd;                 // PID derivative gain
};  // Total: 40 bytes
```

**Binary Example**:

```
01 01 00 01 00 00 01 7A 3B 9A CA 00 00 00 28 A5 B2 00
↑  ↑  ↑     ↑           ↑                 ↑     ↑  ↑
│  │  │     │           │                 │     │  └─ Flags
│  │  │     │           │                 │     └─── CRC16
│  │  │     │           │                 └───────── Payload Length (40)
│  │  │     │           └─────────────────────────── Timestamp
│  │  │     └─────────────────────────────────────── Sequence Number
│  │  └───────────────────────────────────────────── Message Type (COMMAND)
│  └──────────────────────────────────────────────── Version (1)
```

### 4.2 Multi-Motor Command

**Message Type**: `0x01` (COMMAND) with broadcast flag

**Payload Format**:

```c
struct MultiMotorCommandPayload {
    uint8_t  motor_count;        // Number of motors (max 32)
    uint8_t  sync_mode;          // 0=sequential, 1=simultaneous
    uint16_t total_duration_ms;  // Total movement duration
    struct {
        uint32_t motor_id_hash;
        uint8_t  control_mode;
        uint8_t  reserved;
        uint16_t reserved2;
        float    setpoint;
        float    velocity_limit;
    } motors[motor_count];
};
```

---

## 5. Feedback Protocol

### 5.1 Feedback Message

**Message Type**: `0x02` (FEEDBACK)

**Payload Format**:

```c
struct MotorFeedbackPayload {
    uint32_t motor_id_hash;      // CRC32 hash of motor_id
    uint64_t timestamp_us;       // Feedback timestamp (microseconds)
    float    position;           // Current position
    float    velocity;           // Current velocity
    float    acceleration;       // Current acceleration
    float    torque;             // Current torque
    float    current;            // Motor current (A)
    float    voltage;            // Motor voltage (V)
    float    temperature;        // Motor temperature (°C)
    uint16_t status_flags;       // Status bit flags
    uint8_t  health_score;       // Health score (0-100)
    uint8_t  reserved;
};  // Total: 52 bytes
```

**Status Flags**:

```c
#define STATUS_ENABLED           0x0001
#define STATUS_FAULT             0x0002
#define STATUS_WARNING           0x0004
#define STATUS_CALIBRATED        0x0008
#define STATUS_HOMED             0x0010
#define STATUS_IN_MOTION         0x0020
#define STATUS_AT_TARGET         0x0040
#define STATUS_OVER_TEMP         0x0080
#define STATUS_OVER_CURRENT      0x0100
#define STATUS_POSITION_LIMIT    0x0200
#define STATUS_ENCODER_ERROR     0x0400
```

### 5.2 Feedback Streaming

**Update Rates**:

| Control Mode | Feedback Rate | Protocol |
|--------------|---------------|----------|
| Position | 100 Hz | UDP |
| Velocity | 200 Hz | UDP |
| Torque | 500 Hz | UDP |
| Impedance | 1000 Hz | UDP/EtherCAT |
| Trajectory | 100 Hz | UDP |

**Streaming Configuration**:

```json
{
  "motor_id": "shoulder_pitch",
  "stream_config": {
    "enabled": true,
    "rate_hz": 100,
    "protocol": "udp",
    "decimation": 1,
    "fields": ["position", "velocity", "torque", "current", "status"],
    "buffer_size": 1024
  }
}
```

---

## 6. Safety Protocol

### 6.1 Emergency Stop Message

**Message Type**: `0x07` (EMERGENCY_STOP)

**Priority**: Highest (sent immediately, bypasses queues)

**Payload Format**:

```c
struct EmergencyStopPayload {
    uint8_t  stop_type;          // 0=all, 1=specific motors, 2=motor group
    uint8_t  motor_count;        // Number of motors to stop
    uint16_t reserved;
    uint32_t motor_id_hashes[motor_count];  // Target motor IDs
    uint64_t trigger_timestamp;  // When emergency was triggered
    char     reason[32];         // Reason string (null-terminated)
};
```

**Stop Types**:

```c
#define ESTOP_ALL_MOTORS         0x00
#define ESTOP_SPECIFIC_MOTORS    0x01
#define ESTOP_MOTOR_GROUP        0x02
#define ESTOP_RELEASE            0xFF
```

### 6.2 Heartbeat Protocol

**Message Type**: `0x06` (HEARTBEAT)

**Frequency**: 10 Hz (every 100ms)

**Payload Format**:

```c
struct HeartbeatPayload {
    uint32_t node_id;            // Sender node ID
    uint8_t  node_type;          // 0=controller, 1=motor, 2=monitor
    uint8_t  state;              // Node state
    uint16_t uptime_seconds;     // Uptime since boot
    uint8_t  health_score;       // Overall health (0-100)
    uint8_t  warning_count;      // Number of active warnings
    uint8_t  fault_count;        // Number of active faults
    uint8_t  reserved;
};
```

**Node States**:

```c
#define STATE_BOOT               0x00
#define STATE_READY              0x01
#define STATE_OPERATIONAL        0x02
#define STATE_WARNING            0x03
#define STATE_FAULT              0x04
#define STATE_EMERGENCY          0x05
#define STATE_SHUTDOWN           0xFF
```

### 6.3 Watchdog Protocol

**Timeout Behavior**:

| Component | Timeout | Action |
|-----------|---------|--------|
| Controller → Motor | 200ms | Controlled deceleration to stop |
| Motor → Controller | 500ms | Mark motor offline |
| Heartbeat | 1000ms | Trigger fault state |
| Command ACK | 100ms | Resend command (max 3 retries) |

---

## 7. Connection Management

### 7.1 Connection Handshake

```mermaid
sequenceDiagram
    participant C as Controller
    participant M as Motor

    C->>M: CONNECT_REQUEST
    Note over C,M: Version, Capabilities, Auth Token
    M->>C: CONNECT_RESPONSE
    Note over M,C: Accept/Reject, Session ID
    C->>M: CONFIG_SYNC
    Note over C,M: Initial configuration
    M->>C: CONFIG_ACK
    C->>M: START_HEARTBEAT
    M->>C: HEARTBEAT_ACK
    Note over C,M: Connection established
```

### 7.2 Connection Request

```c
struct ConnectRequestPayload {
    uint8_t  protocol_version;   // Protocol version
    uint8_t  client_type;        // 0=controller, 1=monitor
    uint16_t capabilities;       // Capability flags
    char     auth_token[64];     // Authentication token
    uint32_t requested_session_id;
};
```

### 7.3 Connection Response

```c
struct ConnectResponsePayload {
    uint8_t  status;             // 0=accepted, 1=rejected, 2=auth_failed
    uint8_t  assigned_priority;  // Assigned priority level
    uint16_t reserved;
    uint32_t session_id;         // Assigned session ID
    uint32_t heartbeat_interval_ms;
    uint32_t command_timeout_ms;
};
```

---

## 8. Quality of Service (QoS)

### 8.1 Priority Levels

| Priority | Value | Description | Max Latency |
|----------|-------|-------------|-------------|
| Critical | 0 | Emergency stop, safety | < 1ms |
| High | 1 | Real-time commands | < 5ms |
| Normal | 2 | Standard commands | < 20ms |
| Low | 3 | Status queries | < 100ms |
| Background | 4 | Logs, diagnostics | < 1s |

### 8.2 Message Queuing

```typescript
interface QoSConfig {
  priority: number;
  max_retries: number;
  retry_delay_ms: number;
  ttl_ms: number;
  require_ack: boolean;
  encryption: boolean;
}

const qosProfiles = {
  emergency: {
    priority: 0,
    max_retries: 0,
    retry_delay_ms: 0,
    ttl_ms: 10,
    require_ack: false,
    encryption: false
  },
  realtime: {
    priority: 1,
    max_retries: 1,
    retry_delay_ms: 5,
    ttl_ms: 50,
    require_ack: true,
    encryption: false
  },
  reliable: {
    priority: 2,
    max_retries: 3,
    retry_delay_ms: 100,
    ttl_ms: 1000,
    require_ack: true,
    encryption: true
  }
};
```

---

## 9. Time Synchronization

### 9.1 Clock Synchronization Protocol

Uses Precision Time Protocol (PTP) IEEE 1588 for sub-microsecond synchronization.

```mermaid
sequenceDiagram
    participant M as Master Clock
    participant S as Slave Clock

    M->>S: SYNC (t1)
    Note over S: Record receive time (t2)
    M->>S: FOLLOW_UP (t1)
    S->>M: DELAY_REQ (t3)
    Note over M: Record receive time (t4)
    M->>S: DELAY_RESP (t4)
    Note over S: Calculate offset and delay
```

**Offset Calculation**:

```
offset = ((t2 - t1) - (t4 - t3)) / 2
delay = ((t2 - t1) + (t4 - t3)) / 2
```

### 9.2 Timestamp Format

```c
struct Timestamp {
    uint64_t seconds;            // Seconds since epoch
    uint32_t nanoseconds;        // Nanoseconds (0-999999999)
    uint32_t reserved;
};
```

---

## 10. Error Handling

### 10.1 Error Message

**Message Type**: `0x0A` (ERROR)

**Payload Format**:

```c
struct ErrorPayload {
    uint16_t error_code;         // Error code
    uint8_t  severity;           // 0=info, 1=warning, 2=error, 3=critical
    uint8_t  recoverable;        // 0=fatal, 1=recoverable
    uint32_t source_motor_id;    // Source motor ID (if applicable)
    uint64_t error_timestamp;    // When error occurred
    char     description[64];    // Error description
    uint8_t  stack_trace[32];    // Optional stack trace
};
```

### 10.2 Error Codes

| Code | Name | Description | Severity |
|------|------|-------------|----------|
| 0x0001 | INVALID_COMMAND | Invalid command parameters | Warning |
| 0x0002 | LIMIT_EXCEEDED | Command exceeds safety limits | Error |
| 0x0003 | MOTOR_OFFLINE | Motor not responding | Error |
| 0x0004 | TIMEOUT | Command timeout | Warning |
| 0x0005 | OVERCURRENT | Motor overcurrent detected | Critical |
| 0x0006 | OVERTEMP | Motor overtemperature | Critical |
| 0x0007 | ENCODER_FAULT | Encoder error | Error |
| 0x0008 | POSITION_ERROR | Position tracking error too large | Warning |
| 0x0009 | ESTOP_ACTIVE | Emergency stop is active | Critical |
| 0x000A | COLLISION | Collision detected | Critical |

### 10.3 Error Recovery

```typescript
interface ErrorRecovery {
  error_code: number;
  recovery_action: 'retry' | 'recalibrate' | 'disable' | 'estop';
  max_retries: number;
  retry_delay_ms: number;
  fallback_mode?: string;
}

const errorRecoveryTable: Record<number, ErrorRecovery> = {
  0x0001: { error_code: 0x0001, recovery_action: 'retry', max_retries: 3, retry_delay_ms: 100 },
  0x0002: { error_code: 0x0002, recovery_action: 'disable', max_retries: 0, retry_delay_ms: 0 },
  0x0005: { error_code: 0x0005, recovery_action: 'estop', max_retries: 0, retry_delay_ms: 0 },
  0x0006: { error_code: 0x0006, recovery_action: 'disable', max_retries: 0, retry_delay_ms: 0, fallback_mode: 'cooling' },
};
```

---

## 11. Implementation Examples

### 11.1 C++ Implementation

```cpp
#include <wia/motor_control.h>

class MotorProtocolClient {
public:
    MotorProtocolClient(const std::string& address, uint16_t port)
        : endpoint_(address, port) {
        socket_.open(udp::v4());
    }

    void sendCommand(const MotorCommand& cmd) {
        // Build message header
        MessageHeader header;
        header.version = PROTOCOL_VERSION;
        header.msg_type = MSG_TYPE_COMMAND;
        header.sequence = ++sequence_number_;
        header.timestamp = getCurrentTimestamp();

        // Serialize command payload
        MotorCommandPayload payload;
        payload.motor_id_hash = hashMotorId(cmd.motor_id);
        payload.control_mode = static_cast<uint8_t>(cmd.control_mode);
        payload.setpoint = cmd.setpoint;
        payload.velocity_limit = cmd.velocity_limit;
        payload.kp = cmd.pid_gains.kp;
        payload.ki = cmd.pid_gains.ki;
        payload.kd = cmd.pid_gains.kd;

        // Calculate CRC
        header.crc16 = calculateCRC16(&payload, sizeof(payload));
        header.payload_length = sizeof(payload);

        // Send packet
        std::vector<uint8_t> packet;
        packet.insert(packet.end(),
                     reinterpret_cast<uint8_t*>(&header),
                     reinterpret_cast<uint8_t*>(&header) + sizeof(header));
        packet.insert(packet.end(),
                     reinterpret_cast<uint8_t*>(&payload),
                     reinterpret_cast<uint8_t*>(&payload) + sizeof(payload));

        socket_.send_to(boost::asio::buffer(packet), endpoint_);
    }

    void receiveFeedback(std::function<void(const MotorFeedback&)> callback) {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_),
            sender_endpoint_,
            [this, callback](const boost::system::error_code& error, size_t bytes_received) {
                if (!error) {
                    // Parse header
                    MessageHeader* header = reinterpret_cast<MessageHeader*>(recv_buffer_.data());

                    if (header->msg_type == MSG_TYPE_FEEDBACK) {
                        // Verify CRC
                        uint8_t* payload_ptr = recv_buffer_.data() + sizeof(MessageHeader);
                        uint16_t calculated_crc = calculateCRC16(payload_ptr, header->payload_length);

                        if (calculated_crc == header->crc16) {
                            MotorFeedbackPayload* payload =
                                reinterpret_cast<MotorFeedbackPayload*>(payload_ptr);

                            MotorFeedback feedback;
                            feedback.motor_id = reverseHashMotorId(payload->motor_id_hash);
                            feedback.timestamp = payload->timestamp_us;
                            feedback.position = payload->position;
                            feedback.velocity = payload->velocity;
                            feedback.torque = payload->torque;
                            feedback.current = payload->current;
                            feedback.temperature = payload->temperature;

                            callback(feedback);
                        }
                    }
                }

                // Continue receiving
                receiveFeedback(callback);
            }
        );
    }

private:
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    std::array<uint8_t, 1024> recv_buffer_;
    uint16_t sequence_number_ = 0;
};
```

### 11.2 Python Implementation

```python
import struct
import socket
import time
from dataclasses import dataclass
from typing import Callable

# Message types
MSG_TYPE_COMMAND = 0x01
MSG_TYPE_FEEDBACK = 0x02
MSG_TYPE_EMERGENCY_STOP = 0x07

@dataclass
class MotorCommand:
    motor_id: str
    control_mode: int
    setpoint: float
    velocity_limit: float
    kp: float
    ki: float
    kd: float

class MotorProtocolClient:
    def __init__(self, address: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (address, port)
        self.sequence = 0

    def send_command(self, cmd: MotorCommand):
        # Build header
        header = struct.pack(
            '!BBHQHHB',
            1,  # version
            MSG_TYPE_COMMAND,  # msg_type
            self.sequence,  # sequence
            int(time.time() * 1_000_000),  # timestamp
            40,  # payload_length
            0,  # crc16 (calculated below)
            0x40  # flags (require ACK)
        )

        # Build payload
        motor_id_hash = self._hash_motor_id(cmd.motor_id)
        payload = struct.pack(
            '!IBBHffffffff',
            motor_id_hash,
            cmd.control_mode,
            0,  # flags
            0,  # reserved
            cmd.setpoint,
            cmd.velocity_limit,
            0.0,  # acceleration_limit
            0.0,  # torque_limit
            cmd.kp,
            cmd.ki,
            cmd.kd
        )

        # Calculate CRC
        crc = self._calculate_crc16(payload)

        # Update header with CRC
        header = struct.pack(
            '!BBHQHHB',
            1, MSG_TYPE_COMMAND, self.sequence,
            int(time.time() * 1_000_000),
            len(payload), crc, 0x40
        )

        # Send packet
        packet = header + payload
        self.socket.sendto(packet, self.address)
        self.sequence = (self.sequence + 1) % 65536

    def receive_feedback(self, callback: Callable):
        while True:
            data, addr = self.socket.recvfrom(1024)

            # Parse header
            version, msg_type, seq, timestamp, payload_len, crc, flags = \
                struct.unpack('!BBHQHHB', data[:16])

            if msg_type == MSG_TYPE_FEEDBACK:
                # Verify CRC
                payload = data[16:16+payload_len]
                calculated_crc = self._calculate_crc16(payload)

                if calculated_crc == crc:
                    # Parse feedback
                    feedback_data = struct.unpack(
                        '!IQffffffffHBB',
                        payload
                    )

                    feedback = {
                        'motor_id_hash': feedback_data[0],
                        'timestamp': feedback_data[1],
                        'position': feedback_data[2],
                        'velocity': feedback_data[3],
                        'acceleration': feedback_data[4],
                        'torque': feedback_data[5],
                        'current': feedback_data[6],
                        'voltage': feedback_data[7],
                        'temperature': feedback_data[8],
                        'status_flags': feedback_data[9],
                        'health_score': feedback_data[10]
                    }

                    callback(feedback)

    def _hash_motor_id(self, motor_id: str) -> int:
        import zlib
        return zlib.crc32(motor_id.encode()) & 0xFFFFFFFF

    def _calculate_crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
```

---

## 12. Performance Benchmarks

### 12.1 Latency Measurements

| Transport | Avg Latency | 99th Percentile | Jitter |
|-----------|-------------|-----------------|--------|
| UDP (localhost) | 0.05ms | 0.2ms | 0.01ms |
| UDP (LAN) | 0.8ms | 2.5ms | 0.3ms |
| TCP (localhost) | 0.1ms | 0.5ms | 0.05ms |
| TCP (LAN) | 2.5ms | 8.0ms | 1.2ms |
| WebSocket (LAN) | 5.0ms | 15.0ms | 2.5ms |
| EtherCAT | 0.05ms | 0.1ms | 0.005ms |
| CAN (1Mbps) | 0.5ms | 1.5ms | 0.1ms |

### 12.2 Throughput

| Message Type | Size (bytes) | Max Rate (msg/s) | Bandwidth |
|--------------|--------------|------------------|-----------|
| Command | 56 | 10,000 | 560 KB/s |
| Feedback | 68 | 50,000 | 3.4 MB/s |
| Multi-Command | 200 | 5,000 | 1.0 MB/s |
| Heartbeat | 32 | 100 | 3.2 KB/s |

---

## 13. Security Considerations

### 13.1 Encryption

For sensitive applications, use TLS/DTLS encryption:

```typescript
interface SecurityConfig {
  encryption: 'none' | 'aes128' | 'aes256';
  authentication: 'none' | 'hmac' | 'rsa';
  certificate_path?: string;
  private_key_path?: string;
}
```

### 13.2 Authentication

```c
struct AuthPayload {
    uint8_t  auth_type;          // 0=token, 1=certificate, 2=hmac
    uint16_t auth_data_length;
    uint8_t  reserved;
    uint8_t  auth_data[256];     // Authentication data
};
```

---

## 14. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Author**: WIA AI Motor Control Working Group

---

弘益人間 - *Benefit All Humanity*
