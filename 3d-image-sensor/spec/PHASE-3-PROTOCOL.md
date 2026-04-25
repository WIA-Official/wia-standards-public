# WIA-SEMI-013 Phase 3: Protocol Specification

**Version:** 1.0  
**Date:** 2025-01-15  
**Status:** Published  

## Overview

Phase 3 defines communication protocols for networked 3D sensing systems, including real-time streaming, synchronization, and security mechanisms.

## 1. Point Cloud Streaming Protocol (WIA-PC-Stream)

### Protocol Stack

- Transport: UDP with optional reliability layer
- Serialization: Protocol Buffers v3
- Compression: LZ4 (fast) or Zstandard (high ratio)
- Encryption: AES-256-GCM (optional)

### Message Format

**Stream Header:**
```protobuf
message StreamHeader {
  string standard = 1;  // "WIA-SEMI-013"
  string version = 2;   // "1.0"
  string sensor_id = 3;
  uint64 stream_id = 4;
  StreamConfig config = 5;
}

message StreamConfig {
  uint32 width = 1;
  uint32 height = 2;
  float frame_rate = 3;
  Encoding encoding = 4;
  Compression compression = 5;
}
```

**Frame Packet:**
```protobuf
message FramePacket {
  uint64 sequence_number = 1;
  uint64 timestamp_us = 2;
  FrameType type = 3;
  bytes data = 4;
  optional bytes confidence = 5;
  uint32 crc32 = 6;
}

enum FrameType {
  DEPTH = 0;
  RGB = 1;
  INFRARED = 2;
  CONFIDENCE = 3;
  POINT_CLOUD = 4;
}
```

### QoS Parameters

**Latency Modes:**
- `ultra_low`: <10ms latency, minimal buffering, lossy OK
- `low`: <50ms latency, small buffer, packet recovery
- `balanced`: <100ms latency, moderate buffer, full recovery
- `reliable`: <500ms latency, large buffer, guaranteed delivery

**Bandwidth Management:**
```json
{
  "max_bitrate_mbps": 100,
  "adaptive": true,
  "quality_levels": [
    {"bitrate": 10, "resolution": "320x240", "fps": 15},
    {"bitrate": 30, "resolution": "640x480", "fps": 30},
    {"bitrate": 100, "resolution": "1280x960", "fps": 60}
  ]
}
```

## 2. Depth Streaming Protocol (WIA-Depth-Stream)

### Differential Encoding

For temporal compression, stream delta frames relative to keyframes.

**Keyframe (I-Frame):** Full depth map every N frames
**Delta Frame (P-Frame):** Difference from previous frame

```protobuf
message DepthFrame {
  FrameType frame_type = 1;  // KEYFRAME or DELTA
  uint64 reference_frame = 2;  // For delta frames
  
  oneof encoding {
    RawDepth raw = 3;
    CompressedDepth compressed = 4;
    DeltaDepth delta = 5;
  }
}
```

### Region of Interest (ROI)

Stream only changed/relevant regions:

```protobuf
message ROIConfig {
  repeated Rectangle regions = 1;
  bool adaptive = 2;  // Auto-detect motion regions
  float change_threshold = 3;  // Pixel depth change threshold
}

message Rectangle {
  uint32 x = 1;
  uint32 y = 2;
  uint32 width = 3;
  uint32 height = 4;
}
```

## 3. Multi-Sensor Synchronization

### Hardware Synchronization

**Master-Slave Configuration:**
```json
{
  "sync_mode": "hardware",
  "master_sensor": "sensor-001",
  "slave_sensors": ["sensor-002", "sensor-003"],
  "trigger_mode": "simultaneous",
  "trigger_rate_hz": 30
}
```

**Synchronization Signal:**
- GPIO trigger line
- Rising edge triggers frame capture
- Maximum jitter: <100μs between sensors

### Software Synchronization

**NTP-Based Timestamping:**
```protobuf
message SyncedFrame {
  string sensor_id = 1;
  uint64 ntp_timestamp_ns = 2;  // Nanoseconds since epoch
  uint32 frame_sequence = 3;
  bytes frame_data = 4;
}
```

**Synchronization Accuracy:**
- LAN: <1ms synchronization error
- WAN: <10ms synchronization error
- Requires NTP server or PTP (Precision Time Protocol)

### Frame Alignment

Match frames from different sensors by timestamp:

```python
def align_frames(frames, max_time_diff_ms=30):
    """Align frames from multiple sensors."""
    groups = []
    for frame in frames:
        matched = False
        for group in groups:
            if abs(frame.timestamp - group[0].timestamp) < max_time_diff_ms:
                group.append(frame)
                matched = True
                break
        if not matched:
            groups.append([frame])
    return [g for g in groups if len(g) == expected_sensor_count]
```

## 4. Security Protocols

### Authentication

**Device Authentication:**
```protobuf
message AuthRequest {
  string device_id = 1;
  string auth_token = 2;  // JWT or API key
  bytes signature = 3;    // ECDSA signature
}

message AuthResponse {
  bool authenticated = 1;
  string session_token = 2;
  uint64 expires_at = 3;
}
```

**JWT Claims:**
```json
{
  "iss": "wia-authority",
  "sub": "sensor-001",
  "exp": 1735740845,
  "permissions": ["stream", "config", "calibration"]
}
```

### Encryption

**Stream Encryption:**
- Algorithm: AES-256-GCM
- Key exchange: ECDH (Elliptic Curve Diffie-Hellman)
- Rekeying: Every 1GB or 1 hour

**TLS for REST API:**
- TLS 1.3 minimum
- Certificate pinning for trusted sensors
- Perfect forward secrecy required

### Secure Boot

For biometric and safety-critical applications:

```json
{
  "secure_boot": {
    "enabled": true,
    "chain_of_trust": [
      "bootloader_hash_sha256",
      "firmware_hash_sha256",
      "config_hash_sha256"
    ],
    "attestation": "tpm_2.0"
  }
}
```

## 5. Error Handling and Recovery

### Packet Loss Recovery

**Forward Error Correction (FEC):**
- Reed-Solomon coding: Add 20% redundancy
- Recover up to 10% packet loss without retransmission

**Selective Retransmission:**
```protobuf
message NACKMessage {
  uint64 stream_id = 1;
  repeated uint64 missing_sequences = 2;
  uint64 max_retries = 3;
}
```

### Connection Recovery

**Reconnection Strategy:**
1. Detect disconnection (missed heartbeats)
2. Attempt reconnection with exponential backoff
3. Resume stream from last acknowledged frame
4. Resynchronize timestamps

```json
{
  "reconnection_policy": {
    "initial_delay_ms": 100,
    "max_delay_ms": 30000,
    "backoff_multiplier": 2.0,
    "max_attempts": 10
  }
}
```

### Frame Drops

**Adaptive Frame Rate:**
When processing can't keep up, drop frames intelligently:

```python
class FrameDropStrategy:
    def should_drop(self, frame, processing_latency):
        if processing_latency > frame.target_latency * 1.5:
            # Drop P-frames, keep I-frames
            return frame.type == FrameType.DELTA
        return False
```

## 6. Protocol Compliance Testing

### Interoperability Tests

**Mandatory Tests:**
1. Stream establishment and teardown
2. Frame transmission and reception
3. Timestamp synchronization accuracy
4. Packet loss recovery (simulate 5% loss)
5. Authentication and encryption
6. QoS parameter negotiation

**Performance Benchmarks:**
| Metric | Target |
|--------|--------|
| Stream start latency | <500ms |
| Frame latency (ultra_low) | <10ms |
| Throughput | 100+ Mbps |
| Packet loss tolerance | 10% with FEC |
| Sync accuracy (HW) | <100μs |
| Sync accuracy (SW) | <10ms |

### Compliance Levels

**Level 1 (Basic):**
- Single sensor streaming
- Software synchronization
- No encryption required

**Level 2 (Standard):**
- Multi-sensor synchronization
- TLS encryption
- QoS support

**Level 3 (Advanced):**
- Hardware synchronization
- End-to-end encryption
- Real-time FEC

## 7. Protocol Standards Alignment

### 7.1 Real-Time Streaming Reference Frameworks

The WIA-PC-Stream and WIA-Depth-Stream protocols draw their wire-level designs from established IETF and ISO standards rather than redefining solved problems.

| Layer | Reference | Role in WIA-SEMI-013 |
|-------|-----------|----------------------|
| Real-Time Transport | RFC 3550 (RTP) | Sequence numbers, timestamps, jitter handling |
| RTP Control | RFC 3551 (RTP Profiles) | Receiver/Sender reports |
| Secure RTP | RFC 3711 (SRTP) | Authenticated, encrypted media frames |
| DTLS for SRTP keying | RFC 5764 (DTLS-SRTP) | Sensor key exchange |
| QUIC (alternative) | RFC 9000 (QUIC) | Loss-tolerant low-latency depth streams |
| HTTP/3 over QUIC | RFC 9114 | Browser-side streaming for simulators |

When operating in RTP mode, sensors register payload types in the dynamic range (96–127) and announce them via SDP (RFC 8866) using the media type `application/wia-depth`.

### 7.2 Industrial Network Compatibility

For factory and machine-vision deployments where sensors share a network with PLCs and motion controllers, the following industrial profiles are recognized:

- **IEC 61784-2** Real-time Ethernet profiles, with EtherCAT (IEC 61158-3-12) or PROFINET IRT (IEC 61158-3-10) for ≤1 ms cycle determinism.
- **IEC 62439-3** Parallel Redundancy Protocol (PRP) and High-Availability Seamless Redundancy (HSR) for fault-tolerant cells.
- **IEEE 802.1 TSN (Time-Sensitive Networking)** for converged IT/OT links carrying both sensor frames and motion commands.

A sensor compliant with the industrial profile reports its supported timing classes in its capability descriptor.

### 7.3 Network Security

The encryption stack composes:

| Concern | Reference |
|---------|-----------|
| Transport encryption (TCP) | RFC 8446 (TLS 1.3) |
| Transport encryption (UDP) | RFC 9147 (DTLS 1.3) |
| Cipher suites | RFC 8446 §B.4 (mandatory: AES-128-GCM-SHA256) |
| Forward secrecy | RFC 7919 (DH groups) and X25519 per RFC 7748 |
| Certificate format | RFC 5280 (X.509 v3) with SubjectAltName for sensor IDs |
| OCSP stapling | RFC 6066 §8 |
| Network segmentation guidance | ISO/IEC 27033-2:2012 |

For sensors deployed on a wide-area mesh (e.g., autonomous fleets), peer authentication uses EAP-TLS per RFC 5216 on top of IEEE 802.1X.

### 7.4 Time Synchronization at the Protocol Layer

Two synchronization regimes are recognized:

- **Sub-microsecond regime** — IEEE 1588-2019 PTPv2 with hardware timestamping; required for multi-camera reconstruction with sensors >1 m apart and resolutions ≥1 megapixel.
- **Millisecond regime** — RFC 5905 NTPv4 with chrony or ntpd; acceptable for single-sensor or temporally redundant applications (e.g., security cameras).

Frame timestamps in the streaming protocol carry the active synchronization regime in the header `time_source` field, taking values `ptp_v2`, `ntp_v4`, or `none`.

### 7.5 Reliability and Forward Error Correction

The FEC mechanism described in §5 implements Reed-Solomon coding per ISO/IEC 18004:2015 (the same coding family used in 2D barcodes), parameterized as RS(255, 223) for the default 12.5% redundancy mode and RS(255, 191) for the high-loss 25% redundancy mode. Implementations MAY substitute a Raptor or RaptorQ code per RFC 6330 for very-high-loss links.

### 7.6 Multicast and Group Communication

For one-to-many depth streams (e.g., a calibration server pushing sensor configuration), IPv4 multicast follows RFC 1112 and IPv6 multicast follows RFC 4291. SSM (Source-Specific Multicast) per RFC 4607 is the preferred mode.

### 7.7 Conformance

An implementation is conformant with Phase 3 when:

1. Either the WIA-native protocol or RTP profile is offered, and the offered profile passes the corresponding interoperability test suite.
2. TLS 1.3 (or DTLS 1.3 for UDP) is supported, and TLS 1.2 is the minimum offered ciphersuite version.
3. Time synchronization regime is declared per §7.4.
4. FEC mode and Reed-Solomon parameters are declared in the stream header.

## 8. Implementation Notes

### 8.1 Reference test vectors

Conformance tests reuse public test vectors where available:

- TLS 1.3 handshakes per RFC 8448.
- DTLS 1.3 records per RFC 9147 appendix examples.
- RTP packetisation conforms to RFC 3550 §5.1 fixed header semantics.

### 8.2 Diagnostic logging

All security-relevant events (handshake failures, certificate rejections, replay-window violations) are emitted in structured form per RFC 5424 syslog severity classes, and SHOULD include the `instance` URI from the matching RFC 9457 problem-details object so an end-to-end trace links control-plane errors with data-plane drops.

### 8.3 Latency budgets

For Level 3 advanced compliance, the implementation publishes its measured latency budget across capture, encoding, transport, decoding, and rendering. The reference deployment achieves the budget on commodity hardware when each stage is measured with the timers documented in `simulator/index.html`.

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**  
**弘益人間 · Benefit All Humanity**
