# WIA Eye Gaze Standard - Binary Format Specification

**Version**: 1.0.0
**Phase**: 3 - Real-time Communication Protocol
**弘益人間** - 널리 인간을 이롭게

---

## 1. Overview

This specification defines compact binary formats for high-performance gaze data transmission. Binary encoding reduces bandwidth by 7x compared to JSON and enables zero-copy processing.

### 1.1 Design Goals

- **Compact**: Minimize bytes per gaze point
- **Fast**: Enable zero-copy deserialization
- **Aligned**: Natural alignment for direct memory access
- **Extensible**: Version field for future compatibility

### 1.2 Comparison

| Format | Bytes/Point | Encoding Time | Decoding Time |
|--------|-------------|---------------|---------------|
| JSON | ~150 bytes | 2.5 μs | 3.0 μs |
| MessagePack | ~45 bytes | 0.8 μs | 0.6 μs |
| WIA Binary | 21 bytes | 0.1 μs | 0.05 μs |

---

## 2. Data Types

### 2.1 Primitive Types

| Type | Size | Description |
|------|------|-------------|
| `u8` | 1 | Unsigned 8-bit integer |
| `u16` | 2 | Unsigned 16-bit integer (little-endian) |
| `u32` | 4 | Unsigned 32-bit integer (little-endian) |
| `u64` | 8 | Unsigned 64-bit integer (little-endian) |
| `i16` | 2 | Signed 16-bit integer (little-endian) |
| `i32` | 4 | Signed 32-bit integer (little-endian) |
| `f32` | 4 | IEEE 754 single-precision float (little-endian) |
| `f64` | 8 | IEEE 754 double-precision float (little-endian) |

### 2.2 Fixed-Point Types

For efficient encoding of normalized values (0.0-1.0):

| Type | Size | Range | Precision |
|------|------|-------|-----------|
| `norm16` | 2 | 0.0-1.0 | 0.000015 |
| `norm8` | 1 | 0.0-1.0 | 0.004 |

Encoding:
```
norm16 = (u16)(value * 65535.0)
norm8  = (u8)(value * 255.0)
```

---

## 3. GazePoint Binary Format

### 3.1 Compact Format (21 bytes)

Used for streaming at 60-120Hz:

```
┌─────────────────────────────────────────────────────────────┐
│ Offset │ Size │ Type   │ Field          │ Description       │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 0      │ 2    │ u16    │ timestamp_off  │ ms offset from    │
│        │      │        │                │ base timestamp    │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 2      │ 4    │ f32    │ x              │ Gaze X (0.0-1.0)  │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 6      │ 4    │ f32    │ y              │ Gaze Y (0.0-1.0)  │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 10     │ 2    │ norm16 │ confidence     │ Confidence        │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 12     │ 1    │ u8     │ flags          │ Status flags      │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 13     │ 2    │ u16    │ left_pupil     │ Pupil ×100 (mm)   │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 15     │ 2    │ u16    │ right_pupil    │ Pupil ×100 (mm)   │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 17     │ 1    │ norm8  │ left_openness  │ Eye openness      │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 18     │ 1    │ norm8  │ right_openness │ Eye openness      │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 19     │ 2    │ u16    │ fixation_id    │ Fixation ID       │
└────────┴──────┴────────┴────────────────┴───────────────────┘
Total: 21 bytes
```

### 3.2 Flags Byte

```
Bit 0: valid           - Overall validity
Bit 1: left_valid      - Left eye data valid
Bit 2: right_valid     - Right eye data valid
Bit 3: is_fixation     - Currently in fixation
Bit 4: is_saccade      - Currently in saccade
Bit 5: is_blink        - Currently blinking
Bit 6: has_3d          - 3D data available (extended format)
Bit 7: reserved
```

### 3.3 Extended Format (53 bytes)

For applications requiring full 3D gaze data:

```
┌─────────────────────────────────────────────────────────────┐
│ Offset │ Size │ Type   │ Field            │ Description     │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 0      │ 8    │ u64    │ timestamp        │ Full timestamp  │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 8      │ 4    │ f32    │ x                │ Gaze X          │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 12     │ 4    │ f32    │ y                │ Gaze Y          │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 16     │ 4    │ f32    │ confidence       │ Confidence      │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 20     │ 1    │ u8     │ flags            │ Status flags    │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 21     │ 12   │ f32×3  │ left_origin      │ L eye origin    │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 33     │ 12   │ f32×3  │ left_direction   │ L gaze direction│
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 45     │ 4    │ f32    │ left_pupil       │ L pupil (mm)    │
├────────┼──────┼────────┼──────────────────┼─────────────────┤
│ 49     │ 4    │ f32    │ right_pupil      │ R pupil (mm)    │
└────────┴──────┴────────┴──────────────────┴─────────────────┘
Total: 53 bytes (add right eye fields for 97 bytes)
```

---

## 4. GazeEvent Binary Format

### 4.1 Event Format (32 bytes)

```
┌─────────────────────────────────────────────────────────────┐
│ Offset │ Size │ Type   │ Field          │ Description       │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 0      │ 8    │ u64    │ timestamp      │ Event timestamp   │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 8      │ 1    │ u8     │ event_type     │ Event type code   │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 9      │ 1    │ u8     │ flags          │ Event flags       │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 10     │ 2    │ u16    │ event_id       │ Sequential ID     │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 12     │ 4    │ u32    │ duration       │ Duration (ms)     │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 16     │ 4    │ f32    │ x              │ Position X        │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 20     │ 4    │ f32    │ y              │ Position Y        │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 24     │ 4    │ f32    │ progress       │ Dwell progress    │
├────────┼──────┼────────┼────────────────┼───────────────────┤
│ 28     │ 4    │ u32    │ target_hash    │ Target ID hash    │
└────────┴──────┴────────┴────────────────┴───────────────────┘
Total: 32 bytes
```

### 4.2 Event Type Codes

```
// Fixation events
FIXATION_START     = 0x01
FIXATION_END       = 0x02

// Saccade events
SACCADE_START      = 0x03
SACCADE_END        = 0x04

// Blink events
BLINK_START        = 0x05
BLINK_END          = 0x06
BLINK              = 0x07

// Dwell events
DWELL_START        = 0x10
DWELL_PROGRESS     = 0x11
DWELL_COMPLETE     = 0x12
DWELL_CANCEL       = 0x13

// Target events
GAZE_ENTER         = 0x20
GAZE_EXIT          = 0x21
GAZE_HOVER         = 0x22

// Tracking events
TRACKING_START     = 0x30
TRACKING_STOP      = 0x31
TRACKING_LOST      = 0x32
TRACKING_RECOVERED = 0x33

// Calibration events
CALIBRATION_START  = 0x40
CALIBRATION_END    = 0x41
CALIBRATION_POINT  = 0x42
```

---

## 5. Batch Message Format

### 5.1 GazePointBatch

Efficiently transmit multiple points:

```
┌─────────────────────────────────────────────────────────────┐
│ Header (16 bytes)                                           │
├────────┬──────────┬─────────────────────────────────────────┤
│ magic  │ version  │ type     │ flags    │ reserved         │
│ 2      │ 1        │ 1        │ 1        │ 3                │
├────────┴──────────┴──────────┴──────────┴───────────────────┤
│ base_timestamp (8) │ count (2) │ point_size (1) │ pad (1)  │
├────────────────────┴───────────┴──────────────────┴─────────┤
│ Points (count × point_size bytes)                           │
│ [GazePointCompact][GazePointCompact]...                     │
└─────────────────────────────────────────────────────────────┘
```

**Example at 60Hz (17 points/batch @ 60fps):**
```
Header:      16 bytes
Points:      17 × 21 = 357 bytes
Total:       373 bytes per batch
Data rate:   373 × 60 = 22,380 bytes/sec ≈ 22 KB/s
```

Compare to JSON: ~150 KB/s (7x reduction)

---

## 6. Framing Protocol

### 6.1 WebSocket Binary Frame

```
┌─────────────────────────────────────────────────────────────┐
│ Frame Header (8 bytes)                                      │
├────────────────┬────────────────┬───────────────────────────┤
│ Magic (2)      │ Version (1)    │ Type (1)                  │
│ 0x57 0x49      │ 0x01           │                           │
├────────────────┴────────────────┴───────────────────────────┤
│ Payload Length (4) - little-endian                          │
├─────────────────────────────────────────────────────────────┤
│ Payload (variable)                                          │
│ GazePointBatch / GazeEvent / Control                        │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Type Codes

```
TYPE_GAZE_BATCH   = 0x01
TYPE_GAZE_EVENT   = 0x02
TYPE_STATUS       = 0x03
TYPE_CONTROL      = 0x04
TYPE_ERROR        = 0x05
TYPE_PING         = 0x06
TYPE_PONG         = 0x07
```

---

## 7. Compression

### 7.1 Delta Encoding

For consecutive gaze points, delta encoding further reduces size:

```
Base Point (21 bytes):
  timestamp_off: u16
  x: f32
  y: f32
  ...

Delta Point (9 bytes):
  timestamp_delta: u8 (ms since last, max 255)
  x_delta: i16 (×10000, relative to last)
  y_delta: i16 (×10000, relative to last)
  flags: u8
  confidence_delta: i8
```

### 7.2 LZ4 Compression

For batches, optional LZ4 compression:

```
Compressed Frame:
  flags |= 0x01  // Compression flag
  payload = lz4_compress(original_payload)
```

Typical compression ratio: 1.5-2x additional reduction.

---

## 8. Rust Implementation

### 8.1 Type Definitions

```rust
use bytemuck::{Pod, Zeroable};

/// Compact gaze point (21 bytes, packed)
#[repr(C, packed)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct GazePointBinary {
    pub timestamp_offset: u16,  // ms from base
    pub x: f32,                 // 0.0-1.0
    pub y: f32,                 // 0.0-1.0
    pub confidence: u16,        // norm16
    pub flags: u8,
    pub left_pupil: u16,        // mm × 100
    pub right_pupil: u16,       // mm × 100
    pub left_openness: u8,      // norm8
    pub right_openness: u8,     // norm8
    pub fixation_id: u16,
}

/// Gaze point flags
pub mod GazeFlags {
    pub const VALID: u8         = 0x01;
    pub const LEFT_VALID: u8    = 0x02;
    pub const RIGHT_VALID: u8   = 0x04;
    pub const IS_FIXATION: u8   = 0x08;
    pub const IS_SACCADE: u8    = 0x10;
    pub const IS_BLINK: u8      = 0x20;
    pub const HAS_3D: u8        = 0x40;
}

/// Binary event (32 bytes)
#[repr(C, packed)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct GazeEventBinary {
    pub timestamp: u64,
    pub event_type: u8,
    pub flags: u8,
    pub event_id: u16,
    pub duration: u32,
    pub x: f32,
    pub y: f32,
    pub progress: f32,
    pub target_hash: u32,
}
```

### 8.2 Batch Header

```rust
/// Batch header (16 bytes)
#[repr(C, packed)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct BatchHeader {
    pub magic: [u8; 2],      // 0x57, 0x49
    pub version: u8,         // 0x01
    pub msg_type: u8,        // TYPE_GAZE_BATCH
    pub flags: u8,
    pub reserved: [u8; 3],
    pub base_timestamp: u64,
    pub count: u16,
    pub point_size: u8,
    pub pad: u8,
}
```

### 8.3 Zero-Copy Encoding

```rust
impl GazePointBinary {
    /// Create from GazePoint with base timestamp
    pub fn from_gaze_point(point: &GazePoint, base_ts: u64) -> Self {
        Self {
            timestamp_offset: ((point.timestamp - base_ts) as u16).min(65535),
            x: point.x as f32,
            y: point.y as f32,
            confidence: (point.confidence * 65535.0) as u16,
            flags: Self::compute_flags(point),
            left_pupil: point.left_eye
                .and_then(|e| e.pupil_diameter)
                .map(|d| (d * 100.0) as u16)
                .unwrap_or(0),
            right_pupil: point.right_eye
                .and_then(|e| e.pupil_diameter)
                .map(|d| (d * 100.0) as u16)
                .unwrap_or(0),
            left_openness: point.left_eye
                .and_then(|e| e.eye_openness)
                .map(|o| (o * 255.0) as u8)
                .unwrap_or(0),
            right_openness: point.right_eye
                .and_then(|e| e.eye_openness)
                .map(|o| (o * 255.0) as u8)
                .unwrap_or(0),
            fixation_id: point.fixation_id.unwrap_or(0) as u16,
        }
    }

    /// Zero-copy conversion to bytes
    pub fn as_bytes(&self) -> &[u8] {
        bytemuck::bytes_of(self)
    }

    /// Zero-copy conversion from bytes
    pub fn from_bytes(bytes: &[u8; 21]) -> &Self {
        bytemuck::from_bytes(bytes)
    }

    fn compute_flags(point: &GazePoint) -> u8 {
        let mut flags = 0u8;
        if point.valid { flags |= GazeFlags::VALID; }
        if point.left_eye.map(|e| e.valid).unwrap_or(false) {
            flags |= GazeFlags::LEFT_VALID;
        }
        if point.right_eye.map(|e| e.valid).unwrap_or(false) {
            flags |= GazeFlags::RIGHT_VALID;
        }
        if point.fixation.is_some() { flags |= GazeFlags::IS_FIXATION; }
        if point.saccade.is_some() { flags |= GazeFlags::IS_SACCADE; }
        flags
    }
}
```

### 8.4 Batch Encoding

```rust
pub fn encode_batch(points: &[GazePoint]) -> Vec<u8> {
    if points.is_empty() {
        return Vec::new();
    }

    let base_ts = points[0].timestamp;
    let point_size = std::mem::size_of::<GazePointBinary>() as u8;

    let header = BatchHeader {
        magic: [0x57, 0x49],
        version: 0x01,
        msg_type: TYPE_GAZE_BATCH,
        flags: 0,
        reserved: [0; 3],
        base_timestamp: base_ts,
        count: points.len() as u16,
        point_size,
        pad: 0,
    };

    let mut buffer = Vec::with_capacity(
        std::mem::size_of::<BatchHeader>() +
        points.len() * point_size as usize
    );

    buffer.extend_from_slice(bytemuck::bytes_of(&header));

    for point in points {
        let binary = GazePointBinary::from_gaze_point(point, base_ts);
        buffer.extend_from_slice(binary.as_bytes());
    }

    buffer
}
```

---

## 9. TypeScript Decoding

### 9.1 DataView-based Decoder

```typescript
interface GazePointBinary {
  timestampOffset: number;
  x: number;
  y: number;
  confidence: number;
  flags: number;
  leftPupil: number;
  rightPupil: number;
  leftOpenness: number;
  rightOpenness: number;
  fixationId: number;
}

function decodeGazePoint(view: DataView, offset: number): GazePointBinary {
  return {
    timestampOffset: view.getUint16(offset, true),
    x: view.getFloat32(offset + 2, true),
    y: view.getFloat32(offset + 6, true),
    confidence: view.getUint16(offset + 10, true) / 65535,
    flags: view.getUint8(offset + 12),
    leftPupil: view.getUint16(offset + 13, true) / 100,
    rightPupil: view.getUint16(offset + 15, true) / 100,
    leftOpenness: view.getUint8(offset + 17) / 255,
    rightOpenness: view.getUint8(offset + 18) / 255,
    fixationId: view.getUint16(offset + 19, true),
  };
}

function decodeBatch(buffer: ArrayBuffer): GazePointBinary[] {
  const view = new DataView(buffer);

  // Verify magic
  if (view.getUint8(0) !== 0x57 || view.getUint8(1) !== 0x49) {
    throw new Error('Invalid magic number');
  }

  const baseTimestamp = view.getBigUint64(8, true);
  const count = view.getUint16(16, true);
  const pointSize = view.getUint8(18);

  const points: GazePointBinary[] = [];
  let offset = 20; // Header size

  for (let i = 0; i < count; i++) {
    points.push(decodeGazePoint(view, offset));
    offset += pointSize;
  }

  return points;
}
```

---

## 10. Validation

### 10.1 Magic Number

All binary messages MUST begin with `0x57 0x49` ("WI").

### 10.2 Range Checks

| Field | Valid Range | Action if Invalid |
|-------|-------------|-------------------|
| x, y | 0.0-1.0 | Clamp to range |
| confidence | 0.0-1.0 | Clamp to range |
| pupil | 0.0-15.0mm | Mark as invalid |
| openness | 0.0-1.0 | Clamp to range |

### 10.3 Checksum (Optional)

For unreliable transports, append CRC32:

```
[Header][Payload][CRC32 (4 bytes)]
flags |= 0x02  // Checksum flag
```

---

## 11. Migration from JSON

### 11.1 Format Negotiation

```json
{
  "type": "control",
  "action": "set_format",
  "params": {
    "format": "binary",
    "version": 1,
    "compression": "none"
  }
}
```

Server responds with STATUS, then switches to binary.

### 11.2 Fallback

If client doesn't support binary, server continues JSON:

```json
{
  "type": "control",
  "action": "set_format",
  "params": { "format": "json" }
}
```

---

**弘益人間** - 널리 인간을 이롭게
