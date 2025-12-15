# WIA EMG Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines the standard data format for Electromyography (EMG) signals used in myoelectric prosthetic control systems. The format ensures interoperability between different sensor hardware, processing software, and prosthetic devices.

## 2. EMG Data Frame Structure

### 2.1 Top-Level Structure

```json
{
  "version": "1.0.0",
  "metadata": { ... },
  "channels": [ ... ],
  "startTime": 1705312800000,
  "duration": 1000
}
```

### 2.2 Metadata Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sampleRate` | integer | Yes | Sampling frequency in Hz (1000-2000 recommended) |
| `channelCount` | integer | Yes | Number of EMG channels (2-8 typical) |
| `resolution` | integer | Yes | ADC resolution in bits (10-24) |
| `gain` | number | Yes | Amplification gain factor |
| `referenceType` | string | Yes | Electrode configuration type |
| `deviceId` | string | No | Unique identifier for the sensor device |
| `firmwareVersion` | string | No | Device firmware version |

#### 2.2.1 Reference Types

| Type | Description | Use Case |
|------|-------------|----------|
| `monopolar` | Single electrode with remote reference | General purpose, susceptible to noise |
| `bipolar` | Two electrodes over same muscle | Standard clinical use |
| `differential` | Two electrodes with common-mode rejection | High noise environments |

### 2.3 Channel Object

```json
{
  "id": 0,
  "name": "flexor_carpi_radialis",
  "placement": { ... },
  "samples": [0.012, -0.008, 0.015, ...],
  "unit": "mV",
  "calibration": { ... }
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | integer | Yes | Zero-indexed channel identifier |
| `name` | string | Yes | Muscle name (snake_case) |
| `placement` | object | Yes | Electrode placement information |
| `samples` | array | Yes | Raw EMG samples as 32-bit floats |
| `unit` | string | Yes | Measurement unit (typically "mV") |
| `calibration` | object | No | Channel calibration data |

### 2.4 Electrode Placement Object

```json
{
  "muscle": "flexor_carpi_radialis",
  "location": {
    "x": 12.5,
    "y": 0.0,
    "circumference": 45.0
  },
  "anatomicalLandmark": "medial_epicondyle",
  "orientation": "parallel"
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `muscle` | string | Yes | Target muscle name |
| `location.x` | number | Yes | Distance from elbow in cm |
| `location.y` | number | Yes | Offset from muscle center in cm |
| `location.circumference` | number | Yes | Angular position on limb (degrees) |
| `anatomicalLandmark` | string | No | Reference anatomical point |
| `orientation` | string | No | Electrode orientation relative to muscle fibers |

## 3. Supported Muscle Names

### 3.1 Forearm Flexors

| Identifier | Full Name | Function |
|------------|-----------|----------|
| `flexor_carpi_radialis` | Flexor Carpi Radialis | Wrist flexion, radial deviation |
| `flexor_carpi_ulnaris` | Flexor Carpi Ulnaris | Wrist flexion, ulnar deviation |
| `flexor_digitorum_superficialis` | Flexor Digitorum Superficialis | Finger flexion (PIP joints) |
| `flexor_digitorum_profundus` | Flexor Digitorum Profundus | Finger flexion (DIP joints) |
| `flexor_pollicis_longus` | Flexor Pollicis Longus | Thumb flexion |
| `pronator_teres` | Pronator Teres | Forearm pronation |

### 3.2 Forearm Extensors

| Identifier | Full Name | Function |
|------------|-----------|----------|
| `extensor_carpi_radialis_longus` | Extensor Carpi Radialis Longus | Wrist extension, radial deviation |
| `extensor_carpi_radialis_brevis` | Extensor Carpi Radialis Brevis | Wrist extension |
| `extensor_carpi_ulnaris` | Extensor Carpi Ulnaris | Wrist extension, ulnar deviation |
| `extensor_digitorum` | Extensor Digitorum | Finger extension |
| `extensor_pollicis_longus` | Extensor Pollicis Longus | Thumb extension |
| `supinator` | Supinator | Forearm supination |

## 4. Data Transmission Formats

### 4.1 JSON Format (Reference)

```json
{
  "version": "1.0.0",
  "metadata": {
    "sampleRate": 1000,
    "channelCount": 4,
    "resolution": 16,
    "gain": 1000,
    "referenceType": "bipolar"
  },
  "channels": [
    {
      "id": 0,
      "name": "flexor_carpi_radialis",
      "placement": {
        "muscle": "flexor_carpi_radialis",
        "location": { "x": 10.0, "y": 0.0, "circumference": 45.0 }
      },
      "samples": [0.012, -0.008, 0.015],
      "unit": "mV"
    }
  ],
  "startTime": 1705312800000,
  "duration": 1000
}
```

### 4.2 Binary Format (Efficient)

For real-time applications, a compact binary format is supported:

```
Header (32 bytes):
  [0-3]   Magic: "WIA1" (4 bytes)
  [4-5]   Version: uint16
  [6-7]   Channel count: uint16
  [8-11]  Sample rate: uint32
  [12-15] Sample count per channel: uint32
  [16-19] Start time (Unix ms, high): uint32
  [20-23] Start time (Unix ms, low): uint32
  [24-27] Flags: uint32
  [28-31] Reserved: uint32

Channel Headers (16 bytes each):
  [0-3]   Channel ID: uint32
  [4-7]   Muscle code: uint32
  [8-11]  Calibration factor: float32
  [12-15] Offset: float32

Sample Data:
  Interleaved float32 samples: [ch0_s0, ch1_s0, ..., chN_s0, ch0_s1, ...]
```

### 4.3 Streaming Protocol

For continuous data transmission:

```
Packet Structure:
  [0-1]   Sequence number: uint16
  [2-5]   Timestamp delta (us): uint32
  [6-7]   Sample count: uint16
  [8-N]   Samples: float32[]
```

## 5. Quality Requirements

### 5.1 Signal Quality Indicators

| Metric | Acceptable Range | Description |
|--------|------------------|-------------|
| SNR | > 20 dB | Signal-to-noise ratio |
| Baseline drift | < 0.1 mV/s | DC offset change rate |
| 50/60 Hz noise | < -40 dB | Power line interference |
| Cross-talk | < -20 dB | Inter-channel interference |

### 5.2 Data Integrity

- All samples must be timestamped with microsecond precision
- Missing samples must be indicated with NaN values
- Maximum packet loss: 0.1% for real-time control

## 6. Conformance Levels

### Level 1: Basic
- JSON format support
- Minimum 2 channels
- 1000 Hz sample rate

### Level 2: Standard
- JSON and binary format support
- 4-8 channels
- 1000-2000 Hz sample rate
- MVC calibration support

### Level 3: Advanced
- All formats including streaming
- Real-time quality monitoring
- Adaptive gain control
- Impedance monitoring

## 7. References

- SENIAM (Surface EMG for Non-Invasive Assessment of Muscles) guidelines
- IEEE 11073 Health Device Communication standards
- IEC 60601-2-40 Medical electrical equipment for EMG

---

*WIA Myoelectric Standard - EMG Data Format v1.0.0*
