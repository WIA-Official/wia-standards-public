# WIA Myoelectric Standard — Phase 1: EMG Data Format Specification

## Version Information
- **Document Version**: 2.0.0
- **Last Updated**: 2026-04-25
- **Status**: Phase 1 — Complete
- **Standard**: WIA-MYO-DATA-001

---

## 1. Overview

This specification defines the standard data format for surface electromyography (sEMG) signals used in myoelectric prosthetic and exoskeleton control systems. The format ensures interoperability between sEMG sensor hardware (MyoWare 2.0, Delsys Trigno IM, OttoBock electrodes), pattern recognition classifiers, and prosthetic terminal devices.

### 1.1 Scope

| Device Category | Representative Products | Classification |
|-----------------|------------------------|----------------|
| Transradial prosthesis | LUKE Arm, Hero Arm, Ottobock MyoHand | ISO 22523 — P-type partial-hand to wrist |
| Transhumeral prosthesis | Ottobock Michelangelo, BeBionic | ISO 22523 — E-type elbow disarticulation |
| Partial-hand prosthesis | Naked Prosthetics PIPDriver | ISO 22523 — Finger/partial-hand |
| Lower-limb myoelectric | Össur Proprio Foot, Ottobock Kenevo | ISO 22523 — K-type below/above knee |
| EMG-controlled exoskeleton | CYBERDYNE HAL, Ottobock C-Brace | ISO 13482 wearable |

### 1.2 Electrode Types

| Electrode | Channels | Sample Rate | Signal Range | Amplifier |
|-----------|----------|-------------|--------------|-----------|
| MyoWare 2.0 (SparkFun) | 1 per module | 1 kHz | 0–1000 mV | Integrated op-amp |
| Delsys Trigno IM | 16 (wireless) | 2 kHz | ±5 mV input | 1000x gain |
| Ottobock 13E200 | 2 differential | 2 kHz | ±2.5 mV | 60 dB CMRR |
| Coapt COAPT | 8 | 2 kHz | ±5 mV | Pattern recognition board |
| OYMotion GForce | 8 | 1 kHz | ±5 mV | BLE wireless |

---

## 2. EMG Data Frame Schema

### 2.1 Top-Level JSON Structure

```json
{
  "$schema": "https://standards.wia.global/myoelectric/schema/v2/emg-frame.json",
  "version": "2.0.0",
  "frame_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp_us": 1745535600123456,
  "device": {
    "id": "delsys-trigno-001",
    "manufacturer": "Delsys",
    "model": "Trigno IM",
    "num_channels": 16,
    "sample_rate_hz": 2000,
    "bit_depth": 16,
    "gain": 1000,
    "input_range_mv": 5.0
  },
  "channels": [
    {
      "index": 0,
      "label": "FCR",
      "muscle": "Flexor Carpi Radialis",
      "placement": "forearm_volar_proximal",
      "impedance_kohm": 3.2,
      "raw_uv": [12, -8, 24, -15, 31, -22],
      "rms_uv": 18.4,
      "mav_uv": 15.7,
      "snr_db": 28.3
    }
  ],
  "features": null,
  "classification": null,
  "quality": {
    "motion_artifact": false,
    "saturation": false,
    "lead_off": [false, false, false],
    "overall_quality": "good"
  }
}
```

### 2.2 Channel Schema

```typescript
interface EmgChannel {
  index: number;                // 0-based channel index
  label: string;                // muscle abbreviation (FCR, ECR, BB, TB, FDS…)
  muscle: string;               // full muscle name
  placement: string;            // electrode placement descriptor (SENIAM protocol)
  impedance_kohm?: number;      // electrode-skin impedance at 20 Hz
  raw_uv: number[];             // raw EMG samples in microvolts
  rms_uv: number;               // 200 ms RMS window in microvolts
  mav_uv: number;               // mean absolute value window
  wl_uv?: number;               // waveform length (Hudgins feature)
  zc?: number;                  // zero crossing count (Hudgins feature)
  ssc?: number;                 // slope sign change count (Hudgins feature)
  snr_db?: number;              // signal-to-noise ratio in dB
}
```

### 2.3 SENIAM Electrode Placement Protocol

SENIAM (Surface EMG for Non-Invasive Assessment of Muscles) defines standardised electrode placement locations for each target muscle. WIA-MYO-DATA-001 mandates SENIAM-compliant placement descriptors in the `placement` field:

| Placement Code | Muscle | SENIAM Position |
|----------------|--------|-----------------|
| `forearm_volar_proximal` | FCR | 1/3 of distance from medial epicondyle to radial styloid |
| `forearm_volar_middle` | FDS | Middle of forearm, between FCR and ECU |
| `forearm_dorsal_proximal` | ECR | 1/4 of distance lateral epicondyle to radial styloid |
| `forearm_dorsal_middle` | ECU | 2/3 of distance lateral epicondyle to ulnar styloid |
| `upper_arm_anterior` | BB | Anterior upper arm, 1/3 distance from antecubital fossa |
| `upper_arm_posterior` | TB | Posterior upper arm, midpoint of distance |

---

## 3. Feature Extraction

### 3.1 Hudgins TD4 Feature Set

The Hudgins TD4 feature set is the de-facto baseline for EMG pattern recognition. It defines four time-domain features per channel computed over a 200–256 ms analysis window:

```json
{
  "features": {
    "method": "hudgins_td4",
    "window_ms": 200,
    "overlap_ms": 100,
    "channels": 8,
    "feature_vector_length": 32,
    "per_channel": [
      {
        "channel": 0,
        "label": "FCR",
        "mav": 15.7,
        "zc": 8,
        "ssc": 6,
        "wl": 142.3
      }
    ],
    "concatenated_vector": [15.7, 8, 6, 142.3, 22.1, 5, 4, 98.7]
  }
}
```

Feature definitions:
- **MAV** (Mean Absolute Value): `(1/N) × Σ|xᵢ|` — signal magnitude proxy.
- **ZC** (Zero Crossing): count of `sign(xᵢ) ≠ sign(xᵢ₊₁)` — frequency content proxy.
- **SSC** (Slope Sign Change): count of slope direction changes — frequency spectrum.
- **WL** (Waveform Length): `Σ|xᵢ₊₁ - xᵢ|` — signal complexity / duration proxy.

### 3.2 Advanced Feature Sets

```json
{
  "features": {
    "method": "advanced_mixed",
    "window_ms": 200,
    "features_per_channel": {
      "ar4_coefficients": [0.42, -0.18, 0.09, -0.04],
      "rms": 18.4,
      "var": 338.5,
      "iemg": 3140.0,
      "mdf_hz": 124.3,
      "mpf_hz": 139.8,
      "hi_lo_ratio": 0.73,
      "log_detector": 2.84,
      "v_order_3": 42.1,
      "tm_4th_moment": 3.2e6
    }
  }
}
```

AR(4) autoregressive coefficients offer superior separability for multi-class gesture recognition compared to Hudgins TD4 in high-DOF prosthetic hands (>= 4 simultaneous degrees of freedom).

---

## 4. Classification Schema

### 4.1 Pattern Recognition Output

```json
{
  "classification": {
    "method": "lda",
    "model_version": "wia-myo-lda-v2.1",
    "window_ms": 200,
    "classes": [
      "rest", "open_hand", "close_hand", "pronate",
      "supinate", "wrist_flex", "wrist_ext", "tripod_pinch"
    ],
    "probabilities": [0.02, 0.01, 0.94, 0.01, 0.01, 0.01, 0.00, 0.00],
    "predicted_class": "close_hand",
    "predicted_class_index": 2,
    "confidence": 0.94,
    "rejection_threshold": 0.70,
    "rejected": false,
    "classifier_latency_ms": 3.2
  }
}
```

### 4.2 Supported Classifier Types

| Classifier | Field Value | Use Case |
|-----------|-------------|----------|
| Linear Discriminant Analysis | `lda` | Default; fast, low memory, clinical proven |
| Support Vector Machine | `svm_rbf` | Non-linear boundaries; high-accuracy lab setting |
| LSTM Recurrent Neural Network | `lstm` | Sequential gesture transitions; proportional control |
| Temporal Convolutional Network | `tcn` | Real-time streaming; Coapt-style continuous adaptation |
| Gaussian Mixture Model | `gmm` | Rejection zone modelling (rest class separation) |

---

## 5. Proportional Control Schema

Proportional myoelectric control maps sEMG signal amplitude directly to prosthesis velocity or grip force, without discrete classification:

```json
{
  "proportional_control": {
    "mode": "dual_site",
    "dof": 1,
    "channels": {
      "flexion": {"index": 0, "label": "FCR", "rms_uv": 45.2},
      "extension": {"index": 2, "label": "ECR", "rms_uv": 12.1}
    },
    "threshold_uv": {
      "dead_band_low": 8.0,
      "activation_low": 15.0,
      "activation_high": 25.0,
      "saturation": 120.0
    },
    "output": {
      "velocity_fraction": 0.62,
      "direction": "flexion",
      "raw_command": 0.62
    },
    "ramp_rate_per_ms": 0.02
  }
}
```

---

## 6. Session Metadata

```json
{
  "session": {
    "session_id": "wia-myo-session-2026-04-25-001",
    "patient_id": "patient-anonymised-001",
    "practitioner": "Prosthetist-KOR-001",
    "prosthetic_device": {
      "manufacturer": "Ottobock",
      "model": "bebionic",
      "iso_22523_class": "E1",
      "socket_type": "elevated_vacuum"
    },
    "amputation_level": "transradial",
    "dominance": "right",
    "days_post_amputation": 180,
    "prior_myoelectric_experience_months": 0,
    "session_type": "classifier_training",
    "start_time": "2026-04-25T09:00:00+09:00",
    "duration_s": 1200,
    "repetitions_per_class": 25,
    "gestures_trained": ["rest", "open_hand", "close_hand", "tripod_pinch",
                         "key_pinch", "pronate", "supinate", "wrist_flex",
                         "wrist_ext", "point_index"]
  }
}
```

---

## 7. Data Quality Indicators

| Field | Type | Description |
|-------|------|-------------|
| `motion_artifact` | boolean | True if RMS exceeds 3× baseline during rest |
| `saturation` | boolean | True if any sample equals ±ADC_MAX |
| `lead_off` | boolean[] | Per-channel electrode contact loss detection |
| `snr_db` | float | Signal-to-noise ratio; ≥20 dB acceptable; ≥28 dB optimal |
| `overall_quality` | enum | `good` / `fair` / `poor` / `rejected` |
| `impedance_kohm` | float | Electrode-skin impedance; ≤10 kΩ optimal; ≤50 kΩ acceptable |

---

## 8. WIA-MYO-DATA-001 Conformance Checklist

| Requirement | Reference | Mandatory |
|-------------|-----------|-----------|
| JSON schema v2 with `$schema` URI | WIA-MYO-DATA-001 §2 | Yes |
| SENIAM electrode placement codes | SENIAM §3 | Yes |
| Hudgins TD4 feature set minimum | WIA-MYO-DATA-001 §3.1 | Yes |
| ISO 22523 prosthetic classification field | ISO 22523:2006 §5 | Yes |
| Sample rate ≥ 2 kHz for pattern recognition | WIA-MYO-DATA-001 §2.2 | Yes |
| SNR quality indicator | WIA-MYO-DATA-001 §7 | Yes |
| Classifier latency field (target < 30 ms) | WIA-MYO-DATA-001 §4 | Yes |
| Session metadata with anonymised patient ID | WIA-PRIVACY §3 | Yes |
| Proportional control ramp_rate to prevent jerky motion | WIA-MYO-DATA-001 §5 | Recommended |

---

## 9. FHIR R5 Clinical Integration

Myoelectric prosthetic session data maps to HL7 FHIR R5 resources for integration with hospital EMR systems and prosthetic rehabilitation registries:

### 9.1 Device Resource

```json
{
  "resourceType": "Device",
  "id": "ottobock-bebionic-001",
  "status": "active",
  "manufacturer": "Ottobock",
  "modelNumber": "bebionic v4",
  "type": {"coding": [{"system": "http://snomed.info/sct",
                        "code": "449181000124109",
                        "display": "Myoelectric upper extremity prosthesis"}]},
  "udiCarrier": [{"deviceIdentifier": "00314001000003", "issuer": "GS1"}],
  "property": [
    {"type": {"text": "ISO_22523_class"}, "valueString": "E1"},
    {"type": {"text": "max_grip_force_n"}, "valueQuantity": {"value": 136, "unit": "N"}},
    {"type": {"text": "electrode_channels"}, "valueInteger": 2}
  ]
}
```

### 9.2 Observation Resource — Classifier Accuracy

```json
{
  "resourceType": "Observation",
  "status": "final",
  "code": {"coding": [{"system": "https://standards.wia.global/fhir/CodeSystem",
                        "code": "MYO-CLASSIFIER-ACC",
                        "display": "Myoelectric classifier accuracy"}]},
  "subject": {"reference": "Patient/patient-anonymised-001"},
  "device": {"reference": "Device/ottobock-bebionic-001"},
  "valueQuantity": {"value": 94.2, "unit": "%"},
  "component": [
    {"code": {"text": "training_gestures"}, "valueInteger": 10},
    {"code": {"text": "classifier_type"}, "valueString": "LDA"},
    {"code": {"text": "window_ms"}, "valueQuantity": {"value": 200, "unit": "ms"}}
  ]
}
```

---

## 10. Streaming and Real-Time Protocol

### 10.1 ROS 2 Message Types

WIA-MYO-DATA-001 defines custom ROS 2 message types for real-time EMG streaming at 2 kHz:

```
# wia_myo_msgs/msg/EmgFrame
std_msgs/Header header
uint8 num_channels
float32[] raw_uv                # length = num_channels × samples_per_frame
float32[] rms_uv                # length = num_channels
float32[] mav_uv                # length = num_channels
int32[] zc                      # zero crossing counts
int32[] ssc                     # slope sign change counts
float32[] wl_uv                 # waveform lengths
bool[] lead_off                 # electrode contact loss flags
string[] labels                 # muscle abbreviations
```

```
# wia_myo_msgs/msg/EmgClassification
std_msgs/Header header
string[] class_names
float32[] probabilities
string predicted_class
uint8 predicted_class_index
float32 confidence
bool rejected
float32 classifier_latency_ms
```

### 10.2 Streaming Pipeline

```
sEMG Electrodes (2 kHz)
  → USB/BLE Hardware Driver (e.g. delsys_ros2_driver)
    → /emg/raw (wia_myo_msgs/EmgFrame, BEST_EFFORT 2 kHz)
      → Feature Extractor Node (200 ms window, 50% overlap)
        → /emg/features (BEST_EFFORT 10 Hz)
          → Pattern Recognition Node (LDA / LSTM)
            → /emg/classification (wia_myo_msgs/EmgClassification, 10 Hz)
              → Prosthetic Controller Node
                → /prosthetic/command (RELIABLE, mapped to CAN/USB)
```

### 10.3 Latency Budget

End-to-end control latency for myoelectric prosthetic response:

| Stage | Target Latency |
|-------|---------------|
| ADC sampling + driver | 0.5 ms |
| Feature extraction (200 ms window) | 5 ms (overlap processing) |
| LDA classification | 3 ms |
| Prosthetic command transmission (USB/CAN) | 2 ms |
| Mechanical actuation start | 15–30 ms |
| **Total (EMG → motion onset)** | **~25–40 ms** |

Perceived natural delay threshold: 300 ms (Parker & Scott 1986); WIA target well within.

---

## 11. ISO 22523 Prosthetic Classification Reference

ISO 22523:2006 "External limb prostheses and external orthoses" defines the classification system used in WIA-MYO-DATA-001's `iso_22523_class` field:

| ISO 22523 Code | Level | Description |
|----------------|-------|-------------|
| P1 | Wrist disarticulation | Through-wrist amputation |
| P2 | Transradial | Below-elbow |
| P3 | Elbow disarticulation | Through-elbow |
| E1 | Transhumeral | Above-elbow |
| E2 | Shoulder disarticulation | Through-shoulder |
| E3 | Forequarter | Shoulder + clavicle disarticulation |
| K1 | Transtibial | Below-knee |
| K2 | Knee disarticulation | Through-knee |
| K3 | Transfemoral | Above-knee |

WIA-MYO-DATA-001 requires `iso_22523_class` in the session metadata to enable classification benchmarks to be properly stratified by amputation level, since classifier accuracy varies significantly with electrode placement options and residual limb length.
