# WIA BCI Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA BCI Data Format은 뇌-컴퓨터 인터페이스 데이터의 저장, 전송, 교환을 위한 통합 표준입니다. 이 표준은 다양한 BCI 기기와 플랫폼 간의 상호운용성을 보장하고, 연구 및 임상 데이터의 재현성을 높이기 위해 설계되었습니다.

### 1.2 Scope

- **In Scope**:
  - EEG (Electroencephalography) 데이터
  - EMG (Electromyography) 데이터
  - EOG (Electrooculography) 데이터
  - 이벤트/마커 데이터
  - 기기 메타데이터
  - 세션 정보

- **Out of Scope** (Phase 1):
  - 침습적 BCI 고밀도 데이터 (Phase 2+)
  - 실시간 스트리밍 프로토콜 (Phase 3)
  - AI/ML 파이프라인 (Phase 4)

### 1.3 Design Principles

1. **Interoperability**: 기존 표준(EDF+, BIDS, XDF)과 호환
2. **Extensibility**: 새로운 기기/모달리티 추가 용이
3. **Self-describing**: 메타데이터를 통한 자기 기술적 형식
4. **Efficiency**: 대용량 데이터 처리를 위한 최적화
5. **Accessibility**: JSON 기반의 인간 가독성

---

## 2. Data Structure

### 2.1 Top-Level Structure

```
wia-bci-recording/
├── recording.json          # Main recording metadata
├── device.json             # Device information
├── subject.json            # Subject/participant info (optional)
├── channels.json           # Channel definitions
├── events.json             # Event markers
├── data/
│   ├── eeg.bin             # Raw EEG data (binary)
│   ├── eeg.json            # EEG metadata
│   ├── emg.bin             # Raw EMG data (optional)
│   ├── emg.json            # EMG metadata (optional)
│   └── auxiliary.bin       # Auxiliary data (optional)
└── derivatives/            # Processed data (optional)
    ├── filtered/
    └── features/
```

### 2.2 File Naming Convention

```
{subject}_{session}_{task}_{modality}.{extension}

Examples:
- sub-001_ses-01_task-motor_eeg.bin
- sub-001_ses-01_task-motor_events.json
```

---

## 3. Recording Metadata

### 3.1 recording.json

```json
{
  "$schema": "https://wia.live/schemas/bci/recording.schema.json",
  "wia_version": "1.0.0",
  "format_version": "1.0.0",
  "recording_id": "rec-20250115-001",

  "recording_info": {
    "start_time": "2025-01-15T10:30:00.000Z",
    "end_time": "2025-01-15T11:30:00.000Z",
    "duration_seconds": 3600.0,
    "timezone": "Asia/Seoul"
  },

  "session": {
    "session_id": "ses-001",
    "session_type": "research",
    "paradigm": "motor_imagery",
    "description": "4-class motor imagery BCI experiment"
  },

  "data_files": {
    "eeg": {
      "file": "data/eeg.bin",
      "metadata": "data/eeg.json",
      "format": "binary_float32_le"
    },
    "events": {
      "file": "events.json"
    }
  },

  "references": {
    "device": "device.json",
    "subject": "subject.json",
    "channels": "channels.json"
  },

  "software": {
    "recording_software": "OpenBCI GUI",
    "recording_version": "5.1.0",
    "export_software": "WIA BCI Converter",
    "export_version": "1.0.0"
  },

  "notes": "Subject reported fatigue at minute 45"
}
```

---

## 4. Device Information

### 4.1 device.json

```json
{
  "$schema": "https://wia.live/schemas/bci/device.schema.json",
  "device_id": "dev-openbci-001",

  "device_info": {
    "manufacturer": "OpenBCI",
    "model": "Cyton",
    "serial_number": "OB-2024-12345",
    "firmware_version": "3.1.2"
  },

  "device_type": "eeg",
  "invasiveness": "non_invasive",

  "acquisition": {
    "sampling_rate_hz": 250,
    "resolution_bits": 24,
    "gain": 24,
    "reference_type": "common_average",
    "ground_electrode": "A2"
  },

  "communication": {
    "protocol": "bluetooth_le",
    "data_format": "openbci_cyton",
    "packet_size_bytes": 32
  },

  "electrodes": {
    "type": "dry",
    "material": "gold",
    "placement_system": "10-20",
    "total_channels": 8
  },

  "calibration": {
    "last_calibration": "2025-01-10T09:00:00Z",
    "scale_factor_uv_per_count": 0.02235,
    "offset_uv": 0.0
  }
}
```

### 4.2 Device Types

| Type | Code | Description |
|------|------|-------------|
| EEG Headset | `eeg` | Non-invasive EEG recording |
| EEG Cap | `eeg_cap` | Research-grade EEG cap |
| Implant (Cortical) | `implant_cortical` | Intracortical implant |
| Implant (Endovascular) | `implant_endovascular` | Endovascular implant |
| fNIRS | `fnirs` | Functional near-infrared |
| Hybrid | `hybrid` | Multiple modalities |

### 4.3 Invasiveness Classification

| Level | Code | Description |
|-------|------|-------------|
| Non-invasive | `non_invasive` | External sensors only |
| Minimally invasive | `minimally_invasive` | Endovascular, epidural |
| Invasive | `invasive` | Intracortical, subdural |

---

## 5. Subject Information

### 5.1 subject.json

```json
{
  "$schema": "https://wia.live/schemas/bci/subject.schema.json",
  "subject_id": "sub-001",

  "demographics": {
    "age_years": 35,
    "sex": "male",
    "handedness": "right",
    "language": "ko"
  },

  "condition": {
    "diagnosis": "spinal_cord_injury",
    "diagnosis_code_icd10": "S14.1",
    "level": "C4",
    "completeness": "complete",
    "years_since_onset": 5
  },

  "bci_experience": {
    "previous_bci_use": true,
    "training_sessions": 10,
    "proficiency_level": "intermediate"
  },

  "consent": {
    "informed_consent": true,
    "consent_date": "2025-01-10",
    "data_sharing": "anonymized",
    "ethics_approval": "IRB-2024-0123"
  }
}
```

---

## 6. Channel Definitions

### 6.1 channels.json

```json
{
  "$schema": "https://wia.live/schemas/bci/channels.schema.json",
  "channel_count": 8,
  "sampling_rate_hz": 250,

  "channels": [
    {
      "index": 0,
      "label": "Fp1",
      "type": "eeg",
      "unit": "uV",
      "position": {
        "system": "10-20",
        "x": -0.95,
        "y": 0.31,
        "z": 0.0
      },
      "reference": "CAR",
      "filters": {
        "highpass_hz": 0.5,
        "lowpass_hz": 50.0,
        "notch_hz": 60.0
      },
      "impedance_kohm": 5.2,
      "status": "good"
    },
    {
      "index": 1,
      "label": "Fp2",
      "type": "eeg",
      "unit": "uV",
      "position": {
        "system": "10-20",
        "x": 0.95,
        "y": 0.31,
        "z": 0.0
      },
      "reference": "CAR",
      "filters": {
        "highpass_hz": 0.5,
        "lowpass_hz": 50.0,
        "notch_hz": 60.0
      },
      "impedance_kohm": 4.8,
      "status": "good"
    }
  ],

  "reference_channels": [
    {
      "label": "A1",
      "type": "reference",
      "position": {"system": "10-20", "location": "left_mastoid"}
    },
    {
      "label": "A2",
      "type": "ground",
      "position": {"system": "10-20", "location": "right_mastoid"}
    }
  ]
}
```

### 6.2 Channel Types

| Type | Code | Description |
|------|------|-------------|
| EEG | `eeg` | Electroencephalography |
| EMG | `emg` | Electromyography |
| EOG | `eog` | Electrooculography |
| ECG | `ecg` | Electrocardiography |
| Reference | `reference` | Reference electrode |
| Ground | `ground` | Ground electrode |
| Trigger | `trigger` | Event trigger channel |
| Accelerometer | `accel` | Motion sensor |
| Auxiliary | `aux` | Other auxiliary |

### 6.3 Standard Electrode Positions (10-20)

```
        Nasion
          |
    Fp1--Fpz--Fp2
   /              \
  F7--F3--Fz--F4--F8
  |    |   |   |   |
  T3--C3--Cz--C4--T4
  |    |   |   |   |
  T5--P3--Pz--P4--T6
   \              /
    O1---Oz---O2
          |
        Inion
```

---

## 7. Event Markers

### 7.1 events.json

```json
{
  "$schema": "https://wia.live/schemas/bci/events.schema.json",
  "event_count": 120,

  "event_definitions": {
    "stimulus_onset": {
      "code": 1,
      "description": "Visual stimulus appears"
    },
    "stimulus_offset": {
      "code": 2,
      "description": "Visual stimulus disappears"
    },
    "left_hand": {
      "code": 10,
      "description": "Left hand motor imagery cue"
    },
    "right_hand": {
      "code": 11,
      "description": "Right hand motor imagery cue"
    },
    "feet": {
      "code": 12,
      "description": "Feet motor imagery cue"
    },
    "tongue": {
      "code": 13,
      "description": "Tongue motor imagery cue"
    },
    "rest": {
      "code": 20,
      "description": "Rest period"
    }
  },

  "events": [
    {
      "timestamp_seconds": 0.0,
      "sample_index": 0,
      "code": 1,
      "type": "stimulus_onset",
      "value": "left_hand",
      "duration_seconds": 4.0
    },
    {
      "timestamp_seconds": 4.0,
      "sample_index": 1000,
      "code": 2,
      "type": "stimulus_offset",
      "duration_seconds": 0.0
    },
    {
      "timestamp_seconds": 6.0,
      "sample_index": 1500,
      "code": 1,
      "type": "stimulus_onset",
      "value": "right_hand",
      "duration_seconds": 4.0
    }
  ],

  "annotations": [
    {
      "timestamp_seconds": 45.0,
      "text": "Subject blinked",
      "author": "experimenter"
    }
  ]
}
```

### 7.2 Standard Event Codes

| Range | Category | Description |
|-------|----------|-------------|
| 1-9 | System | Session start/end, stimulus on/off |
| 10-19 | Motor Imagery | Left hand, right hand, feet, tongue |
| 20-29 | States | Rest, focus, relax |
| 30-39 | SSVEP | Frequency-specific stimuli |
| 40-49 | P300 | Target/non-target |
| 50-59 | Feedback | Correct, incorrect, timeout |
| 100+ | Custom | User-defined events |

---

## 8. Binary Data Format

### 8.1 EEG Data (eeg.bin)

**Format**: Little-endian, interleaved channels

```
Layout: [ch0_s0, ch1_s0, ..., chN_s0, ch0_s1, ch1_s1, ..., chN_s1, ...]

Data Type: float32 (IEEE 754)
Byte Order: Little-endian
Unit: microvolts (µV) after scaling

File Size = num_channels × num_samples × 4 bytes
```

### 8.2 EEG Metadata (data/eeg.json)

```json
{
  "$schema": "https://wia.live/schemas/bci/eeg-data.schema.json",

  "data_format": {
    "type": "binary",
    "encoding": "float32_le",
    "interleaved": true,
    "byte_order": "little_endian"
  },

  "dimensions": {
    "num_channels": 8,
    "num_samples": 900000,
    "sampling_rate_hz": 250
  },

  "units": {
    "amplitude": "uV",
    "time": "seconds"
  },

  "preprocessing": {
    "applied": true,
    "steps": [
      {
        "name": "scale_to_uv",
        "parameters": {"factor": 0.02235}
      },
      {
        "name": "highpass_filter",
        "parameters": {"cutoff_hz": 0.5, "order": 4}
      }
    ]
  },

  "quality": {
    "good_channels": [0, 1, 2, 3, 4, 5, 6, 7],
    "bad_channels": [],
    "interpolated_channels": [],
    "artifact_segments": [
      {"start_sample": 45000, "end_sample": 45500, "type": "eye_blink"}
    ]
  },

  "checksum": {
    "algorithm": "sha256",
    "value": "a1b2c3d4e5f6..."
  }
}
```

---

## 9. Compatibility

### 9.1 Import from EDF+

```python
# Pseudocode for EDF+ to WIA-BCI conversion
def convert_edf_to_wia(edf_file):
    # Read EDF+ header
    header = read_edf_header(edf_file)

    # Map to WIA-BCI format
    recording = {
        "wia_version": "1.0.0",
        "recording_info": {
            "start_time": header.startdate + header.starttime,
            "duration_seconds": header.num_records * header.record_duration
        }
    }

    # Convert channel info
    channels = []
    for i, signal in enumerate(header.signals):
        channels.append({
            "index": i,
            "label": signal.label,
            "unit": signal.physical_dimension,
            "type": infer_channel_type(signal.label)
        })

    return recording, channels
```

### 9.2 Export to BIDS

```python
# Pseudocode for WIA-BCI to BIDS conversion
def convert_wia_to_bids(wia_dir, bids_root):
    recording = load_json(wia_dir / "recording.json")

    # Create BIDS structure
    bids_subject = f"sub-{recording.subject_id}"
    bids_session = f"ses-{recording.session.session_id}"

    # Write _eeg.json sidecar
    eeg_json = {
        "TaskName": recording.session.paradigm,
        "SamplingFrequency": recording.sampling_rate,
        "EEGReference": recording.reference_type,
        # ... more BIDS fields
    }

    return bids_structure
```

---

## 10. Validation Rules

### 10.1 Required Fields

| File | Required Fields |
|------|-----------------|
| recording.json | wia_version, recording_id, recording_info.start_time, data_files |
| device.json | device_id, device_type, acquisition.sampling_rate_hz |
| channels.json | channel_count, channels[].index, channels[].label, channels[].type |
| events.json | event_count, events[].timestamp_seconds, events[].code |

### 10.2 Value Constraints

```yaml
sampling_rate_hz:
  type: number
  minimum: 1
  maximum: 100000

channel_type:
  type: string
  enum: [eeg, emg, eog, ecg, reference, ground, trigger, accel, aux]

invasiveness:
  type: string
  enum: [non_invasive, minimally_invasive, invasive]

position_system:
  type: string
  enum: ["10-20", "10-10", "10-5", "custom"]
```

---

## 11. Examples

### 11.1 Minimal Recording

```json
// recording.json (minimal)
{
  "wia_version": "1.0.0",
  "recording_id": "rec-001",
  "recording_info": {
    "start_time": "2025-01-15T10:00:00Z",
    "duration_seconds": 600
  },
  "data_files": {
    "eeg": {"file": "data/eeg.bin"}
  },
  "references": {
    "channels": "channels.json"
  }
}
```

### 11.2 Motor Imagery Experiment

See full example in `/bci/examples/motor-imagery/`

### 11.3 P300 Speller

See full example in `/bci/examples/p300-speller/`

---

## 12. Schema Files

All JSON Schema files are available at:

- `schemas/recording.schema.json`
- `schemas/device.schema.json`
- `schemas/subject.schema.json`
- `schemas/channels.schema.json`
- `schemas/events.schema.json`
- `schemas/eeg-data.schema.json`

Online: `https://wia.live/schemas/bci/`

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

## 14. Acknowledgments

This specification is informed by existing standards including:
- EDF+ (European Data Format)
- BIDS (Brain Imaging Data Structure)
- XDF (Extensible Data Format)
- OpenBCI Data Format

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group

---

弘益人間 - *Benefit All Humanity*
