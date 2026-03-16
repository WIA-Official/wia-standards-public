# WIA-CONSCIOUSNESS: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the standardized data formats for consciousness measurement and assessment in the WIA-CONSCIOUSNESS standard. All implementations MUST support these formats to ensure interoperability across research institutions, clinical settings, and AI evaluation frameworks.

## 2. Core Schema: Consciousness Index

### 2.1 Primary Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/consciousness/v1.0.0",
  "$id": "consciousness-index",
  "title": "WIA Consciousness Index",
  "type": "object",
  "required": ["subject_id", "timestamp", "measurement"],
  "properties": {
    "subject_id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the subject"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO8601 timestamp of measurement"
    },
    "iit_metrics": { "$ref": "#/definitions/iit_metrics" },
    "gnw_metrics": { "$ref": "#/definitions/gnw_metrics" },
    "practical_measures": { "$ref": "#/definitions/practical_measures" },
    "state": { "$ref": "#/definitions/consciousness_state" },
    "measurement": { "$ref": "#/definitions/measurement_info" }
  }
}
```

### 2.2 IIT Metrics Definition

```json
{
  "definitions": {
    "iit_metrics": {
      "type": "object",
      "description": "Integrated Information Theory 4.0 metrics",
      "properties": {
        "phi_estimate": {
          "type": "object",
          "properties": {
            "value": {
              "type": "number",
              "minimum": 0,
              "description": "Estimated Φ value in bits"
            },
            "unit": {
              "type": "string",
              "enum": ["bits"],
              "default": "bits"
            },
            "computation_method": {
              "type": "string",
              "enum": ["exact", "approximation", "upper_bound", "lower_bound"],
              "description": "Method used for Φ computation"
            }
          },
          "required": ["value", "unit"]
        },
        "phi_structure": {
          "type": "object",
          "properties": {
            "quality": {
              "type": "string",
              "description": "Qualitative description of experience structure"
            },
            "distinctions": {
              "type": "integer",
              "minimum": 0,
              "description": "Number of distinctions in the structure"
            },
            "relations": {
              "type": "integer",
              "minimum": 0,
              "description": "Number of relations in the structure"
            }
          }
        },
        "cause_effect_power": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Normalized causal power of the system"
        },
        "irreducibility": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Degree to which the system cannot be reduced to parts"
        },
        "intrinsicality": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Degree to which system exists for itself"
        },
        "exclusion_level": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Definiteness of system boundaries"
        }
      }
    }
  }
}
```

### 2.3 GNW Metrics Definition

```json
{
  "definitions": {
    "gnw_metrics": {
      "type": "object",
      "description": "Global Neuronal Workspace Theory metrics",
      "properties": {
        "global_ignition": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Measure of global ignition event occurrence"
        },
        "ignition_threshold": {
          "type": "number",
          "minimum": 0,
          "description": "Threshold for ignition in milliseconds (typically ~300ms)"
        },
        "prefrontal_activation": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Prefrontal cortex activation level"
        },
        "parietal_activation": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Parietal cortex activation level"
        },
        "broadcast_strength": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Strength of global information broadcast"
        },
        "workspace_access": {
          "type": "boolean",
          "description": "Whether information entered global workspace"
        },
        "p3b_amplitude": {
          "type": "number",
          "description": "P3b ERP component amplitude in microvolts"
        }
      }
    }
  }
}
```

### 2.4 Practical Measures Definition

```json
{
  "definitions": {
    "practical_measures": {
      "type": "object",
      "description": "Clinically applicable consciousness measures",
      "properties": {
        "pci": {
          "type": "object",
          "description": "Perturbational Complexity Index",
          "properties": {
            "value": {
              "type": "number",
              "minimum": 0,
              "maximum": 1,
              "description": "PCI value (threshold PCI* = 0.31)"
            },
            "max": {
              "type": "number",
              "default": 1.0
            },
            "classification": {
              "type": "string",
              "enum": ["VS_UWS", "MCS_minus", "MCS_plus", "EMCS", "LIS", "conscious"],
              "description": "Clinical classification based on PCI"
            }
          },
          "required": ["value"]
        },
        "complexity_index": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "General complexity index"
        },
        "lempel_ziv_complexity": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Lempel-Ziv algorithmic complexity"
        },
        "entropy_measures": {
          "type": "object",
          "properties": {
            "permutation_entropy": { "type": "number" },
            "sample_entropy": { "type": "number" },
            "spectral_entropy": { "type": "number" }
          }
        },
        "connectivity_measures": {
          "type": "object",
          "properties": {
            "functional_connectivity": { "type": "number" },
            "effective_connectivity": { "type": "number" },
            "network_integration": { "type": "number" }
          }
        }
      }
    }
  }
}
```

### 2.5 Consciousness State Definition

```json
{
  "definitions": {
    "consciousness_state": {
      "type": "object",
      "properties": {
        "condition": {
          "type": "string",
          "enum": [
            "awake",
            "drowsy",
            "sleep_N1",
            "sleep_N2",
            "sleep_N3",
            "REM",
            "anesthesia_light",
            "anesthesia_moderate",
            "anesthesia_deep",
            "coma",
            "VS_UWS",
            "MCS_minus",
            "MCS_plus",
            "EMCS",
            "LIS",
            "meditation",
            "psychedelic",
            "hypnosis",
            "seizure"
          ],
          "description": "Current consciousness condition"
        },
        "arousal_level": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Arousal/wakefulness level (0=unresponsive, 1=fully alert)"
        },
        "awareness_level": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Awareness/content level (0=none, 1=full)"
        },
        "behavioral_crs_r": {
          "type": "integer",
          "minimum": 0,
          "maximum": 23,
          "description": "Coma Recovery Scale-Revised total score"
        },
        "gcs": {
          "type": "integer",
          "minimum": 3,
          "maximum": 15,
          "description": "Glasgow Coma Scale score"
        }
      }
    }
  }
}
```

### 2.6 Measurement Information

```json
{
  "definitions": {
    "measurement_info": {
      "type": "object",
      "required": ["method"],
      "properties": {
        "method": {
          "type": "string",
          "enum": [
            "TMS-EEG",
            "fMRI",
            "MEG",
            "iEEG",
            "EEG",
            "combined",
            "behavioral",
            "simulated"
          ],
          "description": "Measurement methodology"
        },
        "device": {
          "type": "string",
          "description": "Device/equipment identifier"
        },
        "electrode_count": {
          "type": "integer",
          "description": "Number of EEG electrodes (if applicable)"
        },
        "sampling_rate": {
          "type": "number",
          "description": "Sampling rate in Hz"
        },
        "tms_parameters": {
          "type": "object",
          "properties": {
            "intensity_percent": { "type": "number" },
            "pulse_count": { "type": "integer" },
            "frequency_hz": { "type": "number" },
            "target_region": { "type": "string" }
          }
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Confidence level of measurement"
        },
        "quality_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Data quality assessment"
        }
      }
    }
  }
}
```

## 3. AI Consciousness Assessment Schema

### 3.1 AI System Consciousness Indicators

```json
{
  "$schema": "https://wia.live/schemas/consciousness/ai-assessment/v1.0.0",
  "ai_consciousness_assessment": {
    "system_id": "string",
    "system_type": {
      "type": "string",
      "enum": ["LLM", "multimodal", "embodied", "hybrid", "other"]
    },
    "model_info": {
      "name": "string",
      "version": "string",
      "parameters": "number",
      "architecture": "string"
    },
    "indicators": {
      "description": "Based on Butlin et al. 14 indicators framework",
      "recurrent_processing": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "global_workspace": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "higher_order_thought": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "attention_schema": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "predictive_processing": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "agency_embodiment": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "theory_of_mind": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "metacognition": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "integrated_information": {
        "estimated_phi": "number",
        "evidence": "string",
        "confidence": "number"
      },
      "self_model": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "world_model": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "counterfactual_reasoning": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "emotional_states": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      },
      "subjective_reports": {
        "present": "boolean",
        "evidence": "string",
        "confidence": "number"
      }
    },
    "overall_assessment": {
      "indicators_satisfied": "integer",
      "indicators_partial": "integer",
      "indicators_unsatisfied": "integer",
      "consciousness_likelihood": {
        "type": "string",
        "enum": ["none", "unlikely", "uncertain", "possible", "likely"]
      },
      "confidence": "number",
      "notes": "string"
    }
  }
}
```

## 4. Clinical Classification Thresholds

### 4.1 PCI-Based Classification

| State | PCI Range | Description |
|-------|-----------|-------------|
| VS/UWS | < 0.31 | Vegetative State / Unresponsive Wakefulness Syndrome |
| MCS- | 0.31 - 0.37 | Minimally Conscious State (Minus) |
| MCS+ | 0.37 - 0.49 | Minimally Conscious State (Plus) |
| EMCS | 0.49 - 0.52 | Emerged from MCS |
| LIS | 0.51 - 0.62 | Locked-in Syndrome |
| Conscious | > 0.50 | Full Consciousness |

### 4.2 State Classification Mapping

```json
{
  "state_classification": {
    "normal_states": ["awake", "drowsy", "sleep_N1", "sleep_N2", "sleep_N3", "REM"],
    "altered_states": ["meditation", "psychedelic", "hypnosis", "flow"],
    "pathological_states": ["coma", "VS_UWS", "MCS_minus", "MCS_plus", "LIS"],
    "artificial_states": ["anesthesia_light", "anesthesia_moderate", "anesthesia_deep"]
  }
}
```

## 5. Data Exchange Format

### 5.1 Complete Measurement Record

```json
{
  "$schema": "https://wia.live/schemas/consciousness/v1.0.0",
  "consciousness_index": {
    "subject_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-01-15T10:30:00Z",
    "session_id": "session-001",

    "iit_metrics": {
      "phi_estimate": { "value": 2.5, "unit": "bits", "computation_method": "approximation" },
      "phi_structure": { "quality": "visual-spatial", "distinctions": 45, "relations": 120 },
      "cause_effect_power": 0.72,
      "irreducibility": 0.85,
      "intrinsicality": 0.78,
      "exclusion_level": 0.91
    },

    "gnw_metrics": {
      "global_ignition": 0.68,
      "ignition_threshold": 320,
      "prefrontal_activation": 0.55,
      "parietal_activation": 0.72,
      "broadcast_strength": 0.81,
      "workspace_access": true,
      "p3b_amplitude": 8.5
    },

    "practical_measures": {
      "pci": { "value": 0.48, "max": 1.0, "classification": "MCS_plus" },
      "complexity_index": 0.65,
      "lempel_ziv_complexity": 0.72,
      "entropy_measures": {
        "permutation_entropy": 0.68,
        "sample_entropy": 0.45,
        "spectral_entropy": 0.71
      }
    },

    "state": {
      "condition": "MCS_plus",
      "arousal_level": 0.45,
      "awareness_level": 0.38,
      "behavioral_crs_r": 12,
      "gcs": 9
    },

    "measurement": {
      "method": "TMS-EEG",
      "device": "Nexstim NBS System 5",
      "electrode_count": 60,
      "sampling_rate": 5000,
      "tms_parameters": {
        "intensity_percent": 80,
        "pulse_count": 200,
        "frequency_hz": 0.5,
        "target_region": "left_premotor"
      },
      "confidence": 0.92,
      "quality_score": 0.88
    }
  }
}
```

## 6. Validation Requirements

### 6.1 Required Fields

- `subject_id`: MUST be a valid UUID
- `timestamp`: MUST be ISO8601 format
- `measurement.method`: MUST be from enumerated list
- All numeric values: MUST be within specified ranges

### 6.2 Conditional Requirements

- If `method` = "TMS-EEG", `tms_parameters` SHOULD be present
- If `pci.value` is present, `pci.classification` SHOULD be derived automatically
- If `state.condition` is pathological, `behavioral_crs_r` SHOULD be present

---

**弘益人間 (弘益人間) - Benefit All Humanity**
© 2025 WIA Standards · MIT License
