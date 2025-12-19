# WIA Emotion AI Standard - Phase 1: Data Format Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025

---

## 1. Overview

Phase 1 defines the standardized JSON schema for representing emotion data. This specification ensures interoperability across platforms, vendors, and applications by establishing a common language for emotion data exchange.

### 1.1 Design Principles

- **Interoperability**: Universal format compatible with any emotion AI system
- **Extensibility**: Support for additional emotions and modalities
- **Precision**: Fine-grained emotion representation with intensity and confidence
- **Completeness**: Support for discrete emotions, dimensional models, and FACS

---

## 2. Core Schema

### 2.1 EmotionEvent Root Schema

```json
{
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://wiastandards.com/schemas/emotion-ai/v1/emotion-event.json",
    "title": "WIA Emotion Event",
    "description": "Standardized emotion event data format",
    "type": "object",
    "required": ["event_id", "timestamp", "version", "emotions"],
    "properties": {
        "event_id": {
            "type": "string",
            "format": "uuid",
            "description": "Unique identifier for the event (UUID v4)"
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "ISO 8601 timestamp of emotion detection"
        },
        "version": {
            "type": "string",
            "pattern": "^\\d+\\.\\d+\\.\\d+$",
            "description": "Schema version (SemVer format)"
        },
        "session_id": {
            "type": "string",
            "description": "Optional session identifier for grouping events"
        },
        "subject_id": {
            "type": "string",
            "description": "Anonymized subject identifier"
        },
        "emotions": {
            "$ref": "#/$defs/EmotionArray"
        },
        "action_units": {
            "$ref": "#/$defs/ActionUnitArray"
        },
        "dimensional": {
            "$ref": "#/$defs/DimensionalModel"
        },
        "modalities": {
            "$ref": "#/$defs/ModalityArray"
        },
        "metadata": {
            "$ref": "#/$defs/Metadata"
        }
    }
}
```

### 2.2 Required Fields

| Field | Type | Description |
|-------|------|-------------|
| event_id | string (UUID) | Unique identifier for the event |
| timestamp | string (ISO 8601) | Time of emotion detection |
| version | string (SemVer) | Schema version (e.g., "1.0.0") |
| emotions | array | Array of detected emotions |

### 2.3 Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| session_id | string | Groups multiple events in a session |
| subject_id | string | Anonymized subject identifier |
| action_units | array | FACS Action Unit data |
| dimensional | object | Valence-Arousal-Dominance values |
| modalities | array | Per-modality detailed data |
| metadata | object | Additional metadata |

---

## 3. Emotion Classification

### 3.1 Emotion Object Schema

```json
{
    "$defs": {
        "Emotion": {
            "type": "object",
            "required": ["category", "intensity", "confidence"],
            "properties": {
                "category": {
                    "type": "string",
                    "enum": [
                        "happiness", "sadness", "anger", "fear",
                        "disgust", "surprise", "neutral", "contempt",
                        "confusion", "interest", "boredom", "frustration",
                        "excitement", "anxiety"
                    ]
                },
                "intensity": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0
                },
                "confidence": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0
                },
                "onset_time": {
                    "type": "number",
                    "description": "Start time in seconds (relative to session)"
                },
                "apex_time": {
                    "type": "number",
                    "description": "Peak intensity time in seconds"
                },
                "offset_time": {
                    "type": "number",
                    "description": "End time in seconds"
                },
                "source_modality": {
                    "type": "string",
                    "enum": ["facial", "voice", "text", "biosignal", "multimodal"]
                }
            }
        }
    }
}
```

### 3.2 Emotion Categories

#### Basic Emotions (Ekman)
| Category | Description | Typical AU Combination |
|----------|-------------|------------------------|
| happiness | Joy, pleasure, satisfaction | AU6 + AU12 |
| sadness | Sorrow, disappointment | AU1 + AU4 + AU15 |
| anger | Rage, irritation | AU4 + AU5 + AU7 + AU23 |
| fear | Terror, anxiety | AU1 + AU2 + AU4 + AU5 + AU20 |
| disgust | Revulsion, distaste | AU9 + AU15 + AU16 |
| surprise | Astonishment | AU1 + AU2 + AU5 + AU26 |

#### Extended Emotions
| Category | Description |
|----------|-------------|
| neutral | No emotion detected |
| contempt | Scorn, disdain |
| confusion | Puzzlement, uncertainty |
| interest | Curiosity, attention |
| boredom | Tedium, lack of interest |
| frustration | Dissatisfaction, annoyance |
| excitement | Enthusiasm, eagerness |
| anxiety | Worry, unease |

### 3.3 Intensity Scale

| Range | Level | Description |
|-------|-------|-------------|
| 0.0 - 0.2 | Subtle | Barely perceptible |
| 0.2 - 0.4 | Mild | Slight expression |
| 0.4 - 0.6 | Moderate | Clear expression |
| 0.6 - 0.8 | Strong | Pronounced expression |
| 0.8 - 1.0 | Very Strong | Maximum intensity |

### 3.4 Confidence Interpretation

| Range | Level | Recommended Use |
|-------|-------|-----------------|
| 0.0 - 0.3 | Low | Do not use, needs verification |
| 0.3 - 0.5 | Uncertain | Use as secondary indicator only |
| 0.5 - 0.7 | Moderate | Use with other modalities |
| 0.7 - 0.85 | High | Suitable for most applications |
| 0.85 - 1.0 | Very High | Suitable for sensitive applications |

---

## 4. Action Unit Encoding

### 4.1 ActionUnit Schema

```json
{
    "$defs": {
        "ActionUnit": {
            "type": "object",
            "required": ["au", "intensity"],
            "properties": {
                "au": {
                    "type": "string",
                    "pattern": "^AU\\d{1,2}[LRAB]?$"
                },
                "name": {
                    "type": "string"
                },
                "intensity": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0
                },
                "intensity_label": {
                    "type": "string",
                    "enum": ["A", "B", "C", "D", "E"]
                },
                "confidence": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0
                },
                "symmetric": {
                    "type": "boolean"
                }
            }
        }
    }
}
```

### 4.2 AU Code Format

| Format | Description | Example |
|--------|-------------|---------|
| AU{number} | Bilateral symmetric | AU6, AU12 |
| AU{number}L | Left side only | AU12L |
| AU{number}R | Right side only | AU12R |
| AU{number}A | Asymmetric bilateral | AU12A |

### 4.3 Core Action Units

| AU | Name | Muscle |
|----|------|--------|
| AU1 | Inner Brow Raiser | Frontalis (pars medialis) |
| AU2 | Outer Brow Raiser | Frontalis (pars lateralis) |
| AU4 | Brow Lowerer | Corrugator supercilii |
| AU5 | Upper Lid Raiser | Levator palpebrae |
| AU6 | Cheek Raiser | Orbicularis oculi |
| AU7 | Lid Tightener | Orbicularis oculi |
| AU9 | Nose Wrinkler | Levator labii superioris |
| AU10 | Upper Lip Raiser | Levator labii superioris |
| AU12 | Lip Corner Puller | Zygomaticus major |
| AU15 | Lip Corner Depressor | Depressor anguli oris |
| AU17 | Chin Raiser | Mentalis |
| AU20 | Lip Stretcher | Risorius |
| AU23 | Lip Tightener | Orbicularis oris |
| AU24 | Lip Pressor | Orbicularis oris |
| AU25 | Lips Part | Depressor labii |
| AU26 | Jaw Drop | Masseter, Temporalis |

### 4.4 Intensity Mapping

| FACS Label | Description | Normalized Range |
|------------|-------------|------------------|
| A | Trace | 0.0 - 0.2 |
| B | Slight | 0.2 - 0.4 |
| C | Marked | 0.4 - 0.6 |
| D | Severe | 0.6 - 0.8 |
| E | Maximum | 0.8 - 1.0 |

---

## 5. Dimensional Model

### 5.1 DimensionalModel Schema

```json
{
    "$defs": {
        "DimensionalModel": {
            "type": "object",
            "properties": {
                "valence": {
                    "type": "number",
                    "minimum": -1.0,
                    "maximum": 1.0,
                    "description": "Pleasantness: negative (-1) to positive (+1)"
                },
                "arousal": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "description": "Activation level: calm (0) to excited (1)"
                },
                "dominance": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "description": "Control: submissive (0) to dominant (1)"
                }
            }
        }
    }
}
```

### 5.2 Discrete to Dimensional Mapping

| Emotion | Valence | Arousal | Dominance |
|---------|---------|---------|-----------|
| happiness | 0.8 | 0.6 | 0.7 |
| sadness | -0.7 | 0.3 | 0.3 |
| anger | -0.6 | 0.8 | 0.7 |
| fear | -0.8 | 0.7 | 0.2 |
| disgust | -0.7 | 0.5 | 0.6 |
| surprise | 0.1 | 0.8 | 0.5 |
| neutral | 0.0 | 0.3 | 0.5 |

---

## 6. Multimodal Data

### 6.1 Modality Types

| Type | Input | Analysis |
|------|-------|----------|
| facial | Video frames, images | Expressions, AU, head pose |
| voice | Audio stream | Pitch, tone, rate, intensity |
| text | Text content | Sentiment, emotion words |
| biosignal | Sensor data | Heart rate, EDA, EEG |

### 6.2 FacialData Schema

```json
{
    "$defs": {
        "FacialData": {
            "type": "object",
            "properties": {
                "face_detected": { "type": "boolean" },
                "face_count": { "type": "integer" },
                "face_quality": { "type": "number" },
                "landmarks": { "type": "array" },
                "head_pose": {
                    "type": "object",
                    "properties": {
                        "pitch": { "type": "number" },
                        "yaw": { "type": "number" },
                        "roll": { "type": "number" }
                    }
                },
                "gaze": {
                    "type": "object",
                    "properties": {
                        "x": { "type": "number" },
                        "y": { "type": "number" }
                    }
                },
                "micro_expression_detected": { "type": "boolean" }
            }
        }
    }
}
```

### 6.3 VoiceData Schema

```json
{
    "$defs": {
        "VoiceData": {
            "type": "object",
            "properties": {
                "duration_seconds": { "type": "number" },
                "sample_rate": { "type": "integer" },
                "pitch": {
                    "type": "object",
                    "properties": {
                        "mean": { "type": "number" },
                        "std": { "type": "number" }
                    }
                },
                "intensity": {
                    "type": "object",
                    "properties": {
                        "mean": { "type": "number" },
                        "std": { "type": "number" }
                    }
                },
                "speech_rate": { "type": "number" },
                "voice_quality": {
                    "type": "object",
                    "properties": {
                        "jitter": { "type": "number" },
                        "shimmer": { "type": "number" },
                        "hnr": { "type": "number" }
                    }
                }
            }
        }
    }
}
```

---

## 7. Metadata

### 7.1 Metadata Schema

```json
{
    "$defs": {
        "Metadata": {
            "type": "object",
            "properties": {
                "provider": { "type": "string" },
                "model_version": { "type": "string" },
                "processing_time_ms": { "type": "integer" },
                "cultural_context": { "type": "string" },
                "environment": {
                    "type": "object",
                    "properties": {
                        "lighting": { "type": "string" },
                        "noise_level": { "type": "string" }
                    }
                }
            }
        }
    }
}
```

---

## 8. Complete Example

```json
{
    "event_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "version": "1.0.0",
    "session_id": "session-abc-123",
    "subject_id": "anon-user-789",
    "emotions": [
        {
            "category": "happiness",
            "intensity": 0.85,
            "confidence": 0.92,
            "onset_time": 0.0,
            "apex_time": 1.2,
            "offset_time": 3.5,
            "source_modality": "multimodal"
        }
    ],
    "action_units": [
        {
            "au": "AU6",
            "name": "Cheek Raiser",
            "intensity": 0.75,
            "intensity_label": "D",
            "confidence": 0.91,
            "symmetric": true
        },
        {
            "au": "AU12",
            "name": "Lip Corner Puller",
            "intensity": 0.82,
            "intensity_label": "D",
            "confidence": 0.94,
            "symmetric": true
        }
    ],
    "dimensional": {
        "valence": 0.78,
        "arousal": 0.62,
        "dominance": 0.71
    },
    "modalities": [
        {
            "type": "facial",
            "confidence": 0.92,
            "weight": 0.4,
            "data": {
                "face_detected": true,
                "face_quality": 0.95
            }
        }
    ],
    "metadata": {
        "provider": "WIA-Certified-Provider",
        "model_version": "v2.1.0",
        "processing_time_ms": 45,
        "cultural_context": "ko-KR"
    }
}
```

---

## 9. Validation

### 9.1 Schema Validation

All implementations MUST validate emotion events against the official JSON Schema before processing or storage.

### 9.2 Required Validation Rules

1. `event_id` MUST be a valid UUID v4
2. `timestamp` MUST be valid ISO 8601 format
3. `version` MUST match SemVer pattern
4. `emotions` array MUST contain at least one emotion
5. `intensity` and `confidence` MUST be in range [0.0, 1.0]
6. `valence` MUST be in range [-1.0, 1.0]
7. `arousal` and `dominance` MUST be in range [0.0, 1.0]

---

## 10. References

- Ekman, P. (1992). An argument for basic emotions
- Russell, J. A. (1980). A circumplex model of affect
- Ekman, P., & Friesen, W. V. (1978). Facial Action Coding System

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Emotion AI Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
