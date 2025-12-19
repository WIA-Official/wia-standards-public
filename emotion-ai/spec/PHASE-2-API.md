# WIA Emotion AI Standard - Phase 2: API Interface Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025

---

## 1. Overview

Phase 2 defines the RESTful API interface for emotion AI services. This specification ensures consistent interaction patterns across different providers while maintaining flexibility for various use cases.

### 1.1 Design Principles

- **RESTful**: Follow REST architectural constraints
- **Stateless**: Each request contains all necessary information
- **Versioned**: URL-based versioning for compatibility
- **Secure**: TLS encryption and robust authentication

---

## 2. API Structure

### 2.1 Base URL

```
https://api.{provider}.com/wia/emotion/v1/
```

### 2.2 Versioning

| Version | URL Path | Status |
|---------|----------|--------|
| v1 | /wia/emotion/v1/ | Current (Stable) |
| v1-beta | /wia/emotion/v1-beta/ | Beta features |
| v2 | /wia/emotion/v2/ | Planned |

### 2.3 Common Headers

#### Request Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
Accept-Language: en-US
```

#### Response Headers

```http
Content-Type: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
X-Processing-Time-Ms: 45
X-Rate-Limit-Remaining: 99
X-Rate-Limit-Reset: 1705329600
```

---

## 3. Authentication

### 3.1 Supported Methods

| Method | Use Case | Header |
|--------|----------|--------|
| Bearer Token | Server-to-server | Authorization: Bearer {token} |
| API Key | Client applications | X-API-Key: {api_key} |
| OAuth 2.0 | User delegation | Authorization: Bearer {oauth_token} |

### 3.2 Token Structure (JWT)

```json
{
    "header": {
        "alg": "RS256",
        "typ": "JWT"
    },
    "payload": {
        "sub": "client-id",
        "iss": "https://auth.provider.com",
        "aud": "wia-emotion-api",
        "iat": 1705329000,
        "exp": 1705332600,
        "scope": ["analyze:facial", "analyze:voice", "analyze:text"]
    }
}
```

---

## 4. Facial Emotion Analysis Endpoints

### 4.1 Analyze Image

**Endpoint**: `POST /wia/emotion/v1/analyze/facial/image`

**Request**:
```json
{
    "image": {
        "content": "{base64_encoded_image}",
        "format": "jpeg"
    },
    "options": {
        "detect_action_units": true,
        "detect_landmarks": true,
        "detect_head_pose": true,
        "detect_gaze": false,
        "cultural_context": "ko-KR",
        "confidence_threshold": 0.5
    }
}
```

**Response**:
```json
{
    "request_id": "req-12345-abcde",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "processing_time_ms": 125,
    "faces": [
        {
            "face_id": 1,
            "bounding_box": {
                "x": 120,
                "y": 80,
                "width": 200,
                "height": 250
            },
            "emotions": [
                {
                    "category": "happiness",
                    "intensity": 0.82,
                    "confidence": 0.94
                }
            ],
            "action_units": [...],
            "dimensional": {
                "valence": 0.78,
                "arousal": 0.55,
                "dominance": 0.68
            },
            "head_pose": {
                "pitch": -5.2,
                "yaw": 3.1,
                "roll": 1.5
            }
        }
    ]
}
```

### 4.2 Analyze Video (Async)

**Endpoint**: `POST /wia/emotion/v1/analyze/facial/video`

**Request**:
```json
{
    "video": {
        "url": "https://storage.example.com/video.mp4",
        "format": "mp4"
    },
    "options": {
        "frame_rate": 5,
        "detect_micro_expressions": true,
        "temporal_smoothing": true,
        "aggregate_results": true
    },
    "callback": {
        "url": "https://your-server.com/webhook",
        "method": "POST"
    }
}
```

**Response**:
```json
{
    "job_id": "job-67890-fghij",
    "status": "processing",
    "estimated_completion_seconds": 120,
    "status_url": "/wia/emotion/v1/jobs/job-67890-fghij"
}
```

### 4.3 Get Job Status

**Endpoint**: `GET /wia/emotion/v1/jobs/{job_id}`

**Response**:
```json
{
    "job_id": "job-67890-fghij",
    "status": "completed",
    "created_at": "2025-01-15T14:30:00.000Z",
    "completed_at": "2025-01-15T14:32:05.000Z",
    "result_url": "/wia/emotion/v1/jobs/job-67890-fghij/result"
}
```

---

## 5. Voice Emotion Analysis Endpoints

### 5.1 Analyze Audio

**Endpoint**: `POST /wia/emotion/v1/analyze/voice/audio`

**Request**:
```json
{
    "audio": {
        "content": "{base64_encoded_audio}",
        "format": "wav",
        "sample_rate": 16000
    },
    "options": {
        "language": "ko-KR",
        "extract_prosody": true,
        "extract_voice_quality": true,
        "segment_by_speaker": false
    }
}
```

**Response**:
```json
{
    "request_id": "req-voice-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "duration_seconds": 5.2,
    "segments": [
        {
            "start_time": 0.0,
            "end_time": 5.2,
            "emotions": [
                {
                    "category": "happiness",
                    "intensity": 0.72,
                    "confidence": 0.85
                }
            ],
            "prosody": {
                "pitch": {
                    "mean": 185.5,
                    "std": 28.3
                },
                "intensity": {
                    "mean": 65.2,
                    "std": 12.5
                },
                "speech_rate": 4.5
            },
            "voice_quality": {
                "jitter": 0.012,
                "shimmer": 0.035,
                "hnr": 18.5
            },
            "dimensional": {
                "valence": 0.65,
                "arousal": 0.58
            }
        }
    ],
    "aggregate": {
        "dominant_emotion": "happiness",
        "average_valence": 0.65,
        "average_arousal": 0.58
    }
}
```

### 5.2 Start Streaming Session

**Endpoint**: `POST /wia/emotion/v1/analyze/voice/stream/start`

**Request**:
```json
{
    "session_config": {
        "sample_rate": 16000,
        "encoding": "LINEAR16",
        "language": "ko-KR",
        "interim_results": true
    }
}
```

**Response**:
```json
{
    "session_id": "voice-session-12345",
    "websocket_url": "wss://api.provider.com/wia/emotion/v1/voice/stream/voice-session-12345",
    "expires_at": "2025-01-15T15:30:00.000Z"
}
```

---

## 6. Text Emotion Analysis Endpoints

### 6.1 Analyze Text

**Endpoint**: `POST /wia/emotion/v1/analyze/text`

**Request**:
```json
{
    "text": "I'm so happy with this product! It exceeded my expectations.",
    "options": {
        "language": "en",
        "detect_sarcasm": true,
        "analyze_emoji": true,
        "extract_emotion_words": true,
        "context_window": 3
    }
}
```

**Response**:
```json
{
    "request_id": "req-text-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "text_analysis": {
        "language": "en",
        "word_count": 11,
        "emotions": [
            {
                "category": "happiness",
                "intensity": 0.88,
                "confidence": 0.92
            },
            {
                "category": "excitement",
                "intensity": 0.65,
                "confidence": 0.78
            }
        ],
        "sentiment": {
            "score": 0.85,
            "magnitude": 0.92,
            "label": "positive"
        },
        "dimensional": {
            "valence": 0.82,
            "arousal": 0.68
        },
        "emotion_words": [
            {
                "word": "happy",
                "emotion": "happiness",
                "position": 3
            },
            {
                "word": "exceeded",
                "emotion": "excitement",
                "position": 8
            }
        ],
        "sarcasm_detected": false
    }
}
```

### 6.2 Analyze Conversation

**Endpoint**: `POST /wia/emotion/v1/analyze/text/conversation`

**Request**:
```json
{
    "messages": [
        {
            "role": "user",
            "content": "I'm really disappointed with this product",
            "timestamp": "2025-01-15T14:28:00.000Z"
        },
        {
            "role": "agent",
            "content": "I'm sorry to hear that. What seems to be the issue?",
            "timestamp": "2025-01-15T14:28:30.000Z"
        },
        {
            "role": "user",
            "content": "The delivery was 3 days late",
            "timestamp": "2025-01-15T14:29:00.000Z"
        }
    ],
    "options": {
        "track_emotional_trajectory": true,
        "identify_escalation": true
    }
}
```

**Response**:
```json
{
    "request_id": "req-conv-12345",
    "conversation_analysis": {
        "overall_sentiment": -0.45,
        "escalation_risk": 0.35,
        "emotional_trajectory": [
            {
                "message_index": 0,
                "emotion": "frustration",
                "intensity": 0.72,
                "valence": -0.65
            },
            {
                "message_index": 2,
                "emotion": "anger",
                "intensity": 0.55,
                "valence": -0.48
            }
        ],
        "resolution_recommendation": "empathy_acknowledgment"
    }
}
```

---

## 7. Biosignal Analysis Endpoints

### 7.1 Analyze Biosignals

**Endpoint**: `POST /wia/emotion/v1/analyze/biosignal`

**Request**:
```json
{
    "biosignals": {
        "heart_rate": {
            "values": [72, 75, 78, 82, 85, 88, 92],
            "sample_rate": 1,
            "unit": "bpm"
        },
        "eda": {
            "values": [2.5, 2.8, 3.2, 4.1, 5.2, 6.0, 5.5],
            "sample_rate": 4,
            "unit": "microsiemens"
        }
    },
    "options": {
        "compute_hrv": true,
        "detect_stress": true,
        "baseline_period_seconds": 60
    }
}
```

**Response**:
```json
{
    "request_id": "req-bio-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "analysis": {
        "emotions": [
            {
                "category": "anxiety",
                "intensity": 0.68,
                "confidence": 0.75
            }
        ],
        "dimensional": {
            "valence": -0.35,
            "arousal": 0.72
        },
        "physiological_state": {
            "stress_level": 0.65,
            "relaxation_level": 0.25,
            "engagement_level": 0.70
        },
        "hrv_metrics": {
            "sdnn": 42.5,
            "rmssd": 35.2,
            "lf_hf_ratio": 2.8
        }
    }
}
```

---

## 8. Multimodal Fusion Endpoints

### 8.1 Multimodal Analysis

**Endpoint**: `POST /wia/emotion/v1/analyze/multimodal`

**Request**:
```json
{
    "modalities": {
        "facial": {
            "image": "{base64_encoded_image}"
        },
        "voice": {
            "audio": "{base64_encoded_audio}",
            "format": "wav"
        },
        "text": {
            "content": "Yes, I'm very pleased"
        }
    },
    "options": {
        "fusion_strategy": "late_fusion",
        "modality_weights": {
            "facial": 0.4,
            "voice": 0.35,
            "text": 0.25
        },
        "conflict_resolution": "weighted_average"
    }
}
```

**Response**:
```json
{
    "request_id": "req-multi-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "fused_result": {
        "emotions": [
            {
                "category": "happiness",
                "intensity": 0.78,
                "confidence": 0.91
            }
        ],
        "dimensional": {
            "valence": 0.72,
            "arousal": 0.58,
            "dominance": 0.65
        }
    },
    "modality_results": {
        "facial": {
            "dominant_emotion": "happiness",
            "confidence": 0.92,
            "valence": 0.75
        },
        "voice": {
            "dominant_emotion": "happiness",
            "confidence": 0.85,
            "valence": 0.68
        },
        "text": {
            "dominant_emotion": "happiness",
            "confidence": 0.88,
            "valence": 0.72
        }
    },
    "agreement_score": 0.95,
    "conflict_detected": false
}
```

---

## 9. Error Handling

### 9.1 Error Response Format

```json
{
    "error": {
        "code": "INVALID_INPUT",
        "message": "Image format is not supported",
        "details": {
            "field": "image.format",
            "provided": "bmp",
            "supported": ["jpeg", "png", "webp"]
        },
        "request_id": "req-12345-abcde",
        "documentation_url": "https://docs.wiastandards.com/errors/INVALID_INPUT"
    }
}
```

### 9.2 Error Codes

| HTTP Code | Error Code | Description |
|-----------|------------|-------------|
| 400 | INVALID_INPUT | Malformed request |
| 401 | UNAUTHORIZED | Authentication failed |
| 403 | FORBIDDEN | Access denied |
| 404 | NOT_FOUND | Resource not found |
| 413 | PAYLOAD_TOO_LARGE | Request size exceeded |
| 422 | UNPROCESSABLE | Cannot process content |
| 429 | RATE_LIMITED | Rate limit exceeded |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily down |

### 9.3 Rate Limiting

**Rate Limit Headers**:
```http
X-Rate-Limit-Limit: 100
X-Rate-Limit-Remaining: 0
X-Rate-Limit-Reset: 1705329600
Retry-After: 60
```

| Plan | Requests/min | Requests/day |
|------|--------------|--------------|
| Free | 10 | 1,000 |
| Basic | 60 | 10,000 |
| Pro | 300 | 100,000 |
| Enterprise | Unlimited | Unlimited |

---

## 10. OpenAPI Specification

The complete OpenAPI 3.0 specification is available at:
- YAML: `https://api.{provider}.com/wia/emotion/v1/openapi.yaml`
- JSON: `https://api.{provider}.com/wia/emotion/v1/openapi.json`

---

## 11. SDK Support

| Language | Package | Installation |
|----------|---------|--------------|
| JavaScript | @wia/emotion-ai | npm install @wia/emotion-ai |
| Python | wia-emotion-ai | pip install wia-emotion-ai |
| Java | com.wia.emotion | Maven/Gradle |
| Go | github.com/wia/emotion-go | go get github.com/wia/emotion-go |
| Swift | WIAEmotion | Swift Package Manager |

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Emotion AI Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
