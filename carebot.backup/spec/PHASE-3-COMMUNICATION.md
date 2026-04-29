# WIA CareBot Phase 3: Communication Protocol & Security

## 1. Overview

WIA CareBot 통신 프로토콜은 AI 돌봄 로봇과 가족, 의료진, 응급 서비스 간의 안전하고
신뢰할 수 있는 통신을 위한 표준입니다.

### 1.1 핵심 원칙

1. **개인정보 보호 우선**: 취약 계층(노인) 데이터의 최고 수준 보호
2. **실시간 응급 대응**: 긴급 상황 시 즉각적 통신 보장
3. **가족 연결**: 가족과의 상시 연결 및 알림 제공
4. **의료 연동**: 병원 EMR, 119 응급 서비스와의 표준화된 연동

## 2. Communication Architecture

### 2.1 Protocol Stack

```
┌─────────────────────────────────────────────────────────┐
│                  Application Layer                       │
│   Care Sessions │ Family Calls │ Emergency │ Health     │
├─────────────────────────────────────────────────────────┤
│                  Security Layer                          │
│   TLS 1.3 │ E2E Encryption │ HIPAA Compliance           │
├─────────────────────────────────────────────────────────┤
│                  Transport Layer                         │
│   WebSocket │ WebRTC │ MQTT │ HTTP/3                    │
├─────────────────────────────────────────────────────────┤
│                  Network Layer                           │
│   WiFi │ LTE/5G │ LoRa (backup)                         │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Message Types

| Category | Type | Priority | Protocol | Latency Target |
|----------|------|----------|----------|----------------|
| Emergency | SOS, Fall Alert | Critical | MQTT QoS 2 + SMS | < 1s |
| Health | Vital Alerts | High | MQTT QoS 1 | < 3s |
| Family | Video Call | High | WebRTC | Real-time |
| Daily | Status Update | Normal | HTTP/3 | < 30s |
| Batch | Daily Summary | Low | HTTP/3 | Best effort |

## 3. Protocol Specifications

### 3.1 CareBot WebSocket Protocol (CWP)

실시간 양방향 통신을 위한 WebSocket 기반 프로토콜

```json
{
  "protocol": "cwp",
  "version": "1.0",
  "message_id": "msg-uuid-here",
  "timestamp": "2024-01-15T10:30:00Z",
  "type": "care_event",
  "payload": {},
  "auth": {
    "token": "jwt-token",
    "device_id": "carebot-001"
  }
}
```

#### Message Types

```
cwp://care/emotion          - 감정 상태 전송
cwp://care/conversation     - 대화 세션 데이터
cwp://health/vital          - 바이탈 사인 전송
cwp://health/medication     - 약 복용 상태
cwp://safety/alert          - 안전 이벤트 알림
cwp://safety/emergency      - 응급 상황 (최우선)
cwp://family/call           - 영상통화 시그널링
cwp://family/notification   - 가족 알림
cwp://device/status         - 기기 상태 보고
cwp://device/command        - 기기 명령
```

### 3.2 Emergency Protocol

응급 상황 처리를 위한 멀티채널 프로토콜

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   CareBot    │────▶│  WIA Cloud   │────▶│    Family    │
│              │     │              │     │     App      │
└──────┬───────┘     └──────┬───────┘     └──────────────┘
       │                    │
       │ Primary: MQTT      │ Push + SMS
       │ Backup: LTE SMS    │
       ▼                    ▼
┌──────────────┐     ┌──────────────┐
│   119 서비스  │◀───│  Care Team   │
│   (Emergency) │     │   (Hospital) │
└──────────────┘     └──────────────┘
```

#### Emergency Message Format

```json
{
  "emergency_id": "emg-001-20240115103000",
  "type": "fall_detected",
  "severity": "emergency",
  "recipient": {
    "id": "recipient-001",
    "name": "김영희",
    "age": 78,
    "address": "서울시 강남구 역삼동 123-45",
    "conditions": ["고혈압", "당뇨"],
    "medications": ["혈압약", "당뇨약"]
  },
  "location": {
    "room": "bathroom",
    "coordinates": {"lat": 37.5012, "lng": 127.0396}
  },
  "vitals_at_event": {
    "heart_rate": 92,
    "blood_pressure": "145/95"
  },
  "contacts": [
    {"name": "김철수", "relationship": "아들", "phone": "010-1234-5678"}
  ],
  "timestamp": "2024-01-15T10:30:00Z"
}
```

### 3.3 Video Call Protocol (WebRTC)

가족 영상통화를 위한 WebRTC 시그널링

```json
{
  "signaling": {
    "type": "offer|answer|ice-candidate",
    "from": "carebot-001",
    "to": "family-app-001",
    "session_id": "call-uuid",
    "sdp": "...",
    "ice_candidates": []
  },
  "accessibility": {
    "auto_answer": true,
    "loud_ringtone": true,
    "visual_indicator": "screen_flash",
    "simplified_ui": true
  }
}
```

## 4. Security Framework

### 4.1 Privacy Protection

돌봄 대상자(주로 고령자)의 개인정보 보호

#### Data Classification

| Level | Data Type | Storage | Retention | Access |
|-------|-----------|---------|-----------|--------|
| Critical | 의료정보, 바이탈 | 암호화 필수 | 법적 요건 | 의료진, 본인 |
| Sensitive | 위치, 일상패턴 | 암호화 | 1년 | 가족, 돌봄팀 |
| Private | 대화내용, 감정 | 암호화 | 30일 | 본인 동의 시 |
| Internal | 기기 상태 | 표준 | 90일 | 운영팀 |

#### Consent Management

```json
{
  "consent": {
    "recipient_id": "recipient-001",
    "guardian_id": "family-001",
    "granted_by": "guardian",
    "consents": {
      "data_collection": {
        "vital_signs": true,
        "location_tracking": true,
        "emotion_analysis": true,
        "conversation_recording": false
      },
      "data_sharing": {
        "family": true,
        "care_team": true,
        "emergency_services": true,
        "research_anonymized": false
      },
      "automated_actions": {
        "emergency_call": true,
        "family_notification": true,
        "medication_reminder": true
      }
    },
    "valid_from": "2024-01-01",
    "valid_until": "2024-12-31",
    "review_required": "2024-07-01"
  }
}
```

### 4.2 Authentication & Authorization

#### Device Authentication

```json
{
  "device_auth": {
    "device_id": "carebot-001",
    "certificate": "X.509 device cert",
    "attestation": "TPM-based",
    "registration": {
      "owner_id": "owner-001",
      "recipient_id": "recipient-001",
      "installation_date": "2024-01-15",
      "verified": true
    }
  }
}
```

#### Family App Authentication

```json
{
  "family_auth": {
    "method": "oauth2",
    "mfa": {
      "required": true,
      "methods": ["authenticator_app", "sms"]
    },
    "roles": {
      "primary_guardian": ["full_access", "emergency_contact"],
      "family_member": ["view", "video_call", "receive_alerts"],
      "care_worker": ["view_health", "daily_report"]
    }
  }
}
```

### 4.3 Encryption

#### End-to-End Encryption

```
Video Call: SRTP with DTLS-SRTP key exchange
Messages: Signal Protocol (Double Ratchet)
Health Data: AES-256-GCM with per-session keys
Storage: AES-256-GCM with hardware-backed keys
```

## 5. Integration Protocols

### 5.1 Hospital EMR Integration (HL7 FHIR)

```json
{
  "resourceType": "Observation",
  "id": "carebot-vital-001",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "vital-signs"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "85354-9",
      "display": "Blood pressure panel"
    }]
  },
  "subject": {
    "reference": "Patient/recipient-001"
  },
  "effectiveDateTime": "2024-01-15T10:30:00Z",
  "component": [
    {
      "code": {"coding": [{"code": "8480-6", "display": "Systolic BP"}]},
      "valueQuantity": {"value": 128, "unit": "mmHg"}
    },
    {
      "code": {"coding": [{"code": "8462-4", "display": "Diastolic BP"}]},
      "valueQuantity": {"value": 82, "unit": "mmHg"}
    }
  ],
  "device": {
    "reference": "Device/carebot-001"
  }
}
```

### 5.2 119 Emergency Integration

```json
{
  "119_dispatch_request": {
    "protocol": "CAD-XML",
    "call_id": "119-20240115-103000-001",
    "source": "WIA-CareBot",
    "incident": {
      "type": "medical_emergency",
      "subtype": "fall_elderly",
      "priority": 1
    },
    "location": {
      "address": "서울시 강남구 역삼동 123-45",
      "coordinates": {"lat": 37.5012, "lng": 127.0396},
      "access_notes": "3층 301호, 비밀번호 *1234#"
    },
    "patient": {
      "name": "김영희",
      "age": 78,
      "gender": "female",
      "conditions": ["고혈압", "당뇨"],
      "current_medications": ["아스피린", "메트포르민"],
      "allergies": ["페니실린"]
    },
    "contact": {
      "caregiver": {"name": "돌봄로봇", "phone": "1588-0000"},
      "family": {"name": "김철수", "phone": "010-1234-5678"}
    }
  }
}
```

### 5.3 Smart Home Integration (Matter Protocol)

```json
{
  "matter_integration": {
    "device_type": "carebot",
    "clusters": {
      "basic_information": true,
      "identify": true,
      "groups": true,
      "scenes": true
    },
    "custom_clusters": {
      "care_status": {
        "cluster_id": "0xFC00",
        "attributes": {
          "recipient_status": "0x0000",
          "last_interaction": "0x0001",
          "alert_level": "0x0002"
        }
      }
    },
    "automations": {
      "fall_detected": {
        "actions": ["lights_on_full", "door_unlock", "camera_record"]
      },
      "sleep_time": {
        "actions": ["lights_dim", "tv_off", "night_mode"]
      }
    }
  }
}
```

## 6. Offline & Resilience

### 6.1 Offline Capability

```json
{
  "offline_mode": {
    "local_storage": {
      "conversation_history": "7_days",
      "health_data": "30_days",
      "routine_schedule": "full"
    },
    "local_processing": {
      "emotion_detection": true,
      "fall_detection": true,
      "medication_reminder": true,
      "conversation_basic": true
    },
    "emergency_fallback": {
      "lte_sms": true,
      "local_alarm": true,
      "neighbor_beacon": true
    }
  }
}
```

### 6.2 Network Failover

```
Priority 1: WiFi (primary)
Priority 2: LTE/5G (automatic failover)
Priority 3: LoRa (emergency beacon)
Priority 4: Local mesh (neighbor CareBots)
```

## 7. Quality of Service

### 7.1 Latency Requirements

| Message Type | Target | Maximum | SLA |
|--------------|--------|---------|-----|
| Emergency SOS | 500ms | 2s | 99.99% |
| Fall Alert | 1s | 3s | 99.9% |
| Vital Alert | 3s | 10s | 99.9% |
| Video Call Setup | 2s | 5s | 99% |
| Daily Sync | 30s | 120s | 99% |

### 7.2 Reliability Guarantees

```json
{
  "reliability": {
    "emergency_messages": {
      "delivery_guarantee": "exactly_once",
      "retry_strategy": "exponential_backoff",
      "max_retries": 10,
      "fallback_channels": ["sms", "voice_call"]
    },
    "health_data": {
      "delivery_guarantee": "at_least_once",
      "local_buffer": true,
      "sync_on_reconnect": true
    }
  }
}
```

## 8. API Endpoints

### 8.1 RESTful API

```
Base URL: https://api.wia-carebot.org/v1

# Recipient Management
GET    /recipients/{id}
PUT    /recipients/{id}
GET    /recipients/{id}/health
GET    /recipients/{id}/routine

# Real-time Events
POST   /events/emotion
POST   /events/safety
POST   /events/health

# Family Interface
GET    /family/{id}/dashboard
POST   /family/{id}/video-call
GET    /family/{id}/notifications

# Emergency
POST   /emergency/sos
POST   /emergency/fall
GET    /emergency/{id}/status

# Device Management
POST   /devices/register
PUT    /devices/{id}/config
GET    /devices/{id}/status
```

### 8.2 WebSocket Endpoints

```
# Care Events Stream
wss://stream.wia-carebot.org/v1/care/{recipient_id}

# Family Dashboard Live
wss://stream.wia-carebot.org/v1/family/{family_id}

# Device Control
wss://stream.wia-carebot.org/v1/device/{device_id}
```

## 9. Error Handling

### 9.1 Error Codes

```json
{
  "error_codes": {
    "1xxx": "Authentication errors",
    "2xxx": "Authorization errors",
    "3xxx": "Validation errors",
    "4xxx": "Resource errors",
    "5xxx": "Service errors",
    "6xxx": "Communication errors",
    "7xxx": "Emergency protocol errors"
  },
  "example": {
    "code": 7001,
    "message": "Emergency contact unreachable",
    "action": "fallback_to_119",
    "details": {
      "attempted_contact": "김철수",
      "attempts": 3,
      "last_attempt": "2024-01-15T10:30:30Z"
    }
  }
}
```

## 10. Compliance

### 10.1 Regulatory Compliance

- **개인정보보호법 (PIPA)**: 개인정보 수집/이용 동의
- **의료기기법**: 건강 모니터링 기기 인증
- **HIPAA**: 의료정보 보호 (국제 서비스 시)
- **노인복지법**: 노인 돌봄 서비스 기준

### 10.2 Certification Requirements

```json
{
  "certifications": {
    "required": [
      "KC (한국 전파인증)",
      "의료기기 2등급 인증",
      "개인정보보호 인증 (ISMS-P)",
      "ISO 27001"
    ],
    "recommended": [
      "HL7 FHIR 적합성",
      "Matter 인증",
      "HIPAA 준수"
    ]
  }
}
```
