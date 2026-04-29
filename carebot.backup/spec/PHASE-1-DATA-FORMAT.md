# WIA CareBot Phase 1: Data Format Standard

**Version 1.0.0**

---

## 1. 개요 (Overview)

WIA CareBot은 AI 돌봄 로봇을 위한 데이터 표준입니다. 고령자, 장애인, 환자 등 취약계층을 위한 AI 동반자 로봇의 상호운용성을 보장합니다.

### 목적

- **AI 대화**: 감정 인식, 대화 맥락, 개인화 프로필
- **건강 모니터링**: 바이탈 사인, 약 복용, 낙상 감지
- **일상 지원**: 루틴 관리, 알림, 인지 자극 활동
- **안전**: 긴급 호출, 위치 추적, 이상 행동 감지
- **연동**: 가족 앱, 의료 시스템, 119 응급 서비스

### 대상 로봇 유형

| 유형 | 설명 | 예시 |
|------|------|------|
| Companion | AI 대화 동반자 | 효돌, 다솜이 |
| Health Monitor | 건강 모니터링 | 실버케어봇 |
| Daily Assistant | 일상 생활 지원 | 가사도우미 로봇 |
| Cognitive Care | 인지 자극/치매 케어 | 치매예방 로봇 |
| Emergency Response | 긴급 대응 | 낙상감지 로봇 |

---

## 2. 핵심 데이터 모델

### 2.1 CareBotDevice (돌봄 로봇 기기)

```json
{
  "$schema": "https://wia-standards.org/carebot/device.schema.json",
  "device_id": "carebot-001",
  "device_name": "효돌이",
  "device_type": "companion",
  "manufacturer": "한국로봇",
  "model": "HyoDol-3",
  "firmware_version": "2.1.0",
  "capabilities": {
    "voice_interaction": true,
    "emotion_recognition": true,
    "face_recognition": true,
    "vital_monitoring": true,
    "fall_detection": true,
    "mobility": false,
    "medication_dispensing": false
  },
  "sensors": {
    "camera": { "type": "rgb_depth", "resolution": "1080p" },
    "microphone": { "type": "array", "channels": 4 },
    "speaker": { "type": "stereo", "power_watts": 5 },
    "temperature": true,
    "humidity": true,
    "motion": true,
    "pressure_mat": false
  },
  "ai_models": {
    "speech_recognition": "whisper-large-v3",
    "emotion_detection": "carebot-emotion-v2",
    "face_recognition": "arcface-r100",
    "language_model": "carebot-llm-ko-v1"
  },
  "network": {
    "wifi": true,
    "bluetooth": true,
    "lte": false,
    "emergency_sim": true
  },
  "battery": {
    "capacity_mah": 10000,
    "current_percent": 85,
    "charging": false,
    "estimated_hours": 12
  },
  "registered_at": "2025-01-15T09:00:00Z",
  "last_seen": "2025-01-15T14:30:00Z"
}
```

### 2.2 CareRecipient (돌봄 대상자)

```json
{
  "$schema": "https://wia-standards.org/carebot/recipient.schema.json",
  "recipient_id": "recipient-001",
  "profile": {
    "name": "김영희",
    "preferred_name": "어머니",
    "birth_date": "1945-03-15",
    "gender": "female",
    "blood_type": "A+",
    "photo_url": "/profiles/recipient-001.jpg"
  },
  "health": {
    "conditions": ["hypertension", "diabetes_type2", "mild_cognitive_impairment"],
    "allergies": ["penicillin"],
    "mobility_level": "assisted_walking",
    "cognitive_level": "mild_impairment",
    "hearing_level": "moderate_loss",
    "vision_level": "corrected_normal"
  },
  "medications": [
    {
      "name": "아mlodipine",
      "dosage": "5mg",
      "frequency": "once_daily",
      "time": "08:00",
      "with_food": true,
      "notes": "혈압약"
    },
    {
      "name": "메트포르민",
      "dosage": "500mg",
      "frequency": "twice_daily",
      "times": ["08:00", "18:00"],
      "with_food": true,
      "notes": "당뇨약"
    }
  ],
  "preferences": {
    "wake_time": "06:30",
    "sleep_time": "21:00",
    "preferred_language": "ko",
    "voice_speed": "slow",
    "voice_volume": "loud",
    "preferred_topics": ["family", "health", "news", "weather"],
    "music_genres": ["trot", "classical"],
    "conversation_style": "warm_formal"
  },
  "emergency_contacts": [
    {
      "name": "김철수",
      "relationship": "son",
      "phone": "+82-10-1234-5678",
      "priority": 1,
      "notify_on": ["emergency", "health_alert", "daily_summary"]
    },
    {
      "name": "이영숙",
      "relationship": "daughter",
      "phone": "+82-10-8765-4321",
      "priority": 2,
      "notify_on": ["emergency", "health_alert"]
    }
  ],
  "care_team": [
    {
      "name": "박의사",
      "role": "primary_physician",
      "hospital": "서울대병원",
      "phone": "+82-2-1234-5678",
      "speciality": "internal_medicine"
    }
  ],
  "created_at": "2025-01-01T00:00:00Z",
  "updated_at": "2025-01-15T10:00:00Z"
}
```

### 2.3 ConversationSession (대화 세션)

```json
{
  "$schema": "https://wia-standards.org/carebot/conversation.schema.json",
  "session_id": "conv-20250115-001",
  "device_id": "carebot-001",
  "recipient_id": "recipient-001",
  "started_at": "2025-01-15T09:15:00Z",
  "ended_at": "2025-01-15T09:25:00Z",
  "duration_seconds": 600,
  "conversation_type": "morning_greeting",
  "turns": [
    {
      "turn_id": 1,
      "speaker": "bot",
      "timestamp": "2025-01-15T09:15:00Z",
      "text": "어머니, 좋은 아침이에요! 오늘 날씨가 좋네요.",
      "audio_url": "/audio/conv-001-turn-1.wav",
      "emotion_expressed": "cheerful",
      "intent": "greeting"
    },
    {
      "turn_id": 2,
      "speaker": "recipient",
      "timestamp": "2025-01-15T09:15:05Z",
      "text": "응, 좋은 아침. 오늘 무슨 요일이지?",
      "audio_url": "/audio/conv-001-turn-2.wav",
      "emotion_detected": {
        "primary": "neutral",
        "confidence": 0.85,
        "valence": 0.6,
        "arousal": 0.4
      },
      "speech_analysis": {
        "clarity": 0.75,
        "pace": "normal",
        "volume": "normal"
      }
    },
    {
      "turn_id": 3,
      "speaker": "bot",
      "timestamp": "2025-01-15T09:15:10Z",
      "text": "오늘은 수요일이에요, 어머니. 아침 약 드실 시간이에요. 혈압약과 당뇨약 챙겨드릴게요.",
      "emotion_expressed": "caring",
      "intent": "medication_reminder"
    }
  ],
  "context": {
    "previous_session_id": "conv-20250114-003",
    "ongoing_topics": ["medication", "family_visit"],
    "mood_trend": "stable"
  },
  "analysis": {
    "engagement_score": 0.85,
    "cognitive_indicators": {
      "orientation_time": "partial",
      "orientation_place": "good",
      "memory_short_term": "good",
      "attention": "good"
    },
    "emotional_summary": "stable_positive",
    "concerns": []
  }
}
```

### 2.4 EmotionState (감정 상태)

```json
{
  "$schema": "https://wia-standards.org/carebot/emotion.schema.json",
  "timestamp": "2025-01-15T14:30:00Z",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "detection_method": "multimodal",
  "primary_emotion": {
    "category": "happy",
    "confidence": 0.82,
    "intensity": 0.65
  },
  "secondary_emotions": [
    { "category": "content", "confidence": 0.45 }
  ],
  "dimensional": {
    "valence": 0.7,
    "arousal": 0.5,
    "dominance": 0.6
  },
  "facial_analysis": {
    "detected": true,
    "landmarks_quality": 0.95,
    "expressions": {
      "smile": 0.75,
      "eye_openness": 0.8,
      "brow_raise": 0.2
    }
  },
  "voice_analysis": {
    "detected": true,
    "pitch_mean_hz": 180,
    "pitch_variation": 0.15,
    "energy": 0.6,
    "speech_rate": "normal"
  },
  "behavioral_context": {
    "activity": "watching_tv",
    "social_context": "alone",
    "time_of_day": "afternoon"
  },
  "trend": {
    "vs_1_hour_ago": "stable",
    "vs_yesterday": "improved",
    "weekly_average": "positive"
  }
}
```

### 2.5 HealthMonitoring (건강 모니터링)

```json
{
  "$schema": "https://wia-standards.org/carebot/health.schema.json",
  "timestamp": "2025-01-15T08:00:00Z",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "vital_signs": {
    "heart_rate": {
      "value": 72,
      "unit": "bpm",
      "status": "normal",
      "measured_by": "wearable_watch"
    },
    "blood_pressure": {
      "systolic": 128,
      "diastolic": 82,
      "unit": "mmHg",
      "status": "slightly_elevated",
      "measured_by": "smart_cuff"
    },
    "blood_glucose": {
      "value": 145,
      "unit": "mg/dL",
      "fasting": true,
      "status": "elevated",
      "measured_by": "cgm_sensor"
    },
    "body_temperature": {
      "value": 36.5,
      "unit": "celsius",
      "status": "normal",
      "measured_by": "infrared_sensor"
    },
    "oxygen_saturation": {
      "value": 97,
      "unit": "percent",
      "status": "normal",
      "measured_by": "pulse_oximeter"
    }
  },
  "activity": {
    "steps_today": 1250,
    "active_minutes": 25,
    "sedentary_hours": 5.5,
    "sleep_hours_last_night": 7.2,
    "sleep_quality": "good"
  },
  "medication_adherence": {
    "morning_taken": true,
    "morning_time": "08:05:00",
    "evening_taken": false,
    "adherence_rate_weekly": 0.92
  },
  "alerts": [
    {
      "type": "blood_glucose_elevated",
      "severity": "warning",
      "message": "공복 혈당이 정상범위(70-100)보다 높습니다.",
      "recommendation": "식단 조절과 운동을 권장합니다."
    }
  ],
  "trends": {
    "blood_pressure_7day": "stable",
    "blood_glucose_7day": "slightly_increasing",
    "activity_7day": "decreasing"
  }
}
```

### 2.6 DailyRoutine (일과 루틴)

```json
{
  "$schema": "https://wia-standards.org/carebot/routine.schema.json",
  "date": "2025-01-15",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "schedule": [
    {
      "time": "06:30",
      "activity": "wake_up",
      "type": "routine",
      "status": "completed",
      "actual_time": "06:35",
      "notes": "5분 늦게 기상"
    },
    {
      "time": "07:00",
      "activity": "morning_exercise",
      "type": "health",
      "status": "completed",
      "duration_minutes": 15,
      "exercise_type": "stretching"
    },
    {
      "time": "08:00",
      "activity": "breakfast_and_medication",
      "type": "health",
      "status": "completed",
      "medication_taken": true
    },
    {
      "time": "10:00",
      "activity": "cognitive_game",
      "type": "cognitive",
      "status": "completed",
      "game_type": "memory_match",
      "score": 85,
      "duration_minutes": 20
    },
    {
      "time": "12:00",
      "activity": "lunch",
      "type": "routine",
      "status": "completed"
    },
    {
      "time": "14:00",
      "activity": "video_call_family",
      "type": "social",
      "status": "scheduled",
      "with": "김철수 (아들)"
    },
    {
      "time": "15:00",
      "activity": "afternoon_walk",
      "type": "health",
      "status": "pending",
      "weather_dependent": true
    },
    {
      "time": "18:00",
      "activity": "dinner_and_medication",
      "type": "health",
      "status": "pending"
    },
    {
      "time": "20:00",
      "activity": "evening_conversation",
      "type": "social",
      "status": "pending",
      "topic_suggestions": ["오늘 하루 이야기", "내일 계획"]
    },
    {
      "time": "21:00",
      "activity": "sleep_preparation",
      "type": "routine",
      "status": "pending"
    }
  ],
  "special_events": [
    {
      "time": "14:00",
      "event": "family_video_call",
      "participants": ["김철수"],
      "reminder_sent": true
    }
  ],
  "summary": {
    "completed_activities": 5,
    "pending_activities": 5,
    "medication_adherence": true,
    "social_interactions": 1,
    "cognitive_activities": 1,
    "physical_activities": 1
  }
}
```

### 2.7 SafetyEvent (안전 이벤트)

```json
{
  "$schema": "https://wia-standards.org/carebot/safety.schema.json",
  "event_id": "safety-20250115-001",
  "timestamp": "2025-01-15T15:30:45Z",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "event_type": "fall_detected",
  "severity": "high",
  "location": {
    "room": "bathroom",
    "coordinates": { "x": 2.5, "y": 3.0 },
    "confidence": 0.92
  },
  "detection": {
    "method": "motion_sensor_anomaly",
    "confidence": 0.88,
    "supporting_evidence": [
      "sudden_acceleration",
      "impact_detected",
      "no_movement_30s"
    ]
  },
  "recipient_response": {
    "verbal_response": false,
    "movement_detected": false,
    "button_press": false,
    "response_latency_seconds": null
  },
  "automated_actions": [
    {
      "action": "verbal_check",
      "timestamp": "2025-01-15T15:30:50Z",
      "message": "어머니, 괜찮으세요? 대답해 주세요.",
      "response_received": false
    },
    {
      "action": "alert_family",
      "timestamp": "2025-01-15T15:31:05Z",
      "contacts_notified": ["김철수"],
      "notification_method": "push_and_call"
    },
    {
      "action": "alert_emergency",
      "timestamp": "2025-01-15T15:31:30Z",
      "service": "119",
      "call_initiated": true
    }
  ],
  "resolution": {
    "status": "resolved",
    "resolved_at": "2025-01-15T15:35:00Z",
    "resolved_by": "family_member",
    "outcome": "minor_injury",
    "follow_up_required": true,
    "notes": "화장실에서 미끄러짐. 가벼운 타박상."
  }
}
```

### 2.8 CognitiveAssessment (인지 평가)

```json
{
  "$schema": "https://wia-standards.org/carebot/cognitive.schema.json",
  "assessment_id": "cog-20250115-001",
  "timestamp": "2025-01-15T10:00:00Z",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "assessment_type": "daily_informal",
  "domains": {
    "orientation": {
      "time": {
        "score": 0.75,
        "details": "요일 혼동"
      },
      "place": {
        "score": 1.0,
        "details": "정상"
      },
      "person": {
        "score": 1.0,
        "details": "정상"
      }
    },
    "memory": {
      "immediate": {
        "score": 0.9,
        "task": "3_word_recall_immediate"
      },
      "short_term": {
        "score": 0.7,
        "task": "3_word_recall_5min"
      },
      "long_term": {
        "score": 0.95,
        "task": "biographical_questions"
      }
    },
    "attention": {
      "sustained": {
        "score": 0.8,
        "task": "counting_backward"
      },
      "divided": {
        "score": 0.65,
        "task": "dual_task"
      }
    },
    "language": {
      "comprehension": {
        "score": 0.9,
        "details": "정상"
      },
      "expression": {
        "score": 0.85,
        "details": "간헐적 단어 찾기 어려움"
      },
      "naming": {
        "score": 0.9,
        "task": "object_naming"
      }
    },
    "executive_function": {
      "planning": {
        "score": 0.7,
        "task": "simple_planning"
      },
      "problem_solving": {
        "score": 0.75,
        "task": "daily_scenario"
      }
    }
  },
  "overall_score": 0.82,
  "trend_vs_last_week": "stable",
  "trend_vs_last_month": "slight_decline",
  "concerns": [
    {
      "domain": "orientation_time",
      "observation": "요일 혼동이 증가하는 경향",
      "recommendation": "달력 확인 루틴 강화"
    }
  ],
  "activities_recommended": [
    {
      "type": "memory_game",
      "frequency": "twice_daily",
      "duration_minutes": 10
    },
    {
      "type": "calendar_review",
      "frequency": "morning_evening",
      "duration_minutes": 5
    }
  ]
}
```

### 2.9 FamilyNotification (가족 알림)

```json
{
  "$schema": "https://wia-standards.org/carebot/notification.schema.json",
  "notification_id": "notif-20250115-001",
  "timestamp": "2025-01-15T18:00:00Z",
  "recipient_id": "recipient-001",
  "device_id": "carebot-001",
  "notification_type": "daily_summary",
  "priority": "normal",
  "recipients": [
    {
      "contact_id": "contact-001",
      "name": "김철수",
      "delivery_method": "push_notification",
      "delivered": true,
      "read": true,
      "read_at": "2025-01-15T18:15:00Z"
    }
  ],
  "content": {
    "title": "어머니 오늘 하루 요약",
    "summary": "전반적으로 좋은 하루를 보내셨습니다.",
    "highlights": [
      "아침 운동 15분 완료",
      "약 복용 잘 하셨습니다",
      "기억력 게임 85점 달성"
    ],
    "concerns": [
      "공복 혈당이 약간 높았습니다 (145 mg/dL)"
    ],
    "mood_summary": "안정적이고 긍정적",
    "activity_summary": {
      "steps": 1250,
      "active_minutes": 25,
      "social_interactions": 2,
      "cognitive_activities": 1
    },
    "tomorrow_schedule": [
      "10:00 - 병원 방문 (내과 정기검진)",
      "14:00 - 경로당 모임"
    ]
  },
  "media": {
    "photo_of_day": "/media/recipient-001/2025-01-15/smile.jpg",
    "conversation_highlights_audio": "/media/recipient-001/2025-01-15/highlights.mp3"
  },
  "actions_available": [
    {
      "action": "video_call",
      "label": "영상통화 하기"
    },
    {
      "action": "send_message",
      "label": "메시지 보내기"
    },
    {
      "action": "view_details",
      "label": "상세 보기"
    }
  ]
}
```

---

## 3. 열거형 정의 (Enumerations)

### 3.1 DeviceType (기기 유형)
```
companion, health_monitor, daily_assistant, cognitive_care, emergency_response, multipurpose
```

### 3.2 CognitiveLevel (인지 수준)
```
normal, mild_impairment, moderate_impairment, severe_impairment
```

### 3.3 MobilityLevel (이동 수준)
```
independent, assisted_walking, wheelchair, bedridden
```

### 3.4 EmotionCategory (감정 범주)
```
happy, sad, angry, fearful, surprised, disgusted, neutral, content, anxious, confused, lonely
```

### 3.5 SafetyEventType (안전 이벤트 유형)
```
fall_detected, wandering_detected, abnormal_activity, no_movement_extended, sos_button, medication_missed, vital_sign_alert, environmental_hazard
```

### 3.6 AlertSeverity (알림 심각도)
```
info, warning, urgent, critical, emergency
```

### 3.7 ConversationType (대화 유형)
```
morning_greeting, medication_reminder, health_check, social_chat, cognitive_exercise, emergency_response, night_check, family_message
```

---

## 4. 개인정보 보호 (Privacy & Ethics)

### 4.1 데이터 분류

| 민감도 | 데이터 유형 | 보호 수준 |
|--------|------------|----------|
| 매우 높음 | 얼굴 이미지, 의료 정보 | AES-256 암호화, 접근 로그 |
| 높음 | 대화 녹음, 위치 정보 | 암호화, 동의 기반 접근 |
| 중간 | 활동 데이터, 루틴 | 암호화 저장 |
| 낮음 | 기기 상태, 환경 센서 | 기본 보호 |

### 4.2 동의 모델

```json
{
  "consent_id": "consent-001",
  "recipient_id": "recipient-001",
  "guardian_id": "guardian-001",
  "consents": {
    "voice_recording": {
      "granted": true,
      "granted_by": "guardian",
      "granted_at": "2025-01-01T00:00:00Z",
      "purpose": "conversation_improvement"
    },
    "face_recognition": {
      "granted": true,
      "granted_by": "recipient",
      "granted_at": "2025-01-01T00:00:00Z",
      "purpose": "emotion_detection"
    },
    "health_data_sharing": {
      "granted": true,
      "granted_by": "guardian",
      "granted_at": "2025-01-01T00:00:00Z",
      "share_with": ["primary_physician", "family_app"]
    },
    "location_tracking": {
      "granted": true,
      "granted_by": "guardian",
      "granted_at": "2025-01-01T00:00:00Z",
      "scope": "home_only"
    }
  },
  "data_retention": {
    "conversation_audio": "30_days",
    "health_data": "1_year",
    "activity_logs": "90_days"
  }
}
```

---

## 5. 참고문헌

1. [ISO/IEC 27701:2019](https://www.iso.org/standard/71670.html) - Privacy Information Management
2. [HL7 FHIR R4](https://www.hl7.org/fhir/) - Healthcare Data Interoperability
3. [IEEE 7010-2020](https://standards.ieee.org/ieee/7010/7718/) - Wellbeing Metrics for AI
4. [개인정보보호법](https://www.law.go.kr/법령/개인정보보호법) - 한국 개인정보 보호
5. [노인복지법](https://www.law.go.kr/법령/노인복지법) - 한국 노인 복지

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-15
**Status**: Draft

---

<div align="center">

**WIA CareBot Data Format Standard**

AI Care Companion for All

弘益人間 - Benefit All Humanity

</div>
