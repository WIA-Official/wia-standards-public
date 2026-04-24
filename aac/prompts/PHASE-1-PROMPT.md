# Phase 1: Signal Format Standard
## Claude Code 작업 프롬프트

---

**Phase**: 1 of 4  
**목표**: 모든 AAC 센서의 출력 형식 표준화  
**난이도**: ★★★☆☆  
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 예제 파일

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"시선 추적기, 스위치, 근육 센서, 뇌파 센서...
 전부 다른 형식으로 데이터를 출력한다.

 이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

### 목표
```
AAC 센서 종류에 관계없이
모든 센서가 동일한 JSON 형식으로 데이터를 출력하도록
Signal Format Standard를 정의한다.
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 기존 AAC 센서 조사

아래 센서 유형별로 웹서치하여 실제 출력 형식을 조사하세요:

| 센서 유형 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **Eye Tracker** | Tobii, EyeTech, LC Technologies | "Tobii SDK eye tracking data format" |
| **Switch** | AbleNet, Enabling Devices | "AAC switch interface protocol" |
| **Muscle (EMG)** | MyoWare, OpenBCI | "EMG sensor data format JSON" |
| **Brain (EEG/BCI)** | OpenBCI, NeuroSky, Emotiv | "BCI brain computer interface data format" |
| **Breath** | Puff switch, Sip-and-puff | "sip and puff controller data format" |
| **Head Movement** | HeadMouse, TrackerPro | "head mouse tracking data format" |

### 2단계: 기존 표준 조사

| 표준/프로젝트 | 조사 내용 | 웹서치 키워드 |
|-------------|----------|--------------|
| **Intel ACAT** | 데이터 구조 | "Intel ACAT source code data format" |
| **OASIS OpenAAC** | 있다면 참조 | "OpenAAC standard specification" |
| **HID (Human Interface Device)** | USB HID 리포트 | "USB HID report descriptor" |
| **W3C Pointer Events** | 웹 표준 참조 | "W3C pointer events specification" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. Eye Tracker

### Tobii SDK
- 데이터 형식: [조사 내용]
- 주요 필드: gaze_point, validity, timestamp
- 좌표계: normalized (0.0 ~ 1.0) 또는 pixel

### EyeTech
- 데이터 형식: [조사 내용]
...

## 2. Switch
...

## 3. Muscle Sensor (EMG)
...

## 4. Brain Interface (EEG/BCI)
...

## 5. 공통점 분석
- 모든 센서에 공통으로 필요한 필드: [분석]
- 센서별 고유 필드: [분석]

## 6. 결론
- 표준 형식 설계 방향: [제안]
```

---

## 🏗️ 표준 설계

### 기본 구조 (제안)

```json
{
  "$schema": "https://wia.live/aac/signal/v1/schema.json",
  "version": "1.0.0",
  "type": "센서 유형",
  "device": {
    "manufacturer": "제조사",
    "model": "모델명",
    "firmware": "펌웨어 버전"
  },
  "timestamp": "ISO 8601 또는 Unix timestamp",
  "sequence": "시퀀스 번호",
  "data": {
    "센서별 고유 데이터"
  },
  "meta": {
    "confidence": "신뢰도 (0.0 ~ 1.0)",
    "raw": "원본 데이터 (선택)"
  }
}
```

### 센서별 `data` 필드 정의

#### Eye Tracker
```json
{
  "data": {
    "gaze": {
      "x": 0.45,          // 0.0 ~ 1.0 (normalized)
      "y": 0.32,          // 0.0 ~ 1.0 (normalized)
      "z": null           // 3D 지원 시
    },
    "fixation": {
      "active": true,
      "duration_ms": 850,
      "target_id": "key_A"  // 선택 대상 ID
    },
    "pupil": {
      "left_diameter_mm": 3.5,
      "right_diameter_mm": 3.4
    },
    "blink": {
      "detected": false,
      "duration_ms": 0
    }
  }
}
```

#### Switch
```json
{
  "data": {
    "switch_id": 1,
    "channel": "primary",
    "state": "pressed",       // "pressed", "released", "held"
    "duration_ms": 150,
    "pressure": null          // 압력 감지 지원 시 (0.0 ~ 1.0)
  }
}
```

#### Muscle Sensor (EMG)
```json
{
  "data": {
    "channel_id": 1,
    "muscle_group": "cheek_left",
    "activation": 0.75,       // 0.0 ~ 1.0 (normalized)
    "raw_mv": 125.5,          // 밀리볼트 (선택)
    "threshold_exceeded": true,
    "gesture": "single_twitch"  // 인식된 제스처
  }
}
```

#### Brain Interface (EEG/BCI)
```json
{
  "data": {
    "channel_count": 8,
    "channels": [
      {"id": "Fp1", "value_uv": 12.5},
      {"id": "Fp2", "value_uv": 11.8},
      ...
    ],
    "bands": {
      "delta": 0.15,    // 0.0 ~ 1.0 (relative power)
      "theta": 0.22,
      "alpha": 0.35,
      "beta": 0.18,
      "gamma": 0.10
    },
    "classification": {
      "intent": "select",     // 인식된 의도
      "confidence": 0.82
    }
  }
}
```

#### Breath (Sip-and-Puff)
```json
{
  "data": {
    "action": "sip",          // "sip", "puff", "hard_sip", "hard_puff"
    "pressure_kpa": 2.5,
    "duration_ms": 300,
    "intensity": "soft"       // "soft", "hard"
  }
}
```

#### Head Movement
```json
{
  "data": {
    "position": {
      "x": 0.55,              // 0.0 ~ 1.0 (normalized)
      "y": 0.48
    },
    "rotation": {
      "pitch": 5.2,           // degrees
      "yaw": -3.1,
      "roll": 0.5
    },
    "gesture": "dwell",       // "dwell", "nod", "shake"
    "dwell_time_ms": 1200
  }
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-SIGNAL-FORMAT.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 기본 구조 (Base Structure)
4. 센서별 데이터 형식 (Sensor-Specific Data)
   - Eye Tracker
   - Switch
   - Muscle Sensor (EMG)
   - Brain Interface (EEG/BCI)
   - Breath (Sip-and-Puff)
   - Head Movement
5. 확장성 (Extensibility)
6. 버전 관리 (Versioning)
7. 예제 (Examples)
8. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── wia-aac-signal-v1.schema.json      (기본 스키마)
├── eye-tracker.schema.json
├── switch.schema.json
├── muscle-sensor.schema.json
├── brain-interface.schema.json
├── breath.schema.json
└── head-movement.schema.json
```

### 4. 예제 데이터 파일
```
/examples/sample-data/
├── eye-tracker-sample.json
├── switch-sample.json
├── muscle-sensor-sample.json
├── brain-interface-sample.json
├── breath-sample.json
└── head-movement-sample.json
```

### 5. 검증 스크립트
```
/examples/validators/
├── validate.ts          (TypeScript)
├── validate.py          (Python)
└── README.md
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 6개 이상 실제 AAC 센서 데이터 형식 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-SIGNAL-FORMAT.md 작성 완료
□ JSON Schema 파일 생성 완료 (기본 + 센서별 6개)
□ 예제 데이터 파일 생성 완료 (6개)
□ 검증 스크립트 작성 완료 (TypeScript + Python)
□ 검증 스크립트로 예제 데이터 검증 통과
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 기존 센서 데이터 형식 조사
   ↓
2. /spec/RESEARCH-PHASE-1.md 작성
   ↓
3. 조사 결과 바탕으로 표준 설계
   ↓
4. /spec/PHASE-1-SIGNAL-FORMAT.md 작성
   ↓
5. JSON Schema 파일 생성
   ↓
6. 예제 데이터 파일 생성
   ↓
7. 검증 스크립트 작성 및 테스트
   ↓
8. 완료 체크리스트 확인
   ↓
9. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 실제 제품의 SDK/API 문서를 웹서치로 확인
✅ 모든 필드에 명확한 단위와 범위 명시
✅ 확장 가능한 구조로 설계 (미래 센서 유형 고려)
✅ JSON Schema는 draft-07 표준 사용
✅ 예제 데이터는 실제 사용 시나리오 반영
```

### DON'T (하지 말 것)

```
❌ 추측으로 데이터 형식 정의 (반드시 조사 후)
❌ 센서 제조사 특정 형식에 종속되는 설계
❌ 필수 필드와 선택 필드 구분 없이 작성
❌ 검증 불가능한 스키마 작성
```

---

## 📞 질문이 있을 때

작업 중 판단이 어려운 경우:

1. **기술적 결정**: 웹서치로 업계 표준/관행 확인
2. **설계 방향**: `/docs/WIA-AAC-OVERVIEW.md` 참조
3. **WIA 연동 관련**: `/docs/ISP-SCIENTIFIC-FOUNDATION.md` 등 참조

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 Eye Tracker 데이터 형식 조사**

```
검색 키워드: "Tobii eye tracking SDK data format JSON"
```

화이팅! 🤟

---

<div align="center">

**Phase 1 of 4**

Signal Format Standard

</div>
