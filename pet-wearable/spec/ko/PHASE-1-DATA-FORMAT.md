# WIA-PET-007 PHASE 1: 데이터 형식 사양

**버전:** 1.0.0  
**날짜:** 2025-12-25  
**상태:** 활성 표준

---

## 1. 개요

PHASE 1은 WIA-PET-007 반려동물 웨어러블 기기의 핵심 데이터 형식 및 구조를 정의합니다. 모든 호환 기기는 데이터 저장, 전송 및 API 통신에 이러한 표준화된 JSON 형식을 사용해야 합니다.

### 1.1 설계 원칙

- **사람이 읽을 수 있음:** 쉬운 검사 및 디버깅을 위한 JSON 형식
- **자기 설명적:** 명확한 필드 이름 및 구조
- **확장 가능:** 향후 추가 사항과 호환
- **압축:** 대역폭 효율성을 위한 최적화
- **버전 관리:** 호환성을 위한 스키마 버전 추적

---

## 2. 기본 데이터 구조

모든 WIA-PET-007 데이터 객체는 공통 기본 구조를 공유합니다:

```json
{
  "schemaVersion": "1.0.0",
  "standard": "WIA-PET-007",
  "deviceId": "PW-{SPECIES}-{SERIAL}",
  "timestamp": "ISO8601 UTC",
  "petProfile": { },
  "metadata": { }
}
```

---

## 3. 반려동물 프로필 형식

```json
{
  "petProfile": {
    "petId": "PET-UUID-12345",
    "name": "맥스",
    "species": "dog",
    "breed": "골든 리트리버",
    "birthdate": "YYYY-MM-DD",
    "gender": "male|female|neutered_male|spayed_female",
    "weight": 30.5,
    "weightUnit": "kg|lbs",
    "microchipId": "985112345678901",
    "color": "골든",
    "distinguishingMarks": "가슴에 흰색 반점"
  }
}
```

### 3.1 종 코드

**표준 종 코드:**
- `dog` - 개 (Canis familiaris)
- `cat` - 고양이 (Felis catus)
- `rabbit` - 토끼 (Oryctolagus cuniculus)
- `ferret` - 페럿 (Mustela putorius furo)
- `other` - 기타 반려동물

---

## 4. 건강 데이터 형식

```json
{
  "healthData": {
    "timestamp": "2025-12-25T10:30:00.000Z",
    "heartRate": {
      "value": 95,
      "unit": "bpm",
      "quality": 92,
      "qualityScale": "0-100",
      "context": "resting|light|moderate|vigorous"
    },
    "temperature": {
      "value": 38.5,
      "unit": "celsius|fahrenheit",
      "quality": 88,
      "qualityScale": "0-100",
      "measurementSite": "collar_contact|rectal|ear"
    },
    "respiratoryRate": {
      "value": 22,
      "unit": "breaths_per_minute",
      "quality": 78,
      "qualityScale": "0-100",
      "measurementDuration": 60
    }
  }
}
```

### 4.1 품질 점수 정의

품질 점수 (0-100)는 데이터 신뢰성을 나타냅니다:

- **90-100:** 우수 - 임상 등급 정확도
- **70-89:** 양호 - 웰니스 추적에 적합
- **50-69:** 보통 - 일반 추세만
- **0-49:** 불량 - 폐기해야 함

---

## 5. 활동 데이터 형식

```json
{
  "activityData": {
    "date": "YYYY-MM-DD",
    "summary": {
      "totalSteps": 8523,
      "distance": {"value": 6.2, "unit": "kilometers"},
      "calories": {"value": 245, "unit": "kcal"},
      "activeMinutes": 87,
      "restMinutes": 1353,
      "goalAchievement": {
        "steps": 85,
        "activeMinutes": 72,
        "unit": "percent"
      }
    },
    "activities": [
      {
        "startTime": "ISO8601",
        "type": "walking|running|playing|resting",
        "duration": 900,
        "steps": 1200,
        "distance": 0.8,
        "calories": 45
      }
    ]
  }
}
```

---

## 6. 위치 데이터 형식

```json
{
  "locationData": {
    "timestamp": "ISO8601",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 35.0,
      "accuracy": 5.0,
      "unit": "meters"
    },
    "source": "gps|wifi|cell|bluetooth",
    "satellites": 12,
    "geofenceStatus": {
      "insideZone": true,
      "zoneName": "홈 안전 구역",
      "zoneId": "FENCE-HOME-001"
    }
  }
}
```

---

## 7. 경고 데이터 형식

```json
{
  "alert": {
    "alertId": "ALERT-UUID",
    "timestamp": "ISO8601",
    "severity": "critical|high|medium|low",
    "type": "health|geofence|battery|device",
    "metric": {
      "name": "heartRate",
      "value": 180,
      "unit": "bpm",
      "normalRange": "60-120"
    },
    "recommendation": "면밀히 모니터링하세요. 30분 이상 지속되면 수의사에게 연락하세요."
  }
}
```

---

## 8. 배터리 및 기기 상태 형식

```json
{
  "deviceStatus": {
    "battery": {
      "level": 87,
      "charging": false,
      "timeToEmpty": 9.5,
      "timeUnit": "days"
    },
    "connectivity": {
      "bluetooth": {"connected": true, "signalStrength": -65},
      "gps": {"available": true, "satellites": 12}
    },
    "firmware": {
      "version": "2.3.1",
      "lastUpdate": "ISO8601"
    }
  }
}
```

---

## 9. 유효성 검증 규칙

### 9.1 필수 필드 검증

모든 `required: true` 필드는 존재하고 null이 아니어야 합니다.

### 9.2 범위 검증

숫자 값은 지정된 범위 내에 있어야 합니다.

### 9.3 형식 검증

- 타임스탬프는 유효한 ISO 8601이어야 함
- 기기 ID는 패턴과 일치해야 함: `^PW-[A-Z]+-[0-9A-Z]+$`

---

**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**

© 2025 SmileStory Inc. / WIA  
WIA-PET-007 PHASE 1 사양
