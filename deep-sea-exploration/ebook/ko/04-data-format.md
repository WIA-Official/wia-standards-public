# 제4장: 데이터 형식 사양 (Phase 1)

## 해양학 데이터와 차량 원격측정을 위한 표준화된 형식

---

## 4.1 기본 메시지 형식

### 범용 메시지 구조

모든 WIA 심해 탐사 메시지는 추적 가능성, 검증, 상호운용성을 보장하는 일관된 기본 구조를 따릅니다.

**완전한 기본 메시지 스키마**:

```json
{
  "wiaVersion": "1.0",
  "messageType": "OCEANOGRAPHIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "NORMAL",
  "payload": { ... },
  "metadata": { ... },
  "checksum": "SHA256:abc123..."
}
```

### 필드 사양

| 필드 | 유형 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| wiaVersion | 문자열 | 예 | 표준 버전 | "1.0" |
| messageType | 열거형 | 예 | 메시지 유형 | "OCEANOGRAPHIC_DATA" |
| timestamp | ISO8601 | 예 | 밀리초 포함 UTC | "2025-01-15T14:30:00.123Z" |
| sequenceNumber | 정수 | 예 | 단조 카운터 | 12345 |
| sourceId | 문자열 | 예 | 고유 소스 식별자 | "ROV-ATLANTIS-001" |
| priority | 열거형 | 예 | 전송 우선순위 | "NORMAL" |
| payload | 객체 | 예 | 메시지별 내용 | {...} |
| metadata | 객체 | 아니오 | 선택적 컨텍스트 | {...} |
| checksum | 문자열 | 예 | SHA256 해시 | "SHA256:abc123..." |

### 소스 ID 규약

`sourceId` 필드는 계층적 명명 규약을 사용합니다:

```
[유형]-[플랫폼]-[인스턴스]

예시:
ROV-JASON-002          (Jason이라는 ROV, 유닛 2)
AUV-SENTRY-001         (Sentry라는 AUV, 유닛 1)
SENSOR-CTD-003         (CTD 센서, 유닛 3)
SHIP-ISABU-001         (이사부호 연구선)
```

### 타임스탬프 요구사항

모든 타임스탬프는 다음을 충족해야 합니다:
- UTC 시간대 (현지 시간 없음)
- 'Z' 접미사가 있는 ISO8601 형식
- 최소 밀리초 정밀도
- NTP 또는 GPS 시간 소스에 동기화

---

## 4.2 해양학 데이터 패킷

### CTD 및 환경 데이터

해양학 데이터 패킷은 CTD(전도도, 온도, 깊이) 및 관련 센서로 측정된 수주 특성을 캡처합니다.

**완전한 해양학 데이터 메시지**:

```json
{
  "wiaVersion": "1.0",
  "messageType": "OCEANOGRAPHIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "NORMAL",
  "payload": {
    "location": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5,
      "coordinateSystem": "WGS84",
      "accuracy": {
        "horizontal": 2.3,
        "vertical": 0.8,
        "unit": "meters"
      }
    },
    "environment": {
      "temperature": {
        "value": 2.47,
        "unit": "celsius",
        "sensorId": "TEMP-01",
        "calibrationDate": "2024-12-01",
        "accuracy": 0.001,
        "qualityFlag": "GOOD"
      },
      "pressure": {
        "value": 354.72,
        "unit": "bar",
        "sensorId": "PRESS-01",
        "qualityFlag": "GOOD"
      },
      "salinity": {
        "value": 34.89,
        "unit": "PSU",
        "sensorId": "SAL-01",
        "qualityFlag": "GOOD"
      },
      "dissolvedOxygen": {
        "value": 6.23,
        "unit": "mg/L",
        "sensorId": "DO-01",
        "qualityFlag": "GOOD"
      },
      "pH": {
        "value": 7.82,
        "unit": "pH",
        "sensorId": "PH-01",
        "qualityFlag": "GOOD"
      }
    }
  },
  "metadata": {
    "mission": "HYDROTHERMAL-SURVEY-2025-01",
    "institution": "KIOST",
    "researchVessel": "이사부호",
    "chiefScientist": "Dr. 김해양"
  },
  "checksum": "SHA256:e3b0c44298fc..."
}
```

### 품질 플래그

WIA 표준은 국제 규약과 정렬된 품질 플래그를 사용합니다:

| 플래그 값 | 설명 | 조치 |
|----------|------|------|
| GOOD | 데이터가 모든 QC 통과 | 제한 없이 사용 |
| PROBABLY_GOOD | 경미한 QC 문제 | 인지하고 사용 |
| PROBABLY_BAD | QC 우려 | 주의하여 사용 |
| BAD | QC 실패 | 사용하지 않음 |
| NOT_APPLIED | QC 수행되지 않음 | 사용 전 QC 적용 |
| INTERPOLATED | 공백 채움 데이터 | 인지하고 사용 |
| MISSING | 데이터 없음 | 해당 없음 |

### 매개변수 검증 범위

| 매개변수 | 최소 | 최대 | 단위 | 참고 |
|---------|------|------|------|------|
| 깊이 | 0 | 11,000 | 미터 | 전체 해양 깊이 |
| 온도 | -2 | 400 | °C | 분출구 유체 포함 |
| 압력 | 0 | 1,100 | bar | 전체 해양 깊이 |
| 염분 | 0 | 50 | PSU | 과염분 포함 |
| pH | 0 | 14 | pH | 전체 범위 |
| 용존 산소 | 0 | 20 | mg/L | 과포화 |
| 위도 | -90 | 90 | 도 | WGS84 |
| 경도 | -180 | 180 | 도 | WGS84 |

---

## 4.3 수심측량 데이터

### 다중빔 소나 데이터

다중빔 에코사운더의 수심측량 데이터는 소나 구성 및 처리 매개변수에 대한 상세한 메타데이터가 필요합니다.

**수심측량 데이터 메시지**:

```json
{
  "wiaVersion": "1.0",
  "messageType": "BATHYMETRIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sourceId": "AUV-SENTRY-042",
  "payload": {
    "surveyArea": {
      "boundingBox": {
        "northWest": {"lat": 36.8000, "lon": -121.8500},
        "southEast": {"lat": 36.7900, "lon": -121.8400}
      },
      "gridResolution": 1.0,
      "gridUnit": "meters",
      "coordinateSystem": "WGS84",
      "verticalDatum": "MSL"
    },
    "sonarConfiguration": {
      "type": "MULTIBEAM",
      "manufacturer": "Kongsberg",
      "model": "EM304",
      "frequency": {
        "center": 30,
        "unit": "kHz"
      },
      "beamWidth": {
        "alongTrack": 1.0,
        "acrossTrack": 1.0,
        "unit": "degrees"
      }
    },
    "soundVelocityProfile": {
      "profileId": "SVP-2025-01-15-001",
      "profiles": [
        {"depth": 0, "velocity": 1520.3},
        {"depth": 100, "velocity": 1510.2},
        {"depth": 1000, "velocity": 1485.0}
      ]
    },
    "soundings": [
      {
        "latitude": 36.7977,
        "longitude": -121.8472,
        "depth": 3547.2,
        "uncertainty": 0.3,
        "quality": "VERIFIED"
      }
    ],
    "statistics": {
      "totalSoundings": 125000,
      "validSoundings": 123500,
      "minDepth": 3480.2,
      "maxDepth": 3612.8
    }
  }
}
```

---

## 4.4 샘플 수집 메타데이터

### 생물 샘플

```json
{
  "wiaVersion": "1.0",
  "messageType": "SAMPLE_METADATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sourceId": "ROV-JASON-007",
  "priority": "HIGH",
  "payload": {
    "sampleId": "SAMPLE-2025-01-15-001",
    "sampleType": "BIOLOGICAL",
    "category": "MACROFAUNA",
    "collectionMethod": {
      "type": "MANIPULATOR_ARM",
      "tool": "SUCTION_SAMPLER",
      "duration": 45,
      "durationUnit": "seconds"
    },
    "location": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "coordinateSystem": "WGS84"
    },
    "habitat": {
      "type": "HYDROTHERMAL_VENT",
      "zone": "DIFFUSE_FLOW",
      "substrate": "BASALT"
    },
    "specimen": {
      "taxonCandidate": "Riftia pachyptila",
      "commonName": "거대 튜브웜",
      "lifeStage": "ADULT",
      "count": 3,
      "measurements": {
        "length": {"value": 1.2, "unit": "meters"},
        "weight": {"value": 450, "unit": "grams"}
      },
      "condition": "LIVE",
      "description": "튜브웜 군락, 약 1.2m 길이, 건강한 색상"
    },
    "preservation": {
      "method": "ETHANOL_95",
      "container": {
        "type": "BIO_BOX",
        "containerId": "BB-07"
      }
    },
    "collector": {
      "name": "Dr. 김해양",
      "institution": "한국해양과학기술원",
      "orcid": "0000-0002-1234-5678"
    },
    "permits": [
      {"type": "해양수산부", "number": "MOF-PERMIT-2025-001"}
    ],
    "chainOfCustody": [
      {
        "action": "COLLECTED",
        "timestamp": "2025-01-15T14:30:00.000Z",
        "personnel": "Dr. 김해양",
        "location": "IN_SITU"
      }
    ]
  }
}
```

### 지질 샘플

```json
{
  "sampleType": "GEOLOGICAL",
  "category": "ROCK",
  "specimen": {
    "rockType": "BASALT",
    "formation": "PILLOW_LAVA",
    "mineralogy": ["OLIVINE", "PYROXENE", "PLAGIOCLASE"],
    "texture": "GLASSY_RIND",
    "alteration": "MINIMAL",
    "measurements": {
      "dimensions": {"x": 15, "y": 12, "z": 8, "unit": "centimeters"},
      "weight": {"value": 2.3, "unit": "kilograms"}
    }
  }
}
```

---

## 4.5 차량 원격측정 형식

### 종합 차량 상태

```json
{
  "wiaVersion": "1.0",
  "messageType": "VEHICLE_TELEMETRY",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sourceId": "ROV-HEMIRE-001",
  "priority": "HIGH",
  "payload": {
    "vehicle": {
      "type": "ROV",
      "class": "WORK_CLASS",
      "manufacturer": "한국해양대학교",
      "model": "해미래",
      "maxDepthRating": 6000,
      "maxDepthRatingUnit": "meters"
    },
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5,
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7,
      "positionSource": "USBL"
    },
    "velocity": {
      "forward": 0.5,
      "lateral": 0.1,
      "vertical": -0.02,
      "unit": "m/s",
      "source": "DVL"
    },
    "propulsion": {
      "thrusterStatus": [
        {"id": "FORE_PORT", "power": 45, "rpm": 1200, "status": "OPERATIONAL"},
        {"id": "FORE_STBD", "power": 45, "rpm": 1200, "status": "OPERATIONAL"},
        {"id": "VERT_FORE", "power": 60, "rpm": 1500, "status": "OPERATIONAL"},
        {"id": "VERT_AFT", "power": 58, "rpm": 1480, "status": "OPERATIONAL"}
      ]
    },
    "power": {
      "source": "TETHER",
      "inputVoltage": 3000,
      "inputPower": 75900,
      "batteryBackup": {
        "voltage": 48.2,
        "capacity": 85,
        "capacityUnit": "percent"
      }
    },
    "systems": {
      "hydraulics": {
        "pressure": 3000,
        "status": "NOMINAL"
      },
      "cameras": [
        {"id": "HD_MAIN", "status": "ACTIVE", "recording": true},
        {"id": "HD_ZOOM", "status": "ACTIVE", "zoom": 5.2}
      ],
      "lights": [
        {"id": "MAIN_ARRAY", "intensity": 75, "status": "ON"}
      ],
      "manipulators": {
        "port": {"position": "DEPLOYED", "grip": "OPEN"},
        "starboard": {"position": "STOWED", "grip": "CLOSED"}
      }
    },
    "communication": {
      "tether": {
        "status": "CONNECTED",
        "bandwidth": 1000,
        "bandwidthUnit": "Mbps"
      }
    },
    "alerts": [],
    "missionStatus": {
      "currentWaypoint": 5,
      "totalWaypoints": 12,
      "missionElapsed": 14400,
      "missionElapsedUnit": "seconds"
    }
  }
}
```

---

## 4.6 비디오 및 이미지 메타데이터

```json
{
  "wiaVersion": "1.0",
  "messageType": "VIDEO_METADATA",
  "payload": {
    "mediaType": "VIDEO",
    "recording": {
      "id": "VID-2025-01-15-001",
      "startTime": "2025-01-15T10:00:00.000Z",
      "duration": 16200,
      "format": "H.265",
      "resolution": {"width": 3840, "height": 2160},
      "frameRate": 30
    },
    "camera": {
      "id": "HD_MAIN",
      "manufacturer": "DeepSea Power & Light",
      "model": "4K Ultra"
    },
    "annotations": [
      {
        "frameNumber": 125000,
        "type": "SPECIES_ID",
        "label": "Bathynomus giganteus",
        "confidence": 0.92
      }
    ],
    "storage": {
      "filename": "VID-2025-01-15-001.mp4",
      "size": 50856345600,
      "checksum": "SHA256:..."
    }
  }
}
```

---

## 4.7 저대역폭용 이진 형식

### 이진 헤더 (32 바이트)

대역폭이 극도로 제한된 음향 통신의 경우:

```
오프셋  크기  유형      필드
0       4     uint32    매직 넘버 (0x57494145 = "WIAE")
4       1     uint8     버전 메이저
5       1     uint8     버전 마이너
6       1     uint8     메시지 유형 (열거형)
7       1     uint8     우선순위 (0-3)
8       8     int64     타임스탬프 (Unix 나노초)
16      4     uint32    시퀀스 번호
20      4     uint32    소스 ID (해시)
24      4     uint32    페이로드 길이
28      4     uint32    CRC32 체크섬
```

### 컴팩트 원격측정 (48 바이트)

최소 바이트로 필수 차량 상태:

```
오프셋  크기  유형      필드
0       4     int32     위도 (마이크로도)
4       4     int32     경도 (마이크로도)
8       4     int32     깊이 (밀리미터)
12      2     int16     방향 (센티도)
14      1     int8      피치 (도)
15      1     int8      롤 (도)
16      2     uint16    고도 (센티미터)
18      1     uint8     배터리 (퍼센트)
19      1     uint8     스러스터 상태 (비트필드)
20      2     int16     온도 (센티도)
22      2     uint16    압력 (데시바)
24      1     uint8     시스템 상태 (비트필드)
25      1     uint8     경고 수
26      2     uint16    예약됨
```

---

## 4.8 파일 저장 및 보관

### 미션 데이터 아카이브 구조

```
mission-HYDROTHERMAL-2025-01/
├── metadata.json
├── README.md
├── telemetry/
│   ├── vehicle-telemetry-001.jsonl
│   ├── vehicle-telemetry-002.jsonl
│   └── index.json
├── oceanographic/
│   ├── ctd-data-001.jsonl
│   ├── ctd-profiles/
│   │   └── profile-2025-01-15-001.json
│   └── index.json
├── bathymetry/
│   ├── multibeam-raw/
│   ├── multibeam-processed/
│   ├── grids/
│   │   └── bathymetry-1m.tiff
│   └── index.json
├── samples/
│   ├── biological/
│   ├── geological/
│   └── index.json
├── imagery/
│   ├── video/
│   ├── photos/
│   └── index.json
└── logs/
    ├── mission-log.txt
    └── events.jsonl
```

---

## 장 요약

WIA 심해 탐사 표준의 Phase 1은 수중 운영에서 발생하는 모든 주요 데이터 유형에 대한 포괄적인 데이터 형식 사양을 수립합니다. 실시간 차량 원격측정부터 상세한 샘플 메타데이터까지, 표준은 데이터가 자체 설명적이고 검증되며 시스템 간에 상호운용 가능하도록 보장합니다.

JSON 기반 형식은 인간 가독성과 광범위한 도구 지원을 제공하고, 이진 형식은 대역폭 제한 음향 링크를 통한 필수 데이터 전송을 가능하게 합니다. 품질 플래그, 검증 범위, 체크섬 요구사항은 데이터 수명주기 전반에 걸쳐 데이터 무결성을 보장합니다.

---

## 핵심 요점

1. **모든 메시지에 기본 필드 포함** - 버전, 유형, 타임스탬프, 소스, 우선순위, 체크섬
2. **품질 플래그는 국제 규약 따름** (GOOD, PROBABLY_GOOD, BAD 등)
3. **매개변수 검증이 명백한 오류 방지** - 데이터 저장 전
4. **이진 형식은 메시지 크기를 90%+ 감소** - 음향 전송용
5. **계층적 아카이브 구조**가 미션 데이터를 논리적으로 구성

---

## 복습 질문

1. 모든 WIA 기본 메시지의 필수 필드는 무엇입니까?
2. 검증에 실패한 데이터에 어떤 품질 플래그를 할당해야 합니까?
3. 샘플 메시지 페이로드의 체크섬을 계산하세요.
4. 이진 형식의 목적은 무엇이며 언제 사용해야 합니까?
5. 지질 코어 샘플에 대한 샘플 메타데이터 레코드를 설계하세요.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
