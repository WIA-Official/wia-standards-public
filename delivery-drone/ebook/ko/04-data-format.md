# 제4장: 데이터 형식 및 메시지 사양

## Phase 1: 드론 배송 시스템을 위한 상호운용 데이터 표준

---

## 4.1 데이터 형식 설계 원칙

### 핵심 요구사항

WIA-AUTO-017 데이터 형식은 다음 요구사항을 염두에 두고 설계되었습니다:

1. **기계 파싱 가능**: 효율적으로 처리할 수 있는 구조화된 형식
2. **사람이 읽기 쉬움**: 개발 중 쉽게 디버그하고 이해 가능
3. **버전 관리**: 호환성을 깨지 않는 스키마 진화 지원
4. **컴팩트**: 제한된 대역폭에서 효율적인 전송
5. **자기 설명적**: 메타데이터로 외부 문서 없이 해석 가능

### 형식 선택

| 사용 사례 | 기본 형식 | 대체 형식 |
|----------|----------|----------|
| API 통신 | JSON | Protocol Buffers |
| 원격측정 스트리밍 | MessagePack | Binary |
| 비행 로그 | JSON Lines | Parquet |
| 설정 | YAML | JSON |
| 지리공간 데이터 | GeoJSON | Shapefile |

### 스키마 검증

모든 메시지는 JSON Schema 정의에 따라 검증되어야 합니다:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/delivery-drone/base-message.json",
  "type": "object",
  "required": ["wiaVersion", "messageType", "timestamp", "sourceId"],
  "properties": {
    "wiaVersion": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$"
    },
    "messageType": {
      "type": "string",
      "enum": ["TELEMETRY", "WAYPOINT", "COMMAND", "STATUS", "DELIVERY", "FLIGHT_LOG"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sourceId": {
      "type": "string",
      "pattern": "^WIA-DRN-[A-Z0-9]+-\\d{4}$"
    }
  }
}
```

---

## 4.2 기본 메시지 형식

### 메시지 엔벨로프

모든 WIA 메시지는 공통 엔벨로프 구조를 공유합니다:

```json
{
  "wiaVersion": "1.0",
  "messageType": "TELEMETRY",
  "messageId": "msg-20250101-abcd1234",
  "timestamp": "2025-01-01T10:00:00.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "sequenceNumber": 12345,
  "priority": "NORMAL",
  "payload": {
    // 메시지별 내용
  },
  "checksum": "SHA256:a1b2c3d4..."
}
```

### 필드 사양

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| wiaVersion | string | 예 | 표준 버전 (예: "1.0") |
| messageType | enum | 예 | 메시지 내용 유형 |
| messageId | string | 예 | 고유 메시지 식별자 |
| timestamp | ISO8601 | 예 | 밀리초 포함 UTC 타임스탬프 |
| sourceId | string | 예 | 드론 식별자 |
| sequenceNumber | integer | 아니오 | 순서를 위한 단조 시퀀스 |
| priority | enum | 아니오 | LOW, NORMAL, HIGH, CRITICAL |
| payload | object | 예 | 메시지별 내용 |
| checksum | string | 아니오 | 페이로드의 SHA256 해시 |

### 메시지 타입

```typescript
enum MessageType {
  // 실시간 데이터
  TELEMETRY = "TELEMETRY",
  POSITION = "POSITION",
  STATUS = "STATUS",

  // 미션 데이터
  WAYPOINT = "WAYPOINT",
  FLIGHT_PLAN = "FLIGHT_PLAN",
  MISSION = "MISSION",

  // 배송 데이터
  PACKAGE = "PACKAGE",
  DELIVERY = "DELIVERY",

  // 제어
  COMMAND = "COMMAND",
  RESPONSE = "RESPONSE",

  // 로깅
  FLIGHT_LOG = "FLIGHT_LOG",
  EVENT = "EVENT"
}
```

---

## 4.3 원격측정 형식

### 실시간 위치 원격측정

비행 중 1-10 Hz로 전송:

```json
{
  "wiaVersion": "1.0",
  "messageType": "TELEMETRY",
  "timestamp": "2025-01-01T10:05:30.123Z",
  "sourceId": "WIA-DRN-X1-0042",
  "sequenceNumber": 12345,
  "payload": {
    "position": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitudeMSL": 125.5,
      "altitudeAGL": 35.2,
      "accuracy": {
        "horizontal": 1.5,
        "vertical": 2.0,
        "source": "RTK_FIXED"
      }
    },
    "velocity": {
      "north": 8.5,
      "east": 12.3,
      "down": -0.5,
      "groundSpeed": 15.0,
      "airSpeed": 16.2
    },
    "attitude": {
      "roll": 2.3,
      "pitch": 5.1,
      "yaw": 45.0
    },
    "heading": {
      "magnetic": 48.5,
      "true": 45.0
    },
    "satellites": 14,
    "hdop": 0.8,
    "vdop": 1.2
  }
}
```

### 시스템 상태 원격측정

비행 중 0.2-1 Hz로 전송:

```json
{
  "wiaVersion": "1.0",
  "messageType": "STATUS",
  "timestamp": "2025-01-01T10:05:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "battery": {
      "voltage": 22.1,
      "current": 25.3,
      "capacity": 10000,
      "remaining": 7800,
      "percentage": 78,
      "cellVoltages": [3.68, 3.69, 3.68, 3.70, 3.69, 3.68],
      "temperature": 32.5,
      "estimatedFlightTime": 1200
    },
    "motors": [
      {"id": 1, "rpm": 5200, "current": 4.2, "temperature": 45.0, "status": "OK"},
      {"id": 2, "rpm": 5180, "current": 4.1, "temperature": 44.5, "status": "OK"},
      {"id": 3, "rpm": 5220, "current": 4.3, "temperature": 46.0, "status": "OK"},
      {"id": 4, "rpm": 5190, "current": 4.2, "temperature": 45.5, "status": "OK"},
      {"id": 5, "rpm": 5210, "current": 4.1, "temperature": 45.0, "status": "OK"},
      {"id": 6, "rpm": 5200, "current": 4.2, "temperature": 45.2, "status": "OK"}
    ],
    "flightMode": "WAYPOINT",
    "armed": true,
    "airborne": true,
    "gpsStatus": "RTK_FIXED",
    "linkQuality": {
      "primary": 95,
      "backup": 88,
      "latency": 45
    },
    "sensors": {
      "imu": "OK",
      "gps": "OK",
      "barometer": "OK",
      "magnetometer": "OK",
      "lidar": "OK",
      "camera": "OK"
    },
    "warnings": [],
    "errors": []
  }
}
```

### 컴팩트 원격측정 형식

대역폭이 제한된 시나리오를 위한 컴팩트 바이너리 형식:

```python
import struct

def encode_compact_telemetry(telemetry: dict) -> bytes:
    """
    원격측정을 컴팩트 바이너리 형식으로 인코딩.

    형식 (48 바이트):
    - timestamp: 8 바이트 (uint64, 에포크 이후 밀리초)
    - latitude: 4 바이트 (int32, 마이크로도)
    - longitude: 4 바이트 (int32, 마이크로도)
    - altitude_msl: 2 바이트 (int16, 데시미터)
    - altitude_agl: 2 바이트 (int16, 데시미터)
    - ground_speed: 2 바이트 (uint16, cm/s)
    - heading: 2 바이트 (uint16, 센티도)
    - roll: 2 바이트 (int16, 센티도)
    - pitch: 2 바이트 (int16, 센티도)
    - battery_percent: 1 바이트 (uint8)
    - satellites: 1 바이트 (uint8)
    - status_flags: 2 바이트 (uint16)
    - motor_status: 8 바이트 (8x uint8)
    - reserved: 8 바이트
    """
    pos = telemetry['position']
    vel = telemetry['velocity']
    att = telemetry['attitude']
    bat = telemetry['battery']

    return struct.pack(
        '!QiihhHHhhBBH8s8s',
        int(telemetry['timestamp'] * 1000),  # 에포크 이후 ms
        int(pos['latitude'] * 1e6),
        int(pos['longitude'] * 1e6),
        int(pos['altitudeMSL'] * 10),
        int(pos['altitudeAGL'] * 10),
        int(vel['groundSpeed'] * 100),
        int(att['yaw'] * 100),
        int(att['roll'] * 100),
        int(att['pitch'] * 100),
        int(bat['percentage']),
        telemetry['satellites'],
        telemetry.get('status_flags', 0),
        b'\x00' * 8,  # 모터 상태
        b'\x00' * 8   # 예약
    )

def decode_compact_telemetry(data: bytes) -> dict:
    """바이너리 형식에서 컴팩트 원격측정 디코드."""
    values = struct.unpack('!QiihhHHhhBBH8s8s', data)

    return {
        'timestamp': values[0] / 1000,
        'position': {
            'latitude': values[1] / 1e6,
            'longitude': values[2] / 1e6,
            'altitudeMSL': values[3] / 10,
            'altitudeAGL': values[4] / 10
        },
        'velocity': {
            'groundSpeed': values[5] / 100
        },
        'attitude': {
            'yaw': values[6] / 100,
            'roll': values[7] / 100,
            'pitch': values[8] / 100
        },
        'battery': {
            'percentage': values[9]
        },
        'satellites': values[10],
        'status_flags': values[11]
    }
```

---

## 4.4 웨이포인트 및 비행 계획 형식

### 단일 웨이포인트

```json
{
  "waypointId": "WP-001",
  "position": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitudeMSL": 120,
    "altitudeAGL": 30
  },
  "speed": 15.0,
  "heading": 45,
  "headingMode": "AUTO",
  "arrivalRadius": 5.0,
  "holdTime": 0,
  "actions": [
    {
      "type": "TAKE_PHOTO",
      "params": {}
    }
  ],
  "constraints": {
    "minAltitude": 20,
    "maxAltitude": 120,
    "maxSpeed": 20
  }
}
```

### 웨이포인트 액션 타입

| 액션 | 설명 | 파라미터 |
|------|------|----------|
| TAKE_PHOTO | 이미지 촬영 | resolution, format |
| START_VIDEO | 녹화 시작 | resolution, fps |
| STOP_VIDEO | 녹화 중지 | - |
| HOVER | 위치 유지 | duration |
| CHANGE_SPEED | 속도 조정 | speed |
| CHANGE_ALTITUDE | 고도 조정 | altitude, rate |
| RELEASE_PAYLOAD | 패키지 드롭/하강 | method |
| WAIT_FOR_TRIGGER | 조건 대기 | condition |

### 완전한 비행 계획

```json
{
  "wiaVersion": "1.0",
  "messageType": "FLIGHT_PLAN",
  "messageId": "fp-20250101-1234",
  "timestamp": "2025-01-01T09:00:00.000Z",
  "sourceId": "WIA-GCS-001",
  "payload": {
    "flightPlanId": "FP-20250101-1234",
    "name": "알파 배송 미션",
    "description": "강남구 테헤란로 456으로 패키지 배송",
    "drone": "WIA-DRN-X1-0042",
    "operator": "WIA-OP-001",
    "departure": {
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 10
      },
      "address": "서울시 종로구 세종대로 123",
      "scheduledTime": "2025-01-01T10:00:00Z"
    },
    "arrival": {
      "location": {
        "latitude": 37.5012,
        "longitude": 127.0396,
        "altitude": 0
      },
      "address": "서울시 강남구 테헤란로 456",
      "scheduledTime": "2025-01-01T10:15:00Z"
    },
    "waypoints": [
      {
        "waypointId": "WP-001",
        "type": "TAKEOFF",
        "position": {"latitude": 37.5665, "longitude": 126.9780, "altitudeMSL": 50},
        "speed": 5.0
      },
      {
        "waypointId": "WP-002",
        "type": "CRUISE",
        "position": {"latitude": 37.5400, "longitude": 127.0000, "altitudeMSL": 120},
        "speed": 20.0
      },
      {
        "waypointId": "WP-003",
        "type": "CRUISE",
        "position": {"latitude": 37.5100, "longitude": 127.0300, "altitudeMSL": 120},
        "speed": 20.0
      },
      {
        "waypointId": "WP-004",
        "type": "DESCENT",
        "position": {"latitude": 37.5012, "longitude": 127.0396, "altitudeMSL": 30},
        "speed": 10.0
      },
      {
        "waypointId": "WP-005",
        "type": "DELIVERY",
        "position": {"latitude": 37.5012, "longitude": 127.0396, "altitudeMSL": 5},
        "speed": 2.0,
        "actions": [{"type": "RELEASE_PAYLOAD", "params": {"method": "WINCH"}}]
      }
    ],
    "constraints": {
      "maxAltitude": 120,
      "maxSpeed": 22,
      "geofence": {
        "type": "POLYGON",
        "coordinates": [[37.55, 126.95], [37.55, 127.05], [37.48, 127.05], [37.48, 126.95]]
      }
    },
    "contingency": {
      "lostLink": "RETURN_TO_HOME",
      "lowBattery": "RETURN_TO_HOME",
      "geofenceViolation": "RETURN_TO_HOME",
      "alternativeLandingSites": [
        {"latitude": 37.5300, "longitude": 127.0100, "priority": 1}
      ]
    }
  }
}
```

---

## 4.5 패키지 및 배송 형식

### 패키지 사양

```json
{
  "packageId": "PKG-20250101-1234",
  "trackingCode": "KR999AA10123456784",
  "weight": {
    "value": 2.5,
    "unit": "kg"
  },
  "dimensions": {
    "length": 30,
    "width": 20,
    "height": 15,
    "unit": "cm"
  },
  "volumetricWeight": {
    "value": 1.8,
    "unit": "kg"
  },
  "classification": {
    "fragile": false,
    "hazardous": false,
    "hazardClass": null,
    "temperatureSensitive": false,
    "temperatureRange": null,
    "perishable": false,
    "expiryDate": null
  },
  "value": {
    "amount": 150000,
    "currency": "KRW"
  },
  "insurance": {
    "covered": true,
    "provider": "드론보험주식회사",
    "policyNumber": "DI-2025-001234"
  },
  "sender": {
    "name": "ABC 전자",
    "address": "서울시 종로구 세종대로 123",
    "phone": "+82-10-1234-5678"
  },
  "recipient": {
    "name": "김철수",
    "address": "서울시 강남구 테헤란로 456",
    "phone": "+82-10-9876-5432",
    "instructions": "현관문 앞에 놓아주세요"
  },
  "deliveryPreferences": {
    "leaveAtDoor": true,
    "signatureRequired": false,
    "ageVerification": false,
    "photoConfirmation": true
  }
}
```

### 배송 이벤트 형식

```json
{
  "wiaVersion": "1.0",
  "messageType": "DELIVERY",
  "timestamp": "2025-01-01T10:14:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "deliveryId": "DEL-20250101-1234",
    "missionId": "MSN-20250101-1234",
    "packageId": "PKG-20250101-1234",
    "event": "DELIVERED",
    "eventTimestamp": "2025-01-01T10:14:30.000Z",
    "location": {
      "latitude": 37.5012,
      "longitude": 127.0396,
      "altitudeAGL": 0.5,
      "accuracy": 0.3
    },
    "deliveryMethod": "WINCH",
    "details": {
      "winchDeployed": true,
      "winchLoweredDistance": 3.5,
      "packageReleased": true,
      "groundContactDetected": true,
      "winchRetracted": true
    },
    "confirmation": {
      "photoTaken": true,
      "photoUrl": "https://storage.wia.com/delivery-photos/DEL-20250101-1234.jpg",
      "signatureCollected": false,
      "recipientPresent": false
    },
    "timing": {
      "missionStarted": "2025-01-01T10:00:00.000Z",
      "packagePickedUp": "2025-01-01T10:02:00.000Z",
      "deliveryStarted": "2025-01-01T10:12:00.000Z",
      "deliveryCompleted": "2025-01-01T10:14:30.000Z"
    }
  }
}
```

### 배송 상태 열거형

```typescript
enum DeliveryStatus {
  // 비행 전
  PENDING = "PENDING",           // 대기 중
  SCHEDULED = "SCHEDULED",       // 예약됨
  PACKAGE_RECEIVED = "PACKAGE_RECEIVED",  // 패키지 수령
  PACKAGE_LOADED = "PACKAGE_LOADED",      // 패키지 적재

  // 비행 중
  DEPARTED = "DEPARTED",         // 출발
  EN_ROUTE = "EN_ROUTE",         // 이동 중
  APPROACHING = "APPROACHING",   // 접근 중
  HOVERING = "HOVERING",         // 호버링
  DELIVERING = "DELIVERING",     // 배송 중

  // 완료
  DELIVERED = "DELIVERED",       // 배송 완료
  CONFIRMED = "CONFIRMED",       // 확인됨

  // 문제
  ATTEMPTED = "ATTEMPTED",       // 시도됨
  RETURNED = "RETURNED",         // 반송됨
  CANCELLED = "CANCELLED",       // 취소됨
  FAILED = "FAILED"              // 실패
}
```

---

## 4.6 비행 로그 형식

### 완전한 비행 로그

```json
{
  "wiaVersion": "1.0",
  "messageType": "FLIGHT_LOG",
  "timestamp": "2025-01-01T10:20:00.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "flightId": "FLT-20250101-1234",
    "missionId": "MSN-20250101-1234",
    "droneId": "WIA-DRN-X1-0042",
    "operatorId": "WIA-OP-001",
    "pilotId": "PILOT-001",
    "timing": {
      "scheduled": "2025-01-01T10:00:00.000Z",
      "armed": "2025-01-01T10:00:15.000Z",
      "takeoff": "2025-01-01T10:00:30.000Z",
      "landing": "2025-01-01T10:14:45.000Z",
      "disarmed": "2025-01-01T10:15:00.000Z"
    },
    "duration": {
      "total": 900,
      "airborne": 855,
      "hover": 120,
      "cruise": 735
    },
    "distance": {
      "total": 5200,
      "horizontal": 5150,
      "unit": "meters"
    },
    "altitude": {
      "max": 125,
      "min": 0,
      "average": 85,
      "unit": "meters"
    },
    "speed": {
      "max": 22.5,
      "average": 15.3,
      "unit": "m/s"
    },
    "battery": {
      "start": 100,
      "end": 56,
      "consumed": 44,
      "energyUsed": 98.5,
      "unit": "Wh"
    },
    "waypoints": {
      "total": 5,
      "completed": 5,
      "skipped": 0
    },
    "delivery": {
      "packageId": "PKG-20250101-1234",
      "success": true,
      "method": "WINCH"
    },
    "weather": {
      "temperature": 18.5,
      "windSpeed": 5.2,
      "windDirection": 270,
      "visibility": 10000,
      "precipitation": false
    },
    "incidents": [],
    "telemetryFile": {
      "format": "BINARY",
      "url": "https://storage.wia.com/telemetry/FLT-20250101-1234.bin",
      "size": 2457600,
      "records": 8550
    }
  }
}
```

### 비행 로그 쿼리 파라미터

```
GET /api/v1/flight-logs?
  droneId=WIA-DRN-X1-0042&
  startDate=2025-01-01&
  endDate=2025-01-31&
  status=COMPLETED&
  deliverySuccess=true&
  minDistance=1000&
  maxDistance=10000&
  limit=100&
  offset=0
```

---

## 4.7 명령 및 응답 형식

### 명령 메시지

```json
{
  "wiaVersion": "1.0",
  "messageType": "COMMAND",
  "messageId": "cmd-20250101-abcd",
  "timestamp": "2025-01-01T10:05:00.000Z",
  "sourceId": "WIA-GCS-001",
  "priority": "HIGH",
  "payload": {
    "commandId": "CMD-20250101-1234",
    "targetId": "WIA-DRN-X1-0042",
    "commandType": "RETURN_TO_HOME",
    "parameters": {},
    "timeout": 30000,
    "requiresAck": true
  }
}
```

### 명령 타입

| 명령 | 설명 | 파라미터 |
|------|------|----------|
| ARM | 모터 아밍 | - |
| DISARM | 모터 디스아밍 | - |
| TAKEOFF | 비행 시작 | altitude |
| LAND | 즉시 착륙 | - |
| RETURN_TO_HOME | RTH 절차 | - |
| GOTO_WAYPOINT | 웨이포인트로 항법 | waypointId |
| PAUSE | 제자리 호버 | - |
| RESUME | 미션 계속 | - |
| ABORT | 비상 정지 | - |
| SET_SPEED | 속도 변경 | speed |
| SET_ALTITUDE | 고도 변경 | altitude |
| RELEASE_PAYLOAD | 패키지 드롭 | method |
| TAKE_PHOTO | 이미지 촬영 | - |
| START_VIDEO | 녹화 시작 | - |
| STOP_VIDEO | 녹화 중지 | - |

### 응답 메시지

```json
{
  "wiaVersion": "1.0",
  "messageType": "RESPONSE",
  "messageId": "rsp-20250101-efgh",
  "timestamp": "2025-01-01T10:05:00.250Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "commandId": "CMD-20250101-1234",
    "status": "ACCEPTED",
    "message": "홈 복귀 시작됨",
    "estimatedCompletion": "2025-01-01T10:12:00.000Z",
    "details": {
      "currentPosition": {"latitude": 37.5400, "longitude": 127.0000},
      "homePosition": {"latitude": 37.5665, "longitude": 126.9780},
      "estimatedDistance": 650,
      "estimatedTime": 420
    }
  }
}
```

### 명령 응답 상태

| 상태 | 설명 |
|------|------|
| ACCEPTED | 명령 수신 및 대기열에 추가됨 |
| EXECUTING | 명령 진행 중 |
| COMPLETED | 명령 성공적으로 완료 |
| REJECTED | 명령 거부됨 (사유 참조) |
| FAILED | 명령 실행 실패 |
| TIMEOUT | 명령 시간 초과 |

---

## 4.8 이벤트 및 알림 형식

### 이벤트 메시지

```json
{
  "wiaVersion": "1.0",
  "messageType": "EVENT",
  "timestamp": "2025-01-01T10:06:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "eventId": "EVT-20250101-5678",
    "eventType": "WARNING",
    "category": "BATTERY",
    "code": "BAT_LOW_WARNING",
    "severity": "MEDIUM",
    "message": "배터리 잔량 30% 미만",
    "details": {
      "currentLevel": 28,
      "threshold": 30,
      "estimatedFlightTime": 480
    },
    "recommendedAction": "홈 복귀 시작 고려",
    "autoResponse": {
      "triggered": false,
      "action": null
    },
    "location": {
      "latitude": 37.5300,
      "longitude": 127.0100,
      "altitude": 100
    }
  }
}
```

### 이벤트 카테고리 및 코드

| 카테고리 | 코드 | 심각도 | 설명 |
|----------|------|--------|------|
| BATTERY | BAT_LOW_WARNING | MEDIUM | 배터리 30% 미만 |
| BATTERY | BAT_CRITICAL | CRITICAL | 배터리 15% 미만 |
| GPS | GPS_DEGRADED | MEDIUM | GPS 정확도 저하 |
| GPS | GPS_LOST | HIGH | GPS 신호 없음 |
| MOTOR | MOTOR_ANOMALY | MEDIUM | 모터 성능 이상 |
| MOTOR | MOTOR_FAILED | CRITICAL | 모터 고장 감지 |
| COMM | LINK_DEGRADED | MEDIUM | 통신 품질 저하 |
| COMM | LINK_LOST | HIGH | 통신 두절 |
| GEOFENCE | GEOFENCE_WARNING | HIGH | 경계 접근 |
| GEOFENCE | GEOFENCE_VIOLATION | CRITICAL | 경계 위반 |
| OBSTACLE | OBSTACLE_DETECTED | MEDIUM | 경로에 장애물 |
| WEATHER | WIND_WARNING | MEDIUM | 강풍 감지 |

---

## 한국 특화 데이터 형식

### K-드론 시스템 연동 형식

K-드론 시스템과의 통합을 위한 확장 필드:

```json
{
  "kdrone_extension": {
    "registrationNumber": "K-DRN-2025-001234",
    "pilotLicense": "DRP-1-2025-00567",
    "operatorId": "OP-2025-00890",
    "flightApprovalNumber": "FA-20250101-1234",
    "insuranceCertificate": "DI-2025-001234",
    "noFlyZoneCleared": true,
    "utmProvider": "한국공항공사"
  }
}
```

### 한국 주소 형식

```json
{
  "korean_address": {
    "fullAddress": "서울특별시 강남구 테헤란로 456",
    "roadAddress": "테헤란로 456",
    "jibunAddress": "역삼동 123-45",
    "postalCode": "06232",
    "buildingName": "스마트타워",
    "detail": "15층 1502호"
  }
}
```

---

## 장 요약

WIA-AUTO-017 데이터 형식 사양은 드론 배송 운영의 모든 측면에 대해 포괄적이고 상호운용 가능한 메시지 형식을 정의합니다. 실시간 원격측정부터 비행 계획, 패키지 사양, 배송 이벤트까지 형식은 모든 시스템 구성요소에서 일관된 통신을 보장합니다.

기본 메시지 엔벨로프는 버전 관리, 식별 및 무결성 검증을 제공합니다. 원격측정 형식은 고품질 JSON과 대역폭 효율적인 바이너리 인코딩을 모두 지원합니다. 비행 계획은 웨이포인트, 액션 및 비상 절차를 포함한 완전한 미션 파라미터를 지정합니다.

패키지 및 배송 형식은 패키지 사양부터 확인까지 배송의 전체 생명 주기를 포착합니다. 비행 로그는 분석 및 규정 준수를 위한 포괄적인 기록을 제공합니다. 명령 및 응답 형식은 신뢰할 수 있는 원격 제어를 가능하게 하며, 이벤트 및 알림은 운영 상태의 적시 통지를 보장합니다.

---

## 핵심 요약

1. **모든 메시지는 공통 엔벨로프** 공유 (버전, 타입, 타임스탬프, 소스)
2. **원격측정은 JSON과 컴팩트 바이너리** 모두 지원 (대역폭 요구에 따라)
3. **비행 계획은 완전한 웨이포인트, 제약 및 비상 절차** 포함
4. **배송 형식은 전체 패키지 생명 주기** 추적 (발신자부터 수신자까지)
5. **이벤트와 알림은 심각도 수준** 사용 (적절한 운영자 대응 위해)

---

## 복습 문제

1. 모든 WIA 기본 메시지에서 필수인 필드는?
2. 32 바이트만 사용하는 컴팩트 원격측정 형식을 설계하시오.
3. 규제 준수를 위해 비행 계획에 포함되어야 하는 정보는?
4. 반품 배송을 지원하도록 배송 형식을 어떻게 확장하겠습니까?
5. 공역 충돌 감지 및 보고를 위한 이벤트 형식을 만드시오.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
