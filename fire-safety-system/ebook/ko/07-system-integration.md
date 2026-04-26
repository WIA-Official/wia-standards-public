# 7장: 시스템 통합 (4단계)

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

- 소방 안전 시스템 통합 아키텍처의 구성 요소와 이점을 설명한다
- HVAC, 출입 통제, 엘리베이터 및 조명 시스템과의 통합 메커니즘을 이해한다
- 화재 경보 활성화 시 자동화된 대응 조치의 순서를 파악한다
- 응급 서비스 통합을 위한 데이터 요구 사항을 설명한다
- 타사 애플리케이션 통합을 위한 API 사용 방법을 이해한다

---

## 개요

WIA 표준의 4단계는 소방 안전 시스템이 빌딩 관리 시스템, 출입 통제, HVAC, 엘리베이터 제어, 조명 및 응급 서비스와 조정할 수 있도록 하는 포괄적인 통합 사양을 정의합니다. 이 장에서는 통합 아키텍처, 자동화된 대응 조치, 응급 서비스 알림 및 타사 애플리케이션 통합을 살펴봅니다.

---

## 통합 아키텍처

### 전체적인 빌딩 안전 시스템

```
소방 안전 시스템 통합 아키텍처:

┌─────────────────────────────────────────────────────────┐
│         화재 경보 제어 패널 (FACP)                      │
│              WIA 표준 인터페이스                        │
└────┬─────────┬──────────┬──────────┬──────────┬────────┘
     │         │          │          │          │
     │         │          │          │          │
┌────▼────┐ ┌─▼──────┐ ┌─▼──────┐ ┌─▼──────┐ ┌─▼────────┐
│  HVAC   │ │ 출입   │ │엘리베이│ │  조명  │ │  응급    │
│  제어   │ │ 통제   │ │터 제어 │ │  제어  │ │  서비스  │
└────┬────┘ └───┬────┘ └───┬────┘ └───┬────┘ └────┬─────┘
     │          │          │          │          │
     │          │          │          │          │
 공기팬    모든 출구   1층으로   비상      자동
 차단      잠금 해제   리콜     조명으로   119 호출
                                전환

통합 이점:
✓ 조정된 자동 대응
✓ 더 빠른 대피 (30% 개선)
✓ 향상된 소방관 접근
✓ 재산 피해 감소
✓ 건축 법규 준수
✓ 상황 인식 개선
```

### 통합 프로토콜

WIA 표준은 여러 통합 방법을 지원합니다:

```
통합 프로토콜 스택:

┌─────────────────────────────────────────────┐
│ 애플리케이션 계층                           │
│ • WIA Fire Safety API (RESTful/WebSocket)   │
├─────────────────────────────────────────────┤
│ 표준 빌딩 프로토콜                          │
│ • BACnet (빌딩 자동화)                      │
│ • Modbus (산업 제어)                        │
│ • OPC UA (산업 상호 운용성)                 │
│ • MQTT (IoT 메시징)                         │
│ • CoAP (제약 장치)                          │
├─────────────────────────────────────────────┤
│ 사용자 정의 통합                            │
│ • 어댑터를 통한 벤더별 API                  │
│ • 레거시 프로토콜 게이트웨이                │
│ • 사용자 정의 미들웨어 개발                 │
└─────────────────────────────────────────────┘
```

---

## 빌딩 관리 시스템 (BMS) 통합

### HVAC 제어

**화재 경보 활성화 시:**

```
HVAC 대응 조치:

┌─────────────────────────────────────────────┐
│ 1. 공급/배출 팬 차단                        │
│    • 경보 구역: 즉시 차단                   │
│    • 인접 구역: 지시 시 차단                │
│    • 기타 영역: 전략에 따라 유지            │
│                                             │
│ 2. 화재/연기 댐퍼 닫기                      │
│    • NFPA 90A에 따른 모든 댐퍼              │
│    • 15초 이내에 닫기                       │
│    • 전기적으로 폐쇄 확인                   │
│                                             │
│ 3. 연기 배출 활성화                         │
│    • 지정된 연기 구역                       │
│    • 계단실 압력과 조정                     │
│    • 조건에 따른 가변 속도                  │
│                                             │
│ 4. 계단실 가압                              │
│    • 양압 유지                              │
│    • 연기 침투 방지                         │
│    • 일반적: 0.10-0.35인치 수주             │
│                                             │
│ 5. 엘리베이터 샤프트 가압                   │
│    • 엘리베이터로 연기 진입 방지            │
│    • 엘리베이터 리콜과 조정                 │
│                                             │
│ 6. 최소 환기 유지                           │
│    • 중요 영역 (서버실 등)                  │
│    • 코드 요구 사항에 따라                  │
│    • 생명 안전과 재산의 균형                │
└─────────────────────────────────────────────┘
```

**통합 API 예제:**

```http
POST /api/v1/integrations/hvac/fire-alarm-response HTTP/1.1
Host: bms.building.com
Authorization: Bearer <token>
Content-Type: application/json

{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "location": {
    "building": "메인 타워",
    "floor": 12,
    "zone": "동쪽 구역 E12-A"
  },
  "actions": [
    {
      "system": "HVAC",
      "action": "shutdown_fans",
      "zones": ["E12-A", "E12-B", "E13-A"],
      "priority": "critical"
    },
    {
      "system": "HVAC",
      "action": "close_dampers",
      "dampers": ["FD-12-01", "FD-12-02", "FD-12-03"],
      "priority": "critical"
    },
    {
      "system": "HVAC",
      "action": "activate_smoke_exhaust",
      "zones": ["E12-A"],
      "priority": "high"
    },
    {
      "system": "HVAC",
      "action": "pressurize_stairwells",
      "stairwells": ["ST-A", "ST-B"],
      "targetPressure": 0.25,
      "priority": "high"
    }
  ]
}
```

**HVAC 시스템 응답:**

```json
{
  "responseId": "hvac-resp-1234567890",
  "timestamp": "2025-12-27T14:32:16.500Z",
  "actionsCompleted": [
    {
      "action": "shutdown_fans",
      "status": "completed",
      "completionTime": "2025-12-27T14:32:16.200Z",
      "fansShutdown": 8
    },
    {
      "action": "close_dampers",
      "status": "completed",
      "completionTime": "2025-12-27T14:32:17.100Z",
      "dampersClosed": 3,
      "confirmations": ["FD-12-01", "FD-12-02", "FD-12-03"]
    },
    {
      "action": "activate_smoke_exhaust",
      "status": "in_progress",
      "estimatedCompletion": "2025-12-27T14:32:25.000Z"
    },
    {
      "action": "pressurize_stairwells",
      "status": "in_progress",
      "currentPressure": 0.15,
      "targetPressure": 0.25
    }
  ]
}
```

---

## 출입 통제 통합

### 자동화된 비상구

**화재 경보 활성화 시:**

```
출입 통제 대응 조치:

우선순위 1: 생명 안전 비상구 (T+0초)
┌─────────────────────────────────────────────┐
│ • 모든 출구 문 잠금 해제                    │
│ • 전자기 잠금 장치 해제                     │
│ • 회전식 개찰구 및 게이트 개방              │
│ • 출구에서 카드 리더 비활성화               │
│ • 모든 출구 지점에 녹색 불                  │
└─────────────────────────────────────────────┘

우선순위 2: 층 액세스 (T+5초)
┌─────────────────────────────────────────────┐
│ • 경보 층의 모든 문 잠금 해제               │
│ • 계단실 문 잠금 해제 (재진입)              │
│ • 복도 교차 문 잠금 해제                    │
│ • 스위트룸 보안 유지 (구성 가능)            │
└─────────────────────────────────────────────┘

우선순위 3: 외곽 보안 (지속)
┌─────────────────────────────────────────────┐
│ • 진입 보안 유지                            │
│ • 모든 액세스 이벤트 기록                   │
│ • CCTV 녹화 활성화                          │
│ • 무단 진입 방지                            │
└─────────────────────────────────────────────┘
```

**통합 예제:**

```json
{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T14:32:15.000Z",
  "accessControlActions": {
    "unlockAllExits": {
      "enabled": true,
      "doors": [
        "EXIT-E-12-01",
        "EXIT-E-12-02",
        "STAIR-A-12",
        "STAIR-B-12"
      ],
      "status": "completed",
      "completionTime": "2025-12-27T14:32:15.800Z"
    },
    "releaseElectromagneticLocks": {
      "enabled": true,
      "locks": 24,
      "status": "completed",
      "completionTime": "2025-12-27T14:32:15.500Z"
    },
    "disableReaders": {
      "enabled": true,
      "readers": ["EXIT-READER-001", "EXIT-READER-002"],
      "mode": "free_egress"
    },
    "maintainPerimeter": {
      "enabled": true,
      "entryPoints": ["MAIN-LOBBY", "PARKING-ACCESS"],
      "mode": "secured"
    }
  }
}
```

---

## 엘리베이터 제어 통합

### 엘리베이터 리콜

**소방 서비스 작동:**

```
엘리베이터 리콜 순서:

1단계: 초기 리콜 (T+0초)
┌─────────────────────────────────────────────┐
│ 1. 모든 엘리베이터 카 호출 취소             │
│ 2. 엘리베이터가 리콜 층으로 이동            │
│    (일반적으로 1층)                         │
│ 3. 리콜 층까지 무정차 이동                  │
│ 4. 도착 시 문 열림                          │
│ 5. 음성 메시지: "엘리베이터 서비스 중단"    │
└─────────────────────────────────────────────┘

2단계: 소방 서비스 모드 (T+30초)
┌─────────────────────────────────────────────┐
│ 1. 엘리베이터가 리콜 층에 주차              │
│ 2. 소방관 키 스위치 필요                    │
│ 3. 수동 제어만 가능                         │
│ 4. 카 조명 계속 켜짐                        │
│ 5. 경보 벨 음소거                           │
│ 6. 명령이 있을 때까지 문 열림 유지          │
└─────────────────────────────────────────────┘

3단계: 소방관 작동
┌─────────────────────────────────────────────┐
│ • 키 스위치가 카 활성화                     │
│ • 수동 층 선택만 가능                       │
│ • 소방관이 문 제어                          │
│ • 모든 안전 기능 무시                       │
│ • 소방 지휘소와 직접 통신                   │
└─────────────────────────────────────────────┘

특수 사례:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
시나리오                  조치
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
샤프트 내 연기            가장 가까운 층에 정지
리콜 층의 경보            대체 리콜 층 사용
정전                      배터리로 1층까지 하강
카 내 탑승자              현재 이동 먼저 완료
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**통합 메시지:**

```json
{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T14:32:15.000Z",
  "elevatorControlActions": {
    "recallElevators": {
      "elevatorIds": ["ELEV-A", "ELEV-B", "ELEV-C", "ELEV-D"],
      "recallFloor": 1,
      "alternateRecallFloor": 2,
      "mode": "fire_service_phase_1"
    },
    "cancelAllCalls": true,
    "disableNormalOperation": true,
    "enableFirefighterService": true,
    "audioMessage": "화재 경보로 인해 엘리베이터가 서비스 중단되었습니다. 계단을 이용하세요.",
    "visualIndication": "flashing_red_light"
  }
}
```

---

## 비상 조명 통합

### 자동 조명 제어

```
비상 조명 대응:

즉시 조치 (T+0초):
┌─────────────────────────────────────────────┐
│ • 모든 비상 조명 활성화                     │
│ • 출구 표지판 조명 극대화                   │
│ • 출구로 가는 경로 조명 활성화              │
│ • 계단실 조명을 최대 밝기로                 │
│ • 디밍 시스템 비활성화                      │
└─────────────────────────────────────────────┘

향상된 가시성 (T+5초):
┌─────────────────────────────────────────────┐
│ • 경보 층: 전체 조명                        │
│ • 출구 경로: 최대 조명                      │
│ • 공용 공간: 향상된 조명                    │
│ • 비중요 영역: 최소 유지                    │
└─────────────────────────────────────────────┘

시각적 안내:
┌─────────────────────────────────────────────┐
│ • 깜박이는 출구 표지판 (허용되는 경우)      │
│ • 방향 화살표 표시기                        │
│ • 색상 코드 경로 표시                       │
│ • 축광 길 찾기                              │
└─────────────────────────────────────────────┘
```

---

## 응급 서비스 통합

### 자동 알림

**응급 서비스 API:**

```http
POST /api/v1/emergency-services/dispatch HTTP/1.1
Host: dispatch-system.city.gov
Authorization: Bearer <token>
Content-Type: application/json

{
  "emergencyType": "fire",
  "severity": "critical",
  "facility": {
    "facilityId": "BLDG-12345",
    "facilityName": "아크메 오피스 타워",
    "address": {
      "street": "메인 스트리트 123",
      "city": "애니타운",
      "state": "CA",
      "postalCode": "90210",
      "country": "USA",
      "gpsCoordinates": {
        "latitude": 34.0522,
        "longitude": -118.2437,
        "accuracy": 5
      }
    },
    "facilityType": "commercial_office",
    "occupancyType": "business",
    "constructionType": "type_1A_fireproof",
    "floors": {
      "aboveGrade": 15,
      "belowGrade": 2,
      "totalFloors": 17
    },
    "squareFootage": 285000,
    "yearBuilt": 2018
  },
  "incident": {
    "detectionTime": "2025-12-27T14:32:15Z",
    "location": {
      "building": "메인 타워",
      "floor": 12,
      "zone": "동쪽 구역 E12-A",
      "room": "회의실 1205",
      "description": "12층, 동쪽 구역, 북동쪽 모퉁이"
    },
    "detectionMethod": "automatic_smoke_detector",
    "deviceType": "photoelectric_smoke_detector",
    "confirmationMethod": "multi_sensor_verified",
    "spreadPotential": "high"
  },
  "occupancy": {
    "estimatedOccupants": 450,
    "timeOfDay": "business_hours",
    "specialNeeds": [
      "어린이집 (2층, 어린이 15명)",
      "의료 클리닉 (8층, 잠재적 이동 문제)"
    ],
    "evacuationStatus": "in_progress"
  },
  "hazards": {
    "hazardousMaterials": [
      {
        "material": "리튬 이온 배터리",
        "location": "IT 장비실, 5층",
        "quantity": "500개",
        "msdsAvailable": true
      }
    ],
    "structuralConcerns": [],
    "utilities": {
      "naturalGas": true,
      "fuelOil": false,
      "propane": false,
      "solarPanels": true
    }
  },
  "access": {
    "fireAccess": [
      "북쪽 출입구: 메인 스트리트, 키 박스 #1234",
      "동쪽 출입구: 주차장, 1층",
      "서쪽 출입구: 서비스 도크"
    ],
    "keyBoxLocation": "북쪽 출입구, 정문 오른쪽",
    "knoxBoxNumber": "KB-1234",
    "elevatorFireService": true,
    "standpipeLocations": ["계단 A", "계단 B"],
    "sprinklerSystemType": "wet_pipe",
    "fireDepartmentConnection": "북쪽, 정문"
  },
  "resources": {
    "buildingPlans": {
      "url": "https://files.example.com/building-plans",
      "format": "PDF",
      "lastUpdated": "2025-01-15"
    },
    "videoFeeds": [
      {
        "name": "12층 동쪽 구역",
        "url": "https://cctv.example.com/live/floor12-east",
        "credentials": "보안 채널을 통해 사용 가능"
      },
      {
        "name": "메인 로비",
        "url": "https://cctv.example.com/live/lobby"
      }
    ],
    "buildingAutomation": {
      "url": "https://bms.example.com/emergency-access",
      "capabilities": ["HVAC 제어", "출입 통제", "엘리베이터 상태"]
    }
  },
  "contacts": {
    "onSiteContacts": [
      {
        "role": "빌딩 관리자",
        "name": "김철수",
        "phone": "+82-2-1234-5678",
        "mobile": "+82-10-1234-5678",
        "availability": "on_site"
      },
      {
        "role": "수석 엔지니어",
        "name": "박영희",
        "phone": "+82-2-1234-5679",
        "availability": "on_call"
      }
    ],
    "emergencyContact": {
      "company": "아크메 부동산 관리",
      "phone": "+82-2-1234-0100",
      "available247": true
    }
  },
  "fireProtectionSystems": {
    "fireAlarm": {
      "type": "addressable",
      "standard": "WIA_v1.0",
      "monitoring": "central_station",
      "monitoringCompany": "시큐어워치 모니터링"
    },
    "sprinklers": {
      "type": "wet_pipe",
      "coverage": "full_building",
      "waterSupply": "municipal_plus_onsite_tank"
    },
    "standpipes": {
      "type": "automatic_wet",
      "outlets": "all_floors"
    },
    "fireExtinguishers": {
      "portable": "per_code",
      "locations": "see_building_plans"
    }
  }
}
```

**응급 서비스 응답:**

```json
{
  "dispatchId": "FD-2025-12-27-0156",
  "timestamp": "2025-12-27T14:32:17Z",
  "status": "dispatched",
  "response": {
    "units": [
      {
        "unitId": "ENGINE-5",
        "type": "engine",
        "personnel": 4,
        "status": "en_route",
        "eta": "2025-12-27T14:36:00Z"
      },
      {
        "unitId": "LADDER-2",
        "type": "ladder",
        "personnel": 4,
        "status": "en_route",
        "eta": "2025-12-27T14:36:30Z"
      },
      {
        "unitId": "BATTALION-1",
        "type": "battalion_chief",
        "personnel": 2,
        "status": "en_route",
        "eta": "2025-12-27T14:37:00Z"
      }
    ],
    "additionalResources": {
      "ambulance": "dispatched",
      "hazmat": "standby",
      "police": "notified"
    }
  },
  "incidentCommand": {
    "radioChannel": "FIRE-TAC-3",
    "commandPost": "북쪽 출입구",
    "staging": "메인 스트리트, 북쪽 200피트"
  }
}
```

---

## 타사 애플리케이션 통합

### 분석 및 모니터링

**데이터 내보내기 API:**

```http
GET /api/v1/analytics/alarms/history HTTP/1.1
Host: panel.building.com
Authorization: Bearer <token>

쿼리 파라미터:
  startDate=2025-12-01T00:00:00Z
  endDate=2025-12-27T23:59:59Z
  includeResolved=true
  format=json
```

**응답:**

```json
{
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-27T23:59:59Z"
  },
  "statistics": {
    "totalAlarms": 47,
    "fireAlarms": 3,
    "falseAlarms": 38,
    "supervisory": 4,
    "trouble": 2
  },
  "alarms": [
    {
      "eventId": "...",
      "timestamp": "...",
      "type": "fire",
      "resolved": true,
      "responseTime": 180,
      "resolution": "조리 연기, 화재 없음"
    }
  ],
  "trends": {
    "alarmsPerDay": 1.74,
    "falseAlarmRate": 80.85,
    "averageResponseTime": 245,
    "topAlarmZones": [
      {"zone": "1층 주방", "count": 12},
      {"zone": "15층 기계실", "count": 8}
    ]
  }
}
```

---

## 핵심 요점

1. **조정된 통합**은 빌딩 시스템과 함께 생명 안전 및 재산 보호를 크게 향상시킵니다.

2. **HVAC 통합**은 전략적 팬 차단, 댐퍼 폐쇄 및 가압을 통해 연기 확산을 제어합니다.

3. **출입 통제 통합**은 외곽 보안을 유지하면서 생명 안전 비상구를 우선시합니다.

4. **엘리베이터 리콜**은 엘리베이터를 서비스에서 제거하고 소방관 작동을 가능하게 합니다.

5. **응급 서비스 통합**은 효과적인 대응을 가능하게 하는 포괄적인 사고 정보를 제공합니다.

---

## 복습 질문

1. 화재 경보 활성화 시 트리거되는 HVAC 조치는 무엇입니까?
2. 출입 통제는 어떻게 생명 안전 비상구와 외곽 보안의 균형을 맞춥니까?
3. 소방 서비스 모드에서 엘리베이터 리콜 순서는 무엇입니까?
4. 응급 서비스 알림에 포함되는 정보는 무엇입니까?
5. 타사 애플리케이션은 어떻게 화재 경보 데이터에 액세스합니까?

---

## 다음 단계

8장에서는 로드맵, 테스트 절차, 인증 요구 사항 및 배포 모범 사례를 포함한 구현 지침을 제공합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
