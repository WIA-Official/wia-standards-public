# WIA-CRYO-FACILITY: PHASE 2 - API Interface 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**카테고리:** 극저온 보존 시설 운영
**색상 코드:** #06B6D4 (Cyan)

---

## 1. 소개

### 1.1 목적
본 명세서는 극저온 보존 시설 관리 시스템을 위한 RESTful API 인터페이스를 정의하며, 시설 운영, 모니터링, 직원 관리 및 긴급 대응을 위한 표준화된 통신을 가능하게 합니다.

### 1.2 API 아키텍처
API는 다음과 같은 REST 원칙을 따릅니다:
- JSON 요청/응답 형식
- HTTPS 전용 통신
- OAuth 2.0 인증
- Rate limiting 및 throttling
- 버전 관리된 엔드포인트
- 포괄적인 오류 처리

### 1.3 Base URL 구조
```
https://api.cryo-facility.wia.org/v1/{resource}
```

### 1.4 인증
모든 API 요청은 Bearer 토큰 인증이 필요합니다:
```
Authorization: Bearer {access_token}
```

---

## 2. 시설 관리 API (Facility Management APIs)

### 2.1 GET /facilities

등록된 극저온 보존 시설 목록을 검색합니다.

**요청:**
```http
GET /v1/facilities?status=operational&limit=50&offset=0
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**쿼리 파라미터:**

| 파라미터 | 타입 | 필수 | 설명 |
|-----------|------|----------|-------------|
| status | string | No | 운영 상태별 필터 |
| country | string | No | 국가 코드별 필터 |
| certified | boolean | No | 인증 상태별 필터 |
| limit | integer | No | 결과 수 (기본값: 50, 최대: 100) |
| offset | integer | No | 페이지네이션 오프셋 (기본값: 0) |

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilities": [
      {
        "facilityId": "CRYO-FAC-A7B3C9D2",
        "facilityName": "Phoenix Cryonics Research Facility",
        "location": {
          "city": "Scottsdale",
          "state": "Arizona",
          "country": "United States"
        },
        "operationalStatus": "operational",
        "capacity": {
          "totalCapacity": 200,
          "currentOccupancy": 145,
          "availableSlots": 55
        },
        "certification": {
          "status": "active",
          "expiryDate": "2027-01-15T00:00:00Z"
        }
      }
    ],
    "pagination": {
      "total": 127,
      "limit": 50,
      "offset": 0,
      "hasMore": true
    }
  }
}
```

**오류 응답 (401 Unauthorized):**
```json
{
  "status": "error",
  "timestamp": "2025-12-18T14:22:00Z",
  "error": {
    "code": "AUTH_TOKEN_INVALID",
    "message": "제공된 인증 토큰이 유효하지 않거나 만료되었습니다",
    "details": "토큰이 2025-12-18T12:00:00Z에 만료됨"
  }
}
```

### 2.2 POST /facilities

새로운 극저온 보존 시설을 등록합니다.

**요청:**
```http
POST /v1/facilities
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "facilityName": "New Hope Cryonics Center",
  "location": {
    "address": {
      "street": "1200 Innovation Drive",
      "city": "Austin",
      "state": "Texas",
      "country": "United States",
      "postalCode": "78701"
    },
    "coordinates": {
      "latitude": 30.2672,
      "longitude": -97.7431,
      "altitude": 149
    },
    "timezone": "America/Chicago"
  },
  "capacity": {
    "totalCapacity": 150,
    "capacityByType": {
      "wholebody": 100,
      "neuro": 40,
      "research": 10
    }
  },
  "contactInfo": {
    "primaryContact": {
      "name": "Dr. Robert Chen",
      "role": "Facility Director",
      "email": "r.chen@newhope-cryo.org",
      "phone": "+1-512-555-0200"
    },
    "emergencyContact": {
      "name": "Emergency Operations",
      "phone": "+1-512-555-0911",
      "email": "emergency@newhope-cryo.org",
      "available24x7": true
    }
  }
}
```

**응답 (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-B8C4D1E3",
    "facilityName": "New Hope Cryonics Center",
    "operationalStatus": "offline",
    "message": "시설이 성공적으로 등록되었습니다. 인증 프로세스가 시작되었습니다.",
    "nextSteps": [
      "인증 신청서 작성",
      "시설 검사 일정 잡기",
      "직원 자격 기록 제출"
    ]
  }
}
```

### 2.3 GET /facilities/{facilityId}

특정 시설에 대한 상세 정보를 검색합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "facilityName": "Phoenix Cryonics Research Facility",
    "certification": {
      "certificationId": "CERT-2025-001",
      "issuingBody": "International Cryonics Standards Board",
      "issueDate": "2025-01-15T00:00:00Z",
      "expiryDate": "2027-01-15T00:00:00Z",
      "status": "active"
    },
    "location": {
      "address": {
        "street": "4500 N Scottsdale Road",
        "city": "Scottsdale",
        "state": "Arizona",
        "country": "United States",
        "postalCode": "85251"
      },
      "timezone": "America/Phoenix"
    },
    "capacity": {
      "totalCapacity": 200,
      "currentOccupancy": 145,
      "availableSlots": 55
    },
    "operationalStatus": "operational"
  }
}
```

---

## 3. Dewar 관리 API (Dewar Management APIs)

### 3.1 GET /facilities/{facilityId}/dewars

시설의 모든 Dewar를 나열합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/dewars?status=active
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**쿼리 파라미터:**

| 파라미터 | 타입 | 필수 | 설명 |
|-----------|------|----------|-------------|
| status | string | No | Dewar 상태별 필터 |
| type | string | No | Dewar 유형별 필터 |
| lowNitrogen | boolean | No | 보충 임계값 이하의 Dewar만 표시 |

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "dewars": [
      {
        "dewarId": "DEWAR-BF01XL2025",
        "dewarType": "bigfoot",
        "currentStatus": "active",
        "liquidNitrogenLevel": {
          "currentLevel": 72.5,
          "criticalLevel": 30.0,
          "status": "normal"
        },
        "temperature": {
          "currentTemp": 77.2,
          "criticalTemp": 85.0,
          "status": "normal"
        },
        "capacity": {
          "patientCapacity": 8,
          "currentOccupancy": 6
        }
      }
    ],
    "summary": {
      "totalDewars": 12,
      "activeCount": 10,
      "maintenanceCount": 1,
      "alarmCount": 0
    }
  }
}
```

### 3.2 POST /dewars/{dewarId}/refill

액체 질소 보충 작업을 기록합니다.

**요청:**
```http
POST /v1/dewars/DEWAR-BF01XL2025/refill
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "refillDate": "2025-12-18T08:30:00Z",
  "volumeAdded": 280,
  "performedBy": "STAFF-TC0012ABC",
  "supplier": "AirLiquide",
  "batchNumber": "LN2-2025-12-18-001",
  "notes": "정기 예정 보충. 모든 시스템 정상."
}
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "refillId": "REFILL-2025-1234",
    "dewarId": "DEWAR-BF01XL2025",
    "newLevel": 98.5,
    "nextScheduledRefill": "2025-12-25T08:00:00Z",
    "recorded": true
  }
}
```

---

## 4. 환경 모니터링 API (Environmental Monitoring APIs)

### 4.1 GET /facilities/{facilityId}/environmental

시설의 현재 환경 조건을 가져옵니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/environmental
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "monitoringId": "ENV-MON-2025121814",
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "timestamp": "2025-12-18T14:22:00Z",
    "sensorData": {
      "temperature": {
        "ambient": 22.5,
        "status": "normal",
        "dewars": [
          {
            "dewarId": "DEWAR-BF01XL2025",
            "temperature": 77.2,
            "status": "normal"
          }
        ]
      },
      "pressure": {
        "atmospheric": 101.2,
        "status": "normal"
      },
      "humidity": {
        "relativeHumidity": 35.2,
        "status": "normal"
      },
      "oxygenLevel": {
        "percentage": 20.8,
        "status": "normal"
      }
    },
    "status": "normal",
    "alerts": []
  }
}
```

### 4.2 GET /facilities/{facilityId}/environmental/history

과거 환경 데이터를 검색합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/environmental/history?from=2025-12-01T00:00:00Z&to=2025-12-18T23:59:59Z&interval=1h
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**쿼리 파라미터:**

| 파라미터 | 타입 | 필수 | 설명 |
|-----------|------|----------|-------------|
| from | datetime | Yes | 시작 날짜/시간 (ISO 8601) |
| to | datetime | Yes | 종료 날짜/시간 (ISO 8601) |
| interval | string | No | 데이터 집계 간격 (1m, 5m, 15m, 1h, 1d) |
| metrics | string | No | 포함할 메트릭 (쉼표로 구분) |

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "period": {
      "from": "2025-12-01T00:00:00Z",
      "to": "2025-12-18T23:59:59Z",
      "interval": "1h"
    },
    "dataPoints": 432,
    "readings": [
      {
        "timestamp": "2025-12-01T00:00:00Z",
        "temperature": {
          "ambient": 22.3,
          "dewarAverage": 77.4
        },
        "humidity": 36.1,
        "oxygenLevel": 20.9
      }
    ],
    "summary": {
      "temperature": {
        "min": 21.5,
        "max": 23.2,
        "average": 22.4
      },
      "alertsTriggered": 0
    }
  }
}
```

---

## 5. 직원 관리 API (Staff Management APIs)

### 5.1 GET /facilities/{facilityId}/staff

시설의 모든 직원을 나열합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/staff?status=active&role=cryonics_technician
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**쿼리 파라미터:**

| 파라미터 | 타입 | 필수 | 설명 |
|-----------|------|----------|-------------|
| status | string | No | 고용 상태별 필터 |
| role | string | No | 역할별 필터 |
| certificationExpiring | integer | No | N일 내에 만료되는 인증을 가진 직원 표시 |

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "staff": [
      {
        "staffId": "STAFF-TC0012ABC",
        "personalInfo": {
          "firstName": "James",
          "lastName": "Rodriguez",
          "email": "j.rodriguez@phoenix-cryo.org"
        },
        "employmentStatus": {
          "status": "active",
          "role": "Senior Cryonics Technician",
          "department": "Operations",
          "startDate": "2019-06-01"
        },
        "qualifications": {
          "certificationsCount": 2,
          "expiringCertifications": 1
        }
      }
    ],
    "summary": {
      "totalStaff": 45,
      "activeStaff": 42,
      "onLeave": 2,
      "suspended": 1
    }
  }
}
```

### 5.2 POST /staff/{staffId}/training

교육 과정 완료를 기록합니다.

**요청:**
```http
POST /v1/staff/STAFF-TC0012ABC/training
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "trainingName": "Advanced Dewar Maintenance 2025",
  "trainingType": "advanced",
  "completionDate": "2025-12-15T16:30:00Z",
  "expiryDate": "2027-12-15T16:30:00Z",
  "score": 92,
  "instructor": "Dr. Emily Chen",
  "certificateIssued": true
}
```

**응답 (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "trainingId": "TRAIN-2025-0234",
    "staffId": "STAFF-TC0012ABC",
    "recorded": true,
    "certificateUrl": "https://certificates.cryo-facility.wia.org/TRAIN-2025-0234.pdf"
  }
}
```

---

## 6. 긴급 프로토콜 API (Emergency Protocol APIs)

### 6.1 GET /facilities/{facilityId}/emergency-protocols

시설의 모든 긴급 프로토콜을 나열합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/emergency-protocols
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "protocols": [
      {
        "protocolId": "EMERG-PROTO-001",
        "protocolName": "Dewar Temperature Critical Alert",
        "severity": "critical",
        "lastDrillDate": "2025-09-15T10:00:00Z",
        "nextScheduledDrill": "2026-03-15T10:00:00Z",
        "status": "active"
      },
      {
        "protocolId": "EMERG-PROTO-002",
        "protocolName": "Power Outage Response",
        "severity": "critical",
        "lastDrillDate": "2025-10-20T14:00:00Z",
        "nextScheduledDrill": "2026-04-20T14:00:00Z",
        "status": "active"
      }
    ]
  }
}
```

### 6.2 POST /facilities/{facilityId}/emergency/activate

긴급 프로토콜을 활성화합니다.

**요청:**
```http
POST /v1/facilities/CRYO-FAC-A7B3C9D2/emergency/activate
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "protocolId": "EMERG-PROTO-001",
  "triggerReason": "Dewar DEWAR-BF02XL2025 온도가 임계 임계값 초과",
  "triggeredBy": "STAFF-TC0012ABC",
  "affectedAssets": ["DEWAR-BF02XL2025"],
  "severity": "critical"
}
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "incidentId": "INCIDENT-2025-0089",
    "protocolId": "EMERG-PROTO-001",
    "status": "activated",
    "responseTeam": [
      {
        "role": "Emergency Coordinator",
        "name": "Dr. Sarah Mitchell",
        "notified": true,
        "notificationTime": "2025-12-18T14:22:01Z"
      },
      {
        "role": "Senior Cryonics Technician",
        "name": "James Rodriguez",
        "notified": true,
        "notificationTime": "2025-12-18T14:22:02Z"
      }
    ],
    "responseSteps": [
      {
        "step": 1,
        "action": "immediate_alert",
        "status": "completed",
        "completedAt": "2025-12-18T14:22:05Z"
      },
      {
        "step": 2,
        "action": "initiate_backup_cooling",
        "status": "in_progress",
        "assignedTo": "STAFF-TC0012ABC"
      }
    ],
    "message": "긴급 프로토콜이 활성화되었습니다. 대응팀에 알림이 전송되었습니다."
  }
}
```

---

## 7. 규정 준수 및 보고 API (Compliance and Reporting APIs)

### 7.1 GET /facilities/{facilityId}/compliance/status

시설의 규정 준수 상태를 가져옵니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/compliance/status
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "overallStatus": "compliant",
    "lastAudit": "2025-09-15T10:00:00Z",
    "nextAudit": "2026-03-15T10:00:00Z",
    "complianceAreas": {
      "certification": {
        "status": "compliant",
        "validUntil": "2027-01-15T00:00:00Z",
        "issues": []
      },
      "staffQualifications": {
        "status": "warning",
        "issues": [
          "3명의 직원이 60일 내에 만료되는 인증을 가지고 있음"
        ]
      },
      "maintenanceSchedule": {
        "status": "compliant",
        "overdueItems": 0,
        "upcomingItems": 5
      },
      "emergencyProtocols": {
        "status": "compliant",
        "lastDrillDate": "2025-11-10T09:00:00Z",
        "nextDrillDate": "2026-02-10T09:00:00Z"
      }
    },
    "recommendations": [
      "3명의 직원에 대한 인증 갱신 교육 일정 잡기",
      "새 직원 포함하도록 긴급 연락처 목록 업데이트"
    ]
  }
}
```

### 7.2 GET /facilities/{facilityId}/reports/monthly

월간 운영 보고서를 생성합니다.

**요청:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/reports/monthly?month=2025-12
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "reportPeriod": {
      "month": "2025-12",
      "from": "2025-12-01T00:00:00Z",
      "to": "2025-12-18T23:59:59Z"
    },
    "operationalSummary": {
      "uptimePercentage": 99.97,
      "incidentsReported": 2,
      "criticalIncidents": 0,
      "maintenanceHours": 24,
      "staffHours": 3240
    },
    "dewarPerformance": {
      "totalDewars": 12,
      "averageTemperature": 77.3,
      "nitrogenRefills": 48,
      "totalNitrogenConsumed": "3420 L",
      "alarmsTriggered": 3,
      "criticalAlarms": 0
    },
    "reportUrl": "https://reports.cryo-facility.wia.org/CRYO-FAC-A7B3C9D2/2025-12.pdf"
  }
}
```

---

## 8. API 응답 코드

### 8.1 성공 코드

| 코드 | 메시지 | 설명 |
|------|---------|-------------|
| 200 | OK | 요청 성공 |
| 201 | Created | 리소스 생성 성공 |
| 202 | Accepted | 요청이 수락되었으며 비동기적으로 처리 중 |
| 204 | No Content | 요청 성공, 반환할 콘텐츠 없음 |

### 8.2 클라이언트 오류 코드

| 코드 | 메시지 | 설명 |
|------|---------|-------------|
| 400 | Bad Request | 잘못된 요청 형식 또는 파라미터 |
| 401 | Unauthorized | 인증 필요 또는 실패 |
| 403 | Forbidden | 인증되었으나 권한 없음 |
| 404 | Not Found | 리소스를 찾을 수 없음 |
| 409 | Conflict | 리소스 충돌 (중복, 상태 위반) |
| 422 | Unprocessable Entity | 유효성 검사 오류 |
| 429 | Too Many Requests | Rate limit 초과 |

### 8.3 서버 오류 코드

| 코드 | 메시지 | 설명 |
|------|---------|-------------|
| 500 | Internal Server Error | 예상치 못한 서버 오류 |
| 502 | Bad Gateway | 업스트림 서비스 오류 |
| 503 | Service Unavailable | 서비스 일시적으로 사용 불가 |
| 504 | Gateway Timeout | 업스트림 서비스 시간 초과 |

---

## 9. Rate Limiting

### 9.1 Rate Limit 헤더

모든 API 응답에는 rate limiting 헤더가 포함됩니다:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1703088000
```

### 9.2 Rate Limit 티어

| 티어 | 시간당 요청 | Burst Limit |
|------|---------------|-------------|
| Basic | 1,000 | 50 |
| Professional | 5,000 | 200 |
| Enterprise | 20,000 | 1,000 |
| Emergency | 무제한 | 무제한 |

---

## 10. Webhooks

### 10.1 Webhook 이벤트

| 이벤트 | 설명 | Payload |
|-------|-------------|---------|
| dewar.alert.critical | 치명적 Dewar 경고 발생 | DewarAlert |
| facility.status.changed | 시설 상태 변경 | FacilityStatus |
| staff.certification.expiring | 직원 인증 곧 만료 | StaffCertification |
| emergency.activated | 긴급 프로토콜 활성화 | EmergencyIncident |
| maintenance.overdue | 유지보수 작업 지연 | MaintenanceTask |

### 10.2 Webhook 등록 예제

**요청:**
```http
POST /v1/webhooks
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "url": "https://myapp.example.com/webhooks/cryo-facility",
  "events": [
    "dewar.alert.critical",
    "emergency.activated"
  ],
  "secret": "whsec_7f8d9e0a1b2c3d4e5f6g7h8i9j0k1l2m"
}
```

**응답 (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "webhookId": "WEBHOOK-A1B2C3D4",
    "url": "https://myapp.example.com/webhooks/cryo-facility",
    "events": [
      "dewar.alert.critical",
      "emergency.activated"
    ],
    "status": "active",
    "createdAt": "2025-12-18T14:22:00Z"
  }
}
```

---

## 11. 버전 이력

| 버전 | 날짜 | 변경사항 |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | 초기 API 명세서 |

---

## 12. 참조

- REST API Design Best Practices
- OAuth 2.0 RFC 6749
- JSON Schema Specification
- OpenAPI 3.0 Specification

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
