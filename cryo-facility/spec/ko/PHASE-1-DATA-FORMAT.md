# WIA-CRYO-FACILITY: PHASE 1 - Data Format 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**카테고리:** 극저온 보존 시설 운영
**색상 코드:** #06B6D4 (Cyan)

---

## 1. 소개

### 1.1 목적
본 명세서는 극저온 보존 시설 운영을 위한 표준화된 데이터 형식을 정의합니다. 시설 인증, Dewar 관리, 환경 모니터링, 직원 자격, 긴급 프로토콜 및 환자 배치 시스템을 포함합니다.

### 1.2 범위
본 문서는 다음을 포함합니다:
- 시설 인증 및 승인 데이터 구조
- Dewar 관리 및 액체 질소 시스템
- 환경 모니터링 (온도, 압력, 습도)
- 직원 자격 및 교육 기록
- 긴급 프로토콜 및 재난 복구
- 용량 계획 및 환자 배치
- 보안 시스템 및 접근 제어
- 유지보수 일정 및 규정 준수 추적

### 1.3 대상 독자
- 극저온 보존 시설 관리자
- 시설 관리 소프트웨어 개발자
- 규정 준수 및 규제 담당자
- 품질 보증 담당자
- 긴급 대응 조정자

---

## 2. 핵심 데이터 구조

### 2.1 시설 정보 스키마 (Facility Information Schema)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "CryonicsFacility",
  "type": "object",
  "required": [
    "facilityId",
    "facilityName",
    "certification",
    "location",
    "capacity",
    "operationalStatus",
    "contactInfo"
  ],
  "properties": {
    "facilityId": {
      "type": "string",
      "pattern": "^CRYO-FAC-[A-Z0-9]{8}$",
      "description": "고유 시설 식별자"
    },
    "facilityName": {
      "type": "string",
      "minLength": 3,
      "maxLength": 200,
      "description": "공식 시설 명칭"
    },
    "certification": {
      "type": "object",
      "required": ["certificationId", "issuingBody", "issueDate", "expiryDate", "status"],
      "properties": {
        "certificationId": {
          "type": "string",
          "description": "인증 식별자"
        },
        "issuingBody": {
          "type": "string",
          "description": "인증 기관"
        },
        "issueDate": {
          "type": "string",
          "format": "date-time"
        },
        "expiryDate": {
          "type": "string",
          "format": "date-time"
        },
        "status": {
          "type": "string",
          "enum": ["active", "pending_renewal", "expired", "suspended", "revoked"]
        },
        "accreditations": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "accreditationName": {"type": "string"},
              "accreditationDate": {"type": "string", "format": "date-time"},
              "validUntil": {"type": "string", "format": "date-time"}
            }
          }
        }
      }
    },
    "location": {
      "type": "object",
      "required": ["address", "coordinates", "timezone"],
      "properties": {
        "address": {
          "type": "object",
          "properties": {
            "street": {"type": "string"},
            "city": {"type": "string"},
            "state": {"type": "string"},
            "country": {"type": "string"},
            "postalCode": {"type": "string"}
          }
        },
        "coordinates": {
          "type": "object",
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180},
            "altitude": {"type": "number", "description": "해발 고도 (미터)"}
          }
        },
        "timezone": {
          "type": "string",
          "description": "IANA timezone 식별자"
        }
      }
    },
    "capacity": {
      "type": "object",
      "required": ["totalCapacity", "currentOccupancy", "availableSlots"],
      "properties": {
        "totalCapacity": {"type": "integer", "minimum": 1},
        "currentOccupancy": {"type": "integer", "minimum": 0},
        "availableSlots": {"type": "integer", "minimum": 0},
        "capacityByType": {
          "type": "object",
          "properties": {
            "wholebody": {"type": "integer"},
            "neuro": {"type": "integer"},
            "research": {"type": "integer"}
          }
        }
      }
    },
    "operationalStatus": {
      "type": "string",
      "enum": ["operational", "maintenance", "emergency", "offline"]
    },
    "contactInfo": {
      "type": "object",
      "properties": {
        "primaryContact": {
          "type": "object",
          "properties": {
            "name": {"type": "string"},
            "role": {"type": "string"},
            "email": {"type": "string", "format": "email"},
            "phone": {"type": "string"}
          }
        },
        "emergencyContact": {
          "type": "object",
          "properties": {
            "name": {"type": "string"},
            "phone": {"type": "string"},
            "email": {"type": "string", "format": "email"},
            "available24x7": {"type": "boolean"}
          }
        }
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "createdAt": {"type": "string", "format": "date-time"},
        "updatedAt": {"type": "string", "format": "date-time"},
        "version": {"type": "string"}
      }
    }
  }
}
```

### 2.2 Dewar 관리 스키마 (Dewar Management Schema)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DewarUnit",
  "type": "object",
  "required": [
    "dewarId",
    "facilityId",
    "dewarType",
    "capacity",
    "currentStatus",
    "liquidNitrogenLevel",
    "temperature"
  ],
  "properties": {
    "dewarId": {
      "type": "string",
      "pattern": "^DEWAR-[A-Z0-9]{10}$",
      "description": "고유 Dewar 식별자"
    },
    "facilityId": {
      "type": "string",
      "pattern": "^CRYO-FAC-[A-Z0-9]{8}$",
      "description": "상위 시설 식별자"
    },
    "dewarType": {
      "type": "string",
      "enum": ["bigfoot", "MVE", "custom", "portable", "transport"]
    },
    "manufacturerInfo": {
      "type": "object",
      "properties": {
        "manufacturer": {"type": "string"},
        "model": {"type": "string"},
        "serialNumber": {"type": "string"},
        "manufactureDate": {"type": "string", "format": "date"},
        "warrantyExpiry": {"type": "string", "format": "date"}
      }
    },
    "capacity": {
      "type": "object",
      "required": ["volumeLiters", "patientCapacity"],
      "properties": {
        "volumeLiters": {"type": "number", "minimum": 0},
        "patientCapacity": {"type": "integer", "minimum": 1},
        "currentOccupancy": {"type": "integer", "minimum": 0}
      }
    },
    "currentStatus": {
      "type": "string",
      "enum": ["active", "standby", "maintenance", "alarm", "offline", "decommissioned"]
    },
    "liquidNitrogenLevel": {
      "type": "object",
      "required": ["currentLevel", "criticalLevel", "lastRefill"],
      "properties": {
        "currentLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "전체 용량의 백분율"
        },
        "criticalLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "경고 임계값 백분율"
        },
        "lastRefill": {
          "type": "string",
          "format": "date-time"
        },
        "nextScheduledRefill": {
          "type": "string",
          "format": "date-time"
        },
        "averageConsumptionRate": {
          "type": "number",
          "description": "일일 리터 소비량"
        }
      }
    },
    "temperature": {
      "type": "object",
      "required": ["currentTemp", "criticalTemp"],
      "properties": {
        "currentTemp": {
          "type": "number",
          "description": "켈빈 온도"
        },
        "criticalTemp": {
          "type": "number",
          "description": "최대 안전 온도 (켈빈)"
        },
        "sensorLocations": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "sensorId": {"type": "string"},
              "location": {"type": "string"},
              "currentReading": {"type": "number"},
              "lastCalibration": {"type": "string", "format": "date-time"}
            }
          }
        }
      }
    },
    "patients": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "patientId": {"type": "string"},
          "position": {"type": "string"},
          "admissionDate": {"type": "string", "format": "date-time"},
          "caseType": {"type": "string", "enum": ["wholebody", "neuro"]}
        }
      }
    },
    "maintenanceHistory": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "maintenanceId": {"type": "string"},
          "date": {"type": "string", "format": "date-time"},
          "type": {"type": "string"},
          "performedBy": {"type": "string"},
          "notes": {"type": "string"}
        }
      }
    },
    "alarmHistory": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "alarmId": {"type": "string"},
          "timestamp": {"type": "string", "format": "date-time"},
          "severity": {"type": "string", "enum": ["critical", "warning", "info"]},
          "alarmType": {"type": "string"},
          "resolved": {"type": "boolean"},
          "resolvedAt": {"type": "string", "format": "date-time"}
        }
      }
    }
  }
}
```

### 2.3 환경 모니터링 스키마 (Environmental Monitoring Schema)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "EnvironmentalMonitoring",
  "type": "object",
  "required": [
    "monitoringId",
    "facilityId",
    "timestamp",
    "sensorData",
    "status"
  ],
  "properties": {
    "monitoringId": {
      "type": "string",
      "pattern": "^ENV-MON-[A-Z0-9]{12}$"
    },
    "facilityId": {
      "type": "string",
      "pattern": "^CRYO-FAC-[A-Z0-9]{8}$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sensorData": {
      "type": "object",
      "properties": {
        "temperature": {
          "type": "object",
          "properties": {
            "ambient": {
              "type": "number",
              "description": "섭씨 실내 온도"
            },
            "dewars": {
              "type": "array",
              "items": {
                "type": "object",
                "properties": {
                  "dewarId": {"type": "string"},
                  "temperature": {"type": "number", "description": "켈빈"},
                  "status": {"type": "string", "enum": ["normal", "warning", "critical"]}
                }
              }
            }
          }
        },
        "pressure": {
          "type": "object",
          "properties": {
            "atmospheric": {"type": "number", "description": "kPa 단위 압력"},
            "dewarPressure": {
              "type": "array",
              "items": {
                "type": "object",
                "properties": {
                  "dewarId": {"type": "string"},
                  "pressure": {"type": "number"},
                  "status": {"type": "string"}
                }
              }
            }
          }
        },
        "humidity": {
          "type": "object",
          "properties": {
            "relativeHumidity": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "status": {"type": "string", "enum": ["normal", "high", "low"]}
          }
        },
        "oxygenLevel": {
          "type": "object",
          "properties": {
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            },
            "status": {"type": "string", "enum": ["normal", "warning", "critical"]}
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["normal", "warning", "critical", "maintenance"]
    },
    "alerts": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "alertId": {"type": "string"},
          "severity": {"type": "string", "enum": ["info", "warning", "critical"]},
          "message": {"type": "string"},
          "timestamp": {"type": "string", "format": "date-time"},
          "acknowledged": {"type": "boolean"}
        }
      }
    }
  }
}
```

### 2.4 직원 자격 스키마 (Staff Qualifications Schema)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "StaffMember",
  "type": "object",
  "required": [
    "staffId",
    "facilityId",
    "personalInfo",
    "qualifications",
    "trainingRecords",
    "employmentStatus"
  ],
  "properties": {
    "staffId": {
      "type": "string",
      "pattern": "^STAFF-[A-Z0-9]{10}$"
    },
    "facilityId": {
      "type": "string",
      "pattern": "^CRYO-FAC-[A-Z0-9]{8}$"
    },
    "personalInfo": {
      "type": "object",
      "properties": {
        "firstName": {"type": "string"},
        "lastName": {"type": "string"},
        "email": {"type": "string", "format": "email"},
        "phone": {"type": "string"},
        "emergencyContact": {
          "type": "object",
          "properties": {
            "name": {"type": "string"},
            "relationship": {"type": "string"},
            "phone": {"type": "string"}
          }
        }
      }
    },
    "qualifications": {
      "type": "object",
      "properties": {
        "education": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "degree": {"type": "string"},
              "institution": {"type": "string"},
              "completionDate": {"type": "string", "format": "date"},
              "verified": {"type": "boolean"}
            }
          }
        },
        "certifications": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "certificationName": {"type": "string"},
              "issuingOrganization": {"type": "string"},
              "issueDate": {"type": "string", "format": "date"},
              "expiryDate": {"type": "string", "format": "date"},
              "certificationNumber": {"type": "string"},
              "status": {"type": "string", "enum": ["valid", "expired", "pending_renewal"]}
            }
          }
        },
        "specializations": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": [
              "cryopreservation",
              "liquid_nitrogen_systems",
              "emergency_response",
              "facility_management",
              "quality_assurance",
              "regulatory_compliance"
            ]
          }
        }
      }
    },
    "trainingRecords": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "trainingId": {"type": "string"},
          "trainingName": {"type": "string"},
          "trainingType": {
            "type": "string",
            "enum": ["initial", "refresher", "advanced", "mandatory"]
          },
          "completionDate": {"type": "string", "format": "date-time"},
          "expiryDate": {"type": "string", "format": "date-time"},
          "score": {"type": "number", "minimum": 0, "maximum": 100},
          "instructor": {"type": "string"},
          "certificateIssued": {"type": "boolean"}
        }
      }
    },
    "employmentStatus": {
      "type": "object",
      "properties": {
        "status": {
          "type": "string",
          "enum": ["active", "on_leave", "suspended", "terminated"]
        },
        "role": {"type": "string"},
        "department": {"type": "string"},
        "startDate": {"type": "string", "format": "date"},
        "endDate": {"type": "string", "format": "date"},
        "accessLevel": {
          "type": "string",
          "enum": ["basic", "advanced", "supervisor", "administrator"]
        }
      }
    },
    "accessCredentials": {
      "type": "object",
      "properties": {
        "badgeId": {"type": "string"},
        "biometricEnrolled": {"type": "boolean"},
        "authorizedAreas": {
          "type": "array",
          "items": {"type": "string"}
        },
        "lastAccessReview": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

---

## 3. 데이터 형식 테이블

### 3.1 시설 상태 코드 (Facility Status Codes)

| 상태 코드 | 설명 | 경고 수준 | 자동 대응 |
|-------------|-------------|-------------|---------------|
| OPERATIONAL | 정상 운영 | 없음 | 모니터링 지속 |
| MAINTENANCE | 예정된 유지보수 | Info | 직원 알림 |
| DEGRADED | 부분 기능 | Warning | 감독자 경고 |
| EMERGENCY | 치명적 고장 | Critical | 긴급 프로토콜 활성화 |
| OFFLINE | 시설 오프라인 | Critical | 즉각 대응 필요 |

### 3.2 온도 임계값 (Temperature Thresholds)

| 구역 | 정상 범위 (K) | 경고 임계값 (K) | 치명적 임계값 (K) | 필요 조치 |
|------|------------------|----------------------|------------------------|-----------------|
| Dewar 내부 | 77-78 | 80 | 85 | 즉시 질소 보충 |
| 저장실 | 273-278 | 280 | 285 | HVAC 조정 |
| 장비실 | 288-293 | 295 | 300 | 냉각 시스템 점검 |
| 사무 공간 | 293-297 | 300 | 305 | 환경 제어 |

### 3.3 액체 질소 보충 일정 (Liquid Nitrogen Refill Schedule)

| Dewar 유형 | 용량 (L) | 일반 증발률 (L/일) | 보충 임계값 (%) | 보충 빈도 |
|------------|--------------|----------------------------------|---------------------|------------------|
| Bigfoot XL | 1000 | 5-8 | 30% | 주간 |
| MVE 1500 | 1500 | 8-12 | 25% | 격주 |
| Portable 300 | 300 | 2-3 | 40% | 3-4일 |
| Transport 150 | 150 | 3-5 | 50% | 매일 |

### 3.4 직원 인증 요구사항 (Staff Certification Requirements)

| 역할 | 필수 인증 | 갱신 기간 | 연간 교육 시간 |
|------|------------------------|----------------|---------------------|
| Facility Manager | 시설 관리, 안전 | 2년 | 40 |
| Cryonics Technician | 극저온 보존, LN2 취급 | 1년 | 60 |
| Quality Assurance Officer | 품질 시스템, 규정 준수 | 2년 | 30 |
| Emergency Response Lead | 긴급 관리, 응급 처치 | 1년 | 50 |
| Security Officer | 보안 시스템, 접근 제어 | 2년 | 25 |

---

## 4. 코드 예제

### 4.1 시설 등록 예제 (Facility Registration Example)

```json
{
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "facilityName": "Phoenix Cryonics Research Facility",
  "certification": {
    "certificationId": "CERT-2025-001",
    "issuingBody": "International Cryonics Standards Board",
    "issueDate": "2025-01-15T00:00:00Z",
    "expiryDate": "2027-01-15T00:00:00Z",
    "status": "active",
    "accreditations": [
      {
        "accreditationName": "ISO 9001:2015 Quality Management",
        "accreditationDate": "2025-02-01T00:00:00Z",
        "validUntil": "2028-02-01T00:00:00Z"
      },
      {
        "accreditationName": "OSHA Safety Compliance",
        "accreditationDate": "2025-01-20T00:00:00Z",
        "validUntil": "2026-01-20T00:00:00Z"
      }
    ]
  },
  "location": {
    "address": {
      "street": "4500 N Scottsdale Road",
      "city": "Scottsdale",
      "state": "Arizona",
      "country": "United States",
      "postalCode": "85251"
    },
    "coordinates": {
      "latitude": 33.4942,
      "longitude": -111.9261,
      "altitude": 393
    },
    "timezone": "America/Phoenix"
  },
  "capacity": {
    "totalCapacity": 200,
    "currentOccupancy": 145,
    "availableSlots": 55,
    "capacityByType": {
      "wholebody": 120,
      "neuro": 60,
      "research": 20
    }
  },
  "operationalStatus": "operational",
  "contactInfo": {
    "primaryContact": {
      "name": "Dr. Sarah Mitchell",
      "role": "Facility Director",
      "email": "s.mitchell@phoenix-cryo.org",
      "phone": "+1-480-555-0100"
    },
    "emergencyContact": {
      "name": "Operations Center",
      "phone": "+1-480-555-0911",
      "email": "emergency@phoenix-cryo.org",
      "available24x7": true
    }
  },
  "metadata": {
    "createdAt": "2025-01-15T10:30:00Z",
    "updatedAt": "2025-12-18T14:22:00Z",
    "version": "1.0.0"
  }
}
```

### 4.2 Dewar 모니터링 데이터 예제

```json
{
  "dewarId": "DEWAR-BF01XL2025",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "dewarType": "bigfoot",
  "manufacturerInfo": {
    "manufacturer": "Custom Biogenic Systems",
    "model": "Bigfoot XL-1000",
    "serialNumber": "CBS-BF-2024-0127",
    "manufactureDate": "2024-06-15",
    "warrantyExpiry": "2029-06-15"
  },
  "capacity": {
    "volumeLiters": 1000,
    "patientCapacity": 8,
    "currentOccupancy": 6
  },
  "currentStatus": "active",
  "liquidNitrogenLevel": {
    "currentLevel": 72.5,
    "criticalLevel": 30.0,
    "lastRefill": "2025-12-16T08:30:00Z",
    "nextScheduledRefill": "2025-12-23T08:00:00Z",
    "averageConsumptionRate": 6.8
  },
  "temperature": {
    "currentTemp": 77.2,
    "criticalTemp": 85.0,
    "sensorLocations": [
      {
        "sensorId": "TEMP-SENSOR-001",
        "location": "upper_chamber",
        "currentReading": 77.1,
        "lastCalibration": "2025-11-01T10:00:00Z"
      },
      {
        "sensorId": "TEMP-SENSOR-002",
        "location": "middle_chamber",
        "currentReading": 77.2,
        "lastCalibration": "2025-11-01T10:15:00Z"
      },
      {
        "sensorId": "TEMP-SENSOR-003",
        "location": "lower_chamber",
        "currentReading": 77.3,
        "lastCalibration": "2025-11-01T10:30:00Z"
      }
    ]
  },
  "patients": [
    {
      "patientId": "PATIENT-2023-0145",
      "position": "chamber_1_upper",
      "admissionDate": "2023-08-22T14:30:00Z",
      "caseType": "wholebody"
    },
    {
      "patientId": "PATIENT-2024-0089",
      "position": "chamber_2_middle",
      "admissionDate": "2024-03-15T09:20:00Z",
      "caseType": "wholebody"
    }
  ],
  "maintenanceHistory": [
    {
      "maintenanceId": "MAINT-2025-0234",
      "date": "2025-11-01T09:00:00Z",
      "type": "calibration",
      "performedBy": "STAFF-TC0012ABC",
      "notes": "연간 센서 교정 완료. 모든 센서 정상 범위 내."
    }
  ],
  "alarmHistory": [
    {
      "alarmId": "ALARM-2025-0456",
      "timestamp": "2025-10-15T03:22:00Z",
      "severity": "warning",
      "alarmType": "low_nitrogen_level",
      "resolved": true,
      "resolvedAt": "2025-10-15T08:30:00Z"
    }
  ]
}
```

### 4.3 환경 모니터링 예제

```json
{
  "monitoringId": "ENV-MON-2025121814",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "timestamp": "2025-12-18T14:22:00Z",
  "sensorData": {
    "temperature": {
      "ambient": 22.5,
      "dewars": [
        {
          "dewarId": "DEWAR-BF01XL2025",
          "temperature": 77.2,
          "status": "normal"
        },
        {
          "dewarId": "DEWAR-BF02XL2025",
          "temperature": 77.4,
          "status": "normal"
        },
        {
          "dewarId": "DEWAR-MV01152025",
          "temperature": 78.1,
          "status": "normal"
        }
      ]
    },
    "pressure": {
      "atmospheric": 101.2,
      "dewarPressure": [
        {
          "dewarId": "DEWAR-BF01XL2025",
          "pressure": 0.12,
          "status": "normal"
        },
        {
          "dewarId": "DEWAR-BF02XL2025",
          "pressure": 0.14,
          "status": "normal"
        }
      ]
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
```

### 4.4 직원 교육 기록 예제

```json
{
  "staffId": "STAFF-TC0012ABC",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "personalInfo": {
    "firstName": "James",
    "lastName": "Rodriguez",
    "email": "j.rodriguez@phoenix-cryo.org",
    "phone": "+1-480-555-0234",
    "emergencyContact": {
      "name": "Maria Rodriguez",
      "relationship": "Spouse",
      "phone": "+1-480-555-0235"
    }
  },
  "qualifications": {
    "education": [
      {
        "degree": "Bachelor of Science in Cryobiology",
        "institution": "Arizona State University",
        "completionDate": "2018-05-15",
        "verified": true
      }
    ],
    "certifications": [
      {
        "certificationName": "Certified Cryonics Technician",
        "issuingOrganization": "American Cryonics Society",
        "issueDate": "2019-03-20",
        "expiryDate": "2026-03-20",
        "certificationNumber": "CCT-2019-0456",
        "status": "valid"
      },
      {
        "certificationName": "Liquid Nitrogen Safety Handler",
        "issuingOrganization": "OSHA",
        "issueDate": "2024-06-10",
        "expiryDate": "2025-06-10",
        "certificationNumber": "LN2-2024-7823",
        "status": "pending_renewal"
      }
    ],
    "specializations": [
      "cryopreservation",
      "liquid_nitrogen_systems",
      "emergency_response"
    ]
  },
  "trainingRecords": [
    {
      "trainingId": "TRAIN-2025-0089",
      "trainingName": "Emergency Response Procedures",
      "trainingType": "refresher",
      "completionDate": "2025-11-15T16:30:00Z",
      "expiryDate": "2026-11-15T16:30:00Z",
      "score": 96,
      "instructor": "STAFF-ER0001XYZ",
      "certificateIssued": true
    },
    {
      "trainingId": "TRAIN-2025-0156",
      "trainingName": "Advanced Dewar Maintenance",
      "trainingType": "advanced",
      "completionDate": "2025-09-22T14:00:00Z",
      "expiryDate": "2027-09-22T14:00:00Z",
      "score": 94,
      "instructor": "Dr. Emily Chen",
      "certificateIssued": true
    }
  ],
  "employmentStatus": {
    "status": "active",
    "role": "Senior Cryonics Technician",
    "department": "Operations",
    "startDate": "2019-06-01",
    "accessLevel": "advanced"
  },
  "accessCredentials": {
    "badgeId": "BADGE-TC-0012",
    "biometricEnrolled": true,
    "authorizedAreas": [
      "storage_facility",
      "dewar_room_1",
      "dewar_room_2",
      "equipment_room",
      "monitoring_center"
    ],
    "lastAccessReview": "2025-10-01T10:00:00Z"
  }
}
```

### 4.5 긴급 프로토콜 데이터 예제

```json
{
  "protocolId": "EMERG-PROTO-001",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "protocolName": "Dewar Temperature Critical Alert",
  "severity": "critical",
  "triggerConditions": [
    {
      "condition": "dewar_temperature_above_critical",
      "threshold": 85.0,
      "unit": "kelvin"
    },
    {
      "condition": "nitrogen_level_below_critical",
      "threshold": 10.0,
      "unit": "percent"
    }
  ],
  "responseSteps": [
    {
      "step": 1,
      "action": "immediate_alert",
      "description": "모든 시설 경보 발동 및 긴급 대응팀 알림",
      "requiredResponseTime": 60,
      "assignedRoles": ["emergency_coordinator", "facility_manager"]
    },
    {
      "step": 2,
      "action": "initiate_backup_cooling",
      "description": "백업 액체 질소 공급 시스템 활성화",
      "requiredResponseTime": 300,
      "assignedRoles": ["cryonics_technician"]
    },
    {
      "step": 3,
      "action": "assess_patient_risk",
      "description": "영향받은 Dewar의 모든 환자에 대해 이송 필요성 평가",
      "requiredResponseTime": 600,
      "assignedRoles": ["senior_technician", "medical_director"]
    },
    {
      "step": 4,
      "action": "document_incident",
      "description": "사고 보고서 작성 및 규제 당국 통지",
      "requiredResponseTime": 3600,
      "assignedRoles": ["quality_assurance_officer"]
    }
  ],
  "contactList": [
    {
      "role": "Emergency Coordinator",
      "name": "Dr. Sarah Mitchell",
      "phone": "+1-480-555-0911",
      "email": "emergency@phoenix-cryo.org"
    },
    {
      "role": "Nitrogen Supplier - Emergency",
      "name": "AirLiquide Emergency Services",
      "phone": "+1-800-555-7890",
      "email": "emergency@airliquide.com"
    }
  ],
  "lastDrillDate": "2025-09-15T10:00:00Z",
  "nextScheduledDrill": "2026-03-15T10:00:00Z"
}
```

### 4.6 용량 계획 예제

```json
{
  "planningId": "CAP-PLAN-2025-Q4",
  "facilityId": "CRYO-FAC-A7B3C9D2",
  "planningPeriod": {
    "startDate": "2025-10-01",
    "endDate": "2025-12-31"
  },
  "currentCapacity": {
    "totalSlots": 200,
    "occupiedSlots": 145,
    "availableSlots": 55,
    "reservedSlots": 15
  },
  "projections": {
    "expectedNewCases": 12,
    "expectedTransfersIn": 3,
    "expectedTransfersOut": 2,
    "projectedOccupancyEndOfPeriod": 158
  },
  "capacityThresholds": {
    "optimalOccupancy": 75,
    "warningThreshold": 85,
    "criticalThreshold": 95
  },
  "expansionPlanning": {
    "expansionRequired": false,
    "timeToCapacity": "현재 수용률로 약 18개월",
    "recommendedActions": [
      "분기별 수용률 모니터링",
      "시설 B동 확장 계획 준비",
      "추가 Dewar 구매 평가"
    ]
  }
}
```

---

## 5. 유효성 검사 규칙

### 5.1 데이터 무결성 요구사항

1. **고유 식별자**: 모든 기본 ID는 시스템 전체에서 고유해야 합니다
2. **참조 무결성**: 관련 레코드를 생성하기 전에 Facility ID가 존재해야 합니다
3. **시간적 일관성**: 종료 날짜는 시작 날짜 이후여야 합니다
4. **범위 검증**: 온도, 압력 및 레벨 판독값은 물리적으로 가능한 범위 내에 있어야 합니다
5. **상태 전환**: 상태 변경은 허용된 상태 머신 전환을 따라야 합니다

### 5.2 필수 필드

스키마에서 필수로 표시된 모든 필드는 제공되어야 합니다. 필수 필드에 대해 null 또는 빈 값은 허용되지 않습니다.

### 5.3 날짜/시간 형식

모든 타임스탬프는 UTC 시간대(YYYY-MM-DDTHH:mm:ssZ)와 함께 ISO 8601 형식을 사용해야 합니다.

### 5.4 보안 요구사항

- 직원 자격 증명은 평문으로 전송되어서는 안 됩니다
- 모든 데이터 수정에 대한 액세스 로그를 유지해야 합니다
- 개인 식별 정보(PII)는 저장 시 암호화되어야 합니다

---

## 6. 버전 이력

| 버전 | 날짜 | 작성자 | 변경사항 |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-18 | WIA 표준위원회 | 초기 드래프트 명세서 |

---

## 7. 참조

- ISO 9001:2015 - Quality Management Systems
- OSHA 29 CFR 1910.111 - Storage and Handling of Anhydrous Ammonia (극저온 액체에 대한 유사 기준)
- NFPA 55 - Compressed Gases and Cryogenic Fluids Code
- FDA 21 CFR Part 1271 - Human Cells, Tissues, and Cellular and Tissue-Based Products

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
