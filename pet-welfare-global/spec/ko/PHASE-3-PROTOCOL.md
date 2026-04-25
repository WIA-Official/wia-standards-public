# WIA Pet Welfare Global 프로토콜 표준
## Phase 3 사양서

---

**버전**: 1.0.0
**상태**: Draft
**날짜**: 2025-12-18
**작성자**: WIA Standards Committee
**라이선스**: MIT
**Primary Color**: #F59E0B (Amber)

---

## 목차

1. [개요](#개요)
2. [통신 프로토콜](#통신-프로토콜)
3. [국경 간 조정](#국경-간-조정)
4. [운송 프로토콜](#운송-프로토콜)
5. [긴급 대응](#긴급-대응)
6. [데이터 동기화](#데이터-동기화)
7. [보안 프로토콜](#보안-프로토콜)
8. [코드 예제](#코드-예제)
9. [프로토콜 워크플로우](#프로토콜-워크플로우)

---

## 개요

### 1.1 목적

WIA Pet Welfare Global 프로토콜 표준은 국제 동물 복지 운영을 위한 통신 프로토콜, 데이터 교환 메커니즘, 국경 간 조정 절차, 긴급 대응 워크플로우 및 보안 조치를 정의합니다.

**핵심 목표**:
- 조직과 당국 간 통신 표준화
- 원활한 국경 간 동물 이동 및 입양 지원
- 동물 복지 위기 시 긴급 대응 프로토콜 수립
- 데이터 동기화 및 일관성 메커니즘 정의
- 안전하고 규정을 준수하는 데이터 교환 보장
- 국경을 넘는 실시간 협업 지원

### 1.2 프로토콜 스택

| 계층 | 프로토콜 | 목적 |
|------|---------|------|
| Application | WIA-PET-PROTOCOL | 동물 복지 비즈니스 로직 |
| Transport | HTTPS/WSS | 안전한 데이터 전송 |
| Security | TLS 1.3 | 암호화 및 인증 |
| Data | JSON/Protocol Buffers | 구조화된 데이터 교환 |
| Identity | OAuth 2.0 / X.509 | 인증 및 권한 부여 |

### 1.3 프로토콜 범주

| 범주 | 설명 |
|------|------|
| **Coordination Protocols** | 조직 간 통신 및 협업 |
| **Transport Protocols** | 국제 동물 이동 및 물류 |
| **Emergency Protocols** | 위기 대응 및 신속한 개입 |
| **Sync Protocols** | 데이터 일관성 및 복제 |
| **Compliance Protocols** | 규제 및 법적 준수 검증 |

---

## 통신 프로토콜

### 2.1 조직 간 프로토콜

#### 2.1.1 핸드셰이크 시퀀스

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "HANDSHAKE_REQUEST",
  "from": {
    "organization_id": "WIA-ORG-US-001234",
    "name": "Happy Paws Animal Shelter",
    "country_code": "US",
    "capabilities": [
      "domestic_adoption",
      "international_adoption",
      "transport_coordination",
      "abuse_reporting"
    ],
    "api_endpoint": "https://api.happypaws.org/wia/v1",
    "public_key": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8A...\n-----END PUBLIC KEY-----"
  },
  "to": {
    "organization_id": "WIA-ORG-GB-005678",
    "country_code": "GB"
  },
  "timestamp": "2025-12-18T10:00:00Z",
  "nonce": "a1b2c3d4e5f6g7h8",
  "signature": "base64_encoded_signature"
}
```

**응답**:
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "HANDSHAKE_RESPONSE",
  "status": "accepted",
  "from": {
    "organization_id": "WIA-ORG-GB-005678",
    "name": "London Pet Rescue",
    "country_code": "GB",
    "capabilities": [
      "domestic_adoption",
      "international_adoption",
      "foster_network"
    ],
    "api_endpoint": "https://api.londonpetrescue.org.uk/wia/v1",
    "public_key": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8B...\n-----END PUBLIC KEY-----"
  },
  "session_id": "sess_xyz789",
  "timestamp": "2025-12-18T10:00:05Z",
  "signature": "base64_encoded_signature"
}
```

#### 2.1.2 메시지 타입

| 타입 | 목적 | 빈도 |
|------|------|------|
| `HANDSHAKE_REQUEST` | 연결 시작 | 세션당 1회 |
| `HANDSHAKE_RESPONSE` | 연결 확인 | 핸드셰이크 응답 |
| `DATA_REQUEST` | 동물 데이터 요청 | 필요시 |
| `DATA_RESPONSE` | 동물 데이터 제공 | 요청에 대한 응답 |
| `STATUS_UPDATE` | 동물 상태 업데이트 | 실시간 |
| `ADOPTION_INQUIRY` | 입양 문의 | 필요시 |
| `TRANSPORT_COORDINATION` | 운송 조정 | 운송당 |
| `HEARTBEAT` | 연결 유지 | 60초마다 |
| `DISCONNECT` | 세션 종료 | 세션 종료 시 |

#### 2.1.3 데이터 요청 프로토콜

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DATA_REQUEST",
  "session_id": "sess_xyz789",
  "request_id": "req_abc123",
  "from": "WIA-ORG-GB-005678",
  "to": "WIA-ORG-US-001234",
  "request": {
    "resource_type": "animal_profile",
    "animal_id": "WIA-PET-2025-000001",
    "fields": [
      "basic_info",
      "health_passport",
      "welfare_scores",
      "behavior_profile"
    ],
    "include_photos": true,
    "purpose": "international_adoption_inquiry"
  },
  "timestamp": "2025-12-18T10:05:00Z",
  "signature": "base64_encoded_signature"
}
```

**응답**:
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DATA_RESPONSE",
  "session_id": "sess_xyz789",
  "request_id": "req_abc123",
  "from": "WIA-ORG-US-001234",
  "to": "WIA-ORG-GB-005678",
  "status": "success",
  "data": {
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "basic_info": {},
      "health_passport": {},
      "welfare_scores": {},
      "behavior_profile": {}
    },
    "photos": [
      {
        "url": "https://storage.wia.org/photos/...",
        "type": "profile"
      }
    ]
  },
  "timestamp": "2025-12-18T10:05:02Z",
  "signature": "base64_encoded_signature"
}
```

### 2.2 정부 기관 프로토콜

#### 2.2.1 권한 검증

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "AUTHORITY_VERIFICATION",
  "agency": {
    "agency_id": "USDA-APHIS",
    "name": "United States Department of Agriculture - Animal and Plant Health Inspection Service",
    "country_code": "US",
    "authority_level": "federal",
    "jurisdiction": ["animal_import_export", "health_certification"],
    "contact": {
      "email": "aphis@usda.gov",
      "phone": "+1-301-851-3300"
    },
    "certification": {
      "certificate_id": "USDA-CERT-2025-001",
      "issued_by": "WIA Certification Authority",
      "valid_until": "2026-12-31",
      "public_key_fingerprint": "SHA256:abc123..."
    }
  },
  "timestamp": "2025-12-18T10:00:00Z",
  "signature": "base64_encoded_signature"
}
```

#### 2.2.2 규정 준수 확인 요청

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "COMPLIANCE_CHECK_REQUEST",
  "from": "USDA-APHIS",
  "request_id": "comp_check_001",
  "check_type": "export_authorization",
  "animal_id": "WIA-PET-2025-000001",
  "destination_country": "GB",
  "requirements": [
    "rabies_vaccination_current",
    "health_certificate_valid",
    "microchip_iso_compliant",
    "export_permit_issued"
  ],
  "timestamp": "2025-12-18T10:10:00Z"
}
```

**응답**:
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "COMPLIANCE_CHECK_RESPONSE",
  "request_id": "comp_check_001",
  "status": "compliant",
  "checks": [
    {
      "requirement": "rabies_vaccination_current",
      "status": "pass",
      "details": "Rabies vaccination valid until 2028-03-20"
    },
    {
      "requirement": "health_certificate_valid",
      "status": "pass",
      "details": "Health certificate HC-US-2025-001 valid"
    },
    {
      "requirement": "microchip_iso_compliant",
      "status": "pass",
      "details": "ISO 11784/11785 compliant microchip 900123456789012"
    },
    {
      "requirement": "export_permit_issued",
      "status": "pass",
      "details": "Export permit USDA-EXPORT-2025-001 issued"
    }
  ],
  "authorization": {
    "authorized": true,
    "authorization_number": "USDA-AUTH-2025-001",
    "valid_until": "2026-01-18"
  },
  "timestamp": "2025-12-18T10:10:05Z",
  "signature": "base64_encoded_signature"
}
```

### 2.3 NGO 조정 프로토콜

#### 2.3.1 네트워크 브로드캐스트

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "NETWORK_BROADCAST",
  "from": {
    "organization_id": "WIA-ORG-US-001234",
    "organization_type": "shelter"
  },
  "broadcast_type": "urgent_placement_needed",
  "priority": "high",
  "message": {
    "subject": "긴급: 대형견 배치 필요",
    "animal_id": "WIA-PET-2025-000099",
    "reason": "Current facility at capacity",
    "deadline": "2025-12-20T17:00:00Z",
    "animal_summary": {
      "species": "canis_lupus_familiaris",
      "breed": "German Shepherd",
      "age_years": 3,
      "special_needs": false,
      "temperament": "friendly"
    }
  },
  "recipients": {
    "filter": {
      "country_codes": ["US"],
      "regions": ["California", "Nevada", "Oregon"],
      "capabilities": ["large_dog_housing"]
    }
  },
  "timestamp": "2025-12-18T14:00:00Z"
}
```

#### 2.3.2 브로드캐스트 응답

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "BROADCAST_RESPONSE",
  "broadcast_id": "broadcast_xyz789",
  "from": {
    "organization_id": "WIA-ORG-US-005678",
    "name": "Oakland Animal Services"
  },
  "response": "can_accept",
  "details": {
    "available_space": true,
    "estimated_pickup_time": "2025-12-19T10:00:00Z",
    "transport_available": true,
    "additional_notes": "We have experience with German Shepherds and can provide behavioral support"
  },
  "contact": {
    "name": "Michael Rodriguez",
    "phone": "+1-510-555-0199",
    "email": "michael.rodriguez@oaklandanimals.org"
  },
  "timestamp": "2025-12-18T14:15:00Z"
}
```

---

## 국경 간 조정

### 3.1 국제 입양 프로토콜

#### 3.1.1 입양 문의 워크플로우

**단계 1: 문의 시작**
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "ADOPTION_INQUIRY",
  "inquiry_id": "inq_abc123",
  "from": {
    "organization_id": "WIA-ORG-GB-005678",
    "country_code": "GB"
  },
  "to": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US"
  },
  "animal_id": "WIA-PET-2025-000001",
  "adopter_info": {
    "name": "Emily Wilson",
    "country": "GB",
    "pre_screened": true,
    "home_check_completed": true,
    "reference_checks": "passed"
  },
  "timestamp": "2025-12-10T10:00:00Z"
}
```

**단계 2: 문의 응답**
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "ADOPTION_INQUIRY_RESPONSE",
  "inquiry_id": "inq_abc123",
  "status": "available",
  "animal_details": {
    "animal_id": "WIA-PET-2025-000001",
    "current_status": "available_for_international_adoption",
    "welfare_score": 87,
    "special_requirements": []
  },
  "adoption_terms": {
    "adoption_fee_usd": 250.00,
    "transport_responsibility": "adopter",
    "estimated_timeline_days": 30,
    "required_documents": [
      "home_inspection_report",
      "import_permit",
      "veterinary_agreement"
    ]
  },
  "timestamp": "2025-12-10T10:30:00Z"
}
```

**단계 3: 신청서 제출**
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "ADOPTION_APPLICATION",
  "application_id": "APP-2025-001",
  "inquiry_id": "inq_abc123",
  "animal_id": "WIA-PET-2025-000001",
  "applicant": {
    "name": "Emily Wilson",
    "address": {
      "street": "789 Pet Lane",
      "city": "London",
      "postal_code": "E1 7AA",
      "country_code": "GB"
    },
    "contact": {
      "email": "emily.wilson@email.com",
      "phone": "+44-20-7946-1234"
    }
  },
  "supporting_documents": [
    {
      "type": "home_inspection",
      "url": "https://secure.wia.org/docs/home_inspection_001.pdf",
      "verified_by": "WIA-ORG-GB-005678"
    },
    {
      "type": "import_permit",
      "url": "https://secure.wia.org/docs/import_permit_001.pdf",
      "permit_number": "UK-IMPORT-2025-001"
    }
  ],
  "timestamp": "2025-12-12T14:00:00Z"
}
```

**단계 4: 승인 및 조정**
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "ADOPTION_APPROVED",
  "application_id": "APP-2025-001",
  "animal_id": "WIA-PET-2025-000001",
  "approval_details": {
    "approved_by": "Jessica Martinez",
    "approved_date": "2025-12-15T10:00:00Z",
    "adoption_contract_id": "ADOPT-2025-001",
    "contract_url": "https://secure.wia.org/contracts/ADOPT-2025-001.pdf"
  },
  "next_steps": {
    "timeline": [
      {
        "step": "health_certificate_issuance",
        "responsible_party": "WIA-ORG-US-001234",
        "deadline": "2025-12-18T17:00:00Z",
        "status": "completed"
      },
      {
        "step": "transport_booking",
        "responsible_party": "adopter",
        "deadline": "2025-12-19T17:00:00Z",
        "status": "in_progress"
      },
      {
        "step": "animal_departure",
        "responsible_party": "WIA-ORG-US-001234",
        "scheduled_date": "2025-12-20T08:00:00Z",
        "status": "scheduled"
      },
      {
        "step": "animal_arrival",
        "responsible_party": "WIA-ORG-GB-005678",
        "scheduled_date": "2025-12-21T14:00:00Z",
        "status": "scheduled"
      }
    ]
  },
  "timestamp": "2025-12-15T10:30:00Z"
}
```

### 3.2 문서 검증 프로토콜

#### 3.2.1 건강 증명서 검증

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DOCUMENT_VERIFICATION_REQUEST",
  "verification_id": "verif_001",
  "document_type": "health_certificate",
  "document_id": "HC-US-2025-001",
  "animal_id": "WIA-PET-2025-000001",
  "issuer": {
    "organization_id": "WIA-ORG-US-001234",
    "veterinarian": "Dr. Sarah Johnson",
    "license_number": "CA-VET-12345"
  },
  "verifying_authority": "USDA-APHIS",
  "document_hash": "sha256:abc123def456...",
  "blockchain_record": {
    "chain": "ethereum",
    "transaction_hash": "0x123456...",
    "block_number": 12345678
  },
  "timestamp": "2025-12-18T10:00:00Z"
}
```

**응답**:
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DOCUMENT_VERIFICATION_RESPONSE",
  "verification_id": "verif_001",
  "status": "verified",
  "verification_details": {
    "document_authentic": true,
    "issuer_authorized": true,
    "not_tampered": true,
    "currently_valid": true,
    "expiry_date": "2026-01-18"
  },
  "digital_signature": {
    "algorithm": "RSA-SHA256",
    "valid": true,
    "signer": "Dr. Sarah Johnson, DVM",
    "certificate_chain_valid": true
  },
  "endorsements": [
    {
      "authority": "USDA-APHIS",
      "endorsement_number": "USDA-2025-001234",
      "endorsed_date": "2025-12-16T14:00:00Z"
    }
  ],
  "timestamp": "2025-12-18T10:00:05Z"
}
```

---

## 운송 프로토콜

### 4.1 IATA 살아있는 동물 규정 프로토콜

#### 4.1.1 운송 계획

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "TRANSPORT_PLAN",
  "transport_id": "WIA-TRN-2025-009012",
  "animal_id": "WIA-PET-2025-000001",
  "iata_compliance": {
    "lar_version": "LAR-49",
    "container_requirement": "CR82",
    "species_specific_requirements": [
      "Container must allow animal to stand, turn around, and lie down",
      "Adequate ventilation on all four sides",
      "Food and water containers required"
    ]
  },
  "route": [
    {
      "leg_number": 1,
      "departure": {
        "airport_code": "SFO",
        "datetime": "2025-12-20T10:00:00Z",
        "timezone": "America/Los_Angeles"
      },
      "arrival": {
        "airport_code": "JFK",
        "datetime": "2025-12-20T18:30:00Z",
        "timezone": "America/New_York"
      },
      "carrier": {
        "airline": "American Airlines",
        "flight_number": "AA100",
        "live_animal_acceptance": true
      },
      "conditions": {
        "estimated_duration_hours": 5.5,
        "temperature_controlled": true,
        "pressurized_cargo": true
      }
    }
  ],
  "container_specifications": {
    "container_id": "CRATE-2025-001",
    "type": "airline_approved_crate",
    "dimensions_cm": {
      "length": 91,
      "width": 61,
      "height": 66
    },
    "material": "rigid_plastic",
    "ventilation_area_percentage": 16,
    "door_type": "metal_grill",
    "securing_mechanism": "metal_bolts"
  },
  "timestamp": "2025-12-18T10:00:00Z"
}
```

#### 4.1.2 비행 전 체크리스트

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "PRE_FLIGHT_CHECKLIST",
  "transport_id": "WIA-TRN-2025-009012",
  "animal_id": "WIA-PET-2025-000001",
  "checklist": [
    {
      "item": "health_certificate_valid",
      "status": "checked",
      "verified_by": "Jessica Martinez",
      "timestamp": "2025-12-20T07:00:00Z"
    },
    {
      "item": "vaccination_current",
      "status": "checked",
      "verified_by": "Dr. Sarah Johnson",
      "timestamp": "2025-12-20T07:05:00Z"
    },
    {
      "item": "container_iata_compliant",
      "status": "checked",
      "verified_by": "Airline Cargo Agent",
      "timestamp": "2025-12-20T07:10:00Z"
    },
    {
      "item": "animal_fed_watered",
      "status": "checked",
      "details": "Fed 4 hours before flight, water available",
      "verified_by": "Jessica Martinez",
      "timestamp": "2025-12-20T06:00:00Z"
    },
    {
      "item": "import_documents_attached",
      "status": "checked",
      "verified_by": "Jessica Martinez",
      "timestamp": "2025-12-20T07:15:00Z"
    },
    {
      "item": "emergency_contact_info",
      "status": "checked",
      "contacts": [
        {
          "role": "origin_contact",
          "name": "Jessica Martinez",
          "phone": "+1-415-555-0123"
        },
        {
          "role": "destination_contact",
          "name": "James Thompson",
          "phone": "+44-20-7946-0958"
        }
      ],
      "timestamp": "2025-12-20T07:20:00Z"
    }
  ],
  "overall_status": "ready_for_transport",
  "timestamp": "2025-12-20T07:30:00Z"
}
```

#### 4.1.3 운송 중 업데이트

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "TRANSPORT_STATUS_UPDATE",
  "transport_id": "WIA-TRN-2025-009012",
  "animal_id": "WIA-PET-2025-000001",
  "current_status": "in_transit",
  "location": {
    "type": "in_flight",
    "flight_number": "AA100",
    "current_position": {
      "latitude": 39.8283,
      "longitude": -98.5795
    },
    "estimated_arrival": "2025-12-20T18:30:00Z"
  },
  "welfare_check": {
    "last_observation": "2025-12-20T10:00:00Z",
    "observer": "Flight Cargo Handler",
    "status": "animal_calm_and_comfortable",
    "notes": "No signs of distress"
  },
  "next_update_expected": "2025-12-20T18:30:00Z",
  "timestamp": "2025-12-20T14:00:00Z"
}
```

### 4.2 검역 프로토콜

#### 4.2.1 검역 입국

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "QUARANTINE_ENTRY",
  "quarantine_id": "QUAR-GB-2025-001",
  "animal_id": "WIA-PET-2025-000001",
  "facility": {
    "facility_id": "QUAR-FAC-GB-001",
    "name": "Heathrow Animal Reception Centre",
    "location": "London, UK",
    "license_number": "UK-QUAR-001",
    "contact": {
      "phone": "+44-20-8759-7777",
      "email": "harc@heathrow.com"
    }
  },
  "entry_details": {
    "entry_date": "2025-12-21T14:00:00Z",
    "quarantine_duration_days": 0,
    "reason": "standard_import_protocol",
    "requirements": [
      "health_inspection",
      "document_verification"
    ]
  },
  "health_status_on_arrival": {
    "examined_by": "Dr. James Wilson, MRCVS",
    "examination_date": "2025-12-21T14:30:00Z",
    "general_condition": "good",
    "vital_signs": {
      "temperature_celsius": 38.5,
      "heart_rate_bpm": 90,
      "respiratory_rate": 24
    },
    "findings": "Animal in good health, no signs of illness or distress from transport"
  },
  "timestamp": "2025-12-21T15:00:00Z"
}
```

#### 4.2.2 검역 해제

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "QUARANTINE_RELEASE",
  "quarantine_id": "QUAR-GB-2025-001",
  "animal_id": "WIA-PET-2025-000001",
  "release_details": {
    "release_date": "2025-12-21T16:00:00Z",
    "release_authorized_by": "Dr. James Wilson, MRCVS",
    "authorization_number": "UK-REL-2025-001",
    "conditions_met": [
      {
        "condition": "health_inspection_passed",
        "status": "met"
      },
      {
        "condition": "documents_verified",
        "status": "met"
      },
      {
        "condition": "no_disease_indicators",
        "status": "met"
      }
    ]
  },
  "release_to": {
    "organization_id": "WIA-ORG-GB-005678",
    "contact_person": "James Thompson",
    "contact_phone": "+44-20-7946-0958"
  },
  "post_release_requirements": [
    "30-day monitoring period",
    "Report any health concerns to local veterinary authority"
  ],
  "timestamp": "2025-12-21T16:00:00Z"
}
```

---

## 긴급 대응

### 5.1 동물 복지 위기 프로토콜

#### 5.1.1 위기 선언

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "CRISIS_DECLARATION",
  "crisis_id": "CRISIS-2025-001",
  "crisis_type": "mass_animal_seizure",
  "severity_level": "critical",
  "declared_by": {
    "organization_id": "WIA-ORG-US-005678",
    "authority": "animal_control",
    "contact": {
      "name": "Chief Officer Maria Garcia",
      "phone": "+1-415-555-9999"
    }
  },
  "situation": {
    "description": "Large-scale hoarding case discovered, estimated 200+ animals in urgent need of rescue",
    "location": {
      "address": "Rural property, Sonoma County, CA",
      "gps_coordinates": {
        "latitude": 38.2919,
        "longitude": -122.4580
      }
    },
    "estimated_animals": 200,
    "species_affected": ["dogs", "cats", "rabbits", "chickens"],
    "conditions": "severe_neglect",
    "immediate_needs": [
      "emergency_veterinary_care",
      "temporary_housing",
      "food_and_water",
      "transport"
    ]
  },
  "assistance_requested": {
    "shelter_space_needed": 200,
    "veterinary_staff_needed": 10,
    "volunteers_needed": 50,
    "supplies_needed": [
      "food",
      "medications",
      "crates",
      "cleaning_supplies"
    ],
    "transport_needed": "Multiple vehicles"
  },
  "response_deadline": "2025-12-18T18:00:00Z",
  "timestamp": "2025-12-18T10:00:00Z"
}
```

#### 5.1.2 위기 대응 조정

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "CRISIS_RESPONSE",
  "crisis_id": "CRISIS-2025-001",
  "responding_organizations": [
    {
      "organization_id": "WIA-ORG-US-001234",
      "name": "Happy Paws Animal Shelter",
      "commitment": {
        "shelter_space": 30,
        "veterinary_staff": 2,
        "volunteers": 10,
        "transport_vehicles": 2,
        "supplies": ["dog_food_200kg", "cat_food_100kg", "medical_supplies"]
      },
      "arrival_time": "2025-12-18T12:00:00Z",
      "contact": {
        "name": "Jessica Martinez",
        "phone": "+1-415-555-0123"
      }
    },
    {
      "organization_id": "WIA-ORG-US-002468",
      "name": "Bay Area Pet Rescue",
      "commitment": {
        "shelter_space": 50,
        "volunteers": 20,
        "transport_vehicles": 3
      },
      "arrival_time": "2025-12-18T13:00:00Z",
      "contact": {
        "name": "Robert Chen",
        "phone": "+1-510-555-0456"
      }
    }
  ],
  "coordination_center": {
    "location": "Happy Paws Animal Shelter",
    "coordinator": "Jessica Martinez",
    "communication_channel": "crisis-2025-001-slack"
  },
  "timestamp": "2025-12-18T10:30:00Z"
}
```

#### 5.1.3 위기 상태 업데이트

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "CRISIS_STATUS_UPDATE",
  "crisis_id": "CRISIS-2025-001",
  "update_number": 3,
  "status": "in_progress",
  "progress": {
    "animals_rescued": 150,
    "animals_remaining": 50,
    "animals_in_care": 150,
    "animals_receiving_vet_care": 45,
    "animals_critical_condition": 12,
    "animals_deceased": 3
  },
  "current_situation": {
    "rescue_operation_status": "ongoing",
    "estimated_completion": "2025-12-18T16:00:00Z",
    "challenges": [
      "Some animals difficult to catch",
      "Additional medical supplies needed"
    ],
    "additional_needs": [
      "Emergency veterinarian",
      "Specialized cat traps"
    ]
  },
  "next_update_expected": "2025-12-18T15:00:00Z",
  "timestamp": "2025-12-18T14:00:00Z"
}
```

### 5.2 질병 발생 프로토콜

#### 5.2.1 발생 경고

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DISEASE_OUTBREAK_ALERT",
  "outbreak_id": "OUTBREAK-2025-001",
  "disease": {
    "name": "Canine Parvovirus",
    "pathogen": "Canine parvovirus type 2",
    "severity": "high",
    "transmission": "highly_contagious",
    "affected_species": ["canis_lupus_familiaris"]
  },
  "outbreak_details": {
    "organization_id": "WIA-ORG-US-001234",
    "first_case_date": "2025-12-10T10:00:00Z",
    "confirmed_cases": 5,
    "suspected_cases": 8,
    "deaths": 1,
    "animals_at_risk": 68
  },
  "containment_measures": {
    "quarantine_imposed": true,
    "new_intakes_suspended": true,
    "affected_areas_isolated": true,
    "disinfection_protocol": "bleach_solution_daily",
    "testing_protocol": "all_dogs_tested"
  },
  "notification_scope": {
    "local_shelters": true,
    "veterinary_clinics": true,
    "animal_control": true,
    "recent_adopters": true,
    "health_department": true
  },
  "contact_for_info": {
    "name": "Dr. Sarah Johnson",
    "phone": "+1-415-555-0123",
    "email": "sarah.johnson@happypaws.org"
  },
  "timestamp": "2025-12-15T14:00:00Z"
}
```

---

## 데이터 동기화

### 6.1 실시간 동기화 프로토콜

#### 6.1.1 변경 알림

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "DATA_CHANGE_NOTIFICATION",
  "change_id": "change_abc123",
  "entity_type": "animal_profile",
  "entity_id": "WIA-PET-2025-000001",
  "change_type": "update",
  "changed_fields": [
    "current_status.status",
    "current_status.adoption_date",
    "welfare_scores.overall_score"
  ],
  "changes": {
    "current_status.status": {
      "old_value": "shelter_care",
      "new_value": "adopted"
    },
    "current_status.adoption_date": {
      "old_value": null,
      "new_value": "2025-12-20T10:00:00Z"
    },
    "welfare_scores.overall_score": {
      "old_value": 87,
      "new_value": 92
    }
  },
  "changed_by": {
    "user_id": "user_jessica_martinez",
    "organization_id": "WIA-ORG-US-001234"
  },
  "timestamp": "2025-12-20T10:00:00Z",
  "version": 15
}
```

#### 6.1.2 동기화 요청

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "SYNC_REQUEST",
  "request_id": "sync_req_001",
  "from": "WIA-ORG-GB-005678",
  "sync_scope": {
    "entity_type": "animal_profile",
    "entity_ids": ["WIA-PET-2025-000001"],
    "last_sync_timestamp": "2025-12-18T00:00:00Z",
    "include_related": ["health_passport", "welfare_assessments"]
  },
  "timestamp": "2025-12-20T10:05:00Z"
}
```

**응답**:
```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "SYNC_RESPONSE",
  "request_id": "sync_req_001",
  "changes": [
    {
      "entity_type": "animal_profile",
      "entity_id": "WIA-PET-2025-000001",
      "version": 15,
      "timestamp": "2025-12-20T10:00:00Z",
      "data": {
        "current_status": {
          "status": "adopted",
          "adoption_date": "2025-12-20T10:00:00Z"
        }
      }
    }
  ],
  "sync_complete": true,
  "next_sync_token": "token_xyz789",
  "timestamp": "2025-12-20T10:05:02Z"
}
```

### 6.2 충돌 해결 프로토콜

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "CONFLICT_DETECTED",
  "conflict_id": "conflict_001",
  "entity_type": "animal_profile",
  "entity_id": "WIA-PET-2025-000001",
  "field": "current_status.status",
  "conflicts": [
    {
      "version": 14,
      "value": "foster_care",
      "changed_by": "WIA-ORG-US-001234",
      "timestamp": "2025-12-20T10:00:00Z"
    },
    {
      "version": 14,
      "value": "adopted",
      "changed_by": "WIA-ORG-GB-005678",
      "timestamp": "2025-12-20T10:00:05Z"
    }
  ],
  "resolution_strategy": "last_write_wins",
  "resolved_value": "adopted",
  "resolved_version": 15,
  "timestamp": "2025-12-20T10:00:10Z"
}
```

---

## 보안 프로토콜

### 7.1 암호화 표준

| 계층 | 프로토콜 | 키 크기 |
|------|---------|---------|
| Transport | TLS 1.3 | 256-bit |
| Data at Rest | AES-GCM | 256-bit |
| Message | End-to-End RSA | 4096-bit |
| Signatures | ECDSA | P-384 |

### 7.2 인증 프로토콜

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "AUTH_REQUEST",
  "client_id": "WIA-ORG-US-001234",
  "auth_method": "mutual_tls",
  "client_certificate": {
    "subject": "CN=Happy Paws Animal Shelter, O=WIA, C=US",
    "issuer": "CN=WIA Certificate Authority",
    "serial_number": "1234567890",
    "not_before": "2025-01-01T00:00:00Z",
    "not_after": "2026-01-01T00:00:00Z",
    "fingerprint": "SHA256:abc123def456..."
  },
  "requested_scopes": [
    "animal:read",
    "animal:write",
    "transport:manage"
  ],
  "timestamp": "2025-12-18T10:00:00Z",
  "nonce": "random_nonce_12345"
}
```

### 7.3 데이터 프라이버시 프로토콜

```json
{
  "protocol_version": "WIA-PET-1.0",
  "message_type": "PRIVACY_POLICY",
  "data_classification": {
    "animal_data": "public_with_restrictions",
    "adopter_data": "private",
    "abuse_reporter_data": "confidential",
    "investigation_data": "confidential",
    "organization_data": "public"
  },
  "sharing_rules": {
    "animal_profiles": {
      "shareable_fields": [
        "basic_info",
        "welfare_scores",
        "behavior_profile"
      ],
      "restricted_fields": [
        "previous_owner",
        "detailed_medical_history"
      ],
      "sharing_requires": "organization_agreement"
    },
    "adopter_information": {
      "shareable_fields": ["name", "country"],
      "restricted_fields": ["address", "phone", "email"],
      "sharing_requires": "explicit_consent"
    }
  },
  "retention_policy": {
    "animal_records": "10_years_post_adoption",
    "adoption_records": "permanent",
    "abuse_reports": "permanent",
    "transport_manifests": "7_years"
  }
}
```

---

## 코드 예제

### 예제 1: 조직 핸드셰이크 (Python)

이 예제는 두 조직 간의 안전한 핸드셰이크 프로세스를 보여줍니다.

```python
import requests
import json
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding
import secrets

class WIAProtocolClient:
    def __init__(self, org_id, private_key_path, api_endpoint):
        self.org_id = org_id
        self.api_endpoint = api_endpoint
        with open(private_key_path, 'rb') as f:
            self.private_key = serialization.load_pem_private_key(
                f.read(), password=None
            )

    def create_handshake_request(self, target_org_id, target_country):
        nonce = secrets.token_hex(16)
        message = {
            "protocol_version": "WIA-PET-1.0",
            "message_type": "HANDSHAKE_REQUEST",
            "from": {
                "organization_id": self.org_id,
                "capabilities": [
                    "domestic_adoption",
                    "international_adoption"
                ],
                "api_endpoint": self.api_endpoint
            },
            "to": {
                "organization_id": target_org_id,
                "country_code": target_country
            },
            "timestamp": "2025-12-18T10:00:00Z",
            "nonce": nonce
        }

        # 메시지 서명
        message_bytes = json.dumps(message, sort_keys=True).encode()
        signature = self.private_key.sign(
            message_bytes,
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.MAX_LENGTH
            ),
            hashes.SHA256()
        )

        message["signature"] = signature.hex()
        return message

    def send_handshake(self, target_org_id, target_country):
        message = self.create_handshake_request(target_org_id, target_country)
        response = requests.post(
            f"{self.api_endpoint}/protocol/handshake",
            json=message,
            headers={"Content-Type": "application/json"}
        )
        return response.json()

# 사용 예시
client = WIAProtocolClient(
    org_id="WIA-ORG-US-001234",
    private_key_path="/path/to/private_key.pem",
    api_endpoint="https://api.happypaws.org/wia/v1"
)

response = client.send_handshake("WIA-ORG-GB-005678", "GB")
print(f"핸드셰이크 상태: {response['status']}")
print(f"세션 ID: {response.get('session_id')}")
```

### 예제 2: 실시간 운송 추적 (JavaScript)

```javascript
const WebSocket = require('ws');

class TransportTracker {
  constructor(apiKey, transportId) {
    this.apiKey = apiKey;
    this.transportId = transportId;
    this.ws = null;
  }

  connect() {
    const wsUrl = `wss://api.wia.org/pet-welfare-global/v1/transport/${this.transportId}/track`;

    this.ws = new WebSocket(wsUrl, {
      headers: {
        'Authorization': `Bearer ${this.apiKey}`
      }
    });

    this.ws.on('open', () => {
      console.log('운송 추적에 연결되었습니다');

      // 업데이트 구독
      this.ws.send(JSON.stringify({
        message_type: 'SUBSCRIBE',
        transport_id: this.transportId,
        update_frequency: 'real_time'
      }));
    });

    this.ws.on('message', (data) => {
      const message = JSON.parse(data);

      if (message.message_type === 'TRANSPORT_STATUS_UPDATE') {
        this.handleStatusUpdate(message);
      } else if (message.message_type === 'WELFARE_CHECK') {
        this.handleWelfareCheck(message);
      }
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket 오류:', error);
    });

    this.ws.on('close', () => {
      console.log('운송 추적 연결이 끊어졌습니다');
      // 5초 후 재연결 시도
      setTimeout(() => this.connect(), 5000);
    });
  }

  handleStatusUpdate(message) {
    console.log(`운송 상태: ${message.current_status}`);
    console.log(`위치: ${message.location.type}`);
    console.log(`예상 도착: ${message.location.estimated_arrival}`);

    // 이해관계자에게 알림
    this.notifyStakeholders(message);
  }

  handleWelfareCheck(message) {
    console.log(`복지 확인: ${message.welfare_check.status}`);
    console.log(`관찰자: ${message.welfare_check.observer}`);
    console.log(`메모: ${message.welfare_check.notes}`);
  }

  notifyStakeholders(message) {
    // 출발지 및 목적지 조직에 알림 전송
    // 알림 시스템에 따라 구현
  }

  disconnect() {
    if (this.ws) {
      this.ws.close();
    }
  }
}

// 사용 예시
const tracker = new TransportTracker(
  'your_api_key',
  'WIA-TRN-2025-009012'
);
tracker.connect();
```

---

## 프로토콜 워크플로우

### 9.1 완전한 국제 입양 워크플로우

```
┌─────────────────────────────────────────────────────────────────────┐
│                  국제 입양 프로토콜 워크플로우                         │
└─────────────────────────────────────────────────────────────────────┘

출발지 조직 (US)       목적지 조직 (GB)         당국
     │                            │                         │
     │  1. HANDSHAKE_REQUEST     │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  2. HANDSHAKE_RESPONSE    │                         │
     │<───────────────────────────│                         │
     │                            │                         │
     │  3. ADOPTION_INQUIRY       │                         │
     │<───────────────────────────│                         │
     │                            │                         │
     │  4. INQUIRY_RESPONSE       │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  5. ADOPTION_APPLICATION   │                         │
     │<───────────────────────────│                         │
     │                            │                         │
     │  6. APPLICATION_REVIEW     │                         │
     │         (내부)             │                         │
     │                            │                         │
     │  7. ADOPTION_APPROVED      │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  8. HEALTH_CERTIFICATE     │                         │
     │────────────────────────────┼────────────────────────>│
     │                            │                         │
     │  9. COMPLIANCE_CHECK       │                         │
     │<───────────────────────────┼─────────────────────────│
     │                            │                         │
     │  10. TRANSPORT_PLAN        │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  11. PRE_FLIGHT_CHECKLIST  │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  12. TRANSPORT_DEPARTED    │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  13. IN_TRANSIT_UPDATES    │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │  14. TRANSPORT_ARRIVED     │                         │
     │───────────────────────────>│                         │
     │                            │                         │
     │                            │  15. QUARANTINE_ENTRY   │
     │                            │────────────────────────>│
     │                            │                         │
     │                            │  16. QUARANTINE_RELEASE │
     │                            │<────────────────────────│
     │                            │                         │
     │  17. ADOPTION_COMPLETE     │                         │
     │<───────────────────────────│                         │
     │                            │                         │
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
