# WIA Pet Welfare Global 데이터 형식 표준
## Phase 1 사양서

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
2. [용어정의](#용어정의)
3. [기본 구조](#기본-구조)
4. [데이터 스키마](#데이터-스키마)
5. [필드 사양](#필드-사양)
6. [복지 메트릭](#복지-메트릭)
7. [유효성 검증 규칙](#유효성-검증-규칙)
8. [예제](#예제)
9. [버전 히스토리](#버전-히스토리)

---

## 개요

### 1.1 목적

WIA Pet Welfare Global 데이터 형식 표준은 국제 반려동물 복지 데이터, 동물 건강 기록, 국경 간 입양 추적, 보호소 운영, 동물 보호 집행을 기록, 전송 및 관리하기 위한 통합 JSON 기반 형식을 정의합니다.

**핵심 목표**:
- 모든 국가와 조직에서 동물 복지 메트릭 표준화
- 보호소, 구조 조직 및 정부 기관 간 상호 운용성 활성화
- 출생부터 입양까지 동물 생애 주기 전반에 걸친 데이터 무결성 보장
- 증거 기반 동물 복지 정책 개발 지원
- 국경 간 입양 및 동물 운송 규정 준수 촉진
- 통합 신고 시스템을 통한 동물 학대 추적 및 예방
- 국제 야생동물 및 이국적 반려동물 규정 조정

### 1.2 범위

이 표준은 다음 데이터 도메인을 다룹니다:

| 도메인 | 설명 |
|--------|------|
| Animal Identity | 마이크로칩, 생체인식 및 등록 데이터 |
| Welfare Metrics | 건강, 행동 및 환경 복지 점수 |
| Shelter Operations | 입소, 수용, 의료 케어 및 입양 기록 |
| Cross-Border Movement | 국제 입양, 운송 및 검역 |
| Health Passport | 예방접종, 의료 이력 및 건강 증명서 |
| Abuse Reporting | 사건 추적, 조사 및 집행 |
| Wildlife Regulation | 희귀 종, CITES 준수 및 보존 |
| Organization Data | 보호소, 구조, NGO 및 정부 기관 프로필 |

### 1.3 설계 원칙

1. **동물 중심**: 모든 데이터 구조에서 동물 복지와 보호 우선시
2. **글로벌 상호운용성**: 국제 표준(ISO, IATA, CITES)과 호환
3. **프라이버시 및 보안**: 필요한 공유를 가능하게 하면서 민감한 데이터 보호
4. **실시간 추적**: 동물 복지 상태의 실시간 모니터링 지원
5. **증거 기반**: 데이터 기반 정책 및 복지 개선 가능
6. **다국어 지원**: 국제 언어 및 문화적 요구사항 지원

---

## 용어정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Animal Subject** | 시스템에서 추적되는 개별 동물 |
| **Welfare Score** | 동물 복지에 대한 정량적 평가 (0-100 척도) |
| **Shelter Organization** | 동물을 위한 임시 보호를 제공하는 시설 |
| **Cross-Border Adoption** | 동물 소유권의 국제 이전 |
| **Health Passport** | 완전한 의료 및 예방접종 기록 |
| **Abuse Incident** | 동물 학대에 대한 문서화된 사례 |
| **CITES Permit** | 멸종위기 야생동식물 국제거래 협약 승인 |
| **IATA Compliance** | 국제항공운송협회 동물 운송 표준 |
| **Quarantine Period** | 질병 예방을 위한 의무적 격리 기간 |
| **Microchip ID** | ISO 11784/11785 준수 식별 트랜스폰더 |

### 2.2 데이터 타입

| 타입 | 설명 | 예제 |
|------|------|------|
| `string` | UTF-8 인코딩 텍스트 | `"WIA-PET-2025-001"` |
| `number` | IEEE 754 배정밀도 | `85.5`, `42.3` |
| `integer` | 부호있는 64비트 정수 | `365`, `14` |
| `boolean` | 불린 값 | `true`, `false` |
| `timestamp` | ISO 8601 날짜시간 | `"2025-12-18T14:30:00Z"` |
| `uuid` | UUID v4 식별자 | `"550e8400-e29b-41d4-a716-446655440000"` |
| `iso3166` | ISO 3166-1 alpha-2 국가 코드 | `"US"`, `"KR"`, `"GB"` |
| `microchip` | ISO 11784/11785 15자리 코드 | `"900123456789012"` |

### 2.3 필드 요구사항

| 표시 | 의미 |
|------|------|
| **REQUIRED** | 필수 필드 |
| **OPTIONAL** | 선택적 필드 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 기본 구조

### 3.1 루트 문서 형식

모든 WIA Pet Welfare Global 문서는 다음 기본 구조를 따릅니다:

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "uuid",
  "document_type": "string",
  "created_at": "timestamp",
  "updated_at": "timestamp",
  "metadata": {
    "organization_id": "uuid",
    "country_code": "iso3166",
    "language": "string",
    "timezone": "string"
  },
  "data": {}
}
```

### 3.2 문서 유형

| 타입 | 목적 | 핵심 데이터 |
|------|------|----------|
| `animal_profile` | 개별 동물 기록 | 신원, 건강, 이력 |
| `shelter_intake` | 보호소 입소 | 입소 세부정보, 상태 |
| `adoption_record` | 입양 거래 | 입양자, 동물, 조건 |
| `health_passport` | 의료 이력 | 예방접종, 치료 |
| `transport_manifest` | 국경 간 이동 | 경로, 운송업체, 허가 |
| `abuse_report` | 사건 문서화 | 세부정보, 증거, 상태 |
| `welfare_assessment` | 주기적 평가 | 메트릭, 관찰 |
| `organization_profile` | 시설 정보 | 수용 능력, 서비스, 직원 |

### 3.3 식별자 표준

| 타입 | 형식 | 예제 |
|------|------|------|
| Animal ID | `WIA-PET-{YEAR}-{SEQUENCE}` | `WIA-PET-2025-000001` |
| Organization ID | `WIA-ORG-{COUNTRY}-{SEQUENCE}` | `WIA-ORG-US-001234` |
| Incident ID | `WIA-INC-{YEAR}-{SEQUENCE}` | `WIA-INC-2025-005678` |
| Transport ID | `WIA-TRN-{YEAR}-{SEQUENCE}` | `WIA-TRN-2025-009012` |

---

## 데이터 스키마

### 4.1 동물 프로필 스키마

완전한 동물 프로필에는 신원, 신체적 특성, 행동 특성 및 복지 이력이 포함됩니다.

```json
{
  "animal_profile": {
    "wia_id": "WIA-PET-2025-000001",
    "identifiers": {
      "microchip_id": "900123456789012",
      "passport_number": "PET-US-2025-001",
      "registration_number": "REG-CA-2025-1234",
      "tattoo_id": "ABC123",
      "dna_profile_id": "DNA-LAB-001-2025"
    },
    "basic_info": {
      "name": "Max",
      "species": "canis_lupus_familiaris",
      "breed": "Golden Retriever",
      "breed_mix": ["Golden Retriever", "Labrador Retriever"],
      "sex": "male",
      "neutered": true,
      "date_of_birth": "2023-06-15",
      "age_estimate": {
        "years": 2,
        "months": 6,
        "confidence": "high"
      },
      "color": "golden",
      "markings": "white chest patch",
      "size_category": "large"
    },
    "physical_characteristics": {
      "weight_kg": 32.5,
      "height_cm": 58,
      "body_condition_score": 5,
      "distinguishing_features": [
        "Scar on left hind leg",
        "Black spot on tongue"
      ]
    },
    "current_status": {
      "status": "shelter_care",
      "location": {
        "organization_id": "WIA-ORG-US-001234",
        "facility_name": "Happy Paws Animal Shelter",
        "country_code": "US",
        "region": "California",
        "city": "San Francisco",
        "address": "123 Animal Way, San Francisco, CA 94102",
        "gps_coordinates": {
          "latitude": 37.7749,
          "longitude": -122.4194
        }
      },
      "since": "2025-03-15T10:00:00Z",
      "available_for_adoption": true,
      "special_needs": false,
      "requires_experienced_owner": false
    },
    "origin": {
      "source_type": "stray_pickup",
      "source_location": "San Francisco, CA",
      "intake_date": "2025-03-15T10:00:00Z",
      "previous_owner": null,
      "circumstances": "Found wandering in Golden Gate Park",
      "original_country": "US"
    },
    "welfare_scores": {
      "overall_score": 87,
      "physical_health": 90,
      "mental_wellbeing": 85,
      "social_behavior": 88,
      "environmental_enrichment": 82,
      "last_assessed": "2025-12-18T14:00:00Z",
      "assessed_by": "Dr. Sarah Johnson, DVM",
      "assessment_protocol": "WIA-WELFARE-ASSESSMENT-v1.0"
    },
    "behavior_profile": {
      "temperament": "friendly_outgoing",
      "energy_level": "high",
      "socialization": {
        "good_with_dogs": true,
        "good_with_cats": false,
        "good_with_children": true,
        "age_range_children": "8+",
        "good_with_strangers": true
      },
      "training_level": "basic_commands",
      "known_commands": ["sit", "stay", "come", "down"],
      "behavioral_issues": [],
      "special_considerations": [
        "Needs daily exercise",
        "Prefers active families"
      ]
    }
  }
}
```

### 4.2 건강 여권 스키마

국제 운송 요구사항을 준수하는 포괄적인 의료 이력 및 예방접종 기록입니다.

```json
{
  "health_passport": {
    "passport_id": "PET-US-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "issued_date": "2025-03-20T10:00:00Z",
    "issuing_authority": {
      "organization": "California Department of Food and Agriculture",
      "country": "US",
      "veterinarian": {
        "name": "Dr. Sarah Johnson",
        "license_number": "CA-VET-12345",
        "contact": "sarah.johnson@happypaws.org"
      }
    },
    "vaccinations": [
      {
        "vaccine_id": "VAC-2025-001",
        "vaccine_name": "Rabies",
        "vaccine_type": "killed_virus",
        "manufacturer": "Merial",
        "batch_number": "RAB-2025-0312",
        "date_administered": "2025-03-20T11:00:00Z",
        "expiry_date": "2028-03-20",
        "next_due_date": "2028-03-20",
        "administered_by": "Dr. Sarah Johnson",
        "site": "right_shoulder",
        "route": "subcutaneous",
        "dose_ml": 1.0,
        "adverse_reactions": []
      },
      {
        "vaccine_id": "VAC-2025-002",
        "vaccine_name": "DHPP",
        "vaccine_type": "modified_live_virus",
        "components": [
          "Distemper",
          "Hepatitis",
          "Parvovirus",
          "Parainfluenza"
        ],
        "manufacturer": "Zoetis",
        "batch_number": "DHPP-2025-0315",
        "date_administered": "2025-03-20T11:15:00Z",
        "expiry_date": "2026-03-20",
        "next_due_date": "2026-03-20",
        "administered_by": "Dr. Sarah Johnson",
        "site": "left_shoulder",
        "route": "subcutaneous",
        "dose_ml": 1.0,
        "adverse_reactions": []
      }
    ],
    "medical_history": [
      {
        "record_id": "MED-2025-001",
        "date": "2025-03-15T10:30:00Z",
        "type": "examination",
        "veterinarian": "Dr. Sarah Johnson",
        "diagnosis": "Healthy adult dog, minor skin abrasion on left hind leg",
        "treatment": "Wound cleaning and topical antibiotic",
        "medications": [
          {
            "name": "Neosporin",
            "dosage": "topical application",
            "frequency": "twice daily",
            "duration": "7 days"
          }
        ],
        "follow_up_required": false,
        "notes": "Wound healing well, no complications"
      }
    ],
    "parasite_control": {
      "last_deworming": "2025-03-20T12:00:00Z",
      "product": "Panacur",
      "next_due": "2025-06-20",
      "flea_tick_prevention": {
        "product": "Frontline Plus",
        "last_applied": "2025-12-01T10:00:00Z",
        "next_due": "2026-01-01"
      },
      "heartworm": {
        "last_test_date": "2025-03-20",
        "test_result": "negative",
        "prevention_product": "Heartgard Plus",
        "last_dose": "2025-12-01",
        "next_due": "2026-01-01"
      }
    },
    "health_certificates": [
      {
        "certificate_id": "HC-US-2025-001",
        "type": "international_travel",
        "issuing_country": "US",
        "destination_country": "GB",
        "issue_date": "2025-12-15T10:00:00Z",
        "valid_until": "2026-01-15",
        "certified_by": "Dr. Sarah Johnson, DVM",
        "accreditation_number": "USDA-APHIS-12345",
        "health_status": "fit_to_travel",
        "special_notes": "Compliant with UK pet travel scheme requirements"
      }
    ],
    "genetic_testing": [
      {
        "test_id": "DNA-LAB-001-2025",
        "test_date": "2025-03-25T10:00:00Z",
        "laboratory": "Embark Veterinary",
        "test_type": "breed_identification",
        "results": {
          "primary_breed": "Golden Retriever",
          "percentage": 87.5,
          "secondary_breed": "Labrador Retriever",
          "percentage_secondary": 12.5
        }
      }
    ],
    "spay_neuter_record": {
      "procedure_date": "2024-01-15T09:00:00Z",
      "veterinarian": "Dr. Michael Chen",
      "facility": "Bay Area Veterinary Clinic",
      "method": "ovariohysterectomy",
      "complications": "none",
      "recovery_notes": "Excellent recovery, no complications"
    }
  }
}
```

### 4.3 보호소 입소 스키마

보호소에 입소하는 동물에 대한 표준화된 입소 문서입니다.

```json
{
  "shelter_intake": {
    "intake_id": "INTAKE-2025-12345",
    "animal_id": "WIA-PET-2025-000001",
    "organization_id": "WIA-ORG-US-001234",
    "intake_date": "2025-03-15T10:00:00Z",
    "intake_type": "stray",
    "intake_officer": {
      "name": "Jessica Martinez",
      "employee_id": "EMP-001",
      "contact": "jessica.martinez@happypaws.org"
    },
    "source_information": {
      "brought_by": "animal_control",
      "animal_control_agency": "San Francisco Animal Care & Control",
      "case_number": "SFACC-2025-0315-001",
      "pickup_location": {
        "address": "Golden Gate Park, San Francisco, CA",
        "gps_coordinates": {
          "latitude": 37.7694,
          "longitude": -122.4862
        }
      },
      "circumstances": "Found wandering alone, appeared friendly and well-cared for but no collar or ID",
      "finder_contact": null
    },
    "initial_assessment": {
      "physical_condition": "good",
      "body_condition_score": 5,
      "estimated_age": "2 years",
      "temperament": "friendly",
      "obvious_injuries": false,
      "urgent_medical_needs": false,
      "behavioral_concerns": false,
      "assessed_by": "Jessica Martinez",
      "notes": "Well-socialized dog, likely lost or abandoned. No signs of abuse or neglect."
    },
    "intake_photos": [
      {
        "photo_id": "PHOTO-001",
        "url": "https://storage.wia.org/pet-welfare/photos/WIA-PET-2025-000001-intake-01.jpg",
        "timestamp": "2025-03-15T10:15:00Z",
        "view": "right_side",
        "photographer": "Jessica Martinez"
      }
    ],
    "microchip_scan": {
      "scanned": true,
      "chip_found": true,
      "microchip_id": "900123456789012",
      "manufacturer": "HomeAgain",
      "registration_status": "not_registered",
      "owner_contact_attempted": true,
      "owner_contact_result": "number_disconnected"
    },
    "hold_status": {
      "hold_type": "stray_hold",
      "hold_start": "2025-03-15T10:00:00Z",
      "hold_end": "2025-03-20T10:00:00Z",
      "hold_duration_days": 5,
      "reason": "Statutory stray hold period",
      "can_be_adopted_after": "2025-03-20T10:00:00Z"
    },
    "housing_assignment": {
      "kennel_id": "K-15",
      "building": "Main Shelter",
      "wing": "North",
      "assignment_date": "2025-03-15T11:00:00Z",
      "kennel_type": "standard_large_dog",
      "indoor_outdoor": true,
      "special_requirements": []
    }
  }
}
```

### 4.4 국경 간 운송 스키마

IATA 및 세관 요구사항을 준수하는 국제 동물 이동에 대한 완전한 문서입니다.

```json
{
  "transport_manifest": {
    "transport_id": "WIA-TRN-2025-009012",
    "animal_id": "WIA-PET-2025-000001",
    "transport_type": "international_adoption",
    "status": "in_transit",
    "origin": {
      "country_code": "US",
      "organization_id": "WIA-ORG-US-001234",
      "facility_name": "Happy Paws Animal Shelter",
      "address": "123 Animal Way, San Francisco, CA 94102",
      "contact": {
        "name": "Jessica Martinez",
        "phone": "+1-415-555-0123",
        "email": "jessica.martinez@happypaws.org"
      },
      "departure_date": "2025-12-20T08:00:00Z"
    },
    "destination": {
      "country_code": "GB",
      "organization_id": "WIA-ORG-GB-005678",
      "facility_name": "London Pet Rescue",
      "address": "456 Animal Street, London, UK E1 6AN",
      "contact": {
        "name": "James Thompson",
        "phone": "+44-20-7946-0958",
        "email": "james.thompson@londonpetrescue.org.uk"
      },
      "expected_arrival": "2025-12-21T14:00:00Z"
    },
    "route": [
      {
        "leg_number": 1,
        "departure_airport": "SFO",
        "arrival_airport": "JFK",
        "departure_time": "2025-12-20T10:00:00Z",
        "arrival_time": "2025-12-20T18:30:00Z",
        "airline": "American Airlines",
        "flight_number": "AA100",
        "booking_reference": "ABC123"
      }
    ],
    "iata_compliance": {
      "live_animals_regulations_version": "LAR-49",
      "container_requirement": "CR82",
      "container_id": "CRATE-2025-001",
      "container_dimensions": {
        "length_cm": 91,
        "width_cm": 61,
        "height_cm": 66
      },
      "ventilation_compliance": true,
      "food_water_provision": true,
      "handler_instructions": "Handle with care. Friendly dog. Water every 4 hours.",
      "special_requirements": []
    },
    "health_documentation": {
      "health_certificate_id": "HC-US-2025-001",
      "rabies_certificate_id": "RAB-US-2025-001",
      "veterinary_examination_date": "2025-12-15T10:00:00Z",
      "fit_to_travel": true,
      "certifying_veterinarian": "Dr. Sarah Johnson",
      "usda_endorsement": true,
      "usda_endorsement_number": "USDA-2025-001234"
    },
    "import_permits": [
      {
        "permit_id": "UK-IMPORT-2025-001",
        "issuing_authority": "UK Department for Environment, Food and Rural Affairs",
        "issue_date": "2025-12-01T10:00:00Z",
        "valid_until": "2026-01-01",
        "permit_type": "pet_travel_scheme",
        "special_conditions": [
          "Must be microchipped",
          "Must have valid rabies vaccination",
          "Must have tapeworm treatment 1-5 days before entry"
        ]
      }
    ],
    "quarantine_requirements": {
      "required": false,
      "reason": "Compliant with UK Pet Travel Scheme",
      "facility": null,
      "duration_days": 0
    },
    "customs_declaration": {
      "declared_value_usd": 0,
      "purpose": "adoption",
      "commercial": false,
      "customs_form_number": "UK-CUSTOMS-2025-001234"
    },
    "adopter_information": {
      "name": "Emily Wilson",
      "address": "789 Pet Lane, London, UK E1 7AA",
      "phone": "+44-20-7946-1234",
      "email": "emily.wilson@email.co.uk",
      "adoption_contract_id": "ADOPT-2025-001"
    }
  }
}
```

### 4.5 학대 신고 스키마

동물 학대 및 방치 사건에 대한 표준화된 문서입니다.

```json
{
  "abuse_report": {
    "incident_id": "WIA-INC-2025-005678",
    "report_date": "2025-11-15T14:30:00Z",
    "incident_date": "2025-11-15T10:00:00Z",
    "status": "under_investigation",
    "severity_level": "moderate",
    "incident_type": "neglect",
    "reporter": {
      "reporter_type": "citizen",
      "anonymous": false,
      "name": "John Smith",
      "contact": {
        "phone": "+1-415-555-9876",
        "email": "john.smith@email.com"
      },
      "address": "123 Main Street, San Francisco, CA 94101",
      "relationship_to_animals": "neighbor"
    },
    "location": {
      "address": "456 Oak Street, San Francisco, CA 94102",
      "gps_coordinates": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "property_type": "single_family_residence",
      "access_restrictions": "private_property"
    },
    "animals_involved": [
      {
        "description": "Large mixed-breed dog, brown and white",
        "estimated_age": "5-7 years",
        "sex": "male",
        "microchip_scanned": false,
        "current_location": "seized_by_authority",
        "shelter_id": "WIA-ORG-US-001234",
        "animal_id": "WIA-PET-2025-000099"
      }
    ],
    "incident_details": {
      "description": "Dog found chained outside without shelter, food, or water in freezing temperatures. Animal appears malnourished.",
      "duration": "unknown",
      "environmental_conditions": {
        "temperature_celsius": -2,
        "weather": "freezing_rain",
        "exposure": "no_shelter"
      },
      "visible_injuries": true,
      "injury_description": "Matted fur, visible ribs, pressure sores from chain",
      "behavioral_indicators": "lethargic, fearful"
    },
    "evidence": [
      {
        "evidence_id": "EVID-001",
        "type": "photograph",
        "url": "https://secure.wia.org/evidence/WIA-INC-2025-005678-001.jpg",
        "timestamp": "2025-11-15T10:30:00Z",
        "description": "Photo of dog chained outside",
        "collected_by": "Officer Maria Garcia"
      }
    ],
    "investigation": {
      "assigned_to": "Officer Maria Garcia",
      "agency": "San Francisco Animal Care & Control",
      "case_number": "SFACC-INV-2025-1115-001",
      "investigation_start": "2025-11-15T11:00:00Z",
      "last_updated": "2025-12-15T16:00:00Z",
      "actions_taken": [
        {
          "action_date": "2025-11-15T11:30:00Z",
          "action": "animal_seizure",
          "description": "Dog seized under emergency authority",
          "performed_by": "Officer Maria Garcia"
        }
      ],
      "findings": "Evidence of long-term neglect. Animal malnourished and exposed to dangerous weather conditions without adequate shelter, food, or water.",
      "recommendations": "Recommend criminal charges for animal neglect. Animal should not be returned to owner."
    },
    "legal_proceedings": {
      "charges_filed": true,
      "charge_date": "2025-11-20T10:00:00Z",
      "charges": [
        {
          "statute": "California Penal Code 597",
          "description": "Animal neglect",
          "severity": "misdemeanor"
        }
      ],
      "court_date": "2026-01-15T09:00:00Z",
      "case_number": "SF-2025-CR-1234",
      "prosecutor": "San Francisco District Attorney's Office"
    },
    "outcome": {
      "animal_disposition": "permanent_seizure",
      "available_for_adoption": true,
      "owner_surrender": false,
      "conviction": null,
      "penalties": null
    }
  }
}
```

---

## 복지 메트릭

### 6.1 복지 점수 시스템

WIA Welfare Score는 다섯 가지 영역에 걸친 포괄적인 0-100 척도 평가입니다:

| 영역 | 가중치 | 설명 |
|------|--------|------|
| Physical Health | 25% | 의료 상태, 영양, 체력 |
| Mental Wellbeing | 20% | 스트레스 수준, 불안, 우울증 지표 |
| Social Behavior | 20% | 상호작용 품질, 사회화 |
| Environmental Enrichment | 20% | 주거 품질, 자극, 운동 |
| Care Standards | 15% | 케어 품질, 직원 전문성, 자원 |

### 6.2 점수 기준

| 점수 범위 | 분류 | 설명 |
|-----------|------|------|
| 90-100 | Excellent | 모든 영역에서 최적의 복지 |
| 75-89 | Good | 약간의 개선 가능성이 있는 높은 복지 |
| 60-74 | Acceptable | 적절한 복지, 일부 우려사항 |
| 45-59 | Poor | 개입이 필요한 중대한 복지 문제 |
| 0-44 | Critical | 심각한 복지 손상, 즉각적인 조치 필요 |

### 6.3 평가 프로토콜

복지 평가는 다음 시기에 수행되어야 합니다:
- 입소 후 24시간 이내
- 장기 보호 중인 동물의 경우 월 1회
- 운송 전후
- 입양 배치 전
- 중요한 건강 또는 행동 사건 발생 후

---

## 유효성 검증 규칙

### 7.1 필수 필드 검증

| 필드 | 검증 규칙 |
|------|-----------|
| `microchip_id` | 15자리, ISO 11784/11785 준수 |
| `country_code` | 유효한 ISO 3166-1 alpha-2 코드 |
| `date_of_birth` | 미래 날짜 불가 |
| `welfare_score` | 0-100 정수 |
| `weight_kg` | 양수 |
| `vaccination_date` | 미래 날짜 불가 |

### 7.2 비즈니스 규칙

1. 의무 보류 기간 동안 동물 입양 불가
2. 국제 운송에는 유효한 건강 증명서 필요
3. 국경 간 이동 시 광견병 예방접종 최신 상태 유지 필요
4. 45점 미만의 복지 점수는 필수 개입 프로토콜 시작
5. 학대 신고는 24시간 이내에 담당자 배정 필요

### 7.3 교차 필드 검증

- `available_for_adoption`은 `hold_status.can_be_adopted_after`가 미래인 경우 true일 수 없음
- `age_months` > 6인 경우 입양 시 `neutered`가 true여야 함
- `transport_type` = "international"인 경우 `health_certificate_id` 필요

---

## 예제

### 예제 1: 국제 입양이 포함된 완전한 동물 프로필

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "550e8400-e29b-41d4-a716-446655440000",
  "document_type": "animal_profile",
  "created_at": "2025-03-15T10:00:00Z",
  "updated_at": "2025-12-18T14:30:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "ko",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012",
        "passport_number": "PET-US-2025-001"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "sex": "male",
        "neutered": true,
        "date_of_birth": "2023-06-15"
      },
      "welfare_scores": {
        "overall_score": 87,
        "physical_health": 90,
        "mental_wellbeing": 85,
        "social_behavior": 88,
        "environmental_enrichment": 82,
        "last_assessed": "2025-12-18T14:00:00Z"
      }
    }
  }
}
```

### 예제 2: 조사가 포함된 학대 신고

이 예제는 동물 학대 사건의 신고 및 조사 과정을 보여줍니다.

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "660f9500-f39c-52e5-b827-557766551111",
  "document_type": "abuse_report",
  "created_at": "2025-11-15T14:30:00Z",
  "updated_at": "2025-12-15T16:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-005678",
    "country_code": "US",
    "language": "ko",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "abuse_report": {
      "incident_id": "WIA-INC-2025-005678",
      "report_date": "2025-11-15T14:30:00Z",
      "incident_type": "neglect",
      "severity_level": "moderate",
      "status": "under_investigation",
      "incident_details": {
        "description": "Dog found chained outside without shelter, food, or water in freezing temperatures.",
        "visible_injuries": true
      }
    }
  }
}
```

### 예제 3: 국제 운송 매니페스트

이 예제는 미국에서 영국으로의 동물 운송에 대한 완전한 문서를 보여줍니다.

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "770fa611-g40d-63f6-c938-668877662222",
  "document_type": "transport_manifest",
  "created_at": "2025-12-18T08:00:00Z",
  "updated_at": "2025-12-20T10:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "ko",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "transport_manifest": {
      "transport_id": "WIA-TRN-2025-009012",
      "animal_id": "WIA-PET-2025-000001",
      "transport_type": "international_adoption",
      "status": "in_transit",
      "origin": {
        "country_code": "US",
        "departure_date": "2025-12-20T08:00:00Z"
      },
      "destination": {
        "country_code": "GB",
        "expected_arrival": "2025-12-21T14:00:00Z"
      },
      "iata_compliance": {
        "live_animals_regulations_version": "LAR-49",
        "container_requirement": "CR82"
      }
    }
  }
}
```

### 예제 4: 복지 평가

동물의 포괄적인 복지 평가 기록입니다.

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "880fb722-h51e-74g7-d049-779988773333",
  "document_type": "welfare_assessment",
  "created_at": "2025-12-18T14:00:00Z",
  "updated_at": "2025-12-18T14:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "ko",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "welfare_assessment": {
      "assessment_id": "ASSESS-2025-001",
      "animal_id": "WIA-PET-2025-000001",
      "assessment_date": "2025-12-18T14:00:00Z",
      "assessed_by": "Dr. Sarah Johnson, DVM",
      "protocol_version": "WIA-WELFARE-ASSESSMENT-v1.0",
      "scores": {
        "physical_health": {
          "score": 90,
          "observations": "Excellent body condition, healthy coat, no injuries",
          "metrics": {
            "body_condition_score": 5,
            "mobility": "excellent",
            "pain_indicators": "none"
          }
        },
        "mental_wellbeing": {
          "score": 85,
          "observations": "Alert and engaged, some anxiety around loud noises",
          "metrics": {
            "stress_level": "low",
            "anxiety_indicators": "mild",
            "behavioral_health": "good"
          }
        },
        "overall_score": 87
      },
      "recommendations": [
        "Continue current care protocols",
        "Increase mental enrichment activities",
        "Consider puzzle toys and training sessions"
      ],
      "next_assessment_due": "2026-01-18T14:00:00Z"
    }
  }
}
```

### 예제 5: 보호소 조직 프로필

보호소의 완전한 조직 정보입니다.

```json
{
  "wia_version": "1.0.0",
  "standard": "pet-welfare-global",
  "document_id": "990fc833-i62f-85h8-e150-880099884444",
  "document_type": "organization_profile",
  "created_at": "2025-01-01T10:00:00Z",
  "updated_at": "2025-12-18T10:00:00Z",
  "metadata": {
    "organization_id": "WIA-ORG-US-001234",
    "country_code": "US",
    "language": "ko",
    "timezone": "America/Los_Angeles"
  },
  "data": {
    "organization_profile": {
      "organization_id": "WIA-ORG-US-001234",
      "basic_info": {
        "name": "Happy Paws Animal Shelter",
        "organization_type": "nonprofit_shelter",
        "founded_date": "1995-06-15"
      },
      "contact": {
        "primary_address": {
          "street": "123 Animal Way",
          "city": "San Francisco",
          "region": "California",
          "country_code": "US"
        },
        "phone": "+1-415-555-0123",
        "email": "info@happypaws.org"
      },
      "operations": {
        "services_provided": [
          "animal_shelter",
          "adoption_services",
          "veterinary_care"
        ],
        "euthanasia_policy": "no_kill"
      },
      "capacity": {
        "total_capacity": 150,
        "current_occupancy": 127
      }
    }
  }
}
```

---

## 버전 히스토리

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-12-18 | WIA Pet Welfare Global 데이터 형식 표준 초기 릴리스 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
