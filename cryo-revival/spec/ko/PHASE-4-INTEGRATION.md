# WIA 냉동인간 소생 의료 시스템 통합 표준
## 4단계 명세

---

**버전**: 1.0.0
**상태**: Draft
**작성일**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [통합 아키텍처](#통합-아키텍처)
3. [FHIR 통합](#fhir-통합)
4. [HL7 v2.x 통합](#hl7-v2x-통합)
5. [EHR 시스템 통합](#ehr-시스템-통합)
6. [DICOM 통합](#dicom-통합)
7. [규제 준수](#규제-준수)
8. [데이터 매핑](#데이터-매핑)
9. [구현 가이드](#구현-가이드)

---

## 개요

### 1.1 목적

WIA 냉동인간 소생 의료 시스템 통합 표준은 전자 건강 기록(EHR), 의료 정보 교환(HIE), 의료 영상 시스템 및 규제 보고 시스템을 포함한 기존 의료 인프라와 소생 절차를 연결하기 위한 포괄적인 통합 프로토콜을 정의합니다.

**핵심 목표**:
- 병원 EHR 시스템과의 원활한 통합
- 상호운용성을 위한 FHIR 준수 데이터 교환
- 레거시 의료 시스템을 위한 HL7 v2.x 지원
- 의료 영상 및 모니터링을 위한 DICOM 통합
- 자동화된 규제 준수 보고
- 의료 시스템 전반의 실시간 데이터 동기화

### 1.2 통합 범위

| 시스템 유형 | 표준 | 우선순위 | 사용 사례 |
|-----------|------|---------|----------|
| EHR 시스템 | FHIR R4, HL7 v2.5 | 중요 | 환자 기록, 생체 징후 |
| 의료 영상 | DICOM | 높음 | 뇌 스캔, 조직 평가 |
| 검사실 시스템 | HL7 v2.5, LOINC | 높음 | 혈액 검사, 바이오마커 |
| 약국 시스템 | FHIR, RxNorm | 중간 | 약물 관리 |
| 국가 건강 DB | Custom/FHIR | 높음 | 규제 보고 |
| 의료 기기 | IEEE 11073 | 높음 | 생체 징후 모니터링 |

### 1.3 설계 원칙

1. **표준 우선**: 기존 의료 표준 활용
2. **양방향 동기화**: 시스템 간 양방향 데이터 흐름
3. **실시간 통합**: 즉각적인 데이터 가용성
4. **프라이버시 보호**: HIPAA 및 GDPR 준수
5. **감사 추적**: 완전한 통합 활동 로깅

---

## 통합 아키텍처

### 2.1 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA 냉동인간 소생 플랫폼                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   소생       │  │  모니터링    │  │   의료진     │          │
│  │   기록       │  │    시스템    │  │              │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         │                 │                 │                   │
│         └─────────────────┴─────────────────┘                   │
│                           │                                     │
│                    ┌──────▼──────┐                              │
│                    │ 통합         │                              │
│                    │   게이트웨이 │                              │
│                    └──────┬──────┘                              │
└───────────────────────────┼──────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
┌───────▼────────┐  ┌───────▼────────┐  ┌──────▼───────┐
│  FHIR 서버     │  │  HL7 v2.x      │  │    DICOM     │
│  (병원         │  │  인터페이스    │  │    PACS      │
│   EHR)         │  │  (레거시)      │  │              │
└───────┬────────┘  └───────┬────────┘  └──────┬───────┘
        │                   │                   │
┌───────▼────────────────────▼───────────────────▼───────┐
│           병원 정보 시스템 (HIS)                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │   ADT    │  │   검사실  │  │   약국   │             │
│  └──────────┘  └──────────┘  └──────────┘             │
└─────────────────────────────────────────────────────────┘
```

### 2.2 통합 패턴

#### 패턴 1: 실시간 동기화

```
소생 이벤트 → 통합 게이트웨이 → 변환 → EHR → 확인
```

#### 패턴 2: 일괄 내보내기

```
일일 요약 → 집계기 → FHIR 번들 → HIE → 규제 DB
```

#### 패턴 3: 쿼리/응답

```
EHR 쿼리 → 게이트웨이 → 소생 기록 → 변환 → 응답
```

### 2.3 데이터 흐름 다이어그램

```
┌──────────────┐
│  소생        │
│  절차        │─────┐
└──────────────┘     │
                     │
┌──────────────┐     │    ┌─────────────────┐
│  생체 징후   │────►├───►│  통합           │
│  모니터      │     │    │  변환           │
└──────────────┘     │    │  엔진           │
                     │    └────────┬────────┘
┌──────────────┐     │             │
│  의료        │     │             │
│  평가        │─────┘             │
└──────────────┘                   │
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐        ┌──────────────────┐      ┌─────────────────┐
│  FHIR 번들    │        │  HL7 v2.x ADT    │      │  DICOM 래퍼     │
│  (Patient,    │        │  메시지          │      │  (뇌 스캔)      │
│  Procedure,   │        │                  │      │                 │
│  Observation) │        │                  │      │                 │
└───────┬───────┘        └────────┬─────────┘      └────────┬────────┘
        │                         │                         │
        ▼                         ▼                         ▼
┌─────────────────────────────────────────────────────────────┐
│              병원 정보 시스템                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## FHIR 통합

### 3.1 FHIR 리소스

#### Patient 리소스

```json
{
  "resourceType": "Patient",
  "id": "SUBJ-2025-001",
  "identifier": [
    {
      "system": "https://wia.live/cryo-revival/subject-id",
      "value": "SUBJ-2025-001"
    },
    {
      "system": "https://hospital.example.com/mrn",
      "value": "MRN-2025-001"
    }
  ],
  "extension": [
    {
      "url": "https://wia.live/fhir/extension/preservation-record",
      "valueReference": {
        "reference": "DocumentReference/PRES-2024-001"
      }
    },
    {
      "url": "https://wia.live/fhir/extension/revival-date",
      "valueDateTime": "2025-01-15T08:00:00Z"
    }
  ],
  "birthDate": "1950-01-15",
  "gender": "male",
  "managingOrganization": {
    "reference": "Organization/FAC-KR-REVIVAL-001",
    "display": "WIA 소생 의료센터 서울"
  }
}
```

#### Procedure 리소스 (소생)

```json
{
  "resourceType": "Procedure",
  "id": "REV-2025-001",
  "identifier": [
    {
      "system": "https://wia.live/cryo-revival/revival-id",
      "value": "REV-2025-001"
    }
  ],
  "status": "completed",
  "category": {
    "coding": [
      {
        "system": "http://snomed.info/sct",
        "code": "387713003",
        "display": "수술 절차"
      }
    ]
  },
  "code": {
    "coding": [
      {
        "system": "https://wia.live/procedure-codes",
        "code": "cryo-revival-full-body",
        "display": "냉동인간 소생 - 전신"
      }
    ],
    "text": "전신 냉동인간 소생 절차"
  },
  "subject": {
    "reference": "Patient/SUBJ-2025-001"
  },
  "performedPeriod": {
    "start": "2025-01-15T08:00:00Z",
    "end": "2025-01-16T06:00:00Z"
  },
  "performer": [
    {
      "actor": {
        "reference": "Practitioner/DR-001",
        "display": "김민준 박사"
      },
      "role": {
        "coding": [
          {
            "system": "http://snomed.info/sct",
            "code": "304292004",
            "display": "외과의사"
          }
        ]
      }
    }
  ],
  "location": {
    "reference": "Location/FAC-KR-REVIVAL-001",
    "display": "WIA 소생 의료센터 서울"
  },
  "outcome": {
    "coding": [
      {
        "system": "http://snomed.info/sct",
        "code": "385669000",
        "display": "성공"
      }
    ],
    "text": "완전한 신경학적 회복을 동반한 성공적인 소생"
  },
  "note": [
    {
      "text": "환자는 탁월한 회복을 보이고 있습니다. 모든 생체 징후가 안정적입니다. 기억 통합이 진행 중입니다."
    }
  ]
}
```

#### Observation 리소스 (생체 징후)

```json
{
  "resourceType": "Observation",
  "id": "VS-2025-001-12345",
  "status": "final",
  "category": [
    {
      "coding": [
        {
          "system": "http://terminology.hl7.org/CodeSystem/observation-category",
          "code": "vital-signs",
          "display": "생체 징후"
        }
      ]
    }
  ],
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "8867-4",
        "display": "심박수"
      }
    ]
  },
  "subject": {
    "reference": "Patient/SUBJ-2025-001"
  },
  "effectiveDateTime": "2025-01-15T19:00:00Z",
  "valueQuantity": {
    "value": 72,
    "unit": "회/분",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  },
  "interpretation": [
    {
      "coding": [
        {
          "system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation",
          "code": "N",
          "display": "정상"
        }
      ]
    }
  ],
  "referenceRange": [
    {
      "low": {
        "value": 60,
        "unit": "회/분"
      },
      "high": {
        "value": 100,
        "unit": "회/분"
      }
    }
  ],
  "device": {
    "reference": "Device/CARDIAC-MONITOR-001"
  }
}
```

#### Bundle 리소스 (전체 소생 기록)

```json
{
  "resourceType": "Bundle",
  "id": "revival-bundle-REV-2025-001",
  "type": "transaction",
  "entry": [
    {
      "fullUrl": "Patient/SUBJ-2025-001",
      "resource": {
        "resourceType": "Patient",
        "id": "SUBJ-2025-001"
      },
      "request": {
        "method": "PUT",
        "url": "Patient/SUBJ-2025-001"
      }
    },
    {
      "fullUrl": "Procedure/REV-2025-001",
      "resource": {
        "resourceType": "Procedure",
        "id": "REV-2025-001"
      },
      "request": {
        "method": "POST",
        "url": "Procedure"
      }
    },
    {
      "fullUrl": "Observation/VS-2025-001-12345",
      "resource": {
        "resourceType": "Observation",
        "id": "VS-2025-001-12345"
      },
      "request": {
        "method": "POST",
        "url": "Observation"
      }
    }
  ]
}
```

### 3.2 FHIR API 작업

#### 소생 기록 생성

```http
POST https://fhir.hospital.example.com/Procedure
Content-Type: application/fhir+json
Authorization: Bearer <token>

{
  "resourceType": "Procedure",
  "status": "in-progress",
  "code": {
    "coding": [{
      "system": "https://wia.live/procedure-codes",
      "code": "cryo-revival-full-body"
    }]
  },
  "subject": { "reference": "Patient/SUBJ-2025-001" }
}
```

#### 소생 절차 검색

```http
GET https://fhir.hospital.example.com/Procedure?code=cryo-revival-full-body&status=in-progress
```

#### 절차 상태 업데이트

```http
PATCH https://fhir.hospital.example.com/Procedure/REV-2025-001
Content-Type: application/json-patch+json

[
  {
    "op": "replace",
    "path": "/status",
    "value": "completed"
  }
]
```

---

## HL7 v2.x 통합

### 4.1 ADT 메시지 (입원/퇴원/전원)

#### ADT^A01 - 환자 입원

```
MSH|^~\&|WIA_REVIVAL|FAC-KR-REVIVAL-001|HIS|HOSPITAL|20250115080000||ADT^A01|MSG00001|P|2.5
EVN|A01|20250115080000
PID|1||SUBJ-2025-001^^^WIA^MR||홍^길동^||19500115|M|||서울시 강남구 테헤란로 123^^SEOUL^^06000^KR
PV1|1|I|REVIVAL^UNIT^BED01^FAC-KR-REVIVAL-001|E|||DR-001^김^민준^^^DR|||REVIVAL||||A|||DR-001|REVIVAL|||||||||||||||||||||||||20250115080000
```

#### ADT^A03 - 환자 퇴원

```
MSH|^~\&|WIA_REVIVAL|FAC-KR-REVIVAL-001|HIS|HOSPITAL|20250116060000||ADT^A03|MSG00002|P|2.5
EVN|A03|20250116060000
PID|1||SUBJ-2025-001^^^WIA^MR||홍^길동^||19500115|M
PV1|1|I|REVIVAL^UNIT^BED01^FAC-KR-REVIVAL-001|E|||DR-001^김^민준^^^DR|||REVIVAL||||A|||DR-001|REVIVAL|||||||||||||||||||||||||20250116060000|20250116060000
```

### 4.2 ORU 메시지 (관찰 결과)

#### ORU^R01 - 생체 징후 보고서

```
MSH|^~\&|WIA_REVIVAL|FAC-KR-REVIVAL-001|HIS|HOSPITAL|20250115190000||ORU^R01|MSG00003|P|2.5
PID|1||SUBJ-2025-001^^^WIA^MR||홍^길동^||19500115|M
OBR|1|REV-2025-001|VS-2025-001-12345|^생체 징후|||20250115190000
OBX|1|NM|8867-4^심박수^LN||72|회/분|60-100|N|||F|||20250115190000||CARDIAC-MONITOR-001
OBX|2|NM|8480-6^수축기 혈압^LN||120|mm[Hg]|90-140|N|||F|||20250115190000||BP-MONITOR-001
OBX|3|NM|8462-4^이완기 혈압^LN||80|mm[Hg]|60-90|N|||F|||20250115190000||BP-MONITOR-001
OBX|4|NM|8310-5^체온^LN||37.0|섭씨|36.5-37.5|N|||F|||20250115190000||TEMP-MONITOR-001
OBX|5|NM|2708-6^산소 포화도^LN||98|%|95-100|N|||F|||20250115190000||PULSE-OX-001
```

### 4.3 SIU 메시지 (스케줄링)

#### SIU^S12 - 소생 절차 스케줄링

```
MSH|^~\&|WIA_REVIVAL|FAC-KR-REVIVAL-001|HIS|HOSPITAL|20250114100000||SIU^S12|MSG00004|P|2.5
SCH|||||소생 절차|CRYO-REVIVAL-FULL-BODY|REVIVAL|60|min|^^^20250115080000
PID|1||SUBJ-2025-001^^^WIA^MR||홍^길동^||19500115|M
RGS|1|A
AIG|1|A|DR-001^김^민준^^^DR
AIL|1|A|REVIVAL^UNIT^BED01^FAC-KR-REVIVAL-001
```

---

## EHR 시스템 통합

### 5.1 Epic 통합

#### Epic MyChart 환자 포털

```javascript
// Epic SMART on FHIR 통합
const epicConfig = {
  clientId: 'wia-revival-client-id',
  scope: 'patient/Patient.read patient/Procedure.write patient/Observation.write',
  redirectUri: 'https://revival.wia.live/epic-callback',
  iss: 'https://fhir.epic.com/interconnect-fhir-oauth/api/FHIR/R4'
};

// SMART 앱 실행
FHIR.oauth2.authorize({
  client_id: epicConfig.clientId,
  scope: epicConfig.scope,
  redirect_uri: epicConfig.redirectUri
});

// 인증 후 소생 절차 생성
FHIR.oauth2.ready().then(client => {
  return client.create({
    resourceType: 'Procedure',
    status: 'in-progress',
    code: {
      coding: [{
        system: 'https://wia.live/procedure-codes',
        code: 'cryo-revival-full-body'
      }]
    },
    subject: { reference: `Patient/${client.patient.id}` }
  });
});
```

### 5.2 Cerner 통합

#### Cerner Millennium

```python
from fhirclient import client
from fhirclient.models.procedure import Procedure

# Cerner FHIR 구성
settings = {
    'app_id': 'wia_revival_app',
    'api_base': 'https://fhir-myrecord.cerner.com/r4/',
    'redirect_uri': 'https://revival.wia.live/cerner-callback'
}

smart = client.FHIRClient(settings=settings)

# 절차 생성
procedure = Procedure()
procedure.status = 'in-progress'
procedure.subject = {'reference': f'Patient/{patient_id}'}
procedure.code = {
    'coding': [{
        'system': 'https://wia.live/procedure-codes',
        'code': 'cryo-revival-full-body',
        'display': '냉동인간 소생 - 전신'
    }]
}

procedure.create(smart.server)
```

### 5.3 Allscripts 통합

#### Allscripts API

```http
POST https://api.allscripts.com/v1/procedures
Authorization: Bearer <allscripts_token>
Content-Type: application/json

{
  "patientId": "SUBJ-2025-001",
  "procedureCode": "CRYO-REVIVAL-FB",
  "procedureDescription": "냉동인간 소생 - 전신",
  "scheduledDate": "2025-01-15T08:00:00Z",
  "performingPhysician": "DR-001",
  "location": "REVIVAL-UNIT",
  "status": "IN_PROGRESS"
}
```

---

## DICOM 통합

### 6.1 뇌 영상 통합

#### DICOM Modality Worklist

```xml
<?xml version="1.0" encoding="UTF-8"?>
<DicomDataSet>
  <DicomAttribute tag="00080050" vr="SH">
    <Value number="1">REV-2025-001</Value>
  </DicomAttribute>
  <DicomAttribute tag="00100010" vr="PN">
    <PersonName number="1">
      <Alphabetic>
        <FamilyName>홍</FamilyName>
        <GivenName>길동</GivenName>
      </Alphabetic>
    </PersonName>
  </DicomAttribute>
  <DicomAttribute tag="00100020" vr="LO">
    <Value number="1">SUBJ-2025-001</Value>
  </DicomAttribute>
  <DicomAttribute tag="00400100" vr="SQ">
    <Item number="1">
      <DicomAttribute tag="00080060" vr="CS">
        <Value number="1">MR</Value>
      </DicomAttribute>
      <DicomAttribute tag="00321060" vr="LO">
        <Value number="1">소생 후 뇌 평가</Value>
      </DicomAttribute>
    </Item>
  </DicomAttribute>
</DicomDataSet>
```

### 6.2 DICOM WADO-RS (Web Access to DICOM Objects)

```http
GET https://pacs.hospital.example.com/wado-rs/studies/1.2.840.113619.2.55.3.123456789.100/series/1.2.840.113619.2.55.3.123456789.101
Accept: multipart/related; type="application/dicom"
Authorization: Bearer <token>
```

---

## 규제 준수

### 7.1 FDA 보고

#### 의료 기기 보고서 (MDR)

```json
{
  "reportType": "medical_device_report",
  "reportId": "MDR-2025-001",
  "revivalId": "REV-2025-001",
  "facilityId": "FAC-KR-REVIVAL-001",
  "reportDate": "2025-01-16T10:00:00Z",
  "deviceInformation": {
    "deviceName": "WIA 냉동인간 소생 시스템",
    "manufacturer": "WIA 의료 기술",
    "modelNumber": "CRS-1000",
    "serialNumber": "SN-2024-001"
  },
  "procedureOutcome": {
    "status": "successful",
    "adverseEvents": [],
    "devicePerformance": "nominal",
    "notes": "모든 시스템이 사양 내에서 작동했습니다"
  }
}
```

### 7.2 HIPAA 준수 로깅

```json
{
  "auditEventId": "audit-550e8400",
  "eventType": "data_export",
  "timestamp": "2025-01-16T08:00:00Z",
  "actor": {
    "userId": "DR-001",
    "role": "physician",
    "facility": "FAC-KR-REVIVAL-001"
  },
  "entity": {
    "type": "patient_record",
    "identifier": "SUBJ-2025-001",
    "sensitivity": "PHI"
  },
  "action": "export_to_ehr",
  "outcome": "success",
  "destination": {
    "system": "hospital-ehr-001",
    "type": "FHIR_server"
  },
  "legalBasis": "patient_care",
  "consentReference": "CONSENT-2024-1234"
}
```

---

## 데이터 매핑

### 8.1 생체 징후 매핑

| WIA 소생 필드 | LOINC 코드 | FHIR 요소 | HL7 v2.x OBX-3 |
|--------------|-----------|----------|---------------|
| `heart_rate` | 8867-4 | Observation.valueQuantity | 8867-4^심박수^LN |
| `blood_pressure.systolic` | 8480-6 | Observation.component[0].valueQuantity | 8480-6^수축기 혈압^LN |
| `blood_pressure.diastolic` | 8462-4 | Observation.component[1].valueQuantity | 8462-4^이완기 혈압^LN |
| `body_temperature` | 8310-5 | Observation.valueQuantity | 8310-5^체온^LN |
| `oxygen_saturation` | 2708-6 | Observation.valueQuantity | 2708-6^산소 포화도^LN |
| `respiratory_rate` | 9279-1 | Observation.valueQuantity | 9279-1^호흡수^LN |
| `glasgow_coma_scale` | 9269-2 | Observation.valueQuantity | 9269-2^글래스고 혼수 척도^LN |

### 8.2 절차 상태 매핑

| WIA 상태 | FHIR Procedure.status | SNOMED CT 코드 |
|---------|----------------------|---------------|
| `pending` | preparation | 385651009 |
| `warming` | in-progress | 385651009 |
| `perfusion_reversal` | in-progress | 385651009 |
| `in_progress` | in-progress | 385651009 |
| `successful` | completed | 385669000 |
| `partial_success` | completed | 255617001 |
| `unsuccessful` | stopped | 385654001 |

---

## 구현 가이드

### 9.1 통합 체크리스트

- [ ] **통합 전**
  - [ ] EHR 시스템 자격 증명 및 엔드포인트 획득
  - [ ] FHIR/HL7 v2.x 메시지 템플릿 구성
  - [ ] 안전한 VPN/TLS 연결 설정
  - [ ] HIPAA 비즈니스 제휴 계약 완료
  - [ ] 감사 로깅 구성

- [ ] **FHIR 통합**
  - [ ] SMART on FHIR 애플리케이션 등록
  - [ ] OAuth 2.0 인증 구현
  - [ ] WIA 데이터를 FHIR 리소스로 매핑
  - [ ] FHIR Bundle 생성 테스트
  - [ ] FHIR 명세에 대한 검증

- [ ] **HL7 v2.x 통합**
  - [ ] HL7 인터페이스 엔진 구성
  - [ ] 메시지 파서 구현
  - [ ] 확인 처리 설정
  - [ ] ADT, ORU, SIU 메시지 테스트

- [ ] **테스팅**
  - [ ] 데이터 변환을 위한 단위 테스트
  - [ ] 테스트 EHR 환경과의 통합 테스트
  - [ ] 실시간 동기화를 위한 성능 테스트
  - [ ] 보안 침투 테스트

- [ ] **운영 시작**
  - [ ] 단일 소생 절차로 파일럿
  - [ ] 통합 로그 모니터링
  - [ ] 통합 워크플로우에 대한 의료진 교육
  - [ ] 통합 구성 문서화

### 9.2 샘플 통합 코드

```typescript
import { FHIRClient } from '@wia/fhir-client';
import { HL7Generator } from '@wia/hl7-generator';
import { RevivalRecord } from '@wia/cryo-revival';

class HealthcareIntegration {
  constructor(
    private fhirClient: FHIRClient,
    private hl7Generator: HL7Generator
  ) {}

  async exportRevivalToEHR(revivalId: string): Promise<void> {
    // 소생 기록 가져오기
    const revival = await this.getRevivalRecord(revivalId);

    // FHIR Bundle로 변환
    const bundle = this.transformToFHIRBundle(revival);

    // FHIR 서버로 전송
    const result = await this.fhirClient.transaction(bundle);

    // 레거시 시스템을 위한 HL7 v2.x 메시지 생성
    const adtMessage = this.hl7Generator.createADT(revival);
    const oruMessage = this.hl7Generator.createORU(revival.vitalSigns);

    // HL7 메시지 전송
    await this.sendHL7Message(adtMessage);
    await this.sendHL7Message(oruMessage);

    // 통합 이벤트 로깅
    await this.logIntegrationEvent({
      revivalId,
      timestamp: new Date(),
      systems: ['FHIR', 'HL7v2'],
      status: 'success'
    });
  }

  private transformToFHIRBundle(revival: RevivalRecord) {
    return {
      resourceType: 'Bundle',
      type: 'transaction',
      entry: [
        {
          resource: this.createPatientResource(revival.subject),
          request: { method: 'PUT', url: `Patient/${revival.subject.id}` }
        },
        {
          resource: this.createProcedureResource(revival),
          request: { method: 'POST', url: 'Procedure' }
        },
        ...revival.vitalSigns.map(vs => ({
          resource: this.createObservationResource(vs),
          request: { method: 'POST', url: 'Observation' }
        }))
      ]
    };
  }

  private createObservationResource(vitalSign: any) {
    return {
      resourceType: 'Observation',
      status: 'final',
      category: [{
        coding: [{
          system: 'http://terminology.hl7.org/CodeSystem/observation-category',
          code: 'vital-signs'
        }]
      }],
      code: {
        coding: [{
          system: 'http://loinc.org',
          code: this.getLOINCCode(vitalSign.type)
        }]
      },
      subject: { reference: `Patient/${vitalSign.subjectId}` },
      effectiveDateTime: vitalSign.timestamp,
      valueQuantity: {
        value: vitalSign.value,
        unit: vitalSign.unit,
        system: 'http://unitsofmeasure.org'
      }
    };
  }
}
```

---

<div align="center">

**WIA 냉동인간 소생 의료 시스템 통합 표준 v1.0.0**

**弘益人間 (홍익인간)** - 모든 인류에게 이익을

---

**© 2025 WIA 표준 위원회**

**MIT 라이선스**

</div>
