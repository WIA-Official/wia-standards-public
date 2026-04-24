# WIA 냉동보존 생태계 연동
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
2. [연동 아키텍처](#연동-아키텍처)
3. [WIA 표준 상호운용성](#wia-표준-상호운용성)
4. [외부 시스템 연동](#외부-시스템-연동)
5. [마이그레이션 가이드](#마이그레이션-가이드)
6. [배포](#배포)
7. [모니터링 및 운영](#모니터링-및-운영)
8. [인증](#인증)
9. [참조 구현](#참조-구현)

---

## 개요

### 1.1 목적

4단계는 WIA 냉동보존이 더 넓은 WIA 생태계 및 외부 의료 시스템과 어떻게 통합되는지 정의하여 원활한 데이터 교환, 규제 준수 및 운영 상호운용성을 가능하게 합니다.

### 1.2 연동 목표

| 목표 | 설명 |
|------|------|
| **상호운용성** | 다른 WIA 표준과의 원활한 데이터 교환 |
| **규정 준수** | 의료 및 규제 요구사항 충족 |
| **확장성** | 성장하는 시설 네트워크 지원 |
| **복원력** | 장애 허용 분산 운영 |
| **감사 가능성** | 법적 준수를 위한 완전한 추적성 |

### 1.3 다른 단계와의 관계

```
┌─────────────────────────────────────────────────────────────────┐
│                       4단계: 연동                                │
│           (생태계 연결, 배포, 운영)                              │
├─────────────────────────────────────────────────────────────────┤
│  3단계: 프로토콜  │  2단계: API      │  1단계: 데이터           │
│  (실시간 통신)    │  (REST 인터페이스)│  (형식 및 스키마)        │
└─────────────────────────────────────────────────────────────────┘
```

---

## 연동 아키텍처

### 2.1 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                         외부 시스템                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │   병원   │  │   보험   │  │   정부   │  │   연구   │            │
│  │   EHR    │  │   시스템  │  │  레지스트리│  │  파트너  │            │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘            │
└───────┼─────────────┼─────────────┼─────────────┼───────────────────┘
        │             │             │             │
        │ HL7 FHIR    │ EDI/X12     │ Custom API  │ Research API
        │             │             │             │
┌───────▼─────────────▼─────────────▼─────────────▼───────────────────┐
│                    연동 게이트웨이                                    │
│         (프로토콜 변환, 인증, 라우팅)                                │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
        ▼                       ▼                       ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  냉동보존     │     │  신원 확인    │     │    동의       │
│    서비스     │◄───►│    서비스     │◄───►│   서비스      │
└───────────────┘     └───────────────┘     └───────────────┘
                                │
                    ┌───────────▼───────────┐
                    │   공유 데이터 계층    │
                    │ (블록체인 / IPFS)     │
                    └───────────────────────┘
```

### 2.2 데이터 흐름 아키텍처

```
┌──────────────────────────────────────────────────────────────────┐
│                       데이터 흐름 계층                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │ 데이터 입력 │───►│    처리     │───►│ 데이터 출력 │          │
│  │    계층     │    │    계층     │    │    계층     │          │
│  └─────────────┘    └─────────────┘    └─────────────┘          │
│                                                                   │
│  소스:              작업:              목적지:                    │
│  • 센서             • 검증             • WIA 서비스              │
│  • 수동 입력        • 보강             • 외부 API                │
│  • 임포트           • 변환             • 보고서                  │
│  • API              • 저장             • 알림                    │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

### 2.3 이벤트 기반 아키텍처

```typescript
// 이벤트 유형
type CryoEvent =
  | 'preservation.started'
  | 'preservation.completed'
  | 'status.changed'
  | 'storage.alert'
  | 'transfer.initiated'
  | 'transfer.completed'
  | 'quality.reported'
  | 'consent.updated';

// 이벤트 구조
interface CryoEventMessage {
  eventType: CryoEvent;
  eventId: string;
  timestamp: string;
  source: {
    service: string;
    facilityId: string;
  };
  data: Record<string, unknown>;
  metadata: {
    correlationId: string;
    causationId?: string;
  };
}
```

---

## WIA 표준 상호운용성

### 3.1 관련 WIA 표준

| 표준 | 연동 유형 | 데이터 교환 |
|------|----------|-------------|
| **Cryo-Identity** | 양방향 | 대상자 식별 |
| **Cryo-Consent** | 필수 | 법적 동의 기록 |
| **Cryo-Revival** | 전방 | 소생 절차 |
| **Cryo-Legal** | 필수 | 법적 문서 |
| **Cryo-Asset** | 양방향 | 재무 기록 |
| **Cryo-Facility** | 필수 | 시설 인증 |

### 3.2 Cryo-Identity 연동

보존 기록을 신원에 연결:

```json
{
  "subjectId": "SUBJ-2025-001",
  "identityReference": {
    "standard": "wia-cryo-identity",
    "version": "1.0.0",
    "identityId": "ID-2025-001",
    "verificationLevel": "biometric",
    "lastVerified": "2025-01-15T10:00:00Z"
  }
}
```

**API 연동:**
```typescript
// 보존 전 신원 확인
const identity = await cryoIdentityClient.verify({
  subjectId: 'SUBJ-2025-001',
  biometricData: biometricSample,
  requiredLevel: 'biometric'
});

if (identity.verified) {
  await cryoPreservationClient.startPreservation({
    subjectId: 'SUBJ-2025-001',
    identityId: identity.identityId
  });
}
```

### 3.3 Cryo-Consent 연동

작업 전 동의 검증:

```json
{
  "preservationId": "PRV-2025-001",
  "consentReference": {
    "standard": "wia-cryo-consent",
    "version": "1.0.0",
    "consentId": "CONSENT-2024-1234",
    "scope": ["whole_body_preservation", "research_use", "revival_attempt"],
    "validFrom": "2024-06-01T00:00:00Z",
    "validUntil": null,
    "status": "active"
  }
}
```

**동의 검증 흐름:**
```typescript
// 모든 작업 전 동의 확인
async function validateConsentForOperation(
  subjectId: string,
  operation: string
): Promise<ConsentValidation> {
  const consent = await cryoConsentClient.getActiveConsent(subjectId);

  if (!consent) {
    throw new Error('활성 동의를 찾을 수 없음');
  }

  if (!consent.scope.includes(operation)) {
    throw new Error(`작업 "${operation}"이 동의 범위에 없음`);
  }

  return {
    valid: true,
    consentId: consent.consentId,
    validatedAt: new Date().toISOString()
  };
}
```

### 3.4 교차 표준 이벤트 흐름

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    동의     │     │   보존      │     │    신원     │
│   서비스    │     │   서비스    │     │   서비스    │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       │  consent.created  │                   │
       │──────────────────►│                   │
       │                   │                   │
       │                   │  identity.verify  │
       │                   │──────────────────►│
       │                   │                   │
       │                   │◄──────────────────│
       │                   │  identity.verified│
       │                   │                   │
       │ preservation.started                  │
       │◄──────────────────│                   │
       │                   │                   │
       │                   │ preservation.completed
       │                   │──────────────────►│
       │                   │                   │ 신원 업데이트
```

---

## 외부 시스템 연동

### 4.1 HL7 FHIR 연동

**FHIR 리소스 매핑:**

| 냉동보존 데이터 | FHIR 리소스 |
|----------------|-------------|
| 대상자 인구통계 | Patient |
| 보존 절차 | Procedure |
| 품질 보고서 | DiagnosticReport |
| 동의 | Consent |
| 시설 | Organization |

**FHIR Procedure 예시:**
```json
{
  "resourceType": "Procedure",
  "id": "cryo-preservation-001",
  "status": "completed",
  "category": [
    {
      "coding": [
        {
          "system": "https://wia.live/fhir/procedure-category",
          "code": "cryopreservation",
          "display": "냉동보존"
        }
      ]
    }
  ],
  "code": {
    "coding": [
      {
        "system": "https://wia.live/fhir/procedure-code",
        "code": "whole-body-vitrification",
        "display": "전신 유리화"
      }
    ]
  },
  "subject": {
    "reference": "Patient/SUBJ-2025-001"
  },
  "performedPeriod": {
    "start": "2025-01-15T08:00:00Z",
    "end": "2025-01-16T03:00:00Z"
  }
}
```

### 4.2 블록체인 연동

**불변 기록 저장:**

```typescript
interface BlockchainRecord {
  recordType: 'preservation' | 'transfer' | 'quality';
  hash: string;
  previousHash: string;
  timestamp: number;
  facilityId: string;
  signature: string;
  metadata: {
    version: string;
    schema: string;
  };
}

// 보존 기록 해시 저장
async function storeOnBlockchain(
  record: PreservationRecord
): Promise<BlockchainReceipt> {
  const hash = await calculateHash(record);

  const blockchainRecord: BlockchainRecord = {
    recordType: 'preservation',
    hash: hash,
    previousHash: await getLatestHash(record.subjectId),
    timestamp: Date.now(),
    facilityId: record.facilityId,
    signature: await signRecord(hash),
    metadata: {
      version: '1.0.0',
      schema: 'wia-cryo-preservation'
    }
  };

  return await blockchainClient.store(blockchainRecord);
}
```

### 4.3 연구 데이터 내보내기

**익명화된 데이터 내보내기:**

```typescript
interface ResearchDataset {
  exportId: string;
  exportDate: string;
  consent: {
    researchUseApproved: boolean;
    anonymizationLevel: 'full' | 'partial' | 'coded';
  };
  records: AnonymizedRecord[];
  metadata: {
    recordCount: number;
    dateRange: { from: string; to: string };
    includedFields: string[];
  };
}

// 연구용 익명화 데이터 내보내기
async function exportForResearch(
  criteria: ExportCriteria
): Promise<ResearchDataset> {
  const records = await getRecordsWithResearchConsent(criteria);

  const anonymizedRecords = records.map(record => ({
    preservationType: record.preservationType,
    demographicBucket: anonymizeDemographics(record.demographics),
    qualityMetrics: record.quality,
    timeline: {
      ischemicTime: calculateIschemicTime(record.timeline),
      totalProcedureHours: calculateProcedureTime(record.timeline)
    }
    // 식별 정보 없음
  }));

  return {
    exportId: generateExportId(),
    exportDate: new Date().toISOString(),
    consent: { researchUseApproved: true, anonymizationLevel: 'full' },
    records: anonymizedRecords,
    metadata: {
      recordCount: anonymizedRecords.length,
      dateRange: criteria.dateRange,
      includedFields: ['preservationType', 'qualityMetrics', 'timeline']
    }
  };
}
```

---

## 마이그레이션 가이드

### 5.1 마이그레이션 개요

```
┌─────────────────────────────────────────────────────────────────┐
│                    마이그레이션 단계                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1단계: 평가        2단계: 준비          3단계: 마이그레이션      │
│  ├─ 데이터 감사    ├─ 스키마 매핑       ├─ 데이터 변환          │
│  ├─ 갭 분석       ├─ API 설정          ├─ 검증                 │
│  └─ 위험 평가     └─ 테스트 환경        └─ 전환                 │
│                                                                  │
│  4단계: 검증        5단계: 최적화                                │
│  ├─ 데이터 무결성  ├─ 성능 튜닝                                 │
│  ├─ 규정 준수 확인 ├─ 교육                                     │
│  └─ 사용자 수용    └─ 문서화                                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 마이그레이션 체크리스트

| 단계 | 설명 | 상태 |
|------|------|------|
| 1 | 기존 데이터 백업 | ☐ |
| 2 | WIA API 자격 증명 설정 | ☐ |
| 3 | 레거시 필드를 WIA 스키마에 매핑 | ☐ |
| 4 | 데이터 변환기 생성 | ☐ |
| 5 | 테스트 마이그레이션 실행 | ☐ |
| 6 | 마이그레이션된 데이터 검증 | ☐ |
| 7 | 프로덕션 마이그레이션 실행 | ☐ |
| 8 | 데이터 무결성 확인 | ☐ |
| 9 | 애플리케이션 연동 업데이트 | ☐ |
| 10 | 레거시 시스템 폐기 | ☐ |

---

## 배포

### 6.1 클라우드 배포 (Kubernetes)

```yaml
# deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-preservation-api
  namespace: wia-cryo
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-preservation-api
  template:
    metadata:
      labels:
        app: cryo-preservation-api
    spec:
      containers:
      - name: api
        image: wia/cryo-preservation-api:1.0.0
        ports:
        - containerPort: 8080
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
```

### 6.2 온프레미스 배포

```yaml
# docker-compose.yml
version: '3.8'

services:
  cryo-api:
    image: wia/cryo-preservation-api:1.0.0
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/cryo
      - REDIS_URL=redis://cache:6379
    depends_on:
      - db
      - cache
    restart: unless-stopped

  db:
    image: postgres:15
    volumes:
      - postgres_data:/var/lib/postgresql/data

  cache:
    image: redis:7-alpine
    volumes:
      - redis_data:/data

volumes:
  postgres_data:
  redis_data:
```

---

## 모니터링 및 운영

### 7.1 핵심 지표

| 지표 | 설명 | 알림 임계값 |
|------|------|-------------|
| `storage.temperature` | 듀어 온도 | > -195°C |
| `storage.nitrogen_level` | LN2 충전 수준 | < 20% |
| `api.response_time_p99` | 99번째 백분위 지연 | > 500ms |
| `api.error_rate` | 요청 오류율 | > 1% |
| `system.uptime` | 서비스 가용성 | < 99.9% |

### 7.2 Prometheus 메트릭

```yaml
# prometheus-rules.yaml
groups:
- name: cryo-preservation
  rules:
  - alert: StorageTemperatureHigh
    expr: cryo_storage_temperature_celsius > -195
    for: 5m
    labels:
      severity: critical
    annotations:
      summary: "저장 온도가 안전 임계값을 초과"
      description: "컨테이너 {{ $labels.container_id }} 온도가 {{ $value }}°C"

  - alert: NitrogenLevelLow
    expr: cryo_storage_nitrogen_level < 0.2
    for: 15m
    labels:
      severity: warning
    annotations:
      summary: "액체 질소 수준 낮음"
      description: "컨테이너 {{ $labels.container_id }} LN2 {{ $value | humanizePercentage }}"
```

### 7.3 운영 런북

**알림: 저장 온도 높음**

1. **즉각 조치:**
   - 영향받은 컨테이너의 LN2 수준 확인
   - 수동 측정으로 센서 정확도 검증
   - 시설 HVAC 상태 확인

2. **에스컬레이션:**
   - 온도 > -190°C: 당직 엔지니어 호출
   - 온도 > -180°C: 비상 프로토콜 시작

3. **해결:**
   - 감사 로그에 인시던트 문서화
   - 예방 유지보수 일정 업데이트

---

## 인증

### 8.1 WIA 인증 레벨

| 레벨 | 요구사항 |
|------|----------|
| **브론즈** | 데이터 형식 기본 준수 |
| **실버** | 전체 API 연동, 자동화된 모니터링 |
| **골드** | HA 배포, 블록체인 감사 추적 |
| **플래티넘** | 다중 시설 연동, 연구 기능 |

### 8.2 인증 프로세스

```
┌─────────────────────────────────────────────────────────────────┐
│                    인증 프로세스                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. 신청            2. 평가           3. 감사                    │
│  ├─ 양식 제출      ├─ 기술 검토     ├─ 현장 방문               │
│  ├─ 문서화         ├─ 보안 스캔     ├─ 데이터 검증             │
│  └─ 비용 지불      └─ API 테스트    └─ 직원 인터뷰             │
│                                                                  │
│  4. 개선            5. 인증          6. 유지                     │
│  ├─ 문제 수정      ├─ 인증서       ├─ 연간 감사               │
│  ├─ 재테스트       ├─ 배지/로고    ├─ 업데이트 보고서          │
│  └─ 문서화         └─ 레지스트리   └─ 갱신                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 8.3 규정 준수 체크리스트

| 요구사항 | 브론즈 | 실버 | 골드 | 플래티넘 |
|----------|:------:|:----:|:----:|:--------:|
| 1단계 데이터 형식 | ✓ | ✓ | ✓ | ✓ |
| 2단계 API 구현 | - | ✓ | ✓ | ✓ |
| 3단계 프로토콜 지원 | - | ✓ | ✓ | ✓ |
| 실시간 모니터링 | - | ✓ | ✓ | ✓ |
| 고가용성 | - | - | ✓ | ✓ |
| 블록체인 감사 추적 | - | - | ✓ | ✓ |
| 다중 시설 연동 | - | - | - | ✓ |
| 연구 데이터 내보내기 | - | - | - | ✓ |

---

## 참조 구현

### 9.1 GitHub 저장소

```
https://github.com/WIA-Official/cryo-preservation-reference
```

### 9.2 빠른 시작

```bash
# 저장소 복제
git clone https://github.com/WIA-Official/cryo-preservation-reference.git
cd cryo-preservation-reference

# 의존성 설치
npm install

# 환경 구성
cp .env.example .env
# 설정 편집

# 마이그레이션 실행
npm run db:migrate

# 서버 시작
npm run start

# 테스트 실행
npm run test
```

### 9.3 SDK 다운로드

| 언어 | 패키지 |
|------|--------|
| TypeScript/JS | `npm install @wia/cryo-preservation` |
| Python | `pip install wia-cryo-preservation` |
| Java | `implementation 'live.wia:cryo-preservation:1.0.0'` |
| Go | `go get github.com/WIA-Official/cryo-preservation-go` |

---

<div align="center">

**WIA 냉동보존 생태계 연동 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA 표준 위원회**

**MIT License**

</div>
