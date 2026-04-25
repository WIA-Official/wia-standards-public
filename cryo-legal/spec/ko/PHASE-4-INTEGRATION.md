# WIA Cryo-Legal 표준 - 4단계: 생태계 통합

**버전**: 1.0.0
**상태**: 초안
**날짜**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**주요 색상**: #06B6D4 (Cyan)

---

## 1. 개요

### 1.1 목적

4단계는 WIA Cryo-Legal 표준이 더 넓은 WIA 생태계 및 외부 시스템과 어떻게 통합되는지 정의하며, 냉동보존 시설, 신원 시스템 및 규정 준수 프레임워크 전반에 걸쳐 원활한 법적 문서 관리를 가능하게 합니다.

### 1.2 통합 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                       외부 시스템                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │ 법적     │  │ 신원     │  │ 금융     │  │ 의료     │            │
│  │ 데이터베이스│  │ 제공자   │  │ 시스템   │  │ 기록     │            │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘            │
└───────┼─────────────┼─────────────┼─────────────┼───────────────────┘
        │             │             │             │
        └──────────┬──┴──────┬──────┴──────┬──────┘
                   │         │             │
                   ▼         ▼             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    WIA 통합 계층                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Cryo-Legal 허브                            │   │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐               │   │
│  │  │ 문서      │  │ 서명      │  │ 규정 준수 │               │   │
│  │  │ 관리자    │  │ 검증기    │  │ 엔진      │               │   │
│  │  └───────────┘  └───────────┘  └───────────┘               │   │
│  └─────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
        ▼                       ▼                       ▼
┌──────────────┐       ┌──────────────┐       ┌──────────────┐
│ Cryo-Identity│       │ Cryo-Consent │       │  Cryo-Asset  │
│   표준       │       │   표준       │       │   표준       │
└──────────────┘       └──────────────┘       └──────────────┘
```

---

## 2. WIA 생태계 상호운용성

### 2.1 Cryo-Identity와의 통합

Cryo-Legal 표준은 당사자 검증을 위해 Cryo-Identity와 통합됩니다.

```json
{
  "integration": "cryo-identity",
  "version": "1.0.0",
  "operations": [
    {
      "operation": "verify_party",
      "endpoint": "/cryo-identity/v1/verify",
      "mapping": {
        "cryo-legal.parties[].identity.legalName": "cryo-identity.subject.fullName",
        "cryo-legal.parties[].identity.dateOfBirth": "cryo-identity.subject.birthDate",
        "cryo-legal.parties[].identity.identificationNumber": "cryo-identity.credentials[].number"
      }
    },
    {
      "operation": "link_identity",
      "endpoint": "/cryo-identity/v1/link",
      "mapping": {
        "cryo-legal.documentId": "cryo-identity.linkedDocuments[]"
      }
    }
  ]
}
```

**TypeScript 통합:**
```typescript
import { CryoLegalClient } from '@wia/cryo-legal';
import { CryoIdentityClient } from '@wia/cryo-identity';

async function verifyPartyIdentity(partyId: string): Promise<VerificationResult> {
  const legalClient = new CryoLegalClient();
  const identityClient = new CryoIdentityClient();

  // 법적 문서에서 당사자 가져오기
  const party = await legalClient.parties.get(partyId);

  // 신원 시스템과 대조 검증
  const verification = await identityClient.verify({
    fullName: party.identity.legalName,
    birthDate: party.identity.dateOfBirth,
    documentNumber: party.identity.identificationNumber,
    documentType: party.identity.identificationType
  });

  // 법적 당사자 검증 상태 업데이트
  if (verification.verified) {
    await legalClient.parties.update(partyId, {
      verificationStatus: 'verified',
      verificationId: verification.verificationId
    });
  }

  return verification;
}
```

### 2.2 Cryo-Consent와의 통합

```json
{
  "integration": "cryo-consent",
  "version": "1.0.0",
  "operations": [
    {
      "operation": "create_consent_record",
      "trigger": "document.signed",
      "condition": "documentType IN ['consent_form', 'advance_directive']",
      "action": {
        "endpoint": "/cryo-consent/v1/consents",
        "method": "POST",
        "payload": {
          "subjectId": "${parties[role=subject].partyId}",
          "consentType": "${documentType}",
          "documentReference": "${documentId}",
          "signatureReference": "${signatures[0].signatureId}",
          "effectiveDate": "${effectiveDate}",
          "jurisdiction": "${jurisdiction.primaryCountry}"
        }
      }
    }
  ]
}
```

### 2.3 Cryo-Asset과의 통합

```json
{
  "integration": "cryo-asset",
  "version": "1.0.0",
  "operations": [
    {
      "operation": "link_trust_assets",
      "trigger": "document.executed",
      "condition": "documentType = 'trust_document'",
      "action": {
        "endpoint": "/cryo-asset/v1/trusts/${trustId}/documents",
        "method": "POST",
        "payload": {
          "documentId": "${documentId}",
          "documentType": "legal_trust",
          "effectiveDate": "${effectiveDate}"
        }
      }
    }
  ]
}
```

### 2.4 표준 간 이벤트 버스

```typescript
import { WIAEventBus } from '@wia/event-bus';

const eventBus = new WIAEventBus();

// 표준 간 이벤트 구독
eventBus.subscribe('cryo-legal.document.executed', async (event) => {
  const { documentId, documentType, parties } = event.payload;

  // 관련 표준에 알림
  if (documentType === 'cryopreservation_contract') {
    await eventBus.publish('cryo-consent.consent.required', {
      subjectId: parties.find(p => p.role === 'subject').partyId,
      sourceDocument: documentId
    });

    await eventBus.publish('cryo-facility.contract.registered', {
      facilityId: parties.find(p => p.role === 'facility').partyId,
      contractId: documentId
    });
  }
});
```

---

## 3. 외부 시스템 통합

### 3.1 법적 데이터베이스 통합

**법원 기록 통합:**
```typescript
interface CourtRecordsAdapter {
  // 법원 기록 조회
  queryRecords(params: {
    jurisdiction: string;
    partyName: string;
    documentType?: string;
  }): Promise<CourtRecord[]>;

  // 법원에 문서 제출
  fileDocument(params: {
    jurisdiction: string;
    documentId: string;
    documentType: string;
    filingType: 'original' | 'amendment' | 'withdrawal';
  }): Promise<FilingResult>;

  // 제출 상태 확인
  getFilingStatus(filingId: string): Promise<FilingStatus>;
}

// 구현 예제
class KRCourtRecordsAdapter implements CourtRecordsAdapter {
  async fileDocument(params: FileDocumentParams): Promise<FilingResult> {
    const cryoLegal = new CryoLegalClient();
    const document = await cryoLegal.documents.get(params.documentId);

    // 법원이 수락하는 형식으로 문서 내보내기
    const pdf = await cryoLegal.documents.export(params.documentId, 'pdf');

    // 법원 전자제출 시스템에 제출
    const result = await this.courtAPI.efile({
      jurisdiction: params.jurisdiction,
      caseType: mapDocumentTypeToCaseType(params.documentType),
      documents: [pdf],
      filer: document.parties.find(p => p.role === 'legal_representative')
    });

    return result;
  }
}
```

### 3.2 신원 제공자 통합

**OAuth/OIDC 통합:**
```typescript
interface IdentityProvider {
  provider: string;
  protocol: 'oauth2' | 'oidc' | 'saml';
  endpoints: {
    authorize: string;
    token: string;
    userinfo: string;
  };
  scopes: string[];
  mapping: Record<string, string>;
}

const identityProviders: IdentityProvider[] = [
  {
    provider: 'gov-id-kr',
    protocol: 'oidc',
    endpoints: {
      authorize: 'https://id.go.kr/oauth/authorize',
      token: 'https://id.go.kr/oauth/token',
      userinfo: 'https://id.go.kr/oauth/userinfo'
    },
    scopes: ['openid', 'profile', 'identity_document'],
    mapping: {
      'sub': 'partyId',
      'name': 'identity.legalName',
      'birthdate': 'identity.dateOfBirth',
      'document_number': 'identity.identificationNumber'
    }
  },
  {
    provider: 'bank-kyc',
    protocol: 'oauth2',
    endpoints: {
      authorize: 'https://bank.co.kr/oauth/authorize',
      token: 'https://bank.co.kr/oauth/token',
      userinfo: 'https://bank.co.kr/api/kyc'
    },
    scopes: ['kyc_basic', 'kyc_documents'],
    mapping: {
      'customer_id': 'partyId',
      'full_name': 'identity.legalName',
      'verified_address': 'address'
    }
  }
];
```

### 3.3 공증 서비스

```typescript
interface NotarizationService {
  // 원격 온라인 공증 요청
  requestRON(params: {
    documentId: string;
    notaryJurisdiction: string;
    signerIds: string[];
    scheduledTime?: Date;
  }): Promise<NotarizationSession>;

  // 공증 완료
  completeNotarization(sessionId: string): Promise<NotarizationResult>;
}

// 공증 서비스 통합 구현
class RemoteNotarizationAdapter implements NotarizationService {
  async requestRON(params: RONParams): Promise<NotarizationSession> {
    const cryoLegal = new CryoLegalClient();
    const document = await cryoLegal.documents.get(params.documentId);

    // 공증 세션 생성
    const session = await this.notaryAPI.createSession({
      documentTitle: document.content.title,
      documentPdf: await cryoLegal.documents.export(params.documentId, 'pdf'),
      signers: params.signerIds.map(id => {
        const party = document.parties.find(p => p.partyId === id);
        return {
          name: party.identity.legalName,
          email: party.contact.email
        };
      }),
      jurisdiction: params.notaryJurisdiction,
      scheduledTime: params.scheduledTime
    });

    return session;
  }
}
```

---

## 4. 마이그레이션 가이드

### 4.1 레거시 시스템에서 마이그레이션

**마이그레이션 단계:**

1. **현재 상태 평가**
   - 기존 법적 문서 목록 작성
   - 문서 유형을 WIA Cryo-Legal 유형에 매핑
   - 관할권 요구사항 식별

2. **데이터 추출**
   ```python
   from wia_cryo_legal import MigrationTool

   migration = MigrationTool(
       source_system='legacy_docs',
       target_system='wia_cryo_legal'
   )

   # 레거시 시스템에서 추출
   legacy_docs = migration.extract_from_source(
       query="SELECT * FROM contracts WHERE type = 'cryopreservation'"
   )

   # WIA 형식으로 변환
   wia_docs = migration.transform(legacy_docs, mapping={
       'contract_id': 'documentId',
       'contract_type': 'documentType',
       'client_name': 'parties[0].identity.legalName',
       'signed_date': 'signatures[0].timestamp'
   })

   # 가져오기 전 검증
   validation = migration.validate(wia_docs)
   print(f"유효: {validation.valid_count}, 무효: {validation.invalid_count}")

   # WIA Cryo-Legal로 가져오기
   results = migration.import_to_target(wia_docs)
   ```

3. **서명 마이그레이션**
   - 가능한 경우 기존 서명 재검증
   - 레거시 서명을 `legacy_imported`로 표시
   - 중요 문서는 재서명 요청

4. **병렬 운영**
   - 전환 기간 동안 두 시스템 병렬 운영
   - 양방향 변경사항 동기화
   - 점진적으로 WIA 시스템으로 트래픽 이동

### 4.2 마이그레이션 체크리스트

| 작업 | 상태 | 비고 |
|------|------|------|
| 문서 목록 작성 완료 | ☐ | 모든 문서 수량 파악 및 분류 |
| 유형 매핑 정의됨 | ☐ | 레거시 유형을 WIA 유형에 매핑 |
| 당사자 데이터 추출됨 | ☐ | 이름, 주소, 연락처 |
| 서명 목록화됨 | ☐ | 서명 유형 및 유효성 기록 |
| 관할권 매핑됨 | ☐ | ISO 국가 코드로 매핑 |
| 테스트 마이그레이션 완료 | ☐ | 샘플 문서로 테스트 |
| 검증 통과 | ☐ | 모든 문서 검증됨 |
| 병렬 동기화 활성화 | ☐ | 양방향 동기화 작동 |
| 사용자 교육 완료 | ☐ | 직원 새 시스템 교육 |
| 가동 승인됨 | ☐ | 이해관계자 서명 |

---

## 5. 배포

### 5.1 클라우드 배포

**AWS 아키텍처:**
```yaml
# CloudFormation 템플릿 발췌
Resources:
  CryoLegalAPI:
    Type: AWS::ECS::Service
    Properties:
      Cluster: !Ref ECSCluster
      TaskDefinition: !Ref CryoLegalTaskDef
      DesiredCount: 3
      LoadBalancers:
        - ContainerName: cryo-legal-api
          ContainerPort: 8080
          TargetGroupArn: !Ref CryoLegalTargetGroup

  CryoLegalDatabase:
    Type: AWS::RDS::DBInstance
    Properties:
      DBInstanceClass: db.r5.large
      Engine: postgres
      EngineVersion: '15'
      StorageEncrypted: true
      MultiAZ: true

  DocumentStorage:
    Type: AWS::S3::Bucket
    Properties:
      BucketName: cryo-legal-documents
      VersioningConfiguration:
        Status: Enabled
      BucketEncryption:
        ServerSideEncryptionConfiguration:
          - ServerSideEncryptionByDefault:
              SSEAlgorithm: aws:kms
```

**Kubernetes 배포:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-legal-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-legal-api
  template:
    metadata:
      labels:
        app: cryo-legal-api
    spec:
      containers:
      - name: api
        image: wia/cryo-legal-api:1.0.0
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: cryo-legal-secrets
              key: database-url
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
```

### 5.2 온프레미스 배포

**Docker Compose:**
```yaml
version: '3.8'
services:
  cryo-legal-api:
    image: wia/cryo-legal-api:1.0.0
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://user:pass@db:5432/cryolegal
      - REDIS_URL=redis://redis:6379
      - STORAGE_PATH=/data/documents
    volumes:
      - document-storage:/data/documents
    depends_on:
      - db
      - redis

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=cryolegal
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass
    volumes:
      - postgres-data:/var/lib/postgresql/data

  redis:
    image: redis:7-alpine
    volumes:
      - redis-data:/data

volumes:
  document-storage:
  postgres-data:
  redis-data:
```

### 5.3 고가용성 구성

```
┌─────────────────────────────────────────────────────────────┐
│                     로드 밸런서                              │
│                    (Active-Active)                          │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│   API 노드   │ │   API 노드   │ │   API 노드   │
│   (존 A)     │ │   (존 B)     │ │   (존 C)     │
└──────┬───────┘ └──────┬───────┘ └──────┬───────┘
       │                │                │
       └────────────────┼────────────────┘
                        │
                        ▼
              ┌──────────────────┐
              │  기본 DB         │
              │  (Multi-AZ)     │
              └────────┬─────────┘
                       │
              ┌────────┴─────────┐
              │  복제본 DB       │
              │  (읽기 전용)     │
              └──────────────────┘
```

---

## 6. 모니터링

### 6.1 메트릭

| 메트릭 | 설명 | 알림 임계값 |
|--------|------|------------|
| `cryo_legal_documents_created` | 분당 생성된 문서 | > 1000/분 |
| `cryo_legal_signatures_verified` | 분당 검증된 서명 | - |
| `cryo_legal_api_latency_p99` | 99번째 백분위 지연 시간 | > 500ms |
| `cryo_legal_api_error_rate` | 오류율 백분율 | > 1% |
| `cryo_legal_db_connections` | 활성 데이터베이스 연결 | > 80% 풀 |
| `cryo_legal_storage_usage` | 문서 저장소 사용률 | > 80% |

### 6.2 Prometheus 구성

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'cryo-legal'
    static_configs:
      - targets: ['cryo-legal-api:9090']
    metrics_path: /metrics
    scrape_interval: 15s

# 알림 규칙
groups:
  - name: cryo-legal-alerts
    rules:
      - alert: HighErrorRate
        expr: rate(cryo_legal_api_errors_total[5m]) > 0.01
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "Cryo-Legal API에서 높은 오류율"

      - alert: HighLatency
        expr: histogram_quantile(0.99, rate(cryo_legal_api_latency_bucket[5m])) > 0.5
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "Cryo-Legal API에서 높은 지연 시간"
```

---

## 7. 인증

### 7.1 WIA 인증 레벨

| 레벨 | 이름 | 요구사항 |
|------|------|----------|
| 1 | 기본 | 1단계 데이터 형식 준수 |
| 2 | 표준 | 1-2단계 준수, 기본 API |
| 3 | 고급 | 전체 1-4단계, 다중 관할권 |
| 4 | 엔터프라이즈 | 고급 + 99.9% SLA, 감사 인증 |

### 7.2 인증 프로세스

```
┌──────────────────────────────────────────────────────────────┐
│                  인증 프로세스                                │
├──────────────────────────────────────────────────────────────┤
│  1. 신청                                                      │
│     └─ 인증 요청 제출                                        │
│                                                               │
│  2. 자체 평가                                                 │
│     └─ 규정 준수 체크리스트 완료                             │
│     └─ 자동화된 검증 스위트 실행                             │
│                                                               │
│  3. 기술 검토                                                 │
│     └─ WIA가 구현 검토                                       │
│     └─ API 규정 준수 검증                                    │
│     └─ 보안 평가                                             │
│                                                               │
│  4. 감사                                                      │
│     └─ 제3자 감사 (레벨 3+)                                  │
│     └─ 침투 테스트                                           │
│                                                               │
│  5. 인증                                                      │
│     └─ 인증서 발급                                           │
│     └─ WIA 레지스트리에 등록                                 │
│                                                               │
│  6. 유지                                                      │
│     └─ 연간 재인증                                           │
│     └─ 사고 보고                                             │
└──────────────────────────────────────────────────────────────┘
```

### 7.3 인증 체크리스트

```markdown
## WIA Cryo-Legal 인증 체크리스트

### 1단계: 데이터 형식
- [ ] 모든 문서가 JSON 스키마를 준수함
- [ ] 필수 필드가 존재하고 유효함
- [ ] 서명이 올바르게 인코딩됨
- [ ] 관할권이 ISO 코드를 사용함

### 2단계: API 인터페이스
- [ ] RESTful 엔드포인트 구현됨
- [ ] 인증 작동함 (OAuth 2.0 / API 키)
- [ ] 요청 제한 적용됨
- [ ] 오류 응답이 명세를 준수함

### 3단계: 프로토콜
- [ ] WebSocket 연결 안정적
- [ ] 메시지 형식 올바름
- [ ] 연결 유지 기능 작동
- [ ] 재연결 작동

### 4단계: 통합
- [ ] WIA 생태계 통합 테스트됨
- [ ] 외부 시스템 어댑터 작동
- [ ] 모니터링 운영 중
- [ ] 문서화 완료
```

---

## 8. 버전 이력

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-01 | 최초 릴리스 |

---

<div align="center">

**WIA Cryo-Legal 표준 v1.0.0**

4단계: 생태계 통합

**弘益人間 (홍익인간)** · 널리 인간을 이롭게

---

© 2025 WIA 표준 위원회

MIT 라이선스

</div>
