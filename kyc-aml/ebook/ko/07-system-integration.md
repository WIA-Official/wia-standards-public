# 7장: 시스템 통합

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

- 세 가지 핵심 통합 아키텍처 패턴 (허브 앤 스포크, 마이크로서비스, API 게이트웨이) 이해
- 코어 뱅킹 시스템과의 양방향 데이터 동기화 구현
- 신원 확인 제공업체 통합 (Jumio, Onfido 등)
- 스크리닝 제공업체 연결 (Dow Jones, Refinitiv 등)
- 거래 모니터링 시스템 통합
- 문서 관리 시스템 설정
- 규제 보고 도구와 통합

---

## 개요

이 장에서는 핵심 은행 시스템, RegTech 플랫폼, 신원 확인 제공업체 및 규정 준수 도구를 포함한 기존 기술 생태계에 WIA KYC/AML 표준을 통합하는 방법에 대한 지침을 제공합니다.

---

## 통합 아키텍처 패턴

### 1. 허브 앤 스포크 모델

#### 아키텍처 다이어그램

```
                    ┌─────────────────┐
                    │   WIA KYC/AML   │
                    │   중앙 허브      │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │  코어   │         │  신원   │         │ 스크리닝│
   │  뱅킹   │         │  확인   │         │  서비스 │
   └─────────┘         └─────────┘         └─────────┘
```

#### 사용 사례
여러 공급업체 서비스가 있는 금융 기관

#### 이점
- 각 시스템에 대한 **단일 통합 지점**
- **중앙 집중식 데이터 관리**
- 간소화된 공급업체 변경
- 일관된 데이터 형식

#### 구현

```typescript
// 허브 구성
const kycHub = new WIAKYCHub({
  coreBanking: new CoreBankingAdapter('bank-system'),
  identityProvider: new IdentityAdapter('jumio'),
  screeningProvider: new ScreeningAdapter('dow-jones'),
  transactionMonitoring: new MonitoringAdapter('actimize')
});

// 통합 워크플로
async function onboardCustomer(application) {
  // 허브에서 고객 생성
  const customer = await kycHub.customers.create(application);

  // 허브가 모든 통합 조율
  const verification = await kycHub.identity.verify(customer.id);
  const screening = await kycHub.screening.perform(customer.id);
  const risk = await kycHub.risk.assess(customer.id);

  // 허브가 코어 뱅킹에 동기화
  await kycHub.sync.toCoreBanking(customer.id);

  return customer;
}
```

---

### 2. 마이크로서비스 아키텍처

#### 아키텍처 다이어그램

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   고객       │    │   확인       │    │  스크리닝    │
│   서비스     │    │   서비스     │    │  서비스      │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                    ┌──────▼───────┐
                    │  이벤트 버스  │
                    │ (Kafka/RabbitMQ)
                    └──────────────┘
```

#### 사용 사례
클라우드 네이티브 금융 기관

#### 이점
- **확장성** - 각 서비스 독립적으로 확장
- **독립적인 배포** - 팀 자율성
- **오류 격리** - 하나의 서비스 실패가 다른 서비스에 영향 안 미침
- **기술 다양성** - 각 서비스에 최적의 기술 스택 선택

#### 구현

```typescript
// 고객 서비스가 이벤트 게시
class CustomerService {
  async createCustomer(data) {
    const customer = await this.db.customers.create(data);

    await this.eventBus.publish('customer.created', {
      customerId: customer.id,
      type: customer.type,
      riskIndicators: this.extractRiskIndicators(customer)
    });

    return customer;
  }
}

// 확인 서비스가 구독
class VerificationService {
  constructor() {
    this.eventBus.subscribe('customer.created', this.handleNewCustomer);
  }

  async handleNewCustomer(event) {
    const verification = await this.performVerification(event.customerId);

    await this.eventBus.publish('verification.completed', {
      customerId: event.customerId,
      result: verification.result,
      confidenceScore: verification.confidenceScore
    });
  }
}

// 스크리닝 서비스가 구독
class ScreeningService {
  constructor() {
    this.eventBus.subscribe('customer.created', this.handleNewCustomer);
    this.eventBus.subscribe('verification.completed', this.checkIfReady);
  }

  async handleNewCustomer(event) {
    await this.performScreening(event.customerId);
  }
}
```

---

### 3. API 게이트웨이 패턴

#### 아키텍처 다이어그램

```
         ┌─────────────────┐
         │  API 게이트웨이  │
         │  (Kong/Apigee)  │
         └────────┬────────┘
                  │
    ┌─────────────┼─────────────┐
    │             │             │
┌───▼───┐   ┌────▼────┐   ┌───▼───┐
│  KYC  │   │   AML   │   │  사례 │
│  API  │   │   API   │   │  관리 │
└───────┘   └─────────┘   └───────┘
```

#### 사용 사례
여러 클라이언트 애플리케이션 (웹, 모바일, 파트너 통합)

#### 이점
- **통합 진입점** - 모든 클라이언트에 대한 단일 엔드포인트
- **인증/권한 부여** - 중앙 집중식 보안
- **속도 제한** - 서비스별 제한 적용
- **요청 라우팅** - 지능형 라우팅 및 로드 밸런싱

---

## 코어 뱅킹 시스템 통합

### 통합 지점

#### 시스템 아키텍처

```
┌─────────────────────────────────────────┐
│        코어 뱅킹 시스템                  │
│                                         │
│  ┌─────────────┐    ┌─────────────┐   │
│  │   고객      │    │   계정      │   │
│  │   마스터    │◄───┤   개설      │   │
│  │             │    │             │   │
│  └──────┬──────┘    └─────────────┘   │
│         │                              │
└─────────┼──────────────────────────────┘
          │
          ▼
   ┌──────────────┐
   │  WIA KYC/AML │
   │   어댑터     │
   └──────┬───────┘
          │
   ┌──────▼───────┐
   │  WIA API     │
   └──────────────┘
```

### 데이터 동기화

#### 양방향 동기화 전략

**코어 뱅킹에서 KYC/AML로:**

```json
{
  "syncType": "customer_update",
  "timestamp": "2025-01-09T10:30:00Z",
  "source": "core_banking",
  "data": {
    "customerId": "CUST-789012",
    "updates": {
      "contactInfo": {
        "phone": {
          "number": "+1-555-9999"
        }
      },
      "addresses": [
        {
          "type": "residential",
          "street": "456 New Address St",
          "city": "Seattle",
          "state": "WA",
          "postalCode": "98101",
          "country": "USA"
        }
      ]
    }
  }
}
```

**KYC/AML에서 코어 뱅킹으로:**

```json
{
  "syncType": "risk_assessment_update",
  "timestamp": "2025-01-09T10:35:00Z",
  "source": "kyc_aml_system",
  "data": {
    "customerId": "CUST-789012",
    "riskProfile": {
      "category": "medium",
      "score": 45,
      "assessmentDate": "2025-01-09T10:30:00Z",
      "nextReviewDate": "2026-01-09"
    },
    "transactionLimits": {
      "dailyWithdrawal": 10000,
      "monthlyInternationalWire": 100000
    },
    "flags": {
      "enhancedMonitoring": true,
      "manualApprovalRequired": false
    }
  }
}
```

### 통합 패턴

#### 1. 실시간 API 통합

```typescript
// 코어 뱅킹이 계정 개설 시 KYC 트리거
class AccountOpeningService {
  async openAccount(customerId: string, accountType: string) {
    // KYC 상태 확인
    const kycStatus = await wiaKYC.customers.get(customerId);

    if (kycStatus.status !== 'verified') {
      throw new Error('KYC verification required');
    }

    if (kycStatus.riskProfile.category === 'high') {
      // 수동 승인 필요
      return this.createPendingAccount(customerId, accountType);
    }

    // 계정 개설 진행
    const account = await this.createAccount(customerId, accountType);

    // 위험 기반 제한 적용
    await this.applyLimits(account.id, kycStatus.riskProfile);

    return account;
  }
}
```

#### 2. 배치 동기화

```typescript
// 야간 배치 동기화
class BatchSyncService {
  async syncDailyUpdates() {
    const date = new Date();

    // 코어 뱅킹에서 업데이트된 고객 가져오기
    const updatedCustomers = await coreBanking.getUpdatedCustomers({
      since: this.getLastSyncTime(),
      until: date
    });

    // KYC 시스템으로 동기화
    for (const customer of updatedCustomers) {
      try {
        await wiaKYC.customers.update(customer.id, customer.changes);
        await this.logSync(customer.id, 'success');
      } catch (error) {
        await this.logSync(customer.id, 'failed', error);
        await this.queueForRetry(customer.id);
      }
    }

    // KYC 시스템에서 위험 업데이트 가져오기
    const riskUpdates = await wiaKYC.risk.getUpdatedAssessments({
      since: this.getLastSyncTime(),
      until: date
    });

    // 코어 뱅킹으로 동기화
    for (const update of riskUpdates) {
      await coreBanking.updateCustomerRisk(update.customerId, update.riskProfile);
      await coreBanking.updateTransactionLimits(update.customerId, update.limits);
    }

    this.setLastSyncTime(date);
  }
}
```

#### 3. 이벤트 기반 통합

```typescript
// 코어 뱅킹이 이벤트 발생, KYC 시스템이 반응
coreBanking.on('customer.address_changed', async (event) => {
  // KYC 시스템에서 주소 업데이트
  await wiaKYC.customers.update(event.customerId, {
    addresses: event.newAddress
  });

  // 중요한 변경이면 재확인 트리거
  if (event.countryChanged) {
    await wiaKYC.identity.reverify(event.customerId);
  }

  // 지리적 변경으로 인한 위험 재평가
  await wiaKYC.risk.reassess(event.customerId);
});

// KYC 시스템이 이벤트 발생, 코어 뱅킹이 반응
wiaKYC.on('risk.category_changed', async (event) => {
  // 코어 뱅킹에서 위험 업데이트
  await coreBanking.updateCustomerRisk(
    event.customerId,
    event.newCategory
  );

  // 새 위험에 따라 제한 조정
  if (event.newCategory === 'high') {
    await coreBanking.reduceTransactionLimits(event.customerId);
  }

  // 관계 관리자 검토를 위한 플래그
  if (event.direction === 'increased') {
    await coreBanking.createReviewTask(event.customerId);
  }
});
```

---

## 신원 확인 제공업체 통합

### 지원되는 제공업체

| 제공업체 | 문서 확인 | 생체 인식 | 데이터베이스 확인 | 지원 국가 |
|---------|----------|----------|------------------|----------|
| **Jumio** | ✅ | ✅ | ✅ | 200+ |
| **Onfido** | ✅ | ✅ | ✅ | 195+ |
| **Trulioo** | ❌ | ❌ | ✅ | 195+ |
| **IDology** | ❌ | ❌ | ✅ | 미국, 캐나다 |
| **LexisNexis** | ❌ | ❌ | ✅ | 전 세계 |
| **Experian** | ❌ | ❌ | ✅ | 95+ |

### 제공업체 어댑터 패턴

```typescript
// 표준 인터페이스
interface IdentityVerificationProvider {
  verifyDocument(customerId: string, document: Document): Promise<VerificationResult>;
  verifyBiometric(customerId: string, biometric: Biometric): Promise<BiometricResult>;
  checkDatabase(customerId: string, data: PersonalData): Promise<DatabaseResult>;
}

// Jumio 어댑터
class JumioAdapter implements IdentityVerificationProvider {
  private client: JumioClient;

  async verifyDocument(customerId: string, document: Document) {
    // Jumio API 호출
    const jumioResult = await this.client.initiateNetverify({
      customerInternalReference: customerId,
      userReference: customerId,
      successUrl: `${this.callbackUrl}/success`,
      errorUrl: `${this.callbackUrl}/error`
    });

    // WIA 형식으로 변환
    return this.transformJumioResult(jumioResult);
  }

  private transformJumioResult(jumioResult: any): VerificationResult {
    return {
      verificationId: jumioResult.transactionReference,
      status: this.mapStatus(jumioResult.status),
      overallResult: jumioResult.verificationStatus === 'APPROVED_VERIFIED' ? 'pass' : 'fail',
      confidenceScore: jumioResult.similarity / 100,
      documentVerification: {
        authenticity: jumioResult.documentVerification.status,
        dataExtraction: jumioResult.extractedData
      }
    };
  }
}

// Onfido 어댑터
class OnfidoAdapter implements IdentityVerificationProvider {
  private client: OnfidoClient;

  async verifyDocument(customerId: string, document: Document) {
    // Onfido API 호출
    const applicant = await this.client.applicant.create({
      first_name: document.firstName,
      last_name: document.lastName,
      email: document.email
    });

    const check = await this.client.check.create({
      applicant_id: applicant.id,
      report_names: ['document', 'facial_similarity_photo']
    });

    // WIA 형식으로 변환
    return this.transformOnfidoResult(check);
  }
}

// 팩토리가 제공업체 선택
class IdentityVerificationFactory {
  static createProvider(providerName: string): IdentityVerificationProvider {
    switch (providerName) {
      case 'jumio':
        return new JumioAdapter(config.jumio);
      case 'onfido':
        return new OnfidoAdapter(config.onfido);
      default:
        throw new Error(`Unknown provider: ${providerName}`);
    }
  }
}

// 사용법
const provider = IdentityVerificationFactory.createProvider('jumio');
const result = await provider.verifyDocument(customerId, document);
```

### 웹훅 처리

```typescript
// 통합 웹훅 핸들러
class IdentityWebhookHandler {
  async handleWebhook(provider: string, payload: any, signature: string) {
    // 웹훅 서명 확인
    if (!this.verifySignature(provider, payload, signature)) {
      throw new Error('Invalid webhook signature');
    }

    // 적절한 핸들러로 라우팅
    switch (provider) {
      case 'jumio':
        return this.handleJumioWebhook(payload);
      case 'onfido':
        return this.handleOnfidoWebhook(payload);
      default:
        throw new Error(`Unknown provider: ${provider}`);
    }
  }

  private async handleJumioWebhook(payload: any) {
    const verificationId = payload.transactionReference;
    const customerId = payload.customerInternalReference;

    // WIA 시스템에서 확인 상태 업데이트
    await wiaKYC.identity.updateVerification(verificationId, {
      status: this.mapJumioStatus(payload.status),
      result: payload.verificationStatus,
      completedAt: new Date(payload.timestamp)
    });

    // 확인 완료되면 다음 단계 트리거
    if (payload.status === 'DONE') {
      await this.triggerNextSteps(customerId, verificationId);
    }
  }

  private async triggerNextSteps(customerId: string, verificationId: string) {
    const verification = await wiaKYC.identity.getVerification(verificationId);

    if (verification.overallResult === 'pass') {
      // 스크리닝 진행
      await wiaKYC.screening.perform(customerId);
      // 위험 평가
      await wiaKYC.risk.assess(customerId);
    } else {
      // 확인 실패 처리
      await wiaKYC.customers.updateStatus(customerId, 'verification_failed');
      await this.notifyCustomer(customerId, 'verification_failed');
    }
  }
}
```

---

## 스크리닝 제공업체 통합

### 제공업체 비교

| 제공업체 | 제재 | PEP | 부정적 미디어 | 범위 | 업데이트 빈도 |
|---------|------|-----|--------------|------|--------------|
| **Dow Jones** | ✅ | ✅ | ✅ | 글로벌 | 실시간 |
| **Refinitiv (World-Check)** | ✅ | ✅ | ✅ | 글로벌 | 일일 |
| **LexisNexis** | ✅ | ✅ | ✅ | 글로벌 | 일일 |
| **ComplyAdvantage** | ✅ | ✅ | ✅ | 글로벌 | 실시간 |

### 스크리닝 어댑터

```typescript
interface ScreeningProvider {
  screen(customer: Customer, options: ScreeningOptions): Promise<ScreeningResult>;
  getWatchlists(): Promise<Watchlist[]>;
  enableContinuousMonitoring(customerId: string): Promise<void>;
}

class DowJonesAdapter implements ScreeningProvider {
  async screen(customer: Customer, options: ScreeningOptions) {
    const searchRequest = {
      searchTerm: customer.personalInfo.fullName,
      dateOfBirth: customer.personalInfo.dateOfBirth,
      country: customer.personalInfo.nationality[0]
    };

    const response = await this.client.search(searchRequest);

    return this.transformResults(response);
  }

  private transformResults(djResponse: any): ScreeningResult {
    return {
      screeningId: djResponse.caseId,
      executedAt: new Date(),
      sanctionsScreening: {
        status: this.hasMatches(djResponse, 'SANCTION') ? 'match' : 'no_match',
        matches: this.extractMatches(djResponse, 'SANCTION')
      },
      pepScreening: {
        status: this.hasMatches(djResponse, 'PEP') ? 'match' : 'no_match',
        matches: this.extractMatches(djResponse, 'PEP')
      },
      adverseMediaScreening: {
        status: this.hasMatches(djResponse, 'ADVERSE_MEDIA') ? 'match' : 'no_match',
        articles: this.extractAdverseMedia(djResponse)
      }
    };
  }
}

// 다중 제공업체 스크리닝 (중복성 또는 범위용)
class MultiProviderScreening {
  private providers: ScreeningProvider[];

  async screenWithConsensus(customer: Customer) {
    // 모든 제공업체와 병렬로 스크리닝
    const results = await Promise.all(
      this.providers.map(p => p.screen(customer, {}))
    );

    // 결과 집계
    return this.aggregateResults(results);
  }

  private aggregateResults(results: ScreeningResult[]): ScreeningResult {
    // 모든 제공업체의 일치 항목 결합
    const allMatches = results.flatMap(r => r.sanctionsScreening.matches || []);

    // 중복 제거
    const uniqueMatches = this.deduplicateMatches(allMatches);

    // 통합 결과 반환
    return {
      screeningId: `MULTI-${Date.now()}`,
      executedAt: new Date(),
      sanctionsScreening: {
        status: uniqueMatches.length > 0 ? 'match' : 'no_match',
        matches: uniqueMatches
      }
    };
  }
}
```

---

## 거래 모니터링 시스템 통합

### 기존 TMS와의 통합

```typescript
// 기존 TMS용 어댑터 (예: Actimize, FICO, SAS)
class TransactionMonitoringAdapter {
  // 코어 뱅킹에서 TMS로 거래 푸시
  async sendTransaction(transaction: Transaction) {
    // WIA 형식으로 변환
    const wiaTransaction = this.transformToWIA(transaction);

    // WIA TM 시스템으로 전송
    await wiaKYC.monitoring.submitTransaction(wiaTransaction);

    // 전환 기간 동안 레거시 TMS로도 전송
    if (this.isTransitionMode()) {
      await this.legacyTMS.sendTransaction(transaction);
    }
  }

  // TMS에서 경보 수신
  async handleAlert(alert: any) {
    // WIA 형식으로 경보 변환
    const wiaAlert = this.transformAlertToWIA(alert);

    // WIA 시스템에서 사례 생성
    const case = await wiaKYC.cases.create({
      caseType: 'suspicious_activity_investigation',
      customerId: wiaAlert.customerId,
      trigger: {
        type: 'transaction_monitoring_alert',
        sourceId: wiaAlert.alertId
      }
    });

    return case;
  }

  // 처분을 TMS로 동기화
  async syncDisposition(caseId: string, disposition: string) {
    const case = await wiaKYC.cases.get(caseId);

    // 레거시 TMS 업데이트
    await this.legacyTMS.updateAlertStatus(
      case.trigger.sourceId,
      this.mapDispositionToTMS(disposition)
    );
  }
}
```

---

## 문서 관리 통합

### 문서 저장 옵션

#### 1. 클라우드 저장소 (AWS S3, Azure Blob, GCP Storage)

```typescript
class CloudDocumentStorage {
  private s3: S3Client;

  async storeDocument(customerId: string, document: File): Promise<string> {
    // 문서 암호화
    const encrypted = await this.encrypt(document);

    // 보안 파일 이름 생성
    const fileName = `${customerId}/${document.type}/${uuidv4()}.enc`;

    // S3에 업로드
    await this.s3.putObject({
      Bucket: 'kyc-documents',
      Key: fileName,
      Body: encrypted,
      ServerSideEncryption: 'AES256',
      Metadata: {
        customerId,
        documentType: document.type,
        uploadedAt: new Date().toISOString()
      }
    });

    // 보안 URL 반환
    return `s3://kyc-documents/${fileName}`;
  }

  async retrieveDocument(documentUrl: string): Promise<Buffer> {
    // S3 URL 파싱
    const { bucket, key } = this.parseS3Url(documentUrl);

    // S3에서 다운로드
    const response = await this.s3.getObject({
      Bucket: bucket,
      Key: key
    });

    const encrypted = await response.Body.transformToByteArray();

    // 복호화
    return this.decrypt(encrypted);
  }
}
```

#### 2. DMS 통합 (FileNet, Documentum, SharePoint)

```typescript
class DMSAdapter {
  async storeDocument(customerId: string, document: File): Promise<string> {
    // 폴더 구조가 없으면 생성
    await this.ensureFolder(`/Customers/${customerId}/KYC`);

    // DMS에 저장
    const docId = await this.dmsClient.createDocument({
      folder: `/Customers/${customerId}/KYC`,
      fileName: `${document.type}_${Date.now()}.pdf`,
      content: document.buffer,
      metadata: {
        customerId,
        documentType: document.type,
        uploadedAt: new Date(),
        classification: 'confidential'
      }
    });

    return docId;
  }
}
```

---

## 감사 및 규정 준수 도구 통합

### 감사 추적 내보내기

```typescript
class AuditTrailExporter {
  async exportForRegulator(dateRange: DateRange, format: 'csv' | 'xlsx' | 'json') {
    // 모든 관련 활동 가져오기
    const activities = await wiaKYC.audit.getActivities({
      from: dateRange.start,
      to: dateRange.end,
      includeTypes: [
        'customer.created',
        'verification.completed',
        'screening.performed',
        'risk.assessed',
        'alert.generated',
        'case.created',
        'sar.filed'
      ]
    });

    // 내보내기 형식 지정
    const formatted = activities.map(activity => ({
      timestamp: activity.timestamp,
      activityType: activity.type,
      customerId: activity.customerId,
      performedBy: activity.performedBy,
      details: JSON.stringify(activity.details),
      ipAddress: activity.ipAddress
    }));

    // 요청된 형식으로 내보내기
    switch (format) {
      case 'csv':
        return this.convertToCSV(formatted);
      case 'xlsx':
        return this.convertToExcel(formatted);
      case 'json':
        return JSON.stringify(formatted, null, 2);
    }
  }
}
```

### 규제 보고 통합

```typescript
class RegulatoryReportingService {
  // FinCEN 형식으로 SAR 생성
  async generateFinCENSAR(caseId: string): Promise<string> {
    const case = await wiaKYC.cases.get(caseId);
    const sar = await wiaKYC.cases.getSAR(caseId);

    // FinCEN XML 형식으로 변환
    const fincenXML = this.transformToFinCENFormat(sar);

    // FinCEN 스키마에 대해 검증
    await this.validateFinCENXML(fincenXML);

    // FinCEN BSA E-Filing 시스템에 제출
    const confirmationNumber = await this.submitToFinCEN(fincenXML);

    // 확인으로 사례 업데이트
    await wiaKYC.cases.update(caseId, {
      'resolution.sarConfirmation': confirmationNumber
    });

    return confirmationNumber;
  }

  // CTR 보고서 생성
  async generateCTR(transactions: Transaction[]): Promise<CTRReport> {
    // 거래 집계
    const totalAmount = transactions.reduce((sum, t) => sum + t.amount, 0);

    // CTR 생성
    const ctr = {
      reportDate: new Date(),
      transactions,
      totalAmount,
      customer: await this.getCustomerInfo(transactions[0].customerId),
      institution: this.getInstitutionInfo()
    };

    // 규제 기관에 제출
    await this.submitCTR(ctr);

    return ctr;
  }
}
```

---

## 복습 질문

1. 세 가지 주요 통합 아키텍처 패턴은 무엇이며, 각각 언제 사용합니까?
2. 코어 뱅킹 시스템과 KYC/AML 시스템 간의 양방향 동기화는 어떻게 작동합니까?
3. 신원 확인 제공업체 어댑터 패턴의 목적은 무엇입니까?
4. 다중 스크리닝 제공업체를 사용하는 이점은 무엇입니까?
5. 웹훅 서명 검증이 중요한 이유는 무엇입니까?
6. 실시간, 배치, 이벤트 기반 통합 패턴의 차이점은 무엇입니까?
7. 클라우드 저장소와 DMS 통합 중 어떻게 선택합니까?
8. 감사 추적 내보내기에서 어떤 활동 유형을 캡처해야 합니까?
9. FinCEN SAR 제출 프로세스를 설명하십시오.
10. 통합 구현 시 어떤 보안 고려 사항이 중요합니까?

---

## 주요 요점

1. 🏗️ **여러 아키텍처 패턴** - 허브 앤 스포크, 마이크로서비스, API 게이트웨이
2. 🔄 **양방향 동기화** - 코어 뱅킹 시스템과의 실시간 데이터 동기화
3. 🔌 **제공업체 어댑터** - 신원 확인 및 스크리닝을 위한 표준화된 인터페이스
4. 📊 **거래 모니터링** - 레거시 및 신규 시스템 통합 패턴
5. 📁 **문서 저장** - 클라우드 및 DMS 옵션
6. 📋 **감사 및 보고** - 규제 기관용 내보내기 기능
7. 🌐 **이벤트 기반** - 실시간 통합을 위한 비동기 통신

---

**이전**: [← 6장 - 프로토콜](06-protocol.md) | **다음**: [8장 - 구현 →](08-implementation.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
