# 8장: 구현 가이드

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

- 6개월 구현 로드맵의 6단계 이해
- 현재 상태 평가 및 격차 분석 수행
- 프로젝트 팀 구성 및 역할 할당
- 핵심 KYC/AML 구성 요소 구현
- 데이터 마이그레이션 전략 실행
- 테스트 및 사용자 인수 프로토콜 수행
- 프로덕션 배포 및 하이퍼케어 관리
- 성공 지표 및 KPI 모니터링

---

## 개요

이 장에서는 계획, 배포 전략, 워크플로 자동화 및 성공 지표를 포함하여 조직에서 WIA KYC/AML 표준을 구현하기 위한 실용적인 지침을 제공합니다.

---

## 구현 로드맵

### 1단계: 계획 및 평가 (1-4주)

#### 1-2주: 현재 상태 평가

**활동:**

```
☐ 기존 KYC/AML 프로세스 문서화
   - 현재 온보딩 워크플로 매핑
   - 수동 대 자동화 단계 식별
   - 평균 처리 시간 측정
   - 병목 현상 기록

☐ 현재 기술 스택 인벤토리
   - 코어 뱅킹 시스템
   - 신원 확인 도구
   - 스크리닝 제공업체
   - 문서 관리 시스템
   - 보고 도구

☐ 통합 지점 식별
   - API 가용성
   - 데이터 형식
   - 인증 메커니즘
   - 동기화 요구 사항

☐ 데이터 흐름 매핑
   - 고객 데이터 소스
   - 거래 데이터 흐름
   - 보고 출력

☐ 규제 요구 사항 검토
   - 관할권별 규정
   - 현재 규정 준수 수준
   - 감사 발견 사항

☐ WIA 표준 대 격차 평가
   - 누락된 구성 요소
   - 미흡한 프로세스
   - 기술 부채

☐ 문제점 및 우선순위 식별
   - 비즈니스 영향 평가
   - 기술 실현 가능성
   - 자원 요구 사항
```

**평가 템플릿:**

| 영역 | 현재 상태 | WIA 표준 | 격차 | 우선순위 | 노력 |
|------|----------|---------|------|---------|------|
| 고객 온보딩 | 수동, 48시간 | 자동화, <4시간 | 자동화 필요 | 높음 | 높음 |
| 신원 확인 | 제3자 | WIA 통합 | 표준화 필요 | 중간 | 중간 |
| 스크리닝 | 수동, 주간 | 자동화, 실시간 | 자동화 필요 | 높음 | 중간 |
| 위험 평가 | 스프레드시트 | 자동 채점 | 시스템 필요 | 높음 | 높음 |
| 거래 모니터링 | 레거시 TMS | WIA 통합 | 통합 필요 | 중간 | 높음 |
| 사례 관리 | 이메일 기반 | 전용 시스템 | 플랫폼 필요 | 중간 | 중간 |
| 보고 | 수동 | 자동 감사 추적 | 자동화 필요 | 높음 | 낮음 |

#### 3-4주: 계획 및 설계

**활동:**

```
☐ 대상 아키텍처 정의
   - 아키텍처 패턴 선택 (허브 앤 스포크/마이크로서비스/API 게이트웨이)
   - 구성 요소 다이어그램
   - 통합 아키텍처
   - 데이터 플로우 다이어그램

☐ 배포 모델 선택 (클라우드/온프레미스/하이브리드)
   - 보안 요구 사항
   - 데이터 상주
   - 비용 고려 사항
   - 확장성 요구 사항

☐ 구현 접근 방식 선택 (단계적/빅뱅)
   - 위험 평가
   - 비즈니스 중단 분석
   - 롤백 전략

☐ 데이터 마이그레이션 전략 설계
   - 마이그레이션할 데이터 식별
   - 정리 요구 사항
   - 매핑 규칙
   - 검증 기준

☐ 통합 접근 방식 계획
   - API 대 배치
   - 실시간 대 예약됨
   - 오류 처리
   - 재시도 논리

☐ 성공 지표 정의
   - 운영 KPI
   - 비즈니스 메트릭
   - 규정 준수 메트릭

☐ 프로젝트 계획 및 일정 생성
   - 작업 분해 구조
   - 의존성
   - 중요 경로
   - 마일스톤

☐ 구현 팀 구성
   - 역할 및 책임
   - 리소스 할당
   - 공급업체 참여
```

**팀 구조:**

```
프로젝트 스폰서 (CRO/CTO)
    │
    ├── 프로젝트 관리자
    │   ├── 일정 관리
    │   ├── 위험 관리
    │   └── 이해관계자 소통
    │
    ├── 비즈니스 스트림 (규정 준수 리드)
    │   ├── 규정 준수 분석가 (2명)
    │   ├── 비즈니스 분석가 (2-3명)
    │   └── 프로세스 설계자
    │
    ├── 기술 스트림 (솔루션 아키텍트)
    │   ├── 통합 리드
    │   ├── 백엔드 개발자 (3-4명)
    │   ├── 프론트엔드 개발자 (2명)
    │   ├── DevOps 엔지니어 (1-2명)
    │   └── QA 엔지니어 (2-3명)
    │
    └── 공급업체 관리
        ├── 신원 확인 제공업체
        ├── 스크리닝 제공업체
        └── 인프라 제공업체
```

---

### 2단계: 기반 구축 (5-12주)

#### 5-6주: 환경 설정

**인프라:**

```
☐ 클라우드 인프라 프로비저닝
   AWS 예제:
   - VPC 및 서브넷 생성
   - EKS 클러스터 (Kubernetes)
   - RDS 인스턴스 (PostgreSQL)
   - ElastiCache (Redis)
   - S3 버킷 (문서 저장소)
   - CloudFront (CDN)

☐ 개발 환경 설정
   - Git 저장소 (GitHub/GitLab)
   - 개발 서버
   - 개발 데이터베이스
   - API 게이트웨이

☐ 테스트 환경 설정
   - 테스트 서버
   - 테스트 데이터베이스
   - 제공업체 샌드박스

☐ CI/CD 파이프라인 구성
   - 빌드 파이프라인
   - 테스트 자동화
   - 배포 자동화
   - 롤백 메커니즘

☐ 모니터링 및 로깅 설정
   - 애플리케이션 모니터링 (Datadog/New Relic)
   - 로그 집계 (ELK Stack)
   - 경보 구성
   - 대시보드

☐ 보안 제어 구현
   - 네트워크 보안 그룹
   - WAF 규칙
   - 비밀 관리 (AWS Secrets Manager)
   - 암호화 (저장 중 및 전송 중)

☐ 백업 및 재해 복구 구성
   - 데이터베이스 백업
   - 재해 복구 사이트
   - 복구 절차
   - 정기 테스트
```

**인프라 as 코드 예제 (Terraform):**

```hcl
# VPC 및 네트워킹
resource "aws_vpc" "kyc_vpc" {
  cidr_block = "10.0.0.0/16"

  tags = {
    Name = "kyc-aml-vpc"
    Environment = "production"
  }
}

# EKS 클러스터
resource "aws_eks_cluster" "kyc_cluster" {
  name     = "kyc-aml-cluster"
  role_arn = aws_iam_role.eks_cluster.arn
  version  = "1.27"

  vpc_config {
    subnet_ids = aws_subnet.private[*].id
  }
}

# RDS PostgreSQL
resource "aws_db_instance" "kyc_db" {
  identifier           = "kyc-aml-db"
  engine              = "postgres"
  engine_version      = "15.3"
  instance_class      = "db.r6g.xlarge"
  allocated_storage   = 100
  storage_encrypted   = true
  multi_az           = true

  backup_retention_period = 30
  backup_window          = "03:00-04:00"
}
```

#### 7-9주: 핵심 구현

**고객 관리:**

```typescript
// 고객 프로필 관리 구현
class CustomerService {
  async create(customerData: CustomerInput): Promise<Customer> {
    // 1. 입력 검증
    await this.validate(customerData);

    // 2. WIA 형식으로 고객 생성
    const customer = await this.repository.create({
      ...customerData,
      customerId: this.generateCustomerId(),
      schemaVersion: '1.0',
      createdAt: new Date(),
      status: 'pending_verification',
      auditInfo: {
        createdBy: this.context.userId,
        createdAt: new Date()
      }
    });

    // 3. 다운스트림 처리를 위한 이벤트 발생
    await this.eventBus.publish('customer.created', {
      customerId: customer.customerId,
      type: customer.type,
      riskIndicators: this.extractRiskIndicators(customer)
    });

    // 4. 감사 로그
    await this.auditLog.log({
      action: 'customer.created',
      customerId: customer.customerId,
      performedBy: this.context.userId
    });

    return customer;
  }

  async update(customerId: string, updates: Partial<Customer>): Promise<Customer> {
    // 기존 고객 가져오기
    const existing = await this.repository.findById(customerId);

    if (!existing) {
      throw new Error(`Customer ${customerId} not found`);
    }

    // 검증
    await this.validateUpdates(existing, updates);

    // 업데이트 적용
    const updated = await this.repository.update(customerId, {
      ...updates,
      auditInfo: {
        ...existing.auditInfo,
        updatedBy: this.context.userId,
        updatedAt: new Date()
      }
    });

    // 중요한 변경이면 이벤트 발생
    if (this.isCriticalChange(updates)) {
      await this.eventBus.publish('customer.updated', {
        customerId,
        changes: updates
      });
    }

    return updated;
  }

  private isCriticalChange(updates: Partial<Customer>): boolean {
    const criticalFields = ['addresses', 'nationality', 'occupation'];
    return Object.keys(updates).some(key => criticalFields.includes(key));
  }
}
```

**신원 확인:**

```typescript
class IdentityVerificationService {
  async initiateVerification(
    customerId: string,
    verificationType: VerificationType
  ): Promise<Verification> {
    // 고객 가져오기
    const customer = await this.customerService.get(customerId);

    // 확인 생성
    const verification = await this.repository.create({
      verificationId: this.generateVerificationId(),
      customerId,
      verificationType,
      status: 'pending',
      createdAt: new Date()
    });

    // 적절한 제공업체 선택
    const provider = this.selectProvider(customer, verificationType);

    // 제공업체로 확인 시작
    const session = await provider.initiateSession({
      customerId,
      returnUrl: this.config.returnUrl,
      webhookUrl: this.config.webhookUrl
    });

    // 세션 정보로 확인 업데이트
    await this.repository.update(verification.verificationId, {
      sessionId: session.id,
      sessionUrl: session.url,
      expiresAt: session.expiresAt
    });

    return verification;
  }

  async processWebhook(provider: string, payload: any): Promise<void> {
    // 서명 확인
    if (!this.verifyWebhookSignature(provider, payload)) {
      throw new Error('Invalid webhook signature');
    }

    // 확인 업데이트
    const verification = await this.repository.findBySessionId(payload.sessionId);

    await this.repository.update(verification.verificationId, {
      status: this.mapStatus(payload.status),
      result: this.transformResult(payload),
      completedAt: new Date()
    });

    // 확인 완료되면 다음 단계 트리거
    if (payload.status === 'completed') {
      await this.triggerNextSteps(verification.customerId);
    }
  }

  private async triggerNextSteps(customerId: string): Promise<void> {
    // 스크리닝 시작
    await this.screeningService.performComprehensiveScreening(customerId);

    // 위험 평가
    await this.riskService.assessCustomer(customerId);
  }
}
```

#### 10-12주: 스크리닝 및 위험 평가

**스크리닝 구현:**

```typescript
class ScreeningService {
  async performComprehensiveScreening(
    customerId: string
  ): Promise<ScreeningResult> {
    const customer = await this.customerService.get(customerId);

    // 병렬 스크리닝
    const [sanctions, pep, adverseMedia] = await Promise.all([
      this.screenSanctions(customer),
      this.screenPEP(customer),
      this.screenAdverseMedia(customer)
    ]);

    // 결과 집계
    const result: ScreeningResult = {
      screeningId: this.generateScreeningId(),
      customerId,
      executedAt: new Date(),
      sanctionsScreening: sanctions,
      pepScreening: pep,
      adverseMediaScreening: adverseMedia,
      overallRisk: this.calculateOverallRisk(sanctions, pep, adverseMedia)
    };

    // 저장
    await this.repository.save(result);

    // 일치 항목 처리
    if (this.hasMatches(result)) {
      await this.handleMatches(result);
    }

    // 지속적인 모니터링 활성화
    await this.enableContinuousMonitoring(customerId);

    return result;
  }

  private async screenSanctions(customer: Customer): Promise<SanctionsResult> {
    const searchData = this.extractSearchData(customer);

    // 제공업체 API 호출
    const providerResult = await this.screeningProvider.searchSanctions(searchData);

    // WIA 형식으로 변환
    return {
      status: providerResult.matches.length > 0 ? 'match' : 'no_match',
      listsChecked: providerResult.listsSearched,
      totalRecordsChecked: providerResult.recordCount,
      matches: providerResult.matches.map(m => ({
        matchId: m.id,
        confidence: m.score,
        profile: m.entity,
        source: m.list
      })),
      lastUpdated: new Date()
    };
  }

  private async handleMatches(result: ScreeningResult): Promise<void> {
    // 고위험 일치의 경우 사례 생성
    const highRiskMatches = this.getHighRiskMatches(result);

    if (highRiskMatches.length > 0) {
      await this.caseService.create({
        caseType: 'screening_match_review',
        customerId: result.customerId,
        priority: 'high',
        trigger: {
          type: 'screening_match',
          screeningId: result.screeningId,
          matches: highRiskMatches
        }
      });
    }

    // 고객 상태 업데이트
    await this.customerService.updateStatus(
      result.customerId,
      'screening_review_required'
    );
  }
}
```

**위험 평가 구현:**

```typescript
class RiskAssessmentService {
  async assessCustomer(customerId: string): Promise<RiskAssessment> {
    const customer = await this.customerService.get(customerId);
    const screening = await this.screeningService.getLatest(customerId);

    // 다차원 위험 평가
    const dimensions = await Promise.all([
      this.assessGeographicRisk(customer),
      this.assessProductRisk(customer),
      this.assessCustomerTypeRisk(customer),
      this.assessBehavioralRisk(customer),
      this.assessRelationshipRisk(customer, screening)
    ]);

    // 전체 점수 계산
    const overallScore = this.calculateWeightedScore(dimensions);
    const category = this.categorizeRisk(overallScore);

    const assessment: RiskAssessment = {
      assessmentId: this.generateAssessmentId(),
      customerId,
      assessmentDate: new Date(),
      overallRisk: {
        score: overallScore,
        category
      },
      riskDimensions: dimensions,
      recommendations: this.generateRecommendations(category, dimensions)
    };

    // 저장
    await this.repository.save(assessment);

    // 이벤트 발생
    await this.eventBus.publish('risk.assessed', {
      customerId,
      category,
      score: overallScore
    });

    return assessment;
  }

  private calculateWeightedScore(dimensions: RiskDimension[]): number {
    const weights = {
      geographic: 0.25,
      product: 0.20,
      customerType: 0.20,
      behavioral: 0.20,
      relationship: 0.15
    };

    return dimensions.reduce((total, dim) => {
      return total + (dim.score * weights[dim.dimension]);
    }, 0);
  }

  private categorizeRisk(score: number): RiskCategory {
    if (score < 40) return 'low';
    if (score < 70) return 'medium';
    if (score < 90) return 'high';
    return 'prohibited';
  }

  private generateRecommendations(
    category: RiskCategory,
    dimensions: RiskDimension[]
  ): Recommendations {
    const baseRecommendations = {
      low: {
        approvalDecision: 'approve',
        cddLevel: 'standard',
        reviewFrequency: 'biennial'
      },
      medium: {
        approvalDecision: 'approve',
        cddLevel: 'standard',
        reviewFrequency: 'annual'
      },
      high: {
        approvalDecision: 'senior_approval_required',
        cddLevel: 'enhanced',
        reviewFrequency: 'quarterly'
      },
      prohibited: {
        approvalDecision: 'reject',
        cddLevel: 'n/a',
        reviewFrequency: 'n/a'
      }
    };

    return {
      ...baseRecommendations[category],
      transactionLimits: this.calculateLimits(category),
      additionalActions: this.getAdditionalActions(dimensions)
    };
  }
}
```

---

### 3단계: 통합 및 테스트 (13-18주)

#### 13-14주: 시스템 통합

**코어 뱅킹 통합:**

```typescript
// 양방향 동기화 구현
class CoreBankingIntegration {
  async syncCustomerFromCore(coreBankingCustomer: any): Promise<void> {
    // 코어 뱅킹 형식을 WIA 형식으로 변환
    const wiaCustomer = await this.transformer.toWIA(coreBankingCustomer);

    // 고객이 이미 존재하는지 확인
    const existing = await this.customerService.findByExternalId(
      coreBankingCustomer.id
    );

    if (existing) {
      // 업데이트
      await this.customerService.update(existing.customerId, wiaCustomer);
    } else {
      // 생성
      await this.customerService.create({
        ...wiaCustomer,
        externalId: coreBankingCustomer.id
      });
    }
  }

  async syncRiskToCore(customerId: string): Promise<void> {
    // WIA 시스템에서 위험 평가 가져오기
    const riskAssessment = await this.riskService.getLatest(customerId);
    const customer = await this.customerService.get(customerId);

    // 코어 뱅킹 형식으로 변환
    const coreBankingUpdate = {
      customerId: customer.externalId,
      riskCategory: riskAssessment.overallRisk.category,
      riskScore: riskAssessment.overallRisk.score,
      reviewDate: riskAssessment.recommendations.nextReviewDate,
      limits: riskAssessment.recommendations.transactionLimits
    };

    // 코어 뱅킹 시스템 업데이트
    await this.coreBankingAPI.updateCustomerRisk(coreBankingUpdate);
  }

  // 실시간 이벤트 리스너 설정
  setupEventListeners(): void {
    // WIA 이벤트 수신 → 코어 뱅킹 업데이트
    this.eventBus.on('risk.assessed', async (event) => {
      await this.syncRiskToCore(event.customerId);
    });

    this.eventBus.on('customer.status_changed', async (event) => {
      await this.syncStatusToCore(event.customerId, event.newStatus);
    });

    // 코어 뱅킹 이벤트 수신 → WIA 업데이트
    this.coreBankingEvents.on('customer.updated', async (event) => {
      await this.syncCustomerFromCore(event.customer);
    });
  }
}
```

#### 15-16주: 테스트

**테스트 전략:**

```
1. 단위 테스트 (최소 70% 커버리지)
   ├── 고객 서비스 테스트
   │   ├── 생성, 업데이트, 조회 기능
   │   ├── 검증 논리
   │   └── 오류 처리
   ├── 확인 서비스 테스트
   ├── 스크리닝 서비스 테스트
   └── 위험 평가 테스트

2. 통합 테스트
   ├── API 엔드포인트 테스트
   ├── 데이터베이스 통합 테스트
   └── 외부 제공업체 통합 테스트 (샌드박스)

3. 엔드투엔드 테스트
   ├── 완전한 온보딩 흐름
   ├── 확인 워크플로
   └── 스크리닝 및 위험 평가 흐름

4. 성능 테스트
   ├── 부하 테스트 (1000명 동시 사용자)
   ├── 스트레스 테스트 (피크 부하 + 50%)
   └── 내구성 테스트 (지속 부하, 24시간)
```

**테스트 케이스 예제:**

```typescript
// 단위 테스트
describe('CustomerService', () => {
  describe('create', () => {
    it('should create a valid customer', async () => {
      const input: CustomerInput = {
        type: 'individual',
        personalInfo: {
          firstName: 'John',
          lastName: 'Smith',
          dateOfBirth: '1985-06-15'
        }
      };

      const customer = await customerService.create(input);

      expect(customer.customerId).toBeDefined();
      expect(customer.status).toBe('pending_verification');
      expect(customer.personalInfo.firstName).toBe('John');
    });

    it('should reject invalid date of birth', async () => {
      const input: CustomerInput = {
        type: 'individual',
        personalInfo: {
          firstName: 'John',
          lastName: 'Smith',
          dateOfBirth: 'invalid-date'
        }
      };

      await expect(customerService.create(input))
        .rejects
        .toThrow('Invalid date format');
    });
  });
});

// 통합 테스트
describe('Onboarding Workflow Integration', () => {
  it('should complete full onboarding for low-risk customer', async () => {
    // 1. 고객 생성
    const customer = await api.post('/customers', lowRiskCustomerData);
    expect(customer.status).toBe(201);

    // 2. 확인 시작
    const verification = await api.post('/identity/verifications', {
      customerId: customer.body.data.customerId,
      verificationType: 'document_and_biometric'
    });
    expect(verification.status).toBe(201);

    // 3. 문서 업로드 (샌드박스 모드)
    await api.post(
      `/identity/verifications/${verification.body.data.verificationId}/documents`,
      testDocumentData
    );

    // 4. 완료 대기
    await waitFor(() => verification.status === 'verified', { timeout: 30000 });

    // 5. 스크리닝 확인
    const screening = await api.get(`/screening/${customer.body.data.customerId}`);
    expect(screening.body.data.overallRisk).toBe('low_risk');

    // 6. 위험 평가 확인
    const risk = await api.get(`/risk/score/${customer.body.data.customerId}`);
    expect(risk.body.data.riskCategory).toBe('low');

    // 7. 최종 고객 상태 확인
    const finalCustomer = await api.get(`/customers/${customer.body.data.customerId}`);
    expect(finalCustomer.body.data.status).toBe('active');
  });
});

// 성능 테스트
describe('Performance Tests', () => {
  it('should handle 1000 concurrent customer creations', async () => {
    const startTime = Date.now();

    const promises = Array.from({ length: 1000 }, (_, i) =>
      api.post('/customers', generateCustomerData(i))
    );

    const results = await Promise.all(promises);
    const endTime = Date.now();

    const successCount = results.filter(r => r.status === 201).length;
    const duration = endTime - startTime;
    const throughput = (successCount / duration) * 1000;

    expect(successCount).toBeGreaterThan(950); // 95% 성공률
    expect(throughput).toBeGreaterThan(100); // 초당 100개 이상
  });
});
```

#### 17-18주: 사용자 인수 테스트

**UAT 활동:**

```
☐ 규정 준수 팀이 워크플로 검증
   - 온보딩 프로세스
   - 스크리닝 정확도
   - 위험 평가 논리
   - 사례 조사 워크플로
   - SAR 생성

☐ 운영 팀이 일상 프로세스 테스트
   - 경보 처리
   - 고객 검토
   - 보고서 생성

☐ IT 팀이 통합 검증
   - 코어 뱅킹 동기화
   - 제공업체 통합
   - 데이터 흐름

☐ 비즈니스 사용자가 고객 대면 흐름 테스트
   - 온보딩 UX
   - 문서 업로드
   - 상태 추적

☐ 피드백 및 문제 수집
☐ 우선순위 지정 및 문제 수정
☐ 중요한 경로 재테스트
☐ UAT 승인 획득
```

---

### 4단계: 데이터 마이그레이션 (19-22주)

#### 19-20주: 마이그레이션 계획

**마이그레이션 전략:**

```
1. 기존 데이터 분석
   ☐ 고객 수: _______
   ☐ 확인 기록: _______
   ☐ 스크리닝 기록: _______
   ☐ 거래 이력: _______
   ☐ 문서: _______

2. 데이터 품질 평가
   ☐ 중복 식별
   ☐ 불완전한 기록
   ☐ 형식 일관성 없음
   ☐ 누락된 필수 필드

3. 정리 계획
   ☐ 중복 제거
   ☐ 형식 표준화
   ☐ 누락된 데이터 채우기
   ☐ 유효성 검사

4. 매핑 문서
   ☐ 필드 매핑 생성
   ☐ 변환 규칙 정의
   ☐ 기본값 정의
   ☐ 예외 처리

5. 마이그레이션 접근 방식
   ☐ 빅뱅 vs. 단계적 결정
   ☐ 전환 날짜 선택
   ☐ 롤백 계획 준비
   ☐ 검증 기준 정의
```

**매핑 템플릿:**

| 레거시 필드 | WIA 필드 | 변환 규칙 | 기본값 | 검증 |
|------------|---------|----------|--------|------|
| CUST_NAME | personalInfo.firstName + lastName | 공백으로 분할 | N/A | 비어 있지 않음 |
| DOB | personalInfo.dateOfBirth | YYYY-MM-DD로 변환 | N/A | 유효한 날짜 |
| ADDR | addresses[0].street | 전체 주소 파싱 | N/A | 완전한 주소 |

#### 21-22주: 마이그레이션 실행

**마이그레이션 스크립트:**

```typescript
class DataMigration {
  async migrateCustomers(): Promise<MigrationResult> {
    const batchSize = 1000;
    let offset = 0;
    let totalMigrated = 0;
    let totalFailed = 0;
    const errors: MigrationError[] = [];

    console.log('Starting customer migration...');

    while (true) {
      // 레거시 시스템에서 배치 가져오기
      const batch = await this.legacyDB.query(`
        SELECT * FROM customers
        WHERE migration_status IS NULL
        LIMIT ${batchSize} OFFSET ${offset}
      `);

      if (batch.length === 0) break;

      console.log(`Processing batch ${offset / batchSize + 1} (${batch.length} records)`);

      // 각 고객 처리
      for (const legacyCustomer of batch) {
        try {
          // WIA 형식으로 변환
          const wiaCustomer = await this.transformer.transform(legacyCustomer);

          // 검증
          await this.validator.validate(wiaCustomer);

          // WIA 시스템에 삽입
          await this.wiaDB.customers.create(wiaCustomer);

          // 레거시 DB에서 마이그레이션됨으로 표시
          await this.legacyDB.query(
            `UPDATE customers SET migration_status = 'completed' WHERE id = ${legacyCustomer.id}`
          );

          totalMigrated++;
        } catch (error) {
          console.error(`Failed to migrate customer ${legacyCustomer.id}:`, error);

          errors.push({
            legacyId: legacyCustomer.id,
            error: error.message
          });

          // 실패로 표시
          await this.legacyDB.query(
            `UPDATE customers SET migration_status = 'failed', migration_error = '${error.message}' WHERE id = ${legacyCustomer.id}`
          );

          totalFailed++;
        }
      }

      offset += batchSize;

      // 진행 상황 로그
      console.log(`Progress: ${totalMigrated} migrated, ${totalFailed} failed`);
    }

    return {
      totalMigrated,
      totalFailed,
      errors
    };
  }

  // 검증
  async validateMigration(): Promise<ValidationResult> {
    console.log('Validating migration...');

    // 개수 확인
    const legacyCount = await this.legacyDB.query('SELECT COUNT(*) FROM customers WHERE migration_status = \'completed\'');
    const wiaCount = await this.wiaDB.customers.count({});

    console.log(`Legacy count: ${legacyCount}, WIA count: ${wiaCount}`);

    // 샘플 기록 검증
    const sample = await this.legacyDB.query(`
      SELECT * FROM customers
      WHERE migration_status = 'completed'
      ORDER BY RANDOM()
      LIMIT 100
    `);

    let validCount = 0;
    const discrepancies: any[] = [];

    for (const legacyRecord of sample) {
      const wiaRecord = await this.wiaDB.customers.findByExternalId(legacyRecord.id);

      if (this.recordsMatch(legacyRecord, wiaRecord)) {
        validCount++;
      } else {
        discrepancies.push({
          legacyId: legacyRecord.id,
          wiaId: wiaRecord.customerId,
          differences: this.findDifferences(legacyRecord, wiaRecord)
        });
      }
    }

    return {
      countMatch: legacyCount === wiaCount,
      sampleValidation: {
        total: sample.length,
        valid: validCount,
        invalid: sample.length - validCount
      },
      discrepancies
    };
  }
}
```

---

### 5단계: 배포 (23-24주)

#### 23주: 사전 프로덕션 배포

**사전 프로덕션 체크리스트:**

```
인프라:
☐ 프로덕션 환경 프로비저닝 완료
☐ 데이터베이스 클러스터 구성 및 테스트
☐ 로드 밸런서 구성
☐ CDN 활성화
☐ 모니터링 및 경보 설정
☐ 로그 집계 작동

보안:
☐ 보안 스캔 완료 (OWASP Top 10)
☐ 침투 테스트 통과
☐ SSL 인증서 설치 및 검증
☐ 방화벽 규칙 구성
☐ 비밀 관리 구성
☐ 암호화 키 설정

애플리케이션:
☐ 프로덕션 빌드 배포
☐ 데이터베이스 마이그레이션 실행
☐ 환경 변수 구성
☐ 통합 테스트 (프로덕션)
☐ 성능 벤치마크 확인

백업 및 DR:
☐ 백업 작동 확인
☐ 복구 절차 테스트
☐ DR 사이트 준비
☐ 페일오버 테스트
```

#### 24주: 프로덕션 배포

**배포 프로세스 (주말 배포):**

```
금요일 저녁:
  18:00 - 배포 전 체크리스트 완료
  18:30 - 최종 팀 회의
  19:00 - 레거시 시스템에서 읽기 전용 모드 활성화
  19:30 - 최종 데이터 동기화
  20:00 - WIA KYC/AML 시스템 배포
  20:30 - 스모크 테스트 실행
  21:00 - 쓰기 모드 활성화
  21:30 - 모니터링 확인
  22:00 - 진행/중단 결정

주말:
  - 지속적인 모니터링
  - 문제 해결
  - 성능 조정

월요일 아침:
  08:00 - 비즈니스 검증
  09:00 - 사용자 피드백 수집
  10:00 - 상태 회의
```

**Go-Live 체크리스트:**

```
☐ 모든 P0 및 P1 버그 수정
☐ UAT 승인 획득
☐ 데이터 마이그레이션 검증 완료
☐ 통합 테스트 통과
☐ 성능 벤치마크 충족
☐ 보안 검토 통과
☐ 재해 복구 테스트 완료
☐ 문서 완료 (사용자, 운영, 기술)
☐ 교육 완료
☐ 지원 팀 준비
☐ 롤백 계획 테스트
☐ 경영진 승인
```

**롤백 절차:**

```
롤백이 필요한 경우:
1. 롤백 결정 (30분 이내)
2. 레거시 시스템으로 트래픽 전환
3. WIA 시스템을 유지 보수 모드로 전환
4. 근본 원인 조사
5. 수정 및 재배포 계획
```

---

### 6단계: 하이퍼케어 및 최적화 (25-28주)

#### 25-26주: 하이퍼케어

**활동:**

```
☐ 24/7 워룸 지원
   - 전용 온콜 팀
   - 에스컬레이션 절차
   - 15분 응답 시간 SLA

☐ 일일 상태 회의
   - 새로운 문제 검토
   - 지표 검토
   - 사용자 피드백

☐ 주요 지표 모니터링
   - 시스템 성능
   - 오류율
   - 사용자 채택
   - 비즈니스 메트릭

☐ 신속한 문제 해결
   - 핫픽스 배포
   - 구성 조정
   - 성능 튜닝

☐ 사용자 지원
   - 교육 세션
   - 문서 업데이트
   - FAQ 유지 관리

☐ 성능 조정
   - 데이터베이스 최적화
   - 캐싱 전략
   - 쿼리 최적화

☐ 피드백 수집
   - 사용자 설문 조사
   - 문제 추적
   - 개선 제안
```

**모니터링할 주요 지표:**

| 지표 | 목표 | 경보 임계값 | 현재 값 |
|------|------|------------|---------|
| API 응답 시간 | <500ms | >1000ms | ___ |
| 오류율 | <0.1% | >1% | ___ |
| 확인 성공률 | >95% | <90% | ___ |
| 시스템 가동 시간 | 99.9% | <99.5% | ___ |
| 경보 대기열 깊이 | <100 | >500 | ___ |
| 거짓 양성률 | <30% | >50% | ___ |
| 온보딩 시간 | <4시간 | >8시간 | ___ |
| 사용자 만족도 | >8/10 | <6/10 | ___ |

#### 27-28주: 최적화

**최적화 영역:**

**1. 성능 최적화**

```typescript
// 자주 액세스되는 데이터에 대한 캐싱 추가
class CachedCustomerService {
  async get(customerId: string): Promise<Customer> {
    // 먼저 캐시 확인
    const cacheKey = `customer:${customerId}`;
    const cached = await this.cache.get(cacheKey);

    if (cached) {
      return JSON.parse(cached);
    }

    // DB에서 가져오기
    const customer = await this.repository.findById(customerId);

    // 5분간 캐시
    await this.cache.setex(
      cacheKey,
      300,
      JSON.stringify(customer)
    );

    return customer;
  }
}

// 데이터베이스 쿼리 최적화
// Before: N+1 쿼리
async getCustomersWithRisk() {
  const customers = await db.customers.findMany();
  for (const customer of customers) {
    customer.risk = await db.riskAssessments.findLatest(customer.id);
  }
  return customers;
}

// After: 단일 조인 쿼리
async getCustomersWithRisk() {
  return await db.customers.findMany({
    include: {
      riskAssessments: {
        orderBy: { assessmentDate: 'desc' },
        take: 1
      }
    }
  });
}
```

**2. 거짓 양성 감소**

```typescript
// ML 기반 점수 구현
class MLEnhancedMonitoring {
  async evaluateTransaction(transaction: Transaction): Promise<MLScore> {
    // 고객 행동 프로필 가져오기
    const profile = await this.getCustomerProfile(transaction.customerId);

    // 특성 벡터 생성
    const features = this.extractFeatures(transaction, profile);

    // ML 모델 예측
    const prediction = await this.mlModel.predict(features);

    return {
      suspicionScore: prediction.score,
      confidence: prediction.confidence,
      factors: prediction.featureImportance
    };
  }

  private extractFeatures(transaction: Transaction, profile: CustomerProfile): number[] {
    return [
      this.normalizeAmount(transaction.amount, profile.avgTransaction),
      this.timeOfDayRisk(transaction.timestamp),
      this.locationRisk(transaction.location, profile.usualLocations),
      this.frequencyRisk(transaction, profile.frequency),
      this.counterpartyRisk(transaction.counterparty)
    ];
  }
}
```

**3. 워크플로 자동화**

```typescript
// 저위험에 대한 완전 자동화
class AutomatedOnboardingWorkflow {
  async execute(application: CustomerApplication): Promise<OnboardingResult> {
    // 1. 고객 생성
    const customer = await this.customerService.create(application);

    // 2. 자동 확인
    const verification = await this.autoVerify(customer);

    // 3. 스크리닝
    const screening = await this.screeningService.perform(customer.customerId);

    // 4. 위험 평가
    const risk = await this.riskService.assess(customer.customerId);

    // 5. 자동 승인 (저위험, 클린 스크리닝)
    if (risk.overallRisk.category === 'low' && screening.overallRisk === 'clear') {
      await this.customerService.approve(customer.customerId);

      // 코어 뱅킹에 통지
      await this.coreBankingIntegration.notifyApproval(customer.customerId);

      return {
        status: 'approved',
        timeTaken: this.calculateTime()
      };
    }

    // 수동 검토 필요
    return {
      status: 'manual_review',
      reason: this.getReviewReason(risk, screening)
    };
  }
}
```

---

## 교육 및 변경 관리

### 교육 프로그램

**역할 기반 교육:**

**규정 준수 분석가:**

```
모듈 1: WIA KYC/AML 소개 (2시간)
- 시스템 개요
- 주요 개념
- 인터페이스 탐색

모듈 2: 고객 온보딩 (3시간)
- 온보딩 워크플로
- 신원 확인 프로세스
- 스크리닝 및 위험 평가
- 승인 결정

모듈 3: 경보 조사 (4시간)
- 경보 유형 이해
- 조사 단계
- 증거 수집
- 처분 결정

모듈 4: 사례 관리 (3시간)
- 사례 생성 및 할당
- 조사 문서화
- SAR 준비
- 사례 종료

모듈 5: 보고 및 분석 (2시간)
- 표준 보고서
- 사용자 정의 보고서
- 대시보드
- 데이터 내보내기

모듈 6: 실습 연습 (4시간)
- 시뮬레이션된 시나리오
- 실제 사례
- 문제 해결
```

**IT / 개발자:**

```
모듈 1: 기술 아키텍처 (2시간)
- 시스템 구성 요소
- 통합 지점
- 데이터 흐름

모듈 2: API 개요 (3시간)
- API 설계
- 인증
- 엔드포인트
- 오류 처리

모듈 3: 통합 패턴 (3시간)
- 코어 뱅킹 통합
- 제공업체 통합
- 웹훅 처리

모듈 4: 배포 및 운영 (2시간)
- 배포 프로세스
- 모니터링
- 문제 해결
- 성능 튜닝

모듈 5: 보안 (2시간)
- 인증 및 권한 부여
- 암호화
- 감사 로깅
- 규정 준수
```

---

## 성공 지표

### 운영 지표

| 지표 | 기준선 | 목표 (3개월) | 목표 (6개월) | 목표 (12개월) |
|------|--------|-------------|-------------|-------------|
| **온보딩 시간** | 48시간 | 12시간 | 8시간 | <4시간 |
| **자동 승인률** | 0% | 40% | 60% | 75% |
| **거짓 양성률** | 95% | 60% | 40% | <25% |
| **경보 조사 시간** | 45분 | 35분 | 30분 | <20분 |
| **확인당 비용** | $25 | $15 | $10 | <$5 |
| **고객 만족도** | 5.5/10 | 7.0/10 | 7.5/10 | >8.5/10 |
| **직원 생산성** | 기준선 | +20% | +40% | +50% |

### 비즈니스 지표

| 지표 | 목표 |
|------|------|
| **규정 준수 합격률** | 100% |
| **감사 발견 사항** | <3건/년 |
| **규제 처벌** | $0 |
| **누락된 SAR (감사)** | 0 |
| **시스템 가동 시간** | 99.9% |
| **사용자 채택률** | >90% |
| **ROI** | 18개월 이내 |

---

## 복습 질문

1. 6단계 구현 로드맵은 무엇이며 각 단계는 얼마나 걸립니까?
2. 현재 상태 평가에서 어떤 활동을 수행해야 합니까?
3. 이상적인 구현 팀 구조를 설명하십시오.
4. 단위, 통합, 엔드투엔드, 성능 테스트의 차이점은 무엇입니까?
5. 데이터 마이그레이션 전략에는 어떤 단계가 포함됩니까?
6. Go-Live 체크리스트에는 어떤 항목이 있어야 합니까?
7. 하이퍼케어 기간 동안 모니터링해야 하는 주요 지표는 무엇입니까?
8. 어떻게 거짓 양성률을 줄일 수 있습니까?
9. 워크플로 자동화의 이점은 무엇입니까?
10. 성공적인 구현을 측정하는 핵심 성공 지표는 무엇입니까?

---

## 주요 요점

1. 📅 **6개월 구현** - 계획부터 운영까지 체계적인 접근 방식
2. 🎯 **단계적 접근 방식** - 위험 감소 및 학습 가능
3. 👥 **팀 구성** - 명확한 역할과 책임
4. 🧪 **포괄적인 테스트** - 단위, 통합, E2E, 성능
5. 🔄 **데이터 마이그레이션** - 신중한 계획 및 검증
6. 🚀 **체계적인 배포** - 사전 프로덕션, 프로덕션, 롤백 계획
7. 📊 **지표 기반** - 지속적인 모니터링 및 최적화
8. 🤖 **자동화** - 비용 절감 및 효율성 향상의 핵심

---

**축하합니다!** WIA KYC/AML 표준 전자책을 완료했습니다.

## 다음 단계

### 추가 리소스

- **문서**: https://docs.wia-standards.org/kyc-aml
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **커뮤니티 포럼**: https://community.wia-standards.org
- **기술 지원**: standards@wia-official.org
- **교육**: training@wia-official.org

### 커뮤니티 참여

- **월간 웨비나**: 구현 모범 사례
- **사용자 그룹**: 지역 밋업
- **기여**: GitHub에서 표준 개선
- **피드백**: 귀하의 경험 공유

### 인증 프로그램

- **WIA KYC/AML 실무자 인증**
- **WIA KYC/AML 아키텍트 인증**
- **WIA KYC/AML 감사자 인증**

---

**이전**: [← 7장 - 시스템 통합](07-system-integration.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
