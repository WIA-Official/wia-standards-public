# WIA 냉동보존 동의 관리 표준

## 냉동보존을 위한 사전 동의 관리 시스템

### 포괄적 기술 사양서

---

## 문서 정보

| 속성 | 값 |
|----------|-------|
| **표준 ID** | WIA-CRYO-CONSENT-2025 |
| **버전** | 1.0.0 |
| **상태** | 게시됨 |
| **카테고리** | OTHER - 냉동보존 |
| **최종 업데이트** | 2025-01-10 |

---

## 요약

WIA 냉동보존 동의 관리 표준은 냉동보존 맥락에서 사전 동의를 관리하기 위한 포괄적인 프로토콜을 수립합니다. 즉각적인 치료를 다루는 전통적인 의료 동의와 달리, 냉동보존 동의는 보존 절차, 장기 케어 결정, 회복 조건, 자산 관리 권한을 포함하여 잠재적으로 무한한 시간 범위에 걸친 복잡한 시나리오를 다루어야 합니다.

본 표준은 수십 년 또는 수세기 동안 유효하고 집행 가능해야 할 동의 결정을 캡처, 검증, 저장 및 실행하는 데 필요한 기술 프레임워크, 데이터 구조 및 구현 패턴을 제공합니다.

---

## 목차

1. [소개](#제1장-소개)
2. [시장 분석](#제2장-시장-분석)
3. [데이터 형식](#제3장-데이터-형식)
4. [API 인터페이스](#제4장-api-인터페이스)
5. [제어 프로토콜](#제5장-제어-프로토콜)
6. [통합](#제6장-통합)
7. [보안](#제7장-보안)
8. [구현](#제8장-구현)
9. [미래 동향](#제9장-미래-동향)

---

## 제1장: 소개

### 1.1 목적 및 범위

WIA 냉동보존 동의 표준은 냉동보존 맥락에서 사전 동의의 고유한 과제를 해결합니다. 전통적인 동의 프레임워크는 예측 가능한 일정과 결과를 가진 치료를 위해 설계되었습니다. 냉동보존은 근본적으로 다른 도전과제를 제시합니다:

- **무한한 시간 범위**: 동의는 잠재적으로 수세기 동안 유효해야 함
- **불확실한 기술**: 회복 절차는 동의 시점에 완전히 설명될 수 없음
- **진화하는 가치관**: 환자 가치와 희망은 세대에 걸쳐 해석이 필요할 수 있음
- **복잡한 결정 트리**: 다양한 비상 상황에 대한 문서화된 선호도 필요
- **대리인 권한**: 명확한 의사결정 권한 체인 확립 필요
- **다중 관할권**: 법적 프레임워크가 다양하고 시간이 지남에 따라 변경될 수 있음

### 1.2 핵심 원칙

```typescript
// 핵심 동의 원칙
const consentPrinciples = {
  자발성: {
    definition: '강압 없이 자유롭게 주어지는 동의',
    requirements: [
      '부당한 영향이나 압력 없음',
      '충분한 고려 시간',
      '보존 전 언제든지 철회할 권리',
      '거부에 대한 불이익 없음',
    ],
  },

  충분한정보에기반한결정: {
    definition: '적절한 이해를 바탕으로 한 결정',
    requirements: [
      '절차에 대한 명확한 설명',
      '알려진 위험과 불확실성 공개',
      '대안 옵션 제시',
      '질문 기회 제공',
    ],
  },

  능력: {
    definition: '이해하고 결정을 내릴 수 있는 능력',
    requirements: [
      '필요시 정신 능력 평가',
      '무능력 상태를 위한 사전 지시서',
      '지정된 의사결정권자 식별',
      '정기적 능력 확인',
    ],
  },

  특정성: {
    definition: '식별된 절차와 시나리오에 대한 동의',
    requirements: [
      '각 절차 유형에 대한 특정 동의',
      '알려진 시나리오에 대한 문서화된 선호도',
      '알려지지 않은 시나리오 처리 프레임워크',
      '명확한 범위 경계',
    ],
  },

  내구성: {
    definition: '연장된 기간에 걸친 동의 유효성',
    requirements: [
      '기술 불가지론적 문서화',
      '해석 지침',
      '업데이트 메커니즘',
      '권한 승계',
    ],
  },
};
```

### 1.3 동의 카테고리

```typescript
// 냉동보존 맥락에서의 동의 유형
interface CryonicsConsentCategories {
  보존동의: {
    description: '초기 보존 절차에 대한 동의';
    subCategories: [
      '대기 및 운송',
      '냉동보호제 관류',
      '냉각 및 유리화',
      '장기 보관',
    ];
    criticalElements: [
      '보존 유형 (전신, 신경, 뇌)',
      '응급 절차 승인',
      '해부학적 기증 대안',
      '연구 참여',
    ];
  };

  케어동의: {
    description: '지속적인 케어 결정에 대한 동의';
    subCategories: [
      '보관 시설 선택',
      '이전 승인',
      '비상 이전',
      '유지보수 절차',
    ];
    criticalElements: [
      '시설 선호도',
      '지리적 제한',
      '품질 기준',
      '알림 요구사항',
    ];
  };

  회복동의: {
    description: '미래 회복 시나리오에 대한 동의';
    subCategories: [
      '회복 절차 승인',
      '기술 요구사항',
      '신원 확인',
      '회복 후 케어',
    ];
    criticalElements: [
      '최소 기술 요구사항',
      '삶의 질 기준',
      '신체/기질 선호도',
      '정체성 연속성 기준',
    ];
  };

  대리인동의: {
    description: '타인의 결정 권한 부여';
    subCategories: [
      '의료 대리인',
      '자산 관리 권한',
      '연구 참여',
      '대리 커뮤니케이션',
    ];
    criticalElements: [
      '대리인 식별',
      '권한 범위',
      '승계 계획',
      '분쟁 해결',
    ];
  };

  연구동의: {
    description: '연구 참여에 대한 동의';
    subCategories: [
      '보존 전 연구',
      '보존 기술 연구',
      '장기 보관 연구',
      '회복 기술 연구',
    ];
    criticalElements: [
      '허용되는 연구 유형',
      '데이터 공유 권한',
      '생물학적 샘플 사용',
      '개인정보 보호',
    ];
  };
}
```

### 1.4 동의 서비스 아키텍처

```typescript
// 핵심 동의 관리 서비스
class CryoConsentManagementService {
  private consentRepository: ConsentRepository;
  private validationService: ConsentValidationService;
  private documentService: DocumentService;
  private notificationService: NotificationService;
  private auditService: AuditService;

  constructor(config: ConsentServiceConfig) {
    this.consentRepository = new ConsentRepository(config.database);
    this.validationService = new ConsentValidationService(config.validation);
    this.documentService = new DocumentService(config.documents);
    this.notificationService = new NotificationService(config.notifications);
    this.auditService = new AuditService(config.audit);
  }

  // 새 동의 레코드 생성
  async createConsent(input: ConsentInput): Promise<ConsentRecord> {
    // 입력 검증
    const validationResult = await this.validationService.validateConsentInput(input);
    if (!validationResult.valid) {
      throw new ConsentValidationError(validationResult.errors);
    }

    // 능력 확인
    if (input.requiresCapacityCheck) {
      const capacityResult = await this.assessCapacity(input.patientId);
      if (!capacityResult.hasCapacity) {
        throw new CapacityError('환자가 동의 능력이 없습니다');
      }
    }

    // 동의 레코드 생성
    const consent: ConsentRecord = {
      id: generateConsentId(),
      patientId: input.patientId,
      consentType: input.consentType,
      category: input.category,

      scope: {
        procedures: input.procedures,
        timeframe: input.timeframe || 'INDEFINITE',
        conditions: input.conditions,
        limitations: input.limitations,
      },

      decisions: this.structureDecisions(input.decisions),

      authority: {
        grantor: {
          entityId: input.patientId,
          entityType: 'PATIENT',
          capacityConfirmed: true,
          capacityDate: new Date(),
        },
        witnesses: input.witnesses,
        notarization: input.notarization,
      },

      validity: {
        effectiveDate: input.effectiveDate || new Date(),
        expirationDate: input.expirationDate, // 무기한의 경우 보통 null
        conditions: input.validityConditions,
        status: 'ACTIVE',
      },

      metadata: {
        version: 1,
        createdAt: new Date(),
        updatedAt: new Date(),
        createdBy: input.createdBy,
        documentFormat: 'WIA-CONSENT-V1',
      },
    };

    // 동의 저장
    const stored = await this.consentRepository.create(consent);

    // 문서 생성
    await this.documentService.generateConsentDocuments(stored);

    // 관련 당사자 알림
    await this.notificationService.notifyConsentCreated(stored);

    // 감사 로그
    await this.auditService.logConsentCreation(stored);

    return stored;
  }

  // 결정에 대한 유효 동의 조회
  async getEffectiveConsent(
    patientId: string,
    decisionType: string,
    context: DecisionContext
  ): Promise<EffectiveConsent> {
    // 모든 관련 동의 조회
    const consents = await this.consentRepository.findByPatient(patientId, {
      status: 'ACTIVE',
      includesDecisionType: decisionType,
    });

    // 적용 가능성으로 필터링
    const applicable = consents.filter(c =>
      this.isApplicable(c, decisionType, context)
    );

    if (applicable.length === 0) {
      return {
        found: false,
        patientId,
        decisionType,
        context,
        recommendation: 'NO_CONSENT_FOUND',
      };
    }

    // 다중 적용시 충돌 해결
    const resolved = this.resolveConsentConflicts(applicable);

    // 결정 추출
    const decision = this.extractDecision(resolved, decisionType, context);

    return {
      found: true,
      patientId,
      decisionType,
      context,
      consent: resolved,
      decision,
      confidence: this.calculateConfidence(resolved, decisionType, context),
      interpretation: this.generateInterpretation(resolved, decisionType, context),
    };
  }

  // 기존 동의 업데이트
  async updateConsent(
    consentId: string,
    updates: ConsentUpdates,
    authorization: UpdateAuthorization
  ): Promise<ConsentRecord> {
    const existing = await this.consentRepository.findById(consentId);
    if (!existing) {
      throw new NotFoundError(`동의 ${consentId}를 찾을 수 없습니다`);
    }

    // 권한 검증
    await this.validateUpdateAuthorization(existing, authorization);

    // 현재 상태에서 업데이트 허용 여부 확인
    this.validateUpdateAllowed(existing, updates);

    // 새 버전 생성
    const updated: ConsentRecord = {
      ...existing,
      ...this.applyUpdates(existing, updates),
      metadata: {
        ...existing.metadata,
        version: existing.metadata.version + 1,
        updatedAt: new Date(),
        updatedBy: authorization.updatedBy,
        previousVersion: existing.metadata.version,
      },
    };

    // 버전 이력과 함께 저장
    const stored = await this.consentRepository.update(updated, {
      preserveHistory: true,
    });

    // 감사
    await this.auditService.logConsentUpdate(existing, stored, authorization);

    return stored;
  }

  // 동의 철회
  async revokeConsent(
    consentId: string,
    revocation: ConsentRevocation
  ): Promise<RevokedConsent> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`동의 ${consentId}를 찾을 수 없습니다`);
    }

    // 철회 권한 검증
    await this.validateRevocationAuthority(consent, revocation);

    // 철회 가능 여부 확인
    if (!this.canBeRevoked(consent, revocation)) {
      throw new RevocationError(
        `동의를 철회할 수 없습니다: ${this.getRevocationBlocker(consent)}`
      );
    }

    // 철회 처리
    const revoked = await this.consentRepository.revoke(consentId, {
      revokedAt: new Date(),
      revokedBy: revocation.revokedBy,
      reason: revocation.reason,
      effectiveDate: revocation.effectiveDate || new Date(),
    });

    // 하류 효과 처리
    await this.processRevocationEffects(revoked);

    // 감사
    await this.auditService.logConsentRevocation(revoked);

    return revoked;
  }

  // 동의가 상황에 적용되는지 평가
  private isApplicable(
    consent: ConsentRecord,
    decisionType: string,
    context: DecisionContext
  ): boolean {
    // 상태 확인
    if (consent.validity.status !== 'ACTIVE') {
      return false;
    }

    // 날짜 범위 확인
    const now = new Date();
    if (consent.validity.effectiveDate > now) {
      return false;
    }
    if (consent.validity.expirationDate && consent.validity.expirationDate < now) {
      return false;
    }

    // 결정 유형이 포함되는지 확인
    if (!this.coversDecisionType(consent, decisionType)) {
      return false;
    }

    // 조건 확인
    for (const condition of consent.validity.conditions || []) {
      if (!this.conditionMet(condition, context)) {
        return false;
      }
    }

    return true;
  }
}
```

### 1.5 결정 프레임워크

```typescript
// 동의 기반 결정 프레임워크
interface ConsentDecisionFramework {
  // 불명확한 상황에 대한 결정 계층
  decisionHierarchy: {
    level1: '명시적 환자 동의 문서';
    level2: '환자 지정 대리인 결정';
    level3: '환자 가치와 희망의 해석';
    level4: '윤리위원회와 함께 최선의 이익 기준';
    level5: '법원/법적 기관 결정';
  };

  // 해석 지침
  interpretationPrinciples: {
    specificOverGeneral: '더 구체적인 동의가 일반적인 것보다 우선';
    recentOverOlder: '더 최근의 동의가 일반적으로 우선';
    preservationBias: '불명확할 때 가역적/보존 옵션 선호';
    valueBased: '문서화된 환자 가치의 맥락에서 해석';
    minimumIntervention: '불명확할 때 최소 침습적 옵션';
  };
}

class ConsentDecisionEngine {
  private consentService: CryoConsentManagementService;
  private valueAnalyzer: PatientValueAnalyzer;
  private ethicsService: EthicsConsultationService;

  // 동의 프레임워크에 기반한 결정
  async makeDecision(
    patientId: string,
    decision: DecisionRequest
  ): Promise<ConsentBasedDecision> {
    // 유효 동의 조회
    const effectiveConsent = await this.consentService.getEffectiveConsent(
      patientId,
      decision.type,
      decision.context
    );

    // 명시적 동의가 존재하는 경우
    if (effectiveConsent.found && effectiveConsent.confidence >= 0.8) {
      return {
        decision: effectiveConsent.decision,
        basis: 'EXPLICIT_CONSENT',
        confidence: effectiveConsent.confidence,
        consent: effectiveConsent.consent,
        interpretation: effectiveConsent.interpretation,
      };
    }

    // 대리인 권한 확인
    const proxyDecision = await this.getProxyDecision(patientId, decision);
    if (proxyDecision) {
      return {
        decision: proxyDecision.decision,
        basis: 'PROXY_DECISION',
        confidence: proxyDecision.confidence,
        proxy: proxyDecision.proxy,
        reasoning: proxyDecision.reasoning,
      };
    }

    // 가치 기반 해석
    const valueBasedDecision = await this.interpretFromValues(patientId, decision);
    if (valueBasedDecision.confidence >= 0.6) {
      return {
        decision: valueBasedDecision.decision,
        basis: 'VALUE_INTERPRETATION',
        confidence: valueBasedDecision.confidence,
        valueAnalysis: valueBasedDecision.analysis,
        requiresReview: true,
      };
    }

    // 윤리위원회로 에스컬레이션
    return {
      decision: null,
      basis: 'ESCALATION_REQUIRED',
      confidence: 0,
      escalationRequired: true,
      escalationTarget: 'ETHICS_COMMITTEE',
      context: decision,
    };
  }

  // 지정 대리인으로부터 결정 획득
  private async getProxyDecision(
    patientId: string,
    decision: DecisionRequest
  ): Promise<ProxyDecisionResult | null> {
    // 이 결정 유형에 대한 권한이 있는 활성 대리인 찾기
    const proxy = await this.findAuthorizedProxy(patientId, decision.type);
    if (!proxy) {
      return null;
    }

    // 대리인 가용성 및 의향 확인
    const proxyResponse = await this.requestProxyDecision(proxy, decision);
    if (!proxyResponse) {
      return null;
    }

    return {
      decision: proxyResponse.decision,
      confidence: this.assessProxyDecisionConfidence(proxy, proxyResponse),
      proxy,
      reasoning: proxyResponse.reasoning,
    };
  }

  // 문서화된 가치에서 결정 해석
  private async interpretFromValues(
    patientId: string,
    decision: DecisionRequest
  ): Promise<ValueBasedDecision> {
    // 환자의 문서화된 가치 조회
    const values = await this.valueAnalyzer.getPatientValues(patientId);

    // 가치에 대한 결정 옵션 분석
    const analysis = await this.valueAnalyzer.analyzeDecision(
      values,
      decision.options,
      decision.context
    );

    return {
      decision: analysis.recommendedOption,
      confidence: analysis.confidence,
      analysis,
    };
  }
}
```

---

## 주요 기능

### 동의 생명주기 관리

- **생성**: 적절한 문서화와 함께 상세하고 구조화된 동의 캡처
- **검증**: 동의가 법적 및 윤리적 요구사항을 충족하는지 확인
- **저장**: 무결성 보장과 함께 동의 레코드 보존
- **검색**: 결정에 대한 관련 동의 효율적 액세스
- **업데이트**: 이력 보존과 함께 수정 허용
- **철회**: 동의 철회 적절히 처리

### 결정 지원

- **동의 조회**: 특정 시나리오에 적용 가능한 동의 찾기
- **해석**: 새로운 상황에 동의 적용 지침
- **대리인 관리**: 위임된 의사결정 권한 처리
- **충돌 해결**: 중복되거나 모순되는 동의 처리
- **에스컬레이션**: 모호한 상황에 대한 명확한 경로

### 컴플라이언스 및 감사

- **규제 준수**: 의료 동의 요구사항 충족
- **완전한 감사 추적**: 모든 동의 관련 행동 추적
- **문서 생성**: 법적으로 준수되는 동의 문서 생성
- **보고**: 컴플라이언스 및 상태 보고서 생성

---

*다음 장: 시장 분석 - 동의 관리 환경 이해*
