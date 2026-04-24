# 제4장: API 인터페이스

## 동의 관리 API

본 장에서는 WIA Cryo-Consent 표준의 종합적인 API 인터페이스를 정의합니다. RESTful API, GraphQL 엔드포인트, 그리고 동의 관리 작업을 위한 실시간 WebSocket 인터페이스를 포함합니다.

---

## 4.1 RESTful API 설계

```typescript
// 기본 API 설정
const apiConfig = {
  version: 'v1',
  basePath: '/api/consent',
  contentType: 'application/json',
  authentication: 'Bearer JWT',
  rateLimit: {
    windowMs: 60000,
    maxRequests: 100,
  },
};

// API 라우터 설정
import express from 'express';

const router = express.Router();

// 미들웨어
router.use(authenticationMiddleware);
router.use(rateLimitMiddleware);
router.use(auditMiddleware);
router.use(validationMiddleware);

// 동의 CRUD 엔드포인트
router.post('/consents', createConsentHandler);
router.get('/consents', listConsentsHandler);
router.get('/consents/:id', getConsentHandler);
router.put('/consents/:id', updateConsentHandler);
router.delete('/consents/:id', revokeConsentHandler);

// 동의 생명주기 엔드포인트
router.post('/consents/:id/activate', activateConsentHandler);
router.post('/consents/:id/suspend', suspendConsentHandler);
router.post('/consents/:id/renew', renewConsentHandler);
router.post('/consents/:id/review', recordReviewHandler);

// 의사결정 엔드포인트
router.get('/consents/:id/decisions', listDecisionsHandler);
router.post('/consents/:id/decisions', addDecisionHandler);
router.put('/consents/:id/decisions/:decisionId', updateDecisionHandler);

// 조회 엔드포인트
router.get('/patients/:patientId/consents', getPatientConsentsHandler);
router.get('/patients/:patientId/decisions/:decisionType', queryDecisionHandler);
router.get('/patients/:patientId/effective-consent', getEffectiveConsentHandler);

// 문서 엔드포인트
router.post('/consents/:id/documents', uploadDocumentHandler);
router.get('/consents/:id/documents', listDocumentsHandler);
router.get('/consents/:id/documents/:docId', downloadDocumentHandler);

// 대리인 엔드포인트
router.get('/patients/:patientId/proxies', listProxiesHandler);
router.post('/patients/:patientId/proxies', designateProxyHandler);
router.put('/patients/:patientId/proxies/:proxyId', updateProxyHandler);
router.delete('/patients/:patientId/proxies/:proxyId', removeProxyHandler);

// 감사 엔드포인트
router.get('/consents/:id/audit', getAuditTrailHandler);
router.get('/patients/:patientId/audit', getPatientAuditHandler);

export default router;
```

---

## 4.2 동의 CRUD 작업

```typescript
// 동의 생성 핸들러
async function createConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const input: CreateConsentInput = req.body;

    // 입력 유효성 검사
    const validation = await validateCreateConsentInput(input);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: '유효하지 않은 동의 입력',
          details: validation.errors,
        },
      });
      return;
    }

    // 권한 확인
    const authResult = await checkCreateAuthorization(
      req.user,
      input.patientId
    );
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: authResult.reason,
        },
      });
      return;
    }

    // 동의 생성
    const consent = await consentService.createConsent(input, {
      createdBy: req.user.id,
      ipAddress: req.ip,
      userAgent: req.headers['user-agent'],
    });

    // 응답 생성
    res.status(201).json({
      success: true,
      data: {
        consent: serializeConsent(consent),
        links: {
          self: `/api/consent/v1/consents/${consent.id}`,
          patient: `/api/consent/v1/patients/${consent.patientId}`,
          decisions: `/api/consent/v1/consents/${consent.id}/decisions`,
          documents: `/api/consent/v1/consents/${consent.id}/documents`,
        },
      },
    });
  } catch (error) {
    next(error);
  }
}

// 동의 생성 입력 인터페이스
interface CreateConsentInput {
  patientId: string;
  consentType: ConsentType;
  category: ConsentCategory;
  subcategories?: string[];

  scope: {
    procedures: ProcedureScopeInput[];
    timeframe: TimeframeScopeInput;
    conditions?: ScopeConditionInput[];
    limitations?: ScopeLimitationInput[];
    jurisdictions?: string[];
  };

  decisions: DecisionInput[];

  authority: {
    witnesses?: WitnessInput[];
    notarization?: NotarizationInput;
    capacityAssessment?: CapacityAssessmentInput;
  };

  documents?: DocumentInput[];

  effectiveDate?: string;
  expirationDate?: string;
}

// 동의 조회 핸들러
async function getConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const include = parseIncludeParam(req.query.include);

    // 동의 조회
    const consent = await consentService.getConsent(id, include);

    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `동의 ${id}를 찾을 수 없습니다`,
        },
      });
      return;
    }

    // 읽기 권한 확인
    const authResult = await checkReadAuthorization(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: '이 동의를 볼 권한이 없습니다',
        },
      });
      return;
    }

    // 접근 기록
    await auditService.logAccess(consent.id, req.user.id, 'READ');

    res.json({
      success: true,
      data: {
        consent: serializeConsent(consent, include),
        links: generateConsentLinks(consent),
      },
    });
  } catch (error) {
    next(error);
  }
}

// 동의 수정 핸들러
async function updateConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const updates: UpdateConsentInput = req.body;

    // 기존 동의 조회
    const existing = await consentService.getConsent(id);
    if (!existing) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `동의 ${id}를 찾을 수 없습니다`,
        },
      });
      return;
    }

    // 수정 권한 확인
    const authResult = await checkUpdateAuthorization(
      req.user,
      existing,
      updates
    );
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: authResult.reason,
        },
      });
      return;
    }

    // 수정 내용 검증
    const validation = await validateUpdateConsentInput(existing, updates);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: '유효하지 않은 수정 사항',
          details: validation.errors,
        },
      });
      return;
    }

    // 수정 적용
    const updated = await consentService.updateConsent(id, updates, {
      updatedBy: req.user.id,
      reason: updates.reason,
      ipAddress: req.ip,
    });

    res.json({
      success: true,
      data: {
        consent: serializeConsent(updated),
        previousVersion: existing.metadata.version,
        currentVersion: updated.metadata.version,
      },
    });
  } catch (error) {
    next(error);
  }
}

// 동의 철회 핸들러
async function revokeConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const revocation: RevocationInput = req.body;

    // 동의 조회
    const consent = await consentService.getConsent(id);
    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `동의 ${id}를 찾을 수 없습니다`,
        },
      });
      return;
    }

    // 철회 권한 확인
    const authResult = await checkRevocationAuthorization(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: '이 동의를 철회할 권한이 없습니다',
        },
      });
      return;
    }

    // 철회 가능 여부 확인
    const revocability = await consentService.checkRevocability(consent);
    if (!revocability.revocable) {
      res.status(409).json({
        success: false,
        error: {
          code: 'NOT_REVOCABLE',
          message: revocability.reason,
          blockers: revocability.blockers,
        },
      });
      return;
    }

    // 철회 처리
    const revoked = await consentService.revokeConsent(id, {
      ...revocation,
      revokedBy: req.user.id,
      ipAddress: req.ip,
    });

    res.json({
      success: true,
      data: {
        consent: serializeConsent(revoked),
        revocation: {
          revokedAt: revoked.validity.revocation?.revokedAt,
          effectiveDate: revoked.validity.revocation?.effectiveDate,
          reason: revoked.validity.revocation?.reason,
        },
      },
    });
  } catch (error) {
    next(error);
  }
}

// 동의 목록 핸들러
async function listConsentsHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const query = parseConsentQuery(req.query);

    // 사용자 기반 필터링 적용
    const filteredQuery = applyUserFilters(query, req.user);

    // 쿼리 실행
    const result = await consentService.queryConsents(filteredQuery);

    res.json({
      success: true,
      data: {
        consents: result.consents.map(c => serializeConsentSummary(c)),
        pagination: {
          total: result.total,
          offset: result.offset,
          limit: result.limit,
          hasMore: result.hasMore,
        },
        aggregations: result.aggregations,
      },
      links: {
        self: buildQueryLink(filteredQuery),
        next: result.hasMore
          ? buildQueryLink({ ...filteredQuery, offset: result.offset + result.limit })
          : null,
        prev: result.offset > 0
          ? buildQueryLink({ ...filteredQuery, offset: Math.max(0, result.offset - result.limit) })
          : null,
      },
    });
  } catch (error) {
    next(error);
  }
}
```

---

## 4.3 의사결정 조회 API

```typescript
// 유효 동의 조회 핸들러
async function getEffectiveConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { patientId } = req.params;
    const {
      decisionType,
      scenario,
      context,
      asOfDate,
      interpretationMode,
    } = req.query;

    // 의사결정 쿼리 구성
    const query: DecisionQuery = {
      patientId,
      decisionType: decisionType as string,
      context: context ? JSON.parse(context as string) : {},
      asOfDate: asOfDate ? new Date(asOfDate as string) : undefined,
      interpretationMode: (interpretationMode as 'STRICT' | 'FLEXIBLE') || 'FLEXIBLE',
    };

    // 쿼리 실행
    const result = await consentService.queryEffectiveConsent(query);

    res.json({
      success: true,
      data: {
        found: result.found,
        decision: result.decision,
        consent: result.consent
          ? {
              id: result.consent.id,
              category: result.consent.category,
              effectiveDate: result.consent.validity.effectiveDate,
              version: result.consent.metadata.version,
            }
          : null,
        confidence: result.confidence,
        interpretation: result.interpretation,
        alternatives: result.alternatives,
        warnings: result.warnings,
        recommendations: result.recommendations,
      },
    });
  } catch (error) {
    next(error);
  }
}

// 의사결정 쿼리 서비스
class ConsentDecisionQueryService {
  constructor(
    private consentRepository: ConsentRepository,
    private interpretationEngine: ConsentInterpretationEngine
  ) {}

  async queryEffectiveConsent(query: DecisionQuery): Promise<DecisionQueryResult> {
    // 적용 가능한 동의 찾기
    const consents = await this.consentRepository.findByPatient(
      query.patientId,
      {
        status: 'ACTIVE',
        effectiveAsOf: query.asOfDate || new Date(),
      }
    );

    // 의사결정 유형을 포함하는 동의로 필터링
    const applicable = consents.filter(c =>
      this.coversDecisionType(c, query.decisionType)
    );

    if (applicable.length === 0) {
      return {
        found: false,
        confidence: 0,
        warnings: ['이 의사결정 유형에 해당하는 동의를 찾을 수 없습니다'],
        recommendations: [
          '환자에게 명시적 동의를 요청해야 합니다',
          '지정된 대리인이 있다면 상담을 고려하세요',
        ],
      };
    }

    // 가장 구체적인 동의 찾기
    const ranked = this.rankConsents(applicable, query);
    const best = ranked[0];

    // 의사결정 추출
    const decision = this.extractDecision(best.consent, query.decisionType);

    // 필요시 해석 적용
    const interpretation = await this.interpretationEngine.interpret(
      decision,
      query.context,
      query.interpretationMode
    );

    return {
      found: true,
      decision: interpretation.interpretedDecision,
      consent: best.consent,
      confidence: interpretation.confidence,
      interpretation: interpretation.explanation,
      alternatives: ranked.slice(1, 4).map(r => this.extractDecision(r.consent, query.decisionType)),
      warnings: interpretation.warnings,
      recommendations: interpretation.recommendations,
    };
  }

  private coversDecisionType(consent: ConsentRecord, decisionType: string): boolean {
    return consent.decisions.some(d =>
      d.decisionType === decisionType ||
      this.isRelatedDecisionType(d.decisionType, decisionType)
    );
  }

  private rankConsents(
    consents: ConsentRecord[],
    query: DecisionQuery
  ): RankedConsent[] {
    return consents
      .map(consent => ({
        consent,
        score: this.calculateRelevanceScore(consent, query),
      }))
      .sort((a, b) => b.score - a.score);
  }

  private calculateRelevanceScore(
    consent: ConsentRecord,
    query: DecisionQuery
  ): number {
    let score = 0;

    // 최신 동의일수록 높은 점수
    const age = Date.now() - consent.metadata.updatedAt.getTime();
    const ageScore = Math.max(0, 100 - age / (365 * 24 * 60 * 60 * 1000));
    score += ageScore * 0.2;

    // 더 구체적인 카테고리 일치 시 높은 점수
    if (consent.category === this.getCategoryForDecision(query.decisionType)) {
      score += 30;
    }

    // 해당 유형에 대한 명시적 의사결정 시 최고 점수
    if (consent.decisions.some(d => d.question.toLowerCase().includes(query.decisionType.toLowerCase()))) {
      score += 50;
    }

    // 높은 버전의 동의일수록 높은 점수 (더 정제됨)
    score += Math.min(consent.metadata.version * 2, 10);

    return score;
  }

  private extractDecision(
    consent: ConsentRecord,
    decisionType: string
  ): ConsentDecision | undefined {
    return consent.decisions.find(d =>
      d.decisionType === decisionType ||
      d.question.toLowerCase().includes(decisionType.toLowerCase())
    );
  }

  private getCategoryForDecision(decisionType: string): ConsentCategory {
    const categoryMap: Record<string, ConsentCategory> = {
      PRESERVATION: ConsentCategory.PRESERVATION,
      STANDBY: ConsentCategory.PRESERVATION,
      COOLING: ConsentCategory.PRESERVATION,
      STORAGE: ConsentCategory.CARE,
      TRANSFER: ConsentCategory.CARE,
      REVIVAL: ConsentCategory.REVIVAL,
      TECHNOLOGY: ConsentCategory.REVIVAL,
      RESEARCH: ConsentCategory.RESEARCH,
      DATA_SHARING: ConsentCategory.RESEARCH,
    };
    return categoryMap[decisionType] || ConsentCategory.CARE;
  }
}

interface RankedConsent {
  consent: ConsentRecord;
  score: number;
}
```

---

## 4.4 대리인 관리 API

```typescript
// 대리인 지정 엔드포인트
async function designateProxyHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { patientId } = req.params;
    const proxyInput: ProxyDesignationInput = req.body;

    // 입력 검증
    const validation = validateProxyInput(proxyInput);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: '유효하지 않은 대리인 지정',
          details: validation.errors,
        },
      });
      return;
    }

    // 권한 확인 (환자 또는 권한 있는 대리인이어야 함)
    const authResult = await checkProxyDesignationAuth(req.user, patientId);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: '환자만 대리인을 지정할 수 있습니다',
        },
      });
      return;
    }

    // 대리인 지정 생성
    const proxy = await proxyService.designateProxy(patientId, proxyInput, {
      designatedBy: req.user.id,
      ipAddress: req.ip,
    });

    // 대리인에게 알림
    await notificationService.notifyProxyDesignation(proxy);

    res.status(201).json({
      success: true,
      data: {
        proxy: serializeProxy(proxy),
        acceptanceRequired: true,
        acceptanceDeadline: calculateAcceptanceDeadline(),
      },
    });
  } catch (error) {
    next(error);
  }
}

// 대리인 서비스
class ProxyManagementService {
  constructor(
    private proxyRepository: ProxyRepository,
    private consentRepository: ConsentRepository,
    private notificationService: NotificationService
  ) {}

  async designateProxy(
    patientId: string,
    input: ProxyDesignationInput,
    context: OperationContext
  ): Promise<ProxyDesignation> {
    // 기존 대리인 확인
    const existingProxies = await this.proxyRepository.findByPatient(patientId);

    // 순서 결정
    const order = existingProxies.length + 1;

    // 대리인 레코드 생성
    const proxy: ProxyDesignation = {
      proxyId: generateProxyId(),
      proxyType: input.proxyType,

      individual: input.proxyType === 'INDIVIDUAL' ? {
        name: input.name,
        relationship: input.relationship,
        contactInfo: input.contactInfo,
        identityVerification: {
          verificationMethod: 'PENDING',
          verificationDate: new Date(),
          verifiedBy: 'SYSTEM',
          documentTypes: [],
          documentReferences: [],
          verificationScore: 0,
        },
      } : undefined,

      organization: input.proxyType === 'ORGANIZATION' ? {
        name: input.organizationName!,
        registrationNumber: input.registrationNumber!,
        contactInfo: input.contactInfo,
        authorizedRepresentatives: input.representatives || [],
      } : undefined,

      authorityScope: {
        categories: input.categories,
        specificDecisions: input.specificDecisions || [],
        exclusions: input.exclusions || [],
        financialLimit: input.financialLimit,
        requiresConsultation: input.requiresConsultation || [],
        requiresCommitteeApproval: input.requiresCommitteeApproval || [],
        effectiveFrom: input.effectiveFrom,
        effectiveUntil: input.effectiveUntil,
        activationCondition: input.activationCondition,
      },

      acceptanceStatus: 'PENDING',
    };

    // 저장
    const stored = await this.proxyRepository.create(patientId, proxy, order);

    return stored;
  }

  async acceptProxyDesignation(
    proxyId: string,
    acceptance: ProxyAcceptanceInput
  ): Promise<ProxyDesignation> {
    const proxy = await this.proxyRepository.findById(proxyId);
    if (!proxy) {
      throw new NotFoundError(`대리인 ${proxyId}를 찾을 수 없습니다`);
    }

    if (proxy.acceptanceStatus !== 'PENDING') {
      throw new ConflictError(`대리인이 이미 ${proxy.acceptanceStatus.toLowerCase()} 상태입니다`);
    }

    // 수락자 신원 확인
    await this.verifyAcceptorIdentity(proxy, acceptance);

    // 수락 업데이트
    const updated = await this.proxyRepository.update(proxyId, {
      acceptanceStatus: 'ACCEPTED',
      acceptanceDate: new Date(),
      acceptanceSignature: acceptance.signature,
    });

    // 환자에게 알림
    await this.notificationService.notifyProxyAcceptance(updated);

    return updated;
  }

  async activateProxy(
    patientId: string,
    proxyId: string,
    activationReason: string
  ): Promise<ActivatedProxy> {
    const proxy = await this.proxyRepository.findById(proxyId);
    if (!proxy) {
      throw new NotFoundError(`대리인 ${proxyId}를 찾을 수 없습니다`);
    }

    if (proxy.acceptanceStatus !== 'ACCEPTED') {
      throw new ConflictError('활성화 전에 대리인이 지정을 수락해야 합니다');
    }

    // 활성화 조건 확인
    if (proxy.authorityScope.activationCondition) {
      const conditionMet = await this.evaluateActivationCondition(
        patientId,
        proxy.authorityScope.activationCondition
      );
      if (!conditionMet) {
        throw new ConflictError('활성화 조건이 충족되지 않았습니다');
      }
    }

    // 상위 우선순위 대리인 가용성 확인
    const higherPriority = await this.getHigherPriorityProxies(patientId, proxyId);
    if (higherPriority.length > 0) {
      throw new ConflictError(
        '이 대리인을 활성화하려면 상위 우선순위 대리인이 불가용 상태여야 합니다'
      );
    }

    // 활성화
    const activated = await this.proxyRepository.activate(proxyId, {
      activatedAt: new Date(),
      activationReason,
    });

    return {
      proxy: activated,
      activatedAt: new Date(),
      reason: activationReason,
      authorities: activated.authorityScope.categories,
    };
  }

  async makeProxyDecision(
    proxyId: string,
    decision: ProxyDecisionInput
  ): Promise<ProxyDecisionRecord> {
    const proxy = await this.proxyRepository.findById(proxyId);
    if (!proxy) {
      throw new NotFoundError(`대리인 ${proxyId}를 찾을 수 없습니다`);
    }

    // 대리인 활성 상태 확인
    const isActive = await this.isProxyActive(proxyId);
    if (!isActive) {
      throw new ConflictError('대리인이 현재 활성 상태가 아닙니다');
    }

    // 이 의사결정에 대한 권한 확인
    if (!this.hasAuthorityForDecision(proxy, decision.decisionType)) {
      throw new UnauthorizedError('대리인이 이 의사결정 유형에 대한 권한이 없습니다');
    }

    // 상담 필요 여부 확인
    if (proxy.authorityScope.requiresConsultation.includes(decision.decisionType)) {
      if (!decision.consultationRecords || decision.consultationRecords.length === 0) {
        throw new ValidationError('이 의사결정은 문서화된 상담이 필요합니다');
      }
    }

    // 위원회 승인 필요 여부 확인
    if (proxy.authorityScope.requiresCommitteeApproval.includes(decision.decisionType)) {
      if (!decision.committeeApproval) {
        throw new ValidationError('이 의사결정은 위원회 승인이 필요합니다');
      }
    }

    // 의사결정 기록
    const record: ProxyDecisionRecord = {
      decisionId: generateDecisionId(),
      proxyId,
      patientId: await this.getPatientForProxy(proxyId),
      decisionType: decision.decisionType,
      decision: decision.decision,
      reasoning: decision.reasoning,
      consultationRecords: decision.consultationRecords,
      committeeApproval: decision.committeeApproval,
      timestamp: new Date(),
    };

    const stored = await this.proxyRepository.recordDecision(record);

    return stored;
  }

  private hasAuthorityForDecision(
    proxy: ProxyDesignation,
    decisionType: string
  ): boolean {
    // 제외 항목 먼저 확인
    if (proxy.authorityScope.exclusions.includes(decisionType)) {
      return false;
    }

    // 구체적 의사결정 확인
    if (proxy.authorityScope.specificDecisions.length > 0) {
      return proxy.authorityScope.specificDecisions.includes(decisionType);
    }

    // 카테고리 권한 확인
    const category = this.getCategoryForDecision(decisionType);
    return proxy.authorityScope.categories.includes(category);
  }
}

interface ProxyDesignationInput {
  proxyType: 'INDIVIDUAL' | 'ORGANIZATION';
  name?: string;
  relationship?: string;
  organizationName?: string;
  registrationNumber?: string;
  contactInfo: ContactInfo;
  representatives?: string[];
  categories: ConsentCategory[];
  specificDecisions?: string[];
  exclusions?: string[];
  financialLimit?: number;
  requiresConsultation?: string[];
  requiresCommitteeApproval?: string[];
  effectiveFrom?: Date;
  effectiveUntil?: Date;
  activationCondition?: string;
}

interface ProxyAcceptanceInput {
  acceptorId: string;
  signature: SignatureRecord;
  acknowledgements: string[];
}

interface ProxyDecisionInput {
  decisionType: string;
  decision: any;
  reasoning: string;
  consultationRecords?: ConsultationRecord[];
  committeeApproval?: CommitteeApproval;
}
```

---

## 4.5 문서 관리 API

```typescript
// 문서 업로드 핸들러
async function uploadDocumentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id: consentId } = req.params;

    // 동의 존재 확인
    const consent = await consentService.getConsent(consentId);
    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `동의 ${consentId}를 찾을 수 없습니다`,
        },
      });
      return;
    }

    // 업로드 권한 확인
    const authResult = await checkDocumentUploadAuth(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: '이 동의에 문서를 업로드할 권한이 없습니다',
        },
      });
      return;
    }

    // 업로드 처리
    const files = req.files as Express.Multer.File[];
    const documentType = req.body.documentType as DocumentType;

    const uploadedDocs = await Promise.all(
      files.map(file =>
        documentService.uploadDocument(consentId, {
          file,
          documentType,
          title: req.body.title || file.originalname,
          description: req.body.description,
          language: req.body.language || 'ko',
        })
      )
    );

    res.status(201).json({
      success: true,
      data: {
        documents: uploadedDocs.map(serializeDocument),
      },
    });
  } catch (error) {
    next(error);
  }
}

// 문서 서비스
class ConsentDocumentService {
  constructor(
    private storageService: StorageService,
    private integrityService: IntegrityService,
    private encryptionService: EncryptionService
  ) {}

  async uploadDocument(
    consentId: string,
    input: DocumentUploadInput
  ): Promise<ConsentDocument> {
    // 문서 ID 생성
    const documentId = generateDocumentId();

    // 암호화 전 해시 계산
    const hash = await this.integrityService.calculateHash(input.file.buffer);

    // 파일 암호화
    const encrypted = await this.encryptionService.encrypt(input.file.buffer);

    // 기본 위치에 저장
    const primaryLocation = await this.storageService.store(
      documentId,
      encrypted.data,
      {
        contentType: input.file.mimetype,
        metadata: {
          consentId,
          documentType: input.documentType,
          originalName: input.file.originalname,
        },
      }
    );

    // 백업 위치에 복제
    const backupLocations = await this.replicateToBackups(
      documentId,
      encrypted.data
    );

    // 블록체인에 앵커링
    const blockchainAnchor = await this.anchorToBlockchain(hash);

    // 문서 레코드 생성
    const document: ConsentDocument = {
      documentId,
      documentType: input.documentType,
      title: input.title,
      description: input.description || '',
      language: input.language,

      storage: {
        storageType: 'HYBRID',
        primaryLocation,
        backupLocations,
        encryption: {
          encrypted: true,
          algorithm: encrypted.algorithm,
          keyReference: encrypted.keyId,
        },
        retentionPolicy: {
          retentionPeriod: 'P1000Y', // 1000년
          retentionType: 'INDEFINITE',
        },
      },

      version: {
        versionNumber: '1.0',
        versionDate: new Date(),
        createdBy: input.uploadedBy,
        isCurrent: true,
        isLatest: true,
      },

      integrity: {
        hashAlgorithm: 'SHA-256',
        hashValue: hash,
        additionalHashes: [
          {
            algorithm: 'SHA-3-256',
            value: await this.integrityService.calculateSha3Hash(input.file.buffer),
          },
        ],
        timestamp: {
          timestampDate: new Date(),
          timestampAuthority: 'WIA-TSA',
        },
        blockchainAnchor,
      },

      accessControl: {
        accessLevel: 'CONFIDENTIAL',
        authorizedRoles: ['PATIENT', 'PROXY', 'MEDICAL_STAFF', 'LEGAL'],
        authorizedIndividuals: [],
        accessConditions: [],
        accessLog: [],
      },

      legalStatus: {
        isLegallyBinding: input.documentType !== 'SUPPORTING_EVIDENCE',
        jurisdictions: [],
      },
    };

    return document;
  }

  async downloadDocument(
    documentId: string,
    requestor: User
  ): Promise<DocumentDownload> {
    // 문서 레코드 조회
    const document = await this.getDocument(documentId);
    if (!document) {
      throw new NotFoundError(`문서 ${documentId}를 찾을 수 없습니다`);
    }

    // 접근 권한 확인
    const hasAccess = await this.checkDocumentAccess(document, requestor);
    if (!hasAccess) {
      throw new UnauthorizedError('이 문서에 접근할 권한이 없습니다');
    }

    // 저장소에서 검색
    const encrypted = await this.storageService.retrieve(
      document.storage.primaryLocation.locationUri
    );

    // 복호화
    const decrypted = await this.encryptionService.decrypt(
      encrypted,
      document.storage.encryption.keyReference
    );

    // 무결성 검증
    const hash = await this.integrityService.calculateHash(decrypted);
    if (hash !== document.integrity.hashValue) {
      throw new IntegrityError('문서 무결성 검사 실패');
    }

    // 접근 기록
    await this.logDocumentAccess(document, requestor, 'DOWNLOAD');

    return {
      documentId,
      data: decrypted,
      contentType: this.getContentType(document),
      filename: this.getFilename(document),
    };
  }

  async verifyDocumentIntegrity(documentId: string): Promise<IntegrityVerification> {
    const document = await this.getDocument(documentId);
    if (!document) {
      throw new NotFoundError(`문서 ${documentId}를 찾을 수 없습니다`);
    }

    // 문서 검색
    const encrypted = await this.storageService.retrieve(
      document.storage.primaryLocation.locationUri
    );
    const decrypted = await this.encryptionService.decrypt(
      encrypted,
      document.storage.encryption.keyReference
    );

    // 기본 해시 검증
    const currentHash = await this.integrityService.calculateHash(decrypted);
    const primaryValid = currentHash === document.integrity.hashValue;

    // 추가 해시 검증
    const additionalVerifications = await Promise.all(
      (document.integrity.additionalHashes || []).map(async h => ({
        algorithm: h.algorithm,
        valid: await this.integrityService.verifyHash(decrypted, h.algorithm, h.value),
      }))
    );

    // 블록체인 앵커 검증
    let blockchainValid = false;
    if (document.integrity.blockchainAnchor) {
      blockchainValid = await this.verifyBlockchainAnchor(
        document.integrity.hashValue,
        document.integrity.blockchainAnchor
      );
    }

    return {
      documentId,
      verificationDate: new Date(),
      primaryHashValid: primaryValid,
      additionalHashesValid: additionalVerifications.every(v => v.valid),
      blockchainAnchorValid: blockchainValid,
      overallValid: primaryValid && additionalVerifications.every(v => v.valid),
      details: {
        primaryHash: {
          algorithm: document.integrity.hashAlgorithm,
          expected: document.integrity.hashValue,
          actual: currentHash,
          valid: primaryValid,
        },
        additionalHashes: additionalVerifications,
        blockchainAnchor: document.integrity.blockchainAnchor,
      },
    };
  }

  private async anchorToBlockchain(hash: string): Promise<BlockchainAnchor> {
    // 불변 타임스탬프를 위해 분산 원장에 앵커링
    const tx = await this.blockchainService.anchor({
      hash,
      timestamp: new Date().toISOString(),
      type: 'CONSENT_DOCUMENT',
    });

    return {
      network: 'WIA-CONSENT-CHAIN',
      transactionId: tx.txId,
      blockNumber: tx.blockNumber,
      timestamp: tx.timestamp,
    };
  }
}

interface DocumentUploadInput {
  file: Express.Multer.File;
  documentType: DocumentType;
  title: string;
  description?: string;
  language: string;
  uploadedBy: string;
}

interface DocumentDownload {
  documentId: string;
  data: Buffer;
  contentType: string;
  filename: string;
}

interface IntegrityVerification {
  documentId: string;
  verificationDate: Date;
  primaryHashValid: boolean;
  additionalHashesValid: boolean;
  blockchainAnchorValid: boolean;
  overallValid: boolean;
  details: any;
}
```

---

## 4.6 GraphQL API

```typescript
// GraphQL 스키마
const typeDefs = gql`
  type Query {
    # 동의 쿼리
    consent(id: ID!): Consent
    consents(query: ConsentQueryInput): ConsentConnection!

    # 환자 쿼리
    patientConsents(patientId: ID!): [Consent!]!
    effectiveConsent(
      patientId: ID!
      decisionType: String!
      context: JSON
    ): EffectiveConsentResult

    # 대리인 쿼리
    proxies(patientId: ID!): [Proxy!]!
    proxy(id: ID!): Proxy

    # 문서 쿼리
    consentDocuments(consentId: ID!): [Document!]!
    document(id: ID!): Document
  }

  type Mutation {
    # 동의 뮤테이션
    createConsent(input: CreateConsentInput!): ConsentResult!
    updateConsent(id: ID!, input: UpdateConsentInput!): ConsentResult!
    activateConsent(id: ID!): ConsentResult!
    suspendConsent(id: ID!, reason: String!): ConsentResult!
    revokeConsent(id: ID!, input: RevocationInput!): ConsentResult!

    # 의사결정 뮤테이션
    addDecision(consentId: ID!, input: DecisionInput!): DecisionResult!
    updateDecision(
      consentId: ID!
      decisionId: ID!
      input: DecisionInput!
    ): DecisionResult!

    # 대리인 뮤테이션
    designateProxy(patientId: ID!, input: ProxyInput!): ProxyResult!
    acceptProxy(proxyId: ID!, input: AcceptanceInput!): ProxyResult!
    activateProxy(proxyId: ID!, reason: String!): ProxyResult!
    deactivateProxy(proxyId: ID!, reason: String!): ProxyResult!

    # 문서 뮤테이션
    uploadDocument(consentId: ID!, input: DocumentInput!): DocumentResult!
    deleteDocument(documentId: ID!, reason: String!): DeleteResult!
  }

  type Subscription {
    consentUpdated(patientId: ID!): ConsentUpdate!
    proxyActivated(patientId: ID!): ProxyActivation!
    reviewDue(patientId: ID!): ReviewNotification!
  }

  type Consent {
    id: ID!
    patientId: ID!
    consentType: ConsentType!
    category: ConsentCategory!
    subcategories: [String!]!
    scope: ConsentScope!
    decisions: [Decision!]!
    authority: Authority!
    validity: Validity!
    documents: [Document!]!
    metadata: Metadata!
    auditTrail: AuditTrail
  }

  type ConsentScope {
    procedures: [ProcedureScope!]!
    timeframe: TimeframeScope!
    conditions: [ScopeCondition!]
    limitations: [ScopeLimitation!]
    jurisdictions: [String!]
  }

  type Decision {
    decisionId: ID!
    decisionType: DecisionType!
    question: String!
    answer: DecisionAnswer!
    reasoning: String
    context: DecisionContext
    conditions: [DecisionCondition!]
    metadata: DecisionMetadata!
  }

  type DecisionAnswer {
    type: DecisionType!
    binaryValue: Boolean
    selectedOption: String
    selectedOptions: [String!]
    thresholdValue: Float
    thresholdUnit: String
    rankedOptions: [RankedOption!]
  }

  type Authority {
    grantor: AuthorityEntity!
    witnesses: [Witness!]
    notarization: Notarization
    capacityAssessment: CapacityAssessment
    proxyChain: [ProxyChainEntry!]
  }

  type Validity {
    status: ConsentStatus!
    effectiveDate: DateTime!
    expirationDate: DateTime
    conditions: [ValidityCondition!]
    reviewSchedule: ReviewSchedule
  }

  type Proxy {
    proxyId: ID!
    proxyType: ProxyType!
    individual: IndividualProxy
    organization: OrganizationProxy
    authorityScope: ProxyAuthorityScope!
    acceptanceStatus: AcceptanceStatus!
    acceptanceDate: DateTime
    isActive: Boolean!
  }

  type EffectiveConsentResult {
    found: Boolean!
    decision: Decision
    consent: Consent
    confidence: Float!
    interpretation: String
    alternatives: [Decision!]
    warnings: [String!]
    recommendations: [String!]
  }

  enum ConsentType {
    INITIAL
    UPDATE
    RENEWAL
    REVOCATION
    PROXY
    EMERGENCY
  }

  enum ConsentCategory {
    PRESERVATION
    CARE
    REVIVAL
    RESEARCH
    ASSET
    COMMUNICATION
    PROXY_AUTHORITY
  }

  enum ConsentStatus {
    DRAFT
    PENDING_WITNESS
    PENDING_NOTARIZATION
    PENDING_REVIEW
    ACTIVE
    SUSPENDED
    REVOKED
    EXPIRED
    SUPERSEDED
  }

  enum DecisionType {
    BINARY
    CHOICE
    THRESHOLD
    PREFERENCE
    CONDITIONAL
    DELEGATION
  }

  input CreateConsentInput {
    patientId: ID!
    consentType: ConsentType!
    category: ConsentCategory!
    subcategories: [String!]
    scope: ConsentScopeInput!
    decisions: [DecisionInput!]!
    authority: AuthorityInput
    effectiveDate: DateTime
    expirationDate: DateTime
  }

  input ConsentQueryInput {
    patientId: ID
    consentTypes: [ConsentType!]
    categories: [ConsentCategory!]
    statuses: [ConsentStatus!]
    effectiveAsOf: DateTime
    offset: Int
    limit: Int
    sortBy: ConsentSortField
    sortOrder: SortOrder
  }
`;

// GraphQL 리졸버
const resolvers = {
  Query: {
    consent: async (_: any, { id }: { id: string }, context: Context) => {
      const consent = await context.dataSources.consentService.getConsent(id);
      if (!consent) return null;

      // 권한 확인
      if (!await checkReadAuth(context.user, consent)) {
        throw new ForbiddenError('이 동의를 볼 권한이 없습니다');
      }

      return consent;
    },

    consents: async (_: any, { query }: { query: ConsentQueryInput }, context: Context) => {
      const filteredQuery = applyUserFilters(query, context.user);
      return context.dataSources.consentService.queryConsents(filteredQuery);
    },

    effectiveConsent: async (
      _: any,
      args: { patientId: string; decisionType: string; context: any },
      context: Context
    ) => {
      return context.dataSources.decisionService.queryEffectiveConsent({
        patientId: args.patientId,
        decisionType: args.decisionType,
        context: args.context || {},
      });
    },

    proxies: async (_: any, { patientId }: { patientId: string }, context: Context) => {
      return context.dataSources.proxyService.getProxiesForPatient(patientId);
    },
  },

  Mutation: {
    createConsent: async (
      _: any,
      { input }: { input: CreateConsentInput },
      context: Context
    ) => {
      try {
        const consent = await context.dataSources.consentService.createConsent(
          input,
          { createdBy: context.user.id }
        );
        return { success: true, consent };
      } catch (error) {
        return { success: false, error: error.message };
      }
    },

    designateProxy: async (
      _: any,
      { patientId, input }: { patientId: string; input: ProxyInput },
      context: Context
    ) => {
      // 환자만 지정 가능 확인
      if (context.user.patientId !== patientId && !context.user.isAdmin) {
        throw new ForbiddenError('환자만 대리인을 지정할 수 있습니다');
      }

      const proxy = await context.dataSources.proxyService.designateProxy(
        patientId,
        input,
        { designatedBy: context.user.id }
      );
      return { success: true, proxy };
    },
  },

  Subscription: {
    consentUpdated: {
      subscribe: (_: any, { patientId }: { patientId: string }, context: Context) => {
        return context.pubsub.asyncIterator(`CONSENT_UPDATED_${patientId}`);
      },
    },

    proxyActivated: {
      subscribe: (_: any, { patientId }: { patientId: string }, context: Context) => {
        return context.pubsub.asyncIterator(`PROXY_ACTIVATED_${patientId}`);
      },
    },
  },

  Consent: {
    decisions: (consent: ConsentRecord) => consent.decisions,
    documents: async (consent: ConsentRecord, _: any, context: Context) => {
      return context.dataSources.documentService.getDocumentsForConsent(consent.id);
    },
    auditTrail: async (consent: ConsentRecord, _: any, context: Context) => {
      if (!context.user.canViewAudit) return null;
      return context.dataSources.auditService.getAuditTrail(consent.id);
    },
  },

  Proxy: {
    isActive: async (proxy: ProxyDesignation, _: any, context: Context) => {
      return context.dataSources.proxyService.isProxyActive(proxy.proxyId);
    },
  },
};
```

---

## 4.7 WebSocket 실시간 API

```typescript
// 실시간 동의 업데이트를 위한 WebSocket 서버
class ConsentWebSocketServer {
  private wss: WebSocket.Server;
  private connections: Map<string, WebSocket[]> = new Map();

  constructor(server: http.Server) {
    this.wss = new WebSocket.Server({ server, path: '/ws/consent' });
    this.setupHandlers();
  }

  private setupHandlers(): void {
    this.wss.on('connection', async (ws: WebSocket, req: http.IncomingMessage) => {
      try {
        // 연결 인증
        const token = this.extractToken(req);
        const user = await this.authenticateToken(token);

        // 연결 설정
        this.registerConnection(ws, user);

        // 메시지 처리
        ws.on('message', (data: WebSocket.Data) => {
          this.handleMessage(ws, user, data);
        });

        ws.on('close', () => {
          this.unregisterConnection(ws, user);
        });

        // 연결 확인 전송
        ws.send(JSON.stringify({
          type: 'CONNECTED',
          userId: user.id,
          timestamp: new Date().toISOString(),
        }));
      } catch (error) {
        ws.send(JSON.stringify({
          type: 'ERROR',
          error: '인증 실패',
        }));
        ws.close();
      }
    });
  }

  private handleMessage(ws: WebSocket, user: User, data: WebSocket.Data): void {
    try {
      const message = JSON.parse(data.toString());

      switch (message.type) {
        case 'SUBSCRIBE_PATIENT':
          this.handleSubscribePatient(ws, user, message.patientId);
          break;

        case 'SUBSCRIBE_CONSENT':
          this.handleSubscribeConsent(ws, user, message.consentId);
          break;

        case 'UNSUBSCRIBE':
          this.handleUnsubscribe(ws, user, message.subscriptionId);
          break;

        case 'PING':
          ws.send(JSON.stringify({ type: 'PONG', timestamp: Date.now() }));
          break;

        default:
          ws.send(JSON.stringify({
            type: 'ERROR',
            error: `알 수 없는 메시지 유형: ${message.type}`,
          }));
      }
    } catch (error) {
      ws.send(JSON.stringify({
        type: 'ERROR',
        error: '잘못된 메시지 형식',
      }));
    }
  }

  // 구독 클라이언트에게 동의 업데이트 브로드캐스트
  async broadcastConsentUpdate(
    consentId: string,
    patientId: string,
    update: ConsentUpdate
  ): Promise<void> {
    const subscribers = this.getSubscribers(patientId, consentId);

    const message = JSON.stringify({
      type: 'CONSENT_UPDATE',
      consentId,
      patientId,
      updateType: update.type,
      changes: update.changes,
      timestamp: new Date().toISOString(),
    });

    for (const ws of subscribers) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(message);
      }
    }
  }

  // 대리인 활성화 브로드캐스트
  async broadcastProxyActivation(
    patientId: string,
    proxy: ProxyDesignation
  ): Promise<void> {
    const subscribers = this.getPatientSubscribers(patientId);

    const message = JSON.stringify({
      type: 'PROXY_ACTIVATED',
      patientId,
      proxyId: proxy.proxyId,
      proxyName: proxy.individual?.name || proxy.organization?.name,
      authorities: proxy.authorityScope.categories,
      timestamp: new Date().toISOString(),
    });

    for (const ws of subscribers) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(message);
      }
    }
  }

  // 검토 알림 브로드캐스트
  async broadcastReviewReminder(
    patientId: string,
    consentId: string,
    reviewDue: Date
  ): Promise<void> {
    const subscribers = this.getPatientSubscribers(patientId);

    const message = JSON.stringify({
      type: 'REVIEW_REMINDER',
      patientId,
      consentId,
      reviewDue: reviewDue.toISOString(),
      daysUntilDue: Math.ceil(
        (reviewDue.getTime() - Date.now()) / (24 * 60 * 60 * 1000)
      ),
      timestamp: new Date().toISOString(),
    });

    for (const ws of subscribers) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(message);
      }
    }
  }

  // 구독 요청 처리
  private handleSubscribePatient(
    ws: WebSocket,
    user: User,
    patientId: string
  ): void {
    // 권한 확인
    if (!this.canSubscribeToPatient(user, patientId)) {
      ws.send(JSON.stringify({
        type: 'SUBSCRIPTION_ERROR',
        error: '이 환자를 구독할 권한이 없습니다',
      }));
      return;
    }

    // 구독자 추가
    const key = `patient:${patientId}`;
    if (!this.connections.has(key)) {
      this.connections.set(key, []);
    }
    this.connections.get(key)!.push(ws);

    ws.send(JSON.stringify({
      type: 'SUBSCRIBED',
      subscriptionType: 'PATIENT',
      patientId,
      subscriptionId: `${key}:${Date.now()}`,
    }));
  }

  private canSubscribeToPatient(user: User, patientId: string): boolean {
    // 환자는 자신의 업데이트를 구독할 수 있음
    if (user.patientId === patientId) return true;

    // 적절한 역할의 직원
    if (user.roles.includes('MEDICAL_STAFF') || user.roles.includes('ADMIN')) {
      return true;
    }

    // 환자의 활성 대리인
    // (실제 구현에서는 대리인 상태 확인)
    return false;
  }
}

interface ConsentUpdate {
  type: 'CREATED' | 'UPDATED' | 'ACTIVATED' | 'SUSPENDED' | 'REVOKED';
  changes: Record<string, any>;
}
```

---

## 4.8 API 응답 형식

```typescript
// 표준 API 응답 형식
interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  meta?: ResponseMeta;
  links?: ResponseLinks;
}

interface ApiError {
  code: string;
  message: string;
  details?: any;
  stack?: string;  // 개발 환경에서만
}

interface ResponseMeta {
  requestId: string;
  timestamp: string;
  version: string;
  processingTime: number;
}

interface ResponseLinks {
  self: string;
  [key: string]: string | null;
}

// 응답 직렬화기
class ConsentResponseSerializer {
  serializeConsent(
    consent: ConsentRecord,
    include?: ConsentIncludeOptions
  ): SerializedConsent {
    const base = {
      id: consent.id,
      patientId: consent.patientId,
      consentType: consent.consentType,
      category: consent.category,
      subcategories: consent.subcategories,

      scope: this.serializeScope(consent.scope),
      decisions: consent.decisions.map(d => this.serializeDecision(d)),

      validity: {
        status: consent.validity.status,
        effectiveDate: consent.validity.effectiveDate.toISOString(),
        expirationDate: consent.validity.expirationDate?.toISOString() || null,
      },

      metadata: {
        version: consent.metadata.version,
        createdAt: consent.metadata.createdAt.toISOString(),
        updatedAt: consent.metadata.updatedAt.toISOString(),
      },
    };

    // 선택적 필드 포함
    if (include?.fullAuthority) {
      base['authority'] = this.serializeAuthority(consent.authority);
    }

    if (include?.documents) {
      base['documents'] = consent.documents.map(d => this.serializeDocument(d));
    }

    if (include?.statusHistory) {
      base['statusHistory'] = consent.validity.statusHistory;
    }

    return base;
  }

  serializeConsentSummary(consent: ConsentRecord): ConsentSummary {
    return {
      id: consent.id,
      patientId: consent.patientId,
      consentType: consent.consentType,
      category: consent.category,
      status: consent.validity.status,
      effectiveDate: consent.validity.effectiveDate.toISOString(),
      decisionCount: consent.decisions.length,
      version: consent.metadata.version,
      updatedAt: consent.metadata.updatedAt.toISOString(),
    };
  }

  private serializeScope(scope: ConsentScope): any {
    return {
      procedures: scope.procedures.map(p => ({
        procedureType: p.procedureType,
        specificProcedures: p.specificProcedures,
        exclusions: p.exclusions,
      })),
      timeframe: {
        type: scope.timeframe.type,
        startDate: scope.timeframe.startDate?.toISOString(),
        endDate: scope.timeframe.endDate?.toISOString(),
        duration: scope.timeframe.duration,
      },
      conditions: scope.conditions?.map(c => ({
        conditionId: c.conditionId,
        description: c.description,
      })),
      limitations: scope.limitations?.map(l => ({
        limitationType: l.limitationType,
        description: l.description,
        hardLimit: l.hardLimit,
      })),
      jurisdictions: scope.jurisdictions,
    };
  }

  private serializeDecision(decision: ConsentDecision): any {
    return {
      decisionId: decision.decisionId,
      decisionType: decision.decisionType,
      question: decision.question,
      answer: this.serializeAnswer(decision.answer),
      reasoning: decision.reasoning,
      confidence: decision.metadata.confidenceLevel,
    };
  }

  private serializeAnswer(answer: DecisionAnswer): any {
    const base: any = { type: answer.type };

    switch (answer.type) {
      case 'BINARY':
        base.value = answer.binaryValue;
        break;
      case 'CHOICE':
        base.selected = answer.selectedOption || answer.selectedOptions;
        break;
      case 'THRESHOLD':
        base.value = answer.thresholdValue;
        base.unit = answer.thresholdUnit;
        base.operator = answer.thresholdOperator;
        break;
      case 'PREFERENCE':
        base.ranked = answer.rankedOptions;
        break;
      case 'CONDITIONAL':
        base.rules = answer.conditionalRules;
        break;
      case 'DELEGATION':
        base.delegateTo = answer.delegateTo;
        break;
    }

    return base;
  }

  private serializeAuthority(authority: ConsentAuthority): any {
    return {
      grantor: {
        entityId: authority.grantor.entityId,
        entityType: authority.grantor.entityType,
        capacityConfirmed: authority.grantor.capacityConfirmed,
        capacityDate: authority.grantor.capacityDate.toISOString(),
      },
      witnesses: authority.witnesses?.map(w => ({
        witnessId: w.witnessId,
        name: w.name,
        role: w.role,
        attestationDate: w.attestation.attestationDate.toISOString(),
      })),
      notarization: authority.notarization ? {
        notaryName: authority.notarization.notaryName,
        jurisdiction: authority.notarization.jurisdiction,
        notarizationDate: authority.notarization.notarizationDate.toISOString(),
      } : null,
    };
  }

  private serializeDocument(document: ConsentDocument): any {
    return {
      documentId: document.documentId,
      documentType: document.documentType,
      title: document.title,
      language: document.language,
      version: document.version.versionNumber,
      createdAt: document.version.versionDate.toISOString(),
      integrityVerified: true,  // 실제 검증 확인 필요
    };
  }
}

interface SerializedConsent {
  id: string;
  patientId: string;
  consentType: ConsentType;
  category: ConsentCategory;
  subcategories: string[];
  scope: any;
  decisions: any[];
  validity: any;
  metadata: any;
  authority?: any;
  documents?: any[];
  statusHistory?: any[];
}

interface ConsentSummary {
  id: string;
  patientId: string;
  consentType: ConsentType;
  category: ConsentCategory;
  status: ConsentStatus;
  effectiveDate: string;
  decisionCount: number;
  version: number;
  updatedAt: string;
}
```

---

*다음 장: 제어 프로토콜 - 동의 생명주기 관리 및 의사결정 워크플로우*
