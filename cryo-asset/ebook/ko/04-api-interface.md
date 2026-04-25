# 제4장: API 인터페이스

## 서비스 통합 사양

### 소개

WIA Cryo-Asset API는 냉동보존 환자 자산, 신탁, 포트폴리오 및 관련 금융 운영을 관리하기 위한 포괄적인 RESTful 인터페이스를 제공합니다. 이 장에서는 엔드포인트, 인증, 요청/응답 형식 및 외부 금융 시스템과의 연결을 위한 통합 패턴을 포함한 완전한 API 설계를 명시합니다.

---

## 4.1 API 아키텍처

### 설계 원칙

```typescript
// API 구성 및 아키텍처
interface CryoAssetApiConfig {
  version: 'v1';
  baseUrl: string;
  environment: 'production' | 'staging' | 'development';

  authentication: {
    methods: ['api_key', 'oauth2', 'jwt'];
    oauth2Provider: string;
    tokenEndpoint: string;
  };

  rateLimit: {
    requestsPerMinute: number;
    burstLimit: number;
    quotaPerDay: number;
  };

  features: {
    pagination: boolean;      // 페이지네이션
    filtering: boolean;       // 필터링
    sorting: boolean;         // 정렬
    fieldSelection: boolean;  // 필드 선택
    webhooks: boolean;        // 웹훅
    asyncOperations: boolean; // 비동기 작업
  };
}

// API 엔드포인트 구조
const apiStructure = {
  '/api/v1': {
    '/patients': '환자 관리',
    '/portfolios': '포트폴리오 운영',
    '/assets': '자산 레지스트리',
    '/trusts': '신탁 관리',
    '/transactions': '거래 내역',
    '/valuations': '평가 서비스',
    '/documents': '문서 관리',
    '/reports': '보고 및 분석',
    '/integrations': '외부 시스템 통합',
    '/webhooks': '이벤트 구독',
  },
};
```

### OpenAPI 사양

```yaml
openapi: 3.0.3
info:
  title: WIA Cryo-Asset 관리 API
  version: 1.0.0
  description: |
    냉동보존 환자 자산, 신탁 및 포트폴리오 관리를 위한 포괄적인 API.
    냉동보존된 개인을 위한 장기 자산 보존, 평가 추적 및
    다세대 부 관리를 지원합니다.
  contact:
    name: WIA 냉동보존 표준 위원회
    email: cryonics@wia-standards.org

servers:
  - url: https://api.cryo-asset.wia-standards.org/v1
    description: 프로덕션 서버
  - url: https://staging-api.cryo-asset.wia-standards.org/v1
    description: 스테이징 서버

security:
  - bearerAuth: []
  - apiKeyAuth: []

tags:
  - name: Patients
    description: 환자 신원 및 상태 관리
  - name: Portfolios
    description: 포트폴리오 수명주기 및 평가
  - name: Assets
    description: 자산 등록 및 추적
  - name: Trusts
    description: 신탁 생성 및 관리
  - name: Transactions
    description: 금융 거래 기록
```

---

## 4.2 환자 엔드포인트

### 환자 관리 API

```typescript
// 환자 API 구현
import { Router, Request, Response } from 'express';
import { z } from 'zod';

const patientRouter = Router();

// GET /api/v1/patients
// 필터링 및 페이지네이션으로 모든 환자 목록 조회
patientRouter.get('/', async (req: Request, res: Response) => {
  const querySchema = z.object({
    page: z.coerce.number().min(1).default(1),
    pageSize: z.coerce.number().min(1).max(100).default(20),
    status: z.nativeEnum(PreservationStatus).optional(),
    organizationId: z.string().uuid().optional(),
    search: z.string().optional(),
    sortBy: z.enum(['name', 'createdAt', 'preservationDate', 'portfolioValue'])
      .default('name'),
    sortOrder: z.enum(['asc', 'desc']).default('asc'),
  });

  try {
    const query = querySchema.parse(req.query);

    const patients = await patientService.listPatients({
      page: query.page,
      pageSize: query.pageSize,
      filters: {
        status: query.status,
        organizationId: query.organizationId,
        search: query.search,
      },
      sort: {
        field: query.sortBy,
        order: query.sortOrder,
      },
    });

    res.json({
      success: true,
      data: patients.items.map(formatPatientSummary),
      pagination: {
        page: query.page,
        pageSize: query.pageSize,
        totalItems: patients.totalCount,
        totalPages: Math.ceil(patients.totalCount / query.pageSize),
        hasNext: query.page * query.pageSize < patients.totalCount,
        hasPrevious: query.page > 1,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// GET /api/v1/patients/:id
// 환자 상세 정보 조회
patientRouter.get('/:id', async (req: Request, res: Response) => {
  try {
    const patient = await patientService.getPatient(req.params.id);

    if (!patient) {
      return res.status(404).json({
        success: false,
        error: {
          code: 'PATIENT_NOT_FOUND',
          message: `ID가 ${req.params.id}인 환자를 찾을 수 없습니다`,
        },
      });
    }

    // 관련 데이터 조회
    const [portfolio, trusts, recentActivity] = await Promise.all([
      portfolioService.getPatientPortfolio(patient.id),
      trustService.getPatientTrusts(patient.id),
      activityService.getRecentActivity(patient.id, 10),
    ]);

    res.json({
      success: true,
      data: {
        ...formatPatientDetail(patient),
        portfolio: portfolio ? formatPortfolioSummary(portfolio) : null,
        trusts: trusts.map(formatTrustSummary),
        recentActivity: recentActivity.map(formatActivity),
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/patients
// 신규 환자 생성
patientRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    organizationPatientId: z.string().min(1).max(50),
    legalIdentity: z.object({
      fullLegalName: z.string().min(2).max(200),
      dateOfBirth: z.string().datetime(),
      placeOfBirth: z.string().max(200),
      citizenship: z.array(z.string().length(2)).min(1),
    }),
    preservationStatus: z.object({
      status: z.nativeEnum(PreservationStatus),
      preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
      preservationDate: z.string().datetime().optional(),
    }),
  });

  try {
    const data = createSchema.parse(req.body);

    const patient = await patientService.createPatient({
      organizationId: req.user.organizationId,
      ...data,
    });

    // 포트폴리오 자동 생성
    const portfolio = await portfolioService.createPortfolio(patient.id);

    res.status(201).json({
      success: true,
      data: {
        patient: formatPatientDetail(patient),
        portfolio: formatPortfolioSummary(portfolio),
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/patients/:id/preservation
// 보존 이벤트 기록
patientRouter.post('/:id/preservation', async (req: Request, res: Response) => {
  const preservationSchema = z.object({
    preservationDate: z.string().datetime(),
    preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
    location: z.string(),
    procedureDetails: z.object({
      team: z.array(z.string()),
      protocols: z.array(z.string()),
      notes: z.string().optional(),
    }).optional(),
  });

  try {
    const data = preservationSchema.parse(req.body);

    const result = await patientService.recordPreservation(req.params.id, {
      ...data,
      preservationDate: new Date(data.preservationDate),
    });

    // 포트폴리오 상태 업데이트 트리거
    await portfolioService.updatePreservationStatus(
      result.patient.id,
      'PRESERVED'
    );

    // 이벤트 기록
    await eventService.recordEvent({
      type: EventType.PATIENT_PRESERVED,
      subject: { type: 'PATIENT', id: req.params.id },
      details: data,
    });

    res.json({
      success: true,
      data: {
        patient: formatPatientDetail(result.patient),
        preservationRecord: result.preservationRecord,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

export { patientRouter };
```

---

## 4.3 포트폴리오 엔드포인트

### 포트폴리오 관리 API

```typescript
// 포트폴리오 API 구현
const portfolioRouter = Router();

// GET /api/v1/portfolios/:id
// 포트폴리오 상세 정보 조회
portfolioRouter.get('/:id', async (req: Request, res: Response) => {
  try {
    const portfolio = await portfolioService.getPortfolio(req.params.id);

    if (!portfolio) {
      return res.status(404).json({
        success: false,
        error: {
          code: 'PORTFOLIO_NOT_FOUND',
          message: `포트폴리오 ${req.params.id}를 찾을 수 없습니다`,
        },
      });
    }

    // 상세 보유 및 성과 조회
    const [holdings, performance, allocation] = await Promise.all([
      portfolioService.getHoldings(portfolio.id),
      portfolioService.getPerformance(portfolio.id),
      portfolioService.getAllocation(portfolio.id),
    ]);

    res.json({
      success: true,
      data: {
        ...formatPortfolioDetail(portfolio),
        preservationFund: formatFundDetail(portfolio.preservationFund),
        revivalFund: formatFundDetail(portfolio.revivalFund),
        holdings: holdings.map(formatHolding),
        performance: formatPerformance(performance),
        allocation: formatAllocation(allocation),
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/portfolios/:id/valuation
// 신규 평가 수행
portfolioRouter.post('/:id/valuation', async (req: Request, res: Response) => {
  const valuationSchema = z.object({
    valuationDate: z.string().datetime()
      .default(() => new Date().toISOString()),
    method: z.enum(['FULL', 'QUICK', 'CUSTOM']).default('FULL'),
    overrides: z.array(z.object({
      assetId: z.string().uuid(),
      value: z.number().nonnegative(),
      method: z.nativeEnum(ValuationMethod),
      source: z.string(),
    })).optional(),
  });

  try {
    const data = valuationSchema.parse(req.body);

    const valuation = await valuationService.performValuation(req.params.id, {
      valuationDate: new Date(data.valuationDate),
      method: data.method,
      overrides: data.overrides,
      performedBy: req.user.id,
    });

    // 이벤트 기록
    await eventService.recordEvent({
      type: EventType.PORTFOLIO_VALUED,
      subject: { type: 'PORTFOLIO', id: req.params.id },
      details: { totalValue: valuation.totalValue },
    });

    res.json({
      success: true,
      data: formatValuation(valuation),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// GET /api/v1/portfolios/:id/performance
// 포트폴리오 성과 지표 조회
portfolioRouter.get('/:id/performance', async (req: Request, res: Response) => {
  const querySchema = z.object({
    period: z.enum(['MTD', 'QTD', 'YTD', '1Y', '3Y', '5Y', '10Y', 'ITD'])
      .default('YTD'),
    benchmark: z.string().optional(),
  });

  try {
    const query = querySchema.parse(req.query);

    const performance = await portfolioService.calculatePerformance(
      req.params.id,
      query.period,
      query.benchmark
    );

    res.json({
      success: true,
      data: {
        period: query.period,
        returns: {
          totalReturn: performance.totalReturn,
          annualizedReturn: performance.annualizedReturn,
          absoluteReturn: performance.absoluteReturn,
        },
        risk: {
          volatility: performance.volatility,
          sharpeRatio: performance.sharpeRatio,
          maxDrawdown: performance.maxDrawdown,
          beta: performance.beta,
        },
        comparison: query.benchmark ? {
          benchmark: query.benchmark,
          benchmarkReturn: performance.benchmarkReturn,
          alpha: performance.alpha,
        } : null,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/portfolios/:id/rebalance
// 포트폴리오 리밸런싱 실행
portfolioRouter.post('/:id/rebalance', async (req: Request, res: Response) => {
  const rebalanceSchema = z.object({
    targetAllocation: z.array(z.object({
      assetClass: z.string(),
      targetWeight: z.number().min(0).max(100),
    })).optional(),
    mode: z.enum(['PREVIEW', 'EXECUTE']).default('PREVIEW'),
    constraints: z.object({
      maxTurnover: z.number().min(0).max(100).optional(),
      taxLotMethod: z.enum(['FIFO', 'LIFO', 'HIFO', 'TAX_OPTIMAL']).optional(),
    }).optional(),
  });

  try {
    const data = rebalanceSchema.parse(req.body);

    // 현재 포트폴리오 및 정책 조회
    const portfolio = await portfolioService.getPortfolio(req.params.id);
    const currentAllocation = await portfolioService.getAllocation(portfolio.id);
    const targetAllocation = data.targetAllocation ||
      await portfolioService.getTargetAllocation(portfolio.id);

    // 리밸런싱 거래 계산
    const rebalancePlan = await rebalanceService.calculateRebalancing({
      portfolioId: portfolio.id,
      currentAllocation,
      targetAllocation,
      constraints: data.constraints,
    });

    if (data.mode === 'EXECUTE') {
      // 거래 실행
      const execution = await rebalanceService.executeRebalancing(rebalancePlan);

      res.json({
        success: true,
        data: {
          status: 'EXECUTED',
          execution: formatRebalanceExecution(execution),
        },
      });
    } else {
      res.json({
        success: true,
        data: {
          status: 'PREVIEW',
          plan: formatRebalancePlan(rebalancePlan),
        },
      });
    }
  } catch (error) {
    handleApiError(res, error);
  }
});
```

---

## 4.4 자산 엔드포인트

### 자산 관리 API

```typescript
// 자산 API 구현
const assetRouter = Router();

// POST /api/v1/assets
// 신규 자산 등록
assetRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    portfolioId: z.string().uuid(),
    category: z.nativeEnum(AssetCategory),
    type: z.string(),
    subtype: z.string().optional(),
    name: z.string().min(1).max(500),
    description: z.string().max(5000).optional(),

    ownership: z.object({
      type: z.nativeEnum(OwnershipType),
      holders: z.array(z.object({
        entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION']),
        entityId: z.string(),
        name: z.string(),
        percentage: z.number().min(0).max(100),
      })),
    }),

    acquisition: z.object({
      date: z.string().datetime(),
      cost: z.number().nonnegative(),
      currency: z.string().length(3),
      method: z.string(),
    }),

    currentValue: z.object({
      value: z.number().nonnegative(),
      currency: z.string().length(3),
      valuationMethod: z.nativeEnum(ValuationMethod),
      valuationDate: z.string().datetime(),
    }),
  });

  try {
    const data = createSchema.parse(req.body);

    // 포트폴리오 존재 및 권한 확인
    const portfolio = await portfolioService.getPortfolio(data.portfolioId);
    if (!portfolio || portfolio.organizationId !== req.user.organizationId) {
      return res.status(403).json({
        success: false,
        error: { code: 'FORBIDDEN', message: '포트폴리오 접근 거부' },
      });
    }

    const asset = await assetService.registerAsset({
      ...data,
      registeredBy: req.user.id,
      organizationId: req.user.organizationId,
    });

    // 블록체인에 등록
    const blockchainRef = await blockchainService.registerAsset(asset);
    await assetService.updateBlockchainRef(asset.id, blockchainRef);

    res.status(201).json({
      success: true,
      data: formatAssetDetail(asset),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/assets/:id/transfer
// 자산 소유권 이전
assetRouter.post('/:id/transfer', async (req: Request, res: Response) => {
  const transferSchema = z.object({
    targetPortfolioId: z.string().uuid().optional(),
    newOwnership: z.object({
      type: z.nativeEnum(OwnershipType),
      holders: z.array(z.object({
        entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION']),
        entityId: z.string(),
        name: z.string(),
        percentage: z.number().min(0).max(100),
      })),
    }),
    transferDate: z.string().datetime(),
    transferValue: z.number().nonnegative(),
    transferType: z.enum(['GIFT', 'SALE', 'INHERITANCE', 'TRUST_FUNDING', 'OTHER']),
    documentation: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })),
  });

  try {
    const data = transferSchema.parse(req.body);

    const transfer = await assetService.transferAsset(req.params.id, {
      ...data,
      transferDate: new Date(data.transferDate),
      initiatedBy: req.user.id,
    });

    // 블록체인 이전 기록
    await blockchainService.recordAssetTransfer(transfer);

    res.json({
      success: true,
      data: formatTransfer(transfer),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});
```

---

## 4.5 신탁 엔드포인트

### 신탁 관리 API

```typescript
// 신탁 API 구현
const trustRouter = Router();

// POST /api/v1/trusts
// 신규 신탁 생성
trustRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    name: z.string().min(1).max(500),
    trustType: z.nativeEnum(TrustType),

    jurisdiction: z.object({
      state: z.string().optional(),
      country: z.string().length(2),
      governingLaw: z.string(),
      situs: z.string(),
    }),

    parties: z.object({
      grantor: z.object({
        entityType: z.enum(['INDIVIDUAL', 'CORPORATION', 'TRUST']),
        name: z.string(),
        identifier: z.string(),
      }),

      trustees: z.array(z.object({
        entityType: z.enum(['INDIVIDUAL', 'CORPORATE', 'PROFESSIONAL']),
        name: z.string(),
        powers: z.array(z.string()),
        compensation: z.object({
          type: z.enum(['FIXED', 'PERCENTAGE', 'STATUTORY', 'NONE']),
          amount: z.number().optional(),
        }),
      })).min(1),

      beneficiaries: z.array(z.object({
        beneficiaryType: z.nativeEnum(BeneficiaryType),
        patientId: z.string().uuid().optional(),
        name: z.string().optional(),
        interestType: z.enum(['INCOME', 'PRINCIPAL', 'BOTH', 'CONTINGENT']),
      })).min(1),
    }),

    terms: z.object({
      purpose: z.string(),
      duration: z.object({
        type: z.enum(['PERPETUAL', 'TERM', 'LIFE', 'UNTIL_EVENT']),
        termYears: z.number().optional(),
      }),
      revivalProvisions: z.object({
        revivalDefinition: z.string(),
        identityVerificationMethod: z.string(),
        distributionUponRevival: z.object({
          type: z.string(),
          amount: z.string(),
        }),
      }).optional(),
      spendthrift: z.boolean().default(true),
    }),
  });

  try {
    const data = createSchema.parse(req.body);

    const trust = await trustService.createTrust({
      ...data,
      createdBy: req.user.id,
      organizationId: req.user.organizationId,
    });

    // 신탁 문서 초안 생성
    const trustDocument = await documentService.generateTrustDocument(trust);

    res.status(201).json({
      success: true,
      data: {
        trust: formatTrustDetail(trust),
        documentDraft: {
          id: trustDocument.id,
          status: 'DRAFT',
          downloadUrl: trustDocument.downloadUrl,
        },
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/trusts/:id/fund
// 신탁 자금 조달
trustRouter.post('/:id/fund', async (req: Request, res: Response) => {
  const fundingSchema = z.object({
    amount: z.number().positive(),
    currency: z.string().length(3),
    fundingSource: z.object({
      type: z.enum(['CASH', 'SECURITIES', 'INSURANCE', 'PROPERTY', 'OTHER']),
      description: z.string(),
    }),
    fundingDate: z.string().datetime(),
    assets: z.array(z.object({
      assetId: z.string().uuid(),
      transferValue: z.number().nonnegative(),
    })).optional(),
  });

  try {
    const data = fundingSchema.parse(req.body);

    const funding = await trustService.fundTrust(req.params.id, {
      ...data,
      fundingDate: new Date(data.fundingDate),
      processedBy: req.user.id,
    });

    // 이벤트 기록
    await eventService.recordEvent({
      type: EventType.TRUST_FUNDED,
      subject: { type: 'TRUST', id: req.params.id },
      details: { amount: data.amount, currency: data.currency },
    });

    res.json({
      success: true,
      data: formatFunding(funding),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/trusts/:id/distribution
// 신탁 배분 기록
trustRouter.post('/:id/distribution', async (req: Request, res: Response) => {
  const distributionSchema = z.object({
    beneficiaryId: z.string(),
    amount: z.number().positive(),
    currency: z.string().length(3),
    distributionType: z.enum(['INCOME', 'PRINCIPAL', 'BOTH']),
    distributionReason: z.string(),
    distributionDate: z.string().datetime(),
    paymentMethod: z.enum(['CHECK', 'WIRE', 'ACH', 'CASH', 'IN_KIND']),
  });

  try {
    const data = distributionSchema.parse(req.body);

    const distribution = await trustService.recordDistribution(req.params.id, {
      ...data,
      distributionDate: new Date(data.distributionDate),
      processedBy: req.user.id,
    });

    res.json({
      success: true,
      data: formatDistribution(distribution),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});
```

---

## 4.6 웹훅 구성

### 웹훅 관리

```typescript
// 웹훅 API 구현
const webhookRouter = Router();

// 지원되는 웹훅 이벤트
const WebhookEvents = {
  // 환자 이벤트
  'patient.created': '신규 환자 등록됨',
  'patient.preserved': '환자 보존 기록됨',
  'patient.revived': '환자 소생 기록됨',
  'patient.updated': '환자 정보 업데이트됨',

  // 포트폴리오 이벤트
  'portfolio.valued': '포트폴리오 평가 완료됨',
  'portfolio.rebalanced': '포트폴리오 리밸런싱 실행됨',
  'portfolio.threshold_alert': '포트폴리오 임계값 초과',

  // 자산 이벤트
  'asset.registered': '신규 자산 등록됨',
  'asset.valued': '자산 평가 업데이트됨',
  'asset.transferred': '자산 소유권 이전됨',

  // 신탁 이벤트
  'trust.created': '신규 신탁 생성됨',
  'trust.funded': '신탁 자금 조달됨',
  'trust.distribution': '신탁 배분 실행됨',

  // 규정 준수 이벤트
  'compliance.review_due': '규정 준수 검토 예정',
  'compliance.violation': '규정 준수 위반 감지됨',
};

// POST /api/v1/webhooks
webhookRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    url: z.string().url(),
    events: z.array(z.enum(Object.keys(WebhookEvents) as [string, ...string[]])),
    secret: z.string().min(32).optional(),
    headers: z.record(z.string()).optional(),
  });

  try {
    const data = createSchema.parse(req.body);

    // 비밀 키 생성 (제공되지 않은 경우)
    const secret = data.secret || crypto.randomBytes(32).toString('hex');

    const webhook = await webhookService.createWebhook({
      ...data,
      secret,
      organizationId: req.user.organizationId,
    });

    res.status(201).json({
      success: true,
      data: {
        id: webhook.id,
        url: webhook.url,
        events: webhook.events,
        secret: secret,  // 생성 시에만 반환
        status: webhook.status,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// 웹훅 페이로드 구조
interface WebhookPayload {
  id: string;
  event: string;
  timestamp: string;
  organizationId: string;
  data: Record<string, any>;
  metadata: {
    apiVersion: string;
    deliveryAttempt: number;
  };
}

// 웹훅 서명 검증
function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(`sha256=${expectedSignature}`)
  );
}
```

---

## 4.7 오류 처리 및 응답 형식

### 표준화된 오류 응답

```typescript
// 오류 처리 구현
interface ApiErrorResponse {
  success: false;
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    validationErrors?: ValidationError[];
  };
  requestId: string;
  timestamp: string;
}

// 오류 코드
const ErrorCodes = {
  // 인증/권한
  UNAUTHORIZED: { status: 401, message: '인증이 필요합니다' },
  FORBIDDEN: { status: 403, message: '접근이 거부되었습니다' },
  TOKEN_EXPIRED: { status: 401, message: '인증 토큰이 만료되었습니다' },

  // 리소스 오류
  NOT_FOUND: { status: 404, message: '리소스를 찾을 수 없습니다' },
  PATIENT_NOT_FOUND: { status: 404, message: '환자를 찾을 수 없습니다' },
  PORTFOLIO_NOT_FOUND: { status: 404, message: '포트폴리오를 찾을 수 없습니다' },
  ASSET_NOT_FOUND: { status: 404, message: '자산을 찾을 수 없습니다' },
  TRUST_NOT_FOUND: { status: 404, message: '신탁을 찾을 수 없습니다' },

  // 검증 오류
  VALIDATION_ERROR: { status: 400, message: '검증 오류' },
  INVALID_INPUT: { status: 400, message: '잘못된 입력' },

  // 비즈니스 로직 오류
  INSUFFICIENT_FUNDS: { status: 400, message: '자금 부족' },
  DUPLICATE_ENTRY: { status: 409, message: '중복 항목' },

  // 서버 오류
  INTERNAL_ERROR: { status: 500, message: '내부 서버 오류' },
  SERVICE_UNAVAILABLE: { status: 503, message: '서비스를 일시적으로 사용할 수 없습니다' },
};

// 오류 처리 미들웨어
function handleApiError(res: Response, error: unknown): void {
  const requestId = res.locals.requestId || generateRequestId();

  if (error instanceof z.ZodError) {
    return res.status(400).json({
      success: false,
      error: {
        code: 'VALIDATION_ERROR',
        message: '검증에 실패했습니다',
        validationErrors: error.errors.map(e => ({
          path: e.path.join('.'),
          message: e.message,
        })),
      },
      requestId,
      timestamp: new Date().toISOString(),
    });
  }

  if (error instanceof ApiError) {
    return res.status(error.statusCode).json({
      success: false,
      error: {
        code: error.code,
        message: error.message,
        details: error.details,
      },
      requestId,
      timestamp: new Date().toISOString(),
    });
  }

  // 예기치 않은 오류 로깅
  console.error('예기치 않은 오류:', error);

  return res.status(500).json({
    success: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: '예기치 않은 오류가 발생했습니다',
    },
    requestId,
    timestamp: new Date().toISOString(),
  });
}
```

---

## 장 요약

이 장에서는 WIA Cryo-Asset 관리 플랫폼을 위한 완전한 API 인터페이스를 정의했습니다:

1. **API 아키텍처**: OpenAPI 3.0 사양을 갖춘 RESTful 설계
2. **환자 엔드포인트**: 전체 CRUD 작업 및 보존 기록
3. **포트폴리오 엔드포인트**: 평가, 성과 및 리밸런싱 작업
4. **자산 엔드포인트**: 등록, 평가 업데이트 및 이전
5. **신탁 엔드포인트**: 생성, 자금 조달 및 배분 관리
6. **웹훅 구성**: 이벤트 구독 및 전달 관리
7. **오류 처리**: 표준화된 오류 응답 및 코드

API는 적절한 인증, 검증 및 통합 기능을 갖춘 냉동보존 자산 관리 애플리케이션을 구축하기 위한 포괄적인 인터페이스를 제공합니다.

---

*다음 장: 제어 프로토콜 - 자산 거버넌스 및 관리 규칙*
