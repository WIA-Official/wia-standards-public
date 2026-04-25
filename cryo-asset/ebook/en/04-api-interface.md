# Chapter 4: API Interface

## Service Integration Specifications

### Introduction

The WIA Cryo-Asset API provides a comprehensive RESTful interface for managing cryonics patient assets, trusts, portfolios, and related financial operations. This chapter specifies the complete API design, including endpoints, authentication, request/response formats, and integration patterns for connecting with external financial systems.

---

## 4.1 API Architecture

### Design Principles

```typescript
// API configuration and architecture
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
    pagination: boolean;
    filtering: boolean;
    sorting: boolean;
    fieldSelection: boolean;
    webhooks: boolean;
    asyncOperations: boolean;
  };
}

// API endpoint structure
const apiStructure = {
  '/api/v1': {
    '/patients': 'Patient management',
    '/portfolios': 'Portfolio operations',
    '/assets': 'Asset registry',
    '/trusts': 'Trust management',
    '/transactions': 'Transaction history',
    '/valuations': 'Valuation services',
    '/documents': 'Document management',
    '/reports': 'Reporting and analytics',
    '/integrations': 'External system integrations',
    '/webhooks': 'Event subscriptions',
  },
};
```

### OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: WIA Cryo-Asset Management API
  version: 1.0.0
  description: |
    Comprehensive API for managing cryonics patient assets, trusts, and portfolios.
    Supports long-term asset preservation, valuation tracking, and multi-generational
    wealth management for cryopreserved individuals.
  contact:
    name: WIA Cryonics Standards Committee
    email: cryonics@wia-standards.org
  license:
    name: Apache 2.0
    url: https://www.apache.org/licenses/LICENSE-2.0

servers:
  - url: https://api.cryo-asset.wia-standards.org/v1
    description: Production server
  - url: https://staging-api.cryo-asset.wia-standards.org/v1
    description: Staging server

security:
  - bearerAuth: []
  - apiKeyAuth: []

tags:
  - name: Patients
    description: Patient identity and status management
  - name: Portfolios
    description: Portfolio lifecycle and valuation
  - name: Assets
    description: Asset registration and tracking
  - name: Trusts
    description: Trust creation and administration
  - name: Transactions
    description: Financial transaction recording
  - name: Valuations
    description: Asset and portfolio valuation
  - name: Documents
    description: Document storage and retrieval
  - name: Integrations
    description: External system connections

components:
  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
    apiKeyAuth:
      type: apiKey
      in: header
      name: X-API-Key

  schemas:
    Error:
      type: object
      properties:
        code:
          type: string
        message:
          type: string
        details:
          type: object
        requestId:
          type: string

    PaginatedResponse:
      type: object
      properties:
        data:
          type: array
        pagination:
          $ref: '#/components/schemas/Pagination'

    Pagination:
      type: object
      properties:
        page:
          type: integer
        pageSize:
          type: integer
        totalItems:
          type: integer
        totalPages:
          type: integer
```

---

## 4.2 Patient Endpoints

### Patient Management API

```typescript
// Patient API implementation
import { Router, Request, Response, NextFunction } from 'express';
import { z } from 'zod';

const patientRouter = Router();

// GET /api/v1/patients
// List all patients with filtering and pagination
patientRouter.get('/', async (req: Request, res: Response) => {
  const querySchema = z.object({
    page: z.coerce.number().min(1).default(1),
    pageSize: z.coerce.number().min(1).max(100).default(20),
    status: z.nativeEnum(PreservationStatus).optional(),
    organizationId: z.string().uuid().optional(),
    search: z.string().optional(),
    sortBy: z.enum(['name', 'createdAt', 'preservationDate', 'portfolioValue']).default('name'),
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
// Get patient details
patientRouter.get('/:id', async (req: Request, res: Response) => {
  try {
    const patient = await patientService.getPatient(req.params.id);

    if (!patient) {
      return res.status(404).json({
        success: false,
        error: {
          code: 'PATIENT_NOT_FOUND',
          message: `Patient with ID ${req.params.id} not found`,
        },
      });
    }

    // Get related data
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
// Create new patient
patientRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    organizationPatientId: z.string().min(1).max(50),
    legalIdentity: z.object({
      fullLegalName: z.string().min(2).max(200),
      dateOfBirth: z.string().datetime(),
      placeOfBirth: z.string().max(200),
      citizenship: z.array(z.string().length(2)).min(1),
      taxIdentifiers: z.array(z.object({
        country: z.string().length(2),
        type: z.enum(['SSN', 'TIN', 'EIN', 'ITIN', 'OTHER']),
        value: z.string(),
      })).optional(),
    }),
    preservationStatus: z.object({
      status: z.nativeEnum(PreservationStatus),
      preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
      preservationDate: z.string().datetime().optional(),
    }),
    contacts: z.array(z.object({
      type: z.enum(['EMERGENCY', 'LEGAL_REP', 'FAMILY']),
      name: z.string(),
      relationship: z.string(),
      email: z.string().email(),
      phone: z.string(),
    })).optional(),
  });

  try {
    const data = createSchema.parse(req.body);

    const patient = await patientService.createPatient({
      organizationId: req.user.organizationId,
      ...data,
      legalIdentity: {
        ...data.legalIdentity,
        dateOfBirth: new Date(data.legalIdentity.dateOfBirth),
      },
      preservationStatus: {
        ...data.preservationStatus,
        preservationDate: data.preservationStatus.preservationDate
          ? new Date(data.preservationStatus.preservationDate)
          : null,
      },
    });

    // Auto-create portfolio
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

// PATCH /api/v1/patients/:id
// Update patient
patientRouter.patch('/:id', async (req: Request, res: Response) => {
  const updateSchema = z.object({
    legalIdentity: z.object({
      fullLegalName: z.string().min(2).max(200),
      previousNames: z.array(z.string()),
    }).partial().optional(),
    preservationStatus: z.object({
      status: z.nativeEnum(PreservationStatus),
      preservationDate: z.string().datetime(),
      preservationLocation: z.string(),
    }).partial().optional(),
    contacts: z.array(z.object({
      id: z.string().uuid().optional(),
      type: z.enum(['EMERGENCY', 'LEGAL_REP', 'FAMILY']),
      name: z.string(),
      relationship: z.string(),
      email: z.string().email(),
      phone: z.string(),
    })).optional(),
  });

  try {
    const data = updateSchema.parse(req.body);

    const patient = await patientService.updatePatient(req.params.id, data);

    res.json({
      success: true,
      data: formatPatientDetail(patient),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/patients/:id/preservation
// Record preservation event
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
    documents: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })).optional(),
  });

  try {
    const data = preservationSchema.parse(req.body);

    const result = await patientService.recordPreservation(req.params.id, {
      ...data,
      preservationDate: new Date(data.preservationDate),
    });

    // Trigger portfolio status update
    await portfolioService.updatePreservationStatus(
      result.patient.id,
      'PRESERVED'
    );

    // Record event
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

## 4.3 Portfolio Endpoints

### Portfolio Management API

```typescript
// Portfolio API implementation
const portfolioRouter = Router();

// GET /api/v1/portfolios/:id
// Get portfolio details
portfolioRouter.get('/:id', async (req: Request, res: Response) => {
  try {
    const portfolio = await portfolioService.getPortfolio(req.params.id);

    if (!portfolio) {
      return res.status(404).json({
        success: false,
        error: {
          code: 'PORTFOLIO_NOT_FOUND',
          message: `Portfolio ${req.params.id} not found`,
        },
      });
    }

    // Get detailed holdings and performance
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

// GET /api/v1/portfolios/:id/valuation
// Get current valuation
portfolioRouter.get('/:id/valuation', async (req: Request, res: Response) => {
  const querySchema = z.object({
    asOf: z.string().datetime().optional(),
    includeHistory: z.coerce.boolean().default(false),
    historyPeriod: z.enum(['1M', '3M', '6M', '1Y', '5Y', 'ALL']).default('1Y'),
  });

  try {
    const query = querySchema.parse(req.query);

    const valuation = await valuationService.getPortfolioValuation(
      req.params.id,
      query.asOf ? new Date(query.asOf) : new Date()
    );

    let history = null;
    if (query.includeHistory) {
      history = await valuationService.getValuationHistory(
        req.params.id,
        query.historyPeriod
      );
    }

    res.json({
      success: true,
      data: {
        valuation: formatValuation(valuation),
        history: history ? history.map(formatValuationHistoryPoint) : null,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/portfolios/:id/valuation
// Perform new valuation
portfolioRouter.post('/:id/valuation', async (req: Request, res: Response) => {
  const valuationSchema = z.object({
    valuationDate: z.string().datetime().default(() => new Date().toISOString()),
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

    // Record event
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
// Get portfolio performance metrics
portfolioRouter.get('/:id/performance', async (req: Request, res: Response) => {
  const querySchema = z.object({
    period: z.enum(['MTD', 'QTD', 'YTD', '1Y', '3Y', '5Y', '10Y', 'ITD']).default('YTD'),
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
          trackingError: performance.trackingError,
        } : null,
        attribution: performance.attribution,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/portfolios/:id/rebalance
// Execute portfolio rebalancing
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
      excludeSecurities: z.array(z.string()).optional(),
    }).optional(),
  });

  try {
    const data = rebalanceSchema.parse(req.body);

    // Get current portfolio and policy
    const portfolio = await portfolioService.getPortfolio(req.params.id);
    const currentAllocation = await portfolioService.getAllocation(portfolio.id);
    const targetAllocation = data.targetAllocation ||
      await portfolioService.getTargetAllocation(portfolio.id);

    // Calculate rebalancing trades
    const rebalancePlan = await rebalanceService.calculateRebalancing({
      portfolioId: portfolio.id,
      currentAllocation,
      targetAllocation,
      constraints: data.constraints,
    });

    if (data.mode === 'EXECUTE') {
      // Execute trades
      const execution = await rebalanceService.executeRebalancing(rebalancePlan);

      // Record event
      await eventService.recordEvent({
        type: EventType.PORTFOLIO_REBALANCED,
        subject: { type: 'PORTFOLIO', id: portfolio.id },
        details: {
          tradesExecuted: execution.trades.length,
          turnover: execution.turnover,
        },
      });

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

## 4.4 Asset Endpoints

### Asset Management API

```typescript
// Asset API implementation
const assetRouter = Router();

// GET /api/v1/assets
// List assets with filtering
assetRouter.get('/', async (req: Request, res: Response) => {
  const querySchema = z.object({
    page: z.coerce.number().min(1).default(1),
    pageSize: z.coerce.number().min(1).max(100).default(20),
    portfolioId: z.string().uuid().optional(),
    patientId: z.string().uuid().optional(),
    category: z.nativeEnum(AssetCategory).optional(),
    type: z.string().optional(),
    status: z.nativeEnum(AssetStatus).optional(),
    minValue: z.coerce.number().optional(),
    maxValue: z.coerce.number().optional(),
  });

  try {
    const query = querySchema.parse(req.query);

    const assets = await assetService.listAssets({
      ...query,
      organizationId: req.user.organizationId,
    });

    res.json({
      success: true,
      data: assets.items.map(formatAssetSummary),
      pagination: buildPagination(query, assets.totalCount),
      summary: {
        totalValue: assets.totalValue,
        assetCount: assets.totalCount,
        byCategory: assets.categorySummary,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/assets
// Register new asset
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
        entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION', 'PARTNERSHIP', 'OTHER']),
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

    location: z.object({
      type: z.enum(['PHYSICAL', 'DIGITAL', 'CUSTODIAN']),
      description: z.string(),
      custodian: z.string().optional(),
      accountNumber: z.string().optional(),
    }).optional(),

    documentation: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })).optional(),
  });

  try {
    const data = createSchema.parse(req.body);

    // Validate portfolio exists and belongs to user's organization
    const portfolio = await portfolioService.getPortfolio(data.portfolioId);
    if (!portfolio || portfolio.organizationId !== req.user.organizationId) {
      return res.status(403).json({
        success: false,
        error: { code: 'FORBIDDEN', message: 'Access denied to portfolio' },
      });
    }

    const asset = await assetService.registerAsset({
      ...data,
      registeredBy: req.user.id,
      organizationId: req.user.organizationId,
    });

    // Register on blockchain
    const blockchainRef = await blockchainService.registerAsset(asset);
    await assetService.updateBlockchainRef(asset.id, blockchainRef);

    // Record event
    await eventService.recordEvent({
      type: EventType.ASSET_REGISTERED,
      subject: { type: 'ASSET', id: asset.id },
      details: { category: data.category, value: data.currentValue.value },
    });

    res.status(201).json({
      success: true,
      data: formatAssetDetail(asset),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// GET /api/v1/assets/:id
// Get asset details
assetRouter.get('/:id', async (req: Request, res: Response) => {
  try {
    const asset = await assetService.getAsset(req.params.id);

    if (!asset) {
      return res.status(404).json({
        success: false,
        error: { code: 'ASSET_NOT_FOUND', message: 'Asset not found' },
      });
    }

    // Get valuation history
    const valuationHistory = await valuationService.getAssetValuationHistory(
      asset.id,
      { limit: 12 }
    );

    // Get transaction history
    const transactions = await transactionService.getAssetTransactions(
      asset.id,
      { limit: 20 }
    );

    res.json({
      success: true,
      data: {
        ...formatAssetDetail(asset),
        valuationHistory: valuationHistory.map(formatValuationHistoryPoint),
        transactions: transactions.map(formatTransaction),
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// PUT /api/v1/assets/:id/valuation
// Update asset valuation
assetRouter.put('/:id/valuation', async (req: Request, res: Response) => {
  const valuationSchema = z.object({
    value: z.number().nonnegative(),
    currency: z.string().length(3),
    valuationMethod: z.nativeEnum(ValuationMethod),
    valuationDate: z.string().datetime(),
    source: z.object({
      type: z.enum(['MARKET', 'APPRAISER', 'INSTITUTION', 'INTERNAL']),
      name: z.string(),
      reference: z.string().optional(),
    }),
    documentation: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })).optional(),
  });

  try {
    const data = valuationSchema.parse(req.body);

    const valuation = await valuationService.recordAssetValuation(
      req.params.id,
      {
        ...data,
        valuationDate: new Date(data.valuationDate),
        performedBy: req.user.id,
      }
    );

    // Update blockchain
    await blockchainService.updateAssetValuation(req.params.id, valuation);

    res.json({
      success: true,
      data: formatValuation(valuation),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/assets/:id/transfer
// Transfer asset ownership
assetRouter.post('/:id/transfer', async (req: Request, res: Response) => {
  const transferSchema = z.object({
    targetPortfolioId: z.string().uuid().optional(),
    targetOrganizationId: z.string().uuid().optional(),
    newOwnership: z.object({
      type: z.nativeEnum(OwnershipType),
      holders: z.array(z.object({
        entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION', 'PARTNERSHIP', 'OTHER']),
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

    // Record blockchain transfer
    await blockchainService.recordAssetTransfer(transfer);

    // Record event
    await eventService.recordEvent({
      type: EventType.ASSET_TRANSFERRED,
      subject: { type: 'ASSET', id: req.params.id },
      details: {
        transferType: data.transferType,
        value: data.transferValue,
      },
    });

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

## 4.5 Trust Endpoints

### Trust Management API

```typescript
// Trust API implementation
const trustRouter = Router();

// GET /api/v1/trusts
// List trusts
trustRouter.get('/', async (req: Request, res: Response) => {
  const querySchema = z.object({
    page: z.coerce.number().min(1).default(1),
    pageSize: z.coerce.number().min(1).max(100).default(20),
    patientId: z.string().uuid().optional(),
    trustType: z.nativeEnum(TrustType).optional(),
    status: z.nativeEnum(TrustStatus).optional(),
    role: z.enum(['GRANTOR', 'TRUSTEE', 'BENEFICIARY']).optional(),
  });

  try {
    const query = querySchema.parse(req.query);

    const trusts = await trustService.listTrusts({
      ...query,
      organizationId: req.user.organizationId,
    });

    res.json({
      success: true,
      data: trusts.items.map(formatTrustSummary),
      pagination: buildPagination(query, trusts.totalCount),
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/trusts
// Create new trust
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
        entityType: z.enum(['INDIVIDUAL', 'CORPORATION', 'TRUST', 'OTHER']),
        name: z.string(),
        identifier: z.string(),
        address: z.object({
          street: z.string(),
          city: z.string(),
          state: z.string().optional(),
          postalCode: z.string(),
          country: z.string().length(2),
        }),
      }),

      trustees: z.array(z.object({
        entityType: z.enum(['INDIVIDUAL', 'CORPORATE', 'PROFESSIONAL']),
        name: z.string(),
        identifier: z.string(),
        powers: z.array(z.string()),
        compensation: z.object({
          type: z.enum(['FIXED', 'PERCENTAGE', 'STATUTORY', 'NONE']),
          amount: z.number().optional(),
          percentage: z.number().optional(),
        }),
      })).min(1),

      beneficiaries: z.array(z.object({
        beneficiaryType: z.nativeEnum(BeneficiaryType),
        patientId: z.string().uuid().optional(),
        name: z.string().optional(),
        interestType: z.enum(['INCOME', 'PRINCIPAL', 'BOTH', 'CONTINGENT']),
        share: z.number().optional(),
        conditions: z.array(z.object({
          type: z.string(),
          condition: z.string(),
        })).optional(),
      })).min(1),

      protector: z.object({
        name: z.string(),
        identifier: z.string(),
        powers: z.array(z.string()),
      }).optional(),
    }),

    terms: z.object({
      purpose: z.string(),
      duration: z.object({
        type: z.enum(['PERPETUAL', 'TERM', 'LIFE', 'UNTIL_EVENT']),
        termYears: z.number().optional(),
        terminationEvent: z.string().optional(),
      }),
      revivalProvisions: z.object({
        revivalDefinition: z.string(),
        identityVerificationMethod: z.string(),
        distributionUponRevival: z.object({
          type: z.string(),
          amount: z.string(),
        }),
        rehabilitationPeriod: z.number().optional(),
      }).optional(),
      spendthrift: z.boolean().default(true),
    }),

    initialFunding: z.object({
      amount: z.number().nonnegative(),
      currency: z.string().length(3),
      source: z.string(),
      fundingDate: z.string().datetime(),
    }).optional(),
  });

  try {
    const data = createSchema.parse(req.body);

    const trust = await trustService.createTrust({
      ...data,
      createdBy: req.user.id,
      organizationId: req.user.organizationId,
    });

    // Generate trust document draft
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
// Fund trust
trustRouter.post('/:id/fund', async (req: Request, res: Response) => {
  const fundingSchema = z.object({
    amount: z.number().positive(),
    currency: z.string().length(3),
    fundingSource: z.object({
      type: z.enum(['CASH', 'SECURITIES', 'INSURANCE', 'PROPERTY', 'OTHER']),
      sourceAccountId: z.string().optional(),
      description: z.string(),
    }),
    fundingDate: z.string().datetime(),
    assets: z.array(z.object({
      assetId: z.string().uuid(),
      transferValue: z.number().nonnegative(),
    })).optional(),
    documentation: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })),
  });

  try {
    const data = fundingSchema.parse(req.body);

    const funding = await trustService.fundTrust(req.params.id, {
      ...data,
      fundingDate: new Date(data.fundingDate),
      processedBy: req.user.id,
    });

    // Record event
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
// Record trust distribution
trustRouter.post('/:id/distribution', async (req: Request, res: Response) => {
  const distributionSchema = z.object({
    beneficiaryId: z.string(),
    amount: z.number().positive(),
    currency: z.string().length(3),
    distributionType: z.enum(['INCOME', 'PRINCIPAL', 'BOTH']),
    distributionReason: z.string(),
    distributionDate: z.string().datetime(),
    paymentMethod: z.enum(['CHECK', 'WIRE', 'ACH', 'CASH', 'IN_KIND']),
    paymentDetails: z.object({
      accountNumber: z.string().optional(),
      routingNumber: z.string().optional(),
      reference: z.string().optional(),
    }).optional(),
    taxWithholding: z.object({
      amount: z.number().nonnegative(),
      type: z.string(),
    }).optional(),
    documentation: z.array(z.object({
      type: z.string(),
      documentId: z.string().uuid(),
    })),
  });

  try {
    const data = distributionSchema.parse(req.body);

    const distribution = await trustService.recordDistribution(req.params.id, {
      ...data,
      distributionDate: new Date(data.distributionDate),
      processedBy: req.user.id,
    });

    // Record transaction
    await transactionService.recordTransaction({
      type: TransactionType.TRUST_DISTRIBUTION,
      trustId: req.params.id,
      amount: data.amount,
      beneficiaryId: data.beneficiaryId,
      date: new Date(data.distributionDate),
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

## 4.6 Integration Endpoints

### External System Integration API

```typescript
// Integration API implementation
const integrationRouter = Router();

// GET /api/v1/integrations
// List available integrations
integrationRouter.get('/', async (req: Request, res: Response) => {
  const integrations = await integrationService.listIntegrations(
    req.user.organizationId
  );

  res.json({
    success: true,
    data: integrations.map(integration => ({
      id: integration.id,
      type: integration.type,
      name: integration.name,
      status: integration.status,
      lastSync: integration.lastSync,
      accountsLinked: integration.accountsLinked,
    })),
  });
});

// POST /api/v1/integrations/banking/connect
// Connect banking integration
integrationRouter.post('/banking/connect', async (req: Request, res: Response) => {
  const connectSchema = z.object({
    provider: z.enum(['plaid', 'yodlee', 'mx', 'direct']),
    credentials: z.object({
      publicToken: z.string().optional(),  // For Plaid
      username: z.string().optional(),
      accessCode: z.string().optional(),
    }),
    institutionId: z.string().optional(),
    accountTypes: z.array(z.enum(['CHECKING', 'SAVINGS', 'INVESTMENT', 'CREDIT'])),
  });

  try {
    const data = connectSchema.parse(req.body);

    // Exchange token and connect
    const connection = await bankingIntegration.connect({
      ...data,
      organizationId: req.user.organizationId,
      userId: req.user.id,
    });

    // Sync accounts
    const accounts = await bankingIntegration.syncAccounts(connection.id);

    res.json({
      success: true,
      data: {
        connectionId: connection.id,
        status: connection.status,
        accounts: accounts.map(account => ({
          id: account.id,
          name: account.name,
          type: account.type,
          balance: account.balance,
          currency: account.currency,
          institution: account.institution,
        })),
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/integrations/brokerage/connect
// Connect brokerage integration
integrationRouter.post('/brokerage/connect', async (req: Request, res: Response) => {
  const connectSchema = z.object({
    provider: z.enum(['plaid', 'quovo', 'direct']),
    credentials: z.object({
      publicToken: z.string().optional(),
      apiKey: z.string().optional(),
      apiSecret: z.string().optional(),
    }),
    brokerageId: z.string().optional(),
  });

  try {
    const data = connectSchema.parse(req.body);

    const connection = await brokerageIntegration.connect({
      ...data,
      organizationId: req.user.organizationId,
    });

    // Sync positions
    const positions = await brokerageIntegration.syncPositions(connection.id);

    res.json({
      success: true,
      data: {
        connectionId: connection.id,
        status: connection.status,
        accounts: positions.accounts,
        totalValue: positions.totalValue,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/integrations/sync
// Trigger integration sync
integrationRouter.post('/sync', async (req: Request, res: Response) => {
  const syncSchema = z.object({
    integrationIds: z.array(z.string().uuid()).optional(),
    syncType: z.enum(['FULL', 'INCREMENTAL']).default('INCREMENTAL'),
    entities: z.array(z.enum(['ACCOUNTS', 'POSITIONS', 'TRANSACTIONS'])).optional(),
  });

  try {
    const data = syncSchema.parse(req.body);

    // Start async sync job
    const syncJob = await integrationService.startSync({
      organizationId: req.user.organizationId,
      integrationIds: data.integrationIds,
      syncType: data.syncType,
      entities: data.entities,
    });

    res.json({
      success: true,
      data: {
        jobId: syncJob.id,
        status: syncJob.status,
        estimatedCompletion: syncJob.estimatedCompletion,
        statusUrl: `/api/v1/jobs/${syncJob.id}`,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// Blockchain integration endpoints
// POST /api/v1/integrations/blockchain/register
integrationRouter.post('/blockchain/register', async (req: Request, res: Response) => {
  const registerSchema = z.object({
    entityType: z.enum(['ASSET', 'TRUST', 'VALUATION', 'TRANSACTION']),
    entityId: z.string().uuid(),
    network: z.enum(['ethereum', 'polygon', 'arbitrum']).default('polygon'),
  });

  try {
    const data = registerSchema.parse(req.body);

    // Get entity data
    let entityData;
    switch (data.entityType) {
      case 'ASSET':
        entityData = await assetService.getAsset(data.entityId);
        break;
      case 'TRUST':
        entityData = await trustService.getTrust(data.entityId);
        break;
      default:
        throw new Error('Unsupported entity type');
    }

    // Register on blockchain
    const registration = await blockchainService.register({
      type: data.entityType,
      data: entityData,
      network: data.network,
    });

    res.json({
      success: true,
      data: {
        transactionHash: registration.transactionHash,
        blockNumber: registration.blockNumber,
        contractAddress: registration.contractAddress,
        explorerUrl: registration.explorerUrl,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});
```

---

## 4.7 Webhook Configuration

### Webhook Management

```typescript
// Webhook API implementation
const webhookRouter = Router();

// Supported webhook events
const WebhookEvents = {
  // Patient events
  'patient.created': 'New patient registered',
  'patient.preserved': 'Patient preservation recorded',
  'patient.revived': 'Patient revival recorded',
  'patient.updated': 'Patient information updated',

  // Portfolio events
  'portfolio.valued': 'Portfolio valuation completed',
  'portfolio.rebalanced': 'Portfolio rebalancing executed',
  'portfolio.threshold_alert': 'Portfolio threshold breached',

  // Asset events
  'asset.registered': 'New asset registered',
  'asset.valued': 'Asset valuation updated',
  'asset.transferred': 'Asset ownership transferred',

  // Trust events
  'trust.created': 'New trust created',
  'trust.funded': 'Trust funding received',
  'trust.distribution': 'Trust distribution made',
  'trust.amended': 'Trust terms amended',

  // Compliance events
  'compliance.review_due': 'Compliance review due',
  'compliance.violation': 'Compliance violation detected',
  'compliance.filing_required': 'Regulatory filing required',
};

// GET /api/v1/webhooks
webhookRouter.get('/', async (req: Request, res: Response) => {
  const webhooks = await webhookService.listWebhooks(req.user.organizationId);

  res.json({
    success: true,
    data: webhooks.map(webhook => ({
      id: webhook.id,
      url: webhook.url,
      events: webhook.events,
      status: webhook.status,
      createdAt: webhook.createdAt,
      lastDelivery: webhook.lastDelivery,
      successRate: webhook.successRate,
    })),
  });
});

// POST /api/v1/webhooks
webhookRouter.post('/', async (req: Request, res: Response) => {
  const createSchema = z.object({
    url: z.string().url(),
    events: z.array(z.enum(Object.keys(WebhookEvents) as [string, ...string[]])),
    secret: z.string().min(32).optional(),
    headers: z.record(z.string()).optional(),
    retryPolicy: z.object({
      maxRetries: z.number().min(0).max(10).default(3),
      backoffMultiplier: z.number().min(1).max(10).default(2),
    }).optional(),
  });

  try {
    const data = createSchema.parse(req.body);

    // Generate secret if not provided
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
        secret: secret,  // Only returned on creation
        status: webhook.status,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// POST /api/v1/webhooks/:id/test
webhookRouter.post('/:id/test', async (req: Request, res: Response) => {
  const testSchema = z.object({
    event: z.enum(Object.keys(WebhookEvents) as [string, ...string[]]),
    payload: z.record(z.any()).optional(),
  });

  try {
    const data = testSchema.parse(req.body);

    const result = await webhookService.testWebhook(req.params.id, {
      event: data.event,
      payload: data.payload || generateTestPayload(data.event),
    });

    res.json({
      success: true,
      data: {
        delivered: result.delivered,
        statusCode: result.statusCode,
        responseTime: result.responseTime,
        response: result.response,
        error: result.error,
      },
    });
  } catch (error) {
    handleApiError(res, error);
  }
});

// Webhook payload structure
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

// Webhook signature verification
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

## 4.8 Error Handling and Response Formats

### Standardized Error Responses

```typescript
// Error handling implementation
interface ApiErrorResponse {
  success: false;
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    field?: string;
    validationErrors?: ValidationError[];
  };
  requestId: string;
  timestamp: string;
}

// Error codes
const ErrorCodes = {
  // Authentication/Authorization
  UNAUTHORIZED: { status: 401, message: 'Authentication required' },
  FORBIDDEN: { status: 403, message: 'Access denied' },
  TOKEN_EXPIRED: { status: 401, message: 'Authentication token expired' },

  // Resource errors
  NOT_FOUND: { status: 404, message: 'Resource not found' },
  PATIENT_NOT_FOUND: { status: 404, message: 'Patient not found' },
  PORTFOLIO_NOT_FOUND: { status: 404, message: 'Portfolio not found' },
  ASSET_NOT_FOUND: { status: 404, message: 'Asset not found' },
  TRUST_NOT_FOUND: { status: 404, message: 'Trust not found' },

  // Validation errors
  VALIDATION_ERROR: { status: 400, message: 'Validation error' },
  INVALID_INPUT: { status: 400, message: 'Invalid input' },
  MISSING_REQUIRED_FIELD: { status: 400, message: 'Missing required field' },

  // Business logic errors
  INSUFFICIENT_FUNDS: { status: 400, message: 'Insufficient funds' },
  DUPLICATE_ENTRY: { status: 409, message: 'Duplicate entry' },
  OPERATION_NOT_ALLOWED: { status: 400, message: 'Operation not allowed' },
  INVALID_STATE_TRANSITION: { status: 400, message: 'Invalid state transition' },

  // Rate limiting
  RATE_LIMIT_EXCEEDED: { status: 429, message: 'Rate limit exceeded' },

  // Server errors
  INTERNAL_ERROR: { status: 500, message: 'Internal server error' },
  SERVICE_UNAVAILABLE: { status: 503, message: 'Service temporarily unavailable' },
};

// Error handling middleware
function handleApiError(res: Response, error: unknown): void {
  const requestId = res.locals.requestId || generateRequestId();

  if (error instanceof z.ZodError) {
    res.status(400).json({
      success: false,
      error: {
        code: 'VALIDATION_ERROR',
        message: 'Validation failed',
        validationErrors: error.errors.map(e => ({
          path: e.path.join('.'),
          message: e.message,
          code: e.code,
        })),
      },
      requestId,
      timestamp: new Date().toISOString(),
    });
    return;
  }

  if (error instanceof ApiError) {
    res.status(error.statusCode).json({
      success: false,
      error: {
        code: error.code,
        message: error.message,
        details: error.details,
      },
      requestId,
      timestamp: new Date().toISOString(),
    });
    return;
  }

  // Log unexpected errors
  console.error('Unexpected error:', error);

  res.status(500).json({
    success: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: 'An unexpected error occurred',
    },
    requestId,
    timestamp: new Date().toISOString(),
  });
}

// Custom API error class
class ApiError extends Error {
  constructor(
    public code: string,
    public message: string,
    public statusCode: number = 400,
    public details?: Record<string, any>
  ) {
    super(message);
    this.name = 'ApiError';
  }

  static notFound(resource: string, id?: string): ApiError {
    return new ApiError(
      `${resource.toUpperCase()}_NOT_FOUND`,
      `${resource} ${id ? `with ID ${id} ` : ''}not found`,
      404
    );
  }

  static forbidden(message: string = 'Access denied'): ApiError {
    return new ApiError('FORBIDDEN', message, 403);
  }

  static validation(errors: ValidationError[]): ApiError {
    const error = new ApiError('VALIDATION_ERROR', 'Validation failed', 400);
    error.details = { validationErrors: errors };
    return error;
  }
}
```

---

## Chapter Summary

This chapter defined the complete API interface for the WIA Cryo-Asset Management platform:

1. **API Architecture**: RESTful design with OpenAPI 3.0 specification
2. **Patient Endpoints**: Full CRUD operations and preservation recording
3. **Portfolio Endpoints**: Valuation, performance, and rebalancing operations
4. **Asset Endpoints**: Registration, valuation updates, and transfers
5. **Trust Endpoints**: Creation, funding, and distribution management
6. **Integration Endpoints**: Banking, brokerage, and blockchain connections
7. **Webhook Configuration**: Event subscription and delivery management
8. **Error Handling**: Standardized error responses and codes

The API provides a comprehensive interface for building cryonics asset management applications, with proper authentication, validation, and integration capabilities.

---

*Next Chapter: Control Protocols - Asset governance and management rules*
