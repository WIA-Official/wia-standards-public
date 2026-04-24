# Chapter 4: API Interface

## Consent Management APIs

This chapter defines the comprehensive API interfaces for the WIA Cryo-Consent Standard, including RESTful APIs, GraphQL endpoints, and real-time WebSocket interfaces for consent management operations.

---

## 4.1 RESTful API Design

```typescript
// Base API configuration
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

// API Router setup
import express from 'express';

const router = express.Router();

// Middleware
router.use(authenticationMiddleware);
router.use(rateLimitMiddleware);
router.use(auditMiddleware);
router.use(validationMiddleware);

// Consent CRUD endpoints
router.post('/consents', createConsentHandler);
router.get('/consents', listConsentsHandler);
router.get('/consents/:id', getConsentHandler);
router.put('/consents/:id', updateConsentHandler);
router.delete('/consents/:id', revokeConsentHandler);

// Consent lifecycle endpoints
router.post('/consents/:id/activate', activateConsentHandler);
router.post('/consents/:id/suspend', suspendConsentHandler);
router.post('/consents/:id/renew', renewConsentHandler);
router.post('/consents/:id/review', recordReviewHandler);

// Decision endpoints
router.get('/consents/:id/decisions', listDecisionsHandler);
router.post('/consents/:id/decisions', addDecisionHandler);
router.put('/consents/:id/decisions/:decisionId', updateDecisionHandler);

// Query endpoints
router.get('/patients/:patientId/consents', getPatientConsentsHandler);
router.get('/patients/:patientId/decisions/:decisionType', queryDecisionHandler);
router.get('/patients/:patientId/effective-consent', getEffectiveConsentHandler);

// Document endpoints
router.post('/consents/:id/documents', uploadDocumentHandler);
router.get('/consents/:id/documents', listDocumentsHandler);
router.get('/consents/:id/documents/:docId', downloadDocumentHandler);

// Proxy endpoints
router.get('/patients/:patientId/proxies', listProxiesHandler);
router.post('/patients/:patientId/proxies', designateProxyHandler);
router.put('/patients/:patientId/proxies/:proxyId', updateProxyHandler);
router.delete('/patients/:patientId/proxies/:proxyId', removeProxyHandler);

// Audit endpoints
router.get('/consents/:id/audit', getAuditTrailHandler);
router.get('/patients/:patientId/audit', getPatientAuditHandler);

export default router;
```

---

## 4.2 Consent CRUD Operations

```typescript
// Create consent handler
async function createConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const input: CreateConsentInput = req.body;

    // Validate input
    const validation = await validateCreateConsentInput(input);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Invalid consent input',
          details: validation.errors,
        },
      });
      return;
    }

    // Check authorization
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

    // Create consent
    const consent = await consentService.createConsent(input, {
      createdBy: req.user.id,
      ipAddress: req.ip,
      userAgent: req.headers['user-agent'],
    });

    // Generate response
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

// Create consent input interface
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

// Get consent handler
async function getConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const include = parseIncludeParam(req.query.include);

    // Get consent
    const consent = await consentService.getConsent(id, include);

    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `Consent ${id} not found`,
        },
      });
      return;
    }

    // Check read authorization
    const authResult = await checkReadAuthorization(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: 'Not authorized to view this consent',
        },
      });
      return;
    }

    // Log access
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

// Update consent handler
async function updateConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const updates: UpdateConsentInput = req.body;

    // Get existing consent
    const existing = await consentService.getConsent(id);
    if (!existing) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `Consent ${id} not found`,
        },
      });
      return;
    }

    // Check update authorization
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

    // Validate updates
    const validation = await validateUpdateConsentInput(existing, updates);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Invalid updates',
          details: validation.errors,
        },
      });
      return;
    }

    // Apply updates
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

// Revoke consent handler
async function revokeConsentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id } = req.params;
    const revocation: RevocationInput = req.body;

    // Get consent
    const consent = await consentService.getConsent(id);
    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `Consent ${id} not found`,
        },
      });
      return;
    }

    // Check revocation authorization
    const authResult = await checkRevocationAuthorization(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: 'Not authorized to revoke this consent',
        },
      });
      return;
    }

    // Check if revocable
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

    // Process revocation
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

// List consents handler
async function listConsentsHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const query = parseConsentQuery(req.query);

    // Apply user-based filtering
    const filteredQuery = applyUserFilters(query, req.user);

    // Execute query
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

## 4.3 Decision Query API

```typescript
// Query effective consent for decision
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

    // Build decision query
    const query: DecisionQuery = {
      patientId,
      decisionType: decisionType as string,
      context: context ? JSON.parse(context as string) : {},
      asOfDate: asOfDate ? new Date(asOfDate as string) : undefined,
      interpretationMode: (interpretationMode as 'STRICT' | 'FLEXIBLE') || 'FLEXIBLE',
    };

    // Execute query
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

// Decision query service
class ConsentDecisionQueryService {
  constructor(
    private consentRepository: ConsentRepository,
    private interpretationEngine: ConsentInterpretationEngine
  ) {}

  async queryEffectiveConsent(query: DecisionQuery): Promise<DecisionQueryResult> {
    // Find applicable consents
    const consents = await this.consentRepository.findByPatient(
      query.patientId,
      {
        status: 'ACTIVE',
        effectiveAsOf: query.asOfDate || new Date(),
      }
    );

    // Filter to those covering the decision type
    const applicable = consents.filter(c =>
      this.coversDecisionType(c, query.decisionType)
    );

    if (applicable.length === 0) {
      return {
        found: false,
        confidence: 0,
        warnings: ['No consent found covering this decision type'],
        recommendations: [
          'Patient should be asked to provide explicit consent',
          'Consider consulting proxy if designated',
        ],
      };
    }

    // Find most specific consent
    const ranked = this.rankConsents(applicable, query);
    const best = ranked[0];

    // Extract decision
    const decision = this.extractDecision(best.consent, query.decisionType);

    // Apply interpretation if needed
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

    // More recent consents score higher
    const age = Date.now() - consent.metadata.updatedAt.getTime();
    const ageScore = Math.max(0, 100 - age / (365 * 24 * 60 * 60 * 1000));
    score += ageScore * 0.2;

    // More specific category matches score higher
    if (consent.category === this.getCategoryForDecision(query.decisionType)) {
      score += 30;
    }

    // Explicit decision for type scores highest
    if (consent.decisions.some(d => d.question.toLowerCase().includes(query.decisionType.toLowerCase()))) {
      score += 50;
    }

    // Higher version consents score higher (more refined)
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

## 4.4 Proxy Management API

```typescript
// Proxy designation endpoint
async function designateProxyHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { patientId } = req.params;
    const proxyInput: ProxyDesignationInput = req.body;

    // Validate input
    const validation = validateProxyInput(proxyInput);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Invalid proxy designation',
          details: validation.errors,
        },
      });
      return;
    }

    // Check authorization (must be patient or authorized representative)
    const authResult = await checkProxyDesignationAuth(req.user, patientId);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: 'Only patient can designate proxies',
        },
      });
      return;
    }

    // Create proxy designation
    const proxy = await proxyService.designateProxy(patientId, proxyInput, {
      designatedBy: req.user.id,
      ipAddress: req.ip,
    });

    // Notify proxy
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

// Proxy service
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
    // Check existing proxies
    const existingProxies = await this.proxyRepository.findByPatient(patientId);

    // Determine order
    const order = existingProxies.length + 1;

    // Create proxy record
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

    // Store
    const stored = await this.proxyRepository.create(patientId, proxy, order);

    return stored;
  }

  async acceptProxyDesignation(
    proxyId: string,
    acceptance: ProxyAcceptanceInput
  ): Promise<ProxyDesignation> {
    const proxy = await this.proxyRepository.findById(proxyId);
    if (!proxy) {
      throw new NotFoundError(`Proxy ${proxyId} not found`);
    }

    if (proxy.acceptanceStatus !== 'PENDING') {
      throw new ConflictError(`Proxy already ${proxy.acceptanceStatus.toLowerCase()}`);
    }

    // Verify acceptor identity
    await this.verifyAcceptorIdentity(proxy, acceptance);

    // Update acceptance
    const updated = await this.proxyRepository.update(proxyId, {
      acceptanceStatus: 'ACCEPTED',
      acceptanceDate: new Date(),
      acceptanceSignature: acceptance.signature,
    });

    // Notify patient
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
      throw new NotFoundError(`Proxy ${proxyId} not found`);
    }

    if (proxy.acceptanceStatus !== 'ACCEPTED') {
      throw new ConflictError('Proxy must accept designation before activation');
    }

    // Check activation conditions
    if (proxy.authorityScope.activationCondition) {
      const conditionMet = await this.evaluateActivationCondition(
        patientId,
        proxy.authorityScope.activationCondition
      );
      if (!conditionMet) {
        throw new ConflictError('Activation condition not met');
      }
    }

    // Check if higher-priority proxies are available
    const higherPriority = await this.getHigherPriorityProxies(patientId, proxyId);
    if (higherPriority.length > 0) {
      throw new ConflictError(
        'Higher priority proxies must be unavailable before activating this proxy'
      );
    }

    // Activate
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
      throw new NotFoundError(`Proxy ${proxyId} not found`);
    }

    // Verify proxy is active
    const isActive = await this.isProxyActive(proxyId);
    if (!isActive) {
      throw new ConflictError('Proxy is not currently active');
    }

    // Check authority for this decision
    if (!this.hasAuthorityForDecision(proxy, decision.decisionType)) {
      throw new UnauthorizedError('Proxy lacks authority for this decision type');
    }

    // Check if consultation required
    if (proxy.authorityScope.requiresConsultation.includes(decision.decisionType)) {
      if (!decision.consultationRecords || decision.consultationRecords.length === 0) {
        throw new ValidationError('This decision requires documented consultation');
      }
    }

    // Check if committee approval required
    if (proxy.authorityScope.requiresCommitteeApproval.includes(decision.decisionType)) {
      if (!decision.committeeApproval) {
        throw new ValidationError('This decision requires committee approval');
      }
    }

    // Record decision
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
    // Check exclusions first
    if (proxy.authorityScope.exclusions.includes(decisionType)) {
      return false;
    }

    // Check specific decisions
    if (proxy.authorityScope.specificDecisions.length > 0) {
      return proxy.authorityScope.specificDecisions.includes(decisionType);
    }

    // Check category authority
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

## 4.5 Document Management API

```typescript
// Document upload handler
async function uploadDocumentHandler(
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> {
  try {
    const { id: consentId } = req.params;

    // Check consent exists
    const consent = await consentService.getConsent(consentId);
    if (!consent) {
      res.status(404).json({
        success: false,
        error: {
          code: 'NOT_FOUND',
          message: `Consent ${consentId} not found`,
        },
      });
      return;
    }

    // Check upload authorization
    const authResult = await checkDocumentUploadAuth(req.user, consent);
    if (!authResult.authorized) {
      res.status(403).json({
        success: false,
        error: {
          code: 'UNAUTHORIZED',
          message: 'Not authorized to upload documents to this consent',
        },
      });
      return;
    }

    // Process upload
    const files = req.files as Express.Multer.File[];
    const documentType = req.body.documentType as DocumentType;

    const uploadedDocs = await Promise.all(
      files.map(file =>
        documentService.uploadDocument(consentId, {
          file,
          documentType,
          title: req.body.title || file.originalname,
          description: req.body.description,
          language: req.body.language || 'en',
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

// Document service
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
    // Generate document ID
    const documentId = generateDocumentId();

    // Calculate hash before encryption
    const hash = await this.integrityService.calculateHash(input.file.buffer);

    // Encrypt file
    const encrypted = await this.encryptionService.encrypt(input.file.buffer);

    // Store in primary location
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

    // Replicate to backup locations
    const backupLocations = await this.replicateToBackups(
      documentId,
      encrypted.data
    );

    // Anchor to blockchain
    const blockchainAnchor = await this.anchorToBlockchain(hash);

    // Create document record
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
          retentionPeriod: 'P1000Y', // 1000 years
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
    // Get document record
    const document = await this.getDocument(documentId);
    if (!document) {
      throw new NotFoundError(`Document ${documentId} not found`);
    }

    // Check access authorization
    const hasAccess = await this.checkDocumentAccess(document, requestor);
    if (!hasAccess) {
      throw new UnauthorizedError('Not authorized to access this document');
    }

    // Retrieve from storage
    const encrypted = await this.storageService.retrieve(
      document.storage.primaryLocation.locationUri
    );

    // Decrypt
    const decrypted = await this.encryptionService.decrypt(
      encrypted,
      document.storage.encryption.keyReference
    );

    // Verify integrity
    const hash = await this.integrityService.calculateHash(decrypted);
    if (hash !== document.integrity.hashValue) {
      throw new IntegrityError('Document integrity check failed');
    }

    // Log access
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
      throw new NotFoundError(`Document ${documentId} not found`);
    }

    // Retrieve document
    const encrypted = await this.storageService.retrieve(
      document.storage.primaryLocation.locationUri
    );
    const decrypted = await this.encryptionService.decrypt(
      encrypted,
      document.storage.encryption.keyReference
    );

    // Verify primary hash
    const currentHash = await this.integrityService.calculateHash(decrypted);
    const primaryValid = currentHash === document.integrity.hashValue;

    // Verify additional hashes
    const additionalVerifications = await Promise.all(
      (document.integrity.additionalHashes || []).map(async h => ({
        algorithm: h.algorithm,
        valid: await this.integrityService.verifyHash(decrypted, h.algorithm, h.value),
      }))
    );

    // Verify blockchain anchor
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
    // Anchor to distributed ledger for immutable timestamp
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
// GraphQL schema
const typeDefs = gql`
  type Query {
    # Consent queries
    consent(id: ID!): Consent
    consents(query: ConsentQueryInput): ConsentConnection!

    # Patient queries
    patientConsents(patientId: ID!): [Consent!]!
    effectiveConsent(
      patientId: ID!
      decisionType: String!
      context: JSON
    ): EffectiveConsentResult

    # Proxy queries
    proxies(patientId: ID!): [Proxy!]!
    proxy(id: ID!): Proxy

    # Document queries
    consentDocuments(consentId: ID!): [Document!]!
    document(id: ID!): Document
  }

  type Mutation {
    # Consent mutations
    createConsent(input: CreateConsentInput!): ConsentResult!
    updateConsent(id: ID!, input: UpdateConsentInput!): ConsentResult!
    activateConsent(id: ID!): ConsentResult!
    suspendConsent(id: ID!, reason: String!): ConsentResult!
    revokeConsent(id: ID!, input: RevocationInput!): ConsentResult!

    # Decision mutations
    addDecision(consentId: ID!, input: DecisionInput!): DecisionResult!
    updateDecision(
      consentId: ID!
      decisionId: ID!
      input: DecisionInput!
    ): DecisionResult!

    # Proxy mutations
    designateProxy(patientId: ID!, input: ProxyInput!): ProxyResult!
    acceptProxy(proxyId: ID!, input: AcceptanceInput!): ProxyResult!
    activateProxy(proxyId: ID!, reason: String!): ProxyResult!
    deactivateProxy(proxyId: ID!, reason: String!): ProxyResult!

    # Document mutations
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

// GraphQL resolvers
const resolvers = {
  Query: {
    consent: async (_: any, { id }: { id: string }, context: Context) => {
      const consent = await context.dataSources.consentService.getConsent(id);
      if (!consent) return null;

      // Check authorization
      if (!await checkReadAuth(context.user, consent)) {
        throw new ForbiddenError('Not authorized to view this consent');
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
      // Verify only patient can designate
      if (context.user.patientId !== patientId && !context.user.isAdmin) {
        throw new ForbiddenError('Only patient can designate proxies');
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

## 4.7 WebSocket Real-time API

```typescript
// WebSocket server for real-time consent updates
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
        // Authenticate connection
        const token = this.extractToken(req);
        const user = await this.authenticateToken(token);

        // Set up connection
        this.registerConnection(ws, user);

        // Handle messages
        ws.on('message', (data: WebSocket.Data) => {
          this.handleMessage(ws, user, data);
        });

        ws.on('close', () => {
          this.unregisterConnection(ws, user);
        });

        // Send connection confirmation
        ws.send(JSON.stringify({
          type: 'CONNECTED',
          userId: user.id,
          timestamp: new Date().toISOString(),
        }));
      } catch (error) {
        ws.send(JSON.stringify({
          type: 'ERROR',
          error: 'Authentication failed',
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
            error: `Unknown message type: ${message.type}`,
          }));
      }
    } catch (error) {
      ws.send(JSON.stringify({
        type: 'ERROR',
        error: 'Invalid message format',
      }));
    }
  }

  // Broadcast consent update to subscribed clients
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

  // Broadcast proxy activation
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

  // Broadcast review reminder
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

  // Handle subscription requests
  private handleSubscribePatient(
    ws: WebSocket,
    user: User,
    patientId: string
  ): void {
    // Verify authorization
    if (!this.canSubscribeToPatient(user, patientId)) {
      ws.send(JSON.stringify({
        type: 'SUBSCRIPTION_ERROR',
        error: 'Not authorized to subscribe to this patient',
      }));
      return;
    }

    // Add to subscribers
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
    // Patient can subscribe to own updates
    if (user.patientId === patientId) return true;

    // Staff with appropriate role
    if (user.roles.includes('MEDICAL_STAFF') || user.roles.includes('ADMIN')) {
      return true;
    }

    // Active proxy for patient
    // (would check proxy status in real implementation)
    return false;
  }
}

interface ConsentUpdate {
  type: 'CREATED' | 'UPDATED' | 'ACTIVATED' | 'SUSPENDED' | 'REVOKED';
  changes: Record<string, any>;
}
```

---

## 4.8 API Response Formats

```typescript
// Standard API response formats
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
  stack?: string;  // Only in development
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

// Response serializers
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

    // Include optional fields
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
      integrityVerified: true,  // Would check actual verification
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

*Next Chapter: Control Protocols - Consent lifecycle management and decision workflows*
