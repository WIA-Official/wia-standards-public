/**
 * WIA-CORE-002: Universal Consent SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Privacy & Compliance Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for universal consent management including:
 * - Consent request and grant workflows
 * - GDPR/CCPA compliance validation
 * - Consent propagation and synchronization
 * - Audit trail and compliance reporting
 */

import {
  ConsentRecord,
  ConsentRequest,
  ConsentRequestResponse,
  ConsentGrant,
  ConsentGrantResponse,
  ConsentCheck,
  ConsentCheckResponse,
  ConsentRevocation,
  ConsentRevocationResponse,
  ConsentListRequest,
  ConsentListResponse,
  ConsentExport,
  ConsentExportResponse,
  ConsentManagerConfig,
  PurposeDefinition,
  ConsentReceipt,
  ConsentStatus,
  ConsentEventType,
  LegalBasis,
  ConsentError,
  ConsentErrorCode,
  ValidationResult,
  DEFAULT_CONFIG,
  PropagationResult,
  AuditLogEntry,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-002 Universal Consent Manager
 */
export class ConsentManager {
  private version = '1.0.0';
  private config: ConsentManagerConfig;
  private consents: Map<string, ConsentRecord> = new Map();
  private purposes: Map<string, PurposeDefinition> = new Map();
  private auditLog: AuditLogEntry[] = [];

  constructor(config?: Partial<ConsentManagerConfig>) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      defaultJurisdiction: config?.defaultJurisdiction || 'EU',
      defaultLanguage: config?.defaultLanguage || 'en',
    } as ConsentManagerConfig;

    this.initializeStandardPurposes();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Consent Request & Grant
  // ==========================================================================

  /**
   * Request consent from user
   *
   * @param request - Consent request parameters
   * @returns Consent request response with notice
   */
  async requestConsent(request: ConsentRequest): Promise<ConsentRequestResponse> {
    // Validate request
    this.validateConsentRequest(request);

    // Get purpose definition
    const purpose = this.purposes.get(request.purpose);
    if (!purpose) {
      throw new ConsentError(
        ConsentErrorCode.PURPOSE_NOT_DEFINED,
        `Purpose '${request.purpose}' is not defined`
      );
    }

    // Generate request ID
    const requestId = request.requestId || this.generateId('req');

    // Build consent notice
    const language = request.language || this.config.defaultLanguage;
    const notice = {
      title: `Consent for ${purpose.code}`,
      description: purpose.description[language] || purpose.description['en'],
      processingDetails: `Your data will be processed by: ${purpose.processors.join(', ')}`,
      retention: `Data retention period: ${purpose.retention}`,
      rights: 'You can withdraw this consent at any time without affecting the lawfulness of processing based on consent before its withdrawal.',
      policyUrl: purpose.policyUrl,
      processors: purpose.processors,
    };

    // Calculate expiration (if applicable)
    const expiresAt = this.calculateExpiration(purpose.retention);

    // Log request
    this.logAudit({
      action: 'consent_requested',
      actor: request.userId,
      userId: request.userId,
      purpose: request.purpose,
      metadata: request.metadata,
    });

    return {
      requestId,
      userId: request.userId,
      purpose: request.purpose,
      notice,
      expiresAt,
      metadata: request.metadata,
    };
  }

  /**
   * Grant consent
   *
   * @param grant - Consent grant parameters
   * @returns Consent grant response with receipt
   */
  async grantConsent(grant: ConsentGrant): Promise<ConsentGrantResponse> {
    if (!grant.accepted) {
      return this.denyConsent(grant);
    }

    // Generate IDs
    const consentId = this.generateId('consent');
    const receiptId = this.generateId('receipt');

    // Get purpose
    const purposeCode = grant.requestId.split('_')[0]; // Simplified
    const purpose = this.purposes.get(purposeCode);

    if (!purpose) {
      throw new ConsentError(
        ConsentErrorCode.PURPOSE_NOT_DEFINED,
        `Purpose not found for request ${grant.requestId}`
      );
    }

    // Create consent record
    const now = new Date();
    const expiresAt = this.calculateExpiration(purpose.retention);

    const consent: ConsentRecord = {
      consentId,
      userId: grant.userId,
      receiptId,
      purpose: purpose.code,
      status: ConsentStatus.GRANTED,
      scope: purpose.dataCategories,
      legalBasis: purpose.legalBasis,
      jurisdiction: this.config.defaultJurisdiction,
      version: purpose.version,
      grantedAt: now,
      expiresAt,
      lastChecked: now,
      language: this.config.defaultLanguage,
      noticeText: purpose.description[this.config.defaultLanguage] || '',
      userAgent: grant.metadata?.userAgent,
      ipAddress: grant.metadata?.ipAddress,
      geoLocation: grant.metadata?.geoLocation,
      signature: this.generateSignature(consentId, grant.userId, purpose.code),
      signatureAlgorithm: this.config.signatureAlgorithm || 'SHA256-RSA',
      history: [{
        eventId: this.generateId('evt'),
        eventType: ConsentEventType.GRANTED,
        timestamp: now,
        actor: grant.userId,
        newStatus: ConsentStatus.GRANTED,
      }],
    };

    // Store consent
    this.consents.set(consentId, consent);

    // Propagate consent
    if (this.config.propagation?.enabled) {
      await this.propagateConsent(consent, ConsentEventType.GRANTED);
    }

    // Log grant
    this.logAudit({
      action: 'consent_granted',
      actor: grant.userId,
      consentId,
      userId: grant.userId,
      purpose: purpose.code,
      newState: ConsentStatus.GRANTED,
      metadata: grant.metadata,
    });

    return {
      consentId,
      status: ConsentStatus.GRANTED,
      grantedAt: now,
      expiresAt,
      receiptId,
      receiptUrl: `https://consent.wiastandards.com/receipts/${receiptId}`,
      signature: consent.signature,
    };
  }

  /**
   * Deny consent
   */
  private async denyConsent(grant: ConsentGrant): Promise<ConsentGrantResponse> {
    const consentId = this.generateId('consent');
    const receiptId = this.generateId('receipt');

    // Log denial
    this.logAudit({
      action: 'consent_denied',
      actor: grant.userId,
      userId: grant.userId,
      metadata: grant.metadata,
    });

    return {
      consentId,
      status: ConsentStatus.DENIED,
      grantedAt: new Date(),
      receiptId,
      receiptUrl: `https://consent.wiastandards.com/receipts/${receiptId}`,
      signature: this.generateSignature(consentId, grant.userId, 'denied'),
    };
  }

  // ==========================================================================
  // Consent Check & Validation
  // ==========================================================================

  /**
   * Check if user has valid consent for purpose
   *
   * @param check - Consent check parameters
   * @returns Consent status
   */
  async checkConsent(check: ConsentCheck): Promise<ConsentCheckResponse> {
    // Find active consent for user and purpose
    const consent = Array.from(this.consents.values()).find(
      c =>
        c.userId === check.userId &&
        c.purpose === check.purpose &&
        (c.status === ConsentStatus.GRANTED || c.status === ConsentStatus.RENEWED)
    );

    if (!consent) {
      return {
        hasConsent: false,
        reason: 'No active consent found',
      };
    }

    // Check expiration
    if (consent.expiresAt && new Date() > consent.expiresAt) {
      consent.status = ConsentStatus.EXPIRED;
      return {
        hasConsent: false,
        consentId: consent.consentId,
        status: ConsentStatus.EXPIRED,
        reason: 'Consent has expired',
      };
    }

    // Check scope (if specified)
    if (check.scope && check.scope.length > 0) {
      const hasAllScopes = check.scope.every(s => consent.scope.includes(s));
      if (!hasAllScopes) {
        return {
          hasConsent: false,
          consentId: consent.consentId,
          status: consent.status,
          reason: 'Required data categories not in consent scope',
        };
      }
    }

    // Update last checked
    consent.lastChecked = new Date();

    // Log check
    this.logAudit({
      action: 'consent_checked',
      actor: 'system',
      consentId: consent.consentId,
      userId: check.userId,
      purpose: check.purpose,
    });

    return {
      hasConsent: true,
      consentId: consent.consentId,
      status: consent.status,
      grantedAt: consent.grantedAt,
      expiresAt: consent.expiresAt,
      scope: consent.scope,
    };
  }

  // ==========================================================================
  // Consent Revocation
  // ==========================================================================

  /**
   * Revoke consent
   *
   * @param revocation - Revocation parameters
   * @returns Revocation response
   */
  async revokeConsent(revocation: ConsentRevocation): Promise<ConsentRevocationResponse> {
    // Find consent
    let consent: ConsentRecord | undefined;

    if (revocation.consentId) {
      consent = this.consents.get(revocation.consentId);
    } else if (revocation.purpose) {
      consent = Array.from(this.consents.values()).find(
        c => c.userId === revocation.userId && c.purpose === revocation.purpose
      );
    }

    if (!consent) {
      throw new ConsentError(
        ConsentErrorCode.CONSENT_NOT_FOUND,
        'Consent not found'
      );
    }

    // Check if already revoked
    if (consent.status === ConsentStatus.REVOKED) {
      throw new ConsentError(
        ConsentErrorCode.ALREADY_REVOKED,
        'Consent is already revoked'
      );
    }

    // Revoke consent
    const now = new Date();
    const previousStatus = consent.status;
    consent.status = ConsentStatus.REVOKED;
    consent.revokedAt = now;

    // Add to history
    consent.history.push({
      eventId: this.generateId('evt'),
      eventType: ConsentEventType.REVOKED,
      timestamp: now,
      actor: revocation.userId,
      reason: revocation.reason,
      previousStatus,
      newStatus: ConsentStatus.REVOKED,
    });

    // Propagate revocation
    let propagated = false;
    if (revocation.cascadeToProcessors !== false && this.config.propagation?.enabled) {
      const results = await this.propagateConsent(consent, ConsentEventType.REVOKED);
      propagated = results.every(r => r.success);
    }

    // Log revocation
    this.logAudit({
      action: 'consent_revoked',
      actor: revocation.userId,
      consentId: consent.consentId,
      userId: revocation.userId,
      purpose: consent.purpose,
      previousState: previousStatus,
      newState: ConsentStatus.REVOKED,
      metadata: { reason: revocation.reason },
    });

    const revocationId = this.generateId('rev');

    return {
      revocationId,
      consentId: consent.consentId,
      status: ConsentStatus.REVOKED,
      revokedAt: now,
      propagated,
      dataDeleted: revocation.deleteData,
    };
  }

  // ==========================================================================
  // Consent Listing & Export
  // ==========================================================================

  /**
   * List consents for user
   *
   * @param request - List request parameters
   * @returns List of consents
   */
  async listConsents(request: ConsentListRequest): Promise<ConsentListResponse> {
    // Filter consents
    let consents = Array.from(this.consents.values()).filter(
      c => c.userId === request.userId
    );

    // Apply filters
    if (request.status && request.status.length > 0) {
      consents = consents.filter(c => request.status!.includes(c.status));
    }

    if (request.purpose && request.purpose.length > 0) {
      consents = consents.filter(c => request.purpose!.includes(c.purpose));
    }

    if (request.jurisdiction) {
      consents = consents.filter(c => c.jurisdiction === request.jurisdiction);
    }

    // Sort by granted date (newest first)
    consents.sort((a, b) => {
      const aTime = a.grantedAt?.getTime() || 0;
      const bTime = b.grantedAt?.getTime() || 0;
      return bTime - aTime;
    });

    // Paginate
    const limit = request.limit || 50;
    const offset = request.offset || 0;
    const total = consents.length;
    const page = consents.slice(offset, offset + limit);

    // Map to summaries
    const summaries = page.map(c => ({
      consentId: c.consentId,
      purpose: c.purpose,
      status: c.status,
      grantedAt: c.grantedAt,
      expiresAt: c.expiresAt,
      scope: c.scope,
    }));

    return {
      consents: summaries,
      total,
      limit,
      offset,
    };
  }

  /**
   * Export consent history
   *
   * @param exportRequest - Export parameters
   * @returns Export response with download URL
   */
  async exportConsent(exportRequest: ConsentExport): Promise<ConsentExportResponse> {
    const consents = Array.from(this.consents.values()).filter(
      c => c.userId === exportRequest.userId
    );

    const exportId = this.generateId('export');
    const data = {
      userId: exportRequest.userId,
      exportedAt: new Date().toISOString(),
      consents: exportRequest.includeHistory
        ? consents
        : consents.map(c => ({ ...c, history: undefined })),
    };

    // Generate download URL (in real implementation, this would upload to storage)
    const downloadUrl = `https://consent.wiastandards.com/exports/${exportId}.${exportRequest.format}`;

    // Log export
    this.logAudit({
      action: 'consent_exported',
      actor: exportRequest.userId,
      userId: exportRequest.userId,
      metadata: { format: exportRequest.format, exportId },
    });

    return {
      exportId,
      format: exportRequest.format,
      downloadUrl,
      expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
      size: JSON.stringify(data).length,
    };
  }

  // ==========================================================================
  // Purpose Management
  // ==========================================================================

  /**
   * Register a consent purpose
   *
   * @param purpose - Purpose definition
   */
  registerPurpose(purpose: PurposeDefinition): void {
    this.purposes.set(purpose.code, purpose);
  }

  /**
   * Get purpose definition
   *
   * @param code - Purpose code
   * @returns Purpose definition
   */
  getPurpose(code: string): PurposeDefinition | undefined {
    return this.purposes.get(code);
  }

  /**
   * List all registered purposes
   */
  listPurposes(): PurposeDefinition[] {
    return Array.from(this.purposes.values());
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Initialize standard consent purposes
   */
  private initializeStandardPurposes(): void {
    const standardPurposes: PurposeDefinition[] = [
      {
        code: 'email_marketing',
        category: 'marketing',
        required: false,
        legalBasis: LegalBasis.CONSENT,
        dataCategories: ['email', 'name', 'preferences'],
        retention: 'P2Y',
        description: {
          en: 'Send promotional emails about products and features',
          ko: '제품 및 기능에 대한 홍보 이메일 발송',
        },
        processors: ['sendgrid', 'mailchimp'],
        jurisdictions: ['EU', 'US-CA', 'CA', 'BR'],
        version: '1.0',
      },
      {
        code: 'analytics',
        category: 'functional',
        required: false,
        legalBasis: LegalBasis.CONSENT,
        dataCategories: ['usage_data', 'device_id', 'ip_address'],
        retention: 'P1Y',
        description: {
          en: 'Analyze usage patterns to improve our services',
          ko: '서비스 개선을 위한 사용 패턴 분석',
        },
        processors: ['google_analytics', 'mixpanel'],
        jurisdictions: ['EU', 'US-CA', 'CA', 'BR'],
        version: '1.0',
      },
      {
        code: 'personalization',
        category: 'functional',
        required: false,
        legalBasis: LegalBasis.CONSENT,
        dataCategories: ['preferences', 'browsing_history', 'user_id'],
        retention: 'P1Y',
        description: {
          en: 'Personalize content and recommendations',
          ko: '콘텐츠 및 추천 개인화',
        },
        processors: [],
        jurisdictions: ['EU', 'US-CA', 'CA', 'BR'],
        version: '1.0',
      },
    ];

    standardPurposes.forEach(p => this.registerPurpose(p));
  }

  /**
   * Validate consent request
   */
  private validateConsentRequest(request: ConsentRequest): void {
    if (!request.userId) {
      throw new ConsentError(
        ConsentErrorCode.INVALID_REQUEST,
        'userId is required'
      );
    }

    if (!request.purpose) {
      throw new ConsentError(
        ConsentErrorCode.INVALID_REQUEST,
        'purpose is required'
      );
    }

    if (!request.scope || request.scope.length === 0) {
      throw new ConsentError(
        ConsentErrorCode.INVALID_REQUEST,
        'scope is required and must not be empty'
      );
    }
  }

  /**
   * Calculate expiration date from retention period
   */
  private calculateExpiration(retention: string): Date | undefined {
    if (retention === 'indefinite') {
      return undefined;
    }

    // Parse ISO 8601 duration (simplified)
    // P2Y = 2 years, P1Y = 1 year, etc.
    const match = retention.match(/P(\d+)Y/);
    if (match) {
      const years = parseInt(match[1], 10);
      const expiresAt = new Date();
      expiresAt.setFullYear(expiresAt.getFullYear() + years);
      return expiresAt;
    }

    return undefined;
  }

  /**
   * Generate unique ID
   */
  private generateId(prefix: string): string {
    const timestamp = Date.now();
    const random = Math.random().toString(36).substring(2, 11);
    return `${prefix}_${timestamp}_${random}`;
  }

  /**
   * Generate cryptographic signature
   */
  private generateSignature(consentId: string, userId: string, purpose: string): string {
    // In real implementation, use actual cryptographic signing
    const data = `${consentId}:${userId}:${purpose}`;
    return `SHA256:${Buffer.from(data).toString('base64').substring(0, 32)}`;
  }

  /**
   * Propagate consent to integrated systems
   */
  private async propagateConsent(
    consent: ConsentRecord,
    eventType: ConsentEventType
  ): Promise<PropagationResult[]> {
    if (!this.config.propagation?.enabled) {
      return [];
    }

    const results: PropagationResult[] = [];
    const targets = this.config.propagation.targets || [];

    for (const target of targets) {
      if (!target.enabled) continue;

      const startTime = Date.now();
      try {
        // Simulate propagation (in real implementation, make actual API calls)
        await new Promise(resolve => setTimeout(resolve, 50));

        results.push({
          targetId: target.id,
          success: true,
          timestamp: new Date(),
          latency: Date.now() - startTime,
          retries: 0,
        });
      } catch (error) {
        results.push({
          targetId: target.id,
          success: false,
          timestamp: new Date(),
          latency: Date.now() - startTime,
          error: error instanceof Error ? error.message : 'Unknown error',
          retries: 0,
        });
      }
    }

    return results;
  }

  /**
   * Log audit entry
   */
  private logAudit(entry: Partial<AuditLogEntry>): void {
    const auditEntry: AuditLogEntry = {
      auditId: this.generateId('audit'),
      timestamp: new Date(),
      action: entry.action || 'unknown',
      actor: entry.actor || 'system',
      consentId: entry.consentId,
      userId: entry.userId,
      purpose: entry.purpose,
      previousState: entry.previousState,
      newState: entry.newState,
      metadata: entry.metadata,
      signature: this.generateSignature(
        entry.consentId || '',
        entry.userId || '',
        entry.purpose || ''
      ),
    };

    this.auditLog.push(auditEntry);
  }

  /**
   * Get audit log
   */
  getAuditLog(): AuditLogEntry[] {
    return [...this.auditLog];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Request consent (standalone function)
 */
export async function requestConsent(
  request: ConsentRequest,
  config?: Partial<ConsentManagerConfig>
): Promise<ConsentRequestResponse> {
  const manager = new ConsentManager(config);
  return manager.requestConsent(request);
}

/**
 * Check consent (standalone function)
 */
export async function checkConsent(
  check: ConsentCheck,
  config?: Partial<ConsentManagerConfig>
): Promise<ConsentCheckResponse> {
  const manager = new ConsentManager(config);
  return manager.checkConsent(check);
}

/**
 * Revoke consent (standalone function)
 */
export async function revokeConsent(
  revocation: ConsentRevocation,
  config?: Partial<ConsentManagerConfig>
): Promise<ConsentRevocationResponse> {
  const manager = new ConsentManager(config);
  return manager.revokeConsent(revocation);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ConsentManager };
export default ConsentManager;
