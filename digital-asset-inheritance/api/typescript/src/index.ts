/**
 * WIA-LEG-006: Digital Asset Inheritance Standard
 * TypeScript SDK
 *
 * @version 2.0.0
 * @license MIT
 * @author WIA - World Certification Industry Association
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import { Validator } from 'jsonschema';
import * as types from './types';

// Export all types
export * from './types';

/**
 * Main client for WIA-LEG-006 Digital Asset Inheritance API
 */
export class WIAInheritanceClient {
  private client: AxiosInstance;
  private validator: Validator;

  constructor(config: types.WIAClientConfig) {
    this.client = axios.create({
      baseURL: config.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'LEG-006',
        'X-WIA-Version': '2.0.0',
        ...(config.apiKey && { 'Authorization': `Bearer ${config.apiKey}` })
      }
    });

    this.validator = new Validator();

    // Setup interceptors for retry logic
    if (config.retryAttempts) {
      this.setupRetryInterceptor(config.retryAttempts);
    }
  }

  // ========== Plan Management ==========

  /**
   * Create a new inheritance plan
   */
  async createPlan(plan: types.InheritancePlan): Promise<types.APIResponse<CreatePlanResponse>> {
    const validation = this.validatePlan(plan);
    if (!validation.valid) {
      throw new Error(`Plan validation failed: ${JSON.stringify(validation.errors)}`);
    }

    const response = await this.client.post<types.APIResponse<CreatePlanResponse>>(
      '/plans',
      plan
    );
    return response.data;
  }

  /**
   * Retrieve an inheritance plan
   */
  async getPlan(
    planId: string,
    options?: GetPlanOptions
  ): Promise<types.APIResponse<types.InheritancePlan>> {
    const params = new URLSearchParams();
    if (options?.version) params.append('version', options.version);
    if (options?.include) params.append('include', options.include.join(','));
    if (options?.decrypt) params.append('decrypt', 'true');

    const response = await this.client.get<types.APIResponse<types.InheritancePlan>>(
      `/plans/${planId}?${params.toString()}`
    );
    return response.data;
  }

  /**
   * Update an existing plan
   */
  async updatePlan(
    planId: string,
    changes: PlanChanges
  ): Promise<types.APIResponse<UpdatePlanResponse>> {
    const response = await this.client.put<types.APIResponse<UpdatePlanResponse>>(
      `/plans/${planId}`,
      changes
    );
    return response.data;
  }

  /**
   * Revoke an inheritance plan
   */
  async revokePlan(
    planId: string,
    reason: string
  ): Promise<types.APIResponse<RevokePlanResponse>> {
    const response = await this.client.delete<types.APIResponse<RevokePlanResponse>>(
      `/plans/${planId}`,
      {
        data: { revocationReason: reason }
      }
    );
    return response.data;
  }

  // ========== Asset Discovery ==========

  /**
   * Discover digital assets across blockchains and platforms
   */
  async discoverAssets(
    criteria: AssetDiscoveryCriteria
  ): Promise<types.APIResponse<AssetDiscoveryResponse>> {
    const response = await this.client.post<types.APIResponse<AssetDiscoveryResponse>>(
      '/assets/discover',
      criteria
    );
    return response.data;
  }

  // ========== Beneficiary Management ==========

  /**
   * Verify beneficiary identity and inheritance rights
   */
  async verifyBeneficiary(
    request: BeneficiaryVerificationRequest
  ): Promise<types.APIResponse<BeneficiaryVerificationResponse>> {
    const response = await this.client.post<types.APIResponse<BeneficiaryVerificationResponse>>(
      '/beneficiaries/verify',
      request
    );
    return response.data;
  }

  // ========== Trigger Management ==========

  /**
   * Owner check-in to reset dead man's switch
   */
  async checkIn(
    planId: string,
    authentication: CheckInAuthentication
  ): Promise<types.APIResponse<CheckInResponse>> {
    const response = await this.client.post<types.APIResponse<CheckInResponse>>(
      '/triggers/checkin',
      {
        planId,
        authentication
      }
    );
    return response.data;
  }

  /**
   * Trigger inheritance execution
   */
  async executeInheritance(
    request: ExecuteInheritanceRequest
  ): Promise<types.APIResponse<ExecuteInheritanceResponse>> {
    const response = await this.client.post<types.APIResponse<ExecuteInheritanceResponse>>(
      '/triggers/execute',
      request
    );
    return response.data;
  }

  // ========== Webhook Management ==========

  /**
   * Register a webhook endpoint
   */
  async registerWebhook(
    webhook: WebhookConfig
  ): Promise<types.APIResponse<WebhookRegistrationResponse>> {
    const response = await this.client.post<types.APIResponse<WebhookRegistrationResponse>>(
      '/webhooks',
      webhook
    );
    return response.data;
  }

  /**
   * List registered webhooks
   */
  async listWebhooks(): Promise<types.APIResponse<WebhookConfig[]>> {
    const response = await this.client.get<types.APIResponse<WebhookConfig[]>>('/webhooks');
    return response.data;
  }

  /**
   * Delete a webhook
   */
  async deleteWebhook(webhookId: string): Promise<types.APIResponse<void>> {
    const response = await this.client.delete<types.APIResponse<void>>(`/webhooks/${webhookId}`);
    return response.data;
  }

  // ========== Validation ==========

  /**
   * Validate an inheritance plan against WIA-LEG-006 schema
   */
  validatePlan(plan: types.InheritancePlan): types.ValidationResult {
    const errors: types.ValidationError[] = [];
    const warnings: types.ValidationWarning[] = [];

    // Required field validation
    if (plan.wiaStandard !== 'LEG-006') {
      errors.push({
        field: 'wiaStandard',
        message: 'Must be "LEG-006"',
        code: 'INVALID_STANDARD'
      });
    }

    if (!plan.inheritancePlan?.planId) {
      errors.push({
        field: 'inheritancePlan.planId',
        message: 'Plan ID is required',
        code: 'MISSING_PLAN_ID'
      });
    }

    // Validate beneficiary allocation totals
    const totalAllocation = plan.inheritancePlan.beneficiaries.reduce(
      (sum, b) => sum + b.allocation.percentage,
      0
    );

    if (totalAllocation > 100) {
      errors.push({
        field: 'beneficiaries',
        message: `Total allocation (${totalAllocation}%) exceeds 100%`,
        code: 'ALLOCATION_EXCEEDS_100'
      });
    } else if (totalAllocation < 100) {
      warnings.push({
        field: 'beneficiaries',
        message: `Total allocation (${totalAllocation}%) is less than 100%`,
        severity: 'medium'
      });
    }

    // Validate asset references in distribution rules
    const assetIds = new Set(plan.inheritancePlan.assets.map(a => a.assetId));
    const beneficiaryIds = new Set(plan.inheritancePlan.beneficiaries.map(b => b.beneficiaryId));

    plan.inheritancePlan.distribution.rules.forEach(rule => {
      if (rule.asset && !assetIds.has(rule.asset)) {
        errors.push({
          field: `distribution.rules.${rule.ruleId}.asset`,
          message: `Asset ID "${rule.asset}" not found in assets array`,
          code: 'INVALID_ASSET_REFERENCE'
        });
      }

      if (rule.beneficiary && !beneficiaryIds.has(rule.beneficiary)) {
        errors.push({
          field: `distribution.rules.${rule.ruleId}.beneficiary`,
          message: `Beneficiary ID "${rule.beneficiary}" not found in beneficiaries array`,
          code: 'INVALID_BENEFICIARY_REFERENCE'
        });
      }
    });

    return {
      valid: errors.length === 0,
      errors,
      warnings
    };
  }

  // ========== Helper Methods ==========

  /**
   * Generate a new UUID v4 for plan IDs
   */
  static generatePlanId(): string {
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
  }

  /**
   * Calculate hash of plan for version tracking
   */
  static async hashPlan(plan: types.InheritancePlan): Promise<string> {
    const planString = JSON.stringify(plan);
    const encoder = new TextEncoder();
    const data = encoder.encode(planString);

    // Use Web Crypto API or Node.js crypto
    if (typeof window !== 'undefined' && window.crypto?.subtle) {
      const hashBuffer = await window.crypto.subtle.digest('SHA-256', data);
      const hashArray = Array.from(new Uint8Array(hashBuffer));
      return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
    } else {
      // Node.js environment
      const crypto = require('crypto');
      return crypto.createHash('sha256').update(planString).digest('hex');
    }
  }

  /**
   * Setup retry interceptor for failed requests
   */
  private setupRetryInterceptor(maxRetries: number): void {
    this.client.interceptors.response.use(
      response => response,
      async error => {
        const config = error.config as AxiosRequestConfig & { retryCount?: number };

        if (!config || !config.retryCount) {
          config.retryCount = 0;
        }

        if (config.retryCount < maxRetries && this.shouldRetry(error)) {
          config.retryCount += 1;
          const delay = Math.pow(2, config.retryCount) * 1000; // Exponential backoff
          await new Promise(resolve => setTimeout(resolve, delay));
          return this.client.request(config);
        }

        return Promise.reject(error);
      }
    );
  }

  /**
   * Determine if request should be retried
   */
  private shouldRetry(error: any): boolean {
    const status = error.response?.status;
    return status === 429 || // Rate limit
           status === 503 || // Service unavailable
           status === 504 || // Gateway timeout
           !status; // Network error
  }
}

// ========== Additional Types ==========

export interface CreatePlanResponse {
  planId: string;
  version: string;
  createdAt: string;
  backupLocations: string[];
}

export interface GetPlanOptions {
  version?: string;
  include?: string[];
  decrypt?: boolean;
}

export interface PlanChanges {
  currentVersion: string;
  changes: {
    assets?: {
      add?: types.Asset[];
      update?: types.Asset[];
      remove?: string[];
    };
    beneficiaries?: {
      add?: types.Beneficiary[];
      update?: types.Beneficiary[];
      remove?: string[];
    };
  };
  changeReason: string;
  authentication: {
    mfa: string;
    biometric?: string;
  };
}

export interface UpdatePlanResponse {
  planId: string;
  newVersion: string;
  previousVersionHash: string;
  changesApplied: number;
  warnings?: string[];
}

export interface RevokePlanResponse {
  planId: string;
  revokedAt: string;
  revocationHash: string;
}

export interface AssetDiscoveryCriteria {
  owner: string;
  searchCriteria: {
    walletAddresses?: string[];
    blockchains?: string[];
    platforms?: string[];
    timeRange?: {
      from: string;
      to: string;
    };
  };
  includeHistory: boolean;
  estimateValues: boolean;
}

export interface AssetDiscoveryResponse {
  discoveryId: string;
  status: 'completed' | 'in-progress' | 'failed';
  assetsFound: number;
  totalEstimatedValue: types.MonetaryValue;
  assets: types.Asset[];
  recommendations?: string[];
}

export interface BeneficiaryVerificationRequest {
  planId: string;
  beneficiary: {
    did: string;
    credentials: {
      governmentId?: string;
      verifiableCredential?: string;
      biometric?: string;
    };
  };
  verificationLevel: 'low' | 'medium' | 'high-assurance';
}

export interface BeneficiaryVerificationResponse {
  verificationId: string;
  status: 'verified' | 'pending' | 'rejected';
  confidence: number;
  beneficiaryId: string;
  entitlements: {
    assets: string[];
    percentage: number;
    estimatedValue: number;
    conditions?: Record<string, any>;
  };
  nextSteps: string[];
}

export interface CheckInAuthentication {
  method: 'biometric' | 'multi-factor';
  data: string;
  location?: {
    latitude: number;
    longitude: number;
  };
  deviceId?: string;
}

export interface CheckInResponse {
  checkInId: string;
  status: 'confirmed' | 'failed';
  nextCheckInDue: string;
  daysUntilTrigger: number;
  notificationSettings: {
    firstWarning: number;
    secondWarning: number;
    finalWarning: number;
  };
}

export interface ExecuteInheritanceRequest {
  planId: string;
  trigger: {
    type: types.TriggerType;
    reason: string;
    documentation: DocumentSubmission[];
  };
  executors: ExecutorSignature[];
}

export interface DocumentSubmission {
  type: types.DocumentationType;
  issuer: string;
  documentHash: string;
  storageUrl: string;
}

export interface ExecutorSignature {
  did: string;
  signature: string;
}

export interface ExecuteInheritanceResponse {
  executionId: string;
  status: 'initiated' | 'rejected';
  validationResults: {
    documentationComplete: boolean;
    signaturesValid: boolean;
    legalRequirementsMet: boolean;
  };
  timeline: {
    notificationPeriod: string;
    contestPeriod: string;
    estimatedDistribution: string;
  };
  nextSteps: string[];
}

export interface WebhookConfig {
  url: string;
  events: string[];
  authentication: {
    method: 'hmac-sha256';
    secret: string;
  };
  retryPolicy: {
    maxAttempts: number;
    backoff: 'exponential' | 'linear';
  };
}

export interface WebhookRegistrationResponse {
  webhookId: string;
  status: 'active';
  createdAt: string;
}

/**
 * Utility function to create a minimal valid inheritance plan template
 */
export function createMinimalPlan(owner: types.Owner): types.InheritancePlan {
  return {
    wiaStandard: 'LEG-006',
    version: '2.0.0',
    inheritancePlan: {
      planId: WIAInheritanceClient.generatePlanId(),
      metadata: {
        created: new Date().toISOString(),
        lastModified: new Date().toISOString(),
        version: '1.0.0',
        title: 'Digital Asset Inheritance Plan',
        description: 'Comprehensive digital asset inheritance planning',
        jurisdiction: 'US-CA',
        language: 'en-US',
        currency: 'USD',
        tags: [],
        confidentiality: 'private'
      },
      owner,
      assets: [],
      beneficiaries: [],
      distribution: {
        method: 'equal',
        rules: [],
        simultaneousDeath: {
          presumption: 'beneficiary-predeceased',
          survivorshipPeriod: '30 days'
        }
      },
      triggers: [],
      legal: {
        governingLaw: 'US-CA',
        witnesses: [],
        noContest: false
      }
    },
    signatures: []
  };
}

// Default export
export default WIAInheritanceClient;
