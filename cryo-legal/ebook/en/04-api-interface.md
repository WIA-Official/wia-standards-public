# Chapter 4: API Interface - REST, GraphQL, and WebSocket

## Complete API Implementation for Cryo Legal Standard

This chapter provides comprehensive API implementations for the WIA Cryo Legal Standard, including REST endpoints, GraphQL schema, and real-time WebSocket connections.

## REST API Implementation

```typescript
/**
 * WIA Cryo Legal Standard - REST API Implementation
 * Express-based RESTful API for legal operations
 */

import express, { Router, Request, Response, NextFunction } from 'express';
import { z } from 'zod';
import {
  WIACryoLegalProject,
  Contract,
  ContractSchema,
  Dispute,
  DisputeSchema,
  ComplianceRequirement,
  ComplianceRequirementSchema,
  Regulation,
  RegulationSchema,
} from './types';

// ============================================================================
// API Configuration
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
  rateLimiting?: RateLimitConfig;
}

export interface RateLimitConfig {
  windowMs: number;
  maxRequests: number;
}

// ============================================================================
// Request/Response Types
// ============================================================================

export interface ProjectResponse {
  id: string;
  name: string;
  status: string;
  createdAt: string;
  updatedAt?: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMeta;
}

export interface APIError {
  code: string;
  message: string;
  details?: unknown;
}

export interface ResponseMeta {
  requestId: string;
  timestamp: string;
  processingTime: number;
}

// ============================================================================
// Middleware
// ============================================================================

export function authMiddleware(
  req: Request,
  res: Response,
  next: NextFunction
): void {
  const apiKey = req.headers['x-api-key'] as string;
  const authHeader = req.headers.authorization;

  if (!apiKey && !authHeader) {
    res.status(401).json({
      success: false,
      error: {
        code: 'UNAUTHORIZED',
        message: 'API key or authorization token required',
      },
    });
    return;
  }

  // Validate API key or token
  if (apiKey) {
    if (!validateApiKey(apiKey)) {
      res.status(401).json({
        success: false,
        error: {
          code: 'INVALID_API_KEY',
          message: 'Invalid API key',
        },
      });
      return;
    }
  }

  if (authHeader) {
    const token = authHeader.replace('Bearer ', '');
    const validation = validateJWT(token);

    if (!validation.valid) {
      res.status(401).json({
        success: false,
        error: {
          code: 'INVALID_TOKEN',
          message: validation.error || 'Invalid authorization token',
        },
      });
      return;
    }

    (req as any).user = validation.payload;
  }

  next();
}

function validateApiKey(apiKey: string): boolean {
  // API key validation logic
  return apiKey.startsWith('wia_') && apiKey.length === 40;
}

function validateJWT(token: string): { valid: boolean; payload?: any; error?: string } {
  // JWT validation logic
  try {
    // Simplified - use proper JWT library in production
    const parts = token.split('.');
    if (parts.length !== 3) {
      return { valid: false, error: 'Invalid token format' };
    }
    return { valid: true, payload: JSON.parse(atob(parts[1])) };
  } catch {
    return { valid: false, error: 'Token decode failed' };
  }
}

export function validationMiddleware<T>(schema: z.ZodSchema<T>) {
  return (req: Request, res: Response, next: NextFunction): void => {
    const result = schema.safeParse(req.body);

    if (!result.success) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Request validation failed',
          details: result.error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
          })),
        },
      });
      return;
    }

    req.body = result.data;
    next();
  };
}

export function errorHandler(
  error: Error,
  req: Request,
  res: Response,
  next: NextFunction
): void {
  console.error('API Error:', error);

  if (error instanceof z.ZodError) {
    res.status(400).json({
      success: false,
      error: {
        code: 'VALIDATION_ERROR',
        message: 'Data validation failed',
        details: error.errors,
      },
    });
    return;
  }

  res.status(500).json({
    success: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: 'An internal error occurred',
    },
  });
}

// ============================================================================
// Legal Framework Router
// ============================================================================

export function createLegalFrameworkRouter(service: LegalFrameworkService): Router {
  const router = Router();

  // Get all regulations
  router.get('/regulations', async (req: Request, res: Response) => {
    try {
      const { category, jurisdiction, status } = req.query;
      const filters: RegulationFilters = {
        category: category as string,
        jurisdiction: jurisdiction as string,
        status: status as string,
      };

      const regulations = await service.getRegulations(filters);

      res.json({
        success: true,
        data: regulations,
        meta: {
          requestId: crypto.randomUUID(),
          timestamp: new Date().toISOString(),
          processingTime: 0,
        },
      });
    } catch (error) {
      throw error;
    }
  });

  // Get regulation by ID
  router.get('/regulations/:id', async (req: Request, res: Response) => {
    try {
      const regulation = await service.getRegulationById(req.params.id);

      if (!regulation) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: `Regulation ${req.params.id} not found`,
          },
        });
        return;
      }

      res.json({
        success: true,
        data: regulation,
      });
    } catch (error) {
      throw error;
    }
  });

  // Create regulation
  router.post(
    '/regulations',
    validationMiddleware(RegulationSchema),
    async (req: Request, res: Response) => {
      try {
        const regulation = await service.createRegulation(req.body);

        res.status(201).json({
          success: true,
          data: regulation,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // Update regulation
  router.put(
    '/regulations/:id',
    validationMiddleware(RegulationSchema.partial()),
    async (req: Request, res: Response) => {
      try {
        const regulation = await service.updateRegulation(req.params.id, req.body);

        res.json({
          success: true,
          data: regulation,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // Get legal rights
  router.get('/rights', async (req: Request, res: Response) => {
    try {
      const { holder } = req.query;
      const rights = await service.getRights(holder as string);

      res.json({
        success: true,
        data: rights,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get liabilities
  router.get('/liabilities', async (req: Request, res: Response) => {
    try {
      const { type } = req.query;
      const liabilities = await service.getLiabilities(type as string);

      res.json({
        success: true,
        data: liabilities,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get legal precedents
  router.get('/precedents', async (req: Request, res: Response) => {
    try {
      const { jurisdiction, dateFrom, dateTo } = req.query;
      const precedents = await service.getPrecedents({
        jurisdiction: jurisdiction as string,
        dateFrom: dateFrom as string,
        dateTo: dateTo as string,
      });

      res.json({
        success: true,
        data: precedents,
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// Contract Router
// ============================================================================

export function createContractRouter(service: ContractService): Router {
  const router = Router();

  // List contracts with pagination
  router.get('/', async (req: Request, res: Response) => {
    try {
      const { type, status, limit = 20, offset = 0 } = req.query;

      const result = await service.listContracts({
        type: type as string,
        status: status as string,
        limit: Number(limit),
        offset: Number(offset),
      });

      res.json({
        success: true,
        data: result.contracts,
        pagination: {
          total: result.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: result.total > Number(offset) + Number(limit),
        },
      });
    } catch (error) {
      throw error;
    }
  });

  // Get contract by ID
  router.get('/:id', async (req: Request, res: Response) => {
    try {
      const contract = await service.getContractById(req.params.id);

      if (!contract) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: `Contract ${req.params.id} not found`,
          },
        });
        return;
      }

      res.json({
        success: true,
        data: contract,
      });
    } catch (error) {
      throw error;
    }
  });

  // Create contract
  router.post(
    '/',
    validationMiddleware(ContractSchema),
    async (req: Request, res: Response) => {
      try {
        const contract = await service.createContract(req.body);

        res.status(201).json({
          success: true,
          data: contract,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // Update contract
  router.put(
    '/:id',
    validationMiddleware(ContractSchema.partial()),
    async (req: Request, res: Response) => {
      try {
        const contract = await service.updateContract(req.params.id, req.body);

        res.json({
          success: true,
          data: contract,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // Add signature to contract
  router.post('/:id/signatures', async (req: Request, res: Response) => {
    try {
      const signature = await service.addSignature(req.params.id, req.body);

      res.status(201).json({
        success: true,
        data: signature,
      });
    } catch (error) {
      throw error;
    }
  });

  // Execute contract (activate)
  router.post('/:id/execute', async (req: Request, res: Response) => {
    try {
      const contract = await service.executeContract(req.params.id);

      res.json({
        success: true,
        data: contract,
        message: 'Contract executed successfully',
      });
    } catch (error) {
      throw error;
    }
  });

  // Terminate contract
  router.post('/:id/terminate', async (req: Request, res: Response) => {
    try {
      const { reason, effectiveDate } = req.body;
      const contract = await service.terminateContract(req.params.id, reason, effectiveDate);

      res.json({
        success: true,
        data: contract,
        message: 'Contract terminated successfully',
      });
    } catch (error) {
      throw error;
    }
  });

  // Get contract templates
  router.get('/templates', async (req: Request, res: Response) => {
    try {
      const { type, jurisdiction } = req.query;
      const templates = await service.getTemplates({
        type: type as string,
        jurisdiction: jurisdiction as string,
      });

      res.json({
        success: true,
        data: templates,
      });
    } catch (error) {
      throw error;
    }
  });

  // Create contract from template
  router.post('/from-template/:templateId', async (req: Request, res: Response) => {
    try {
      const contract = await service.createFromTemplate(
        req.params.templateId,
        req.body
      );

      res.status(201).json({
        success: true,
        data: contract,
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// Dispute Router
// ============================================================================

export function createDisputeRouter(service: DisputeService): Router {
  const router = Router();

  // List disputes
  router.get('/', async (req: Request, res: Response) => {
    try {
      const { type, status, limit = 20, offset = 0 } = req.query;

      const result = await service.listDisputes({
        type: type as string,
        status: status as string,
        limit: Number(limit),
        offset: Number(offset),
      });

      res.json({
        success: true,
        data: result.disputes,
        pagination: {
          total: result.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: result.total > Number(offset) + Number(limit),
        },
      });
    } catch (error) {
      throw error;
    }
  });

  // Get dispute by ID
  router.get('/:id', async (req: Request, res: Response) => {
    try {
      const dispute = await service.getDisputeById(req.params.id);

      if (!dispute) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: `Dispute ${req.params.id} not found`,
          },
        });
        return;
      }

      res.json({
        success: true,
        data: dispute,
      });
    } catch (error) {
      throw error;
    }
  });

  // File new dispute
  router.post(
    '/',
    validationMiddleware(DisputeSchema.omit({ id: true, status: true, history: true })),
    async (req: Request, res: Response) => {
      try {
        const dispute = await service.fileDispute(req.body);

        res.status(201).json({
          success: true,
          data: dispute,
        });
      } catch (error) {
        throw error;
      }
    }
  );

  // Update dispute status
  router.patch('/:id/status', async (req: Request, res: Response) => {
    try {
      const { status, reason } = req.body;
      const dispute = await service.updateDisputeStatus(
        req.params.id,
        status,
        reason
      );

      res.json({
        success: true,
        data: dispute,
      });
    } catch (error) {
      throw error;
    }
  });

  // Escalate dispute
  router.post('/:id/escalate', async (req: Request, res: Response) => {
    try {
      const { mechanism, reason } = req.body;
      const dispute = await service.escalateDispute(
        req.params.id,
        mechanism,
        reason
      );

      res.json({
        success: true,
        data: dispute,
        message: `Dispute escalated to ${mechanism}`,
      });
    } catch (error) {
      throw error;
    }
  });

  // Resolve dispute
  router.post('/:id/resolve', async (req: Request, res: Response) => {
    try {
      const resolution = req.body;
      const dispute = await service.resolveDispute(req.params.id, resolution);

      res.json({
        success: true,
        data: dispute,
        message: 'Dispute resolved successfully',
      });
    } catch (error) {
      throw error;
    }
  });

  // Add dispute event
  router.post('/:id/events', async (req: Request, res: Response) => {
    try {
      const event = await service.addDisputeEvent(req.params.id, req.body);

      res.status(201).json({
        success: true,
        data: event,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get dispute timeline
  router.get('/:id/timeline', async (req: Request, res: Response) => {
    try {
      const timeline = await service.getDisputeTimeline(req.params.id);

      res.json({
        success: true,
        data: timeline,
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// Compliance Router
// ============================================================================

export function createComplianceRouter(service: ComplianceService): Router {
  const router = Router();

  // Get compliance requirements
  router.get('/requirements', async (req: Request, res: Response) => {
    try {
      const { regulation, status } = req.query;
      const requirements = await service.getRequirements({
        regulation: regulation as string,
        status: status as string,
      });

      res.json({
        success: true,
        data: requirements,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get compliance score
  router.get('/score', async (req: Request, res: Response) => {
    try {
      const score = await service.getComplianceScore();

      res.json({
        success: true,
        data: score,
      });
    } catch (error) {
      throw error;
    }
  });

  // Update requirement status
  router.patch('/requirements/:id', async (req: Request, res: Response) => {
    try {
      const { status, evidence } = req.body;
      const requirement = await service.updateRequirementStatus(
        req.params.id,
        status,
        evidence
      );

      res.json({
        success: true,
        data: requirement,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get audit schedule
  router.get('/audits', async (req: Request, res: Response) => {
    try {
      const audits = await service.getAuditSchedule();

      res.json({
        success: true,
        data: audits,
      });
    } catch (error) {
      throw error;
    }
  });

  // Report violation
  router.post('/violations', async (req: Request, res: Response) => {
    try {
      const violation = await service.reportViolation(req.body);

      res.status(201).json({
        success: true,
        data: violation,
      });
    } catch (error) {
      throw error;
    }
  });

  // Get training records
  router.get('/training', async (req: Request, res: Response) => {
    try {
      const { employeeId } = req.query;
      const training = await service.getTrainingRecords(employeeId as string);

      res.json({
        success: true,
        data: training,
      });
    } catch (error) {
      throw error;
    }
  });

  // Record training completion
  router.post('/training/complete', async (req: Request, res: Response) => {
    try {
      const record = await service.recordTrainingCompletion(req.body);

      res.status(201).json({
        success: true,
        data: record,
      });
    } catch (error) {
      throw error;
    }
  });

  return router;
}

// ============================================================================
// Main API Application
// ============================================================================

export function createCryoLegalAPI(
  services: CryoLegalServices,
  config: APIConfig
): express.Application {
  const app = express();

  // Middleware
  app.use(express.json());
  app.use(authMiddleware);

  // Request logging
  app.use((req, res, next) => {
    const start = Date.now();

    res.on('finish', () => {
      const duration = Date.now() - start;
      console.log(`${req.method} ${req.path} ${res.statusCode} ${duration}ms`);
    });

    next();
  });

  // API Routes
  app.use('/api/v1/legal-framework', createLegalFrameworkRouter(services.legalFramework));
  app.use('/api/v1/contracts', createContractRouter(services.contracts));
  app.use('/api/v1/disputes', createDisputeRouter(services.disputes));
  app.use('/api/v1/compliance', createComplianceRouter(services.compliance));

  // Health check
  app.get('/health', (req, res) => {
    res.json({
      status: 'healthy',
      version: '1.0.0',
      timestamp: new Date().toISOString(),
    });
  });

  // Error handler
  app.use(errorHandler);

  return app;
}

// ============================================================================
// Service Interfaces
// ============================================================================

export interface CryoLegalServices {
  legalFramework: LegalFrameworkService;
  contracts: ContractService;
  disputes: DisputeService;
  compliance: ComplianceService;
}

export interface LegalFrameworkService {
  getRegulations(filters: RegulationFilters): Promise<Regulation[]>;
  getRegulationById(id: string): Promise<Regulation | null>;
  createRegulation(data: Regulation): Promise<Regulation>;
  updateRegulation(id: string, data: Partial<Regulation>): Promise<Regulation>;
  getRights(holder?: string): Promise<any[]>;
  getLiabilities(type?: string): Promise<any[]>;
  getPrecedents(filters: PrecedentFilters): Promise<any[]>;
}

export interface ContractService {
  listContracts(params: ListParams): Promise<{ contracts: Contract[]; total: number }>;
  getContractById(id: string): Promise<Contract | null>;
  createContract(data: Contract): Promise<Contract>;
  updateContract(id: string, data: Partial<Contract>): Promise<Contract>;
  addSignature(id: string, signature: any): Promise<any>;
  executeContract(id: string): Promise<Contract>;
  terminateContract(id: string, reason: string, effectiveDate: string): Promise<Contract>;
  getTemplates(filters: TemplateFilters): Promise<any[]>;
  createFromTemplate(templateId: string, data: any): Promise<Contract>;
}

export interface DisputeService {
  listDisputes(params: ListParams): Promise<{ disputes: Dispute[]; total: number }>;
  getDisputeById(id: string): Promise<Dispute | null>;
  fileDispute(data: any): Promise<Dispute>;
  updateDisputeStatus(id: string, status: string, reason: string): Promise<Dispute>;
  escalateDispute(id: string, mechanism: string, reason: string): Promise<Dispute>;
  resolveDispute(id: string, resolution: any): Promise<Dispute>;
  addDisputeEvent(id: string, event: any): Promise<any>;
  getDisputeTimeline(id: string): Promise<any[]>;
}

export interface ComplianceService {
  getRequirements(filters: RequirementFilters): Promise<ComplianceRequirement[]>;
  getComplianceScore(): Promise<any>;
  updateRequirementStatus(id: string, status: string, evidence: string[]): Promise<ComplianceRequirement>;
  getAuditSchedule(): Promise<any[]>;
  reportViolation(data: any): Promise<any>;
  getTrainingRecords(employeeId?: string): Promise<any[]>;
  recordTrainingCompletion(data: any): Promise<any>;
}

export interface RegulationFilters {
  category?: string;
  jurisdiction?: string;
  status?: string;
}

export interface PrecedentFilters {
  jurisdiction?: string;
  dateFrom?: string;
  dateTo?: string;
}

export interface ListParams {
  type?: string;
  status?: string;
  limit: number;
  offset: number;
}

export interface TemplateFilters {
  type?: string;
  jurisdiction?: string;
}

export interface RequirementFilters {
  regulation?: string;
  status?: string;
}
```

## GraphQL API Implementation

```typescript
/**
 * WIA Cryo Legal Standard - GraphQL API Implementation
 * Apollo Server based GraphQL API
 */

import { ApolloServer } from '@apollo/server';
import { expressMiddleware } from '@apollo/server/express4';
import { gql } from 'graphql-tag';
import { GraphQLDateTime } from 'graphql-scalars';

// ============================================================================
// GraphQL Type Definitions
// ============================================================================

export const typeDefs = gql`
  scalar DateTime

  # Common Types
  type ContactInfo {
    name: String!
    email: String!
    phone: String
  }

  type LegalCounsel {
    name: String!
    firm: String
    bar: String!
    contact: ContactInfo!
  }

  type Jurisdiction {
    country: String!
    state: String
    type: JurisdictionType!
    applicableLaws: [String!]!
  }

  enum JurisdictionType {
    PRIMARY
    SECONDARY
  }

  # Organization
  type Organization {
    id: ID!
    name: String!
    type: OrganizationType!
    country: String!
    registrationNumber: String
    contact: ContactInfo!
    legalCounsel: LegalCounsel
  }

  enum OrganizationType {
    FACILITY
    LAW_FIRM
    AUTHORITY
    ASSOCIATION
  }

  # Legal Framework
  type Regulation {
    id: ID!
    name: String!
    jurisdiction: String!
    category: RegulationCategory!
    requirements: [String!]!
    penalties: [String!]!
    effectiveDate: DateTime!
    lastReview: DateTime!
    status: RegulationStatus!
  }

  enum RegulationCategory {
    TISSUE_BANKING
    REPRODUCTIVE
    RESEARCH
    TRANSPLANTATION
    PRIVACY
    CONSUMER_PROTECTION
  }

  enum RegulationStatus {
    CURRENT
    AMENDED
    SUPERSEDED
  }

  type LegalRight {
    id: ID!
    name: String!
    holder: RightHolder!
    description: String!
    basis: [String!]!
    limitations: [String!]
    enforcement: String!
  }

  enum RightHolder {
    DONOR
    PATIENT
    FACILITY
    BENEFICIARY
  }

  type Liability {
    id: ID!
    type: LiabilityType!
    description: String!
    coverage: String!
    limit: Float
    insurance: String
    mitigation: [String!]!
  }

  enum LiabilityType {
    STRICT
    NEGLIGENCE
    CONTRACTUAL
    STATUTORY
    PRODUCT
  }

  type LegalPrecedent {
    id: ID!
    case: String!
    court: String!
    date: DateTime!
    jurisdiction: String!
    ruling: String!
    relevance: String!
    citation: String!
  }

  type LegalFramework {
    regulations: [Regulation!]!
    rights: [LegalRight!]!
    obligations: [LegalObligation!]!
    liabilities: [Liability!]!
    precedents: [LegalPrecedent!]!
  }

  type LegalObligation {
    id: ID!
    name: String!
    obligor: String!
    description: String!
    basis: String!
    deadline: String
    penalty: String
  }

  # Contract Types
  type Contract {
    id: ID!
    templateId: String
    type: ContractType!
    parties: [Party!]!
    subject: ContractSubject
    terms: ContractTerms!
    signatures: [Signature!]!
    effectiveDate: DateTime!
    expiryDate: DateTime
    status: ContractStatus!
    history: [ContractEvent!]!
  }

  enum ContractType {
    STORAGE_AGREEMENT
    CONSENT_FORM
    SERVICE_AGREEMENT
    TRANSFER_AGREEMENT
    RESEARCH_AGREEMENT
    DONATION_AGREEMENT
  }

  enum ContractStatus {
    DRAFT
    PENDING
    ACTIVE
    EXPIRED
    TERMINATED
    DISPUTED
  }

  type Party {
    id: ID!
    type: PartyType!
    name: String!
    role: PartyRole!
    representative: String
    contact: ContactInfo!
  }

  enum PartyType {
    INDIVIDUAL
    ORGANIZATION
  }

  enum PartyRole {
    PRIMARY
    COUNTERPARTY
    GUARANTOR
    WITNESS
  }

  type ContractSubject {
    type: SubjectType!
    identifier: String!
    description: String!
  }

  enum SubjectType {
    SPECIMEN
    SERVICE
    FACILITY
    RESEARCH
  }

  type ContractTerms {
    duration: String!
    renewal: RenewalType!
    fees: [Fee!]
    obligations: [PartyObligation!]!
    warranties: [String!]!
    limitations: [String!]!
    governing: String!
    venue: String!
  }

  enum RenewalType {
    AUTOMATIC
    MANUAL
    NONE
  }

  type Fee {
    type: String!
    amount: Float!
    currency: String!
    frequency: FeeFrequency!
    due: String!
  }

  enum FeeFrequency {
    ONE_TIME
    MONTHLY
    ANNUAL
  }

  type PartyObligation {
    party: String!
    obligation: String!
  }

  type Signature {
    party: String!
    signatory: String!
    date: DateTime!
    method: SignatureMethod!
    witness: String
    notarized: Boolean
  }

  enum SignatureMethod {
    WET
    ELECTRONIC
    DIGITAL
  }

  type ContractEvent {
    id: ID!
    type: String!
    date: DateTime!
    actor: String!
    description: String!
    documents: [String!]
  }

  # Dispute Types
  type Dispute {
    id: ID!
    type: DisputeType!
    parties: [String!]!
    subject: String!
    description: String!
    filedDate: DateTime!
    status: DisputeStatus!
    currentMechanism: String
    resolution: Resolution
    history: [DisputeEvent!]!
  }

  enum DisputeType {
    CONTRACT
    CONSENT
    OWNERSHIP
    NEGLIGENCE
    REGULATORY
  }

  enum DisputeStatus {
    FILED
    UNDER_REVIEW
    NEGOTIATION
    MEDIATION
    ARBITRATION
    LITIGATION
    RESOLVED
    DISMISSED
  }

  type Resolution {
    date: DateTime!
    mechanism: String!
    outcome: String!
    terms: [String!]!
    binding: Boolean!
    enforcement: String
  }

  type DisputeEvent {
    date: DateTime!
    type: String!
    description: String!
    actor: String!
    documents: [String!]
  }

  # Compliance Types
  type ComplianceRequirement {
    id: ID!
    regulation: String!
    requirement: String!
    responsible: String!
    evidence: [String!]!
    frequency: String!
    lastCheck: DateTime!
    status: ComplianceStatus!
  }

  enum ComplianceStatus {
    COMPLIANT
    NON_COMPLIANT
    PARTIAL
    PENDING
  }

  type ComplianceScore {
    total: Int!
    compliant: Int!
    nonCompliant: Int!
    partial: Int!
    pending: Int!
    score: Float!
    rating: String!
  }

  # Queries
  type Query {
    # Legal Framework
    regulations(
      category: RegulationCategory
      jurisdiction: String
      status: RegulationStatus
    ): [Regulation!]!
    regulation(id: ID!): Regulation
    rights(holder: RightHolder): [LegalRight!]!
    liabilities(type: LiabilityType): [Liability!]!
    precedents(
      jurisdiction: String
      dateFrom: DateTime
      dateTo: DateTime
    ): [LegalPrecedent!]!

    # Contracts
    contracts(
      type: ContractType
      status: ContractStatus
      limit: Int
      offset: Int
    ): ContractConnection!
    contract(id: ID!): Contract
    contractTemplates(
      type: ContractType
      jurisdiction: String
    ): [ContractTemplate!]!

    # Disputes
    disputes(
      type: DisputeType
      status: DisputeStatus
      limit: Int
      offset: Int
    ): DisputeConnection!
    dispute(id: ID!): Dispute

    # Compliance
    complianceRequirements(
      regulation: String
      status: ComplianceStatus
    ): [ComplianceRequirement!]!
    complianceScore: ComplianceScore!
  }

  type ContractConnection {
    nodes: [Contract!]!
    totalCount: Int!
    pageInfo: PageInfo!
  }

  type DisputeConnection {
    nodes: [Dispute!]!
    totalCount: Int!
    pageInfo: PageInfo!
  }

  type PageInfo {
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
    startCursor: String
    endCursor: String
  }

  type ContractTemplate {
    id: ID!
    name: String!
    type: ContractType!
    version: String!
    jurisdiction: String!
    clauses: [Clause!]!
    status: TemplateStatus!
  }

  type Clause {
    id: ID!
    title: String!
    text: String!
    type: ClauseType!
    mandatory: Boolean!
    category: String!
  }

  enum ClauseType {
    STANDARD
    OPTIONAL
    NEGOTIABLE
  }

  enum TemplateStatus {
    DRAFT
    APPROVED
    ACTIVE
    DEPRECATED
  }

  # Mutations
  type Mutation {
    # Contracts
    createContract(input: CreateContractInput!): Contract!
    updateContract(id: ID!, input: UpdateContractInput!): Contract!
    addSignature(contractId: ID!, input: SignatureInput!): Contract!
    executeContract(id: ID!): Contract!
    terminateContract(id: ID!, reason: String!, effectiveDate: DateTime!): Contract!

    # Disputes
    fileDispute(input: FileDisputeInput!): Dispute!
    updateDisputeStatus(id: ID!, status: DisputeStatus!, reason: String!): Dispute!
    escalateDispute(id: ID!, mechanism: String!, reason: String!): Dispute!
    resolveDispute(id: ID!, input: ResolutionInput!): Dispute!

    # Compliance
    updateComplianceStatus(
      id: ID!
      status: ComplianceStatus!
      evidence: [String!]
    ): ComplianceRequirement!
    reportViolation(input: ViolationInput!): Violation!
  }

  # Input Types
  input CreateContractInput {
    type: ContractType!
    parties: [PartyInput!]!
    subject: ContractSubjectInput
    terms: ContractTermsInput!
    effectiveDate: DateTime!
    expiryDate: DateTime
  }

  input UpdateContractInput {
    terms: ContractTermsInput
    expiryDate: DateTime
  }

  input PartyInput {
    type: PartyType!
    name: String!
    role: PartyRole!
    representative: String
    contact: ContactInfoInput!
  }

  input ContactInfoInput {
    name: String!
    email: String!
    phone: String
  }

  input ContractSubjectInput {
    type: SubjectType!
    identifier: String!
    description: String!
  }

  input ContractTermsInput {
    duration: String!
    renewal: RenewalType!
    fees: [FeeInput!]
    obligations: [PartyObligationInput!]!
    warranties: [String!]!
    limitations: [String!]!
    governing: String!
    venue: String!
  }

  input FeeInput {
    type: String!
    amount: Float!
    currency: String!
    frequency: FeeFrequency!
    due: String!
  }

  input PartyObligationInput {
    party: String!
    obligation: String!
  }

  input SignatureInput {
    party: String!
    signatory: String!
    method: SignatureMethod!
    witness: String
    notarized: Boolean
  }

  input FileDisputeInput {
    type: DisputeType!
    parties: [String!]!
    subject: String!
    description: String!
  }

  input ResolutionInput {
    mechanism: String!
    outcome: String!
    terms: [String!]!
    binding: Boolean!
    enforcement: String
  }

  input ViolationInput {
    regulationId: String!
    description: String!
    severity: String!
    discoveredDate: DateTime!
  }

  type Violation {
    id: ID!
    regulationId: String!
    description: String!
    severity: String!
    status: String!
    reportedDate: DateTime!
  }

  # Subscriptions
  type Subscription {
    contractUpdated(contractId: ID!): Contract!
    disputeStatusChanged(disputeId: ID): Dispute!
    complianceAlert: ComplianceAlert!
  }

  type ComplianceAlert {
    type: String!
    requirementId: String!
    message: String!
    severity: String!
    timestamp: DateTime!
  }
`;

// ============================================================================
// GraphQL Resolvers
// ============================================================================

export const resolvers = {
  DateTime: GraphQLDateTime,

  Query: {
    regulations: async (_: any, args: any, context: any) => {
      return context.services.legalFramework.getRegulations(args);
    },

    regulation: async (_: any, { id }: any, context: any) => {
      return context.services.legalFramework.getRegulationById(id);
    },

    rights: async (_: any, { holder }: any, context: any) => {
      return context.services.legalFramework.getRights(holder);
    },

    liabilities: async (_: any, { type }: any, context: any) => {
      return context.services.legalFramework.getLiabilities(type);
    },

    precedents: async (_: any, args: any, context: any) => {
      return context.services.legalFramework.getPrecedents(args);
    },

    contracts: async (_: any, args: any, context: any) => {
      const result = await context.services.contracts.listContracts(args);
      return {
        nodes: result.contracts,
        totalCount: result.total,
        pageInfo: {
          hasNextPage: result.total > (args.offset || 0) + (args.limit || 20),
          hasPreviousPage: (args.offset || 0) > 0,
        },
      };
    },

    contract: async (_: any, { id }: any, context: any) => {
      return context.services.contracts.getContractById(id);
    },

    contractTemplates: async (_: any, args: any, context: any) => {
      return context.services.contracts.getTemplates(args);
    },

    disputes: async (_: any, args: any, context: any) => {
      const result = await context.services.disputes.listDisputes(args);
      return {
        nodes: result.disputes,
        totalCount: result.total,
        pageInfo: {
          hasNextPage: result.total > (args.offset || 0) + (args.limit || 20),
          hasPreviousPage: (args.offset || 0) > 0,
        },
      };
    },

    dispute: async (_: any, { id }: any, context: any) => {
      return context.services.disputes.getDisputeById(id);
    },

    complianceRequirements: async (_: any, args: any, context: any) => {
      return context.services.compliance.getRequirements(args);
    },

    complianceScore: async (_: any, __: any, context: any) => {
      return context.services.compliance.getComplianceScore();
    },
  },

  Mutation: {
    createContract: async (_: any, { input }: any, context: any) => {
      return context.services.contracts.createContract(input);
    },

    updateContract: async (_: any, { id, input }: any, context: any) => {
      return context.services.contracts.updateContract(id, input);
    },

    addSignature: async (_: any, { contractId, input }: any, context: any) => {
      return context.services.contracts.addSignature(contractId, input);
    },

    executeContract: async (_: any, { id }: any, context: any) => {
      return context.services.contracts.executeContract(id);
    },

    terminateContract: async (_: any, { id, reason, effectiveDate }: any, context: any) => {
      return context.services.contracts.terminateContract(id, reason, effectiveDate);
    },

    fileDispute: async (_: any, { input }: any, context: any) => {
      return context.services.disputes.fileDispute(input);
    },

    updateDisputeStatus: async (_: any, { id, status, reason }: any, context: any) => {
      return context.services.disputes.updateDisputeStatus(id, status, reason);
    },

    escalateDispute: async (_: any, { id, mechanism, reason }: any, context: any) => {
      return context.services.disputes.escalateDispute(id, mechanism, reason);
    },

    resolveDispute: async (_: any, { id, input }: any, context: any) => {
      return context.services.disputes.resolveDispute(id, input);
    },

    updateComplianceStatus: async (_: any, { id, status, evidence }: any, context: any) => {
      return context.services.compliance.updateRequirementStatus(id, status, evidence);
    },

    reportViolation: async (_: any, { input }: any, context: any) => {
      return context.services.compliance.reportViolation(input);
    },
  },

  Subscription: {
    contractUpdated: {
      subscribe: (_: any, { contractId }: any, context: any) => {
        return context.pubsub.asyncIterator([`CONTRACT_UPDATED_${contractId}`]);
      },
    },

    disputeStatusChanged: {
      subscribe: (_: any, { disputeId }: any, context: any) => {
        const topic = disputeId
          ? `DISPUTE_STATUS_${disputeId}`
          : 'DISPUTE_STATUS_ALL';
        return context.pubsub.asyncIterator([topic]);
      },
    },

    complianceAlert: {
      subscribe: (_: any, __: any, context: any) => {
        return context.pubsub.asyncIterator(['COMPLIANCE_ALERT']);
      },
    },
  },
};
```

## WebSocket Real-Time API

```typescript
/**
 * WIA Cryo Legal Standard - WebSocket Real-Time API
 * Real-time event streaming for legal operations
 */

import { WebSocketServer, WebSocket } from 'ws';
import { v4 as uuidv4 } from 'uuid';

// ============================================================================
// WebSocket Server Implementation
// ============================================================================

export interface WSMessage {
  type: string;
  payload: unknown;
  timestamp: string;
  messageId: string;
}

export interface WSClient {
  id: string;
  socket: WebSocket;
  subscriptions: Set<string>;
  authenticated: boolean;
  userId?: string;
}

export class CryoLegalWebSocketServer {
  private wss: WebSocketServer;
  private clients: Map<string, WSClient> = new Map();
  private subscriptionClients: Map<string, Set<string>> = new Map();

  constructor(port: number) {
    this.wss = new WebSocketServer({ port });
    this.initialize();
  }

  private initialize(): void {
    this.wss.on('connection', (socket: WebSocket) => {
      const clientId = uuidv4();

      const client: WSClient = {
        id: clientId,
        socket,
        subscriptions: new Set(),
        authenticated: false,
      };

      this.clients.set(clientId, client);

      // Send connection acknowledgment
      this.sendToClient(client, {
        type: 'connection_ack',
        payload: { clientId },
        timestamp: new Date().toISOString(),
        messageId: uuidv4(),
      });

      // Handle messages
      socket.on('message', (data: Buffer) => {
        this.handleMessage(client, data.toString());
      });

      // Handle close
      socket.on('close', () => {
        this.handleDisconnect(client);
      });

      // Handle error
      socket.on('error', (error) => {
        console.error(`WebSocket error for client ${clientId}:`, error);
      });
    });
  }

  private handleMessage(client: WSClient, data: string): void {
    try {
      const message = JSON.parse(data);

      switch (message.type) {
        case 'authenticate':
          this.handleAuthenticate(client, message.payload);
          break;
        case 'subscribe':
          this.handleSubscribe(client, message.payload);
          break;
        case 'unsubscribe':
          this.handleUnsubscribe(client, message.payload);
          break;
        case 'ping':
          this.handlePing(client);
          break;
        default:
          this.sendError(client, 'UNKNOWN_MESSAGE_TYPE', `Unknown message type: ${message.type}`);
      }
    } catch (error) {
      this.sendError(client, 'PARSE_ERROR', 'Failed to parse message');
    }
  }

  private handleAuthenticate(client: WSClient, payload: { token: string }): void {
    // Validate token
    const validation = this.validateToken(payload.token);

    if (validation.valid) {
      client.authenticated = true;
      client.userId = validation.userId;

      this.sendToClient(client, {
        type: 'authenticated',
        payload: { userId: validation.userId },
        timestamp: new Date().toISOString(),
        messageId: uuidv4(),
      });
    } else {
      this.sendError(client, 'AUTH_FAILED', 'Authentication failed');
    }
  }

  private validateToken(token: string): { valid: boolean; userId?: string } {
    // Token validation logic
    if (token && token.length > 0) {
      return { valid: true, userId: 'user-123' };
    }
    return { valid: false };
  }

  private handleSubscribe(client: WSClient, payload: { topics: string[] }): void {
    if (!client.authenticated) {
      this.sendError(client, 'NOT_AUTHENTICATED', 'Authentication required');
      return;
    }

    for (const topic of payload.topics) {
      // Validate topic access
      if (!this.canAccessTopic(client.userId!, topic)) {
        this.sendError(client, 'ACCESS_DENIED', `Access denied to topic: ${topic}`);
        continue;
      }

      client.subscriptions.add(topic);

      // Track subscription
      if (!this.subscriptionClients.has(topic)) {
        this.subscriptionClients.set(topic, new Set());
      }
      this.subscriptionClients.get(topic)!.add(client.id);
    }

    this.sendToClient(client, {
      type: 'subscribed',
      payload: { topics: Array.from(client.subscriptions) },
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private canAccessTopic(userId: string, topic: string): boolean {
    // Topic access control logic
    return true;
  }

  private handleUnsubscribe(client: WSClient, payload: { topics: string[] }): void {
    for (const topic of payload.topics) {
      client.subscriptions.delete(topic);

      const topicClients = this.subscriptionClients.get(topic);
      if (topicClients) {
        topicClients.delete(client.id);
      }
    }

    this.sendToClient(client, {
      type: 'unsubscribed',
      payload: { topics: payload.topics },
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private handlePing(client: WSClient): void {
    this.sendToClient(client, {
      type: 'pong',
      payload: {},
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private handleDisconnect(client: WSClient): void {
    // Remove from subscriptions
    for (const topic of client.subscriptions) {
      const topicClients = this.subscriptionClients.get(topic);
      if (topicClients) {
        topicClients.delete(client.id);
      }
    }

    this.clients.delete(client.id);
  }

  private sendToClient(client: WSClient, message: WSMessage): void {
    if (client.socket.readyState === WebSocket.OPEN) {
      client.socket.send(JSON.stringify(message));
    }
  }

  private sendError(client: WSClient, code: string, message: string): void {
    this.sendToClient(client, {
      type: 'error',
      payload: { code, message },
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  // ============================================================================
  // Public Methods for Event Publishing
  // ============================================================================

  public publishContractEvent(contractId: string, event: ContractUpdateEvent): void {
    const topic = `contract:${contractId}`;
    this.publishToTopic(topic, {
      type: 'contract_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });

    // Also publish to general contracts topic
    this.publishToTopic('contracts:all', {
      type: 'contract_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  public publishDisputeEvent(disputeId: string, event: DisputeUpdateEvent): void {
    const topic = `dispute:${disputeId}`;
    this.publishToTopic(topic, {
      type: 'dispute_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });

    // Also publish to general disputes topic
    this.publishToTopic('disputes:all', {
      type: 'dispute_update',
      payload: event,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  public publishComplianceAlert(alert: ComplianceAlertEvent): void {
    this.publishToTopic('compliance:alerts', {
      type: 'compliance_alert',
      payload: alert,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  public publishDeadlineReminder(reminder: DeadlineReminderEvent): void {
    this.publishToTopic('deadlines', {
      type: 'deadline_reminder',
      payload: reminder,
      timestamp: new Date().toISOString(),
      messageId: uuidv4(),
    });
  }

  private publishToTopic(topic: string, message: WSMessage): void {
    const clientIds = this.subscriptionClients.get(topic);

    if (clientIds) {
      for (const clientId of clientIds) {
        const client = this.clients.get(clientId);
        if (client) {
          this.sendToClient(client, message);
        }
      }
    }
  }

  public getConnectionStats(): ConnectionStats {
    const topicStats: Record<string, number> = {};

    for (const [topic, clients] of this.subscriptionClients) {
      topicStats[topic] = clients.size;
    }

    return {
      totalConnections: this.clients.size,
      authenticatedConnections: Array.from(this.clients.values())
        .filter(c => c.authenticated).length,
      topicSubscriptions: topicStats,
    };
  }
}

// ============================================================================
// Event Types
// ============================================================================

export interface ContractUpdateEvent {
  contractId: string;
  action: 'created' | 'updated' | 'signed' | 'executed' | 'terminated';
  previousStatus?: string;
  newStatus: string;
  actor: string;
  details?: Record<string, unknown>;
}

export interface DisputeUpdateEvent {
  disputeId: string;
  action: 'filed' | 'status_changed' | 'escalated' | 'resolved' | 'event_added';
  previousStatus?: string;
  newStatus?: string;
  mechanism?: string;
  actor: string;
  details?: Record<string, unknown>;
}

export interface ComplianceAlertEvent {
  alertId: string;
  type: 'requirement_due' | 'status_change' | 'violation' | 'audit_scheduled';
  severity: 'low' | 'medium' | 'high' | 'critical';
  requirementId?: string;
  message: string;
  dueDate?: string;
  actions?: string[];
}

export interface DeadlineReminderEvent {
  deadlineId: string;
  type: 'contract_expiry' | 'compliance_check' | 'audit' | 'report_due' | 'training';
  daysRemaining: number;
  itemId: string;
  itemName: string;
  dueDate: string;
  assignee?: string;
}

export interface ConnectionStats {
  totalConnections: number;
  authenticatedConnections: number;
  topicSubscriptions: Record<string, number>;
}

// ============================================================================
// WebSocket Client SDK
// ============================================================================

export class CryoLegalWSClient {
  private socket: WebSocket | null = null;
  private clientId: string | null = null;
  private authenticated = false;
  private subscriptions = new Set<string>();
  private messageHandlers = new Map<string, Set<(payload: unknown) => void>>();
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;

  constructor(private readonly url: string) {}

  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.socket = new WebSocket(this.url);

      this.socket.onopen = () => {
        this.reconnectAttempts = 0;
        resolve();
      };

      this.socket.onerror = (error) => {
        reject(error);
      };

      this.socket.onmessage = (event) => {
        this.handleMessage(JSON.parse(event.data.toString()));
      };

      this.socket.onclose = () => {
        this.handleDisconnect();
      };
    });
  }

  async authenticate(token: string): Promise<void> {
    return new Promise((resolve, reject) => {
      const handler = (payload: unknown) => {
        this.authenticated = true;
        this.off('authenticated', handler);
        resolve();
      };

      const errorHandler = (payload: unknown) => {
        this.off('error', errorHandler);
        reject(new Error((payload as any).message));
      };

      this.on('authenticated', handler);
      this.on('error', errorHandler);

      this.send({
        type: 'authenticate',
        payload: { token },
      });
    });
  }

  async subscribe(topics: string[]): Promise<void> {
    return new Promise((resolve) => {
      const handler = (payload: unknown) => {
        for (const topic of (payload as any).topics) {
          this.subscriptions.add(topic);
        }
        this.off('subscribed', handler);
        resolve();
      };

      this.on('subscribed', handler);

      this.send({
        type: 'subscribe',
        payload: { topics },
      });
    });
  }

  async unsubscribe(topics: string[]): Promise<void> {
    return new Promise((resolve) => {
      const handler = () => {
        for (const topic of topics) {
          this.subscriptions.delete(topic);
        }
        this.off('unsubscribed', handler);
        resolve();
      };

      this.on('unsubscribed', handler);

      this.send({
        type: 'unsubscribe',
        payload: { topics },
      });
    });
  }

  on(eventType: string, handler: (payload: unknown) => void): void {
    if (!this.messageHandlers.has(eventType)) {
      this.messageHandlers.set(eventType, new Set());
    }
    this.messageHandlers.get(eventType)!.add(handler);
  }

  off(eventType: string, handler: (payload: unknown) => void): void {
    this.messageHandlers.get(eventType)?.delete(handler);
  }

  private handleMessage(message: WSMessage): void {
    // Handle connection acknowledgment
    if (message.type === 'connection_ack') {
      this.clientId = (message.payload as any).clientId;
    }

    // Dispatch to handlers
    const handlers = this.messageHandlers.get(message.type);
    if (handlers) {
      for (const handler of handlers) {
        handler(message.payload);
      }
    }
  }

  private handleDisconnect(): void {
    this.authenticated = false;

    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.pow(2, this.reconnectAttempts) * 1000;

      setTimeout(() => {
        this.connect().catch(console.error);
      }, delay);
    }
  }

  private send(message: { type: string; payload: unknown }): void {
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(message));
    }
  }

  disconnect(): void {
    this.socket?.close();
    this.socket = null;
    this.authenticated = false;
    this.subscriptions.clear();
  }
}
```

---

## Chapter Summary

This chapter provided complete API implementations:

- **REST API**: Express-based endpoints with middleware
- **GraphQL API**: Apollo Server with full schema and resolvers
- **WebSocket API**: Real-time event streaming with subscription management

---

**Next Chapter**: [Contract Management - Templates, Clauses, and Lifecycle](./05-contract-management.md)
