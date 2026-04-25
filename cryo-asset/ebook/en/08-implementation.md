# Chapter 8: Implementation

## Development Guidelines and Deployment Strategies

### Introduction

Implementing a cryonics asset management system requires careful attention to long-term sustainability, data integrity, and operational resilience. This chapter provides comprehensive guidance on developing, deploying, and maintaining systems built on the WIA Cryo-Asset Standard, with particular emphasis on ensuring systems can operate reliably across the extended time horizons required for cryonics applications.

---

## 8.1 Development Environment Setup

### Project Initialization

```typescript
// Project structure for Cryo-Asset implementation
/*
cryo-asset-platform/
├── apps/
│   ├── api/                    # REST API service
│   ├── web/                    # Web application
│   ├── admin/                  # Admin dashboard
│   └── mobile/                 # Mobile application
├── packages/
│   ├── core/                   # Core business logic
│   ├── database/               # Database schemas and migrations
│   ├── crypto/                 # Cryptographic utilities
│   ├── integrations/           # External service integrations
│   ├── blockchain/             # Blockchain interaction
│   └── shared/                 # Shared types and utilities
├── infrastructure/
│   ├── terraform/              # Infrastructure as Code
│   ├── kubernetes/             # Kubernetes manifests
│   ├── docker/                 # Docker configurations
│   └── scripts/                # Deployment scripts
├── docs/
│   ├── api/                    # API documentation
│   ├── architecture/           # Architecture diagrams
│   └── operations/             # Operations runbooks
└── tests/
    ├── unit/
    ├── integration/
    └── e2e/
*/

// package.json for monorepo root
const packageJson = {
  "name": "cryo-asset-platform",
  "version": "1.0.0",
  "private": true,
  "workspaces": [
    "apps/*",
    "packages/*"
  ],
  "scripts": {
    "dev": "turbo run dev",
    "build": "turbo run build",
    "test": "turbo run test",
    "lint": "turbo run lint",
    "format": "prettier --write \"**/*.{ts,tsx,md}\"",
    "db:migrate": "pnpm --filter database migrate",
    "db:seed": "pnpm --filter database seed",
    "deploy:staging": "./infrastructure/scripts/deploy.sh staging",
    "deploy:production": "./infrastructure/scripts/deploy.sh production"
  },
  "devDependencies": {
    "@types/node": "^20.0.0",
    "prettier": "^3.0.0",
    "turbo": "^1.10.0",
    "typescript": "^5.2.0",
    "vitest": "^0.34.0"
  },
  "engines": {
    "node": ">=20.0.0",
    "pnpm": ">=8.0.0"
  }
};

// TypeScript configuration for core package
const tsconfigCore = {
  "compilerOptions": {
    "target": "ES2022",
    "module": "NodeNext",
    "moduleResolution": "NodeNext",
    "lib": ["ES2022"],
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "declaration": true,
    "declarationMap": true,
    "sourceMap": true,
    "outDir": "./dist",
    "rootDir": "./src"
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
};
```

### Core Package Implementation

```typescript
// packages/core/src/index.ts
export * from './entities';
export * from './services';
export * from './repositories';
export * from './events';
export * from './errors';

// packages/core/src/entities/Patient.ts
import { z } from 'zod';
import { Entity, Column, PrimaryColumn, CreateDateColumn, UpdateDateColumn } from 'typeorm';

export const PatientSchema = z.object({
  id: z.string().uuid(),
  organizationId: z.string().uuid(),
  organizationPatientId: z.string().min(1).max(50),

  legalIdentity: z.object({
    fullLegalName: z.string().min(2).max(200),
    previousNames: z.array(z.string()).default([]),
    dateOfBirth: z.date(),
    dateOfLegalDeath: z.date().nullable(),
    placeOfBirth: z.string().max(200),
    citizenship: z.array(z.string().length(2)).min(1),
  }),

  preservationStatus: z.object({
    status: z.enum([
      'MEMBER_ACTIVE',
      'MEMBER_STANDBY',
      'MEMBER_SUSPENDED',
      'IN_TRANSPORT',
      'PRESERVED',
      'REVIVED',
      'TERMINATED',
    ]),
    preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
    preservationDate: z.date().nullable(),
    preservationLocation: z.string().nullable(),
  }),

  createdAt: z.date(),
  updatedAt: z.date(),
});

export type PatientInput = z.input<typeof PatientSchema>;
export type Patient = z.output<typeof PatientSchema>;

@Entity('patients')
export class PatientEntity {
  @PrimaryColumn('uuid')
  id: string;

  @Column('uuid')
  organizationId: string;

  @Column({ length: 50 })
  organizationPatientId: string;

  @Column('jsonb')
  legalIdentity: Patient['legalIdentity'];

  @Column('jsonb')
  preservationStatus: Patient['preservationStatus'];

  @CreateDateColumn()
  createdAt: Date;

  @UpdateDateColumn()
  updatedAt: Date;
}

// packages/core/src/services/PatientService.ts
import { EventEmitter } from 'events';
import { PatientRepository } from '../repositories/PatientRepository';
import { Patient, PatientSchema } from '../entities/Patient';
import { CryoAssetError } from '../errors';

export class PatientService extends EventEmitter {
  constructor(
    private readonly patientRepository: PatientRepository,
    private readonly portfolioService: PortfolioService,
  ) {
    super();
  }

  async createPatient(input: PatientInput): Promise<Patient> {
    // Validate input
    const validated = PatientSchema.parse({
      ...input,
      id: input.id || crypto.randomUUID(),
      createdAt: new Date(),
      updatedAt: new Date(),
    });

    // Check for duplicate
    const existing = await this.patientRepository.findByOrganizationPatientId(
      validated.organizationId,
      validated.organizationPatientId
    );

    if (existing) {
      throw new CryoAssetError(
        'DUPLICATE_PATIENT',
        `Patient with ID ${validated.organizationPatientId} already exists`
      );
    }

    // Create patient
    const patient = await this.patientRepository.create(validated);

    // Create portfolio
    await this.portfolioService.createPortfolio({
      patientId: patient.id,
      organizationId: patient.organizationId,
    });

    // Emit event
    this.emit('patient:created', { patient });

    return patient;
  }

  async updatePreservationStatus(
    patientId: string,
    status: Patient['preservationStatus']
  ): Promise<Patient> {
    const patient = await this.patientRepository.findById(patientId);

    if (!patient) {
      throw new CryoAssetError('PATIENT_NOT_FOUND', `Patient ${patientId} not found`);
    }

    // Validate status transition
    this.validateStatusTransition(patient.preservationStatus.status, status.status);

    // Update status
    const updated = await this.patientRepository.update(patientId, {
      preservationStatus: status,
      updatedAt: new Date(),
    });

    // Emit event
    this.emit('patient:status_changed', {
      patient: updated,
      previousStatus: patient.preservationStatus.status,
      newStatus: status.status,
    });

    return updated;
  }

  private validateStatusTransition(from: string, to: string): void {
    const validTransitions: Record<string, string[]> = {
      'MEMBER_ACTIVE': ['MEMBER_STANDBY', 'MEMBER_SUSPENDED', 'IN_TRANSPORT'],
      'MEMBER_STANDBY': ['MEMBER_ACTIVE', 'MEMBER_SUSPENDED', 'IN_TRANSPORT'],
      'MEMBER_SUSPENDED': ['MEMBER_ACTIVE'],
      'IN_TRANSPORT': ['PRESERVED'],
      'PRESERVED': ['REVIVED', 'TERMINATED'],
      'REVIVED': [],
      'TERMINATED': [],
    };

    if (!validTransitions[from]?.includes(to)) {
      throw new CryoAssetError(
        'INVALID_STATUS_TRANSITION',
        `Cannot transition from ${from} to ${to}`
      );
    }
  }
}

// packages/core/src/services/AssetService.ts
export class AssetService extends EventEmitter {
  constructor(
    private readonly assetRepository: AssetRepository,
    private readonly valuationService: ValuationService,
    private readonly blockchainService: BlockchainService,
    private readonly cryptoService: CryptoService,
  ) {
    super();
  }

  async registerAsset(input: AssetInput): Promise<Asset> {
    // Validate input
    const validated = AssetSchema.parse({
      ...input,
      id: input.id || crypto.randomUUID(),
      status: 'ACTIVE',
      createdAt: new Date(),
      updatedAt: new Date(),
    });

    // Encrypt sensitive data
    if (validated.sensitiveData) {
      validated.sensitiveData = await this.cryptoService.encrypt(
        JSON.stringify(validated.sensitiveData),
        validated.portfolioId
      );
    }

    // Create asset
    const asset = await this.assetRepository.create(validated);

    // Register on blockchain
    const blockchainRef = await this.blockchainService.registerAsset(asset);
    await this.assetRepository.updateBlockchainRef(asset.id, blockchainRef);

    // Get initial valuation
    const valuation = await this.valuationService.getValuation(asset);
    await this.valuationService.recordValuation(asset.id, valuation);

    // Emit event
    this.emit('asset:registered', { asset, blockchainRef, valuation });

    return asset;
  }

  async transferAsset(
    assetId: string,
    transfer: AssetTransferInput
  ): Promise<AssetTransfer> {
    const asset = await this.assetRepository.findById(assetId);

    if (!asset) {
      throw new CryoAssetError('ASSET_NOT_FOUND', `Asset ${assetId} not found`);
    }

    // Validate transfer
    await this.validateTransfer(asset, transfer);

    // Create transfer record
    const transferRecord = await this.assetRepository.createTransfer({
      id: crypto.randomUUID(),
      assetId,
      ...transfer,
      status: 'PENDING',
      initiatedAt: new Date(),
    });

    // Update asset ownership
    const updatedAsset = await this.assetRepository.update(assetId, {
      ownership: transfer.newOwnership,
      portfolioId: transfer.targetPortfolioId || asset.portfolioId,
      updatedAt: new Date(),
    });

    // Record on blockchain
    await this.blockchainService.recordTransfer(transferRecord);

    // Update transfer status
    await this.assetRepository.updateTransferStatus(transferRecord.id, 'COMPLETED');

    // Emit event
    this.emit('asset:transferred', { asset: updatedAsset, transfer: transferRecord });

    return { ...transferRecord, status: 'COMPLETED' };
  }
}
```

---

## 8.2 Database Implementation

### PostgreSQL Schema

```typescript
// packages/database/src/migrations/001_initial_schema.ts
import { Knex } from 'knex';

export async function up(knex: Knex): Promise<void> {
  // Enable extensions
  await knex.raw('CREATE EXTENSION IF NOT EXISTS "uuid-ossp"');
  await knex.raw('CREATE EXTENSION IF NOT EXISTS "pgcrypto"');

  // Organizations table
  await knex.schema.createTable('organizations', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.string('name', 200).notNullable();
    table.string('type', 50).notNullable();
    table.jsonb('settings').defaultTo('{}');
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());
  });

  // Patients table
  await knex.schema.createTable('patients', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('organization_id').notNullable().references('id').inTable('organizations');
    table.string('organization_patient_id', 50).notNullable();
    table.jsonb('legal_identity').notNullable();
    table.jsonb('preservation_status').notNullable();
    table.jsonb('relationships').defaultTo('{}');
    table.jsonb('documents').defaultTo('[]');
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());

    table.unique(['organization_id', 'organization_patient_id']);
    table.index('organization_id');
    table.index(['preservation_status->status'], 'idx_patient_status');
  });

  // Portfolios table
  await knex.schema.createTable('portfolios', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('patient_id').notNullable().references('id').inTable('patients');
    table.uuid('organization_id').notNullable().references('id').inTable('organizations');
    table.string('status', 50).notNullable().defaultTo('ACTIVE');
    table.jsonb('preservation_fund').defaultTo('{}');
    table.jsonb('revival_fund').defaultTo('{}');
    table.jsonb('governance').defaultTo('{}');
    table.decimal('total_value', 20, 2).defaultTo(0);
    table.timestamp('last_valuation_date').nullable();
    table.timestamp('next_review_date').nullable();
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());

    table.index('patient_id');
    table.index('organization_id');
  });

  // Assets table
  await knex.schema.createTable('assets', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('portfolio_id').notNullable().references('id').inTable('portfolios');
    table.uuid('patient_id').notNullable().references('id').inTable('patients');
    table.string('category', 50).notNullable();
    table.string('type', 100).notNullable();
    table.string('subtype', 100).nullable();
    table.string('name', 500).notNullable();
    table.text('description').nullable();
    table.jsonb('ownership').notNullable();
    table.jsonb('acquisition_info').notNullable();
    table.jsonb('location').nullable();
    table.jsonb('custody').nullable();
    table.decimal('current_value', 20, 2).notNullable();
    table.string('currency', 3).notNullable().defaultTo('USD');
    table.string('status', 50).notNullable().defaultTo('ACTIVE');
    table.jsonb('blockchain_ref').nullable();
    table.string('encryption_key_id').nullable();
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());

    table.index('portfolio_id');
    table.index('patient_id');
    table.index('category');
    table.index('status');
  });

  // Valuations table
  await knex.schema.createTable('valuations', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('asset_id').notNullable().references('id').inTable('assets');
    table.timestamp('valuation_date').notNullable();
    table.timestamp('effective_date').notNullable();
    table.decimal('value', 20, 2).notNullable();
    table.string('currency', 3).notNullable();
    table.string('valuation_method', 50).notNullable();
    table.string('valuation_basis', 50).notNullable();
    table.jsonb('valuation_source').notNullable();
    table.jsonb('market_data').nullable();
    table.jsonb('appraisal_data').nullable();
    table.string('confidence_level', 20).notNullable();
    table.string('performed_by').notNullable();
    table.string('reviewed_by').nullable();
    table.string('blockchain_hash').nullable();
    table.timestamp('created_at').defaultTo(knex.fn.now());

    table.index('asset_id');
    table.index('valuation_date');
  });

  // Trusts table
  await knex.schema.createTable('trusts', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.string('external_id', 100).nullable();
    table.string('name', 500).notNullable();
    table.string('trust_type', 50).notNullable();
    table.string('tax_classification', 50).nullable();
    table.jsonb('jurisdiction').notNullable();
    table.jsonb('parties').notNullable();
    table.jsonb('terms').notNullable();
    table.jsonb('assets').defaultTo('{}');
    table.jsonb('governance').defaultTo('{}');
    table.jsonb('documentation').defaultTo('{}');
    table.string('status', 50).notNullable().defaultTo('DRAFT');
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());

    table.index('trust_type');
    table.index('status');
  });

  // Transactions table
  await knex.schema.createTable('transactions', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('portfolio_id').notNullable().references('id').inTable('portfolios');
    table.uuid('asset_id').nullable().references('id').inTable('assets');
    table.uuid('trust_id').nullable().references('id').inTable('trusts');
    table.string('type', 50).notNullable();
    table.string('subtype', 50).nullable();
    table.timestamp('transaction_date').notNullable();
    table.timestamp('settlement_date').nullable();
    table.decimal('amount', 20, 2).notNullable();
    table.string('currency', 3).notNullable();
    table.jsonb('fees').defaultTo('[]');
    table.jsonb('taxes').defaultTo('[]');
    table.decimal('net_amount', 20, 2).notNullable();
    table.jsonb('security').nullable();
    table.decimal('quantity', 20, 8).nullable();
    table.decimal('price', 20, 8).nullable();
    table.jsonb('counterparty').nullable();
    table.string('external_reference').nullable();
    table.string('status', 50).notNullable().defaultTo('PENDING');
    table.jsonb('blockchain_ref').nullable();
    table.timestamp('created_at').defaultTo(knex.fn.now());

    table.index('portfolio_id');
    table.index('asset_id');
    table.index('transaction_date');
    table.index('type');
    table.index('status');
  });

  // Events table
  await knex.schema.createTable('events', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.string('event_type', 100).notNullable();
    table.string('category', 50).notNullable();
    table.timestamp('occurred_at').notNullable();
    table.timestamp('recorded_at').defaultTo(knex.fn.now());
    table.jsonb('subject').notNullable();
    table.text('summary').notNullable();
    table.jsonb('details').defaultTo('{}');
    table.jsonb('impact').nullable();
    table.jsonb('related_entities').defaultTo('[]');
    table.jsonb('source').notNullable();
    table.string('blockchain_ref').nullable();

    table.index('event_type');
    table.index('category');
    table.index('occurred_at');
    table.index(['subject->type', 'subject->id'], 'idx_event_subject');
  });

  // Audit log table
  await knex.schema.createTable('audit_log', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.timestamp('timestamp').notNullable().defaultTo(knex.fn.now());
    table.string('event_type', 100).notNullable();
    table.string('event_category', 50).notNullable();
    table.string('severity', 20).notNullable().defaultTo('INFO');
    table.jsonb('actor').notNullable();
    table.jsonb('target').notNullable();
    table.jsonb('changes').nullable();
    table.jsonb('context').defaultTo('{}');
    table.string('hash', 64).notNullable();
    table.string('blockchain_ref').nullable();

    table.index('timestamp');
    table.index('event_type');
    table.index(['actor->userId'], 'idx_audit_actor');
    table.index(['target->entityId'], 'idx_audit_target');
  });

  // Encryption keys table
  await knex.schema.createTable('encryption_keys', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('asset_id').nullable().references('id').inTable('assets');
    table.string('algorithm', 50).notNullable();
    table.binary('key_material').notNullable();  // Encrypted with master key
    table.integer('version').notNullable().defaultTo(1);
    table.string('status', 20).notNullable().defaultTo('ACTIVE');
    table.jsonb('rotation_policy').defaultTo('{}');
    table.jsonb('previous_versions').defaultTo('[]');
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('rotated_at').nullable();
    table.timestamp('expires_at').nullable();

    table.index('asset_id');
    table.index('status');
  });
}

export async function down(knex: Knex): Promise<void> {
  await knex.schema.dropTableIfExists('encryption_keys');
  await knex.schema.dropTableIfExists('audit_log');
  await knex.schema.dropTableIfExists('events');
  await knex.schema.dropTableIfExists('transactions');
  await knex.schema.dropTableIfExists('trusts');
  await knex.schema.dropTableIfExists('valuations');
  await knex.schema.dropTableIfExists('assets');
  await knex.schema.dropTableIfExists('portfolios');
  await knex.schema.dropTableIfExists('patients');
  await knex.schema.dropTableIfExists('organizations');
}
```

---

## 8.3 API Implementation

### Express Application Setup

```typescript
// apps/api/src/app.ts
import express, { Express, Request, Response, NextFunction } from 'express';
import helmet from 'helmet';
import cors from 'cors';
import compression from 'compression';
import { rateLimit } from 'express-rate-limit';
import { pinoHttp } from 'pino-http';
import { ZodError } from 'zod';

import { patientRouter } from './routes/patients';
import { portfolioRouter } from './routes/portfolios';
import { assetRouter } from './routes/assets';
import { trustRouter } from './routes/trusts';
import { integrationRouter } from './routes/integrations';
import { webhookRouter } from './routes/webhooks';
import { authMiddleware } from './middleware/auth';
import { ApiError } from './errors';

export function createApp(dependencies: AppDependencies): Express {
  const app = express();

  // Security middleware
  app.use(helmet());
  app.use(cors({
    origin: process.env.ALLOWED_ORIGINS?.split(',') || [],
    credentials: true,
  }));

  // Compression
  app.use(compression());

  // Body parsing
  app.use(express.json({ limit: '10mb' }));
  app.use(express.urlencoded({ extended: true }));

  // Logging
  app.use(pinoHttp({
    logger: dependencies.logger,
    customProps: (req) => ({
      requestId: req.headers['x-request-id'],
    }),
  }));

  // Rate limiting
  app.use(rateLimit({
    windowMs: 60 * 1000,  // 1 minute
    max: 100,  // 100 requests per minute
    standardHeaders: true,
    legacyHeaders: false,
    keyGenerator: (req) => req.headers['x-api-key'] as string || req.ip,
  }));

  // Request ID
  app.use((req, res, next) => {
    req.requestId = req.headers['x-request-id'] as string || crypto.randomUUID();
    res.setHeader('X-Request-ID', req.requestId);
    next();
  });

  // Health check (no auth)
  app.get('/health', (req, res) => {
    res.json({ status: 'healthy', timestamp: new Date().toISOString() });
  });

  // API routes
  app.use('/api/v1', authMiddleware(dependencies.authService));
  app.use('/api/v1/patients', patientRouter(dependencies));
  app.use('/api/v1/portfolios', portfolioRouter(dependencies));
  app.use('/api/v1/assets', assetRouter(dependencies));
  app.use('/api/v1/trusts', trustRouter(dependencies));
  app.use('/api/v1/integrations', integrationRouter(dependencies));
  app.use('/api/v1/webhooks', webhookRouter(dependencies));

  // Error handling
  app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
    const requestId = req.requestId || 'unknown';

    if (err instanceof ZodError) {
      return res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Validation failed',
          validationErrors: err.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
          })),
        },
        requestId,
        timestamp: new Date().toISOString(),
      });
    }

    if (err instanceof ApiError) {
      return res.status(err.statusCode).json({
        success: false,
        error: {
          code: err.code,
          message: err.message,
          details: err.details,
        },
        requestId,
        timestamp: new Date().toISOString(),
      });
    }

    // Log unexpected errors
    dependencies.logger.error({ err, requestId }, 'Unexpected error');

    return res.status(500).json({
      success: false,
      error: {
        code: 'INTERNAL_ERROR',
        message: 'An unexpected error occurred',
      },
      requestId,
      timestamp: new Date().toISOString(),
    });
  });

  return app;
}

// apps/api/src/routes/assets.ts
import { Router } from 'express';
import { z } from 'zod';
import { AppDependencies } from '../types';

export function assetRouter(deps: AppDependencies): Router {
  const router = Router();
  const { assetService, authService } = deps;

  // Create asset
  router.post('/', async (req, res, next) => {
    try {
      const input = CreateAssetSchema.parse(req.body);

      // Check authorization
      await authService.authorize({
        userId: req.user.id,
        action: 'CREATE',
        resource: 'ASSET',
        context: { portfolioId: input.portfolioId },
      });

      const asset = await assetService.registerAsset({
        ...input,
        registeredBy: req.user.id,
      });

      res.status(201).json({
        success: true,
        data: formatAsset(asset),
      });
    } catch (error) {
      next(error);
    }
  });

  // Get asset
  router.get('/:id', async (req, res, next) => {
    try {
      const asset = await assetService.getAsset(req.params.id);

      if (!asset) {
        throw new ApiError('ASSET_NOT_FOUND', 'Asset not found', 404);
      }

      await authService.authorize({
        userId: req.user.id,
        action: 'READ',
        resource: 'ASSET',
        resourceId: asset.id,
      });

      const valuations = await assetService.getValuationHistory(asset.id, { limit: 12 });
      const transactions = await assetService.getTransactions(asset.id, { limit: 20 });

      res.json({
        success: true,
        data: {
          ...formatAsset(asset),
          valuationHistory: valuations.map(formatValuation),
          transactions: transactions.map(formatTransaction),
        },
      });
    } catch (error) {
      next(error);
    }
  });

  // Update valuation
  router.put('/:id/valuation', async (req, res, next) => {
    try {
      const input = UpdateValuationSchema.parse(req.body);

      await authService.authorize({
        userId: req.user.id,
        action: 'UPDATE_VALUATION',
        resource: 'ASSET',
        resourceId: req.params.id,
      });

      const valuation = await assetService.recordValuation(req.params.id, {
        ...input,
        performedBy: req.user.id,
      });

      res.json({
        success: true,
        data: formatValuation(valuation),
      });
    } catch (error) {
      next(error);
    }
  });

  // Transfer asset
  router.post('/:id/transfer', async (req, res, next) => {
    try {
      const input = TransferAssetSchema.parse(req.body);

      await authService.authorize({
        userId: req.user.id,
        action: 'TRANSFER',
        resource: 'ASSET',
        resourceId: req.params.id,
      });

      const transfer = await assetService.transferAsset(req.params.id, {
        ...input,
        initiatedBy: req.user.id,
      });

      res.json({
        success: true,
        data: formatTransfer(transfer),
      });
    } catch (error) {
      next(error);
    }
  });

  return router;
}
```

---

## 8.4 Kubernetes Deployment

### Infrastructure Configuration

```yaml
# infrastructure/kubernetes/base/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-asset-api
  labels:
    app: cryo-asset-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-asset-api
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxUnavailable: 1
      maxSurge: 1
  template:
    metadata:
      labels:
        app: cryo-asset-api
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "3000"
        prometheus.io/path: "/metrics"
    spec:
      serviceAccountName: cryo-asset-api
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000
      containers:
        - name: api
          image: cryo-asset-api:latest
          imagePullPolicy: Always
          ports:
            - name: http
              containerPort: 3000
              protocol: TCP
          env:
            - name: NODE_ENV
              value: "production"
            - name: PORT
              value: "3000"
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: cryo-asset-secrets
                  key: database-url
            - name: REDIS_URL
              valueFrom:
                secretKeyRef:
                  name: cryo-asset-secrets
                  key: redis-url
            - name: JWT_SECRET
              valueFrom:
                secretKeyRef:
                  name: cryo-asset-secrets
                  key: jwt-secret
            - name: MASTER_KEY
              valueFrom:
                secretKeyRef:
                  name: cryo-asset-secrets
                  key: master-key
          resources:
            requests:
              memory: "256Mi"
              cpu: "100m"
            limits:
              memory: "512Mi"
              cpu: "500m"
          readinessProbe:
            httpGet:
              path: /health
              port: 3000
            initialDelaySeconds: 10
            periodSeconds: 5
          livenessProbe:
            httpGet:
              path: /health
              port: 3000
            initialDelaySeconds: 30
            periodSeconds: 10
          securityContext:
            allowPrivilegeEscalation: false
            readOnlyRootFilesystem: true
            capabilities:
              drop:
                - ALL
          volumeMounts:
            - name: tmp
              mountPath: /tmp
      volumes:
        - name: tmp
          emptyDir: {}
      affinity:
        podAntiAffinity:
          preferredDuringSchedulingIgnoredDuringExecution:
            - weight: 100
              podAffinityTerm:
                labelSelector:
                  matchLabels:
                    app: cryo-asset-api
                topologyKey: kubernetes.io/hostname
---
# infrastructure/kubernetes/base/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: cryo-asset-api
  labels:
    app: cryo-asset-api
spec:
  type: ClusterIP
  ports:
    - port: 80
      targetPort: 3000
      protocol: TCP
      name: http
  selector:
    app: cryo-asset-api
---
# infrastructure/kubernetes/base/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: cryo-asset-api
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: cryo-asset-api
  minReplicas: 3
  maxReplicas: 10
  metrics:
    - type: Resource
      resource:
        name: cpu
        target:
          type: Utilization
          averageUtilization: 70
    - type: Resource
      resource:
        name: memory
        target:
          type: Utilization
          averageUtilization: 80
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
        - type: Percent
          value: 10
          periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 0
      policies:
        - type: Percent
          value: 100
          periodSeconds: 15
        - type: Pods
          value: 4
          periodSeconds: 15
      selectPolicy: Max
```

### Database Deployment

```yaml
# infrastructure/kubernetes/database/postgresql.yaml
apiVersion: postgresql.cnpg.io/v1
kind: Cluster
metadata:
  name: cryo-asset-db
spec:
  instances: 3
  primaryUpdateStrategy: unsupervised

  postgresql:
    parameters:
      max_connections: "200"
      shared_buffers: "256MB"
      effective_cache_size: "768MB"
      maintenance_work_mem: "64MB"
      checkpoint_completion_target: "0.9"
      wal_buffers: "16MB"
      default_statistics_target: "100"
      random_page_cost: "1.1"
      effective_io_concurrency: "200"
      min_wal_size: "1GB"
      max_wal_size: "4GB"

  storage:
    size: 100Gi
    storageClass: premium-ssd

  backup:
    barmanObjectStore:
      destinationPath: "s3://cryo-asset-backups/postgresql"
      s3Credentials:
        accessKeyId:
          name: s3-credentials
          key: ACCESS_KEY_ID
        secretAccessKey:
          name: s3-credentials
          key: SECRET_ACCESS_KEY
      wal:
        compression: gzip
      data:
        compression: gzip
    retentionPolicy: "30d"

  monitoring:
    enablePodMonitor: true

  resources:
    requests:
      memory: "2Gi"
      cpu: "1"
    limits:
      memory: "4Gi"
      cpu: "2"
```

---

## 8.5 Testing Framework

### Comprehensive Testing

```typescript
// tests/unit/services/AssetService.test.ts
import { describe, it, expect, beforeEach, vi } from 'vitest';
import { AssetService } from '@cryo-asset/core';
import { createMockAssetRepository, createMockBlockchainService } from '../mocks';

describe('AssetService', () => {
  let assetService: AssetService;
  let mockAssetRepository: ReturnType<typeof createMockAssetRepository>;
  let mockBlockchainService: ReturnType<typeof createMockBlockchainService>;

  beforeEach(() => {
    mockAssetRepository = createMockAssetRepository();
    mockBlockchainService = createMockBlockchainService();

    assetService = new AssetService(
      mockAssetRepository,
      createMockValuationService(),
      mockBlockchainService,
      createMockCryptoService(),
    );
  });

  describe('registerAsset', () => {
    it('should create asset and register on blockchain', async () => {
      const input = {
        portfolioId: 'portfolio-123',
        category: 'FINANCIAL',
        type: 'BANK_ACCOUNT',
        name: 'Savings Account',
        ownership: {
          type: 'SOLE',
          holders: [{ entityId: 'patient-123', percentage: 100 }],
        },
        acquisitionInfo: {
          date: new Date('2024-01-01'),
          cost: 50000,
          currency: 'USD',
        },
        currentValue: 52000,
      };

      mockAssetRepository.create.mockResolvedValue({
        id: 'asset-123',
        ...input,
        status: 'ACTIVE',
        createdAt: new Date(),
        updatedAt: new Date(),
      });

      mockBlockchainService.registerAsset.mockResolvedValue({
        transactionHash: '0x123...',
        blockNumber: 12345,
      });

      const result = await assetService.registerAsset(input);

      expect(result.id).toBe('asset-123');
      expect(mockAssetRepository.create).toHaveBeenCalledOnce();
      expect(mockBlockchainService.registerAsset).toHaveBeenCalledOnce();
    });

    it('should emit asset:registered event', async () => {
      const eventHandler = vi.fn();
      assetService.on('asset:registered', eventHandler);

      const input = createValidAssetInput();
      await assetService.registerAsset(input);

      expect(eventHandler).toHaveBeenCalledOnce();
      expect(eventHandler).toHaveBeenCalledWith(
        expect.objectContaining({
          asset: expect.any(Object),
          blockchainRef: expect.any(Object),
        })
      );
    });
  });

  describe('transferAsset', () => {
    it('should validate transfer and update ownership', async () => {
      const existingAsset = createMockAsset();
      mockAssetRepository.findById.mockResolvedValue(existingAsset);

      const transfer = {
        newOwnership: {
          type: 'TRUST',
          holders: [{ entityId: 'trust-123', percentage: 100 }],
        },
        transferType: 'TRUST_FUNDING',
        transferValue: 100000,
        documentation: [{ type: 'DEED', documentId: 'doc-123' }],
      };

      const result = await assetService.transferAsset(existingAsset.id, transfer);

      expect(result.status).toBe('COMPLETED');
      expect(mockAssetRepository.update).toHaveBeenCalledWith(
        existingAsset.id,
        expect.objectContaining({
          ownership: transfer.newOwnership,
        })
      );
    });
  });
});

// tests/integration/api/assets.test.ts
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import request from 'supertest';
import { createApp } from '@cryo-asset/api';
import { createTestDependencies, cleanupDatabase } from '../helpers';

describe('Assets API', () => {
  let app: Express;
  let authToken: string;

  beforeAll(async () => {
    const deps = await createTestDependencies();
    app = createApp(deps);
    authToken = await getTestAuthToken(deps.authService);
  });

  afterAll(async () => {
    await cleanupDatabase();
  });

  describe('POST /api/v1/assets', () => {
    it('should create asset with valid input', async () => {
      const response = await request(app)
        .post('/api/v1/assets')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          portfolioId: testPortfolioId,
          category: 'FINANCIAL',
          type: 'BANK_ACCOUNT',
          name: 'Test Savings Account',
          ownership: {
            type: 'SOLE',
            holders: [{ entityId: testPatientId, percentage: 100 }],
          },
          acquisition: {
            date: '2024-01-01',
            cost: 50000,
            currency: 'USD',
          },
          currentValue: {
            value: 52000,
            currency: 'USD',
            valuationMethod: 'BOOK_VALUE',
            valuationDate: new Date().toISOString(),
          },
        });

      expect(response.status).toBe(201);
      expect(response.body.success).toBe(true);
      expect(response.body.data).toHaveProperty('id');
      expect(response.body.data.name).toBe('Test Savings Account');
    });

    it('should reject invalid input with validation errors', async () => {
      const response = await request(app)
        .post('/api/v1/assets')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          // Missing required fields
          name: 'Test Asset',
        });

      expect(response.status).toBe(400);
      expect(response.body.success).toBe(false);
      expect(response.body.error.code).toBe('VALIDATION_ERROR');
      expect(response.body.error.validationErrors).toBeInstanceOf(Array);
    });
  });

  describe('GET /api/v1/assets/:id', () => {
    it('should return asset with valuation history', async () => {
      const response = await request(app)
        .get(`/api/v1/assets/${testAssetId}`)
        .set('Authorization', `Bearer ${authToken}`);

      expect(response.status).toBe(200);
      expect(response.body.data).toHaveProperty('valuationHistory');
      expect(response.body.data).toHaveProperty('transactions');
    });

    it('should return 404 for non-existent asset', async () => {
      const response = await request(app)
        .get('/api/v1/assets/non-existent-id')
        .set('Authorization', `Bearer ${authToken}`);

      expect(response.status).toBe(404);
      expect(response.body.error.code).toBe('ASSET_NOT_FOUND');
    });
  });
});
```

---

## 8.6 Monitoring and Observability

### Metrics and Logging

```typescript
// packages/core/src/monitoring/metrics.ts
import { Counter, Histogram, Gauge, Registry } from 'prom-client';

export const metricsRegistry = new Registry();

// API metrics
export const httpRequestsTotal = new Counter({
  name: 'http_requests_total',
  help: 'Total number of HTTP requests',
  labelNames: ['method', 'path', 'status'],
  registers: [metricsRegistry],
});

export const httpRequestDuration = new Histogram({
  name: 'http_request_duration_seconds',
  help: 'HTTP request duration in seconds',
  labelNames: ['method', 'path'],
  buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5],
  registers: [metricsRegistry],
});

// Business metrics
export const assetsRegistered = new Counter({
  name: 'cryo_assets_registered_total',
  help: 'Total number of assets registered',
  labelNames: ['category', 'organization'],
  registers: [metricsRegistry],
});

export const portfolioValue = new Gauge({
  name: 'cryo_portfolio_value_usd',
  help: 'Total portfolio value in USD',
  labelNames: ['patient_id', 'fund_type'],
  registers: [metricsRegistry],
});

export const valuationsPerformed = new Counter({
  name: 'cryo_valuations_performed_total',
  help: 'Total number of valuations performed',
  labelNames: ['method', 'asset_category'],
  registers: [metricsRegistry],
});

export const trustDistributions = new Counter({
  name: 'cryo_trust_distributions_total',
  help: 'Total number of trust distributions',
  labelNames: ['trust_type', 'distribution_type'],
  registers: [metricsRegistry],
});

export const blockchainRegistrations = new Counter({
  name: 'cryo_blockchain_registrations_total',
  help: 'Total number of blockchain registrations',
  labelNames: ['network', 'entity_type'],
  registers: [metricsRegistry],
});

// Integration metrics
export const integrationSyncDuration = new Histogram({
  name: 'cryo_integration_sync_duration_seconds',
  help: 'Integration sync duration in seconds',
  labelNames: ['integration_type', 'status'],
  buckets: [1, 5, 10, 30, 60, 120],
  registers: [metricsRegistry],
});

export const integrationErrors = new Counter({
  name: 'cryo_integration_errors_total',
  help: 'Total number of integration errors',
  labelNames: ['integration_type', 'error_code'],
  registers: [metricsRegistry],
});
```

---

## Chapter Summary

This chapter provided comprehensive implementation guidance:

1. **Development Environment**: Project structure and TypeScript configuration
2. **Core Package**: Entity definitions and service implementations
3. **Database**: PostgreSQL schema with proper indexing and encryption
4. **API Implementation**: Express routes with validation and error handling
5. **Kubernetes Deployment**: Production-ready container orchestration
6. **Testing Framework**: Unit and integration testing strategies
7. **Monitoring**: Metrics collection and observability

These implementations provide a solid foundation for building production-ready cryonics asset management systems.

---

*Next Chapter: Future Trends - Evolution of cryonics finance and technology*
