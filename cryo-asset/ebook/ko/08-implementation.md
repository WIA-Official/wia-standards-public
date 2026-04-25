# 제8장: 구현

## 개발 지침 및 배포 전략

### 소개

냉동 보존 자산 관리 시스템을 구현하려면 장기적인 지속 가능성, 데이터 무결성 및 운영 복원력에 대한 세심한 주의가 필요합니다. 이 장에서는 WIA Cryo-Asset 표준을 기반으로 구축된 시스템을 개발, 배포 및 유지관리하는 포괄적인 지침을 제공하며, 특히 냉동 보존 애플리케이션에 필요한 확장된 시간 범위에 걸쳐 시스템이 안정적으로 작동할 수 있도록 보장하는 데 중점을 둡니다.

---

## 8.1 개발 환경 설정

### 프로젝트 초기화

```typescript
// Cryo-Asset 구현을 위한 프로젝트 구조
/*
cryo-asset-platform/
├── apps/
│   ├── api/                    # REST API 서비스
│   ├── web/                    # 웹 애플리케이션
│   ├── admin/                  # 관리자 대시보드
│   └── mobile/                 # 모바일 애플리케이션
├── packages/
│   ├── core/                   # 핵심 비즈니스 로직
│   ├── database/               # 데이터베이스 스키마 및 마이그레이션
│   ├── crypto/                 # 암호화 유틸리티
│   ├── integrations/           # 외부 서비스 통합
│   ├── blockchain/             # 블록체인 상호작용
│   └── shared/                 # 공유 타입 및 유틸리티
├── infrastructure/
│   ├── terraform/              # 코드형 인프라
│   ├── kubernetes/             # Kubernetes 매니페스트
│   ├── docker/                 # Docker 구성
│   └── scripts/                # 배포 스크립트
├── docs/
│   ├── api/                    # API 문서
│   ├── architecture/           # 아키텍처 다이어그램
│   └── operations/             # 운영 런북
└── tests/
    ├── unit/
    ├── integration/
    └── e2e/
*/

// 모노레포 루트용 package.json
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

// 코어 패키지용 TypeScript 구성
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

### 코어 패키지 구현

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

// 환자 스키마 정의
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
      'MEMBER_ACTIVE',       // 활성 회원
      'MEMBER_STANDBY',      // 대기 회원
      'MEMBER_SUSPENDED',    // 정지 회원
      'IN_TRANSPORT',        // 이송 중
      'PRESERVED',           // 보존됨
      'REVIVED',             // 소생됨
      'TERMINATED',          // 종료됨
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

// TypeORM 엔티티
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

// 환자 서비스 클래스
export class PatientService extends EventEmitter {
  constructor(
    private readonly patientRepository: PatientRepository,
    private readonly portfolioService: PortfolioService,
  ) {
    super();
  }

  // 환자 생성
  async createPatient(input: PatientInput): Promise<Patient> {
    // 입력 검증
    const validated = PatientSchema.parse({
      ...input,
      id: input.id || crypto.randomUUID(),
      createdAt: new Date(),
      updatedAt: new Date(),
    });

    // 중복 확인
    const existing = await this.patientRepository.findByOrganizationPatientId(
      validated.organizationId,
      validated.organizationPatientId
    );

    if (existing) {
      throw new CryoAssetError(
        'DUPLICATE_PATIENT',
        `ID ${validated.organizationPatientId}을(를) 가진 환자가 이미 존재합니다`
      );
    }

    // 환자 생성
    const patient = await this.patientRepository.create(validated);

    // 포트폴리오 생성
    await this.portfolioService.createPortfolio({
      patientId: patient.id,
      organizationId: patient.organizationId,
    });

    // 이벤트 발생
    this.emit('patient:created', { patient });

    return patient;
  }

  // 보존 상태 업데이트
  async updatePreservationStatus(
    patientId: string,
    status: Patient['preservationStatus']
  ): Promise<Patient> {
    const patient = await this.patientRepository.findById(patientId);

    if (!patient) {
      throw new CryoAssetError('PATIENT_NOT_FOUND', `환자 ${patientId}을(를) 찾을 수 없습니다`);
    }

    // 상태 전환 검증
    this.validateStatusTransition(patient.preservationStatus.status, status.status);

    // 상태 업데이트
    const updated = await this.patientRepository.update(patientId, {
      preservationStatus: status,
      updatedAt: new Date(),
    });

    // 이벤트 발생
    this.emit('patient:status_changed', {
      patient: updated,
      previousStatus: patient.preservationStatus.status,
      newStatus: status.status,
    });

    return updated;
  }

  // 상태 전환 검증
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
        `${from}에서 ${to}(으)로 전환할 수 없습니다`
      );
    }
  }
}

// packages/core/src/services/AssetService.ts
// 자산 서비스 클래스
export class AssetService extends EventEmitter {
  constructor(
    private readonly assetRepository: AssetRepository,
    private readonly valuationService: ValuationService,
    private readonly blockchainService: BlockchainService,
    private readonly cryptoService: CryptoService,
  ) {
    super();
  }

  // 자산 등록
  async registerAsset(input: AssetInput): Promise<Asset> {
    // 입력 검증
    const validated = AssetSchema.parse({
      ...input,
      id: input.id || crypto.randomUUID(),
      status: 'ACTIVE',
      createdAt: new Date(),
      updatedAt: new Date(),
    });

    // 민감한 데이터 암호화
    if (validated.sensitiveData) {
      validated.sensitiveData = await this.cryptoService.encrypt(
        JSON.stringify(validated.sensitiveData),
        validated.portfolioId
      );
    }

    // 자산 생성
    const asset = await this.assetRepository.create(validated);

    // 블록체인에 등록
    const blockchainRef = await this.blockchainService.registerAsset(asset);
    await this.assetRepository.updateBlockchainRef(asset.id, blockchainRef);

    // 초기 평가 획득
    const valuation = await this.valuationService.getValuation(asset);
    await this.valuationService.recordValuation(asset.id, valuation);

    // 이벤트 발생
    this.emit('asset:registered', { asset, blockchainRef, valuation });

    return asset;
  }

  // 자산 이전
  async transferAsset(
    assetId: string,
    transfer: AssetTransferInput
  ): Promise<AssetTransfer> {
    const asset = await this.assetRepository.findById(assetId);

    if (!asset) {
      throw new CryoAssetError('ASSET_NOT_FOUND', `자산 ${assetId}을(를) 찾을 수 없습니다`);
    }

    // 이전 검증
    await this.validateTransfer(asset, transfer);

    // 이전 기록 생성
    const transferRecord = await this.assetRepository.createTransfer({
      id: crypto.randomUUID(),
      assetId,
      ...transfer,
      status: 'PENDING',
      initiatedAt: new Date(),
    });

    // 자산 소유권 업데이트
    const updatedAsset = await this.assetRepository.update(assetId, {
      ownership: transfer.newOwnership,
      portfolioId: transfer.targetPortfolioId || asset.portfolioId,
      updatedAt: new Date(),
    });

    // 블록체인에 기록
    await this.blockchainService.recordTransfer(transferRecord);

    // 이전 상태 업데이트
    await this.assetRepository.updateTransferStatus(transferRecord.id, 'COMPLETED');

    // 이벤트 발생
    this.emit('asset:transferred', { asset: updatedAsset, transfer: transferRecord });

    return { ...transferRecord, status: 'COMPLETED' };
  }
}
```

---

## 8.2 데이터베이스 구현

### PostgreSQL 스키마

```typescript
// packages/database/src/migrations/001_initial_schema.ts
import { Knex } from 'knex';

export async function up(knex: Knex): Promise<void> {
  // 확장 활성화
  await knex.raw('CREATE EXTENSION IF NOT EXISTS "uuid-ossp"');
  await knex.raw('CREATE EXTENSION IF NOT EXISTS "pgcrypto"');

  // 조직 테이블
  await knex.schema.createTable('organizations', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.string('name', 200).notNullable();
    table.string('type', 50).notNullable();
    table.jsonb('settings').defaultTo('{}');
    table.timestamp('created_at').defaultTo(knex.fn.now());
    table.timestamp('updated_at').defaultTo(knex.fn.now());
  });

  // 환자 테이블
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

  // 포트폴리오 테이블
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

  // 자산 테이블
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

  // 평가 테이블
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

  // 신탁 테이블
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

  // 거래 테이블
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

  // 이벤트 테이블
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

  // 감사 로그 테이블
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

  // 암호화 키 테이블
  await knex.schema.createTable('encryption_keys', (table) => {
    table.uuid('id').primary().defaultTo(knex.raw('uuid_generate_v4()'));
    table.uuid('asset_id').nullable().references('id').inTable('assets');
    table.string('algorithm', 50).notNullable();
    table.binary('key_material').notNullable();  // 마스터 키로 암호화
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

// 롤백
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

## 8.3 API 구현

### Express 애플리케이션 설정

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

  // 보안 미들웨어
  app.use(helmet());
  app.use(cors({
    origin: process.env.ALLOWED_ORIGINS?.split(',') || [],
    credentials: true,
  }));

  // 압축
  app.use(compression());

  // 바디 파싱
  app.use(express.json({ limit: '10mb' }));
  app.use(express.urlencoded({ extended: true }));

  // 로깅
  app.use(pinoHttp({
    logger: dependencies.logger,
    customProps: (req) => ({
      requestId: req.headers['x-request-id'],
    }),
  }));

  // 속도 제한
  app.use(rateLimit({
    windowMs: 60 * 1000,  // 1분
    max: 100,  // 분당 100개 요청
    standardHeaders: true,
    legacyHeaders: false,
    keyGenerator: (req) => req.headers['x-api-key'] as string || req.ip,
  }));

  // 요청 ID
  app.use((req, res, next) => {
    req.requestId = req.headers['x-request-id'] as string || crypto.randomUUID();
    res.setHeader('X-Request-ID', req.requestId);
    next();
  });

  // 헬스 체크 (인증 불필요)
  app.get('/health', (req, res) => {
    res.json({ status: 'healthy', timestamp: new Date().toISOString() });
  });

  // API 라우트
  app.use('/api/v1', authMiddleware(dependencies.authService));
  app.use('/api/v1/patients', patientRouter(dependencies));
  app.use('/api/v1/portfolios', portfolioRouter(dependencies));
  app.use('/api/v1/assets', assetRouter(dependencies));
  app.use('/api/v1/trusts', trustRouter(dependencies));
  app.use('/api/v1/integrations', integrationRouter(dependencies));
  app.use('/api/v1/webhooks', webhookRouter(dependencies));

  // 에러 처리
  app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
    const requestId = req.requestId || 'unknown';

    if (err instanceof ZodError) {
      return res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: '검증 실패',
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

    // 예상치 못한 오류 로깅
    dependencies.logger.error({ err, requestId }, '예상치 못한 오류');

    return res.status(500).json({
      success: false,
      error: {
        code: 'INTERNAL_ERROR',
        message: '예상치 못한 오류가 발생했습니다',
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

  // 자산 생성
  router.post('/', async (req, res, next) => {
    try {
      const input = CreateAssetSchema.parse(req.body);

      // 권한 확인
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

  // 자산 조회
  router.get('/:id', async (req, res, next) => {
    try {
      const asset = await assetService.getAsset(req.params.id);

      if (!asset) {
        throw new ApiError('ASSET_NOT_FOUND', '자산을 찾을 수 없습니다', 404);
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

  // 평가 업데이트
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

  // 자산 이전
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

## 8.4 Kubernetes 배포

### 인프라 구성

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
# 서비스
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
# 수평 파드 오토스케일러
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

---

## 8.5 테스트 프레임워크

### 포괄적인 테스트

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
    it('자산을 생성하고 블록체인에 등록해야 합니다', async () => {
      const input = {
        portfolioId: 'portfolio-123',
        category: 'FINANCIAL',
        type: 'BANK_ACCOUNT',
        name: '예금 계좌',
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

    it('asset:registered 이벤트를 발생시켜야 합니다', async () => {
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
    it('이전을 검증하고 소유권을 업데이트해야 합니다', async () => {
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
    it('유효한 입력으로 자산을 생성해야 합니다', async () => {
      const response = await request(app)
        .post('/api/v1/assets')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          portfolioId: testPortfolioId,
          category: 'FINANCIAL',
          type: 'BANK_ACCOUNT',
          name: '테스트 예금 계좌',
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
      expect(response.body.data.name).toBe('테스트 예금 계좌');
    });

    it('유효하지 않은 입력에 대해 검증 오류를 반환해야 합니다', async () => {
      const response = await request(app)
        .post('/api/v1/assets')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          // 필수 필드 누락
          name: '테스트 자산',
        });

      expect(response.status).toBe(400);
      expect(response.body.success).toBe(false);
      expect(response.body.error.code).toBe('VALIDATION_ERROR');
      expect(response.body.error.validationErrors).toBeInstanceOf(Array);
    });
  });

  describe('GET /api/v1/assets/:id', () => {
    it('평가 내역과 함께 자산을 반환해야 합니다', async () => {
      const response = await request(app)
        .get(`/api/v1/assets/${testAssetId}`)
        .set('Authorization', `Bearer ${authToken}`);

      expect(response.status).toBe(200);
      expect(response.body.data).toHaveProperty('valuationHistory');
      expect(response.body.data).toHaveProperty('transactions');
    });

    it('존재하지 않는 자산에 대해 404를 반환해야 합니다', async () => {
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

## 8.6 모니터링 및 관찰 가능성

### 메트릭 및 로깅

```typescript
// packages/core/src/monitoring/metrics.ts
import { Counter, Histogram, Gauge, Registry } from 'prom-client';

export const metricsRegistry = new Registry();

// API 메트릭
export const httpRequestsTotal = new Counter({
  name: 'http_requests_total',
  help: '총 HTTP 요청 수',
  labelNames: ['method', 'path', 'status'],
  registers: [metricsRegistry],
});

export const httpRequestDuration = new Histogram({
  name: 'http_request_duration_seconds',
  help: 'HTTP 요청 지속 시간(초)',
  labelNames: ['method', 'path'],
  buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5],
  registers: [metricsRegistry],
});

// 비즈니스 메트릭
export const assetsRegistered = new Counter({
  name: 'cryo_assets_registered_total',
  help: '등록된 총 자산 수',
  labelNames: ['category', 'organization'],
  registers: [metricsRegistry],
});

export const portfolioValue = new Gauge({
  name: 'cryo_portfolio_value_usd',
  help: '총 포트폴리오 가치(USD)',
  labelNames: ['patient_id', 'fund_type'],
  registers: [metricsRegistry],
});

export const valuationsPerformed = new Counter({
  name: 'cryo_valuations_performed_total',
  help: '수행된 총 평가 수',
  labelNames: ['method', 'asset_category'],
  registers: [metricsRegistry],
});

export const trustDistributions = new Counter({
  name: 'cryo_trust_distributions_total',
  help: '총 신탁 분배 수',
  labelNames: ['trust_type', 'distribution_type'],
  registers: [metricsRegistry],
});

export const blockchainRegistrations = new Counter({
  name: 'cryo_blockchain_registrations_total',
  help: '총 블록체인 등록 수',
  labelNames: ['network', 'entity_type'],
  registers: [metricsRegistry],
});

// 통합 메트릭
export const integrationSyncDuration = new Histogram({
  name: 'cryo_integration_sync_duration_seconds',
  help: '통합 동기화 지속 시간(초)',
  labelNames: ['integration_type', 'status'],
  buckets: [1, 5, 10, 30, 60, 120],
  registers: [metricsRegistry],
});

export const integrationErrors = new Counter({
  name: 'cryo_integration_errors_total',
  help: '총 통합 오류 수',
  labelNames: ['integration_type', 'error_code'],
  registers: [metricsRegistry],
});
```

---

## 장 요약

이 장에서는 포괄적인 구현 지침을 제공했습니다:

1. **개발 환경**: 프로젝트 구조 및 TypeScript 구성
2. **코어 패키지**: 엔티티 정의 및 서비스 구현
3. **데이터베이스**: 적절한 인덱싱 및 암호화가 포함된 PostgreSQL 스키마
4. **API 구현**: 검증 및 오류 처리가 포함된 Express 라우트
5. **Kubernetes 배포**: 프로덕션 준비 컨테이너 오케스트레이션
6. **테스트 프레임워크**: 단위 및 통합 테스트 전략
7. **모니터링**: 메트릭 수집 및 관찰 가능성

이러한 구현은 프로덕션 준비 냉동 보존 자산 관리 시스템을 구축하기 위한 견고한 기반을 제공합니다.

---

*다음 장: 미래 동향 - 냉동 보존 금융 및 기술의 진화*
