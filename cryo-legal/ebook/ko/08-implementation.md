# 제8장: 구현 가이드 - 배포 및 운영

## 종합 구현 프레임워크

이 장에서는 WIA 극저온 법률 표준의 완전한 구현 가이드를 제공하며, 배포 전략, 데이터베이스 마이그레이션, 성능 최적화 및 상태 모니터링을 포함합니다.

## 배포 관리자

```typescript
/**
 * WIA 극저온 법률 표준 - 배포 관리자
 * 시스템 배포 및 설정 관리
 */

import { z } from 'zod';

// ============================================================================
// 배포 설정 스키마
// ============================================================================

export const DeploymentConfigSchema = z.object({
  environment: z.enum(['development', 'staging', 'production']),
  region: z.string(),
  database: z.object({
    type: z.enum(['postgresql', 'mysql', 'mongodb']),
    host: z.string(),
    port: z.number(),
    name: z.string(),
    ssl: z.boolean(),
    poolSize: z.number().min(5).max(100),
    replication: z.boolean(),
  }),
  cache: z.object({
    type: z.enum(['redis', 'memcached', 'local']),
    host: z.string().optional(),
    port: z.number().optional(),
    ttl: z.number(),
  }),
  storage: z.object({
    type: z.enum(['local', 's3', 'gcs', 'azure']),
    bucket: z.string().optional(),
    region: z.string().optional(),
    encryption: z.boolean(),
  }),
  scaling: z.object({
    minInstances: z.number().min(1),
    maxInstances: z.number().max(100),
    targetCpuUtilization: z.number().min(50).max(90),
    targetMemoryUtilization: z.number().min(50).max(90),
  }),
  monitoring: z.object({
    enabled: z.boolean(),
    metricsInterval: z.number(),
    alerting: z.boolean(),
    logLevel: z.enum(['debug', 'info', 'warn', 'error']),
  }),
});

export type DeploymentConfig = z.infer<typeof DeploymentConfigSchema>;

// ============================================================================
// 한국 리전 배포 설정
// ============================================================================

export const koreanProductionConfig: DeploymentConfig = {
  environment: 'production',
  region: 'ap-northeast-2',  // 서울
  database: {
    type: 'postgresql',
    host: 'db.cryo-legal.kr',
    port: 5432,
    name: 'cryo_legal_prod',
    ssl: true,
    poolSize: 50,
    replication: true,
  },
  cache: {
    type: 'redis',
    host: 'redis.cryo-legal.kr',
    port: 6379,
    ttl: 3600,
  },
  storage: {
    type: 's3',
    bucket: 'cryo-legal-documents-kr',
    region: 'ap-northeast-2',
    encryption: true,
  },
  scaling: {
    minInstances: 3,
    maxInstances: 20,
    targetCpuUtilization: 70,
    targetMemoryUtilization: 75,
  },
  monitoring: {
    enabled: true,
    metricsInterval: 60,
    alerting: true,
    logLevel: 'info',
  },
};

// ============================================================================
// 배포 관리자
// ============================================================================

export class DeploymentManager {
  private config: DeploymentConfig;
  private status: DeploymentStatus = {
    phase: 'pending',
    phaseKr: '대기 중',
    progress: 0,
    steps: [],
    startedAt: '',
    completedAt: '',
  };

  constructor(config: DeploymentConfig) {
    this.config = DeploymentConfigSchema.parse(config);
  }

  async deploy(): Promise<DeploymentResult> {
    this.status.phase = 'deploying';
    this.status.phaseKr = '배포 중';
    this.status.startedAt = new Date().toISOString();

    const steps: DeploymentStep[] = [
      { name: 'validate', nameKr: '검증', status: 'pending', statusKr: '대기' },
      { name: 'prepare', nameKr: '준비', status: 'pending', statusKr: '대기' },
      { name: 'database', nameKr: '데이터베이스', status: 'pending', statusKr: '대기' },
      { name: 'cache', nameKr: '캐시', status: 'pending', statusKr: '대기' },
      { name: 'storage', nameKr: '스토리지', status: 'pending', statusKr: '대기' },
      { name: 'application', nameKr: '애플리케이션', status: 'pending', statusKr: '대기' },
      { name: 'healthcheck', nameKr: '상태 확인', status: 'pending', statusKr: '대기' },
      { name: 'finalize', nameKr: '완료', status: 'pending', statusKr: '대기' },
    ];

    this.status.steps = steps;

    try {
      // 1. 검증
      await this.executeStep('validate', async () => {
        await this.validateConfiguration();
        await this.validateDependencies();
      });

      // 2. 준비
      await this.executeStep('prepare', async () => {
        await this.prepareEnvironment();
        await this.backupCurrentState();
      });

      // 3. 데이터베이스
      await this.executeStep('database', async () => {
        await this.setupDatabase();
        await this.runMigrations();
      });

      // 4. 캐시
      await this.executeStep('cache', async () => {
        await this.setupCache();
      });

      // 5. 스토리지
      await this.executeStep('storage', async () => {
        await this.setupStorage();
      });

      // 6. 애플리케이션
      await this.executeStep('application', async () => {
        await this.deployApplication();
        await this.configureLoadBalancer();
      });

      // 7. 상태 확인
      await this.executeStep('healthcheck', async () => {
        await this.runHealthChecks();
      });

      // 8. 완료
      await this.executeStep('finalize', async () => {
        await this.updateDnsRecords();
        await this.notifyStakeholders();
      });

      this.status.phase = 'completed';
      this.status.phaseKr = '완료';
      this.status.completedAt = new Date().toISOString();

      return {
        success: true,
        successKr: '성공',
        status: this.status,
        duration: this.calculateDuration(),
      };

    } catch (error) {
      this.status.phase = 'failed';
      this.status.phaseKr = '실패';
      this.status.error = error instanceof Error ? error.message : String(error);

      await this.rollback();

      return {
        success: false,
        successKr: '실패',
        status: this.status,
        error: this.status.error,
        duration: this.calculateDuration(),
      };
    }
  }

  private async executeStep(name: string, action: () => Promise<void>): Promise<void> {
    const step = this.status.steps.find(s => s.name === name);
    if (!step) return;

    step.status = 'running';
    step.statusKr = '실행 중';
    step.startedAt = new Date().toISOString();

    try {
      await action();
      step.status = 'completed';
      step.statusKr = '완료';
      step.completedAt = new Date().toISOString();
      this.updateProgress();
    } catch (error) {
      step.status = 'failed';
      step.statusKr = '실패';
      step.error = error instanceof Error ? error.message : String(error);
      throw error;
    }
  }

  private updateProgress(): void {
    const completed = this.status.steps.filter(s => s.status === 'completed').length;
    this.status.progress = Math.round((completed / this.status.steps.length) * 100);
  }

  private calculateDuration(): number {
    if (!this.status.startedAt) return 0;
    const end = this.status.completedAt ? new Date(this.status.completedAt) : new Date();
    return end.getTime() - new Date(this.status.startedAt).getTime();
  }

  // ============================================================================
  // 배포 단계 구현
  // ============================================================================

  private async validateConfiguration(): Promise<void> {
    console.log('설정 검증 중...');
    DeploymentConfigSchema.parse(this.config);
  }

  private async validateDependencies(): Promise<void> {
    console.log('의존성 검증 중...');
    // 필요한 서비스 연결 확인
  }

  private async prepareEnvironment(): Promise<void> {
    console.log('환경 준비 중...');
    // 환경 변수 설정, 시크릿 로드
  }

  private async backupCurrentState(): Promise<void> {
    console.log('현재 상태 백업 중...');
    // 데이터베이스 스냅샷, 설정 백업
  }

  private async setupDatabase(): Promise<void> {
    console.log('데이터베이스 설정 중...');
    // 데이터베이스 연결 및 초기 설정
  }

  private async runMigrations(): Promise<void> {
    console.log('마이그레이션 실행 중...');
    // 데이터베이스 마이그레이션
  }

  private async setupCache(): Promise<void> {
    console.log('캐시 설정 중...');
    // Redis/캐시 연결 및 설정
  }

  private async setupStorage(): Promise<void> {
    console.log('스토리지 설정 중...');
    // S3/스토리지 버킷 설정
  }

  private async deployApplication(): Promise<void> {
    console.log('애플리케이션 배포 중...');
    // 컨테이너/서버 배포
  }

  private async configureLoadBalancer(): Promise<void> {
    console.log('로드밸런서 설정 중...');
    // 로드밸런서 설정
  }

  private async runHealthChecks(): Promise<void> {
    console.log('상태 확인 중...');
    // API 엔드포인트 테스트
  }

  private async updateDnsRecords(): Promise<void> {
    console.log('DNS 레코드 업데이트 중...');
    // DNS 설정 업데이트
  }

  private async notifyStakeholders(): Promise<void> {
    console.log('관계자 알림 중...');
    // 배포 완료 알림
  }

  private async rollback(): Promise<void> {
    console.log('롤백 실행 중...');
    // 이전 상태로 복구
  }

  getStatus(): DeploymentStatus {
    return this.status;
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface DeploymentStatus {
  phase: 'pending' | 'deploying' | 'completed' | 'failed' | 'rolling-back';
  phaseKr: string;
  progress: number;
  steps: DeploymentStep[];
  startedAt: string;
  completedAt: string;
  error?: string;
}

export interface DeploymentStep {
  name: string;
  nameKr: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'skipped';
  statusKr: string;
  startedAt?: string;
  completedAt?: string;
  error?: string;
}

export interface DeploymentResult {
  success: boolean;
  successKr: string;
  status: DeploymentStatus;
  error?: string;
  duration: number;
}
```

## 마이그레이션 관리자

```typescript
/**
 * 데이터베이스 마이그레이션 관리자
 * 스키마 버전 관리 및 마이그레이션 실행
 */

export class MigrationManager {
  private migrations: Migration[] = [];
  private appliedMigrations: Set<string> = new Set();

  constructor(
    private readonly db: DatabaseConnection,
    private readonly config: MigrationConfig
  ) {}

  async initialize(): Promise<void> {
    // 마이그레이션 테이블 생성
    await this.ensureMigrationTable();

    // 적용된 마이그레이션 로드
    await this.loadAppliedMigrations();

    // 마이그레이션 파일 로드
    this.loadMigrations();
  }

  private async ensureMigrationTable(): Promise<void> {
    const sql = `
      CREATE TABLE IF NOT EXISTS _migrations (
        id VARCHAR(255) PRIMARY KEY,
        name VARCHAR(255) NOT NULL,
        applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        checksum VARCHAR(64),
        execution_time INTEGER
      )
    `;

    await this.db.execute(sql);
  }

  private async loadAppliedMigrations(): Promise<void> {
    const result = await this.db.query('SELECT id FROM _migrations');
    this.appliedMigrations = new Set(result.rows.map(r => r.id));
  }

  private loadMigrations(): void {
    // 극저온 법률 표준 마이그레이션
    this.migrations = [
      // 초기 스키마
      {
        id: '001_initial_schema',
        name: '초기 스키마 생성',
        up: `
          -- 조직 테이블
          CREATE TABLE organizations (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            name VARCHAR(255) NOT NULL,
            name_kr VARCHAR(255),
            type VARCHAR(50) NOT NULL,
            registration_number VARCHAR(100),
            address JSONB,
            contact JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_organizations_type ON organizations(type);

          -- 계약 테이블
          CREATE TABLE contracts (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            contract_number VARCHAR(50) UNIQUE NOT NULL,
            type VARCHAR(50) NOT NULL,
            status VARCHAR(50) NOT NULL DEFAULT 'draft',
            parties JSONB NOT NULL,
            terms JSONB NOT NULL,
            effective_date DATE,
            expiry_date DATE,
            created_by UUID REFERENCES organizations(id),
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_contracts_status ON contracts(status);
          CREATE INDEX idx_contracts_type ON contracts(type);

          -- 분쟁 테이블
          CREATE TABLE disputes (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            dispute_number VARCHAR(50) UNIQUE NOT NULL,
            type VARCHAR(50) NOT NULL,
            status VARCHAR(50) NOT NULL DEFAULT 'filed',
            parties JSONB NOT NULL,
            subject TEXT NOT NULL,
            description TEXT,
            filed_date DATE NOT NULL,
            resolved_date DATE,
            resolution JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_disputes_status ON disputes(status);
        `,
        down: `
          DROP TABLE IF EXISTS disputes;
          DROP TABLE IF EXISTS contracts;
          DROP TABLE IF EXISTS organizations;
        `,
      },

      // 컴플라이언스 테이블
      {
        id: '002_compliance_tables',
        name: '컴플라이언스 테이블 추가',
        up: `
          -- 규정 테이블
          CREATE TABLE regulations (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            code VARCHAR(100) NOT NULL,
            name VARCHAR(255) NOT NULL,
            name_kr VARCHAR(255),
            jurisdiction VARCHAR(50) NOT NULL,
            category VARCHAR(100),
            effective_date DATE,
            requirements JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_regulations_jurisdiction ON regulations(jurisdiction);

          -- 컴플라이언스 상태 테이블
          CREATE TABLE compliance_status (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            organization_id UUID REFERENCES organizations(id),
            regulation_id UUID REFERENCES regulations(id),
            status VARCHAR(50) NOT NULL DEFAULT 'pending',
            assessment_date DATE,
            findings JSONB,
            remediation_plan JSONB,
            next_review_date DATE,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            UNIQUE(organization_id, regulation_id)
          );

          CREATE INDEX idx_compliance_status ON compliance_status(status);
        `,
        down: `
          DROP TABLE IF EXISTS compliance_status;
          DROP TABLE IF EXISTS regulations;
        `,
      },

      // 감사 로그 테이블
      {
        id: '003_audit_logs',
        name: '감사 로그 테이블 추가',
        up: `
          CREATE TABLE audit_logs (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            event_type VARCHAR(100) NOT NULL,
            category VARCHAR(50) NOT NULL,
            severity VARCHAR(20) NOT NULL,
            description TEXT,
            user_id UUID,
            ip_address INET,
            resource VARCHAR(100),
            resource_id UUID,
            action VARCHAR(50),
            details JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_audit_logs_event_type ON audit_logs(event_type);
          CREATE INDEX idx_audit_logs_user_id ON audit_logs(user_id);
          CREATE INDEX idx_audit_logs_created_at ON audit_logs(created_at);

          -- 파티션 설정 (월별)
          -- PostgreSQL 10+ 필요
        `,
        down: `
          DROP TABLE IF EXISTS audit_logs;
        `,
      },

      // 서명 테이블
      {
        id: '004_signatures',
        name: '전자서명 테이블 추가',
        up: `
          CREATE TABLE signatures (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            contract_id UUID REFERENCES contracts(id),
            party_id UUID REFERENCES organizations(id),
            signatory_name VARCHAR(255) NOT NULL,
            signatory_email VARCHAR(255),
            signature_method VARCHAR(50) NOT NULL,
            signature_data TEXT,
            certificate_info JSONB,
            signed_at TIMESTAMP,
            ip_address INET,
            user_agent TEXT,
            verification_status VARCHAR(50) DEFAULT 'pending',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_signatures_contract ON signatures(contract_id);
          CREATE INDEX idx_signatures_party ON signatures(party_id);
        `,
        down: `
          DROP TABLE IF EXISTS signatures;
        `,
      },

      // 문서 테이블
      {
        id: '005_documents',
        name: '문서 관리 테이블 추가',
        up: `
          CREATE TABLE documents (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            name VARCHAR(255) NOT NULL,
            type VARCHAR(50) NOT NULL,
            mime_type VARCHAR(100),
            size_bytes BIGINT,
            storage_path VARCHAR(500) NOT NULL,
            checksum VARCHAR(64),
            encryption_key_id VARCHAR(100),
            uploaded_by UUID,
            contract_id UUID REFERENCES contracts(id),
            dispute_id UUID REFERENCES disputes(id),
            metadata JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_documents_contract ON documents(contract_id);
          CREATE INDEX idx_documents_dispute ON documents(dispute_id);
          CREATE INDEX idx_documents_type ON documents(type);
        `,
        down: `
          DROP TABLE IF EXISTS documents;
        `,
      },

      // 한국 특화 테이블
      {
        id: '006_korean_specific',
        name: '한국 특화 테이블 추가',
        up: `
          -- 생명윤리 승인 테이블
          CREATE TABLE bioethics_approvals (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            organization_id UUID REFERENCES organizations(id),
            approval_type VARCHAR(100) NOT NULL,
            approval_number VARCHAR(100) UNIQUE NOT NULL,
            issuing_authority VARCHAR(255) NOT NULL,
            issue_date DATE NOT NULL,
            expiry_date DATE,
            scope TEXT,
            conditions JSONB,
            status VARCHAR(50) DEFAULT 'active',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_bioethics_org ON bioethics_approvals(organization_id);

          -- 개인정보 처리 동의 테이블
          CREATE TABLE privacy_consents (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id VARCHAR(100) NOT NULL,
            purpose VARCHAR(255) NOT NULL,
            consent_given BOOLEAN NOT NULL,
            consent_date TIMESTAMP NOT NULL,
            withdrawal_date TIMESTAMP,
            retention_period VARCHAR(50),
            third_party_sharing BOOLEAN DEFAULT FALSE,
            overseas_transfer BOOLEAN DEFAULT FALSE,
            details JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
          );

          CREATE INDEX idx_privacy_consents_subject ON privacy_consents(subject_id);
        `,
        down: `
          DROP TABLE IF EXISTS privacy_consents;
          DROP TABLE IF EXISTS bioethics_approvals;
        `,
      },
    ];
  }

  async migrate(): Promise<MigrationResult> {
    const pending = this.getPendingMigrations();

    if (pending.length === 0) {
      return {
        success: true,
        message: '적용할 마이그레이션이 없습니다',
        applied: [],
      };
    }

    const applied: string[] = [];

    for (const migration of pending) {
      try {
        const startTime = Date.now();

        await this.db.transaction(async (trx) => {
          // 마이그레이션 실행
          await trx.execute(migration.up);

          // 마이그레이션 기록
          await trx.execute(`
            INSERT INTO _migrations (id, name, checksum, execution_time)
            VALUES ($1, $2, $3, $4)
          `, [
            migration.id,
            migration.name,
            this.calculateChecksum(migration.up),
            Date.now() - startTime,
          ]);
        });

        this.appliedMigrations.add(migration.id);
        applied.push(migration.id);

        console.log(`✓ ${migration.id}: ${migration.name}`);

      } catch (error) {
        return {
          success: false,
          message: `마이그레이션 실패: ${migration.id}`,
          error: error instanceof Error ? error.message : String(error),
          applied,
        };
      }
    }

    return {
      success: true,
      message: `${applied.length}개 마이그레이션 적용 완료`,
      applied,
    };
  }

  async rollback(steps: number = 1): Promise<MigrationResult> {
    const applied = this.getAppliedMigrations().reverse();
    const toRollback = applied.slice(0, steps);

    if (toRollback.length === 0) {
      return {
        success: true,
        message: '롤백할 마이그레이션이 없습니다',
        applied: [],
      };
    }

    const rolledBack: string[] = [];

    for (const migration of toRollback) {
      try {
        await this.db.transaction(async (trx) => {
          // 롤백 실행
          await trx.execute(migration.down);

          // 마이그레이션 기록 삭제
          await trx.execute('DELETE FROM _migrations WHERE id = $1', [migration.id]);
        });

        this.appliedMigrations.delete(migration.id);
        rolledBack.push(migration.id);

        console.log(`↩ ${migration.id}: ${migration.name} (롤백됨)`);

      } catch (error) {
        return {
          success: false,
          message: `롤백 실패: ${migration.id}`,
          error: error instanceof Error ? error.message : String(error),
          applied: rolledBack,
        };
      }
    }

    return {
      success: true,
      message: `${rolledBack.length}개 마이그레이션 롤백 완료`,
      applied: rolledBack,
    };
  }

  getPendingMigrations(): Migration[] {
    return this.migrations.filter(m => !this.appliedMigrations.has(m.id));
  }

  getAppliedMigrations(): Migration[] {
    return this.migrations.filter(m => this.appliedMigrations.has(m.id));
  }

  getStatus(): MigrationStatus {
    return {
      total: this.migrations.length,
      applied: this.appliedMigrations.size,
      pending: this.migrations.length - this.appliedMigrations.size,
      migrations: this.migrations.map(m => ({
        id: m.id,
        name: m.name,
        applied: this.appliedMigrations.has(m.id),
        appliedKr: this.appliedMigrations.has(m.id) ? '적용됨' : '대기 중',
      })),
    };
  }

  private calculateChecksum(sql: string): string {
    const crypto = require('crypto');
    return crypto.createHash('sha256').update(sql).digest('hex').substring(0, 16);
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface Migration {
  id: string;
  name: string;
  up: string;
  down: string;
}

export interface MigrationConfig {
  tableName: string;
  migrationsPath: string;
}

export interface MigrationResult {
  success: boolean;
  message: string;
  error?: string;
  applied: string[];
}

export interface MigrationStatus {
  total: number;
  applied: number;
  pending: number;
  migrations: {
    id: string;
    name: string;
    applied: boolean;
    appliedKr: string;
  }[];
}

export interface DatabaseConnection {
  execute(sql: string, params?: unknown[]): Promise<void>;
  query(sql: string, params?: unknown[]): Promise<{ rows: any[] }>;
  transaction<T>(fn: (trx: DatabaseConnection) => Promise<T>): Promise<T>;
}
```

## 성능 최적화기

```typescript
/**
 * 성능 최적화기
 * 쿼리 최적화, 캐싱, 인덱싱
 */

export class PerformanceOptimizer {
  private queryCache: Map<string, CachedQuery> = new Map();
  private metrics: PerformanceMetrics = {
    queries: [],
    cacheHits: 0,
    cacheMisses: 0,
    slowQueries: [],
  };

  constructor(
    private readonly db: DatabaseConnection,
    private readonly cache: CacheService,
    private readonly config: PerformanceConfig
  ) {}

  // ============================================================================
  // 쿼리 최적화
  // ============================================================================

  async executeOptimized<T>(
    query: string,
    params: unknown[] = [],
    options: QueryOptions = {}
  ): Promise<T[]> {
    const cacheKey = this.generateCacheKey(query, params);

    // 캐시 확인
    if (options.cache !== false) {
      const cached = await this.getFromCache<T[]>(cacheKey);
      if (cached) {
        this.metrics.cacheHits++;
        return cached;
      }
      this.metrics.cacheMisses++;
    }

    // 쿼리 실행
    const startTime = Date.now();
    const result = await this.db.query(query, params);
    const duration = Date.now() - startTime;

    // 메트릭 기록
    this.recordQueryMetric(query, duration);

    // 느린 쿼리 기록
    if (duration > this.config.slowQueryThreshold) {
      this.recordSlowQuery(query, params, duration);
    }

    // 캐시 저장
    if (options.cache !== false && options.cacheTtl) {
      await this.saveToCache(cacheKey, result.rows, options.cacheTtl);
    }

    return result.rows as T[];
  }

  private generateCacheKey(query: string, params: unknown[]): string {
    const crypto = require('crypto');
    const data = JSON.stringify({ query, params });
    return `query:${crypto.createHash('md5').update(data).digest('hex')}`;
  }

  private async getFromCache<T>(key: string): Promise<T | null> {
    try {
      const cached = this.queryCache.get(key);
      if (cached && Date.now() < cached.expiresAt) {
        return cached.data as T;
      }
      return await this.cache.get<T>(key);
    } catch {
      return null;
    }
  }

  private async saveToCache(key: string, data: unknown, ttl: number): Promise<void> {
    // 로컬 캐시
    this.queryCache.set(key, {
      data,
      expiresAt: Date.now() + ttl * 1000,
    });

    // 분산 캐시
    await this.cache.set(key, data, ttl);
  }

  private recordQueryMetric(query: string, duration: number): void {
    this.metrics.queries.push({
      query: this.normalizeQuery(query),
      duration,
      timestamp: new Date().toISOString(),
    });

    // 최근 1000개만 유지
    if (this.metrics.queries.length > 1000) {
      this.metrics.queries = this.metrics.queries.slice(-1000);
    }
  }

  private recordSlowQuery(query: string, params: unknown[], duration: number): void {
    this.metrics.slowQueries.push({
      query: this.normalizeQuery(query),
      params,
      duration,
      timestamp: new Date().toISOString(),
    });

    console.warn(`느린 쿼리 감지 (${duration}ms): ${query.substring(0, 100)}...`);
  }

  private normalizeQuery(query: string): string {
    return query.replace(/\s+/g, ' ').trim();
  }

  // ============================================================================
  // 인덱스 분석
  // ============================================================================

  async analyzeIndexes(): Promise<IndexAnalysis[]> {
    const unusedIndexes = await this.findUnusedIndexes();
    const missingIndexes = await this.suggestMissingIndexes();
    const duplicateIndexes = await this.findDuplicateIndexes();

    return [
      ...unusedIndexes.map(idx => ({
        ...idx,
        recommendation: 'unused',
        recommendationKr: '미사용 - 삭제 고려',
      })),
      ...missingIndexes.map(idx => ({
        ...idx,
        recommendation: 'missing',
        recommendationKr: '누락 - 생성 권장',
      })),
      ...duplicateIndexes.map(idx => ({
        ...idx,
        recommendation: 'duplicate',
        recommendationKr: '중복 - 통합 권장',
      })),
    ];
  }

  private async findUnusedIndexes(): Promise<IndexInfo[]> {
    const query = `
      SELECT
        schemaname,
        tablename,
        indexname,
        idx_scan,
        pg_size_pretty(pg_relation_size(indexrelid)) as index_size
      FROM pg_stat_user_indexes
      WHERE idx_scan = 0
      AND indexrelid NOT IN (
        SELECT conindid FROM pg_constraint WHERE contype = 'p'
      )
      ORDER BY pg_relation_size(indexrelid) DESC
    `;

    const result = await this.db.query(query);

    return result.rows.map(row => ({
      name: row.indexname,
      table: row.tablename,
      size: row.index_size,
      scans: row.idx_scan,
    }));
  }

  private async suggestMissingIndexes(): Promise<IndexInfo[]> {
    // 느린 쿼리 분석을 통한 인덱스 제안
    const suggestions: IndexInfo[] = [];

    for (const slowQuery of this.metrics.slowQueries) {
      const tables = this.extractTables(slowQuery.query);
      const conditions = this.extractConditions(slowQuery.query);

      for (const table of tables) {
        for (const condition of conditions) {
          suggestions.push({
            name: `idx_${table}_${condition}`,
            table,
            columns: [condition],
            suggestedSql: `CREATE INDEX idx_${table}_${condition} ON ${table}(${condition})`,
          });
        }
      }
    }

    return suggestions;
  }

  private async findDuplicateIndexes(): Promise<IndexInfo[]> {
    const query = `
      SELECT
        a.indexname as index1,
        b.indexname as index2,
        a.tablename
      FROM pg_indexes a
      JOIN pg_indexes b ON a.tablename = b.tablename
        AND a.indexdef LIKE b.indexdef || '%'
        AND a.indexname != b.indexname
    `;

    const result = await this.db.query(query);

    return result.rows.map(row => ({
      name: row.index1,
      table: row.tablename,
      duplicateOf: row.index2,
    }));
  }

  private extractTables(query: string): string[] {
    const matches = query.match(/FROM\s+(\w+)/gi) || [];
    return matches.map(m => m.replace(/FROM\s+/i, ''));
  }

  private extractConditions(query: string): string[] {
    const matches = query.match(/WHERE\s+(\w+)\s*=/gi) || [];
    return matches.map(m => m.replace(/WHERE\s+/i, '').replace(/\s*=.*/i, ''));
  }

  // ============================================================================
  // 메트릭
  // ============================================================================

  getMetrics(): PerformanceMetrics {
    return {
      ...this.metrics,
      cacheHitRate: this.metrics.cacheHits + this.metrics.cacheMisses > 0
        ? (this.metrics.cacheHits / (this.metrics.cacheHits + this.metrics.cacheMisses)) * 100
        : 0,
    };
  }

  getSlowQueries(): SlowQuery[] {
    return this.metrics.slowQueries.sort((a, b) => b.duration - a.duration);
  }

  clearCache(): void {
    this.queryCache.clear();
    this.cache.clear();
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface PerformanceConfig {
  slowQueryThreshold: number;  // ms
  cacheEnabled: boolean;
  defaultCacheTtl: number;     // seconds
}

export interface QueryOptions {
  cache?: boolean;
  cacheTtl?: number;
}

export interface CachedQuery {
  data: unknown;
  expiresAt: number;
}

export interface PerformanceMetrics {
  queries: QueryMetric[];
  cacheHits: number;
  cacheMisses: number;
  cacheHitRate?: number;
  slowQueries: SlowQuery[];
}

export interface QueryMetric {
  query: string;
  duration: number;
  timestamp: string;
}

export interface SlowQuery {
  query: string;
  params: unknown[];
  duration: number;
  timestamp: string;
}

export interface IndexAnalysis {
  name: string;
  table: string;
  recommendation: string;
  recommendationKr: string;
  size?: string;
  scans?: number;
  columns?: string[];
  suggestedSql?: string;
  duplicateOf?: string;
}

export interface IndexInfo {
  name: string;
  table: string;
  size?: string;
  scans?: number;
  columns?: string[];
  suggestedSql?: string;
  duplicateOf?: string;
}

export interface CacheService {
  get<T>(key: string): Promise<T | null>;
  set(key: string, value: unknown, ttl: number): Promise<void>;
  delete(key: string): Promise<void>;
  clear(): Promise<void>;
}
```

## 상태 모니터링 서비스

```typescript
/**
 * 상태 모니터링 서비스
 * 시스템 상태 확인 및 알림
 */

export class HealthMonitoringService {
  private checks: Map<string, HealthCheck> = new Map();
  private history: HealthCheckResult[] = [];
  private alertHandlers: AlertHandler[] = [];

  constructor(private readonly config: MonitoringConfig) {}

  async initialize(): Promise<void> {
    this.registerDefaultChecks();
    this.startPeriodicChecks();
  }

  private registerDefaultChecks(): void {
    // 데이터베이스 상태 확인
    this.registerCheck({
      name: 'database',
      nameKr: '데이터베이스',
      type: 'critical',
      interval: 30000,
      timeout: 5000,
      check: async () => {
        // 데이터베이스 연결 테스트
        return { healthy: true, latency: 10 };
      },
    });

    // 캐시 상태 확인
    this.registerCheck({
      name: 'cache',
      nameKr: '캐시',
      type: 'important',
      interval: 30000,
      timeout: 3000,
      check: async () => {
        return { healthy: true, latency: 2 };
      },
    });

    // 스토리지 상태 확인
    this.registerCheck({
      name: 'storage',
      nameKr: '스토리지',
      type: 'important',
      interval: 60000,
      timeout: 10000,
      check: async () => {
        return { healthy: true, spaceAvailable: '500GB' };
      },
    });

    // 메모리 상태 확인
    this.registerCheck({
      name: 'memory',
      nameKr: '메모리',
      type: 'warning',
      interval: 10000,
      timeout: 1000,
      check: async () => {
        const used = process.memoryUsage();
        const heapUsedPercent = (used.heapUsed / used.heapTotal) * 100;
        return {
          healthy: heapUsedPercent < 90,
          heapUsed: Math.round(used.heapUsed / 1024 / 1024),
          heapTotal: Math.round(used.heapTotal / 1024 / 1024),
          heapUsedPercent: Math.round(heapUsedPercent),
        };
      },
    });

    // CPU 상태 확인
    this.registerCheck({
      name: 'cpu',
      nameKr: 'CPU',
      type: 'warning',
      interval: 10000,
      timeout: 1000,
      check: async () => {
        const os = require('os');
        const cpus = os.cpus();
        const avgLoad = os.loadavg()[0] / cpus.length * 100;
        return {
          healthy: avgLoad < 80,
          loadAverage: Math.round(avgLoad),
          cores: cpus.length,
        };
      },
    });

    // 외부 서비스 상태 확인
    this.registerCheck({
      name: 'external-services',
      nameKr: '외부 서비스',
      type: 'important',
      interval: 60000,
      timeout: 15000,
      check: async () => {
        return { healthy: true, services: ['email', 'sms', 'payment'] };
      },
    });
  }

  registerCheck(check: HealthCheck): void {
    this.checks.set(check.name, check);
  }

  private startPeriodicChecks(): void {
    for (const [name, check] of this.checks) {
      setInterval(async () => {
        await this.runCheck(name);
      }, check.interval);
    }
  }

  async runCheck(name: string): Promise<HealthCheckResult> {
    const check = this.checks.get(name);
    if (!check) {
      throw new Error(`상태 확인을 찾을 수 없습니다: ${name}`);
    }

    const startTime = Date.now();

    try {
      const timeoutPromise = new Promise<never>((_, reject) => {
        setTimeout(() => reject(new Error('시간 초과')), check.timeout);
      });

      const result = await Promise.race([check.check(), timeoutPromise]);
      const duration = Date.now() - startTime;

      const checkResult: HealthCheckResult = {
        name: check.name,
        nameKr: check.nameKr,
        healthy: result.healthy,
        healthyKr: result.healthy ? '정상' : '비정상',
        duration,
        timestamp: new Date().toISOString(),
        details: result,
      };

      this.recordResult(checkResult);

      if (!result.healthy) {
        await this.handleUnhealthy(check, checkResult);
      }

      return checkResult;

    } catch (error) {
      const duration = Date.now() - startTime;

      const checkResult: HealthCheckResult = {
        name: check.name,
        nameKr: check.nameKr,
        healthy: false,
        healthyKr: '비정상',
        duration,
        timestamp: new Date().toISOString(),
        error: error instanceof Error ? error.message : String(error),
      };

      this.recordResult(checkResult);
      await this.handleUnhealthy(check, checkResult);

      return checkResult;
    }
  }

  async runAllChecks(): Promise<HealthStatus> {
    const results: HealthCheckResult[] = [];

    for (const name of this.checks.keys()) {
      results.push(await this.runCheck(name));
    }

    const healthy = results.every(r => r.healthy);
    const critical = results.filter(r => !r.healthy && this.checks.get(r.name)?.type === 'critical');

    return {
      status: healthy ? 'healthy' : (critical.length > 0 ? 'critical' : 'degraded'),
      statusKr: healthy ? '정상' : (critical.length > 0 ? '심각' : '저하'),
      timestamp: new Date().toISOString(),
      checks: results,
      summary: {
        total: results.length,
        healthy: results.filter(r => r.healthy).length,
        unhealthy: results.filter(r => !r.healthy).length,
      },
    };
  }

  private recordResult(result: HealthCheckResult): void {
    this.history.push(result);

    // 최근 1000개만 유지
    if (this.history.length > 1000) {
      this.history = this.history.slice(-1000);
    }
  }

  private async handleUnhealthy(check: HealthCheck, result: HealthCheckResult): Promise<void> {
    console.warn(`상태 이상: ${check.nameKr} - ${result.error || '비정상'}`);

    if (this.config.alerting) {
      const alert: HealthAlert = {
        id: require('crypto').randomUUID(),
        checkName: check.name,
        checkNameKr: check.nameKr,
        severity: check.type,
        severityKr: this.getSeverityKr(check.type),
        message: `${check.nameKr} 상태 이상: ${result.error || '비정상'}`,
        timestamp: new Date().toISOString(),
      };

      for (const handler of this.alertHandlers) {
        await handler(alert);
      }
    }
  }

  private getSeverityKr(severity: string): string {
    const map: Record<string, string> = {
      'critical': '심각',
      'important': '중요',
      'warning': '경고',
    };
    return map[severity] || severity;
  }

  registerAlertHandler(handler: AlertHandler): void {
    this.alertHandlers.push(handler);
  }

  getHistory(checkName?: string, limit: number = 100): HealthCheckResult[] {
    let filtered = this.history;

    if (checkName) {
      filtered = filtered.filter(r => r.name === checkName);
    }

    return filtered.slice(-limit);
  }

  getUptime(): UptimeStats {
    const now = new Date();
    const last24h = new Date(now.getTime() - 24 * 60 * 60 * 1000);

    const recentHistory = this.history.filter(r => new Date(r.timestamp) >= last24h);

    const byCheck: Record<string, { total: number; healthy: number }> = {};

    for (const result of recentHistory) {
      if (!byCheck[result.name]) {
        byCheck[result.name] = { total: 0, healthy: 0 };
      }
      byCheck[result.name].total++;
      if (result.healthy) {
        byCheck[result.name].healthy++;
      }
    }

    const uptimes: Record<string, number> = {};
    for (const [name, stats] of Object.entries(byCheck)) {
      uptimes[name] = stats.total > 0 ? (stats.healthy / stats.total) * 100 : 100;
    }

    const overallUptime = Object.values(uptimes).reduce((a, b) => a + b, 0) / Object.keys(uptimes).length || 100;

    return {
      overall: Math.round(overallUptime * 100) / 100,
      overallKr: `${Math.round(overallUptime * 100) / 100}%`,
      byCheck: uptimes,
      period: '24시간',
    };
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface MonitoringConfig {
  alerting: boolean;
  alertChannels: string[];
}

export interface HealthCheck {
  name: string;
  nameKr: string;
  type: 'critical' | 'important' | 'warning';
  interval: number;
  timeout: number;
  check: () => Promise<{ healthy: boolean; [key: string]: unknown }>;
}

export interface HealthCheckResult {
  name: string;
  nameKr: string;
  healthy: boolean;
  healthyKr: string;
  duration: number;
  timestamp: string;
  details?: Record<string, unknown>;
  error?: string;
}

export interface HealthStatus {
  status: 'healthy' | 'degraded' | 'critical';
  statusKr: string;
  timestamp: string;
  checks: HealthCheckResult[];
  summary: {
    total: number;
    healthy: number;
    unhealthy: number;
  };
}

export interface HealthAlert {
  id: string;
  checkName: string;
  checkNameKr: string;
  severity: string;
  severityKr: string;
  message: string;
  timestamp: string;
}

export interface UptimeStats {
  overall: number;
  overallKr: string;
  byCheck: Record<string, number>;
  period: string;
}

type AlertHandler = (alert: HealthAlert) => Promise<void>;
```

---

## 장 요약

이 장에서는 종합 구현 가이드에 대해 다루었습니다:

- **배포 관리자**: 단계별 배포 프로세스 및 롤백
- **마이그레이션 관리자**: 데이터베이스 스키마 버전 관리
- **성능 최적화기**: 쿼리 캐싱, 인덱스 분석
- **상태 모니터링**: 실시간 시스템 상태 확인 및 알림
- **한국 특화**: 서울 리전 배포, 한국어 로깅

---

**다음 장**: [미래 트렌드 - 기술 발전 전망](./09-future-trends.md)
