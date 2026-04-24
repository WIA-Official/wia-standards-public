# 제8장: 구현 가이드

## 개요

이 장에서는 WIA-CRYO-IDENTITY 표준을 프로덕션 환경에서 구현하기 위한 포괄적인 가이드를 제공합니다. 배포 전략, 마이그레이션 계획, 성능 최적화, 그리고 운영 모범 사례를 다룹니다.

## 배포 아키텍처

### 시스템 컴포넌트

```typescript
// 배포 구성 타입
interface DeploymentConfig {
  environment: 'development' | 'staging' | 'production';
  region: string;
  components: ComponentConfig[];
  infrastructure: InfrastructureConfig;
  scaling: ScalingConfig;
  monitoring: MonitoringConfig;
}

interface ComponentConfig {
  name: string;
  type: 'api' | 'worker' | 'scheduler' | 'gateway';
  replicas: number;
  resources: ResourceRequirements;
  healthCheck: HealthCheckConfig;
  dependencies: string[];
}

interface ResourceRequirements {
  cpu: string;
  memory: string;
  storage?: string;
  gpu?: string;
}

interface HealthCheckConfig {
  endpoint: string;
  interval: number;
  timeout: number;
  healthyThreshold: number;
  unhealthyThreshold: number;
}

interface InfrastructureConfig {
  database: DatabaseConfig;
  cache: CacheConfig;
  storage: ObjectStorageConfig;
  messaging: MessagingConfig;
  secrets: SecretsConfig;
}

interface DatabaseConfig {
  type: 'postgresql' | 'mysql' | 'mongodb';
  host: string;
  port: number;
  database: string;
  replication: ReplicationConfig;
  backup: BackupConfig;
}

interface ReplicationConfig {
  enabled: boolean;
  readReplicas: number;
  syncMode: 'sync' | 'async';
  failoverAutomatic: boolean;
}

interface BackupConfig {
  enabled: boolean;
  schedule: string;
  retention: number;
  encryption: boolean;
  offsite: boolean;
}

interface CacheConfig {
  type: 'redis' | 'memcached';
  cluster: boolean;
  nodes: number;
  maxMemory: string;
  evictionPolicy: string;
}

interface ScalingConfig {
  autoScaling: boolean;
  minReplicas: number;
  maxReplicas: number;
  targetCpuUtilization: number;
  targetMemoryUtilization: number;
  scaleDownDelay: number;
}

// 프로덕션 배포 관리자
class DeploymentManager {
  private config: DeploymentConfig;
  private healthChecker: HealthChecker;
  private resourceManager: ResourceManager;

  constructor(config: DeploymentConfig) {
    this.config = config;
    this.healthChecker = new HealthChecker();
    this.resourceManager = new ResourceManager(config.infrastructure);
  }

  async deploy(): Promise<DeploymentResult> {
    const startTime = Date.now();
    const results: ComponentDeployResult[] = [];

    try {
      // 1단계: 구성 검증
      await this.validateConfig();

      // 2단계: 인프라 준비
      await this.prepareInfrastructure();

      // 3단계: 의존성 순서대로 컴포넌트 배포
      const orderedComponents = this.topologicalSort(this.config.components);

      for (const component of orderedComponents) {
        const result = await this.deployComponent(component);
        results.push(result);

        if (!result.success) {
          throw new DeploymentError(`${component.name} 배포 실패`, result.error);
        }
      }

      // 4단계: 헬스 체크 실행
      await this.runHealthChecks();

      // 5단계: 트래픽 라우팅 업데이트
      await this.updateRouting();

      return {
        success: true,
        duration: Date.now() - startTime,
        components: results,
        environment: this.config.environment
      };

    } catch (error) {
      // 실패 시 롤백
      await this.rollback(results);

      return {
        success: false,
        duration: Date.now() - startTime,
        components: results,
        environment: this.config.environment,
        error: error instanceof Error ? error.message : '알 수 없는 오류'
      };
    }
  }

  private async validateConfig(): Promise<void> {
    // 필수 필드 검증
    if (!this.config.environment) {
      throw new ConfigurationError('환경이 지정되지 않았습니다');
    }

    // 컴포넌트 의존성 검증
    for (const component of this.config.components) {
      for (const dep of component.dependencies) {
        const exists = this.config.components.some(c => c.name === dep);
        if (!exists) {
          throw new ConfigurationError(
            `컴포넌트 ${component.name}이 존재하지 않는 ${dep}에 의존합니다`
          );
        }
      }
    }

    // 인프라 구성 검증
    await this.resourceManager.validateConfig();
  }

  private async prepareInfrastructure(): Promise<void> {
    // 데이터베이스 초기화
    await this.resourceManager.initializeDatabase();

    // 캐시 설정
    await this.resourceManager.initializeCache();

    // 오브젝트 스토리지 설정
    await this.resourceManager.initializeStorage();

    // 메시징 설정
    await this.resourceManager.initializeMessaging();

    // 시크릿 설정
    await this.resourceManager.initializeSecrets();
  }

  private async deployComponent(component: ComponentConfig): Promise<ComponentDeployResult> {
    const startTime = Date.now();

    try {
      // 이미지 풀 및 검증
      await this.pullImage(component);

      // 구성 적용
      await this.applyConfiguration(component);

      // 레플리카 배포
      await this.deployReplicas(component);

      // 준비 상태 대기
      await this.waitForReadiness(component);

      return {
        name: component.name,
        success: true,
        duration: Date.now() - startTime,
        replicas: component.replicas
      };

    } catch (error) {
      return {
        name: component.name,
        success: false,
        duration: Date.now() - startTime,
        error: error instanceof Error ? error.message : '알 수 없는 오류'
      };
    }
  }

  private topologicalSort(components: ComponentConfig[]): ComponentConfig[] {
    const sorted: ComponentConfig[] = [];
    const visited = new Set<string>();
    const visiting = new Set<string>();

    const visit = (component: ComponentConfig) => {
      if (visited.has(component.name)) return;
      if (visiting.has(component.name)) {
        throw new ConfigurationError(`${component.name}에서 순환 의존성 감지됨`);
      }

      visiting.add(component.name);

      for (const depName of component.dependencies) {
        const dep = components.find(c => c.name === depName);
        if (dep) visit(dep);
      }

      visiting.delete(component.name);
      visited.add(component.name);
      sorted.push(component);
    };

    for (const component of components) {
      visit(component);
    }

    return sorted;
  }

  private async pullImage(component: ComponentConfig): Promise<void> {
    // 컨테이너 이미지 풀 구현
  }

  private async applyConfiguration(component: ComponentConfig): Promise<void> {
    // 환경별 구성 적용
  }

  private async deployReplicas(component: ComponentConfig): Promise<void> {
    // 지정된 수의 레플리카 배포
  }

  private async waitForReadiness(component: ComponentConfig): Promise<void> {
    const maxWait = 300000; // 5분
    const startTime = Date.now();

    while (Date.now() - startTime < maxWait) {
      const healthy = await this.healthChecker.check(component);
      if (healthy) return;
      await this.sleep(5000);
    }

    throw new DeploymentError(`컴포넌트 ${component.name}이 준비 상태가 되지 않았습니다`);
  }

  private async runHealthChecks(): Promise<void> {
    for (const component of this.config.components) {
      const healthy = await this.healthChecker.check(component);
      if (!healthy) {
        throw new DeploymentError(`${component.name}의 헬스 체크 실패`);
      }
    }
  }

  private async updateRouting(): Promise<void> {
    // 로드 밸런서 또는 서비스 메시 라우팅 업데이트
  }

  private async rollback(results: ComponentDeployResult[]): Promise<void> {
    // 역순으로 롤백
    const deployed = results.filter(r => r.success);
    for (const result of deployed.reverse()) {
      const component = this.config.components.find(c => c.name === result.name);
      if (component) {
        await this.rollbackComponent(component);
      }
    }
  }

  private async rollbackComponent(component: ComponentConfig): Promise<void> {
    // 이전 버전으로 롤백
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

interface DeploymentResult {
  success: boolean;
  duration: number;
  components: ComponentDeployResult[];
  environment: string;
  error?: string;
}

interface ComponentDeployResult {
  name: string;
  success: boolean;
  duration: number;
  replicas?: number;
  error?: string;
}

class DeploymentError extends Error {
  constructor(message: string, public cause?: unknown) {
    super(message);
    this.name = 'DeploymentError';
  }
}

class ConfigurationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'ConfigurationError';
  }
}
```

### 데이터베이스 스키마 마이그레이션

```typescript
// 데이터베이스 마이그레이션 시스템
interface Migration {
  version: string;
  name: string;
  up: () => Promise<void>;
  down: () => Promise<void>;
  checksum: string;
}

interface MigrationHistory {
  version: string;
  name: string;
  appliedAt: Date;
  executionTime: number;
  checksum: string;
}

class MigrationManager {
  private migrations: Migration[] = [];
  private db: DatabaseClient;

  constructor(db: DatabaseClient) {
    this.db = db;
    this.loadMigrations();
  }

  private loadMigrations(): void {
    // 핵심 신원 테이블
    this.migrations.push({
      version: '001',
      name: 'create_subjects_table',
      checksum: 'abc123',
      up: async () => {
        await this.db.query(`
          CREATE TABLE subjects (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            external_id VARCHAR(255) UNIQUE,
            type VARCHAR(50) NOT NULL CHECK (type IN ('individual', 'minor', 'incapacitated', 'posthumous')),
            status VARCHAR(50) NOT NULL DEFAULT 'pending' CHECK (status IN ('pending', 'active', 'suspended', 'inactive', 'deceased')),
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            created_by UUID,
            facility_id UUID NOT NULL
          );

          CREATE INDEX idx_subjects_type ON subjects(type);
          CREATE INDEX idx_subjects_status ON subjects(status);
          CREATE INDEX idx_subjects_facility ON subjects(facility_id);
          CREATE INDEX idx_subjects_created_at ON subjects(created_at);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS subjects CASCADE');
      }
    });

    this.migrations.push({
      version: '002',
      name: 'create_subject_identifiers_table',
      checksum: 'def456',
      up: async () => {
        await this.db.query(`
          CREATE TABLE subject_identifiers (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            identifier_type VARCHAR(50) NOT NULL,
            identifier_value_encrypted BYTEA NOT NULL,
            identifier_hash VARCHAR(255) NOT NULL,
            issuing_authority VARCHAR(255),
            issue_date DATE,
            expiration_date DATE,
            is_primary BOOLEAN DEFAULT FALSE,
            verified BOOLEAN DEFAULT FALSE,
            verified_at TIMESTAMP WITH TIME ZONE,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

            UNIQUE(subject_id, identifier_type, identifier_hash)
          );

          CREATE INDEX idx_identifiers_subject ON subject_identifiers(subject_id);
          CREATE INDEX idx_identifiers_type ON subject_identifiers(identifier_type);
          CREATE INDEX idx_identifiers_hash ON subject_identifiers(identifier_hash);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS subject_identifiers CASCADE');
      }
    });

    this.migrations.push({
      version: '003',
      name: 'create_subject_profiles_table',
      checksum: 'ghi789',
      up: async () => {
        await this.db.query(`
          CREATE TABLE subject_profiles (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            legal_name_encrypted BYTEA,
            date_of_birth_encrypted BYTEA,
            nationality VARCHAR(3),
            contact_info_encrypted BYTEA,
            emergency_contact_encrypted BYTEA,
            medical_notes_encrypted BYTEA,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

            UNIQUE(subject_id)
          );

          CREATE INDEX idx_profiles_subject ON subject_profiles(subject_id);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS subject_profiles CASCADE');
      }
    });

    this.migrations.push({
      version: '004',
      name: 'create_biometric_templates_table',
      checksum: 'jkl012',
      up: async () => {
        await this.db.query(`
          CREATE TABLE biometric_templates (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            modality VARCHAR(50) NOT NULL CHECK (modality IN ('fingerprint', 'facial', 'iris', 'voice', 'palm')),
            template_encrypted BYTEA NOT NULL,
            template_version VARCHAR(20) NOT NULL,
            quality_score DECIMAL(5,4),
            captured_at TIMESTAMP WITH TIME ZONE NOT NULL,
            captured_by UUID,
            device_id VARCHAR(255),
            is_active BOOLEAN DEFAULT TRUE,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

            UNIQUE(subject_id, modality, is_active) WHERE is_active = TRUE
          );

          CREATE INDEX idx_biometric_subject ON biometric_templates(subject_id);
          CREATE INDEX idx_biometric_modality ON biometric_templates(modality);
          CREATE INDEX idx_biometric_active ON biometric_templates(is_active);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS biometric_templates CASCADE');
      }
    });

    this.migrations.push({
      version: '005',
      name: 'create_verification_sessions_table',
      checksum: 'mno345',
      up: async () => {
        await this.db.query(`
          CREATE TABLE verification_sessions (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            verification_level VARCHAR(20) NOT NULL,
            required_methods TEXT[] NOT NULL,
            completed_methods TEXT[] DEFAULT '{}',
            status VARCHAR(20) NOT NULL DEFAULT 'pending',
            context JSONB NOT NULL,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
            completed_at TIMESTAMP WITH TIME ZONE
          );

          CREATE INDEX idx_verification_subject ON verification_sessions(subject_id);
          CREATE INDEX idx_verification_status ON verification_sessions(status);
          CREATE INDEX idx_verification_expires ON verification_sessions(expires_at);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS verification_sessions CASCADE');
      }
    });

    this.migrations.push({
      version: '006',
      name: 'create_verification_results_table',
      checksum: 'pqr678',
      up: async () => {
        await this.db.query(`
          CREATE TABLE verification_results (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            session_id UUID NOT NULL REFERENCES verification_sessions(id) ON DELETE CASCADE,
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            method VARCHAR(50) NOT NULL,
            status VARCHAR(20) NOT NULL,
            confidence DECIMAL(5,4) NOT NULL,
            evidence_hash VARCHAR(255),
            metadata JSONB,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            expires_at TIMESTAMP WITH TIME ZONE
          );

          CREATE INDEX idx_results_session ON verification_results(session_id);
          CREATE INDEX idx_results_subject ON verification_results(subject_id);
          CREATE INDEX idx_results_method ON verification_results(method);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS verification_results CASCADE');
      }
    });

    this.migrations.push({
      version: '007',
      name: 'create_audit_logs_table',
      checksum: 'vwx234',
      up: async () => {
        await this.db.query(`
          CREATE TABLE audit_logs (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            event_type VARCHAR(50) NOT NULL,
            severity VARCHAR(20) NOT NULL,
            actor_type VARCHAR(20) NOT NULL,
            actor_id VARCHAR(255) NOT NULL,
            actor_roles TEXT[],
            actor_ip VARCHAR(45),
            target_type VARCHAR(50),
            target_id VARCHAR(255),
            target_category VARCHAR(50),
            action VARCHAR(100) NOT NULL,
            result VARCHAR(20) NOT NULL,
            details JSONB,
            metadata JSONB,
            signature VARCHAR(255),
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );

          CREATE INDEX idx_audit_event_type ON audit_logs(event_type);
          CREATE INDEX idx_audit_actor ON audit_logs(actor_id);
          CREATE INDEX idx_audit_target ON audit_logs(target_id);
          CREATE INDEX idx_audit_created ON audit_logs(created_at);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS audit_logs CASCADE');
      }
    });
  }

  async migrate(): Promise<MigrationResult> {
    await this.ensureMigrationTable();
    const applied = await this.getAppliedMigrations();
    const pending = this.migrations.filter(m =>
      !applied.some(a => a.version === m.version)
    );

    const results: MigrationStepResult[] = [];

    for (const migration of pending) {
      const result = await this.applyMigration(migration);
      results.push(result);

      if (!result.success) {
        return {
          success: false,
          applied: results.filter(r => r.success).length,
          failed: migration.version,
          error: result.error
        };
      }
    }

    return {
      success: true,
      applied: results.length,
      pending: 0
    };
  }

  private async ensureMigrationTable(): Promise<void> {
    await this.db.query(`
      CREATE TABLE IF NOT EXISTS migration_history (
        version VARCHAR(50) PRIMARY KEY,
        name VARCHAR(255) NOT NULL,
        applied_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
        execution_time INTEGER,
        checksum VARCHAR(255) NOT NULL
      )
    `);
  }

  private async getAppliedMigrations(): Promise<MigrationHistory[]> {
    const result = await this.db.query(
      'SELECT * FROM migration_history ORDER BY version'
    );
    return result.rows;
  }

  private async applyMigration(migration: Migration): Promise<MigrationStepResult> {
    const startTime = Date.now();

    try {
      await this.db.query('BEGIN');
      await migration.up();

      await this.db.query(
        `INSERT INTO migration_history (version, name, execution_time, checksum)
         VALUES ($1, $2, $3, $4)`,
        [migration.version, migration.name, Date.now() - startTime, migration.checksum]
      );

      await this.db.query('COMMIT');

      return {
        version: migration.version,
        name: migration.name,
        success: true,
        duration: Date.now() - startTime
      };

    } catch (error) {
      await this.db.query('ROLLBACK');

      return {
        version: migration.version,
        name: migration.name,
        success: false,
        duration: Date.now() - startTime,
        error: error instanceof Error ? error.message : '알 수 없는 오류'
      };
    }
  }

  async rollback(targetVersion: string): Promise<MigrationResult> {
    const applied = await this.getAppliedMigrations();
    const toRollback = applied
      .filter(a => a.version > targetVersion)
      .reverse();

    for (const migration of toRollback) {
      const migrationDef = this.migrations.find(m => m.version === migration.version);
      if (migrationDef) {
        await migrationDef.down();
        await this.db.query(
          'DELETE FROM migration_history WHERE version = $1',
          [migration.version]
        );
      }
    }

    return {
      success: true,
      applied: 0,
      rolledBack: toRollback.length
    };
  }
}

interface MigrationResult {
  success: boolean;
  applied: number;
  pending?: number;
  rolledBack?: number;
  failed?: string;
  error?: string;
}

interface MigrationStepResult {
  version: string;
  name: string;
  success: boolean;
  duration: number;
  error?: string;
}
```

### 성능 최적화

```typescript
// 성능 최적화 전략
interface PerformanceConfig {
  caching: CachingStrategy;
  queryOptimization: QueryOptimizationConfig;
  connectionPooling: ConnectionPoolConfig;
  batchProcessing: BatchConfig;
}

interface CachingStrategy {
  enabled: boolean;
  ttl: Record<string, number>;
  maxSize: number;
  evictionPolicy: 'lru' | 'lfu' | 'ttl';
  layers: CacheLayer[];
}

interface CacheLayer {
  name: string;
  type: 'memory' | 'redis' | 'cdn';
  priority: number;
}

interface QueryOptimizationConfig {
  useReadReplicas: boolean;
  queryTimeout: number;
  slowQueryThreshold: number;
  explainAnalyze: boolean;
}

interface ConnectionPoolConfig {
  min: number;
  max: number;
  acquireTimeout: number;
  idleTimeout: number;
  reapInterval: number;
}

interface BatchConfig {
  enabled: boolean;
  batchSize: number;
  flushInterval: number;
  maxRetries: number;
}

class PerformanceOptimizer {
  private cacheManager: CacheManager;
  private queryAnalyzer: QueryAnalyzer;
  private metricsCollector: MetricsCollector;

  constructor(private config: PerformanceConfig) {
    this.cacheManager = new CacheManager(config.caching);
    this.queryAnalyzer = new QueryAnalyzer(config.queryOptimization);
    this.metricsCollector = new MetricsCollector();
  }

  // 캐싱을 사용한 최적화된 대상자 조회
  async getSubject(id: string): Promise<Subject | null> {
    const cacheKey = `subject:${id}`;

    // 먼저 캐시 시도
    const cached = await this.cacheManager.get<Subject>(cacheKey);
    if (cached) {
      this.metricsCollector.increment('cache.hit', { type: 'subject' });
      return cached;
    }

    this.metricsCollector.increment('cache.miss', { type: 'subject' });

    // 최적화된 쿼리로 데이터베이스에서 조회
    const subject = await this.fetchSubjectOptimized(id);

    if (subject) {
      await this.cacheManager.set(cacheKey, subject, this.config.caching.ttl.subject);
    }

    return subject;
  }

  private async fetchSubjectOptimized(id: string): Promise<Subject | null> {
    // 조인을 사용한 단일 최적화 쿼리
    const query = `
      SELECT
        s.*,
        json_agg(DISTINCT si.*) FILTER (WHERE si.id IS NOT NULL) as identifiers,
        row_to_json(sp.*) as profile,
        json_agg(DISTINCT bt.*) FILTER (WHERE bt.id IS NOT NULL AND bt.is_active = TRUE) as biometrics
      FROM subjects s
      LEFT JOIN subject_identifiers si ON s.id = si.subject_id
      LEFT JOIN subject_profiles sp ON s.id = sp.subject_id
      LEFT JOIN biometric_templates bt ON s.id = bt.subject_id
      WHERE s.id = $1
      GROUP BY s.id, sp.id
    `;

    const startTime = Date.now();
    const result = await this.executeQuery(query, [id]);
    const duration = Date.now() - startTime;

    this.metricsCollector.histogram('query.duration', duration, { query: 'getSubject' });

    if (duration > this.config.queryOptimization.slowQueryThreshold) {
      await this.queryAnalyzer.logSlowQuery(query, duration);
    }

    return result.rows[0] || null;
  }

  // 배치 인증 처리
  async processVerificationBatch(requests: VerificationRequest[]): Promise<VerificationResult[]> {
    if (!this.config.batchProcessing.enabled) {
      return Promise.all(requests.map(r => this.processVerification(r)));
    }

    const batches = this.chunk(requests, this.config.batchProcessing.batchSize);
    const results: VerificationResult[] = [];

    for (const batch of batches) {
      const batchResults = await this.processBatch(batch);
      results.push(...batchResults);
    }

    return results;
  }

  private async processBatch(batch: VerificationRequest[]): Promise<VerificationResult[]> {
    // 배치 내 병렬 처리
    return Promise.all(batch.map(r => this.processVerification(r)));
  }

  private async processVerification(request: VerificationRequest): Promise<VerificationResult> {
    // 구현
    return {} as VerificationResult;
  }

  // 연결 풀 최적화
  createOptimizedPool(): DatabasePool {
    return new DatabasePool({
      min: this.config.connectionPooling.min,
      max: this.config.connectionPooling.max,
      acquireTimeoutMillis: this.config.connectionPooling.acquireTimeout,
      idleTimeoutMillis: this.config.connectionPooling.idleTimeout,
      reapIntervalMillis: this.config.connectionPooling.reapInterval,
      // 읽기 전용 쿼리에 읽기 레플리카 사용
      readReplicaEnabled: this.config.queryOptimization.useReadReplicas
    });
  }

  // 쿼리 결과 페이지네이션
  async paginateResults<T>(
    query: string,
    params: any[],
    options: PaginationOptions
  ): Promise<PaginatedResult<T>> {
    const countQuery = `SELECT COUNT(*) FROM (${query}) AS total`;
    const paginatedQuery = `${query} LIMIT $${params.length + 1} OFFSET $${params.length + 2}`;

    const [countResult, dataResult] = await Promise.all([
      this.executeQuery(countQuery, params),
      this.executeQuery(paginatedQuery, [...params, options.limit, options.offset])
    ]);

    return {
      data: dataResult.rows,
      total: parseInt(countResult.rows[0].count),
      limit: options.limit,
      offset: options.offset,
      hasMore: options.offset + options.limit < parseInt(countResult.rows[0].count)
    };
  }

  private chunk<T>(array: T[], size: number): T[][] {
    const chunks: T[][] = [];
    for (let i = 0; i < array.length; i += size) {
      chunks.push(array.slice(i, i + size));
    }
    return chunks;
  }

  private async executeQuery(query: string, params: any[]): Promise<QueryResult> {
    // 실제 데이터베이스 클라이언트 사용
    return { rows: [] } as QueryResult;
  }
}

interface PaginationOptions {
  limit: number;
  offset: number;
}

interface PaginatedResult<T> {
  data: T[];
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

// 캐시 관리자 구현
class CacheManager {
  private memoryCache: Map<string, CacheEntry> = new Map();
  private redisClient?: RedisClient;

  constructor(private config: CachingStrategy) {
    if (config.layers.some(l => l.type === 'redis')) {
      this.initializeRedis();
    }
  }

  private initializeRedis(): void {
    // Redis 클라이언트 초기화
  }

  async get<T>(key: string): Promise<T | null> {
    // 먼저 메모리 캐시 시도
    const memoryEntry = this.memoryCache.get(key);
    if (memoryEntry && memoryEntry.expiresAt > Date.now()) {
      return memoryEntry.value as T;
    }

    // Redis 시도
    if (this.redisClient) {
      const redisValue = await this.redisClient.get(key);
      if (redisValue) {
        const parsed = JSON.parse(redisValue) as T;
        // 메모리 캐시 채우기
        this.memoryCache.set(key, {
          value: parsed,
          expiresAt: Date.now() + 60000 // 1분 로컬 캐시
        });
        return parsed;
      }
    }

    return null;
  }

  async set<T>(key: string, value: T, ttlSeconds: number): Promise<void> {
    // 메모리에 설정
    this.memoryCache.set(key, {
      value,
      expiresAt: Date.now() + (ttlSeconds * 1000)
    });

    // Redis에 설정
    if (this.redisClient) {
      await this.redisClient.setex(key, ttlSeconds, JSON.stringify(value));
    }

    // 크기 제한 초과 시 제거
    this.evictIfNeeded();
  }

  async invalidate(key: string): Promise<void> {
    this.memoryCache.delete(key);
    if (this.redisClient) {
      await this.redisClient.del(key);
    }
  }

  async invalidatePattern(pattern: string): Promise<void> {
    // 일치하는 키 무효화
    for (const key of this.memoryCache.keys()) {
      if (key.match(pattern)) {
        this.memoryCache.delete(key);
      }
    }

    if (this.redisClient) {
      const keys = await this.redisClient.keys(pattern);
      if (keys.length > 0) {
        await this.redisClient.del(...keys);
      }
    }
  }

  private evictIfNeeded(): void {
    if (this.memoryCache.size <= this.config.maxSize) return;

    const entries = Array.from(this.memoryCache.entries());

    switch (this.config.evictionPolicy) {
      case 'lru':
        // 마지막 접근 기준 정렬 (간소화 - 접근 시간 추적 필요)
        entries.sort((a, b) => a[1].expiresAt - b[1].expiresAt);
        break;
      case 'ttl':
        // 만료 기준 정렬
        entries.sort((a, b) => a[1].expiresAt - b[1].expiresAt);
        break;
    }

    // 오래된 항목 제거
    const toRemove = entries.slice(0, entries.length - this.config.maxSize);
    for (const [key] of toRemove) {
      this.memoryCache.delete(key);
    }
  }
}

interface CacheEntry {
  value: unknown;
  expiresAt: number;
}
```

## 운영 절차

### 상태 모니터링

```typescript
// 포괄적인 상태 모니터링
interface HealthStatus {
  status: 'healthy' | 'degraded' | 'unhealthy';
  components: ComponentHealth[];
  timestamp: Date;
  version: string;
  uptime: number;
}

interface ComponentHealth {
  name: string;
  status: 'healthy' | 'degraded' | 'unhealthy';
  latency?: number;
  lastCheck: Date;
  details?: Record<string, any>;
}

class HealthMonitoringService {
  private checks: Map<string, HealthCheck> = new Map();
  private lastStatus?: HealthStatus;

  constructor(private config: MonitoringConfig) {
    this.registerDefaultChecks();
  }

  private registerDefaultChecks(): void {
    // 데이터베이스 상태
    this.checks.set('database', {
      name: 'database',
      interval: 30000,
      timeout: 5000,
      check: async () => {
        const start = Date.now();
        await this.checkDatabase();
        return {
          status: 'healthy',
          latency: Date.now() - start
        };
      }
    });

    // 캐시 상태
    this.checks.set('cache', {
      name: 'cache',
      interval: 30000,
      timeout: 3000,
      check: async () => {
        const start = Date.now();
        await this.checkCache();
        return {
          status: 'healthy',
          latency: Date.now() - start
        };
      }
    });

    // 스토리지 상태
    this.checks.set('storage', {
      name: 'storage',
      interval: 60000,
      timeout: 10000,
      check: async () => {
        const start = Date.now();
        await this.checkStorage();
        return {
          status: 'healthy',
          latency: Date.now() - start
        };
      }
    });

    // 외부 서비스 상태
    this.checks.set('external-services', {
      name: 'external-services',
      interval: 60000,
      timeout: 30000,
      check: async () => {
        const results = await this.checkExternalServices();
        const unhealthy = results.filter(r => r.status !== 'healthy');
        return {
          status: unhealthy.length === 0 ? 'healthy' :
                  unhealthy.length < results.length / 2 ? 'degraded' : 'unhealthy',
          details: { services: results }
        };
      }
    });
  }

  async getHealth(): Promise<HealthStatus> {
    const components: ComponentHealth[] = [];

    for (const [name, check] of this.checks) {
      try {
        const result = await Promise.race([
          check.check(),
          this.timeout(check.timeout)
        ]);

        components.push({
          name,
          status: result.status,
          latency: result.latency,
          lastCheck: new Date(),
          details: result.details
        });
      } catch (error) {
        components.push({
          name,
          status: 'unhealthy',
          lastCheck: new Date(),
          details: { error: error instanceof Error ? error.message : '알 수 없는 오류' }
        });
      }
    }

    const unhealthyCount = components.filter(c => c.status === 'unhealthy').length;
    const degradedCount = components.filter(c => c.status === 'degraded').length;

    let overallStatus: 'healthy' | 'degraded' | 'unhealthy';
    if (unhealthyCount > 0) {
      overallStatus = 'unhealthy';
    } else if (degradedCount > 0) {
      overallStatus = 'degraded';
    } else {
      overallStatus = 'healthy';
    }

    this.lastStatus = {
      status: overallStatus,
      components,
      timestamp: new Date(),
      version: process.env.APP_VERSION || '1.0.0',
      uptime: process.uptime()
    };

    return this.lastStatus;
  }

  private async checkDatabase(): Promise<void> {
    // 데이터베이스 연결 확인을 위한 간단한 쿼리 실행
  }

  private async checkCache(): Promise<void> {
    // 간단한 캐시 작업 실행
  }

  private async checkStorage(): Promise<void> {
    // 스토리지 접근 확인
  }

  private async checkExternalServices(): Promise<ServiceHealth[]> {
    // 모든 외부 서비스 통합 확인
    return [];
  }

  private timeout(ms: number): Promise<never> {
    return new Promise((_, reject) =>
      setTimeout(() => reject(new Error('상태 체크 시간 초과')), ms)
    );
  }

  // Liveness 프로브 - 서비스가 살아있는지?
  async liveness(): Promise<{ alive: boolean }> {
    return { alive: true };
  }

  // Readiness 프로브 - 서비스가 트래픽을 받을 준비가 되었는지?
  async readiness(): Promise<{ ready: boolean; reason?: string }> {
    const health = await this.getHealth();

    if (health.status === 'unhealthy') {
      const unhealthy = health.components.filter(c => c.status === 'unhealthy');
      return {
        ready: false,
        reason: `비정상 컴포넌트: ${unhealthy.map(c => c.name).join(', ')}`
      };
    }

    return { ready: true };
  }
}

interface HealthCheck {
  name: string;
  interval: number;
  timeout: number;
  check: () => Promise<{ status: 'healthy' | 'degraded' | 'unhealthy'; latency?: number; details?: Record<string, any> }>;
}

interface ServiceHealth {
  name: string;
  status: 'healthy' | 'degraded' | 'unhealthy';
  latency?: number;
}
```

## 요약

이 장에서 다룬 내용:

1. **배포 아키텍처**: 컴포넌트 구성 및 배포 관리
2. **데이터베이스 마이그레이션**: 스키마 버전 관리 및 마이그레이션 시스템
3. **성능 최적화**: 캐싱, 쿼리 최적화, 배치 처리
4. **상태 모니터링**: 포괄적인 상태 체크 및 준비 프로브

다음 장에서는 냉동보존 신원 관리의 미래 동향과 신흥 기술을 다룹니다.
