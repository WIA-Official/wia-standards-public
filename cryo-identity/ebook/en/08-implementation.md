# Chapter 8: Implementation Guide

## Overview

This chapter provides comprehensive guidance for implementing the WIA Cryo-Identity standard in production environments. It covers deployment strategies, migration planning, performance optimization, and operational best practices.

## Deployment Architecture

### System Components

```typescript
// Deployment configuration types
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

interface ObjectStorageConfig {
  type: 's3' | 'azure-blob' | 'gcs' | 'minio';
  bucket: string;
  encryption: boolean;
  versioning: boolean;
  replication: boolean;
}

interface MessagingConfig {
  type: 'rabbitmq' | 'kafka' | 'sqs';
  cluster: boolean;
  partitions?: number;
  replicationFactor?: number;
}

interface SecretsConfig {
  provider: 'aws-secrets' | 'azure-keyvault' | 'hashicorp-vault' | 'kubernetes';
  rotationEnabled: boolean;
  rotationDays: number;
}

interface ScalingConfig {
  autoScaling: boolean;
  minReplicas: number;
  maxReplicas: number;
  targetCpuUtilization: number;
  targetMemoryUtilization: number;
  scaleDownDelay: number;
}

interface MonitoringConfig {
  metrics: MetricsConfig;
  logging: LoggingConfig;
  tracing: TracingConfig;
  alerting: AlertingConfig;
}

// Production deployment manager
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
      // Step 1: Validate configuration
      await this.validateConfig();

      // Step 2: Prepare infrastructure
      await this.prepareInfrastructure();

      // Step 3: Deploy components in dependency order
      const orderedComponents = this.topologicalSort(this.config.components);

      for (const component of orderedComponents) {
        const result = await this.deployComponent(component);
        results.push(result);

        if (!result.success) {
          throw new DeploymentError(`Failed to deploy ${component.name}`, result.error);
        }
      }

      // Step 4: Run health checks
      await this.runHealthChecks();

      // Step 5: Update traffic routing
      await this.updateRouting();

      return {
        success: true,
        duration: Date.now() - startTime,
        components: results,
        environment: this.config.environment
      };

    } catch (error) {
      // Rollback on failure
      await this.rollback(results);

      return {
        success: false,
        duration: Date.now() - startTime,
        components: results,
        environment: this.config.environment,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  private async validateConfig(): Promise<void> {
    // Validate all required fields
    if (!this.config.environment) {
      throw new ConfigurationError('Environment not specified');
    }

    // Validate component dependencies
    for (const component of this.config.components) {
      for (const dep of component.dependencies) {
        const exists = this.config.components.some(c => c.name === dep);
        if (!exists) {
          throw new ConfigurationError(`Component ${component.name} depends on missing component ${dep}`);
        }
      }
    }

    // Validate infrastructure config
    await this.resourceManager.validateConfig();
  }

  private async prepareInfrastructure(): Promise<void> {
    // Initialize database
    await this.resourceManager.initializeDatabase();

    // Setup cache
    await this.resourceManager.initializeCache();

    // Setup object storage
    await this.resourceManager.initializeStorage();

    // Setup messaging
    await this.resourceManager.initializeMessaging();

    // Setup secrets
    await this.resourceManager.initializeSecrets();
  }

  private async deployComponent(component: ComponentConfig): Promise<ComponentDeployResult> {
    const startTime = Date.now();

    try {
      // Pull and validate image
      await this.pullImage(component);

      // Apply configuration
      await this.applyConfiguration(component);

      // Deploy replicas
      await this.deployReplicas(component);

      // Wait for readiness
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
        error: error instanceof Error ? error.message : 'Unknown error'
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
        throw new ConfigurationError(`Circular dependency detected for ${component.name}`);
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
    // Implementation for pulling container image
  }

  private async applyConfiguration(component: ComponentConfig): Promise<void> {
    // Apply environment-specific configuration
  }

  private async deployReplicas(component: ComponentConfig): Promise<void> {
    // Deploy the specified number of replicas
  }

  private async waitForReadiness(component: ComponentConfig): Promise<void> {
    const maxWait = 300000; // 5 minutes
    const startTime = Date.now();

    while (Date.now() - startTime < maxWait) {
      const healthy = await this.healthChecker.check(component);
      if (healthy) return;
      await this.sleep(5000);
    }

    throw new DeploymentError(`Component ${component.name} did not become ready`);
  }

  private async runHealthChecks(): Promise<void> {
    for (const component of this.config.components) {
      const healthy = await this.healthChecker.check(component);
      if (!healthy) {
        throw new DeploymentError(`Health check failed for ${component.name}`);
      }
    }
  }

  private async updateRouting(): Promise<void> {
    // Update load balancer or service mesh routing
  }

  private async rollback(results: ComponentDeployResult[]): Promise<void> {
    // Rollback in reverse order
    const deployed = results.filter(r => r.success);
    for (const result of deployed.reverse()) {
      const component = this.config.components.find(c => c.name === result.name);
      if (component) {
        await this.rollbackComponent(component);
      }
    }
  }

  private async rollbackComponent(component: ComponentConfig): Promise<void> {
    // Rollback to previous version
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

### Database Schema Migration

```typescript
// Database migration system
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
    // Core identity tables
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
      name: 'create_subject_relationships_table',
      checksum: 'stu901',
      up: async () => {
        await this.db.query(`
          CREATE TABLE subject_relationships (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            related_subject_id UUID NOT NULL REFERENCES subjects(id) ON DELETE CASCADE,
            relationship_type VARCHAR(50) NOT NULL,
            is_primary BOOLEAN DEFAULT FALSE,
            legal_authority TEXT,
            valid_from DATE NOT NULL,
            valid_until DATE,
            verified BOOLEAN DEFAULT FALSE,
            verified_at TIMESTAMP WITH TIME ZONE,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

            UNIQUE(subject_id, related_subject_id, relationship_type)
          );

          CREATE INDEX idx_relationships_subject ON subject_relationships(subject_id);
          CREATE INDEX idx_relationships_related ON subject_relationships(related_subject_id);
          CREATE INDEX idx_relationships_type ON subject_relationships(relationship_type);
        `);
      },
      down: async () => {
        await this.db.query('DROP TABLE IF EXISTS subject_relationships CASCADE');
      }
    });

    this.migrations.push({
      version: '008',
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

          -- Partition by month for better performance
          -- Production would use partitioning
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
        error: error instanceof Error ? error.message : 'Unknown error'
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

### Performance Optimization

```typescript
// Performance optimization strategies
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

  // Optimized subject lookup with caching
  async getSubject(id: string): Promise<Subject | null> {
    const cacheKey = `subject:${id}`;

    // Try cache first
    const cached = await this.cacheManager.get<Subject>(cacheKey);
    if (cached) {
      this.metricsCollector.increment('cache.hit', { type: 'subject' });
      return cached;
    }

    this.metricsCollector.increment('cache.miss', { type: 'subject' });

    // Fetch from database with optimized query
    const subject = await this.fetchSubjectOptimized(id);

    if (subject) {
      await this.cacheManager.set(cacheKey, subject, this.config.caching.ttl.subject);
    }

    return subject;
  }

  private async fetchSubjectOptimized(id: string): Promise<Subject | null> {
    // Use a single optimized query with joins
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

  // Batch verification processing
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
    // Parallel processing within batch
    return Promise.all(batch.map(r => this.processVerification(r)));
  }

  private async processVerification(request: VerificationRequest): Promise<VerificationResult> {
    // Implementation
    return {} as VerificationResult;
  }

  // Connection pool optimization
  createOptimizedPool(): DatabasePool {
    return new DatabasePool({
      min: this.config.connectionPooling.min,
      max: this.config.connectionPooling.max,
      acquireTimeoutMillis: this.config.connectionPooling.acquireTimeout,
      idleTimeoutMillis: this.config.connectionPooling.idleTimeout,
      reapIntervalMillis: this.config.connectionPooling.reapInterval,
      // Use read replicas for read-only queries
      readReplicaEnabled: this.config.queryOptimization.useReadReplicas
    });
  }

  // Query result pagination
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
    // Implementation would use actual database client
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

// Cache manager implementation
class CacheManager {
  private memoryCache: Map<string, CacheEntry> = new Map();
  private redisClient?: RedisClient;

  constructor(private config: CachingStrategy) {
    if (config.layers.some(l => l.type === 'redis')) {
      this.initializeRedis();
    }
  }

  private initializeRedis(): void {
    // Initialize Redis client
  }

  async get<T>(key: string): Promise<T | null> {
    // Try memory cache first
    const memoryEntry = this.memoryCache.get(key);
    if (memoryEntry && memoryEntry.expiresAt > Date.now()) {
      return memoryEntry.value as T;
    }

    // Try Redis
    if (this.redisClient) {
      const redisValue = await this.redisClient.get(key);
      if (redisValue) {
        const parsed = JSON.parse(redisValue) as T;
        // Populate memory cache
        this.memoryCache.set(key, {
          value: parsed,
          expiresAt: Date.now() + 60000 // 1 minute local cache
        });
        return parsed;
      }
    }

    return null;
  }

  async set<T>(key: string, value: T, ttlSeconds: number): Promise<void> {
    // Set in memory
    this.memoryCache.set(key, {
      value,
      expiresAt: Date.now() + (ttlSeconds * 1000)
    });

    // Set in Redis
    if (this.redisClient) {
      await this.redisClient.setex(key, ttlSeconds, JSON.stringify(value));
    }

    // Evict if over size limit
    this.evictIfNeeded();
  }

  async invalidate(key: string): Promise<void> {
    this.memoryCache.delete(key);
    if (this.redisClient) {
      await this.redisClient.del(key);
    }
  }

  async invalidatePattern(pattern: string): Promise<void> {
    // Invalidate matching keys
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
        // Sort by last access (simplified - would track access times)
        entries.sort((a, b) => a[1].expiresAt - b[1].expiresAt);
        break;
      case 'ttl':
        // Sort by expiration
        entries.sort((a, b) => a[1].expiresAt - b[1].expiresAt);
        break;
    }

    // Remove oldest entries
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

## Operational Procedures

### Health Monitoring

```typescript
// Comprehensive health monitoring
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
    // Database health
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

    // Cache health
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

    // Storage health
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

    // External services health
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
    const startTime = Date.now();

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
          details: { error: error instanceof Error ? error.message : 'Unknown error' }
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
    // Execute simple query to verify database connectivity
  }

  private async checkCache(): Promise<void> {
    // Execute simple cache operation
  }

  private async checkStorage(): Promise<void> {
    // Verify storage accessibility
  }

  private async checkExternalServices(): Promise<ServiceHealth[]> {
    // Check all external service integrations
    return [];
  }

  private timeout(ms: number): Promise<never> {
    return new Promise((_, reject) =>
      setTimeout(() => reject(new Error('Health check timeout')), ms)
    );
  }

  // Liveness probe - is the service alive?
  async liveness(): Promise<{ alive: boolean }> {
    return { alive: true };
  }

  // Readiness probe - is the service ready to accept traffic?
  async readiness(): Promise<{ ready: boolean; reason?: string }> {
    const health = await this.getHealth();

    if (health.status === 'unhealthy') {
      const unhealthy = health.components.filter(c => c.status === 'unhealthy');
      return {
        ready: false,
        reason: `Unhealthy components: ${unhealthy.map(c => c.name).join(', ')}`
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

## Summary

This chapter covered:

1. **Deployment Architecture**: Component configuration and deployment management
2. **Database Migration**: Schema versioning and migration system
3. **Performance Optimization**: Caching, query optimization, and batch processing
4. **Health Monitoring**: Comprehensive health checks and readiness probes

The next chapter covers future trends and emerging technologies in cryogenic identity management.
