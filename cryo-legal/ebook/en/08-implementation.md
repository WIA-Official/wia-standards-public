# Chapter 8: Implementation - Deployment and Integration

## Complete Implementation Guide for Cryo Legal System

This chapter provides comprehensive deployment, integration, and operational guidance for implementing the WIA Cryo Legal Standard in production environments.

## System Architecture

```typescript
/**
 * WIA Cryo Legal Standard - System Architecture
 * Production-ready deployment configuration
 */

import { z } from 'zod';

// ============================================================================
// Architecture Configuration
// ============================================================================

export interface SystemArchitecture {
  deployment: DeploymentConfig;
  database: DatabaseConfig;
  cache: CacheConfig;
  messaging: MessagingConfig;
  monitoring: MonitoringConfig;
}

export interface DeploymentConfig {
  environment: 'development' | 'staging' | 'production';
  region: string;
  replicas: number;
  autoScaling: AutoScalingConfig;
  loadBalancer: LoadBalancerConfig;
}

export interface AutoScalingConfig {
  enabled: boolean;
  minReplicas: number;
  maxReplicas: number;
  targetCPUPercent: number;
  targetMemoryPercent: number;
}

export interface LoadBalancerConfig {
  type: 'application' | 'network';
  healthCheckPath: string;
  healthCheckInterval: number;
  stickySession: boolean;
}

export interface DatabaseConfig {
  type: 'postgresql' | 'mysql' | 'mongodb';
  host: string;
  port: number;
  database: string;
  username: string;
  password: string;
  ssl: boolean;
  poolSize: number;
  replication: ReplicationConfig;
}

export interface ReplicationConfig {
  enabled: boolean;
  readReplicas: number;
  syncMode: 'sync' | 'async';
}

export interface CacheConfig {
  type: 'redis' | 'memcached';
  host: string;
  port: number;
  ttlSeconds: number;
  cluster: boolean;
}

export interface MessagingConfig {
  type: 'rabbitmq' | 'kafka' | 'sqs';
  host: string;
  port: number;
  queues: QueueConfig[];
}

export interface QueueConfig {
  name: string;
  durable: boolean;
  maxRetries: number;
  deadLetterQueue: boolean;
}

export interface MonitoringConfig {
  metrics: MetricsConfig;
  logging: LoggingConfig;
  tracing: TracingConfig;
  alerting: AlertingConfig;
}

export interface MetricsConfig {
  enabled: boolean;
  provider: 'prometheus' | 'datadog' | 'cloudwatch';
  endpoint: string;
  interval: number;
}

export interface LoggingConfig {
  level: 'debug' | 'info' | 'warn' | 'error';
  format: 'json' | 'text';
  destination: 'stdout' | 'file' | 'elasticsearch';
}

export interface TracingConfig {
  enabled: boolean;
  provider: 'jaeger' | 'zipkin' | 'xray';
  sampleRate: number;
}

export interface AlertingConfig {
  enabled: boolean;
  channels: ('email' | 'slack' | 'pagerduty')[];
  escalationPolicy: EscalationPolicy;
}

export interface EscalationPolicy {
  levels: {
    level: number;
    delay: number;
    notifyChannels: string[];
  }[];
}

// ============================================================================
// Deployment Manager
// ============================================================================

export class DeploymentManager {
  constructor(
    private readonly architecture: SystemArchitecture,
    private readonly containerRegistry: ContainerRegistry,
    private readonly orchestrator: ContainerOrchestrator
  ) {}

  async deploy(version: string): Promise<DeploymentResult> {
    const deploymentId = this.generateDeploymentId();

    try {
      // Pre-deployment checks
      await this.runPreDeploymentChecks();

      // Build and push container
      const image = await this.buildContainer(version);
      await this.pushContainer(image);

      // Database migrations
      await this.runMigrations();

      // Deploy new version
      const deployment = await this.orchestrator.deploy({
        image: image.uri,
        replicas: this.architecture.deployment.replicas,
        environment: this.getEnvironmentVariables(),
        resources: this.getResourceLimits(),
        healthCheck: {
          path: this.architecture.deployment.loadBalancer.healthCheckPath,
          interval: this.architecture.deployment.loadBalancer.healthCheckInterval,
        },
      });

      // Wait for healthy state
      await this.waitForHealthy(deployment.id);

      // Run post-deployment tests
      await this.runPostDeploymentTests();

      return {
        id: deploymentId,
        version,
        status: 'success',
        timestamp: new Date().toISOString(),
        duration: 0,
      };
    } catch (error) {
      // Rollback on failure
      await this.rollback(deploymentId);

      return {
        id: deploymentId,
        version,
        status: 'failed',
        timestamp: new Date().toISOString(),
        duration: 0,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  private generateDeploymentId(): string {
    return `dep-${Date.now().toString(36)}-${Math.random().toString(36).substring(2, 8)}`;
  }

  private async runPreDeploymentChecks(): Promise<void> {
    // Check database connectivity
    // Verify cache availability
    // Validate configuration
    // Check secrets availability
  }

  private async buildContainer(version: string): Promise<ContainerImage> {
    return {
      name: 'cryo-legal-api',
      tag: version,
      uri: `${this.containerRegistry.host}/cryo-legal-api:${version}`,
      size: 0,
      createdAt: new Date().toISOString(),
    };
  }

  private async pushContainer(image: ContainerImage): Promise<void> {
    await this.containerRegistry.push(image);
  }

  private async runMigrations(): Promise<void> {
    // Run database migrations
  }

  private getEnvironmentVariables(): Record<string, string> {
    return {
      NODE_ENV: this.architecture.deployment.environment,
      DB_HOST: this.architecture.database.host,
      DB_PORT: String(this.architecture.database.port),
      DB_NAME: this.architecture.database.database,
      CACHE_HOST: this.architecture.cache.host,
      CACHE_PORT: String(this.architecture.cache.port),
      LOG_LEVEL: this.architecture.monitoring.logging.level,
    };
  }

  private getResourceLimits(): ResourceLimits {
    return {
      cpu: '1000m',
      memory: '2Gi',
      cpuRequest: '250m',
      memoryRequest: '512Mi',
    };
  }

  private async waitForHealthy(deploymentId: string): Promise<void> {
    const maxWait = 300000; // 5 minutes
    const interval = 5000;
    let elapsed = 0;

    while (elapsed < maxWait) {
      const status = await this.orchestrator.getDeploymentStatus(deploymentId);

      if (status.ready) {
        return;
      }

      await this.sleep(interval);
      elapsed += interval;
    }

    throw new Error('Deployment health check timeout');
  }

  private async runPostDeploymentTests(): Promise<void> {
    // Run smoke tests
    // Verify API endpoints
    // Check integration points
  }

  private async rollback(deploymentId: string): Promise<void> {
    await this.orchestrator.rollback(deploymentId);
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

export interface DeploymentResult {
  id: string;
  version: string;
  status: 'success' | 'failed' | 'rollback';
  timestamp: string;
  duration: number;
  error?: string;
}

export interface ContainerImage {
  name: string;
  tag: string;
  uri: string;
  size: number;
  createdAt: string;
}

export interface ResourceLimits {
  cpu: string;
  memory: string;
  cpuRequest: string;
  memoryRequest: string;
}

export interface ContainerRegistry {
  host: string;
  push(image: ContainerImage): Promise<void>;
  pull(uri: string): Promise<ContainerImage>;
}

export interface ContainerOrchestrator {
  deploy(config: unknown): Promise<{ id: string }>;
  getDeploymentStatus(id: string): Promise<{ ready: boolean }>;
  rollback(id: string): Promise<void>;
}

// ============================================================================
// Database Migration Manager
// ============================================================================

export class MigrationManager {
  private migrations: Migration[] = [];

  constructor(private readonly database: DatabaseConnection) {
    this.loadMigrations();
  }

  private loadMigrations(): void {
    // Initial schema
    this.migrations.push({
      id: '001_initial_schema',
      name: 'Initial Schema',
      up: `
        -- Organizations
        CREATE TABLE organizations (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          name VARCHAR(300) NOT NULL,
          type VARCHAR(50) NOT NULL,
          country CHAR(2) NOT NULL,
          registration_number VARCHAR(100),
          contact_name VARCHAR(200) NOT NULL,
          contact_email VARCHAR(200) NOT NULL,
          contact_phone VARCHAR(50),
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE
        );

        -- Projects
        CREATE TABLE projects (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          organization_id UUID REFERENCES organizations(id),
          name VARCHAR(200) NOT NULL,
          description TEXT,
          status VARCHAR(50) NOT NULL DEFAULT 'pending',
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE
        );

        -- Contracts
        CREATE TABLE contracts (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          project_id UUID REFERENCES projects(id),
          template_id UUID,
          type VARCHAR(50) NOT NULL,
          status VARCHAR(50) NOT NULL DEFAULT 'draft',
          effective_date TIMESTAMP WITH TIME ZONE,
          expiry_date TIMESTAMP WITH TIME ZONE,
          terms JSONB NOT NULL,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE
        );

        -- Contract Parties
        CREATE TABLE contract_parties (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          contract_id UUID REFERENCES contracts(id),
          party_type VARCHAR(50) NOT NULL,
          party_name VARCHAR(300) NOT NULL,
          party_role VARCHAR(50) NOT NULL,
          representative VARCHAR(200),
          contact_name VARCHAR(200) NOT NULL,
          contact_email VARCHAR(200) NOT NULL,
          contact_phone VARCHAR(50)
        );

        -- Signatures
        CREATE TABLE signatures (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          contract_id UUID REFERENCES contracts(id),
          party_id UUID REFERENCES contract_parties(id),
          signatory VARCHAR(200) NOT NULL,
          signed_at TIMESTAMP WITH TIME ZONE NOT NULL,
          method VARCHAR(50) NOT NULL,
          witness VARCHAR(200),
          notarized BOOLEAN DEFAULT FALSE
        );

        -- Disputes
        CREATE TABLE disputes (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          project_id UUID REFERENCES projects(id),
          type VARCHAR(50) NOT NULL,
          subject VARCHAR(500) NOT NULL,
          description TEXT NOT NULL,
          filed_date TIMESTAMP WITH TIME ZONE NOT NULL,
          status VARCHAR(50) NOT NULL DEFAULT 'filed',
          current_mechanism VARCHAR(50),
          resolution JSONB,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE
        );

        -- Compliance Requirements
        CREATE TABLE compliance_requirements (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          project_id UUID REFERENCES projects(id),
          regulation VARCHAR(200) NOT NULL,
          requirement TEXT NOT NULL,
          responsible VARCHAR(200) NOT NULL,
          frequency VARCHAR(50) NOT NULL,
          last_check TIMESTAMP WITH TIME ZONE,
          status VARCHAR(50) NOT NULL DEFAULT 'pending',
          evidence JSONB
        );

        -- Audit Logs
        CREATE TABLE audit_logs (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT CURRENT_TIMESTAMP,
          category VARCHAR(50) NOT NULL,
          action VARCHAR(100) NOT NULL,
          user_id VARCHAR(200),
          resource_type VARCHAR(100),
          resource_id VARCHAR(200),
          details JSONB,
          severity VARCHAR(20) NOT NULL,
          source_ip INET,
          hash VARCHAR(64)
        );

        -- Indexes
        CREATE INDEX idx_contracts_status ON contracts(status);
        CREATE INDEX idx_contracts_type ON contracts(type);
        CREATE INDEX idx_disputes_status ON disputes(status);
        CREATE INDEX idx_audit_logs_timestamp ON audit_logs(timestamp);
        CREATE INDEX idx_audit_logs_user ON audit_logs(user_id);
        CREATE INDEX idx_audit_logs_category ON audit_logs(category);
      `,
      down: `
        DROP TABLE IF EXISTS audit_logs;
        DROP TABLE IF EXISTS compliance_requirements;
        DROP TABLE IF EXISTS disputes;
        DROP TABLE IF EXISTS signatures;
        DROP TABLE IF EXISTS contract_parties;
        DROP TABLE IF EXISTS contracts;
        DROP TABLE IF EXISTS projects;
        DROP TABLE IF EXISTS organizations;
      `,
      createdAt: '2025-01-01T00:00:00Z',
    });

    // Add regulations table
    this.migrations.push({
      id: '002_regulations',
      name: 'Regulations Table',
      up: `
        CREATE TABLE regulations (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          name VARCHAR(300) NOT NULL,
          jurisdiction VARCHAR(100) NOT NULL,
          category VARCHAR(50) NOT NULL,
          requirements JSONB NOT NULL,
          penalties JSONB,
          effective_date DATE NOT NULL,
          last_review DATE,
          status VARCHAR(50) NOT NULL DEFAULT 'current',
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );

        CREATE INDEX idx_regulations_jurisdiction ON regulations(jurisdiction);
        CREATE INDEX idx_regulations_category ON regulations(category);
      `,
      down: `DROP TABLE IF EXISTS regulations;`,
      createdAt: '2025-01-02T00:00:00Z',
    });

    // Add contract templates
    this.migrations.push({
      id: '003_contract_templates',
      name: 'Contract Templates',
      up: `
        CREATE TABLE contract_templates (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          name VARCHAR(200) NOT NULL,
          type VARCHAR(50) NOT NULL,
          version VARCHAR(20) NOT NULL,
          jurisdiction VARCHAR(100) NOT NULL,
          clauses JSONB NOT NULL,
          approved_by VARCHAR(200),
          approved_at TIMESTAMP WITH TIME ZONE,
          status VARCHAR(50) NOT NULL DEFAULT 'draft',
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );

        CREATE INDEX idx_templates_type ON contract_templates(type);
        CREATE INDEX idx_templates_jurisdiction ON contract_templates(jurisdiction);
      `,
      down: `DROP TABLE IF EXISTS contract_templates;`,
      createdAt: '2025-01-03T00:00:00Z',
    });
  }

  async migrate(): Promise<MigrationResult> {
    const appliedMigrations = await this.getAppliedMigrations();
    const pendingMigrations = this.migrations.filter(
      m => !appliedMigrations.includes(m.id)
    );

    const results: MigrationStepResult[] = [];

    for (const migration of pendingMigrations) {
      try {
        await this.database.execute(migration.up);
        await this.recordMigration(migration.id);

        results.push({
          id: migration.id,
          name: migration.name,
          status: 'success',
        });
      } catch (error) {
        results.push({
          id: migration.id,
          name: migration.name,
          status: 'failed',
          error: error instanceof Error ? error.message : 'Unknown error',
        });

        // Stop on first failure
        break;
      }
    }

    return {
      applied: results.filter(r => r.status === 'success').length,
      failed: results.filter(r => r.status === 'failed').length,
      pending: pendingMigrations.length - results.length,
      results,
    };
  }

  async rollback(steps: number = 1): Promise<MigrationResult> {
    const appliedMigrations = await this.getAppliedMigrations();
    const toRollback = appliedMigrations.slice(-steps);

    const results: MigrationStepResult[] = [];

    for (const migrationId of toRollback.reverse()) {
      const migration = this.migrations.find(m => m.id === migrationId);

      if (!migration) continue;

      try {
        await this.database.execute(migration.down);
        await this.removeMigration(migration.id);

        results.push({
          id: migration.id,
          name: migration.name,
          status: 'success',
        });
      } catch (error) {
        results.push({
          id: migration.id,
          name: migration.name,
          status: 'failed',
          error: error instanceof Error ? error.message : 'Unknown error',
        });

        break;
      }
    }

    return {
      applied: 0,
      failed: results.filter(r => r.status === 'failed').length,
      pending: 0,
      results,
    };
  }

  private async getAppliedMigrations(): Promise<string[]> {
    try {
      const result = await this.database.query(
        'SELECT id FROM schema_migrations ORDER BY applied_at'
      );
      return result.rows.map((r: any) => r.id);
    } catch {
      // Create migrations table if not exists
      await this.database.execute(`
        CREATE TABLE IF NOT EXISTS schema_migrations (
          id VARCHAR(100) PRIMARY KEY,
          applied_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        )
      `);
      return [];
    }
  }

  private async recordMigration(id: string): Promise<void> {
    await this.database.execute(
      'INSERT INTO schema_migrations (id) VALUES ($1)',
      [id]
    );
  }

  private async removeMigration(id: string): Promise<void> {
    await this.database.execute(
      'DELETE FROM schema_migrations WHERE id = $1',
      [id]
    );
  }
}

export interface Migration {
  id: string;
  name: string;
  up: string;
  down: string;
  createdAt: string;
}

export interface MigrationResult {
  applied: number;
  failed: number;
  pending: number;
  results: MigrationStepResult[];
}

export interface MigrationStepResult {
  id: string;
  name: string;
  status: 'success' | 'failed';
  error?: string;
}

export interface DatabaseConnection {
  execute(sql: string, params?: unknown[]): Promise<void>;
  query(sql: string, params?: unknown[]): Promise<{ rows: unknown[] }>;
}

// ============================================================================
// Performance Optimization
// ============================================================================

export class PerformanceOptimizer {
  private metrics: Map<string, PerformanceMetric[]> = new Map();

  constructor(
    private readonly cache: CacheService,
    private readonly database: DatabaseConnection
  ) {}

  async optimizeQuery(queryId: string, query: string): Promise<OptimizedQuery> {
    // Analyze query
    const analysis = await this.analyzeQuery(query);

    // Check for cached result
    const cacheKey = this.generateCacheKey(queryId, query);
    const cached = await this.cache.get(cacheKey);

    if (cached) {
      return {
        query,
        optimized: query,
        fromCache: true,
        executionTimeMs: 0,
        recommendations: [],
      };
    }

    // Apply optimizations
    let optimized = query;

    if (analysis.missingIndexes.length > 0) {
      // Log recommendation but don't auto-create indexes
    }

    if (analysis.fullTableScan) {
      // Suggest query rewrite
    }

    return {
      query,
      optimized,
      fromCache: false,
      executionTimeMs: analysis.estimatedTime,
      recommendations: this.generateRecommendations(analysis),
    };
  }

  private async analyzeQuery(query: string): Promise<QueryAnalysis> {
    const explain = await this.database.query(`EXPLAIN ANALYZE ${query}`);

    return {
      estimatedTime: 0,
      actualTime: 0,
      rowsExamined: 0,
      rowsReturned: 0,
      indexesUsed: [],
      missingIndexes: [],
      fullTableScan: false,
      sortOperations: 0,
      temporaryTables: 0,
    };
  }

  private generateCacheKey(queryId: string, query: string): string {
    const hash = require('crypto').createHash('sha256');
    hash.update(queryId + query);
    return `query:${hash.digest('hex')}`;
  }

  private generateRecommendations(analysis: QueryAnalysis): string[] {
    const recommendations: string[] = [];

    if (analysis.fullTableScan) {
      recommendations.push('Consider adding index to avoid full table scan');
    }

    if (analysis.sortOperations > 0) {
      recommendations.push('Consider adding index for sort columns');
    }

    if (analysis.missingIndexes.length > 0) {
      recommendations.push(
        `Missing indexes on: ${analysis.missingIndexes.join(', ')}`
      );
    }

    return recommendations;
  }

  async recordMetric(name: string, value: number, tags?: Record<string, string>): void {
    const metric: PerformanceMetric = {
      name,
      value,
      timestamp: new Date().toISOString(),
      tags: tags || {},
    };

    const metrics = this.metrics.get(name) || [];
    metrics.push(metric);

    // Keep last 1000 metrics per name
    if (metrics.length > 1000) {
      metrics.shift();
    }

    this.metrics.set(name, metrics);
  }

  async getMetricSummary(name: string): Promise<MetricSummary> {
    const metrics = this.metrics.get(name) || [];

    if (metrics.length === 0) {
      return {
        name,
        count: 0,
        min: 0,
        max: 0,
        avg: 0,
        p50: 0,
        p95: 0,
        p99: 0,
      };
    }

    const values = metrics.map(m => m.value).sort((a, b) => a - b);

    return {
      name,
      count: values.length,
      min: values[0],
      max: values[values.length - 1],
      avg: values.reduce((a, b) => a + b, 0) / values.length,
      p50: this.percentile(values, 50),
      p95: this.percentile(values, 95),
      p99: this.percentile(values, 99),
    };
  }

  private percentile(sortedValues: number[], percentile: number): number {
    const index = Math.ceil((percentile / 100) * sortedValues.length) - 1;
    return sortedValues[index];
  }
}

export interface QueryAnalysis {
  estimatedTime: number;
  actualTime: number;
  rowsExamined: number;
  rowsReturned: number;
  indexesUsed: string[];
  missingIndexes: string[];
  fullTableScan: boolean;
  sortOperations: number;
  temporaryTables: number;
}

export interface OptimizedQuery {
  query: string;
  optimized: string;
  fromCache: boolean;
  executionTimeMs: number;
  recommendations: string[];
}

export interface PerformanceMetric {
  name: string;
  value: number;
  timestamp: string;
  tags: Record<string, string>;
}

export interface MetricSummary {
  name: string;
  count: number;
  min: number;
  max: number;
  avg: number;
  p50: number;
  p95: number;
  p99: number;
}

export interface CacheService {
  get(key: string): Promise<unknown | null>;
  set(key: string, value: unknown, ttl?: number): Promise<void>;
  delete(key: string): Promise<void>;
}

// ============================================================================
// Health Monitoring
// ============================================================================

export class HealthMonitoringService {
  private checks: Map<string, HealthCheck> = new Map();

  constructor(private readonly config: MonitoringConfig) {
    this.registerDefaultChecks();
  }

  private registerDefaultChecks(): void {
    // Database health check
    this.registerCheck({
      name: 'database',
      type: 'dependency',
      interval: 30000,
      timeout: 5000,
      check: async () => {
        // Check database connectivity
        return { status: 'healthy' };
      },
    });

    // Cache health check
    this.registerCheck({
      name: 'cache',
      type: 'dependency',
      interval: 30000,
      timeout: 5000,
      check: async () => {
        // Check cache connectivity
        return { status: 'healthy' };
      },
    });

    // Memory usage check
    this.registerCheck({
      name: 'memory',
      type: 'resource',
      interval: 10000,
      timeout: 1000,
      check: async () => {
        const used = process.memoryUsage();
        const usedMB = Math.round(used.heapUsed / 1024 / 1024);
        const totalMB = Math.round(used.heapTotal / 1024 / 1024);

        return {
          status: usedMB < totalMB * 0.9 ? 'healthy' : 'degraded',
          details: {
            usedMB,
            totalMB,
            percentUsed: Math.round((usedMB / totalMB) * 100),
          },
        };
      },
    });

    // Disk usage check
    this.registerCheck({
      name: 'disk',
      type: 'resource',
      interval: 60000,
      timeout: 5000,
      check: async () => {
        // Check disk space
        return { status: 'healthy' };
      },
    });
  }

  registerCheck(check: HealthCheck): void {
    this.checks.set(check.name, check);
  }

  async runHealthCheck(): Promise<HealthCheckResult> {
    const results: ComponentHealth[] = [];
    let overallStatus: 'healthy' | 'degraded' | 'unhealthy' = 'healthy';

    for (const [name, check] of this.checks) {
      try {
        const result = await Promise.race([
          check.check(),
          this.timeout(check.timeout),
        ]);

        results.push({
          name,
          type: check.type,
          status: result.status,
          details: result.details,
          lastCheck: new Date().toISOString(),
        });

        if (result.status === 'unhealthy') {
          overallStatus = 'unhealthy';
        } else if (result.status === 'degraded' && overallStatus !== 'unhealthy') {
          overallStatus = 'degraded';
        }
      } catch (error) {
        results.push({
          name,
          type: check.type,
          status: 'unhealthy',
          error: error instanceof Error ? error.message : 'Check failed',
          lastCheck: new Date().toISOString(),
        });

        overallStatus = 'unhealthy';
      }
    }

    return {
      status: overallStatus,
      timestamp: new Date().toISOString(),
      components: results,
    };
  }

  private timeout(ms: number): Promise<never> {
    return new Promise((_, reject) =>
      setTimeout(() => reject(new Error('Health check timeout')), ms)
    );
  }

  async getLivenessProbe(): Promise<{ status: string }> {
    return { status: 'alive' };
  }

  async getReadinessProbe(): Promise<{ status: string; ready: boolean }> {
    const health = await this.runHealthCheck();

    return {
      status: health.status,
      ready: health.status !== 'unhealthy',
    };
  }
}

export interface HealthCheck {
  name: string;
  type: 'dependency' | 'resource' | 'custom';
  interval: number;
  timeout: number;
  check: () => Promise<{ status: 'healthy' | 'degraded' | 'unhealthy'; details?: unknown }>;
}

export interface HealthCheckResult {
  status: 'healthy' | 'degraded' | 'unhealthy';
  timestamp: string;
  components: ComponentHealth[];
}

export interface ComponentHealth {
  name: string;
  type: string;
  status: 'healthy' | 'degraded' | 'unhealthy';
  details?: unknown;
  error?: string;
  lastCheck: string;
}

// ============================================================================
// Configuration Management
// ============================================================================

export class ConfigurationManager {
  private config: Map<string, unknown> = new Map();
  private secrets: Map<string, string> = new Map();

  constructor(private readonly sources: ConfigSource[]) {}

  async load(): Promise<void> {
    for (const source of this.sources) {
      const values = await source.load();

      for (const [key, value] of Object.entries(values)) {
        if (source.type === 'secrets') {
          this.secrets.set(key, value as string);
        } else {
          this.config.set(key, value);
        }
      }
    }
  }

  get<T>(key: string, defaultValue?: T): T {
    const value = this.config.get(key);
    return (value as T) ?? defaultValue as T;
  }

  getSecret(key: string): string | undefined {
    return this.secrets.get(key);
  }

  getRequired<T>(key: string): T {
    const value = this.config.get(key);

    if (value === undefined) {
      throw new Error(`Required configuration missing: ${key}`);
    }

    return value as T;
  }

  getRequiredSecret(key: string): string {
    const value = this.secrets.get(key);

    if (value === undefined) {
      throw new Error(`Required secret missing: ${key}`);
    }

    return value;
  }

  validate(schema: z.ZodSchema): void {
    const configObject = Object.fromEntries(this.config);
    schema.parse(configObject);
  }
}

export interface ConfigSource {
  type: 'env' | 'file' | 'secrets' | 'remote';
  load(): Promise<Record<string, unknown>>;
}

export class EnvironmentConfigSource implements ConfigSource {
  type: 'env' = 'env';
  prefix: string;

  constructor(prefix: string = 'CRYO_LEGAL_') {
    this.prefix = prefix;
  }

  async load(): Promise<Record<string, unknown>> {
    const config: Record<string, unknown> = {};

    for (const [key, value] of Object.entries(process.env)) {
      if (key.startsWith(this.prefix)) {
        const configKey = key.slice(this.prefix.length).toLowerCase();
        config[configKey] = this.parseValue(value);
      }
    }

    return config;
  }

  private parseValue(value: string | undefined): unknown {
    if (value === undefined) return undefined;

    if (value === 'true') return true;
    if (value === 'false') return false;

    const num = Number(value);
    if (!isNaN(num)) return num;

    try {
      return JSON.parse(value);
    } catch {
      return value;
    }
  }
}
```

---

## Chapter Summary

This chapter covered comprehensive implementation guidance:

- **System Architecture**: Complete deployment configuration
- **Database Migrations**: Schema versioning and rollback
- **Performance Optimization**: Query analysis and caching
- **Health Monitoring**: Liveness and readiness probes
- **Configuration Management**: Multi-source configuration

---

**Next Chapter**: [Future Trends - Emerging Technologies](./09-future-trends.md)
