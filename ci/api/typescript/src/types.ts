/**
 * WIA CI Standard - Type Definitions
 * @packageDocumentation
 * @module wia-ci
 */

export enum PipelineStatus {
  Pending = 'pending',
  Running = 'running',
  Success = 'success',
  Failed = 'failed',
  Cancelled = 'cancelled',
  Skipped = 'skipped',
  Timeout = 'timeout'
}

export enum StageType {
  Build = 'build',
  Test = 'test',
  Lint = 'lint',
  Security = 'security',
  Deploy = 'deploy',
  Release = 'release',
  Notify = 'notify',
  Cleanup = 'cleanup'
}

export enum TriggerType {
  Push = 'push',
  PullRequest = 'pull_request',
  Tag = 'tag',
  Schedule = 'schedule',
  Manual = 'manual',
  Webhook = 'webhook',
  Api = 'api'
}

export enum ArtifactType {
  Binary = 'binary',
  Docker = 'docker',
  Package = 'package',
  Report = 'report',
  Log = 'log',
  Coverage = 'coverage',
  Documentation = 'documentation'
}

export enum EnvironmentType {
  Development = 'development',
  Staging = 'staging',
  Production = 'production',
  Testing = 'testing',
  Preview = 'preview'
}

export interface Pipeline {
  id: string;
  name: string;
  description?: string;
  repository: RepositoryConfig;
  triggers: TriggerConfig[];
  stages: Stage[];
  environment: EnvironmentVariables;
  timeout: number;
  retryPolicy?: RetryPolicy;
  notifications: NotificationConfig[];
  createdAt: Date;
  updatedAt: Date;
}

export interface RepositoryConfig {
  url: string;
  branch: string;
  provider: 'github' | 'gitlab' | 'bitbucket' | 'azure' | 'custom';
  credentials?: CredentialRef;
  submodules?: boolean;
  depth?: number;
}

export interface CredentialRef {
  type: 'token' | 'ssh' | 'oauth' | 'basic';
  secretName: string;
}

export interface TriggerConfig {
  type: TriggerType;
  branches?: string[];
  tags?: string[];
  paths?: string[];
  excludePaths?: string[];
  schedule?: CronSchedule;
  conditions?: TriggerCondition[];
}

export interface CronSchedule {
  expression: string;
  timezone: string;
}

export interface TriggerCondition {
  field: string;
  operator: 'equals' | 'contains' | 'matches' | 'not_equals';
  value: string;
}

export interface Stage {
  id: string;
  name: string;
  type: StageType;
  steps: Step[];
  dependsOn?: string[];
  condition?: string;
  environment?: EnvironmentVariables;
  timeout?: number;
  retryPolicy?: RetryPolicy;
  parallel?: boolean;
}

export interface Step {
  id: string;
  name: string;
  image?: string;
  command: string | string[];
  workdir?: string;
  environment?: EnvironmentVariables;
  secrets?: SecretRef[];
  artifacts?: ArtifactConfig[];
  cache?: CacheConfig[];
  timeout?: number;
  continueOnError?: boolean;
}

export interface SecretRef {
  name: string;
  envVar: string;
  path?: string;
}

export interface ArtifactConfig {
  name: string;
  type: ArtifactType;
  path: string;
  retention?: number;
  expireAt?: Date;
}

export interface CacheConfig {
  key: string;
  paths: string[];
  restoreKeys?: string[];
  ttl?: number;
}

export interface RetryPolicy {
  maxAttempts: number;
  delay: number;
  backoffMultiplier?: number;
  retryOn?: PipelineStatus[];
}

export interface EnvironmentVariables {
  [key: string]: string | SecretRef;
}

export interface NotificationConfig {
  type: 'slack' | 'email' | 'webhook' | 'teams' | 'discord';
  target: string;
  events: ('success' | 'failure' | 'started' | 'cancelled')[];
  template?: string;
}

export interface PipelineRun {
  id: string;
  pipelineId: string;
  number: number;
  status: PipelineStatus;
  trigger: TriggerInfo;
  stages: StageRun[];
  startedAt?: Date;
  finishedAt?: Date;
  duration?: number;
  artifacts: Artifact[];
  logs: LogEntry[];
}

export interface TriggerInfo {
  type: TriggerType;
  actor?: string;
  commit?: CommitInfo;
  branch?: string;
  tag?: string;
  pullRequest?: PullRequestInfo;
}

export interface CommitInfo {
  sha: string;
  message: string;
  author: string;
  authorEmail: string;
  timestamp: Date;
}

export interface PullRequestInfo {
  number: number;
  title: string;
  sourceBranch: string;
  targetBranch: string;
  author: string;
}

export interface StageRun {
  stageId: string;
  stageName: string;
  status: PipelineStatus;
  steps: StepRun[];
  startedAt?: Date;
  finishedAt?: Date;
  duration?: number;
}

export interface StepRun {
  stepId: string;
  stepName: string;
  status: PipelineStatus;
  exitCode?: number;
  output?: string;
  error?: string;
  startedAt?: Date;
  finishedAt?: Date;
  duration?: number;
  attempt: number;
}

export interface Artifact {
  id: string;
  name: string;
  type: ArtifactType;
  size: number;
  path: string;
  downloadUrl?: string;
  checksum: string;
  createdAt: Date;
  expiresAt?: Date;
}

export interface LogEntry {
  timestamp: Date;
  level: 'debug' | 'info' | 'warn' | 'error';
  stage?: string;
  step?: string;
  message: string;
}

export interface TestResult {
  suite: string;
  name: string;
  status: 'passed' | 'failed' | 'skipped' | 'error';
  duration: number;
  error?: string;
  stackTrace?: string;
}

export interface CoverageReport {
  lines: CoverageMetric;
  branches: CoverageMetric;
  functions: CoverageMetric;
  statements: CoverageMetric;
  files: FileCoverage[];
}

export interface CoverageMetric {
  total: number;
  covered: number;
  percentage: number;
}

export interface FileCoverage {
  path: string;
  lines: CoverageMetric;
  branches: CoverageMetric;
  functions: CoverageMetric;
}

export interface DeploymentTarget {
  id: string;
  name: string;
  environment: EnvironmentType;
  provider: string;
  config: Record<string, unknown>;
  credentials: CredentialRef;
  healthCheck?: HealthCheckConfig;
}

export interface HealthCheckConfig {
  endpoint: string;
  interval: number;
  timeout: number;
  successThreshold: number;
  failureThreshold: number;
}

export interface CIConfig {
  apiEndpoint: string;
  apiKey?: string;
  defaultTimeout: number;
  maxConcurrentRuns: number;
  artifactRetention: number;
  logRetention: number;
  enableMetrics: boolean;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-CI';
  testDate: string;
  config: CIConfig;
  targetLevel: CertificationLevel;
  tests: ComplianceTest[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface ComplianceTest {
  testName: string;
  passed: boolean;
  notes?: string;
}
