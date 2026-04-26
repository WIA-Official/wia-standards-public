/**
 * WIA-COMM-012: Cloud Computing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Cloud Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cloud Types
// ============================================================================

/**
 * Cloud service provider
 */
export type CloudProvider = 'aws' | 'azure' | 'gcp' | 'oracle' | 'ibm' | 'alibaba';

/**
 * Cloud service model
 */
export type ServiceModel = 'IaaS' | 'PaaS' | 'SaaS' | 'FaaS';

/**
 * Cloud deployment model
 */
export type DeploymentModel = 'public' | 'private' | 'hybrid' | 'multi-cloud';

/**
 * Cloud provider credentials
 */
export interface CloudCredentials {
  /** Provider name */
  provider: CloudProvider;

  /** Access key ID or client ID */
  accessKeyId?: string;

  /** Secret access key or client secret */
  secretAccessKey?: string;

  /** Azure subscription ID */
  subscriptionId?: string;

  /** GCP project ID */
  projectId?: string;

  /** GCP service account key */
  serviceAccountKey?: string;

  /** Session token (for temporary credentials) */
  sessionToken?: string;

  /** Role ARN (for assuming roles) */
  roleArn?: string;
}

/**
 * Cloud region
 */
export interface CloudRegion {
  /** Provider */
  provider: CloudProvider;

  /** Region code (e.g., 'us-east-1', 'eastus', 'us-central1') */
  code: string;

  /** Region name */
  name: string;

  /** Availability zones */
  availabilityZones?: string[];

  /** Geographic location */
  location: {
    continent: string;
    country: string;
    city?: string;
  };
}

// ============================================================================
// Infrastructure as a Service (IaaS)
// ============================================================================

/**
 * Instance types
 */
export type InstanceFamily =
  | 'general-purpose'
  | 'compute-optimized'
  | 'memory-optimized'
  | 'storage-optimized'
  | 'accelerated-computing';

/**
 * Virtual machine instance specification
 */
export interface InstanceSpec {
  /** Instance identifier */
  id?: string;

  /** Instance name */
  name: string;

  /** Instance type (e.g., 't3.medium', 'Standard_D2s_v3') */
  instanceType: string;

  /** Instance family */
  family?: InstanceFamily;

  /** Number of vCPUs */
  vcpus: number;

  /** Memory in GiB */
  memory: number;

  /** Operating system image */
  image: string;

  /** Region */
  region: string;

  /** Availability zone (optional) */
  availabilityZone?: string;

  /** Network configuration */
  network: NetworkConfig;

  /** Storage volumes */
  storage?: StorageVolume[];

  /** SSH key pair name */
  keyPair?: string;

  /** User data script */
  userData?: string;

  /** Resource tags */
  tags?: Record<string, string>;

  /** IAM role/instance profile */
  iamRole?: string;

  /** Monitoring enabled */
  monitoring?: boolean;
}

/**
 * Network configuration
 */
export interface NetworkConfig {
  /** Virtual Private Cloud ID */
  vpcId: string;

  /** Subnet ID */
  subnetId: string;

  /** Security group IDs */
  securityGroups: string[];

  /** Assign public IP */
  publicIp?: boolean;

  /** Elastic IP allocation ID */
  elasticIp?: string;

  /** Private IP address */
  privateIp?: string;
}

/**
 * Storage volume
 */
export interface StorageVolume {
  /** Volume ID */
  id?: string;

  /** Device name (e.g., '/dev/sda1') */
  device: string;

  /** Volume type (e.g., 'gp3', 'io2', 'standard') */
  type: string;

  /** Size in GiB */
  size: number;

  /** IOPS (for provisioned IOPS volumes) */
  iops?: number;

  /** Throughput in MiB/s */
  throughput?: number;

  /** Encrypted */
  encrypted?: boolean;

  /** KMS key ID */
  kmsKeyId?: string;

  /** Delete on termination */
  deleteOnTermination?: boolean;
}

/**
 * Instance state
 */
export type InstanceState =
  | 'pending'
  | 'running'
  | 'stopping'
  | 'stopped'
  | 'terminating'
  | 'terminated';

/**
 * Instance details
 */
export interface Instance {
  /** Instance ID */
  instanceId: string;

  /** Instance name */
  name: string;

  /** Instance type */
  instanceType: string;

  /** State */
  state: InstanceState;

  /** Launch time */
  launchTime: Date;

  /** Public IP address */
  publicIp?: string;

  /** Private IP address */
  privateIp: string;

  /** Public DNS name */
  publicDns?: string;

  /** Private DNS name */
  privateDns: string;

  /** Availability zone */
  availabilityZone: string;

  /** Tags */
  tags: Record<string, string>;
}

// ============================================================================
// Object Storage
// ============================================================================

/**
 * Storage bucket/container
 */
export interface StorageBucket {
  /** Bucket name */
  name: string;

  /** Region */
  region: string;

  /** Versioning enabled */
  versioning?: boolean;

  /** Encryption configuration */
  encryption?: {
    type: 'AES256' | 'aws:kms';
    kmsKeyId?: string;
  };

  /** Lifecycle rules */
  lifecycle?: LifecycleRule[];

  /** CORS configuration */
  cors?: CorsConfig;

  /** Access control */
  acl?: 'private' | 'public-read' | 'public-read-write';

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Lifecycle rule
 */
export interface LifecycleRule {
  /** Rule ID */
  id: string;

  /** Enabled */
  enabled: boolean;

  /** Prefix filter */
  prefix?: string;

  /** Tag filters */
  tags?: Record<string, string>;

  /** Transitions */
  transitions?: Array<{
    days: number;
    storageClass: string;
  }>;

  /** Expiration */
  expiration?: {
    days?: number;
    date?: Date;
  };
}

/**
 * CORS configuration
 */
export interface CorsConfig {
  /** Allowed origins */
  allowedOrigins: string[];

  /** Allowed methods */
  allowedMethods: string[];

  /** Allowed headers */
  allowedHeaders?: string[];

  /** Exposed headers */
  exposedHeaders?: string[];

  /** Max age in seconds */
  maxAge?: number;
}

// ============================================================================
// Platform as a Service (PaaS)
// ============================================================================

/**
 * Application platform runtime
 */
export type PlatformRuntime =
  | 'nodejs'
  | 'python'
  | 'java'
  | 'go'
  | 'dotnet'
  | 'ruby'
  | 'php'
  | 'docker';

/**
 * Application deployment configuration
 */
export interface ApplicationDeployment {
  /** Application name */
  name: string;

  /** Platform runtime */
  platform: PlatformRuntime;

  /** Runtime version */
  version: string;

  /** Source code location */
  source: {
    type: 'git' | 'zip' | 'docker';
    repository?: string;
    branch?: string;
    path?: string;
  };

  /** Environment variables */
  environment?: Record<string, string>;

  /** Scaling configuration */
  scaling?: ScalingConfig;

  /** Health check */
  healthCheck?: HealthCheck;

  /** Resource limits */
  resources?: {
    cpu?: string;
    memory?: string;
  };
}

/**
 * Scaling configuration
 */
export interface ScalingConfig {
  /** Minimum instances */
  minInstances: number;

  /** Maximum instances */
  maxInstances: number;

  /** Target CPU utilization (percentage) */
  targetCPU?: number;

  /** Target memory utilization (percentage) */
  targetMemory?: number;

  /** Custom metrics */
  customMetrics?: Array<{
    name: string;
    targetValue: number;
  }>;
}

/**
 * Health check configuration
 */
export interface HealthCheck {
  /** Path */
  path: string;

  /** Interval in seconds */
  interval: number;

  /** Timeout in seconds */
  timeout: number;

  /** Healthy threshold */
  healthyThreshold: number;

  /** Unhealthy threshold */
  unhealthyThreshold: number;

  /** HTTP status codes */
  matcher?: string;
}

/**
 * Managed database configuration
 */
export interface ManagedDatabase {
  /** Database identifier */
  id?: string;

  /** Database name */
  name: string;

  /** Engine (e.g., 'postgres', 'mysql', 'mongodb') */
  engine: string;

  /** Engine version */
  version: string;

  /** Instance class */
  instanceClass: string;

  /** Storage in GiB */
  storage: number;

  /** Storage type */
  storageType?: string;

  /** IOPS */
  iops?: number;

  /** Multi-AZ deployment */
  multiAZ?: boolean;

  /** Backup retention period (days) */
  backupRetention?: number;

  /** Backup window */
  backupWindow?: string;

  /** Maintenance window */
  maintenanceWindow?: string;

  /** Encryption enabled */
  encrypted?: boolean;

  /** KMS key ID */
  kmsKeyId?: string;

  /** Performance Insights enabled */
  performanceInsights?: boolean;

  /** Publicly accessible */
  publiclyAccessible?: boolean;

  /** VPC security groups */
  securityGroups?: string[];

  /** Subnet group */
  subnetGroup?: string;

  /** Tags */
  tags?: Record<string, string>;
}

// ============================================================================
// Function as a Service (Serverless)
// ============================================================================

/**
 * Serverless function runtime
 */
export type FunctionRuntime =
  | 'nodejs18.x'
  | 'nodejs20.x'
  | 'python3.9'
  | 'python3.10'
  | 'python3.11'
  | 'java11'
  | 'java17'
  | 'go1.x'
  | 'dotnet6'
  | 'dotnet8'
  | 'ruby3.2';

/**
 * Function trigger type
 */
export type FunctionTrigger =
  | 's3'
  | 'dynamodb'
  | 'kinesis'
  | 'sqs'
  | 'sns'
  | 'api-gateway'
  | 'cloudwatch-events'
  | 'http';

/**
 * Serverless function configuration
 */
export interface ServerlessFunction {
  /** Function name */
  name: string;

  /** Runtime */
  runtime: FunctionRuntime;

  /** Handler (e.g., 'index.handler') */
  handler: string;

  /** Code location */
  code: {
    type: 'zip' | 's3' | 'inline';
    path?: string;
    bucket?: string;
    key?: string;
    inline?: string;
  };

  /** Memory in MB */
  memory: number;

  /** Timeout in seconds */
  timeout: number;

  /** Environment variables */
  environment?: Record<string, string>;

  /** IAM role ARN */
  role?: string;

  /** Triggers */
  triggers?: FunctionTriggerConfig[];

  /** Layers */
  layers?: string[];

  /** VPC configuration */
  vpc?: {
    subnetIds: string[];
    securityGroupIds: string[];
  };

  /** Concurrency configuration */
  concurrency?: {
    reserved?: number;
    provisioned?: number;
  };

  /** Dead letter queue */
  deadLetterQueue?: {
    type: 'SQS' | 'SNS';
    targetArn: string;
  };

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Function trigger configuration
 */
export interface FunctionTriggerConfig {
  /** Trigger type */
  type: FunctionTrigger;

  /** Event source ARN */
  sourceArn?: string;

  /** Batch size (for stream sources) */
  batchSize?: number;

  /** Starting position (for stream sources) */
  startingPosition?: 'LATEST' | 'TRIM_HORIZON';

  /** Filter patterns */
  filter?: Record<string, any>;

  /** Enabled */
  enabled?: boolean;
}

/**
 * Function invocation result
 */
export interface FunctionInvocationResult {
  /** Status code */
  statusCode: number;

  /** Response payload */
  payload: any;

  /** Execution time in milliseconds */
  executionTime: number;

  /** Memory used in MB */
  memoryUsed: number;

  /** Log output */
  logs?: string;

  /** Request ID */
  requestId: string;
}

// ============================================================================
// Container Orchestration (Kubernetes)
// ============================================================================

/**
 * Kubernetes cluster configuration
 */
export interface KubernetesCluster {
  /** Cluster name */
  name: string;

  /** Kubernetes version */
  version: string;

  /** Region */
  region: string;

  /** VPC ID */
  vpcId?: string;

  /** Subnet IDs */
  subnetIds?: string[];

  /** Node groups */
  nodeGroups: NodeGroup[];

  /** Cluster endpoint */
  endpoint?: string;

  /** Certificate authority data */
  certificateAuthority?: string;

  /** Cluster addons */
  addons?: string[];

  /** Logging enabled */
  logging?: {
    types: string[];
    enabled: boolean;
  };

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Kubernetes node group
 */
export interface NodeGroup {
  /** Node group name */
  name: string;

  /** Instance type */
  instanceType: string;

  /** Minimum size */
  minSize: number;

  /** Maximum size */
  maxSize: number;

  /** Desired size */
  desiredSize: number;

  /** Disk size in GiB */
  diskSize?: number;

  /** AMI type */
  amiType?: string;

  /** SSH key pair */
  keyPair?: string;

  /** Labels */
  labels?: Record<string, string>;

  /** Taints */
  taints?: Array<{
    key: string;
    value: string;
    effect: 'NoSchedule' | 'PreferNoSchedule' | 'NoExecute';
  }>;

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Kubernetes deployment
 */
export interface KubernetesDeployment {
  /** Deployment name */
  name: string;

  /** Namespace */
  namespace: string;

  /** Replicas */
  replicas: number;

  /** Selector labels */
  selector: Record<string, string>;

  /** Pod template */
  template: {
    labels: Record<string, string>;
    containers: Container[];
  };

  /** Strategy */
  strategy?: {
    type: 'RollingUpdate' | 'Recreate';
    rollingUpdate?: {
      maxUnavailable?: number | string;
      maxSurge?: number | string;
    };
  };
}

/**
 * Container specification
 */
export interface Container {
  /** Container name */
  name: string;

  /** Image */
  image: string;

  /** Ports */
  ports?: Array<{
    containerPort: number;
    protocol?: 'TCP' | 'UDP';
  }>;

  /** Environment variables */
  env?: Array<{
    name: string;
    value?: string;
    valueFrom?: any;
  }>;

  /** Resource requirements */
  resources?: {
    requests?: {
      cpu?: string;
      memory?: string;
    };
    limits?: {
      cpu?: string;
      memory?: string;
    };
  };

  /** Liveness probe */
  livenessProbe?: Probe;

  /** Readiness probe */
  readinessProbe?: Probe;

  /** Volume mounts */
  volumeMounts?: Array<{
    name: string;
    mountPath: string;
    readOnly?: boolean;
  }>;
}

/**
 * Container probe
 */
export interface Probe {
  /** HTTP GET probe */
  httpGet?: {
    path: string;
    port: number;
    scheme?: 'HTTP' | 'HTTPS';
  };

  /** Exec probe */
  exec?: {
    command: string[];
  };

  /** TCP socket probe */
  tcpSocket?: {
    port: number;
  };

  /** Initial delay in seconds */
  initialDelaySeconds?: number;

  /** Period in seconds */
  periodSeconds?: number;

  /** Timeout in seconds */
  timeoutSeconds?: number;

  /** Success threshold */
  successThreshold?: number;

  /** Failure threshold */
  failureThreshold?: number;
}

// ============================================================================
// Auto-Scaling
// ============================================================================

/**
 * Auto-scaling policy
 */
export interface AutoScalingPolicy {
  /** Policy name */
  name: string;

  /** Resource identifier */
  resourceId: string;

  /** Policy type */
  type: 'TargetTrackingScaling' | 'StepScaling' | 'SimpleScaling';

  /** Target metric */
  metric?: {
    name: string;
    namespace?: string;
    statistic?: 'Average' | 'Sum' | 'Minimum' | 'Maximum';
    unit?: string;
  };

  /** Target value */
  targetValue?: number;

  /** Cooldown period in seconds */
  cooldown?: {
    scaleOut?: number;
    scaleIn?: number;
  };

  /** Minimum capacity */
  minCapacity: number;

  /** Maximum capacity */
  maxCapacity: number;
}

// ============================================================================
// Cloud Networking
// ============================================================================

/**
 * Virtual Private Cloud
 */
export interface VirtualPrivateCloud {
  /** VPC ID */
  id?: string;

  /** VPC name */
  name: string;

  /** CIDR block */
  cidr: string;

  /** Region */
  region: string;

  /** Enable DNS hostnames */
  enableDnsHostnames?: boolean;

  /** Enable DNS support */
  enableDnsSupport?: boolean;

  /** Subnets */
  subnets?: Subnet[];

  /** Route tables */
  routeTables?: RouteTable[];

  /** Internet gateway */
  internetGateway?: boolean;

  /** NAT gateways */
  natGateways?: NatGateway[];

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Subnet
 */
export interface Subnet {
  /** Subnet ID */
  id?: string;

  /** Subnet name */
  name: string;

  /** CIDR block */
  cidr: string;

  /** Availability zone */
  availabilityZone: string;

  /** Type */
  type: 'public' | 'private';

  /** Route table */
  routeTable?: string;

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Route table
 */
export interface RouteTable {
  /** Route table ID */
  id?: string;

  /** Route table name */
  name: string;

  /** Routes */
  routes: Route[];

  /** Associated subnets */
  subnets?: string[];
}

/**
 * Route
 */
export interface Route {
  /** Destination CIDR */
  destination: string;

  /** Target (gateway, NAT, instance, etc.) */
  target: string;
}

/**
 * NAT Gateway
 */
export interface NatGateway {
  /** NAT Gateway ID */
  id?: string;

  /** Subnet ID */
  subnetId: string;

  /** Elastic IP allocation ID */
  allocationId?: string;

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Load balancer type
 */
export type LoadBalancerType = 'application' | 'network' | 'gateway';

/**
 * Load balancer configuration
 */
export interface LoadBalancer {
  /** Load balancer name */
  name: string;

  /** Type */
  type: LoadBalancerType;

  /** Scheme */
  scheme: 'internet-facing' | 'internal';

  /** IP address type */
  ipAddressType?: 'ipv4' | 'dualstack';

  /** Subnets */
  subnets: string[];

  /** Security groups (for ALB) */
  securityGroups?: string[];

  /** Listeners */
  listeners: LoadBalancerListener[];

  /** Target groups */
  targetGroups?: TargetGroup[];

  /** Tags */
  tags?: Record<string, string>;
}

/**
 * Load balancer listener
 */
export interface LoadBalancerListener {
  /** Port */
  port: number;

  /** Protocol */
  protocol: 'HTTP' | 'HTTPS' | 'TCP' | 'TLS' | 'UDP';

  /** SSL policy */
  sslPolicy?: string;

  /** Certificates */
  certificates?: Array<{
    arn: string;
  }>;

  /** Default actions */
  defaultActions: Array<{
    type: 'forward' | 'redirect' | 'fixed-response';
    targetGroup?: string;
    redirectConfig?: any;
    fixedResponseConfig?: any;
  }>;
}

/**
 * Target group
 */
export interface TargetGroup {
  /** Target group name */
  name: string;

  /** Protocol */
  protocol: 'HTTP' | 'HTTPS' | 'TCP' | 'TLS';

  /** Port */
  port: number;

  /** VPC ID */
  vpcId: string;

  /** Health check */
  healthCheck?: HealthCheck;

  /** Targets */
  targets?: Array<{
    id: string;
    port?: number;
  }>;

  /** Tags */
  tags?: Record<string, string>;
}

// ============================================================================
// Cloud Security
// ============================================================================

/**
 * Security group rule
 */
export interface SecurityGroupRule {
  /** Protocol */
  protocol: 'tcp' | 'udp' | 'icmp' | '-1';

  /** Port */
  port: number;

  /** Port range */
  portRange?: {
    from: number;
    to: number;
  };

  /** Source (CIDR or security group) */
  source?: string;

  /** Destination (CIDR or security group) */
  destination?: string;

  /** Description */
  description?: string;
}

/**
 * IAM policy
 */
export interface IamPolicy {
  /** Policy name */
  name: string;

  /** Policy version */
  version: string;

  /** Statements */
  statements: IamStatement[];
}

/**
 * IAM statement
 */
export interface IamStatement {
  /** Statement ID */
  sid?: string;

  /** Effect */
  effect: 'Allow' | 'Deny';

  /** Actions */
  actions: string[];

  /** Resources */
  resources: string[];

  /** Conditions */
  conditions?: Record<string, any>;
}

// ============================================================================
// Cost Management (FinOps)
// ============================================================================

/**
 * Cost allocation tag
 */
export interface CostAllocationTag {
  /** Tag key */
  key: string;

  /** Tag values */
  values?: string[];

  /** Required */
  required?: boolean;

  /** Validation pattern */
  pattern?: string;
}

/**
 * Budget configuration
 */
export interface Budget {
  /** Budget name */
  name: string;

  /** Amount */
  amount: number;

  /** Currency */
  currency: string;

  /** Time period */
  period: 'monthly' | 'quarterly' | 'annually';

  /** Filters */
  filters?: {
    tags?: Record<string, string>;
    services?: string[];
    regions?: string[];
  };

  /** Alerts */
  alerts: BudgetAlert[];
}

/**
 * Budget alert
 */
export interface BudgetAlert {
  /** Threshold */
  threshold: number;

  /** Type */
  type: 'percentage' | 'absolute';

  /** Recipients */
  recipients: string[];

  /** SNS topic ARN */
  snsTopicArn?: string;
}

/**
 * Cost analysis result
 */
export interface CostAnalysis {
  /** Total cost */
  total: number;

  /** Currency */
  currency: string;

  /** Time range */
  timeRange: {
    start: Date;
    end: Date;
  };

  /** Breakdown by service */
  byService: Array<{
    service: string;
    cost: number;
    percentage: number;
  }>;

  /** Breakdown by resource */
  byResource?: Array<{
    resourceId: string;
    resourceType: string;
    cost: number;
  }>;

  /** Recommendations */
  recommendations?: CostOptimizationRecommendation[];
}

/**
 * Cost optimization recommendation
 */
export interface CostOptimizationRecommendation {
  /** Recommendation ID */
  id: string;

  /** Type */
  type: 'rightsizing' | 'reserved-instances' | 'spot-instances' | 'storage-tiering';

  /** Resource */
  resource: string;

  /** Current cost */
  currentCost: number;

  /** Estimated savings */
  estimatedSavings: number;

  /** Description */
  description: string;

  /** Action required */
  action: string;
}

// ============================================================================
// Monitoring and Observability
// ============================================================================

/**
 * Metric data point
 */
export interface MetricDataPoint {
  /** Timestamp */
  timestamp: Date;

  /** Value */
  value: number;

  /** Unit */
  unit?: string;

  /** Dimensions */
  dimensions?: Record<string, string>;
}

/**
 * Metric query
 */
export interface MetricQuery {
  /** Metric name */
  name: string;

  /** Namespace */
  namespace: string;

  /** Statistic */
  statistic: 'Average' | 'Sum' | 'Minimum' | 'Maximum' | 'SampleCount';

  /** Period in seconds */
  period: number;

  /** Dimensions */
  dimensions?: Record<string, string>;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;
}

/**
 * Alarm configuration
 */
export interface Alarm {
  /** Alarm name */
  name: string;

  /** Description */
  description?: string;

  /** Metric */
  metric: {
    name: string;
    namespace: string;
    statistic: string;
    period: number;
    dimensions?: Record<string, string>;
  };

  /** Comparison operator */
  comparisonOperator: 'GreaterThan' | 'LessThan' | 'GreaterThanOrEqualTo' | 'LessThanOrEqualTo';

  /** Threshold */
  threshold: number;

  /** Evaluation periods */
  evaluationPeriods: number;

  /** Actions */
  actions?: {
    alarm?: string[];
    ok?: string[];
    insufficientData?: string[];
  };

  /** Treat missing data */
  treatMissingData?: 'notBreaching' | 'breaching' | 'ignore' | 'missing';
}

// ============================================================================
// Disaster Recovery
// ============================================================================

/**
 * Backup configuration
 */
export interface BackupConfiguration {
  /** Backup name */
  name: string;

  /** Resources to backup */
  resources: string[];

  /** Schedule */
  schedule: {
    frequency: 'hourly' | 'daily' | 'weekly' | 'monthly';
    time?: string;
    dayOfWeek?: string;
    dayOfMonth?: number;
  };

  /** Retention */
  retention: {
    daily?: number;
    weekly?: number;
    monthly?: number;
    yearly?: number;
  };

  /** Cross-region replication */
  crossRegion?: {
    enabled: boolean;
    destinations: string[];
  };

  /** Encryption */
  encryption?: boolean;

  /** Lifecycle */
  lifecycle?: {
    moveToArchive?: number;
    delete?: number;
  };
}

/**
 * Disaster recovery tier
 */
export type DisasterRecoveryTier = 'tier1' | 'tier2' | 'tier3' | 'tier4';

/**
 * Disaster recovery configuration
 */
export interface DisasterRecoveryConfig {
  /** Tier */
  tier: DisasterRecoveryTier;

  /** Recovery Time Objective (RTO) in minutes */
  rto: number;

  /** Recovery Point Objective (RPO) in minutes */
  rpo: number;

  /** Strategy */
  strategy: 'backup-restore' | 'pilot-light' | 'warm-standby' | 'active-active';

  /** Primary region */
  primaryRegion: string;

  /** DR region */
  drRegion: string;

  /** Failover automation */
  automaticFailover?: boolean;

  /** Testing schedule */
  testingSchedule?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-012 error codes
 */
export enum CloudComputingErrorCode {
  INVALID_CREDENTIALS = 'CC001',
  REGION_NOT_AVAILABLE = 'CC002',
  INSTANCE_LAUNCH_FAILED = 'CC003',
  INSUFFICIENT_CAPACITY = 'CC004',
  INVALID_PARAMETER = 'CC005',
  RESOURCE_NOT_FOUND = 'CC006',
  PERMISSION_DENIED = 'CC007',
  QUOTA_EXCEEDED = 'CC008',
  NETWORK_ERROR = 'CC009',
  TIMEOUT = 'CC010',
  DEPLOYMENT_FAILED = 'CC011',
  SCALING_FAILED = 'CC012',
  BACKUP_FAILED = 'CC013',
  UNKNOWN_ERROR = 'CC999',
}

/**
 * Cloud computing error
 */
export class CloudComputingError extends Error {
  constructor(
    public code: CloudComputingErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CloudComputingError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page token */
  nextToken?: string;

  /** Maximum results */
  maxResults?: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items */
  items: T[];

  /** Next page token */
  nextToken?: string;

  /** Total count (if available) */
  totalCount?: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Cloud computing constants
 */
export const CLOUD_CONSTANTS = {
  /** Default region */
  DEFAULT_REGION: 'us-east-1',

  /** Default instance type */
  DEFAULT_INSTANCE_TYPE: 't3.medium',

  /** Default timeout in milliseconds */
  DEFAULT_TIMEOUT: 30000,

  /** Maximum retries */
  MAX_RETRIES: 3,

  /** Default page size */
  DEFAULT_PAGE_SIZE: 100,

  /** Minimum RTO (minutes) */
  MIN_RTO: 1,

  /** Minimum RPO (minutes) */
  MIN_RPO: 0,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  CloudProvider,
  ServiceModel,
  DeploymentModel,
  CloudCredentials,
  CloudRegion,
  InstanceFamily,
  InstanceSpec,
  NetworkConfig,
  StorageVolume,
  InstanceState,
  Instance,
  StorageBucket,
  LifecycleRule,
  CorsConfig,
  PlatformRuntime,
  ApplicationDeployment,
  ScalingConfig,
  HealthCheck,
  ManagedDatabase,
  FunctionRuntime,
  FunctionTrigger,
  ServerlessFunction,
  FunctionTriggerConfig,
  FunctionInvocationResult,
  KubernetesCluster,
  NodeGroup,
  KubernetesDeployment,
  Container,
  Probe,
  AutoScalingPolicy,
  VirtualPrivateCloud,
  Subnet,
  RouteTable,
  Route,
  NatGateway,
  LoadBalancerType,
  LoadBalancer,
  LoadBalancerListener,
  TargetGroup,
  SecurityGroupRule,
  IamPolicy,
  IamStatement,
  CostAllocationTag,
  Budget,
  BudgetAlert,
  CostAnalysis,
  CostOptimizationRecommendation,
  MetricDataPoint,
  MetricQuery,
  Alarm,
  BackupConfiguration,
  DisasterRecoveryTier,
  DisasterRecoveryConfig,
  PaginationParams,
  PaginatedResponse,
};

export { CLOUD_CONSTANTS, CloudComputingErrorCode, CloudComputingError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
