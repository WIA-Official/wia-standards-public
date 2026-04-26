/**
 * WIA-COMP-006: Container Technology - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Container state
 */
export type ContainerState =
  | 'created'
  | 'running'
  | 'paused'
  | 'stopped'
  | 'removing'
  | 'exited'
  | 'dead';

/**
 * Container restart policy
 */
export type RestartPolicy =
  | 'no'
  | 'always'
  | 'on-failure'
  | 'unless-stopped';

/**
 * Network mode
 */
export type NetworkMode = 'bridge' | 'host' | 'none' | 'container' | 'custom';

/**
 * Volume type
 */
export type VolumeType = 'bind' | 'volume' | 'tmpfs';

// ============================================================================
// Container Configuration
// ============================================================================

/**
 * Port mapping configuration
 */
export interface PortMapping {
  /** Host port */
  host: number;

  /** Container port */
  container: number;

  /** Protocol (tcp/udp) */
  protocol?: 'tcp' | 'udp';

  /** Host IP to bind to */
  hostIp?: string;
}

/**
 * Environment variable
 */
export interface EnvironmentVariable {
  /** Variable name */
  name: string;

  /** Variable value */
  value: string;
}

/**
 * Resource limits
 */
export interface ResourceLimits {
  /** CPU cores (1.0 = 1 core) */
  cpu?: number;

  /** Memory limit (e.g., "512Mi", "1Gi") */
  memory?: string;

  /** CPU shares (relative weight) */
  cpuShares?: number;

  /** Memory reservation (soft limit) */
  memoryReservation?: string;

  /** Disk I/O weight (10-1000) */
  ioWeight?: number;

  /** PID limit */
  pidsLimit?: number;
}

/**
 * Health check configuration
 */
export interface HealthCheck {
  /** Command to run */
  test: string[];

  /** Interval between checks */
  interval?: string;

  /** Timeout for each check */
  timeout?: string;

  /** Number of retries before unhealthy */
  retries?: number;

  /** Start period before checks begin */
  startPeriod?: string;
}

/**
 * Volume mount
 */
export interface VolumeMount {
  /** Source (host path or volume name) */
  source: string;

  /** Target path in container */
  target: string;

  /** Mount type */
  type?: VolumeType;

  /** Read-only mount */
  readonly?: boolean;

  /** Volume options */
  options?: Record<string, string>;
}

/**
 * Container creation configuration
 */
export interface ContainerConfig {
  /** Container image */
  image: string;

  /** Container name */
  name?: string;

  /** Command to run */
  command?: string[];

  /** Entrypoint override */
  entrypoint?: string[];

  /** Working directory */
  workingDir?: string;

  /** Environment variables */
  environment?: Record<string, string>;

  /** Port mappings */
  ports?: PortMapping[];

  /** Volume mounts */
  volumes?: VolumeMount[];

  /** Network mode */
  network?: NetworkMode;

  /** Network to connect to */
  networks?: string[];

  /** Resource limits */
  resources?: ResourceLimits;

  /** Health check */
  healthCheck?: HealthCheck;

  /** Restart policy */
  restart?: RestartPolicy;

  /** Remove container when stopped */
  autoRemove?: boolean;

  /** Run in privileged mode */
  privileged?: boolean;

  /** User to run as */
  user?: string;

  /** Hostname */
  hostname?: string;

  /** DNS servers */
  dns?: string[];

  /** Labels */
  labels?: Record<string, string>;
}

/**
 * Container information
 */
export interface ContainerInfo {
  /** Container ID */
  id: string;

  /** Container name */
  name: string;

  /** Image used */
  image: string;

  /** Current state */
  state: ContainerState;

  /** Container status message */
  status: string;

  /** Creation timestamp */
  created: Date;

  /** Started timestamp */
  started?: Date;

  /** Finished timestamp */
  finished?: Date;

  /** Exit code */
  exitCode?: number;

  /** Port mappings */
  ports: PortMapping[];

  /** IP addresses */
  ipAddresses: Record<string, string>;

  /** Resource usage */
  resources?: ResourceUsage;
}

/**
 * Resource usage statistics
 */
export interface ResourceUsage {
  /** CPU usage percentage */
  cpuPercent: number;

  /** Memory usage in bytes */
  memoryUsage: number;

  /** Memory limit in bytes */
  memoryLimit: number;

  /** Memory usage percentage */
  memoryPercent: number;

  /** Network I/O */
  networkIO: {
    rxBytes: number;
    txBytes: number;
  };

  /** Block I/O */
  blockIO: {
    readBytes: number;
    writeBytes: number;
  };

  /** PIDs */
  pids: number;
}

// ============================================================================
// Image Management
// ============================================================================

/**
 * Build arguments
 */
export type BuildArgs = Record<string, string>;

/**
 * Image build configuration
 */
export interface BuildConfig {
  /** Path to Dockerfile */
  dockerfile?: string;

  /** Build context path */
  context: string;

  /** Image tags */
  tags: string[];

  /** Build arguments */
  buildArgs?: BuildArgs;

  /** Target stage for multi-stage builds */
  target?: string;

  /** Build cache mode */
  cache?: boolean;

  /** Network mode during build */
  network?: NetworkMode;

  /** Platform (e.g., "linux/amd64") */
  platform?: string;

  /** Labels */
  labels?: Record<string, string>;

  /** Squash layers */
  squash?: boolean;
}

/**
 * Image information
 */
export interface ImageInfo {
  /** Image ID */
  id: string;

  /** Repository tags */
  tags: string[];

  /** Digest */
  digest?: string;

  /** Created timestamp */
  created: Date;

  /** Size in bytes */
  size: number;

  /** Virtual size (including parent layers) */
  virtualSize: number;

  /** Number of layers */
  layers: number;

  /** Author */
  author?: string;

  /** Architecture */
  architecture: string;

  /** OS */
  os: string;

  /** Labels */
  labels: Record<string, string>;
}

/**
 * Image pull progress
 */
export interface PullProgress {
  /** Layer ID */
  layer: string;

  /** Progress status */
  status: 'downloading' | 'extracting' | 'complete';

  /** Current bytes */
  current: number;

  /** Total bytes */
  total: number;
}

// ============================================================================
// Network Management
// ============================================================================

/**
 * Network driver
 */
export type NetworkDriver = 'bridge' | 'overlay' | 'macvlan' | 'host' | 'none';

/**
 * IP address configuration
 */
export interface IPAMConfig {
  /** Subnet */
  subnet: string;

  /** IP range */
  ipRange?: string;

  /** Gateway */
  gateway?: string;
}

/**
 * Network configuration
 */
export interface NetworkConfig {
  /** Network name */
  name: string;

  /** Network driver */
  driver: NetworkDriver;

  /** Enable IPv6 */
  ipv6?: boolean;

  /** Internal network */
  internal?: boolean;

  /** Attachable to swarm services */
  attachable?: boolean;

  /** IPAM configuration */
  ipam?: {
    driver: string;
    config: IPAMConfig[];
  };

  /** Driver options */
  options?: Record<string, string>;

  /** Labels */
  labels?: Record<string, string>;
}

/**
 * Network information
 */
export interface NetworkInfo {
  /** Network ID */
  id: string;

  /** Network name */
  name: string;

  /** Driver */
  driver: NetworkDriver;

  /** Scope */
  scope: 'local' | 'global' | 'swarm';

  /** IPv6 enabled */
  ipv6: boolean;

  /** Internal */
  internal: boolean;

  /** Containers connected */
  containers: Record<string, { ipAddress: string; macAddress: string }>;

  /** Created timestamp */
  created: Date;
}

// ============================================================================
// Volume Management
// ============================================================================

/**
 * Volume driver
 */
export type VolumeDriver = 'local' | 'nfs' | 'cifs' | 'ceph' | 'glusterfs';

/**
 * Volume configuration
 */
export interface VolumeConfig {
  /** Volume name */
  name: string;

  /** Volume driver */
  driver?: VolumeDriver;

  /** Driver options */
  driverOpts?: Record<string, string>;

  /** Labels */
  labels?: Record<string, string>;
}

/**
 * Volume information
 */
export interface VolumeInfo {
  /** Volume name */
  name: string;

  /** Driver */
  driver: VolumeDriver;

  /** Mount point */
  mountpoint: string;

  /** Created timestamp */
  created: Date;

  /** Labels */
  labels: Record<string, string>;

  /** Scope */
  scope: 'local' | 'global';

  /** Size in bytes (if available) */
  size?: number;
}

// ============================================================================
// Execution and Logs
// ============================================================================

/**
 * Exec configuration
 */
export interface ExecConfig {
  /** Container ID or name */
  container: string;

  /** Command to execute */
  command: string[];

  /** Working directory */
  workingDir?: string;

  /** Environment variables */
  environment?: Record<string, string>;

  /** Attach to stdin */
  stdin?: boolean;

  /** Attach to stdout */
  stdout?: boolean;

  /** Attach to stderr */
  stderr?: boolean;

  /** Run as user */
  user?: string;

  /** Privileged mode */
  privileged?: boolean;
}

/**
 * Exec result
 */
export interface ExecResult {
  /** Exit code */
  exitCode: number;

  /** Standard output */
  stdout: string;

  /** Standard error */
  stderr: string;

  /** Execution time (ms) */
  duration: number;
}

/**
 * Log options
 */
export interface LogOptions {
  /** Follow log output */
  follow?: boolean;

  /** Show timestamps */
  timestamps?: boolean;

  /** Number of lines from end */
  tail?: number;

  /** Show logs since timestamp */
  since?: Date;

  /** Show logs until timestamp */
  until?: Date;

  /** Show stdout */
  stdout?: boolean;

  /** Show stderr */
  stderr?: boolean;
}

/**
 * Log entry
 */
export interface LogEntry {
  /** Timestamp */
  timestamp: Date;

  /** Stream (stdout/stderr) */
  stream: 'stdout' | 'stderr';

  /** Log message */
  message: string;
}

// ============================================================================
// Registry
// ============================================================================

/**
 * Registry authentication
 */
export interface RegistryAuth {
  /** Registry URL */
  registry: string;

  /** Username */
  username: string;

  /** Password */
  password: string;

  /** Email */
  email?: string;

  /** Auth token (alternative to username/password) */
  token?: string;
}

/**
 * Push/Pull options
 */
export interface RegistryOptions {
  /** Registry authentication */
  auth?: RegistryAuth;

  /** Platform */
  platform?: string;

  /** Quiet mode */
  quiet?: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Container error codes
 */
export enum ContainerErrorCode {
  CONTAINER_NOT_FOUND = 'C001',
  IMAGE_NOT_FOUND = 'C002',
  NETWORK_NOT_FOUND = 'C003',
  VOLUME_NOT_FOUND = 'C004',
  INVALID_CONFIG = 'C005',
  BUILD_FAILED = 'C006',
  START_FAILED = 'C007',
  EXEC_FAILED = 'C008',
  PULL_FAILED = 'C009',
  PUSH_FAILED = 'C010',
}

/**
 * Container error
 */
export class ContainerError extends Error {
  constructor(
    public code: ContainerErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ContainerError';
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

// ============================================================================
// Export All
// ============================================================================

export type {
  ContainerState,
  RestartPolicy,
  NetworkMode,
  VolumeType,
  PortMapping,
  EnvironmentVariable,
  ResourceLimits,
  HealthCheck,
  VolumeMount,
  ContainerConfig,
  ContainerInfo,
  ResourceUsage,
  BuildArgs,
  BuildConfig,
  ImageInfo,
  PullProgress,
  NetworkDriver,
  IPAMConfig,
  NetworkConfig,
  NetworkInfo,
  VolumeDriver,
  VolumeConfig,
  VolumeInfo,
  ExecConfig,
  ExecResult,
  LogOptions,
  LogEntry,
  RegistryAuth,
  RegistryOptions,
};

export { ContainerErrorCode, ContainerError };
