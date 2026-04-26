/**
 * WIA-COMP-006: Container Technology SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for container management including:
 * - Container lifecycle management
 * - Image building and distribution
 * - Network and volume management
 * - Resource monitoring
 */

import {
  ContainerConfig,
  ContainerInfo,
  BuildConfig,
  ImageInfo,
  NetworkConfig,
  NetworkInfo,
  VolumeConfig,
  VolumeInfo,
  ExecConfig,
  ExecResult,
  LogOptions,
  LogEntry,
  ResourceUsage,
  ContainerErrorCode,
  ContainerError,
  ContainerState,
  RegistryOptions,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMP-006 Container Technology SDK
 */
export class ContainerSDK {
  private version = '1.0.0';
  private runtime = 'containerd'; // Default runtime

  constructor(runtime?: string) {
    if (runtime) {
      this.runtime = runtime;
    }
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get container runtime
   */
  getRuntime(): string {
    return this.runtime;
  }

  /**
   * Create a new container
   *
   * @param config - Container configuration
   * @returns Container information
   */
  async createContainer(config: ContainerConfig): Promise<ContainerInfo> {
    // Validate configuration
    if (!config.image) {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        'Image is required'
      );
    }

    // Generate container ID
    const id = this.generateId('container');
    const name = config.name || `container-${id.substring(0, 12)}`;

    // Create container
    const container: ContainerInfo = {
      id,
      name,
      image: config.image,
      state: 'created',
      status: 'Created',
      created: new Date(),
      ports: config.ports || [],
      ipAddresses: {},
    };

    return container;
  }

  /**
   * Start a container
   *
   * @param containerId - Container ID or name
   * @returns Updated container information
   */
  async startContainer(containerId: string): Promise<ContainerInfo> {
    const container = await this.getContainer(containerId);

    if (container.state === 'running') {
      throw new ContainerError(
        ContainerErrorCode.START_FAILED,
        `Container ${containerId} is already running`
      );
    }

    container.state = 'running';
    container.status = 'Running';
    container.started = new Date();

    return container;
  }

  /**
   * Stop a container
   *
   * @param containerId - Container ID or name
   * @param timeout - Graceful shutdown timeout (seconds)
   * @returns Updated container information
   */
  async stopContainer(
    containerId: string,
    timeout = 10
  ): Promise<ContainerInfo> {
    const container = await this.getContainer(containerId);

    if (container.state !== 'running') {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        `Container ${containerId} is not running`
      );
    }

    // Simulate graceful shutdown
    await this.sleep(Math.min(timeout * 100, 1000));

    container.state = 'exited';
    container.status = 'Exited (0)';
    container.finished = new Date();
    container.exitCode = 0;

    return container;
  }

  /**
   * Remove a container
   *
   * @param containerId - Container ID or name
   * @param force - Force removal even if running
   */
  async removeContainer(containerId: string, force = false): Promise<void> {
    const container = await this.getContainer(containerId);

    if (container.state === 'running' && !force) {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        `Container ${containerId} is running. Use force=true to remove.`
      );
    }

    // Container removed
  }

  /**
   * List containers
   *
   * @param all - Include stopped containers
   * @returns Array of container information
   */
  async listContainers(all = false): Promise<ContainerInfo[]> {
    // Simulate listing containers
    const containers: ContainerInfo[] = [
      {
        id: this.generateId('container'),
        name: 'example-web',
        image: 'nginx:latest',
        state: 'running',
        status: 'Up 2 hours',
        created: new Date(Date.now() - 7200000),
        started: new Date(Date.now() - 7200000),
        ports: [{ host: 8080, container: 80 }],
        ipAddresses: { bridge: '172.17.0.2' },
      },
    ];

    return all ? containers : containers.filter((c) => c.state === 'running');
  }

  /**
   * Get container information
   *
   * @param containerId - Container ID or name
   * @returns Container information
   */
  async getContainer(containerId: string): Promise<ContainerInfo> {
    // Simulate container lookup
    const containers = await this.listContainers(true);
    const container = containers.find(
      (c) => c.id === containerId || c.name === containerId
    );

    if (!container) {
      throw new ContainerError(
        ContainerErrorCode.CONTAINER_NOT_FOUND,
        `Container ${containerId} not found`
      );
    }

    return container;
  }

  /**
   * Get container resource usage
   *
   * @param containerId - Container ID or name
   * @returns Resource usage statistics
   */
  async getContainerStats(containerId: string): Promise<ResourceUsage> {
    await this.getContainer(containerId); // Verify exists

    // Simulate resource usage
    const stats: ResourceUsage = {
      cpuPercent: Math.random() * 50, // 0-50%
      memoryUsage: Math.floor(Math.random() * 512 * 1024 * 1024), // 0-512MB
      memoryLimit: 512 * 1024 * 1024, // 512MB
      memoryPercent: Math.random() * 60, // 0-60%
      networkIO: {
        rxBytes: Math.floor(Math.random() * 1024 * 1024 * 100), // 0-100MB
        txBytes: Math.floor(Math.random() * 1024 * 1024 * 50), // 0-50MB
      },
      blockIO: {
        readBytes: Math.floor(Math.random() * 1024 * 1024 * 200), // 0-200MB
        writeBytes: Math.floor(Math.random() * 1024 * 1024 * 100), // 0-100MB
      },
      pids: Math.floor(Math.random() * 50) + 10, // 10-60 processes
    };

    return stats;
  }

  /**
   * Execute command in container
   *
   * @param config - Exec configuration
   * @returns Execution result
   */
  async execContainer(config: ExecConfig): Promise<ExecResult> {
    await this.getContainer(config.container);

    const startTime = Date.now();

    // Simulate command execution
    const result: ExecResult = {
      exitCode: 0,
      stdout: `Executed: ${config.command.join(' ')}\n`,
      stderr: '',
      duration: Date.now() - startTime,
    };

    return result;
  }

  /**
   * Get container logs
   *
   * @param containerId - Container ID or name
   * @param options - Log options
   * @returns Array of log entries
   */
  async getContainerLogs(
    containerId: string,
    options: LogOptions = {}
  ): Promise<LogEntry[]> {
    await this.getContainer(containerId);

    // Simulate log entries
    const logs: LogEntry[] = [
      {
        timestamp: new Date(),
        stream: 'stdout',
        message: 'Server started on port 8080',
      },
      {
        timestamp: new Date(),
        stream: 'stdout',
        message: 'Ready to accept connections',
      },
    ];

    if (options.tail) {
      return logs.slice(-options.tail);
    }

    return logs;
  }

  /**
   * Run a container (create + start)
   *
   * @param config - Container configuration
   * @returns Container information
   */
  async runContainer(config: ContainerConfig): Promise<ContainerInfo> {
    const container = await this.createContainer(config);
    return await this.startContainer(container.id);
  }

  // ============================================================================
  // Image Management
  // ============================================================================

  /**
   * Build an image
   *
   * @param config - Build configuration
   * @returns Image information
   */
  async buildImage(config: BuildConfig): Promise<ImageInfo> {
    if (!config.context) {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        'Build context is required'
      );
    }

    // Generate image ID
    const id = this.generateId('image');

    const image: ImageInfo = {
      id,
      tags: config.tags,
      created: new Date(),
      size: Math.floor(Math.random() * 500 * 1024 * 1024), // 0-500MB
      virtualSize: Math.floor(Math.random() * 800 * 1024 * 1024), // 0-800MB
      layers: Math.floor(Math.random() * 10) + 3, // 3-12 layers
      architecture: 'amd64',
      os: 'linux',
      labels: config.labels || {},
    };

    return image;
  }

  /**
   * Pull an image
   *
   * @param image - Image name and tag
   * @param options - Pull options
   * @returns Image information
   */
  async pullImage(
    image: string,
    options: RegistryOptions = {}
  ): Promise<ImageInfo> {
    // Validate image name
    if (!image) {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        'Image name is required'
      );
    }

    const [repo, tag = 'latest'] = image.split(':');

    const imageInfo: ImageInfo = {
      id: this.generateId('image'),
      tags: [`${repo}:${tag}`],
      created: new Date(),
      size: Math.floor(Math.random() * 300 * 1024 * 1024),
      virtualSize: Math.floor(Math.random() * 500 * 1024 * 1024),
      layers: Math.floor(Math.random() * 8) + 2,
      architecture: options.platform?.split('/')[1] || 'amd64',
      os: options.platform?.split('/')[0] || 'linux',
      labels: {},
    };

    return imageInfo;
  }

  /**
   * Push an image
   *
   * @param image - Image name and tag
   * @param options - Push options
   */
  async pushImage(
    image: string,
    options: RegistryOptions = {}
  ): Promise<void> {
    if (!options.auth) {
      throw new ContainerError(
        ContainerErrorCode.PUSH_FAILED,
        'Registry authentication required'
      );
    }

    // Simulate push
    await this.sleep(500);
  }

  /**
   * List images
   *
   * @returns Array of image information
   */
  async listImages(): Promise<ImageInfo[]> {
    const images: ImageInfo[] = [
      {
        id: this.generateId('image'),
        tags: ['nginx:latest'],
        created: new Date(),
        size: 142 * 1024 * 1024, // 142MB
        virtualSize: 142 * 1024 * 1024,
        layers: 6,
        architecture: 'amd64',
        os: 'linux',
        labels: {},
      },
      {
        id: this.generateId('image'),
        tags: ['node:18-alpine'],
        created: new Date(),
        size: 174 * 1024 * 1024, // 174MB
        virtualSize: 174 * 1024 * 1024,
        layers: 5,
        architecture: 'amd64',
        os: 'linux',
        labels: {},
      },
    ];

    return images;
  }

  /**
   * Remove an image
   *
   * @param imageId - Image ID or name
   * @param force - Force removal
   */
  async removeImage(imageId: string, force = false): Promise<void> {
    // Simulate image removal
    await this.sleep(100);
  }

  // ============================================================================
  // Network Management
  // ============================================================================

  /**
   * Create a network
   *
   * @param config - Network configuration
   * @returns Network information
   */
  async createNetwork(config: NetworkConfig): Promise<NetworkInfo> {
    const network: NetworkInfo = {
      id: this.generateId('network'),
      name: config.name,
      driver: config.driver,
      scope: 'local',
      ipv6: config.ipv6 || false,
      internal: config.internal || false,
      containers: {},
      created: new Date(),
    };

    return network;
  }

  /**
   * List networks
   *
   * @returns Array of network information
   */
  async listNetworks(): Promise<NetworkInfo[]> {
    const networks: NetworkInfo[] = [
      {
        id: this.generateId('network'),
        name: 'bridge',
        driver: 'bridge',
        scope: 'local',
        ipv6: false,
        internal: false,
        containers: {},
        created: new Date(),
      },
    ];

    return networks;
  }

  /**
   * Remove a network
   *
   * @param networkId - Network ID or name
   */
  async removeNetwork(networkId: string): Promise<void> {
    // Simulate network removal
    await this.sleep(100);
  }

  // ============================================================================
  // Volume Management
  // ============================================================================

  /**
   * Create a volume
   *
   * @param config - Volume configuration
   * @returns Volume information
   */
  async createVolume(config: VolumeConfig): Promise<VolumeInfo> {
    const volume: VolumeInfo = {
      name: config.name,
      driver: config.driver || 'local',
      mountpoint: `/var/lib/docker/volumes/${config.name}/_data`,
      created: new Date(),
      labels: config.labels || {},
      scope: 'local',
    };

    return volume;
  }

  /**
   * List volumes
   *
   * @returns Array of volume information
   */
  async listVolumes(): Promise<VolumeInfo[]> {
    const volumes: VolumeInfo[] = [
      {
        name: 'app-data',
        driver: 'local',
        mountpoint: '/var/lib/docker/volumes/app-data/_data',
        created: new Date(),
        labels: {},
        scope: 'local',
      },
    ];

    return volumes;
  }

  /**
   * Remove a volume
   *
   * @param volumeName - Volume name
   * @param force - Force removal
   */
  async removeVolume(volumeName: string, force = false): Promise<void> {
    // Simulate volume removal
    await this.sleep(100);
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Generate unique ID
   */
  private generateId(prefix: string): string {
    const random = Math.random().toString(36).substring(2, 15);
    const timestamp = Date.now().toString(36);
    return `${prefix}-${timestamp}-${random}`;
  }

  /**
   * Sleep utility
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Format bytes to human-readable
   */
  formatBytes(bytes: number): string {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KB`;
    if (bytes < 1024 * 1024 * 1024)
      return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
    return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GB`;
  }

  /**
   * Format resource limit string to bytes
   */
  parseResourceLimit(limit: string): number {
    const units: Record<string, number> = {
      K: 1024,
      Ki: 1024,
      M: 1024 * 1024,
      Mi: 1024 * 1024,
      G: 1024 * 1024 * 1024,
      Gi: 1024 * 1024 * 1024,
    };

    const match = limit.match(/^(\d+(?:\.\d+)?)\s*([KMG]i?)$/);
    if (!match) {
      throw new ContainerError(
        ContainerErrorCode.INVALID_CONFIG,
        `Invalid resource limit: ${limit}`
      );
    }

    const value = parseFloat(match[1]);
    const unit = match[2];

    return Math.floor(value * units[unit]);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create and start a container (standalone)
 */
export async function runContainer(
  config: ContainerConfig
): Promise<ContainerInfo> {
  const sdk = new ContainerSDK();
  return await sdk.runContainer(config);
}

/**
 * Build an image (standalone)
 */
export async function buildImage(config: BuildConfig): Promise<ImageInfo> {
  const sdk = new ContainerSDK();
  return await sdk.buildImage(config);
}

/**
 * Create a network (standalone)
 */
export async function createNetwork(
  config: NetworkConfig
): Promise<NetworkInfo> {
  const sdk = new ContainerSDK();
  return await sdk.createNetwork(config);
}

/**
 * Create a volume (standalone)
 */
export async function createVolume(config: VolumeConfig): Promise<VolumeInfo> {
  const sdk = new ContainerSDK();
  return await sdk.createVolume(config);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { ContainerSDK };
