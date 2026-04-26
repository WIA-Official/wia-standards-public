/**
 * WIA-COMM-011: Edge Computing SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Edge Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for edge computing including:
 * - Latency calculation
 * - Workload placement optimization
 * - Edge AI/ML inference
 * - MEC service integration
 * - Resource management
 */

import {
  EdgeNode,
  EdgeTier,
  NetworkType,
  LatencyParams,
  LatencyResult,
  WorkloadSpec,
  WorkloadPlacement,
  DeploymentResult,
  EdgeAIConfig,
  InferenceRequest,
  InferenceResult,
  OptimizationRequest,
  OptimizationResult,
  EdgeMetrics,
  RadioNetworkInfo,
  LocationRequest,
  LocationResponse,
  PlacementStrategy,
  AIAccelerator,
  EDGE_CONSTANTS,
  EdgeErrorCode,
  EdgeComputingError,
  GeoLocation,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-011 Edge Computing SDK
 */
export class EdgeComputingSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Calculate edge latency
   *
   * @param params - Latency calculation parameters
   * @returns Latency calculation result
   */
  calculateEdgeLatency(params: LatencyParams): LatencyResult {
    const {
      sourceType,
      targetType,
      distance,
      networkType,
      processingTime = 0,
      queueingDelay = 0,
    } = params;

    // Network latency calculation
    const baseLatency = EDGE_CONSTANTS.NETWORK_LATENCY[networkType];

    // Propagation delay (distance / speed of light)
    // Add 50% overhead for non-direct paths
    const propagationDelay =
      (distance / EDGE_CONSTANTS.SPEED_OF_LIGHT) * 1000 * 1.5;

    // Transmission delay (simplified, assumes 1500 byte packet at 1 Gbps)
    const transmissionDelay = 0.012; // ms

    // Additional latency based on tier difference
    const tierLatency = this.calculateTierLatency(sourceType, targetType);

    const networkLatency =
      baseLatency + propagationDelay + transmissionDelay + tierLatency;

    const totalLatency =
      networkLatency + processingTime + queueingDelay;

    const breakdown = {
      propagation: propagationDelay,
      transmission: transmissionDelay,
      processing: processingTime,
      queueing: queueingDelay,
    };

    return {
      network: networkLatency,
      processing: processingTime,
      queueing: queueingDelay,
      total: totalLatency,
      breakdown,
    };
  }

  /**
   * Calculate additional latency based on tier difference
   */
  private calculateTierLatency(
    source: EdgeTier,
    target: EdgeTier
  ): number {
    const tierOrder: EdgeTier[] = ['device', 'access', 'regional', 'cloud'];
    const sourceIndex = tierOrder.indexOf(source);
    const targetIndex = tierOrder.indexOf(target);
    const diff = Math.abs(targetIndex - sourceIndex);

    // Each tier hop adds latency
    return diff * 2; // ms per tier
  }

  /**
   * Calculate distance between two geographic locations
   *
   * @param loc1 - First location
   * @param loc2 - Second location
   * @returns Distance in meters
   */
  calculateDistance(loc1: GeoLocation, loc2: GeoLocation): number {
    const R = 6371000; // Earth's radius in meters
    const φ1 = (loc1.latitude * Math.PI) / 180;
    const φ2 = (loc2.latitude * Math.PI) / 180;
    const Δφ = ((loc2.latitude - loc1.latitude) * Math.PI) / 180;
    const Δλ = ((loc2.longitude - loc1.longitude) * Math.PI) / 180;

    const a =
      Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
      Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
  }

  /**
   * Optimize workload placement
   *
   * @param request - Optimization request
   * @returns Optimization result
   */
  optimizePlacement(request: OptimizationRequest): OptimizationResult {
    const { workload, objective, constraints, availableNodes } = request;

    // Filter nodes based on constraints
    const eligibleNodes = this.filterNodesByConstraints(
      availableNodes,
      constraints
    );

    if (eligibleNodes.length === 0) {
      throw new EdgeComputingError(
        EdgeErrorCode.INVALID_PLACEMENT,
        'No eligible nodes found matching constraints'
      );
    }

    // Score each node based on objective
    const scoredNodes = eligibleNodes.map((node) => {
      const score = this.scoreNode(node, workload, objective, constraints);
      const latency = this.estimateNodeLatency(node, constraints);
      const cost = this.estimateNodeCost(node, workload);

      return {
        nodeId: node.id,
        node,
        latency,
        cost,
        score,
      };
    });

    // Sort by score (descending)
    scoredNodes.sort((a, b) => b.score - a.score);

    const best = scoredNodes[0];
    const alternatives = scoredNodes.slice(1, 4).map((n) => ({
      nodeId: n.nodeId,
      latency: n.latency,
      cost: n.cost,
      score: n.score,
    }));

    return {
      nodeId: best.nodeId,
      expectedLatency: best.latency,
      expectedCost: best.cost,
      expectedAvailability: this.estimateAvailability(best.node),
      score: best.score,
      alternatives,
      reasoning: this.generateReasoning(best.node, objective),
    };
  }

  /**
   * Filter nodes by constraints
   */
  private filterNodesByConstraints(
    nodes: EdgeNode[],
    constraints: OptimizationRequest['constraints']
  ): EdgeNode[] {
    return nodes.filter((node) => {
      // Check status
      if (node.status !== 'active') return false;

      // Check tier
      if (constraints.tier && node.tier !== constraints.tier) return false;

      // Check node type
      if (constraints.nodeType && node.type !== constraints.nodeType)
        return false;

      // Check GPU requirement
      if (constraints.requireGPU && !node.resources.gpu) return false;

      // Check location distance
      if (constraints.location && constraints.maxDistance) {
        const distance = this.calculateDistance(
          node.location,
          constraints.location
        );
        if (distance > constraints.maxDistance) return false;
      }

      return true;
    });
  }

  /**
   * Score a node based on objective
   */
  private scoreNode(
    node: EdgeNode,
    workload: WorkloadSpec,
    objective: OptimizationRequest['objective'],
    constraints: OptimizationRequest['constraints']
  ): number {
    let score = 0;

    switch (objective) {
      case 'minimize-latency':
        // Lower latency = higher score
        const latency = this.estimateNodeLatency(node, constraints);
        score = 100 - latency * 2; // Approximate scoring
        break;

      case 'minimize-cost':
        // Lower cost = higher score
        const cost = this.estimateNodeCost(node, workload);
        score = 100 - cost * 10; // Approximate scoring
        break;

      case 'maximize-availability':
        score = node.health;
        break;
    }

    // Penalize high utilization
    const avgUtilization =
      (node.utilization.cpu + node.utilization.memory) / 2;
    score *= 1 - avgUtilization * 0.3;

    return Math.max(0, Math.min(100, score));
  }

  /**
   * Estimate latency to a node
   */
  private estimateNodeLatency(
    node: EdgeNode,
    constraints: OptimizationRequest['constraints']
  ): number {
    // Base latency from node's network
    let latency = node.network.latency;

    // Add distance-based latency if location constraint exists
    if (constraints.location) {
      const distance = this.calculateDistance(
        node.location,
        constraints.location
      );
      const propagationDelay =
        (distance / EDGE_CONSTANTS.SPEED_OF_LIGHT) * 1000;
      latency += propagationDelay;
    }

    // Add tier-based latency
    latency += EDGE_CONSTANTS.LATENCY_TARGETS[node.tier];

    return latency;
  }

  /**
   * Estimate cost per hour for a workload on a node
   */
  private estimateNodeCost(
    node: EdgeNode,
    workload: WorkloadSpec
  ): number {
    // Simplified cost model ($/hour)
    const cpuCost = parseFloat(workload.resources.requests.cpu) * 0.05;
    const memCost =
      parseFloat(workload.resources.requests.memory.replace(/[^0-9.]/g, '')) *
      0.01;
    const gpuCost = (workload.resources.requests.gpu || 0) * 0.5;

    // Edge nodes are typically more expensive than cloud
    const edgePremium = node.tier === 'access' ? 1.5 : 1.2;

    return (cpuCost + memCost + gpuCost) * edgePremium;
  }

  /**
   * Estimate node availability
   */
  private estimateAvailability(node: EdgeNode): number {
    // Base availability from health score
    const baseAvailability = 0.99 + node.health / 10000;

    // Adjust based on tier (edge nodes typically have slightly lower availability)
    const tierFactor =
      node.tier === 'cloud' ? 1.0 : node.tier === 'regional' ? 0.999 : 0.998;

    return Math.min(0.9999, baseAvailability * tierFactor);
  }

  /**
   * Generate reasoning for placement decision
   */
  private generateReasoning(
    node: EdgeNode,
    objective: OptimizationRequest['objective']
  ): string {
    const reasons: string[] = [];

    reasons.push(`Selected ${node.type} node at ${node.tier} tier`);

    if (objective === 'minimize-latency') {
      reasons.push(
        `Optimized for low latency (~${node.network.latency.toFixed(1)}ms)`
      );
    } else if (objective === 'minimize-cost') {
      reasons.push('Optimized for cost efficiency');
    } else {
      reasons.push(`High availability node (health: ${node.health}%)`);
    }

    const avgUtil = (node.utilization.cpu + node.utilization.memory) / 2;
    reasons.push(`Current utilization: ${(avgUtil * 100).toFixed(0)}%`);

    return reasons.join('. ');
  }

  /**
   * Deploy workload to edge
   *
   * @param placement - Workload placement specification
   * @returns Deployment result
   */
  async deployWorkload(
    placement: WorkloadPlacement
  ): Promise<DeploymentResult> {
    const { workload, placement: placementConfig } = placement;

    // This is a simplified simulation
    // In production, this would call Kubernetes API or edge orchestrator

    const deploymentId = this.generateDeploymentId();

    // Simulate deployment
    const result: DeploymentResult = {
      deploymentId,
      status: 'running',
      nodes: ['simulated-node-1'],
      latency: 5, // Simulated
      allocatedResources: workload.resources.requests,
      costPerHour: 0.5, // Simulated
      deploymentTime: new Date(),
    };

    return result;
  }

  /**
   * Generate unique deployment ID
   */
  private generateDeploymentId(): string {
    return `edge-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

// ============================================================================
// Edge AI Class
// ============================================================================

/**
 * Edge AI inference engine
 */
export class EdgeAI {
  private config: EdgeAIConfig;

  constructor(config: EdgeAIConfig) {
    this.config = config;
  }

  /**
   * Perform inference
   *
   * @param request - Inference request
   * @returns Inference result
   */
  async infer(request: InferenceRequest): Promise<InferenceResult> {
    const startTime = Date.now();

    // Simulate inference
    // In production, this would call TensorFlow Lite, ONNX Runtime, etc.

    const baseLatency =
      EDGE_CONSTANTS.AI_LATENCY[this.config.accelerator] || 50;

    // Quantization reduces latency
    const quantizationSpeedup =
      this.config.quantization === 'int8'
        ? 0.25
        : this.config.quantization === 'fp16'
        ? 0.5
        : 1.0;

    const latency = baseLatency * quantizationSpeedup;

    // Simulate processing delay
    await new Promise((resolve) => setTimeout(resolve, latency));

    const result: InferenceResult = {
      output: { prediction: 'simulated-result' },
      latency,
      confidence: 0.95,
      modelVersion: '1.0.0',
      accelerator: this.config.accelerator,
      timestamp: new Date(),
    };

    return result;
  }

  /**
   * Get model info
   */
  getModelInfo() {
    return {
      model: this.config.model,
      accelerator: this.config.accelerator,
      quantization: this.config.quantization,
      framework: this.config.framework || 'tensorflow-lite',
    };
  }
}

// ============================================================================
// MEC Service Class
// ============================================================================

/**
 * Multi-access Edge Computing (MEC) services
 */
export class MECService {
  /**
   * Get radio network information
   *
   * @param ueId - User equipment ID
   * @returns Radio network information
   */
  async getRadioNetworkInfo(ueId: string): Promise<RadioNetworkInfo> {
    // Simulate RNIS query
    // In production, this would query the MEC platform

    return {
      ueId,
      cellId: '310-410-0x1234',
      cellLoad: Math.random() * 100,
      signalStrength: -75 - Math.random() * 20,
      rsrq: -10 - Math.random() * 5,
      rsrp: -80 - Math.random() * 20,
      latency: 1 + Math.random() * 4,
      throughput: 50 + Math.random() * 150,
    };
  }

  /**
   * Get device location
   *
   * @param request - Location request
   * @returns Location response
   */
  async getLocation(request: LocationRequest): Promise<LocationResponse> {
    // Simulate location service
    // In production, this would query the MEC location service

    return {
      location: {
        latitude: 37.7749 + (Math.random() - 0.5) * 0.1,
        longitude: -122.4194 + (Math.random() - 0.5) * 0.1,
        altitude: 10,
      },
      accuracy: request.accuracy || 10,
      timestamp: new Date(),
      method: 'hybrid',
    };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate edge data rate
 */
export function calculateEdgeDataRate(params: {
  bandwidth: number; // Hz
  snr: number; // dB
  efficiency?: number; // 0-1
}): number {
  const { bandwidth, snr, efficiency = 0.8 } = params;

  // Shannon capacity: C = B × log₂(1 + SNR)
  const snrLinear = Math.pow(10, snr / 10);
  const theoreticalCapacity = bandwidth * Math.log2(1 + snrLinear);

  // Apply efficiency factor for real-world conditions
  return theoreticalCapacity * efficiency;
}

/**
 * Estimate edge processing time
 */
export function estimateProcessingTime(params: {
  dataSize: number; // bytes
  cpuSpeed: number; // operations per second
  complexity: number; // operations per byte
}): number {
  const { dataSize, cpuSpeed, complexity } = params;

  const totalOperations = dataSize * complexity;
  const timeSeconds = totalOperations / cpuSpeed;

  return timeSeconds * 1000; // Convert to ms
}

/**
 * Format latency for display
 */
export function formatLatency(latencyMs: number): string {
  if (latencyMs < 1) {
    return `${(latencyMs * 1000).toFixed(0)}μs`;
  } else if (latencyMs < 1000) {
    return `${latencyMs.toFixed(2)}ms`;
  } else {
    return `${(latencyMs / 1000).toFixed(2)}s`;
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export {
  EdgeComputingSDK,
  EdgeAI,
  MECService,
  calculateEdgeDataRate,
  estimateProcessingTime,
  formatLatency,
};
