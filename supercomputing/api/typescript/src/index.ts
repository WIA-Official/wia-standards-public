/**
 * WIA-COMP-001: Supercomputing SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for supercomputing including:
 * - FLOPS calculations
 * - Scalability analysis
 * - Performance benchmarking
 * - Energy efficiency estimation
 * - Workload optimization
 */

import {
  FLOPSParams,
  FLOPSResult,
  ClusterConfig,
  TopologyValidation,
  ScalabilityParams,
  ScalabilityResult,
  BenchmarkConfig,
  BenchmarkResult,
  EnergyParams,
  EnergyResult,
  CheckpointConfig,
  ReliabilityAnalysis,
  HPC_CONSTANTS,
  CompErrorCode,
  SupercomputingError,
  PerformanceTier,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMP-001 Supercomputing SDK
 */
export class SupercomputingSDK {
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
   * Calculate theoretical and sustained FLOPS
   *
   * @param params - FLOPS calculation parameters
   * @returns FLOPS calculation result
   */
  calculateFLOPS(params: FLOPSParams): FLOPSResult {
    const {
      nodes,
      coresPerNode,
      clockSpeed,
      flopPerCycle,
      accelerators,
      vectorizationEfficiency = 0.9,
      memoryEfficiency = 0.8,
    } = params;

    // Validate inputs
    if (nodes <= 0 || coresPerNode <= 0 || clockSpeed <= 0 || flopPerCycle <= 0) {
      throw new SupercomputingError(
        CompErrorCode.INVALID_PARAMETERS,
        'All performance parameters must be positive'
      );
    }

    // Calculate CPU FLOPS
    const cpuFLOPS = nodes * coresPerNode * clockSpeed * flopPerCycle;

    // Calculate accelerator FLOPS
    let acceleratorFLOPS = 0;
    if (accelerators) {
      const tflopsPerAccelerator = accelerators.peakTFLOPS * 1e12;
      acceleratorFLOPS = nodes * accelerators.count * tflopsPerAccelerator;
    }

    // Total peak FLOPS
    const peakFLOPS = cpuFLOPS + acceleratorFLOPS;

    // Sustained FLOPS (accounting for inefficiencies)
    const overallEfficiency = vectorizationEfficiency * memoryEfficiency;
    const sustainedFLOPS = peakFLOPS * overallEfficiency;

    // Determine performance tier
    const tier = this.determinePerformanceTier(peakFLOPS);

    return {
      peakFLOPS,
      cpuFLOPS,
      acceleratorFLOPS,
      tier,
      sustainedFLOPS,
      efficiency: overallEfficiency,
      formatted: this.formatFLOPS(peakFLOPS),
    };
  }

  /**
   * Validate cluster configuration
   *
   * @param config - Cluster configuration
   * @returns Validation result with recommendations
   */
  validateClusterConfig(config: ClusterConfig): TopologyValidation {
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Check node count
    if (config.nodes < 100) {
      warnings.push('Small cluster (<100 nodes) may not require complex topology');
    } else if (config.nodes > 100000) {
      warnings.push('Very large cluster (>100k nodes) requires careful topology design');
    }

    // Check interconnect bandwidth
    const bitsPerSecond = config.interconnect.bandwidth;
    const gbitsPerSecond = bitsPerSecond / 1e9;

    if (gbitsPerSecond < 100) {
      warnings.push(`Low interconnect bandwidth (${gbitsPerSecond} Gb/s)`);
      recommendations.push('Consider upgrading to InfiniBand HDR (200 Gb/s) or higher');
    }

    // Check latency
    const latencyMicroseconds = config.interconnect.latency * 1e6;
    if (latencyMicroseconds > 5) {
      warnings.push(`High network latency (${latencyMicroseconds.toFixed(2)} μs)`);
      recommendations.push('Optimize network topology or use RDMA-capable interconnect');
    }

    // Check PUE
    if (config.infrastructure.pue > 1.5) {
      warnings.push(`Poor power efficiency (PUE: ${config.infrastructure.pue})`);
      recommendations.push('Improve cooling efficiency to reduce PUE below 1.3');
    }

    // Calculate topology metrics
    const diameter = this.estimateDiameter(config.interconnect.topology, config.nodes);
    const bisectionBandwidth = this.estimateBisectionBandwidth(
      config.interconnect.topology,
      config.nodes,
      bitsPerSecond
    );

    const isValid = warnings.length < 3; // Arbitrary threshold

    return {
      isValid,
      metrics: {
        diameter,
        bisectionBandwidth,
        avgHops: diameter / 2,
        linkUtilization: 0.7, // Placeholder
      },
      warnings,
      recommendations,
    };
  }

  /**
   * Analyze application scalability
   *
   * @param params - Scalability parameters
   * @returns Scalability analysis result
   */
  estimateScalability(params: ScalabilityParams): ScalabilityResult {
    const { baseline, target, scalingType, algorithm } = params;

    const nodeRatio = target.nodes / baseline.nodes;
    const bottlenecks: string[] = [];

    let speedup: number;
    let estimatedTime: number;

    if (scalingType === 'strong') {
      // Strong scaling: fixed problem size
      const serialFraction = algorithm.serialFraction || 0.05;

      // Amdahl's Law: Speedup = 1 / (s + (1-s)/p)
      speedup = 1 / (serialFraction + (1 - serialFraction) / nodeRatio);

      // Communication overhead
      if (algorithm.commOverhead) {
        const { latency, bandwidthPerByte, messageSize, messagesPerIteration } =
          algorithm.commOverhead;
        const commTime = messagesPerIteration * (latency + messageSize * bandwidthPerByte);
        const commOverheadRatio = commTime / baseline.time;

        // Reduce speedup by communication overhead
        speedup *= 1 - commOverheadRatio * Math.log(nodeRatio);

        if (commOverheadRatio > 0.2) {
          bottlenecks.push('High communication overhead');
        }
      }

      estimatedTime = baseline.time / speedup;

      if (serialFraction > 0.1) {
        bottlenecks.push(`High serial fraction (${(serialFraction * 100).toFixed(1)}%)`);
      }
    } else {
      // Weak scaling: problem size scales with processors
      // Ideal: constant time
      const overhead = algorithm.commOverhead ? 0.1 * Math.log(nodeRatio) : 0;
      estimatedTime = baseline.time * (1 + overhead);
      speedup = baseline.time / estimatedTime;
    }

    const efficiency = speedup / nodeRatio;

    // Determine assessment
    let assessment: 'excellent' | 'good' | 'moderate' | 'poor';
    if (efficiency >= 0.9) {
      assessment = 'excellent';
    } else if (efficiency >= 0.7) {
      assessment = 'good';
    } else if (efficiency >= 0.5) {
      assessment = 'moderate';
    } else {
      assessment = 'poor';
      bottlenecks.push('Low parallel efficiency');
    }

    // Recommended nodes (where efficiency drops below 70%)
    const recommendedNodes = this.findOptimalNodeCount(baseline, algorithm, 0.7);

    return {
      estimatedTime,
      speedup,
      efficiency,
      assessment,
      recommendedNodes,
      bottlenecks,
    };
  }

  /**
   * Run performance benchmark
   *
   * @param config - Benchmark configuration
   * @returns Benchmark result
   */
  benchmarkPerformance(config: BenchmarkConfig): BenchmarkResult {
    const { type, problemSize, nodes, coresPerNode } = config;

    const details: string[] = [];

    // Simulate benchmark execution
    let performance: number;
    let theoretical: number;
    let efficiency: number;
    let executionTime: number;
    const metrics: Record<string, number> = {};

    switch (type) {
      case 'LINPACK':
        // LINPACK solves dense linear systems
        // Performance: 2/3 * N^3 / time FLOPS
        theoretical = nodes * coresPerNode * 2.5e9 * 32; // Assume 2.5 GHz, 32 FLOP/cycle
        efficiency = 0.75; // Typical LINPACK efficiency
        performance = theoretical * efficiency;

        // Time ≈ 2/3 * N^3 / performance
        const ops = (2 / 3) * Math.pow(problemSize, 3);
        executionTime = ops / performance;

        metrics.problemSize = problemSize;
        metrics.rMax = performance;
        metrics.rPeak = theoretical;

        details.push(`Solved ${problemSize}x${problemSize} dense linear system`);
        details.push(`Achieved ${(efficiency * 100).toFixed(1)}% of peak performance`);
        break;

      case 'HPCG':
        // HPCG: sparse iterative solver
        theoretical = nodes * coresPerNode * 2.5e9 * 32;
        efficiency = 0.03; // HPCG is much harder to optimize
        performance = theoretical * efficiency;
        executionTime = 1800; // 30 minutes typical

        metrics.iterations = 50;
        metrics.convergence = 1e-12;

        details.push('Sparse conjugate gradient benchmark');
        details.push('More realistic than LINPACK for many applications');
        break;

      case 'STREAM':
        // Memory bandwidth benchmark
        const memBandwidthPerNode = 200e9; // 200 GB/s
        performance = nodes * memBandwidthPerNode; // bytes/sec
        theoretical = nodes * 300e9; // theoretical max
        efficiency = performance / theoretical;
        executionTime = 60; // 1 minute

        metrics.copy = performance * 0.8;
        metrics.scale = performance * 0.85;
        metrics.add = performance * 0.9;
        metrics.triad = performance * 0.95;

        details.push('Memory bandwidth test');
        break;

      case 'IOR':
        // I/O benchmark
        const ioPerNode = 5e9; // 5 GB/s per node
        performance = Math.min(nodes * ioPerNode, 100e9); // capped by storage
        theoretical = 100e9; // PFS limit
        efficiency = performance / theoretical;
        executionTime = (problemSize * 1e9) / performance; // assuming GB

        metrics.writeRate = performance;
        metrics.readRate = performance * 1.2;

        details.push('Parallel I/O benchmark');
        break;

      default:
        throw new SupercomputingError(
          CompErrorCode.BENCHMARK_FAILED,
          `Unknown benchmark type: ${type}`
        );
    }

    const passed = efficiency > 0.5; // Arbitrary threshold

    return {
      type,
      performance,
      theoretical,
      efficiency,
      executionTime,
      metrics,
      passed,
      details,
    };
  }

  /**
   * Analyze energy consumption
   *
   * @param params - Energy analysis parameters
   * @returns Energy consumption analysis
   */
  analyzeEnergy(params: EnergyParams): EnergyResult {
    const { system, workload, powerManagement } = params;

    // Calculate power consumption for each component
    const { nodes, nodeConfig, interconnect, storage, infrastructure } = system;

    // Compute power
    const cpuUtilization = workload.utilizationCPU;
    const cpuPowerPerNode =
      nodeConfig.power.idle + (nodeConfig.power.max - nodeConfig.power.idle) * cpuUtilization;

    let gpuPowerPerNode = 0;
    if (nodeConfig.accelerators && workload.utilizationGPU) {
      const gpuCount = nodeConfig.accelerators.count;
      const gpuPowerEach = 300; // watts (typical)
      gpuPowerPerNode = gpuCount * gpuPowerEach * workload.utilizationGPU;
    }

    const computePower = nodes * (cpuPowerPerNode + gpuPowerPerNode);

    // Network power (10% of compute)
    const networkPower = computePower * 0.1;

    // Storage power (5% of compute)
    const storagePower = computePower * 0.05;

    // Total IT power
    const itPower = computePower + networkPower + storagePower;

    // Cooling and other (PUE factor)
    const totalPower = itPower * infrastructure.pue;
    const coolingPower = totalPower - itPower;

    // Total energy over duration
    const totalEnergy = totalPower * workload.duration;

    // FLOPS/Watt calculation
    const peakFLOPS = this.calculateFLOPS({
      nodes,
      coresPerNode: nodeConfig.cpu.coresPerSocket * nodeConfig.cpu.sockets,
      clockSpeed: nodeConfig.cpu.clockSpeed,
      flopPerCycle: nodeConfig.cpu.flopPerCycle,
      accelerators: nodeConfig.accelerators,
    }).peakFLOPS;

    const flopsPerWatt = peakFLOPS / totalPower;

    // Cost estimate
    const energyKWh = totalEnergy / 3600 / 1000; // joules to kWh
    const energyCost = energyKWh * HPC_CONSTANTS.ENERGY_COST_PER_KWH;
    const carbonFootprint = energyKWh * HPC_CONSTANTS.CO2_PER_KWH;

    return {
      totalEnergy,
      averagePower: totalPower,
      breakdown: {
        compute: computePower,
        network: networkPower,
        storage: storagePower,
        cooling: coolingPower,
        other: 0,
      },
      metrics: {
        flopsPerWatt,
        pue: infrastructure.pue,
        dcie: 1 / infrastructure.pue, // Data Center Infrastructure Efficiency
      },
      costEstimate: {
        energyCost,
        carbonFootprint,
      },
    };
  }

  /**
   * Optimize checkpoint strategy
   *
   * @param config - Checkpoint configuration
   * @param nodes - Number of nodes
   * @param jobDuration - Expected job duration (seconds)
   * @returns Reliability analysis with recommendations
   */
  optimizeCheckpointing(
    config: CheckpointConfig,
    nodes: number,
    jobDuration: number
  ): ReliabilityAnalysis {
    // Estimate system MTBF
    const nodeYearMTBF = 5; // 5 years per node
    const nodeSecondMTBF = nodeYearMTBF * 365.25 * 24 * 3600;
    const mtbf = nodeSecondMTBF / nodes;

    // Checkpoint overhead
    const checkpointTime = config.dataSize / (1e9); // assume 1 GB/s I/O
    const checkpointsPerJob = jobDuration / config.interval;
    const totalCheckpointTime = checkpointsPerJob * checkpointTime;
    const checkpointOverhead = totalCheckpointTime / jobDuration;

    // Expected failures
    const expectedFailures = jobDuration / mtbf;

    // Optimal checkpoint interval (Young's formula)
    const optimalInterval = Math.sqrt(2 * checkpointTime * mtbf);

    // Estimated total time including restarts
    const wastedWorkPerFailure = config.interval / 2; // on average
    const restartOverhead = expectedFailures * (wastedWorkPerFailure + checkpointTime);
    const estimatedTotalTime = jobDuration + totalCheckpointTime + restartOverhead;

    return {
      mtbf,
      optimalCheckpointInterval: optimalInterval,
      expectedFailures,
      checkpointOverhead,
      estimatedTotalTime,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Determine performance tier based on FLOPS
   */
  private determinePerformanceTier(flops: number): PerformanceTier {
    if (flops >= HPC_CONSTANTS.EXAFLOPS) {
      return 'exascale';
    } else if (flops >= HPC_CONSTANTS.PETAFLOPS) {
      return 'petascale';
    } else if (flops >= HPC_CONSTANTS.TERAFLOPS) {
      return 'terascale';
    } else {
      return 'high-performance';
    }
  }

  /**
   * Format FLOPS for human readability
   */
  private formatFLOPS(flops: number): string {
    if (flops >= HPC_CONSTANTS.EXAFLOPS) {
      return `${(flops / HPC_CONSTANTS.EXAFLOPS).toFixed(2)} exaFLOPS`;
    } else if (flops >= HPC_CONSTANTS.PETAFLOPS) {
      return `${(flops / HPC_CONSTANTS.PETAFLOPS).toFixed(2)} petaFLOPS`;
    } else if (flops >= HPC_CONSTANTS.TERAFLOPS) {
      return `${(flops / HPC_CONSTANTS.TERAFLOPS).toFixed(2)} teraFLOPS`;
    } else if (flops >= HPC_CONSTANTS.GIGAFLOPS) {
      return `${(flops / HPC_CONSTANTS.GIGAFLOPS).toFixed(2)} gigaFLOPS`;
    } else {
      return `${flops.toFixed(2)} FLOPS`;
    }
  }

  /**
   * Estimate network diameter based on topology
   */
  private estimateDiameter(
    topology: string,
    nodes: number
  ): number {
    switch (topology) {
      case 'fat-tree':
        return Math.ceil(Math.log2(nodes)) * 2;
      case 'dragonfly':
        return 3; // Typically 3 hops max
      case 'torus':
        const dimension = 3; // 3D torus
        const nodesPerDim = Math.pow(nodes, 1 / dimension);
        return dimension * (nodesPerDim / 2);
      default:
        return Math.log2(nodes); // Generic estimate
    }
  }

  /**
   * Estimate bisection bandwidth
   */
  private estimateBisectionBandwidth(
    topology: string,
    nodes: number,
    linkBandwidth: number
  ): number {
    switch (topology) {
      case 'fat-tree':
        return (nodes / 2) * linkBandwidth; // Full bisection
      case 'dragonfly':
        return (nodes / 4) * linkBandwidth; // Approximate
      case 'torus':
        const dimension = 3;
        const nodesPerDim = Math.pow(nodes, 1 / dimension);
        return nodesPerDim ** (dimension - 1) * linkBandwidth;
      default:
        return (nodes / 2) * linkBandwidth;
    }
  }

  /**
   * Find optimal node count for target efficiency
   */
  private findOptimalNodeCount(
    baseline: { nodes: number; time: number },
    algorithm: { serialFraction?: number },
    targetEfficiency: number
  ): number {
    const serialFraction = algorithm.serialFraction || 0.05;

    // Binary search for node count where efficiency = target
    let low = baseline.nodes;
    let high = baseline.nodes * 1000;

    while (high - low > 1) {
      const mid = Math.floor((low + high) / 2);
      const nodeRatio = mid / baseline.nodes;
      const speedup = 1 / (serialFraction + (1 - serialFraction) / nodeRatio);
      const efficiency = speedup / nodeRatio;

      if (efficiency >= targetEfficiency) {
        low = mid;
      } else {
        high = mid;
      }
    }

    return low;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate FLOPS (standalone)
 */
export function calculateFLOPS(params: FLOPSParams): FLOPSResult {
  const sdk = new SupercomputingSDK();
  return sdk.calculateFLOPS(params);
}

/**
 * Validate cluster configuration (standalone)
 */
export function validateClusterConfig(config: ClusterConfig): TopologyValidation {
  const sdk = new SupercomputingSDK();
  return sdk.validateClusterConfig(config);
}

/**
 * Estimate scalability (standalone)
 */
export function estimateScalability(params: ScalabilityParams): ScalabilityResult {
  const sdk = new SupercomputingSDK();
  return sdk.estimateScalability(params);
}

/**
 * Benchmark performance (standalone)
 */
export function benchmarkPerformance(config: BenchmarkConfig): BenchmarkResult {
  const sdk = new SupercomputingSDK();
  return sdk.benchmarkPerformance(config);
}

/**
 * Analyze energy consumption (standalone)
 */
export function analyzeEnergy(params: EnergyParams): EnergyResult {
  const sdk = new SupercomputingSDK();
  return sdk.analyzeEnergy(params);
}

/**
 * Optimize checkpointing (standalone)
 */
export function optimizeCheckpointing(
  config: CheckpointConfig,
  nodes: number,
  jobDuration: number
): ReliabilityAnalysis {
  const sdk = new SupercomputingSDK();
  return sdk.optimizeCheckpointing(config, nodes, jobDuration);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { SupercomputingSDK };
