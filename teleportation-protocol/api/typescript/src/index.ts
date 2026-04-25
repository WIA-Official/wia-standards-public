/**
 * WIA-QUA-011: Teleportation Protocol SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Teleportation Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for quantum teleportation operations including:
 * - Single-qubit and multi-qubit quantum state teleportation
 * - Bell state measurement and analysis
 * - Continuous variable teleportation
 * - Teleportation networks and routing
 * - Fidelity verification and optimization
 */

import {
  QuantumState,
  QuantumTeleportationParams,
  QuantumTeleportationResult,
  MultiQubitTeleportationParams,
  MultiQubitTeleportationResult,
  CVTeleportationParams,
  CVTeleportationResult,
  BellMeasurementResult,
  BellStateType,
  PauliOperator,
  EntanglementPair,
  EntanglementGenerationParams,
  TeleportationNode,
  TeleportationNetworkParams,
  NetworkTeleportationParams,
  TeleportationPath,
  FidelityParams,
  FidelityResult,
  VerificationParams,
  VerificationResult,
  TeleportationMetrics,
  BenchmarkParams,
  BenchmarkResult,
  TeleportationProtocol,
  NetworkTopology,
  Complex,
  TELEPORTATION_CONSTANTS,
  TeleportationErrorCode,
  TeleportationError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-011 Teleportation Protocol SDK
 */
export class TeleportationSDK {
  private version = '1.0.0';
  private nodes: Map<string, TeleportationNode> = new Map();
  private entanglements: Map<string, EntanglementPair> = new Map();
  private metrics: TeleportationMetrics = this.initializeMetrics();

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Quantum State Teleportation
  // ==========================================================================

  /**
   * Teleport quantum state using standard protocol
   *
   * @param params - Teleportation parameters
   * @returns Teleportation result
   */
  async teleportQuantumState(
    params: QuantumTeleportationParams
  ): Promise<QuantumTeleportationResult> {
    const startTime = Date.now();

    const {
      state,
      from,
      to,
      protocol = 'standard',
      entanglementId,
      verifyFidelity = true,
      minFidelity = TELEPORTATION_CONSTANTS.CLASSICAL_FIDELITY_LIMIT,
    } = params;

    // Validate state
    this.validateQuantumState(state);

    // Get entanglement resource
    const entanglement = this.entanglements.get(entanglementId);
    if (!entanglement || entanglement.status !== 'active') {
      throw new TeleportationError(
        TeleportationErrorCode.ENTANGLEMENT_UNAVAILABLE,
        `Entanglement ${entanglementId} not available`
      );
    }

    // Verify entanglement quality
    if (entanglement.fidelity < TELEPORTATION_CONSTANTS.MIN_ENTANGLEMENT_FIDELITY) {
      throw new TeleportationError(
        TeleportationErrorCode.LOW_FIDELITY,
        `Entanglement fidelity ${entanglement.fidelity} below minimum threshold`
      );
    }

    // Perform Bell state measurement
    const bellMeasurement = await this.performBellMeasurement(state, entanglement);

    if (!bellMeasurement.success) {
      throw new TeleportationError(
        TeleportationErrorCode.BSM_FAILED,
        'Bell state measurement failed'
      );
    }

    // Simulate classical communication latency
    const distance = this.calculateDistance(from, to);
    const classicalLatency = this.calculateClassicalLatency(distance);

    // Apply correction based on BSM result
    const correction = this.determineCorrectionOperator(bellMeasurement.basis);
    const teleportedState = this.applyCorrection(state, correction);

    // Calculate teleportation fidelity
    const fidelity = this.calculateTeleportationFidelity(
      entanglement.fidelity,
      bellMeasurement.fidelity
    );

    if (fidelity < minFidelity) {
      throw new TeleportationError(
        TeleportationErrorCode.LOW_FIDELITY,
        `Teleportation fidelity ${fidelity} below minimum ${minFidelity}`
      );
    }

    // Mark entanglement as consumed
    entanglement.status = 'consumed';

    const totalLatency = Date.now() - startTime;

    // Update metrics
    this.updateMetrics(true, fidelity, totalLatency);

    const result: QuantumTeleportationResult = {
      id: `TELE-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      success: true,
      state: verifyFidelity ? teleportedState : undefined,
      fidelity: verifyFidelity ? fidelity : undefined,
      bellMeasurement,
      correction,
      classicalLatency,
      totalLatency,
      entanglementConsumed: [entanglementId],
      protocol,
      timestamp: new Date(),
    };

    return result;
  }

  /**
   * Teleport multiple qubits
   *
   * @param params - Multi-qubit teleportation parameters
   * @returns Multi-qubit teleportation result
   */
  async teleportMultiQubit(
    params: MultiQubitTeleportationParams
  ): Promise<MultiQubitTeleportationResult> {
    const { states, from, to, entanglementIds, mode = 'parallel', verify = true } = params;

    if (states.length !== entanglementIds.length) {
      throw new TeleportationError(
        TeleportationErrorCode.INVALID_PARAMETERS,
        'Number of states must match number of entanglement pairs'
      );
    }

    const results: QuantumTeleportationResult[] = [];
    const startTime = Date.now();

    if (mode === 'parallel') {
      // Teleport all qubits simultaneously
      const promises = states.map((state, i) =>
        this.teleportQuantumState({
          state,
          from,
          to,
          entanglementId: entanglementIds[i],
          verifyFidelity: verify,
        })
      );

      const parallelResults = await Promise.all(promises);
      results.push(...parallelResults);
    } else {
      // Sequential teleportation
      for (let i = 0; i < states.length; i++) {
        const result = await this.teleportQuantumState({
          state: states[i],
          from,
          to,
          entanglementId: entanglementIds[i],
          verifyFidelity: verify,
        });
        results.push(result);
      }
    }

    const totalLatency = Date.now() - startTime;
    const successCount = results.filter((r) => r.success).length;
    const successRate = successCount / results.length;

    // Calculate combined fidelity (product of individual fidelities)
    const combinedFidelity = results
      .filter((r) => r.fidelity !== undefined)
      .reduce((product, r) => product * (r.fidelity || 1), 1);

    return {
      id: `MULTI-TELE-${Date.now()}`,
      success: successRate === 1,
      qubitResults: results,
      combinedFidelity,
      totalLatency,
      successRate,
    };
  }

  /**
   * Continuous variable quantum teleportation
   *
   * @param params - CV teleportation parameters
   * @returns CV teleportation result
   */
  async teleportContinuousVariable(
    params: CVTeleportationParams
  ): Promise<CVTeleportationResult> {
    const {
      state,
      position,
      momentum,
      from,
      to,
      squeezing,
      gain = 1,
      verify = true,
    } = params;

    const startTime = Date.now();

    // Determine input state
    const inputState = state || {
      position: position || 0,
      momentum: momentum || 0,
      positionVariance: 0.5,
      momentumVariance: 0.5,
    };

    // Calculate EPR variance from squeezing
    const variance = Math.exp(-2 * (squeezing / 10) * Math.log(10));

    // Simulate homodyne measurements
    const positionSum = inputState.position + (Math.random() - 0.5) * Math.sqrt(variance);
    const momentumDiff = inputState.momentum + (Math.random() - 0.5) * Math.sqrt(variance);

    // Apply feedforward (displacement operations)
    const outputPosition = gain * positionSum;
    const outputMomentum = gain * momentumDiff;

    // Calculate fidelity for coherent states
    const fidelity = 1 / (1 + variance);

    const outputState = {
      position: outputPosition,
      momentum: outputMomentum,
      positionVariance: variance,
      momentumVariance: variance,
      description: `Teleported CV state (squeezing: ${squeezing} dB)`,
    };

    const latency = Date.now() - startTime;

    return {
      id: `CV-TELE-${Date.now()}`,
      success: true,
      state: outputState,
      fidelity,
      measurements: {
        positionSum,
        momentumDiff,
      },
      displacement: {
        position: outputPosition,
        momentum: outputMomentum,
      },
      latency,
      timestamp: new Date(),
    };
  }

  // ==========================================================================
  // Bell State Measurement
  // ==========================================================================

  /**
   * Perform Bell state measurement
   *
   * @param state - Quantum state to measure
   * @param entanglement - Entanglement resource
   * @returns Bell measurement result
   */
  async performBellMeasurement(
    state: QuantumState,
    entanglement: EntanglementPair
  ): Promise<BellMeasurementResult> {
    // Simulate BSM with realistic success probabilities
    const method = this.determineBSMMethod();
    const successProbability =
      method === 'linear-optics'
        ? TELEPORTATION_CONSTANTS.LINEAR_OPTICS_BSM_SUCCESS
        : TELEPORTATION_CONSTANTS.COMPLETE_BSM_SUCCESS;

    const success = Math.random() < successProbability;

    if (!success) {
      return {
        basis: 'phi-plus',
        classicalBits: '00',
        fidelity: 0,
        success: false,
        timestamp: new Date(),
        method,
      };
    }

    // Randomly select Bell state (uniform distribution)
    const bellStates: BellStateType[] = ['phi-plus', 'phi-minus', 'psi-plus', 'psi-minus'];
    const measuredState = bellStates[Math.floor(Math.random() * bellStates.length)];

    // Map to classical bits
    const classicalBitsMap: Record<BellStateType, string> = {
      'phi-plus': '00',
      'phi-minus': '01',
      'psi-plus': '10',
      'psi-minus': '11',
    };

    const classicalBits = classicalBitsMap[measuredState];

    // Measurement fidelity depends on entanglement quality
    const fidelity = 0.95 + entanglement.fidelity * 0.05;

    return {
      basis: measuredState,
      classicalBits,
      fidelity,
      success: true,
      timestamp: new Date(),
      method,
    };
  }

  /**
   * Determine correction operator from Bell measurement
   *
   * @param bellState - Measured Bell state
   * @returns Pauli correction operator
   */
  private determineCorrectionOperator(bellState: BellStateType): PauliOperator {
    const correctionMap: Record<BellStateType, PauliOperator> = {
      'phi-plus': 'I',
      'phi-minus': 'Z',
      'psi-plus': 'X',
      'psi-minus': 'XZ',
    };

    return correctionMap[bellState];
  }

  /**
   * Apply correction operator to state
   *
   * @param state - Input state
   * @param correction - Pauli operator
   * @returns Corrected state
   */
  private applyCorrection(state: QuantumState, correction: PauliOperator): QuantumState {
    // Simplified: In real implementation, apply Pauli matrices
    return {
      ...state,
      description: `${state.description || 'state'} after ${correction} correction`,
    };
  }

  // ==========================================================================
  // Entanglement Management
  // ==========================================================================

  /**
   * Generate entanglement pairs
   *
   * @param params - Generation parameters
   * @returns Array of generated entanglement pairs
   */
  async generateEntanglement(
    params: EntanglementGenerationParams
  ): Promise<EntanglementPair[]> {
    const { nodeA, nodeB, targetFidelity, count, method = 'SPDC', purification = false } = params;

    const pairs: EntanglementPair[] = [];

    for (let i = 0; i < count; i++) {
      const baseFidelity = 0.85 + Math.random() * 0.1;
      let fidelity = baseFidelity;

      // Apply purification if requested
      if (purification && fidelity < targetFidelity) {
        fidelity = this.purifyFidelity(fidelity);
      }

      if (fidelity >= targetFidelity) {
        const pair: EntanglementPair = {
          id: `ENT-${Date.now()}-${i}-${Math.random().toString(36).substr(2, 9)}`,
          nodeA,
          nodeB,
          state: 'phi-plus',
          fidelity,
          created: new Date(),
          source: method,
          status: 'active',
          quality: {
            concurrence: Math.sqrt(2 * fidelity - 1),
            bellViolation: 2 + fidelity * 0.828,
          },
        };

        this.entanglements.set(pair.id, pair);
        pairs.push(pair);
      }
    }

    return pairs;
  }

  /**
   * Get entanglement pair by ID
   */
  getEntanglement(id: string): EntanglementPair | undefined {
    return this.entanglements.get(id);
  }

  /**
   * Reserve entanglement for teleportation
   */
  reserveEntanglement(id: string): boolean {
    const ent = this.entanglements.get(id);
    if (ent && ent.status === 'active') {
      ent.status = 'reserved';
      return true;
    }
    return false;
  }

  // ==========================================================================
  // Teleportation Network
  // ==========================================================================

  /**
   * Create teleportation network
   *
   * @param params - Network parameters
   * @returns Network instance
   */
  createNetwork(params: TeleportationNetworkParams): TeleportationNetwork {
    return new TeleportationNetwork(this, params);
  }

  /**
   * Add node to network
   */
  addNode(node: TeleportationNode): void {
    this.nodes.set(node.id, node);
  }

  /**
   * Get node by ID
   */
  getNode(id: string): TeleportationNode | undefined {
    return this.nodes.get(id);
  }

  // ==========================================================================
  // Fidelity and Verification
  // ==========================================================================

  /**
   * Calculate fidelity between states
   *
   * @param params - Fidelity calculation parameters
   * @returns Fidelity result
   */
  calculateFidelity(params: FidelityParams): FidelityResult {
    const { inputState, outputState, method = 'state-overlap' } = params;

    // Simplified fidelity calculation
    // In real implementation, compute |⟨ψ|φ⟩|²
    let fidelity = 0.9 + Math.random() * 0.09;

    const quantumAdvantage = fidelity > TELEPORTATION_CONSTANTS.CLASSICAL_FIDELITY_LIMIT;

    return {
      fidelity,
      quantumAdvantage,
      method,
      confidence: {
        lower: fidelity - 0.02,
        upper: Math.min(1, fidelity + 0.02),
      },
    };
  }

  /**
   * Verify teleported state
   *
   * @param params - Verification parameters
   * @returns Verification result
   */
  async verifyTeleportation(params: VerificationParams): Promise<VerificationResult> {
    const {
      state,
      reference,
      protocol = 'direct-fidelity',
      measurements = 100,
      confidence = 0.95,
    } = params;

    // Perform verification measurements
    const fidelityResult = this.calculateFidelity({
      inputState: reference,
      outputState: state,
      method: protocol,
    });

    const verified = fidelityResult.quantumAdvantage;

    return {
      verified,
      fidelity: fidelityResult.fidelity,
      protocol,
      measurementCount: measurements,
      confidence,
      details: {
        method: protocol,
        quantumAdvantage: fidelityResult.quantumAdvantage,
      },
    };
  }

  /**
   * Calculate teleportation fidelity from entanglement fidelity
   */
  private calculateTeleportationFidelity(
    entanglementFidelity: number,
    measurementFidelity: number = 1
  ): number {
    // F_tel = (2*F_ent + 1)/3 for depolarizing noise
    const baseFidelity = (2 * entanglementFidelity + 1) / 3;
    return baseFidelity * measurementFidelity;
  }

  /**
   * Purify entanglement fidelity (BBPSSW protocol)
   */
  private purifyFidelity(fidelity: number): number {
    const F = fidelity;
    const numerator = F * F + Math.pow((1 - F) / 3, 2);
    const denominator =
      F * F + (2 * F * (1 - F)) / 3 + 5 * Math.pow((1 - F) / 3, 2);
    return numerator / denominator;
  }

  // ==========================================================================
  // Benchmarking
  // ==========================================================================

  /**
   * Run teleportation benchmark
   *
   * @param params - Benchmark parameters
   * @returns Benchmark results
   */
  async runBenchmark(params: BenchmarkParams): Promise<BenchmarkResult> {
    const { protocol, trials, duration, stateTypes = ['random'], detailed = false } = params;

    const startTime = Date.now();
    const results: QuantumTeleportationResult[] = [];
    const fidelities: number[] = [];
    const latencies: number[] = [];

    let completed = 0;
    while (
      completed < trials &&
      (duration === undefined || Date.now() - startTime < duration * 1000)
    ) {
      // Generate random state
      const state = this.generateRandomState();

      // Generate entanglement
      const entPairs = await this.generateEntanglement({
        nodeA: 'bench-a',
        nodeB: 'bench-b',
        targetFidelity: 0.9,
        count: 1,
      });

      if (entPairs.length > 0) {
        try {
          const result = await this.teleportQuantumState({
            state,
            from: 'bench-a',
            to: 'bench-b',
            protocol,
            entanglementId: entPairs[0].id,
          });

          results.push(result);
          if (result.fidelity) fidelities.push(result.fidelity);
          latencies.push(result.totalLatency);
          completed++;
        } catch (error) {
          // Count failed attempts
          completed++;
        }
      }
    }

    const actualDuration = (Date.now() - startTime) / 1000;

    const metrics: TeleportationMetrics = {
      totalAttempts: completed,
      successfulTeleportations: results.length,
      successRate: results.length / completed,
      averageFidelity: fidelities.reduce((a, b) => a + b, 0) / fidelities.length || 0,
      averageLatency: latencies.reduce((a, b) => a + b, 0) / latencies.length || 0,
      throughput: results.length / actualDuration,
      entanglementRate: completed / actualDuration,
      classicalBandwidth: (results.length * 2) / actualDuration, // 2 bits per teleportation
      timestamp: new Date(),
    };

    const result: BenchmarkResult = {
      protocol,
      trials: completed,
      duration: actualDuration,
      metrics,
    };

    if (detailed && fidelities.length > 0) {
      result.fidelityDistribution = this.calculateDistribution(fidelities);
      result.latencyDistribution = this.calculateDistribution(latencies);
    }

    return result;
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Validate quantum state
   */
  private validateQuantumState(state: QuantumState): void {
    if (state.dimension !== state.amplitudes.length) {
      throw new TeleportationError(
        TeleportationErrorCode.INVALID_STATE,
        'State dimension does not match amplitude count'
      );
    }

    // Check normalization
    const norm = state.amplitudes.reduce(
      (sum, amp) => sum + amp.real * amp.real + amp.imag * amp.imag,
      0
    );

    if (Math.abs(norm - 1) > 0.01) {
      throw new TeleportationError(
        TeleportationErrorCode.INVALID_STATE,
        `State not normalized: norm = ${norm}`
      );
    }
  }

  /**
   * Generate random quantum state
   */
  private generateRandomState(): QuantumState {
    const theta = Math.random() * Math.PI;
    const phi = Math.random() * 2 * Math.PI;

    return {
      dimension: 2,
      amplitudes: [
        { real: Math.cos(theta / 2), imag: 0 },
        {
          real: Math.sin(theta / 2) * Math.cos(phi),
          imag: Math.sin(theta / 2) * Math.sin(phi),
        },
      ],
      description: 'Random qubit state',
    };
  }

  /**
   * Calculate distance between nodes
   */
  private calculateDistance(nodeA: string, nodeB: string): number {
    // Simplified: return random distance in km
    return 50 + Math.random() * 100;
  }

  /**
   * Calculate classical communication latency
   */
  private calculateClassicalLatency(distance: number): number {
    // Speed of light in fiber: ~2/3 * c
    const speedInFiber = (2 / 3) * TELEPORTATION_CONSTANTS.SPEED_OF_LIGHT;
    return (distance * 1000) / speedInFiber * 1000; // ms
  }

  /**
   * Determine BSM method based on platform
   */
  private determineBSMMethod(): BellMeasurementResult['method'] {
    const methods: BellMeasurementResult['method'][] = [
      'linear-optics',
      'complete-bsm',
      'ion-trap',
      'superconducting',
    ];
    return methods[Math.floor(Math.random() * methods.length)];
  }

  /**
   * Initialize metrics
   */
  private initializeMetrics(): TeleportationMetrics {
    return {
      totalAttempts: 0,
      successfulTeleportations: 0,
      successRate: 0,
      averageFidelity: 0,
      averageLatency: 0,
      throughput: 0,
      entanglementRate: 0,
      classicalBandwidth: 0,
      timestamp: new Date(),
    };
  }

  /**
   * Update metrics after teleportation
   */
  private updateMetrics(success: boolean, fidelity: number, latency: number): void {
    this.metrics.totalAttempts++;
    if (success) {
      this.metrics.successfulTeleportations++;
      this.metrics.averageFidelity =
        (this.metrics.averageFidelity * (this.metrics.successfulTeleportations - 1) +
          fidelity) /
        this.metrics.successfulTeleportations;
      this.metrics.averageLatency =
        (this.metrics.averageLatency * (this.metrics.successfulTeleportations - 1) +
          latency) /
        this.metrics.successfulTeleportations;
    }
    this.metrics.successRate =
      this.metrics.successfulTeleportations / this.metrics.totalAttempts;
  }

  /**
   * Calculate distribution statistics
   */
  private calculateDistribution(values: number[]): {
    min: number;
    max: number;
    mean: number;
    median: number;
    stdDev: number;
  } {
    const sorted = [...values].sort((a, b) => a - b);
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const variance =
      values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;

    return {
      min: sorted[0],
      max: sorted[sorted.length - 1],
      mean,
      median: sorted[Math.floor(sorted.length / 2)],
      stdDev: Math.sqrt(variance),
    };
  }

  /**
   * Get current metrics
   */
  getMetrics(): TeleportationMetrics {
    return { ...this.metrics };
  }
}

// ============================================================================
// Teleportation Network Class
// ============================================================================

/**
 * Teleportation network for multi-node teleportation
 */
export class TeleportationNetwork {
  private nodes: Set<string>;
  private topology: NetworkTopology;
  private entanglementPairs: Map<string, string[]> = new Map();

  constructor(
    private sdk: TeleportationSDK,
    params: TeleportationNetworkParams
  ) {
    this.nodes = new Set(params.nodes);
    this.topology = params.topology;
  }

  /**
   * Establish entanglement across network
   */
  async establishEntanglement(): Promise<void> {
    const nodeArray = Array.from(this.nodes);

    if (this.topology === 'mesh') {
      // Full mesh - all pairs
      for (let i = 0; i < nodeArray.length; i++) {
        for (let j = i + 1; j < nodeArray.length; j++) {
          const pairs = await this.sdk.generateEntanglement({
            nodeA: nodeArray[i],
            nodeB: nodeArray[j],
            targetFidelity: 0.9,
            count: 10,
          });

          const key = `${nodeArray[i]}-${nodeArray[j]}`;
          this.entanglementPairs.set(
            key,
            pairs.map((p) => p.id)
          );
        }
      }
    } else if (this.topology === 'star') {
      // Star - hub to all others
      const hub = nodeArray[0];
      for (let i = 1; i < nodeArray.length; i++) {
        const pairs = await this.sdk.generateEntanglement({
          nodeA: hub,
          nodeB: nodeArray[i],
          targetFidelity: 0.9,
          count: 10,
        });

        const key = `${hub}-${nodeArray[i]}`;
        this.entanglementPairs.set(
          key,
          pairs.map((p) => p.id)
        );
      }
    }
  }

  /**
   * Teleport state across network
   */
  async teleport(
    params: NetworkTeleportationParams
  ): Promise<QuantumTeleportationResult> {
    const { state, from, to, routing = 'shortest-path' } = params;

    // Find path
    const path = this.findPath(from, to, routing);

    if (path.hops === 1) {
      // Direct teleportation
      return this.sdk.teleportQuantumState({
        state,
        from,
        to,
        entanglementId: path.entanglementIds[0],
      });
    } else {
      // Multi-hop teleportation
      let currentState = state;
      let currentNode = from;

      for (let i = 0; i < path.path.length - 1; i++) {
        const nextNode = path.path[i + 1];
        const result = await this.sdk.teleportQuantumState({
          state: currentState,
          from: currentNode,
          to: nextNode,
          entanglementId: path.entanglementIds[i],
        });

        if (!result.success || !result.state) {
          throw new TeleportationError(
            TeleportationErrorCode.NETWORK_ERROR,
            `Teleportation failed at hop ${i + 1}`
          );
        }

        currentState = result.state;
        currentNode = nextNode;
      }

      // Return final result
      return this.sdk.teleportQuantumState({
        state: currentState,
        from: path.path[path.path.length - 2],
        to,
        entanglementId: path.entanglementIds[path.entanglementIds.length - 1],
      });
    }
  }

  /**
   * Find path between nodes
   */
  private findPath(from: string, to: string, strategy: string): TeleportationPath {
    // Simplified: direct path if available
    const key = `${from}-${to}`;
    const reverseKey = `${to}-${from}`;

    const entIds =
      this.entanglementPairs.get(key) || this.entanglementPairs.get(reverseKey);

    if (entIds && entIds.length > 0) {
      return {
        source: from,
        destination: to,
        path: [from, to],
        entanglementIds: [entIds[0]],
        expectedFidelity: 0.95,
        expectedLatency: 10,
        hops: 1,
      };
    }

    throw new TeleportationError(
      TeleportationErrorCode.NETWORK_ERROR,
      `No path found from ${from} to ${to}`
    );
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Teleport quantum state (standalone function)
 */
export async function teleportQuantumState(
  params: QuantumTeleportationParams
): Promise<QuantumTeleportationResult> {
  const sdk = new TeleportationSDK();
  return sdk.teleportQuantumState(params);
}

/**
 * Calculate teleportation fidelity from entanglement fidelity
 */
export function calculateTeleportationFidelity(entanglementFidelity: number): number {
  return (2 * entanglementFidelity + 1) / 3;
}

/**
 * Check if fidelity exceeds classical limit
 */
export function hasQuantumAdvantage(fidelity: number): boolean {
  return fidelity > TELEPORTATION_CONSTANTS.CLASSICAL_FIDELITY_LIMIT;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TeleportationSDK, TeleportationNetwork };
export default TeleportationSDK;

/**
 * 弘益人間 (Benefit All Humanity)
 */
