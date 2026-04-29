/**
 * WIA-TIME-023: Temporal Tether SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for temporal tether establishment, maintenance,
 * monitoring, and recovery across spacetime.
 */

import {
  TemporalTether,
  TetherType,
  TetherStatus,
  TetherMode,
  TetherHealth,
  TetherEstablishment,
  TetherResult,
  MultiPointTether,
  TetherNetwork,
  NetworkResult,
  NetworkTopology,
  TetherEndpoint,
  DataTransfer,
  TransferResult,
  TransferProgress,
  MonitoringConfig,
  TetherAlert,
  FailoverConfig,
  FailoverEvent,
  RecoveryParams,
  RecoveryResult,
  RecoveryProgress,
  BoostParams,
  BoostResult,
  MaintenanceSchedule,
  MaintenanceResult,
  MaintenanceOperation,
  SpacetimeCoordinates,
  Vector3,
  TETHER_CONSTANTS,
  TetherErrorCode,
  TetherError,
  TetherEvent,
} from './types';

// ============================================================================
// Event Emitter for Tether Events
// ============================================================================

type EventCallback = (event: TetherEvent) => void;

class EventEmitter {
  private listeners: Map<string, EventCallback[]> = new Map();

  on(event: string, callback: EventCallback): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, []);
    }
    this.listeners.get(event)!.push(callback);
  }

  emit(event: string, data: TetherEvent): void {
    const callbacks = this.listeners.get(event);
    if (callbacks) {
      callbacks.forEach((cb) => cb(data));
    }
  }

  off(event: string, callback: EventCallback): void {
    const callbacks = this.listeners.get(event);
    if (callbacks) {
      const index = callbacks.indexOf(callback);
      if (index !== -1) {
        callbacks.splice(index, 1);
      }
    }
  }
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-023 Temporal Tether SDK
 */
export class TemporalTetherSDK extends EventEmitter {
  private version = '1.0.0';
  private tethers: Map<string, TemporalTether> = new Map();
  private networks: Map<string, TetherNetwork> = new Map();
  private monitoringIntervals: Map<string, NodeJS.Timeout> = new Map();

  constructor() {
    super();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Establish temporal tether between two spacetime points
   *
   * @param params - Tether establishment parameters
   * @returns Tether establishment result
   */
  async establishTether(params: TetherEstablishment): Promise<TetherResult> {
    const {
      endpoint1,
      endpoint2,
      bandwidth,
      strength,
      mode,
      redundancy = 1,
      encryption = 'quantum-resistant',
      metadata,
    } = params;

    const startTime = Date.now();

    // Validate parameters
    this.validateEndpoints(endpoint1, endpoint2);
    this.validateBandwidth(bandwidth);
    this.validateStrength(strength);

    // Generate tether ID
    const tetherId = this.generateTetherId();

    // Calculate temporal and spatial distances
    const temporalDist = this.calculateTemporalDistance(endpoint1, endpoint2);
    const spatialDist = this.calculateSpatialDistance(
      endpoint1.position,
      endpoint2.position
    );

    // Calculate establishment time
    const establishmentTime = this.calculateEstablishmentTime(
      temporalDist,
      bandwidth
    );

    // Simulate establishment delay
    await this.delay(Math.min(establishmentTime, 1) * 1000);

    // Calculate energy consumption
    const energyConsumed = this.calculateEnergy(temporalDist, bandwidth);

    // Calculate actual strength (may be lower than target)
    const actualStrength = this.calculateActualStrength(
      strength,
      temporalDist,
      spatialDist
    );

    // Estimate lifetime
    const estimatedLifetime = this.estimateLifetime(
      actualStrength,
      temporalDist
    );

    // Create tether object
    const tether: TemporalTether = {
      id: tetherId,
      type: 'point_to_point',
      endpoint1,
      endpoint2,
      bandwidth,
      strength: actualStrength,
      mode,
      status: 'active',
      established: new Date(),
      encryption,
      redundancy,
      metadata,
    };

    // Store tether
    this.tethers.set(tetherId, tether);

    // Start monitoring
    this.startBasicMonitoring(tetherId);

    // Emit event
    this.emit('established', {
      type: 'established',
      tetherId,
      timestamp: new Date(),
      data: { strength: actualStrength },
    });

    const warnings: string[] = [];
    if (actualStrength < strength) {
      warnings.push(
        `Actual strength (${actualStrength.toFixed(3)}) is lower than target (${strength})`
      );
    }
    if (temporalDist > 3.154e9) {
      // > 100 years
      warnings.push('Temporal distance exceeds 100 years - decoherence will be significant');
    }

    return {
      success: true,
      tetherId,
      actualStrength,
      establishmentTime: (Date.now() - startTime) / 1000,
      estimatedLifetime,
      energyConsumed,
      warnings: warnings.length > 0 ? warnings : undefined,
    };
  }

  /**
   * Get tether by ID
   */
  getTether(tetherId: string): TemporalTether | undefined {
    return this.tethers.get(tetherId);
  }

  /**
   * Get tether health metrics
   */
  async getTetherHealth(tetherId: string): Promise<TetherHealth> {
    const tether = this.tethers.get(tetherId);
    if (!tether) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        `Tether ${tetherId} not found`
      );
    }

    // Calculate time since establishment
    const uptime = (Date.now() - tether.established.getTime()) / 1000;

    // Calculate current strength (degrades over time)
    const decoherenceRate = this.calculateDecoherenceRate(tether);
    const currentStrength = tether.strength * Math.exp(-decoherenceRate * uptime);

    // Simulate other metrics
    const bitErrorRate = this.calculateBER(currentStrength);
    const latency = this.calculateLatency(tether);
    const packetLoss = this.calculatePacketLoss(currentStrength);
    const quantumCorrelation = Math.max(0, currentStrength - 0.05);
    const snr = this.calculateSNR(currentStrength);

    // Predict time to failure
    const timeToFailure = this.predictTimeToFailure(
      currentStrength,
      decoherenceRate
    );

    const overall = this.determineOverallHealth(currentStrength, bitErrorRate);

    const health: TetherHealth = {
      overall,
      strength: currentStrength,
      decoherenceRate,
      bitErrorRate,
      latency,
      packetLoss,
      quantumCorrelation,
      snr,
      uptime,
      dataTransferred: 0, // Would track in real implementation
      lastHeartbeat: new Date(),
      timeToFailure,
    };

    // Update tether
    tether.health = health;
    tether.strength = currentStrength;

    return health;
  }

  /**
   * Send data through tether
   */
  async sendData(tetherId: string, transfer: DataTransfer): Promise<TransferResult> {
    const tether = this.tethers.get(tetherId);
    if (!tether) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        `Tether ${tetherId} not found`
      );
    }

    if (tether.status !== 'active') {
      throw new TetherError(
        TetherErrorCode.STRENGTH_TOO_LOW,
        `Tether ${tetherId} is not active (status: ${tether.status})`
      );
    }

    const startTime = Date.now();

    // Determine data size
    const dataSize = this.getDataSize(transfer.data);

    // Calculate transfer time
    const transferTime = (dataSize * 8) / tether.bandwidth; // seconds

    // Simulate transfer with progress updates
    if (transfer.onProgress) {
      const steps = 10;
      for (let i = 1; i <= steps; i++) {
        await this.delay((transferTime * 1000) / steps);
        const progress: TransferProgress = {
          bytesTransferred: (dataSize * i) / steps,
          totalBytes: dataSize,
          percentage: (i * 100) / steps,
          transferRate: tether.bandwidth / 8,
          estimatedTimeRemaining: (transferTime * (steps - i)) / steps,
        };
        transfer.onProgress(progress);
      }
    } else {
      await this.delay(Math.min(transferTime, 5) * 1000);
    }

    // Get current health
    const health = await this.getTetherHealth(tetherId);

    const duration = Date.now() - startTime;

    return {
      success: true,
      bytesTransferred: dataSize,
      duration,
      averageRate: (dataSize * 1000) / duration,
      finalStrength: health.strength,
    };
  }

  /**
   * Boost tether strength
   */
  async boostTether(params: BoostParams): Promise<BoostResult> {
    const { tetherId, targetStrength, energyBudget, duration = 10, mode } = params;

    const tether = this.tethers.get(tetherId);
    if (!tether) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        `Tether ${tetherId} not found`
      );
    }

    const strengthBefore = tether.strength;

    // Calculate energy needed
    const energyNeeded = this.calculateBoostEnergy(
      strengthBefore,
      targetStrength
    );
    const energyConsumed = energyBudget
      ? Math.min(energyNeeded, energyBudget)
      : energyNeeded;

    // Calculate actual strength increase
    const strengthIncrease =
      (targetStrength - strengthBefore) * (energyConsumed / energyNeeded);

    // Apply boost
    if (mode === 'gradual') {
      // Gradual boost over duration
      const steps = 10;
      for (let i = 1; i <= steps; i++) {
        await this.delay((duration * 1000) / steps);
        tether.strength = strengthBefore + (strengthIncrease * i) / steps;
      }
    } else {
      // Immediate boost
      await this.delay(100);
      tether.strength = strengthBefore + strengthIncrease;
    }

    // Clamp to [0, 1]
    tether.strength = Math.max(0, Math.min(1, tether.strength));

    const strengthAfter = tether.strength;

    // Estimate how long the boost will last
    const decoherenceRate = this.calculateDecoherenceRate(tether);
    const effectDuration = Math.log(
      strengthAfter / strengthBefore
    ) / decoherenceRate;

    // Emit event
    this.emit('boosted', {
      type: 'boosted',
      tetherId,
      timestamp: new Date(),
      data: { strengthBefore, strengthAfter },
    });

    return {
      success: true,
      tetherId,
      strengthBefore,
      strengthAfter,
      energyConsumed,
      duration,
      effectDuration,
      timestamp: new Date(),
    };
  }

  /**
   * Create multi-point tether network
   */
  async createMultiPointTether(
    params: MultiPointTether
  ): Promise<NetworkResult> {
    const { endpoints, topology, bandwidth, redundancy, encryption } = params;

    if (endpoints.length < 2) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        'Need at least 2 endpoints for multi-point tether'
      );
    }

    const startTime = Date.now();
    const networkId = `NET-${Date.now()}`;
    const tetherIds: string[] = [];

    // Build network based on topology
    const connections = this.buildTopology(endpoints, topology);

    // Establish individual tethers
    for (const [i, j] of connections) {
      const result = await this.establishTether({
        endpoint1: endpoints[i],
        endpoint2: endpoints[j],
        bandwidth,
        strength: 0.95,
        mode: 'bidirectional',
        redundancy,
        encryption,
      });

      if (result.success) {
        tetherIds.push(result.tetherId);
      }
    }

    // Create network object
    const network: TetherNetwork = {
      networkId,
      name: `Network-${networkId}`,
      topology,
      endpoints: endpoints.map((coords, idx) => ({
        id: `EP-${idx}`,
        coordinates: coords,
        connectedTethers: [],
        status: 'active',
      })),
      tethers: tetherIds.map((id) => this.tethers.get(id)!),
      totalCapacity: tetherIds.length * bandwidth,
      averageStrength: 0.95,
      status: 'operational',
    };

    // Store network
    this.networks.set(networkId, network);

    const establishmentTime = (Date.now() - startTime) / 1000;

    return {
      success: true,
      networkId,
      tethers: tetherIds,
      totalCapacity: network.totalCapacity,
      redundancyLevel: redundancy,
      establishmentTime,
    };
  }

  /**
   * Monitor tether with custom configuration
   */
  startMonitoring(tetherId: string, config: MonitoringConfig): void {
    // Stop existing monitoring
    this.stopMonitoring(tetherId);

    // Start new monitoring
    const interval = setInterval(async () => {
      const health = await this.getTetherHealth(tetherId);

      // Check thresholds and generate alerts
      if (
        config.thresholds.minStrength &&
        health.strength < config.thresholds.minStrength
      ) {
        const alert: TetherAlert = {
          id: `ALERT-${Date.now()}`,
          tetherId,
          severity: health.strength < 0.5 ? 'critical' : 'warning',
          type: 'strength_low',
          message: `Tether strength (${health.strength.toFixed(3)}) below threshold (${config.thresholds.minStrength})`,
          value: health.strength,
          threshold: config.thresholds.minStrength,
          timestamp: new Date(),
          recommendation: 'Consider boosting tether or re-establishing connection',
        };

        if (config.onAlert) {
          config.onAlert(alert);
        }
      }

      // Call update callback
      if (config.onUpdate) {
        config.onUpdate(health);
      }
    }, config.interval);

    this.monitoringIntervals.set(tetherId, interval);
  }

  /**
   * Stop monitoring tether
   */
  stopMonitoring(tetherId: string): void {
    const interval = this.monitoringIntervals.get(tetherId);
    if (interval) {
      clearInterval(interval);
      this.monitoringIntervals.delete(tetherId);
    }
  }

  /**
   * Attempt to recover failed tether
   */
  async recoverTether(params: RecoveryParams): Promise<RecoveryResult> {
    const {
      tetherId,
      timeout,
      maxAttempts,
      forceReestablish,
      onProgress,
    } = params;

    const tether = this.tethers.get(tetherId);
    if (!tether) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        `Tether ${tetherId} not found`
      );
    }

    const startTime = Date.now();
    let attempts = 0;
    const phasesCompleted: string[] = [];
    const errors: string[] = [];

    const reportProgress = (
      phase: RecoveryProgress['phase'],
      percentage: number,
      message: string
    ) => {
      if (onProgress) {
        onProgress({
          phase,
          percentage,
          attempt: attempts + 1,
          message,
          timestamp: new Date(),
        });
      }
    };

    while (attempts < maxAttempts) {
      attempts++;

      try {
        // Phase 1: Diagnosis
        reportProgress('diagnosis', 10, 'Diagnosing tether failure...');
        await this.delay(1000);
        phasesCompleted.push('diagnosis');

        // Phase 2: Isolation
        reportProgress('isolation', 30, 'Isolating failed segment...');
        await this.delay(2000);
        tether.status = 'recovering';
        phasesCompleted.push('isolation');

        // Phase 3: Repair
        reportProgress('repair', 50, 'Attempting to repair connection...');
        await this.delay(5000);

        // Try to boost strength
        const boostResult = await this.boostTether({
          tetherId,
          targetStrength: 0.9,
          mode: 'immediate',
        });

        if (boostResult.strengthAfter >= 0.8) {
          phasesCompleted.push('repair');

          // Phase 4: Testing
          reportProgress('testing', 70, 'Testing repaired connection...');
          await this.delay(2000);
          const health = await this.getTetherHealth(tetherId);

          if (health.strength >= 0.8 && health.bitErrorRate < 0.01) {
            phasesCompleted.push('testing');

            // Phase 5: Restoration
            reportProgress('restoration', 90, 'Restoring full operation...');
            await this.delay(1000);
            tether.status = 'active';
            phasesCompleted.push('restoration');

            reportProgress('restoration', 100, 'Recovery complete!');

            return {
              success: true,
              tetherId,
              duration: (Date.now() - startTime) / 1000,
              attempts,
              finalStrength: health.strength,
              phasesCompleted,
            };
          }
        }

        errors.push(`Attempt ${attempts} failed: insufficient strength after boost`);
      } catch (error) {
        errors.push(`Attempt ${attempts} failed: ${error}`);
      }

      // Check timeout
      if ((Date.now() - startTime) / 1000 > timeout) {
        break;
      }
    }

    // Recovery failed - try re-establishment if requested
    if (forceReestablish) {
      reportProgress('repair', 50, 'Re-establishing tether from scratch...');

      const result = await this.establishTether({
        endpoint1: tether.endpoint1,
        endpoint2: tether.endpoint2,
        bandwidth: tether.bandwidth,
        strength: 0.95,
        mode: tether.mode,
        redundancy: tether.redundancy,
        encryption: tether.encryption,
      });

      if (result.success) {
        // Replace old tether
        this.tethers.delete(tetherId);
        const newTether = this.tethers.get(result.tetherId)!;
        newTether.id = tetherId;
        this.tethers.set(tetherId, newTether);
        this.tethers.delete(result.tetherId);

        return {
          success: true,
          tetherId,
          duration: (Date.now() - startTime) / 1000,
          attempts,
          finalStrength: result.actualStrength,
          phasesCompleted: ['re-establishment'],
        };
      }
    }

    // Recovery failed
    tether.status = 'failed';

    return {
      success: false,
      tetherId,
      duration: (Date.now() - startTime) / 1000,
      attempts,
      finalStrength: tether.strength,
      phasesCompleted,
      errors,
    };
  }

  /**
   * Perform maintenance operation
   */
  async performMaintenance(
    tetherId: string,
    operation: MaintenanceOperation
  ): Promise<MaintenanceResult> {
    const tether = this.tethers.get(tetherId);
    if (!tether) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_UNREACHABLE,
        `Tether ${tetherId} not found`
      );
    }

    const startTime = Date.now();
    const strengthBefore = tether.strength;

    // Perform operation
    switch (operation) {
      case 'calibration':
        await this.delay(2000);
        // Slight improvement
        tether.strength = Math.min(1.0, tether.strength + 0.05);
        break;

      case 'boost':
        await this.boostTether({
          tetherId,
          targetStrength: 0.95,
          mode: 'gradual',
          duration: 5,
        });
        break;

      case 'purification':
        await this.delay(3000);
        // Moderate improvement
        tether.strength = Math.min(1.0, tether.strength + 0.1);
        break;

      case 'error_correction':
        await this.delay(1000);
        // Small improvement
        tether.strength = Math.min(1.0, tether.strength + 0.03);
        break;

      case 'synchronization':
        await this.delay(1500);
        // Minimal improvement but important for stability
        tether.strength = Math.min(1.0, tether.strength + 0.02);
        break;
    }

    const strengthAfter = tether.strength;
    tether.lastMaintenance = new Date();

    return {
      success: true,
      tetherId,
      operation,
      duration: (Date.now() - startTime) / 1000,
      strengthBefore,
      strengthAfter,
      improvement: strengthAfter - strengthBefore,
      timestamp: new Date(),
    };
  }

  /**
   * Disconnect and remove tether
   */
  async disconnectTether(tetherId: string): Promise<boolean> {
    const tether = this.tethers.get(tetherId);
    if (!tether) {
      return false;
    }

    // Stop monitoring
    this.stopMonitoring(tetherId);

    // Mark as offline
    tether.status = 'offline';

    // Remove from storage
    this.tethers.delete(tetherId);

    return true;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate endpoints for causality
   */
  private validateEndpoints(
    ep1: SpacetimeCoordinates,
    ep2: SpacetimeCoordinates
  ): void {
    // Check for same endpoint (invalid)
    if (
      ep1.position.x === ep2.position.x &&
      ep1.position.y === ep2.position.y &&
      ep1.position.z === ep2.position.z &&
      new Date(ep1.time).getTime() === new Date(ep2.time).getTime()
    ) {
      throw new TetherError(
        TetherErrorCode.ENDPOINT_COLLISION,
        'Cannot create tether to same spacetime point'
      );
    }

    // In real implementation, check for causality violations
  }

  /**
   * Validate bandwidth
   */
  private validateBandwidth(bandwidth: number): void {
    if (bandwidth < TETHER_CONSTANTS.BANDWIDTH.MIN) {
      throw new TetherError(
        TetherErrorCode.BANDWIDTH_EXCEEDED,
        `Bandwidth too low (min: ${TETHER_CONSTANTS.BANDWIDTH.MIN} bps)`
      );
    }
    if (bandwidth > TETHER_CONSTANTS.BANDWIDTH.MAX) {
      throw new TetherError(
        TetherErrorCode.BANDWIDTH_EXCEEDED,
        `Bandwidth too high (max: ${TETHER_CONSTANTS.BANDWIDTH.MAX} bps)`
      );
    }
  }

  /**
   * Validate strength
   */
  private validateStrength(strength: number): void {
    if (strength < 0 || strength > 1) {
      throw new TetherError(
        TetherErrorCode.STRENGTH_TOO_LOW,
        'Strength must be between 0 and 1'
      );
    }
    if (strength < 0.5) {
      throw new TetherError(
        TetherErrorCode.STRENGTH_TOO_LOW,
        'Target strength too low (min: 0.5)'
      );
    }
  }

  /**
   * Generate unique tether ID
   */
  private generateTetherId(): string {
    return `TT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Calculate temporal distance in seconds
   */
  private calculateTemporalDistance(
    ep1: SpacetimeCoordinates,
    ep2: SpacetimeCoordinates
  ): number {
    const t1 = new Date(ep1.time).getTime();
    const t2 = new Date(ep2.time).getTime();
    return Math.abs(t2 - t1) / 1000;
  }

  /**
   * Calculate spatial distance in meters
   */
  private calculateSpatialDistance(p1: Vector3, p2: Vector3): number {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const dz = p2.z - p1.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  /**
   * Calculate tether establishment time
   */
  private calculateEstablishmentTime(
    temporalDist: number,
    bandwidth: number
  ): number {
    const t0 = 5; // Base time in seconds
    const alpha = 1e-9; // Temporal scaling
    const beta = 0.5;
    const nPhotons = bandwidth / 1e9; // Simplified
    return t0 + alpha * temporalDist + beta * Math.log(nPhotons);
  }

  /**
   * Calculate energy consumption
   */
  private calculateEnergy(temporalDist: number, bandwidth: number): number {
    const baseEnergy = TETHER_CONSTANTS.ENERGY.ESTABLISH;
    const temporalFactor = 1 + temporalDist / 3.154e7; // 1 year
    const bandwidthFactor = bandwidth / 1e12; // Normalized to 1 Tb/s
    return baseEnergy * temporalFactor * bandwidthFactor;
  }

  /**
   * Calculate actual achievable strength
   */
  private calculateActualStrength(
    targetStrength: number,
    temporalDist: number,
    spatialDist: number
  ): number {
    // Strength degrades with distance
    const temporalPenalty = Math.exp(-temporalDist / 3.154e9); // 100 years
    const spatialPenalty = Math.exp(-spatialDist / 1e10); // 10,000 km
    return targetStrength * temporalPenalty * spatialPenalty;
  }

  /**
   * Estimate tether lifetime
   */
  private estimateLifetime(strength: number, temporalDist: number): number {
    const decoherenceRate =
      TETHER_CONSTANTS.DECOHERENCE.MEDIUM * (1 + temporalDist / 3.154e7);
    // Time until strength drops to 0.5
    return Math.log(strength / 0.5) / decoherenceRate;
  }

  /**
   * Calculate decoherence rate
   */
  private calculateDecoherenceRate(tether: TemporalTether): number {
    const temporalDist = this.calculateTemporalDistance(
      tether.endpoint1,
      tether.endpoint2
    );
    return (
      TETHER_CONSTANTS.DECOHERENCE.MEDIUM * (1 + temporalDist / 3.154e7)
    );
  }

  /**
   * Calculate bit error rate from strength
   */
  private calculateBER(strength: number): number {
    // BER increases exponentially as strength decreases
    return Math.pow(10, -9 + 8 * (1 - strength));
  }

  /**
   * Calculate latency
   */
  private calculateLatency(tether: TemporalTether): number {
    const spatialDist = this.calculateSpatialDistance(
      tether.endpoint1.position,
      tether.endpoint2.position
    );
    const lightTime = (spatialDist / TETHER_CONSTANTS.SPEED_OF_LIGHT) * 1000; // ms
    const processingTime = 0.5; // ms
    return lightTime + processingTime;
  }

  /**
   * Calculate packet loss from strength
   */
  private calculatePacketLoss(strength: number): number {
    return Math.max(0, 0.1 * (1 - strength));
  }

  /**
   * Calculate signal-to-noise ratio
   */
  private calculateSNR(strength: number): number {
    // SNR in dB
    return 10 * Math.log10(strength / (1 - strength + 0.001));
  }

  /**
   * Predict time to failure
   */
  private predictTimeToFailure(
    currentStrength: number,
    decoherenceRate: number
  ): number {
    const criticalStrength = TETHER_CONSTANTS.STRENGTH.CRITICAL;
    if (currentStrength <= criticalStrength) {
      return 0;
    }
    return Math.log(currentStrength / criticalStrength) / decoherenceRate;
  }

  /**
   * Determine overall health status
   */
  private determineOverallHealth(
    strength: number,
    ber: number
  ): 'excellent' | 'good' | 'fair' | 'poor' | 'critical' {
    if (strength >= 0.95 && ber <= 1e-9) return 'excellent';
    if (strength >= 0.8 && ber <= 1e-6) return 'good';
    if (strength >= 0.6 && ber <= 1e-3) return 'fair';
    if (strength >= 0.4) return 'poor';
    return 'critical';
  }

  /**
   * Get data size in bytes
   */
  private getDataSize(data: Buffer | Uint8Array | string | object): number {
    if (Buffer.isBuffer(data)) return data.length;
    if (data instanceof Uint8Array) return data.length;
    if (typeof data === 'string') return data.length;
    return JSON.stringify(data).length;
  }

  /**
   * Calculate energy for boosting
   */
  private calculateBoostEnergy(
    currentStrength: number,
    targetStrength: number
  ): number {
    const delta = targetStrength - currentStrength;
    return TETHER_CONSTANTS.ENERGY.BOOST * delta * 1000;
  }

  /**
   * Build network topology connections
   */
  private buildTopology(
    endpoints: SpacetimeCoordinates[],
    topology: NetworkTopology
  ): Array<[number, number]> {
    const connections: Array<[number, number]> = [];
    const n = endpoints.length;

    switch (topology) {
      case 'star':
        // All connect to first endpoint
        for (let i = 1; i < n; i++) {
          connections.push([0, i]);
        }
        break;

      case 'mesh':
        // Fully connected
        for (let i = 0; i < n; i++) {
          for (let j = i + 1; j < n; j++) {
            connections.push([i, j]);
          }
        }
        break;

      case 'tree':
        // Binary tree structure
        for (let i = 0; i < n; i++) {
          const left = 2 * i + 1;
          const right = 2 * i + 2;
          if (left < n) connections.push([i, left]);
          if (right < n) connections.push([i, right]);
        }
        break;

      case 'ring':
        // Ring topology
        for (let i = 0; i < n; i++) {
          connections.push([i, (i + 1) % n]);
        }
        break;
    }

    return connections;
  }

  /**
   * Start basic monitoring for a tether
   */
  private startBasicMonitoring(tetherId: string): void {
    this.startMonitoring(tetherId, {
      interval: 10000, // 10 seconds
      metrics: ['strength', 'bit_error_rate'],
      thresholds: {
        minStrength: 0.5,
        maxBitErrorRate: 0.01,
      },
    });
  }

  /**
   * Delay helper
   */
  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Establish temporal tether (standalone function)
 */
export async function establishTether(
  params: TetherEstablishment
): Promise<TetherResult> {
  const sdk = new TemporalTetherSDK();
  return sdk.establishTether(params);
}

/**
 * Create multi-point tether (standalone function)
 */
export async function createMultiPointTether(
  params: MultiPointTether
): Promise<NetworkResult> {
  const sdk = new TemporalTetherSDK();
  return sdk.createMultiPointTether(params);
}

/**
 * Boost tether strength (standalone function)
 */
export async function boostTether(params: BoostParams): Promise<BoostResult> {
  const sdk = new TemporalTetherSDK();
  return sdk.boostTether(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TemporalTetherSDK };
export default TemporalTetherSDK;
