/**
 * WIA-DEF-015: Missile Defense SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive missile defense capabilities including:
 * - Multi-sensor threat detection
 * - Trajectory tracking and prediction
 * - Intercept point calculation
 * - Engagement management
 * - Kill assessment
 */

import {
  Threat,
  ThreatDetectionParams,
  ThreatDetectionResult,
  TrackingParams,
  TrackingResult,
  InterceptParams,
  InterceptResult,
  LaunchParams,
  LaunchResult,
  KillAssessmentParams,
  KillAssessment,
  MonitoringParams,
  DefenseDashboard,
  MissileDefenseConfig,
  ThreatType,
  ThreatLevel,
  InterceptorType,
  DefenseLayer,
  GeoLocation,
  Velocity,
  TrajectoryState,
  ImpactPrediction,
  InterceptorSystemSpec,
  AssessmentMethod,
  AssessmentEvidence,
  ConfidenceLevel,
  EngagementStatus,
  Engagement,
  PHYSICAL_CONSTANTS,
  KILL_PROBABILITY_THRESHOLDS,
  MissileDefenseErrorCode,
  MissileDefenseError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-015 Missile Defense SDK
 */
export class MissileDefenseSDK {
  private version = '1.0.0';
  private config: MissileDefenseConfig;
  private initialized = false;

  constructor(config: MissileDefenseConfig = {}) {
    this.config = {
      sensorFusion: true,
      autoEngagement: false, // Require human authorization by default
      minKillProbability: KILL_PROBABILITY_THRESHOLDS.MINIMUM,
      rulesOfEngagement: {
        requireHumanAuthorization: true,
        minThreatLevel: 'MEDIUM',
        noFireZones: [],
      },
      logging: {
        enabled: true,
        level: 'info',
      },
      ...config,
    };
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Get current configuration
   */
  getConfig(): MissileDefenseConfig {
    return { ...this.config };
  }

  /**
   * Detect missile threats using multi-sensor network
   *
   * @param params - Threat detection parameters
   * @returns Detected threats and confidence scores
   */
  async detectMissileThreat(
    params: ThreatDetectionParams
  ): Promise<ThreatDetectionResult> {
    const {
      sensorNetwork,
      detectionThreshold,
      threatTypes = ['ballistic', 'cruise'],
      searchVolume,
      enableFusion = true,
    } = params;

    this.log('info', `Detecting threats using ${sensorNetwork.length} sensors`);

    // Simulate multi-sensor detection (in production, would query actual sensor systems)
    const threats = this.simulateThreatDetection(
      sensorNetwork,
      detectionThreshold,
      threatTypes
    );

    // Filter by search volume if specified
    let filteredThreats = threats;
    if (searchVolume) {
      filteredThreats = threats.filter((t) =>
        this.isWithinSearchVolume(t.position, searchVolume)
      );
    }

    const result: ThreatDetectionResult = {
      threats: filteredThreats,
      timestamp: new Date(),
      confidence: this.calculateOverallConfidence(filteredThreats),
      sensorContributors: sensorNetwork,
      metadata: {
        searchVolume,
        processingTime: 50, // ms
        falseAlarmProbability: 0.01,
      },
    };

    this.log('info', `Detected ${result.threats.length} threats`);
    return result;
  }

  /**
   * Track missile trajectory and predict impact
   *
   * @param params - Tracking parameters
   * @returns Trajectory history and impact prediction
   */
  async trackMissile(params: TrackingParams): Promise<TrackingResult> {
    const {
      threatId,
      updateInterval,
      predictionHorizon,
      filterType = 'kalman',
      enableAtmospheric = true,
    } = params;

    this.log('info', `Tracking threat ${threatId} with ${filterType} filter`);

    // Simulate trajectory tracking
    const trajectory = this.simulateTrajectory(threatId, predictionHorizon);

    // Predict impact point
    const prediction = this.predictImpact(trajectory, enableAtmospheric);

    const result: TrackingResult = {
      threatId,
      trajectory,
      prediction,
      uncertainty: {
        semiMajor: 100, // meters
        semiMinor: 50,
        orientation: 45,
        confidence: 0.95,
      },
      trackQuality: 0.92,
      status: 'tracking',
    };

    this.log('info', `Impact predicted at ${prediction.impactLocation.latitude}, ${prediction.impactLocation.longitude}`);
    return result;
  }

  /**
   * Calculate optimal intercept point and trajectory
   *
   * @param params - Intercept calculation parameters
   * @returns Intercept solution with kill probability
   */
  calculateInterceptPoint(params: InterceptParams): InterceptResult {
    const {
      threatTrajectory,
      interceptorType,
      interceptorLocation,
      constraints = {},
      engagementTime = new Date(),
    } = params;

    this.log('info', `Calculating intercept using ${interceptorType}`);

    // Get interceptor specifications
    const interceptorSpec = this.getInterceptorSpec(interceptorType);

    // Calculate intercept geometry
    const intercept = this.computeInterceptGeometry(
      threatTrajectory,
      interceptorLocation,
      interceptorSpec,
      constraints
    );

    if (!intercept.feasible) {
      this.log('warn', `Intercept not feasible: ${intercept.infeasibilityReasons?.join(', ')}`);
    } else {
      this.log('info', `Intercept feasible with P(kill)=${intercept.killProbability}`);
    }

    return intercept;
  }

  /**
   * Launch interceptor missile
   *
   * @param params - Launch parameters
   * @returns Launch result with engagement tracking
   */
  async launchInterceptor(params: LaunchParams): Promise<LaunchResult> {
    const {
      interceptResult,
      interceptorId,
      batteryId,
      launchCommand,
      authorization,
      roeVerified = false,
    } = params;

    // Verify Rules of Engagement
    if (!roeVerified && this.config.rulesOfEngagement?.requireHumanAuthorization) {
      if (!authorization) {
        throw new MissileDefenseError(
          MissileDefenseErrorCode.LAUNCH_AUTHORIZATION_DENIED,
          'Launch authorization required for engagement'
        );
      }
    }

    // Verify intercept is feasible
    if (!interceptResult.feasible) {
      throw new MissileDefenseError(
        MissileDefenseErrorCode.INTERCEPT_NOT_FEASIBLE,
        'Cannot launch: intercept not feasible',
        { reasons: interceptResult.infeasibilityReasons }
      );
    }

    this.log('info', `Launching interceptor ${interceptorId}`);

    // Simulate launch (in production, would send actual launch command)
    const engagementId = `ENG-${Date.now()}`;
    const launchTime = new Date();

    const result: LaunchResult = {
      engagementId,
      status: launchCommand === 'ENGAGE' ? 'launched' : 'failed',
      launchTime,
      interceptor: {
        id: interceptorId,
        type: interceptResult.trajectory?.launchPoint ? 'PAC-3' : 'THAAD', // Infer type
        batteryId,
      },
      expectedInterceptTime: interceptResult.launchTime
        ? new Date(interceptResult.launchTime.getTime() + (interceptResult.timeToIntercept ?? 0) * 1000)
        : undefined,
      telemetryAvailable: true,
    };

    this.log('info', `Interceptor launched successfully. Engagement ID: ${engagementId}`);
    return result;
  }

  /**
   * Assess kill effectiveness after intercept
   *
   * @param params - Kill assessment parameters
   * @returns Kill assessment with confidence level
   */
  async assessKill(params: KillAssessmentParams): Promise<KillAssessment> {
    const {
      engagementId,
      sensors,
      assessmentMethod,
      observationTime = 5, // seconds
    } = params;

    this.log('info', `Assessing kill for engagement ${engagementId} using ${assessmentMethod}`);

    // Collect evidence from multiple sensors
    const evidence = this.collectAssessmentEvidence(sensors, assessmentMethod);

    // Calculate kill probability using Bayesian fusion
    const killProbability = this.calculateKillProbability(evidence);

    // Determine confidence level
    const confidence = this.determineConfidenceLevel(evidence, killProbability);

    // Generate recommendation
    let recommendation: KillAssessment['recommendation'];
    if (killProbability >= 0.9) {
      recommendation = 'confirmed-kill';
    } else if (killProbability >= 0.7) {
      recommendation = 'probable-kill';
    } else if (killProbability >= 0.5) {
      recommendation = 'uncertain';
    } else if (killProbability >= 0.3) {
      recommendation = 'second-shot-required';
    } else {
      recommendation = 'missed';
    }

    const result: KillAssessment = {
      engagementId,
      timestamp: new Date(),
      killProbability,
      threatNeutralized: killProbability >= 0.8,
      confidence,
      method: Array.isArray(assessmentMethod) ? assessmentMethod : [assessmentMethod],
      evidence,
      recommendation,
      analysis: {
        debrisPattern: 'Fragmentation cloud observed, consistent with kinetic impact',
        thermalSignature: 'High-intensity flash detected, temperature spike > 2000K',
        radarSignature: 'RCS reduction > 90%, multiple debris pieces tracked',
        visualConfirmation: true,
      },
    };

    this.log('info', `Kill assessment: ${recommendation} (P(kill)=${killProbability.toFixed(2)})`);
    return result;
  }

  /**
   * Monitor defense system status in real-time
   *
   * @param params - Monitoring parameters
   * @returns Defense dashboard metrics
   */
  async monitorDefense(params: MonitoringParams = {}): Promise<DefenseDashboard> {
    const { realTime = false, updateInterval = 1000, filters } = params;

    this.log('info', `Monitoring defense systems (real-time: ${realTime})`);

    // Generate current dashboard metrics
    const dashboard = this.generateDashboardMetrics(filters);

    if (realTime) {
      this.log('info', `Real-time monitoring enabled with ${updateInterval}ms interval`);
    }

    return dashboard;
  }

  /**
   * Create a threat monitor with event callbacks
   *
   * @param params - Monitoring parameters
   * @returns Threat monitor instance
   */
  createThreatMonitor(params: MonitoringParams): ThreatMonitor {
    return new ThreatMonitor(this, params);
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Simulate threat detection (placeholder for real sensor integration)
   */
  private simulateThreatDetection(
    sensors: string[],
    threshold: number,
    threatTypes: ThreatType[]
  ): Threat[] {
    // In production, this would query actual sensor systems
    const sampleThreats: Threat[] = [
      {
        id: 'THR-001',
        type: 'srbm',
        level: 'CRITICAL',
        detectedAt: new Date(),
        confidence: 0.95,
        position: {
          latitude: 38.5,
          longitude: 127.2,
          altitude: 85000,
        },
        velocity: {
          x: -2500,
          y: 1800,
          z: -3200,
          magnitude: 4435,
        },
        acceleration: {
          x: 0,
          y: 0,
          z: -9.81,
        },
        predictedImpact: {
          impactTime: new Date(Date.now() + 510000), // +8.5 minutes
          impactLocation: {
            latitude: 37.5,
            longitude: 126.9,
            altitude: 0,
          },
          cep: 100,
          timeToImpact: 510,
          confidence: 0.92,
        },
        detectedBy: sensors,
        trackQuality: 0.92,
        characteristics: {
          rcs: 0.5,
          speed: 4435,
          apogee: 150000,
          range: 800000,
        },
      },
    ];

    return sampleThreats.filter((t) => t.confidence >= threshold);
  }

  /**
   * Simulate trajectory tracking
   */
  private simulateTrajectory(threatId: string, horizon: number): TrajectoryState[] {
    const trajectory: TrajectoryState[] = [];
    const startTime = Date.now();

    for (let i = 0; i <= horizon; i += 10) {
      trajectory.push({
        time: new Date(startTime + i * 1000),
        position: {
          latitude: 38.5 - i * 0.01,
          longitude: 127.2 - i * 0.008,
          altitude: Math.max(0, 85000 - i * 1400),
        },
        velocity: {
          x: -2500,
          y: 1800,
          z: -3200 + i * 10,
          magnitude: 4435,
        },
        acceleration: {
          x: 0,
          y: 0,
          z: -9.81,
        },
      });
    }

    return trajectory;
  }

  /**
   * Predict impact point from trajectory
   */
  private predictImpact(
    trajectory: TrajectoryState[],
    enableAtmospheric: boolean
  ): ImpactPrediction {
    // Use last trajectory point for simplicity (in production, would extrapolate)
    const lastState = trajectory[trajectory.length - 1];

    return {
      impactTime: new Date(Date.now() + 510000), // +8.5 minutes
      impactLocation: {
        latitude: 37.5,
        longitude: 126.9,
        altitude: 0,
        horizontalUncertainty: 100,
        verticalUncertainty: 20,
      },
      cep: 100,
      timeToImpact: 510,
      confidence: 0.92,
    };
  }

  /**
   * Get interceptor system specifications
   */
  private getInterceptorSpec(type: InterceptorType): InterceptorSystemSpec {
    const specs: Record<InterceptorType, InterceptorSystemSpec> = {
      THAAD: {
        type: 'THAAD',
        name: 'Terminal High Altitude Area Defense',
        performance: {
          maxRange: 200000,
          minRange: 40000,
          maxAltitude: 150000,
          minAltitude: 40000,
          maxVelocity: 2800,
          maxAcceleration: 490, // 50g
          killProbability: 0.85,
          reactionTime: 9,
        },
        envelope: {
          optimalAltitude: 95000,
          optimalRange: 120000,
          layer: 'terminal',
        },
        availability: {
          ready: 8,
          total: 16,
          reloading: 0,
        },
      },
      'PAC-3': {
        type: 'PAC-3',
        name: 'Patriot Advanced Capability-3',
        performance: {
          maxRange: 70000,
          minRange: 3000,
          maxAltitude: 40000,
          minAltitude: 0,
          maxVelocity: 1700,
          maxAcceleration: 490, // 50g
          killProbability: 0.90,
          reactionTime: 6,
        },
        envelope: {
          optimalAltitude: 20000,
          optimalRange: 40000,
          layer: 'terminal',
        },
        availability: {
          ready: 12,
          total: 16,
          reloading: 4,
        },
      },
      'SM-3': {
        type: 'SM-3',
        name: 'Standard Missile-3',
        performance: {
          maxRange: 2500000,
          minRange: 90000,
          maxAltitude: 700000,
          minAltitude: 160000,
          maxVelocity: 4500,
          maxAcceleration: 490,
          killProbability: 0.75,
          reactionTime: 12,
        },
        envelope: {
          optimalAltitude: 375000,
          optimalRange: 1250000,
          layer: 'midcourse',
        },
      },
      'SM-6': {
        type: 'SM-6',
        name: 'Standard Missile-6',
        performance: {
          maxRange: 370000,
          minRange: 20000,
          maxAltitude: 33000,
          minAltitude: 0,
          maxVelocity: 1200,
          maxAcceleration: 392,
          killProbability: 0.85,
          reactionTime: 8,
        },
        envelope: {
          optimalAltitude: 16000,
          optimalRange: 185000,
          layer: 'terminal',
        },
      },
      Aegis: {
        type: 'Aegis',
        name: 'Aegis Ballistic Missile Defense',
        performance: {
          maxRange: 2500000,
          minRange: 90000,
          maxAltitude: 700000,
          minAltitude: 160000,
          maxVelocity: 4500,
          maxAcceleration: 490,
          killProbability: 0.75,
          reactionTime: 12,
        },
        envelope: {
          optimalAltitude: 375000,
          optimalRange: 1250000,
          layer: 'midcourse',
        },
      },
      'Arrow-3': {
        type: 'Arrow-3',
        name: 'Arrow 3 Exo-atmospheric',
        performance: {
          maxRange: 2400000,
          minRange: 100000,
          maxAltitude: 1000000,
          minAltitude: 100000,
          maxVelocity: 2500,
          maxAcceleration: 490,
          killProbability: 0.80,
          reactionTime: 10,
        },
        envelope: {
          optimalAltitude: 550000,
          optimalRange: 1200000,
          layer: 'midcourse',
        },
      },
      'Iron-Dome': {
        type: 'Iron-Dome',
        name: 'Iron Dome Point Defense',
        performance: {
          maxRange: 70000,
          minRange: 4000,
          maxAltitude: 10000,
          minAltitude: 0,
          maxVelocity: 700,
          maxAcceleration: 392,
          killProbability: 0.90,
          reactionTime: 15,
        },
        envelope: {
          optimalAltitude: 3000,
          optimalRange: 35000,
          layer: 'point',
        },
      },
      CIWS: {
        type: 'CIWS',
        name: 'Close-In Weapon System',
        performance: {
          maxRange: 3000,
          minRange: 100,
          maxAltitude: 3000,
          minAltitude: 0,
          maxVelocity: 1100,
          maxAcceleration: 588,
          killProbability: 0.95,
          reactionTime: 2,
        },
        envelope: {
          optimalAltitude: 1500,
          optimalRange: 1500,
          layer: 'point',
        },
      },
      Custom: {
        type: 'Custom',
        name: 'Custom Interceptor',
        performance: {
          maxRange: 100000,
          minRange: 5000,
          maxAltitude: 50000,
          minAltitude: 0,
          maxVelocity: 2000,
          maxAcceleration: 490,
          killProbability: 0.75,
          reactionTime: 10,
        },
        envelope: {
          optimalAltitude: 25000,
          optimalRange: 50000,
          layer: 'terminal',
        },
      },
    };

    return specs[type];
  }

  /**
   * Compute intercept geometry
   */
  private computeInterceptGeometry(
    threatTrajectory: TrajectoryState[],
    interceptorLocation: GeoLocation,
    interceptorSpec: InterceptorSystemSpec,
    constraints: any
  ): InterceptResult {
    // Simplified intercept calculation (in production, would use full kinematics)
    const currentThreat = threatTrajectory[0];
    const distance = this.calculateDistance(interceptorLocation, currentThreat.position);

    // Check basic feasibility
    const feasible =
      distance <= interceptorSpec.performance.maxRange &&
      distance >= interceptorSpec.performance.minRange &&
      currentThreat.position.altitude <= interceptorSpec.performance.maxAltitude &&
      currentThreat.position.altitude >= interceptorSpec.performance.minAltitude;

    if (!feasible) {
      return {
        feasible: false,
        infeasibilityReasons: [
          distance > interceptorSpec.performance.maxRange ? 'Target beyond maximum range' : '',
          distance < interceptorSpec.performance.minRange ? 'Target too close (minimum range violation)' : '',
          currentThreat.position.altitude > interceptorSpec.performance.maxAltitude ? 'Target altitude too high' : '',
          currentThreat.position.altitude < interceptorSpec.performance.minAltitude ? 'Target altitude too low' : '',
        ].filter(Boolean),
      };
    }

    // Calculate time to intercept
    const closingVelocity = interceptorSpec.performance.maxVelocity + (currentThreat.velocity.magnitude ?? 0);
    const timeToIntercept = distance / closingVelocity;

    // Estimate kill probability (simplified)
    const killProbability = interceptorSpec.performance.killProbability * 0.95; // Slight reduction for non-optimal conditions

    return {
      feasible: true,
      interceptPoint: {
        ...currentThreat.position,
        altitude: currentThreat.position.altitude - 5000, // Intercept slightly below current altitude
      },
      timeToIntercept,
      killProbability,
      launchTime: new Date(Date.now() + 5000), // Launch in 5 seconds
      constraintsSatisfied: {
        altitude: true,
        range: true,
        timeToImpact: true,
        acceleration: true,
      },
    };
  }

  /**
   * Collect kill assessment evidence from sensors
   */
  private collectAssessmentEvidence(
    sensors: string[],
    method: AssessmentMethod
  ): AssessmentEvidence[] {
    const evidence: AssessmentEvidence[] = [];

    // Simulate radar evidence
    if (method === 'radar' || method === 'multi-sensor') {
      evidence.push({
        type: 'radar',
        timestamp: new Date(),
        sensorId: sensors[0] || 'radar-1',
        data: {
          debrisCount: 12,
          trajectoryDeviation: 45,
          rcsChange: -35, // dB reduction
        },
        confidence: 0.92,
      });
    }

    // Simulate infrared evidence
    if (method === 'infrared' || method === 'multi-sensor') {
      evidence.push({
        type: 'infrared',
        timestamp: new Date(),
        sensorId: sensors[1] || 'ir-sat-1',
        data: {
          flashIntensity: 1.2e6, // watts/steradian
        },
        confidence: 0.88,
      });
    }

    // Simulate optical evidence
    if (method === 'optical' || method === 'multi-sensor') {
      evidence.push({
        type: 'optical',
        timestamp: new Date(),
        sensorId: sensors[2] || 'optical-1',
        data: {
          fragmentationObserved: true,
          trajectoryDeviation: 50,
        },
        confidence: 0.90,
      });
    }

    return evidence;
  }

  /**
   * Calculate kill probability from evidence using Bayesian fusion
   */
  private calculateKillProbability(evidence: AssessmentEvidence[]): number {
    if (evidence.length === 0) return 0;

    // Simplified Bayesian fusion (in production, would use full probabilistic model)
    let totalConfidence = 0;
    let weightSum = 0;

    for (const ev of evidence) {
      let evidenceScore = 0;

      if (ev.data.debrisCount && ev.data.debrisCount > 5) {
        evidenceScore += 0.3;
      }
      if (ev.data.flashIntensity && ev.data.flashIntensity > 1e6) {
        evidenceScore += 0.3;
      }
      if (ev.data.trajectoryDeviation && ev.data.trajectoryDeviation > 30) {
        evidenceScore += 0.2;
      }
      if (ev.data.rcsChange && ev.data.rcsChange < -30) {
        evidenceScore += 0.2;
      }
      if (ev.data.fragmentationObserved) {
        evidenceScore += 0.3;
      }

      totalConfidence += evidenceScore * ev.confidence;
      weightSum += ev.confidence;
    }

    return Math.min(0.98, totalConfidence / weightSum);
  }

  /**
   * Determine confidence level from evidence
   */
  private determineConfidenceLevel(
    evidence: AssessmentEvidence[],
    killProbability: number
  ): ConfidenceLevel {
    const avgConfidence = evidence.reduce((sum, e) => sum + e.confidence, 0) / evidence.length;

    if (avgConfidence >= 0.9 && killProbability >= 0.85) return 'high';
    if (avgConfidence >= 0.75 && killProbability >= 0.7) return 'medium';
    if (avgConfidence >= 0.6 && killProbability >= 0.5) return 'low';
    return 'uncertain';
  }

  /**
   * Generate dashboard metrics
   */
  private generateDashboardMetrics(filters?: any): DefenseDashboard {
    return {
      timestamp: new Date(),
      activeThreats: {
        total: 3,
        byType: {
          ballistic: 2,
          cruise: 1,
          hypersonic: 0,
          srbm: 1,
          mrbm: 1,
          irbm: 0,
          icbm: 0,
        },
        byLevel: {
          CRITICAL: 1,
          HIGH: 1,
          MEDIUM: 1,
          LOW: 0,
        },
      },
      activeEngagements: {
        total: 2,
        byLayer: {
          boost: 0,
          midcourse: 0,
          terminal: 2,
          point: 0,
        },
        byStatus: {
          detecting: 0,
          tracking: 1,
          calculating: 0,
          authorized: 0,
          launched: 1,
          intercepting: 0,
          intercepted: 0,
          missed: 0,
          aborted: 0,
        },
      },
      systemReadiness: {
        interceptorsAvailable: {
          THAAD: 8,
          'PAC-3': 12,
          'SM-3': 24,
          'SM-6': 32,
          Aegis: 24,
          'Arrow-3': 16,
          'Iron-Dome': 60,
          CIWS: 100,
          Custom: 0,
        },
        sensorAvailability: 0.95,
        readinessScore: 0.92,
      },
      performance: {
        totalEngagements: 15,
        successfulIntercepts: 14,
        successRate: 0.93,
        avgResponseTime: 8.5,
      },
      alertStatus: 'ELEVATED',
    };
  }

  /**
   * Check if position is within search volume
   */
  private isWithinSearchVolume(position: GeoLocation, volume: any): boolean {
    const distance = this.calculateDistance(position, volume.center);
    const withinRadius = distance <= volume.radius;
    const withinAltitude =
      (!volume.minAltitude || position.altitude >= volume.minAltitude) &&
      (!volume.maxAltitude || position.altitude <= volume.maxAltitude);
    return withinRadius && withinAltitude;
  }

  /**
   * Calculate distance between two points (simplified)
   */
  private calculateDistance(p1: GeoLocation, p2: GeoLocation): number {
    // Simplified distance (in production, would use proper geodesic calculation)
    const dx = (p2.latitude - p1.latitude) * 111000; // rough conversion to meters
    const dy = (p2.longitude - p1.longitude) * 111000 * Math.cos((p1.latitude * Math.PI) / 180);
    const dz = p2.altitude - p1.altitude;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  /**
   * Calculate overall confidence from multiple detections
   */
  private calculateOverallConfidence(threats: Threat[]): number {
    if (threats.length === 0) return 0;
    return threats.reduce((sum, t) => sum + t.confidence, 0) / threats.length;
  }

  /**
   * Log message
   */
  private log(level: string, message: string): void {
    if (this.config.logging?.enabled) {
      const prefix = `[WIA-DEF-015] [${level.toUpperCase()}]`;
      console.log(`${prefix} ${message}`);
    }
  }
}

// ============================================================================
// Threat Monitor Class
// ============================================================================

/**
 * Real-time threat monitor with event callbacks
 */
export class ThreatMonitor {
  private sdk: MissileDefenseSDK;
  private params: MonitoringParams;
  private running = false;
  private intervalId?: NodeJS.Timeout;
  private eventHandlers: Map<string, Function[]> = new Map();

  constructor(sdk: MissileDefenseSDK, params: MonitoringParams) {
    this.sdk = sdk;
    this.params = params;
  }

  /**
   * Register event handler
   */
  on(event: 'threat-detected' | 'engagement-started' | 'intercept-success' | 'intercept-failed', handler: Function): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, []);
    }
    this.eventHandlers.get(event)!.push(handler);
  }

  /**
   * Start monitoring
   */
  async start(): Promise<void> {
    if (this.running) return;

    this.running = true;
    console.log('[ThreatMonitor] Starting real-time monitoring...');

    // Simulate periodic updates
    if (this.params.realTime) {
      const interval = this.params.updateInterval || 1000;
      this.intervalId = setInterval(() => this.checkForThreats(), interval);
    }
  }

  /**
   * Stop monitoring
   */
  stop(): void {
    if (!this.running) return;

    this.running = false;
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = undefined;
    }
    console.log('[ThreatMonitor] Stopped monitoring');
  }

  /**
   * Check for threats (internal)
   */
  private async checkForThreats(): Promise<void> {
    // Simulate threat detection
    // In production, this would poll the actual sensor network
  }

  /**
   * Emit event
   */
  private emit(event: string, data: any): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.forEach((handler) => handler(data));
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Detect missile threat (standalone function)
 */
export async function detectMissileThreat(
  params: ThreatDetectionParams
): Promise<ThreatDetectionResult> {
  const sdk = new MissileDefenseSDK();
  return sdk.detectMissileThreat(params);
}

/**
 * Track missile (standalone function)
 */
export async function trackMissile(params: TrackingParams): Promise<TrackingResult> {
  const sdk = new MissileDefenseSDK();
  return sdk.trackMissile(params);
}

/**
 * Calculate intercept point (standalone function)
 */
export function calculateInterceptPoint(params: InterceptParams): InterceptResult {
  const sdk = new MissileDefenseSDK();
  return sdk.calculateInterceptPoint(params);
}

/**
 * Launch interceptor (standalone function)
 */
export async function launchInterceptor(params: LaunchParams): Promise<LaunchResult> {
  const sdk = new MissileDefenseSDK();
  return sdk.launchInterceptor(params);
}

/**
 * Assess kill (standalone function)
 */
export async function assessKill(params: KillAssessmentParams): Promise<KillAssessment> {
  const sdk = new MissileDefenseSDK();
  return sdk.assessKill(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MissileDefenseSDK, ThreatMonitor };
export default MissileDefenseSDK;
