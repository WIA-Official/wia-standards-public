/**
 * WIA-AUTO-011: Intelligent Transportation System SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mobility Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for intelligent transportation systems including:
 * - Traffic flow analysis and optimization
 * - Smart signal control
 * - Congestion prediction
 * - Emergency vehicle preemption
 * - Vehicle detection and classification
 * - Traveler information services
 */

import {
  TrafficFlowParams,
  TrafficFlowResult,
  LevelOfService,
  SignalOptimizationParams,
  SignalOptimizationResult,
  SignalPhase,
  CongestionPredictionParams,
  CongestionPrediction,
  CongestionLevel,
  VehicleDetectionParams,
  VehicleDetectionResult,
  VehicleClass,
  PreemptionRequest,
  PreemptionResponse,
  TollCalculation,
  DynamicTollPricing,
  RouteInfo,
  TravelTimePrediction,
  ITSManagerConfig,
  TrafficConditions,
  NetworkOptimization,
  ITS_CONSTANTS,
  ITSErrorCode,
  ITSError,
  GeoCoordinate,
  TrafficApproach,
  LOSCriteria,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-011 Intelligent Transportation System SDK
 */
export class ITSManager {
  private version = '1.0.0';
  private config: ITSManagerConfig;
  private monitoringActive = false;

  constructor(config: ITSManagerConfig) {
    this.config = config;
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
  getConfig(): ITSManagerConfig {
    return { ...this.config };
  }

  /**
   * Start traffic monitoring
   */
  async startMonitoring(): Promise<void> {
    if (this.monitoringActive) {
      throw new ITSError(
        ITSErrorCode.INVALID_PARAMETERS,
        'Monitoring is already active'
      );
    }

    this.monitoringActive = true;
    // In a real implementation, this would start background monitoring
  }

  /**
   * Stop traffic monitoring
   */
  stopMonitoring(): void {
    this.monitoringActive = false;
  }

  /**
   * Get current traffic conditions
   */
  getTrafficConditions(): TrafficConditions {
    // In a real implementation, this would fetch actual data
    return {
      region: this.config.region,
      averageSpeed: 55.5,
      congestionLevel: 'light',
      activeIncidents: 2,
      totalSegments: 150,
      segmentsByLOS: {
        A: 45,
        B: 60,
        C: 30,
        D: 10,
        E: 4,
        F: 1,
      },
      lastUpdate: new Date(),
    };
  }

  /**
   * Optimize signal network
   */
  optimizeSignalNetwork(): NetworkOptimization {
    // In a real implementation, this would perform actual optimization
    return {
      signalsOptimized: 25,
      improvement: 15.7,
      delayReduction: 125.5,
      emissionsReduction: 450.2,
      timestamp: new Date(),
      recommendations: [
        'Adjust cycle length during peak hours',
        'Enable coordination on Main Street corridor',
        'Consider adaptive control at high-volume intersections',
      ],
    };
  }
}

// ============================================================================
// Traffic Flow Functions
// ============================================================================

/**
 * Calculate traffic flow using fundamental traffic flow equation
 *
 * @param params - Traffic flow parameters
 * @returns Traffic flow result with LOS
 */
export function calculateTrafficFlow(params: TrafficFlowParams): TrafficFlowResult {
  const {
    density,
    speed,
    freeFlowSpeed = ITS_CONSTANTS.HIGHWAY_FREE_FLOW_SPEED,
    jamDensity = ITS_CONSTANTS.JAM_DENSITY,
  } = params;

  // Validate inputs
  if (density < 0) {
    throw new ITSError(ITSErrorCode.INVALID_PARAMETERS, 'Density must be non-negative');
  }

  if (density > jamDensity) {
    throw new ITSError(
      ITSErrorCode.INVALID_PARAMETERS,
      `Density exceeds jam density (${jamDensity})`
    );
  }

  // Calculate speed using Greenshields model if not provided
  const calculatedSpeed =
    speed !== undefined ? speed : freeFlowSpeed * (1 - density / jamDensity);

  // Calculate flow: q = k × v
  const flow = density * calculatedSpeed;

  // Calculate critical density and maximum flow
  const criticalDensity = jamDensity / 2;
  const maxFlow = (freeFlowSpeed * jamDensity) / 4;

  // Determine flow regime
  let regime: TrafficFlowResult['regime'];
  if (density < criticalDensity * 0.3) {
    regime = 'free-flow';
  } else if (density < criticalDensity) {
    regime = 'stable';
  } else if (density < jamDensity * 0.8) {
    regime = 'unstable';
  } else {
    regime = 'congested';
  }

  // Determine Level of Service
  const levelOfService = determineLOS(density, calculatedSpeed, freeFlowSpeed);

  return {
    flow,
    density,
    speed: calculatedSpeed,
    levelOfService,
    criticalDensity,
    maxFlow,
    regime,
  };
}

/**
 * Determine Level of Service based on density and speed
 */
function determineLOS(
  density: number,
  speed: number,
  freeFlowSpeed: number
): LevelOfService {
  const speedRatio = speed / freeFlowSpeed;

  if (density < 7) return 'A';
  if (density < 11 && speedRatio > 0.85) return 'B';
  if (density < 18 && speedRatio > 0.7) return 'C';
  if (density < 26 && speedRatio > 0.55) return 'D';
  if (density < 45) return 'E';
  return 'F';
}

/**
 * Get Level of Service criteria
 */
export function getLOSCriteria(los: LevelOfService): LOSCriteria {
  const criteria: Record<LevelOfService, LOSCriteria> = {
    A: {
      levelOfService: 'A',
      densityRange: [0, 7],
      speedRatio: [0.95, 1.0],
      description: 'Free flow - excellent conditions',
    },
    B: {
      levelOfService: 'B',
      densityRange: [7, 11],
      speedRatio: [0.85, 0.95],
      description: 'Reasonably free flow - good conditions',
    },
    C: {
      levelOfService: 'C',
      densityRange: [11, 18],
      speedRatio: [0.7, 0.85],
      description: 'Stable flow - acceptable conditions',
    },
    D: {
      levelOfService: 'D',
      densityRange: [18, 26],
      speedRatio: [0.55, 0.7],
      description: 'Approaching unstable flow - tolerable conditions',
    },
    E: {
      levelOfService: 'E',
      densityRange: [26, 45],
      speedRatio: [0.4, 0.55],
      description: 'Unstable flow - poor conditions',
    },
    F: {
      levelOfService: 'F',
      densityRange: [45, 200],
      speedRatio: [0, 0.4],
      description: 'Forced/breakdown flow - unacceptable conditions',
    },
  };

  return criteria[los];
}

// ============================================================================
// Signal Optimization Functions
// ============================================================================

/**
 * Optimize signal timing using Webster's method
 *
 * @param params - Signal optimization parameters
 * @returns Optimized signal timing
 */
export function optimizeSignalTiming(
  params: SignalOptimizationParams
): SignalOptimizationResult {
  const {
    approaches,
    cycleLength,
    minCycleLength = ITS_CONSTANTS.MIN_CYCLE_LENGTH,
    maxCycleLength = ITS_CONSTANTS.MAX_CYCLE_LENGTH,
    objective = 'minimize-delay',
  } = params;

  // Validate inputs
  if (approaches.length === 0) {
    throw new ITSError(
      ITSErrorCode.INVALID_PARAMETERS,
      'At least one approach is required'
    );
  }

  // Calculate flow ratios for each approach
  const flowRatios = approaches.map((a) => ({
    ...a,
    flowRatio: a.flow / a.saturationFlow,
  }));

  // Calculate Y (sum of critical flow ratios)
  const Y = flowRatios.reduce((sum, a) => sum + a.flowRatio, 0);

  // Check if intersection is oversaturated
  if (Y >= 1.0) {
    throw new ITSError(
      ITSErrorCode.OPTIMIZATION_FAILED,
      `Intersection is oversaturated (Y=${Y.toFixed(3)}). Reduce demand or increase capacity.`
    );
  }

  // Calculate total lost time
  const totalLostTime = approaches.reduce((sum, a) => sum + a.lostTime, 0);

  // Calculate optimal cycle length using Webster's formula
  let optimalCycle: number;
  if (cycleLength === 'auto') {
    // C_opt = (1.5L + 5) / (1 - Y)
    optimalCycle = (1.5 * totalLostTime + 5) / (1 - Y);

    // Constrain to min/max limits
    optimalCycle = Math.max(minCycleLength, Math.min(maxCycleLength, optimalCycle));

    // Round to nearest 5 seconds for practicality
    optimalCycle = Math.round(optimalCycle / 5) * 5;
  } else {
    optimalCycle = cycleLength;
  }

  // Allocate green times proportionally
  const effectiveGreen = optimalCycle - totalLostTime;
  const phases: SignalPhase[] = approaches.map((approach, index) => {
    const greenTime = (effectiveGreen * approach.flow) / approaches.reduce((sum, a) => sum + a.flow, 0);

    return {
      phaseId: index + 1,
      greenTime: Math.round(greenTime),
      yellowTime: ITS_CONSTANTS.YELLOW_TIME,
      redTime: optimalCycle - Math.round(greenTime) - ITS_CONSTANTS.YELLOW_TIME,
      minGreen: ITS_CONSTANTS.MIN_GREEN_TIME,
      maxGreen: ITS_CONSTANTS.MAX_GREEN_TIME,
    };
  });

  // Calculate performance metrics
  const volumeCapacityRatio = Y;

  // Estimate delay using Webster's delay formula
  const totalDelay = approaches.reduce((sum, approach, i) => {
    const g = phases[i].greenTime;
    const c = optimalCycle;
    const x = approach.flow / approach.saturationFlow;

    // Webster's delay formula
    const delay =
      (c * Math.pow(1 - g / c, 2)) / (2 * (1 - x * (g / c))) +
      (x * x) / (2 * approach.flow * (1 - x));

    return sum + delay * approach.flow;
  }, 0);

  const totalFlow = approaches.reduce((sum, a) => sum + a.flow, 0);
  const averageDelay = totalDelay / totalFlow;

  // Determine LOS based on delay
  let los: LevelOfService;
  if (averageDelay < 10) los = 'A';
  else if (averageDelay < 20) los = 'B';
  else if (averageDelay < 35) los = 'C';
  else if (averageDelay < 55) los = 'D';
  else if (averageDelay < 80) los = 'E';
  else los = 'F';

  // Generate recommendations
  const recommendations: string[] = [];
  if (volumeCapacityRatio > 0.9) {
    recommendations.push('High v/c ratio. Consider capacity improvements.');
  }
  if (optimalCycle === maxCycleLength) {
    recommendations.push('Cycle length at maximum. Consider split phasing or signal coordination.');
  }
  if (los === 'E' || los === 'F') {
    recommendations.push('Poor LOS. Implement adaptive control or reduce demand.');
  }

  return {
    cycleLength: optimalCycle,
    phases,
    totalDelay,
    averageDelay,
    levelOfService: los,
    volumeCapacityRatio,
    recommendations,
  };
}

// ============================================================================
// Congestion Prediction Functions
// ============================================================================

/**
 * Predict traffic congestion
 *
 * @param params - Prediction parameters
 * @returns Congestion predictions
 */
export async function predictCongestion(
  params: CongestionPredictionParams
): Promise<CongestionPrediction[]> {
  const { timeWindow, historicalData = true } = params;

  // In a real implementation, this would use ML models
  // For now, return sample predictions
  const predictions: CongestionPrediction[] = [];
  const intervals = Math.ceil(timeWindow / 15); // 15-minute intervals

  for (let i = 0; i < intervals; i++) {
    const time = new Date(Date.now() + i * 15 * 60 * 1000);

    // Simulate prediction with some variation
    const hour = time.getHours();
    let baseSpeed = 80;
    let congestionLevel: CongestionLevel = 'none';

    // Simulate peak hours
    if ((hour >= 7 && hour <= 9) || (hour >= 16 && hour <= 18)) {
      baseSpeed = 45;
      congestionLevel = 'moderate';
    } else if (hour >= 22 || hour <= 5) {
      baseSpeed = 95;
      congestionLevel = 'none';
    }

    const speed = baseSpeed + (Math.random() - 0.5) * 10;
    const flow = speed * 30; // Simplified

    const freeFlowTime = 30; // minutes
    const actualTime = (freeFlowTime * 80) / speed;
    const congestionIndex = ((actualTime - freeFlowTime) / freeFlowTime) * 100;

    predictions.push({
      time,
      speed,
      flow,
      congestionLevel,
      congestionIndex,
      estimatedDelay: actualTime - freeFlowTime,
      confidence: 0.75 + Math.random() * 0.2,
      factors: congestionLevel !== 'none' ? ['rush hour', 'high demand'] : [],
    });
  }

  return predictions;
}

/**
 * Calculate congestion index
 *
 * @param actualTime - Actual travel time (minutes)
 * @param freeFlowTime - Free-flow travel time (minutes)
 * @returns Congestion index and level
 */
export function calculateCongestionIndex(
  actualTime: number,
  freeFlowTime: number
): { index: number; level: CongestionLevel } {
  const index = ((actualTime - freeFlowTime) / freeFlowTime) * 100;

  let level: CongestionLevel;
  if (index < 10) level = 'none';
  else if (index < 30) level = 'light';
  else if (index < 50) level = 'moderate';
  else if (index < 100) level = 'heavy';
  else level = 'severe';

  return { index, level };
}

// ============================================================================
// Vehicle Detection Functions
// ============================================================================

/**
 * Detect and classify vehicles
 *
 * @param params - Detection parameters
 * @returns Detection results
 */
export function detectVehicles(params: VehicleDetectionParams): VehicleDetectionResult {
  // In a real implementation, this would interface with actual detection hardware
  // For now, return simulated results

  const mockDetections = {
    count: 156,
    byClass: {
      motorcycle: 8,
      car: 120,
      van: 15,
      truck: 10,
      semi: 3,
      bus: 0,
      other: 0,
    },
    averageSpeed: 62.5,
    occupancy: 18.3,
    period: {
      start: new Date(Date.now() - 300000), // 5 minutes ago
      end: new Date(),
    },
  };

  return mockDetections as VehicleDetectionResult;
}

/**
 * Classify vehicle based on length
 *
 * @param length - Vehicle length in meters
 * @returns Vehicle classification
 */
export function classifyVehicle(length: number): VehicleClass {
  if (length < 2.5) return 'motorcycle';
  if (length < 6) return 'car';
  if (length < 8) return 'van';
  if (length < 12) return 'truck';
  if (length >= 12) return 'semi';
  return 'other';
}

// ============================================================================
// Emergency Vehicle Preemption Functions
// ============================================================================

/**
 * Process emergency vehicle preemption request
 *
 * @param request - Preemption request
 * @returns Preemption response
 */
export function processPreemption(request: PreemptionRequest): PreemptionResponse {
  const { vehicle, intersections, eta } = request;

  // In a real implementation, this would coordinate with traffic signals
  const intersectionTimings = intersections.map((id, index) => ({
    intersectionId: id,
    clearanceTime: 10, // seconds
    greenTime: 30, // seconds
    recoveryTime: 15, // seconds
  }));

  const timeSavings = intersections.length * 25; // Estimated 25 seconds per intersection

  return {
    requestId: request.requestId,
    granted: true,
    intersectionTimings,
    timeSavings,
    timestamp: new Date(),
  };
}

// ============================================================================
// Toll Collection Functions
// ============================================================================

/**
 * Calculate dynamic toll
 *
 * @param pricing - Dynamic toll pricing parameters
 * @returns Toll calculation
 */
export function calculateDynamicToll(pricing: DynamicTollPricing): TollCalculation {
  const {
    baseRate,
    demandLevel,
    congestionIndex,
    timeOfDayFactor,
    vehicleClassMultiplier,
  } = pricing;

  // Calculate surcharges
  const congestionSurcharge = baseRate * (congestionIndex / 100) * 0.5;
  const timeOfDaySurcharge = baseRate * (timeOfDayFactor - 1);
  const vehicleClassSurcharge = baseRate * (vehicleClassMultiplier - 1);

  const baseToll = baseRate;
  const totalToll =
    baseToll + congestionSurcharge + timeOfDaySurcharge + vehicleClassSurcharge;

  const breakdown = [
    `Base toll: $${baseToll.toFixed(2)}`,
    `Congestion surcharge: $${congestionSurcharge.toFixed(2)}`,
    `Time of day adjustment: $${timeOfDaySurcharge.toFixed(2)}`,
    `Vehicle class adjustment: $${vehicleClassSurcharge.toFixed(2)}`,
  ];

  return {
    baseToll,
    surcharges: {
      congestion: congestionSurcharge,
      timeOfDay: timeOfDaySurcharge,
      vehicleClass: vehicleClassSurcharge,
    },
    totalToll,
    breakdown,
  };
}

// ============================================================================
// Traveler Information Functions
// ============================================================================

/**
 * Calculate travel time
 *
 * @param route - Route information
 * @returns Travel time in minutes
 */
export function calculateTravelTime(route: RouteInfo): number {
  return route.segments.reduce((total, segment) => total + segment.travelTime, 0);
}

/**
 * Predict travel time
 *
 * @param routeId - Route identifier
 * @param departureTime - Departure time
 * @returns Travel time prediction
 */
export async function predictTravelTime(
  routeId: string,
  departureTime: Date
): Promise<TravelTimePrediction> {
  // In a real implementation, this would use historical data and ML models
  const baseTravelTime = 35; // minutes
  const variation = 10; // minutes

  const travelTime = baseTravelTime + (Math.random() - 0.5) * variation;
  const arrivalTime = new Date(departureTime.getTime() + travelTime * 60 * 1000);

  return {
    routeId,
    departureTime,
    arrivalTime,
    travelTime,
    confidence: {
      low: travelTime - 5,
      high: travelTime + 5,
    },
    accuracy: 0.82,
  };
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate distance between two coordinates (Haversine formula)
 *
 * @param point1 - First coordinate
 * @param point2 - Second coordinate
 * @returns Distance in meters
 */
export function calculateDistance(point1: GeoCoordinate, point2: GeoCoordinate): number {
  const R = 6371000; // Earth radius in meters
  const φ1 = (point1.lat * Math.PI) / 180;
  const φ2 = (point2.lat * Math.PI) / 180;
  const Δφ = ((point2.lat - point1.lat) * Math.PI) / 180;
  const Δλ = ((point2.lon - point1.lon) * Math.PI) / 180;

  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ITSManager };
export default ITSManager;
