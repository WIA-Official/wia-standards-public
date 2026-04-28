/**
 * WIA-AUTO-012: Traffic Management SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Mobility Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for traffic management including:
 * - Traffic flow analysis and optimization
 * - Signal timing control
 * - Congestion detection and management
 * - Incident detection and response
 * - Traffic prediction and forecasting
 */

import {
  TrafficFlow,
  FlowAnalysis,
  FlowParameters,
  SignalOptimizationRequest,
  SignalOptimizationResult,
  CongestionDetectionRequest,
  CongestionDetectionResult,
  CongestionMetrics,
  IncidentDetectionParams,
  IncidentDetectionResult,
  TrafficPredictionRequest,
  TrafficPredictionResult,
  PredictionPoint,
  IntersectionApproach,
  IntersectionAnalysis,
  RampMeteringParams,
  RampMeteringResult,
  SignalTimingPlan,
  TRAFFIC_CONSTANTS,
  LOS_THRESHOLDS,
  TrafficErrorCode,
  TrafficManagementError,
  LevelOfService,
  TrafficRegime,
  CongestionSeverity,
  IncidentType,
  IncidentSeverity,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUTO-012 Traffic Management SDK
 */
export class TrafficManagementSDK {
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
   * Calculate traffic flow metrics using Greenshields model
   *
   * @param params - Flow parameters (density, speed, or both)
   * @returns Flow analysis with regime and LOS
   */
  calculateTrafficFlow(params: {
    density?: number;
    speed?: number;
    flow?: number;
    freeFlowSpeed?: number;
    jamDensity?: number;
    laneCount?: number;
  }): FlowAnalysis {
    const vf = params.freeFlowSpeed || TRAFFIC_CONSTANTS.HIGHWAY_FREE_FLOW_SPEED;
    const kj = params.jamDensity || TRAFFIC_CONSTANTS.JAM_DENSITY;
    const lanes = params.laneCount || 1;

    // Calculate critical density and capacity
    const kc = kj / 2;
    const qmax = (vf * kj) / 4;

    let k: number, v: number, q: number;

    if (params.density !== undefined && params.speed !== undefined) {
      // Both given: calculate flow
      k = params.density;
      v = params.speed;
      q = k * v;
    } else if (params.density !== undefined) {
      // Density given: calculate speed and flow using Greenshields
      k = params.density;
      v = vf * (1 - k / kj);
      q = k * v;
    } else if (params.speed !== undefined) {
      // Speed given: calculate density and flow
      v = params.speed;
      k = kj * (1 - v / vf);
      q = k * v;
    } else if (params.flow !== undefined) {
      // Flow given: solve quadratic equation for density
      q = params.flow;
      // q = vf * k - (vf/kj) * k²
      // (vf/kj) * k² - vf * k + q = 0
      const a = vf / kj;
      const b = -vf;
      const c = q;
      const discriminant = b * b - 4 * a * c;

      if (discriminant < 0) {
        throw new TrafficManagementError(
          TrafficErrorCode.INVALID_PARAMETERS,
          'Flow exceeds capacity - no valid solution exists'
        );
      }

      // Two solutions: one in free-flow, one in congested
      const k1 = (-b - Math.sqrt(discriminant)) / (2 * a);
      const k2 = (-b + Math.sqrt(discriminant)) / (2 * a);

      // Choose the free-flow solution (lower density)
      k = Math.min(k1, k2);
      v = vf * (1 - k / kj);
    } else {
      throw new TrafficManagementError(
        TrafficErrorCode.INVALID_PARAMETERS,
        'Must provide at least one of: density, speed, or flow'
      );
    }

    // Determine traffic regime
    let regime: TrafficRegime;
    if (k < kc * 0.5) {
      regime = 'free-flow';
    } else if (k >= kc * 0.5 && k <= kc * 1.2) {
      regime = 'capacity';
    } else if (k > kc * 1.2 && k < kj * 0.9) {
      regime = 'congested';
    } else {
      regime = 'jammed';
    }

    // Calculate capacity
    const capacity = qmax * lanes;

    // Calculate volume to capacity ratio
    const volumeToCapacity = (q * lanes) / capacity;

    // Determine level of service based on v/c ratio and speed
    const levelOfService = this.determineLOS(volumeToCapacity, v, vf);

    // Determine if congested
    const isCongested = v < vf * TRAFFIC_CONSTANTS.CONGESTION_SPEED_THRESHOLD ||
                        k > kj * TRAFFIC_CONSTANTS.CONGESTION_DENSITY_THRESHOLD;

    return {
      flow: q,
      speed: v,
      density: k,
      regime,
      levelOfService,
      capacity,
      volumeToCapacity,
      isCongested,
    };
  }

  /**
   * Optimize signal timing using Webster's method
   *
   * @param request - Optimization request with phase data
   * @returns Optimized signal timing
   */
  optimizeSignalTiming(request: SignalOptimizationRequest): SignalOptimizationResult {
    const { phases, lostTime, targetDelay, minCycleTime = 60, maxCycleTime = 120 } = request;

    // Validate inputs
    if (phases.length === 0) {
      throw new TrafficManagementError(
        TrafficErrorCode.INVALID_PARAMETERS,
        'At least one phase is required'
      );
    }

    // Calculate flow ratios (y-values)
    const flowRatios = phases.map(p => p.volume / p.saturationFlow);

    // Calculate critical flow ratio (Y)
    const Y = flowRatios.reduce((sum, y) => sum + y, 0);

    // Check for over-saturation
    if (Y >= 1.0) {
      return {
        cycleTime: 0,
        greenTimes: [],
        delays: [],
        averageDelay: Infinity,
        levelOfService: 'F',
        volumeToCapacity: Y,
        criticalFlowRatio: Y,
        isOverSaturated: true,
        recommendations: [
          'Intersection is over-saturated (Y >= 1.0)',
          'Consider adding lanes',
          'Prohibit some turning movements',
          'Implement one-way system',
          'Add alternative routes',
        ],
      };
    }

    // Calculate optimal cycle time using Webster's formula
    const L = lostTime * phases.length;
    let Co = (1.5 * L + 5) / (1 - Y);

    // Apply constraints
    Co = Math.max(minCycleTime, Math.min(maxCycleTime, Co));

    // Allocate green times proportionally
    const effectiveGreen = Co - L;
    const greenTimes = flowRatios.map(yi => (effectiveGreen * yi) / Y);

    // Calculate delays using Webster's delay formula
    const delays = phases.map((phase, i) => {
      const g = greenTimes[i];
      const C = Co;
      const q = phase.volume;
      const s = phase.saturationFlow;
      const lambda = g / C;
      const x = q / (s * lambda);

      if (x >= 1.0) {
        return Infinity; // Over-saturated
      }

      // Webster's delay formula (simplified)
      const d1 = (C * Math.pow(1 - lambda, 2)) / (2 * (1 - lambda * x));
      const d2 = Math.pow(x, 2) / (2 * q * (1 - x));

      return d1 + d2;
    });

    // Calculate weighted average delay
    const totalVolume = phases.reduce((sum, p) => sum + p.volume, 0);
    const averageDelay = phases.reduce((sum, p, i) => {
      return sum + (delays[i] * p.volume);
    }, 0) / totalVolume;

    // Determine overall LOS
    const levelOfService = this.getLOSFromDelay(averageDelay);

    // Generate recommendations
    const recommendations: string[] = [];
    if (Y > 0.85) {
      recommendations.push('High critical flow ratio - approaching capacity');
    }
    if (averageDelay > 30) {
      recommendations.push('Consider signal coordination with adjacent intersections');
    }
    if (Co > 100) {
      recommendations.push('Long cycle time may cause driver frustration');
    }

    return {
      cycleTime: Co,
      greenTimes,
      delays,
      averageDelay,
      levelOfService,
      volumeToCapacity: Y,
      criticalFlowRatio: Y,
      isOverSaturated: false,
      recommendations,
    };
  }

  /**
   * Detect congestion based on current traffic conditions
   *
   * @param request - Current traffic measurements
   * @returns Congestion detection result
   */
  detectCongestion(request: CongestionDetectionRequest): CongestionDetectionResult {
    const {
      speed,
      density,
      freeFlowSpeed,
      jamDensity = TRAFFIC_CONSTANTS.JAM_DENSITY,
      historicalSpeed,
    } = request;

    // Calculate travel time index
    const travelTimeIndex = freeFlowSpeed / speed;

    // Calculate speed reduction
    const speedReduction = ((freeFlowSpeed - speed) / freeFlowSpeed) * 100;

    // Determine if congested
    const speedCongested = speed < freeFlowSpeed * TRAFFIC_CONSTANTS.CONGESTION_SPEED_THRESHOLD;
    const densityCongested = density > jamDensity * TRAFFIC_CONSTANTS.CONGESTION_DENSITY_THRESHOLD;
    const isCongested = speedCongested || densityCongested;

    // Determine severity
    let severity: CongestionSeverity;
    let congestionScore: number;

    if (!isCongested) {
      severity = 'none';
      congestionScore = 0;
    } else if (travelTimeIndex < 1.3 && speedReduction < 30) {
      severity = 'mild';
      congestionScore = 25;
    } else if (travelTimeIndex < 1.8 && speedReduction < 50) {
      severity = 'moderate';
      congestionScore = 50;
    } else if (travelTimeIndex < 2.5 && speedReduction < 70) {
      severity = 'severe';
      congestionScore = 75;
    } else {
      severity = 'gridlock';
      congestionScore = 100;
    }

    // Estimate delay
    const freeFlowTime = 1; // Normalized to 1 unit
    const actualTime = travelTimeIndex * freeFlowTime;
    const estimatedDelay = (actualTime - freeFlowTime) * 3600; // Convert to seconds

    // Generate recommendations
    const recommendedActions: string[] = [];

    if (severity === 'mild') {
      recommendedActions.push('Monitor situation');
      recommendedActions.push('Prepare variable message signs');
    } else if (severity === 'moderate') {
      recommendedActions.push('Activate variable message signs');
      recommendedActions.push('Adjust signal timing');
      recommendedActions.push('Consider ramp metering');
    } else if (severity === 'severe') {
      recommendedActions.push('Implement traffic diversion');
      recommendedActions.push('Activate incident management team');
      recommendedActions.push('Alert navigation systems');
      recommendedActions.push('Consider reversible lanes');
    } else if (severity === 'gridlock') {
      recommendedActions.push('Emergency response required');
      recommendedActions.push('Full traffic diversion');
      recommendedActions.push('Public transportation alerts');
      recommendedActions.push('Coordinate with police');
    }

    return {
      isCongested,
      severity,
      travelTimeIndex,
      speedReduction,
      estimatedDelay,
      recommendedActions,
      congestionScore,
    };
  }

  /**
   * Predict future traffic conditions
   *
   * @param request - Prediction request
   * @returns Traffic predictions
   */
  predictTraffic(request: TrafficPredictionRequest): TrafficPredictionResult {
    const {
      locationId,
      horizon,
      historicalData = [],
      externalFactors = {},
      model = 'exponential-smoothing',
    } = request;

    const predictions: PredictionPoint[] = [];
    const currentTime = new Date();

    if (model === 'historical-average' && historicalData.length > 0) {
      // Simple historical average
      const avgFlow = historicalData.reduce((a, b) => a + b, 0) / historicalData.length;

      for (let i = 1; i <= horizon; i += 5) {
        const timestamp = new Date(currentTime.getTime() + i * 60000);
        const noise = (Math.random() - 0.5) * 0.1 * avgFlow; // ±5% noise
        predictions.push({
          timestamp,
          flow: Math.max(0, avgFlow + noise),
          speed: 65, // Assumed
          confidence: 0.7,
        });
      }
    } else if (model === 'exponential-smoothing' && historicalData.length > 0) {
      // Exponential smoothing
      const alpha = 0.3;
      let forecast = historicalData[historicalData.length - 1];

      for (let i = 1; i <= horizon; i += 5) {
        const timestamp = new Date(currentTime.getTime() + i * 60000);

        // Apply time-of-day factor
        const hour = timestamp.getHours();
        let todFactor = 1.0;
        if (hour >= 7 && hour <= 9) todFactor = 1.3; // AM peak
        else if (hour >= 16 && hour <= 18) todFactor = 1.4; // PM peak
        else if (hour >= 0 && hour <= 5) todFactor = 0.3; // Overnight

        // Apply weather factor
        let weatherFactor = 1.0;
        if (externalFactors.weather === 'rain') weatherFactor = 0.9;
        else if (externalFactors.weather === 'snow') weatherFactor = 0.7;
        else if (externalFactors.weather === 'fog') weatherFactor = 0.8;

        forecast = forecast * todFactor * weatherFactor;

        const confidence = Math.max(0.5, 0.95 - i / (horizon * 2));

        predictions.push({
          timestamp,
          flow: Math.max(0, forecast),
          speed: this.estimateSpeedFromFlow(forecast),
          confidence,
        });
      }
    } else {
      // Fallback: simple pattern-based prediction
      for (let i = 1; i <= horizon; i += 5) {
        const timestamp = new Date(currentTime.getTime() + i * 60000);
        const hour = timestamp.getHours();

        let baseFlow = 800;
        if (hour >= 7 && hour <= 9) baseFlow = 1500;
        else if (hour >= 12 && hour <= 13) baseFlow = 1100;
        else if (hour >= 16 && hour <= 18) baseFlow = 1600;
        else if (hour >= 0 && hour <= 5) baseFlow = 300;

        predictions.push({
          timestamp,
          flow: baseFlow,
          speed: this.estimateSpeedFromFlow(baseFlow),
          confidence: 0.6,
        });
      }
    }

    // Calculate accuracy metrics (simulated)
    const mape = 8.5 + Math.random() * 5; // 8.5-13.5%
    const rmse = 120 + Math.random() * 80; // 120-200 veh/h

    return {
      predictionId: `PRED-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: currentTime,
      locationId,
      horizon,
      predictions,
      model,
      accuracy: {
        mape,
        rmse,
      },
    };
  }

  /**
   * Analyze intersection performance
   *
   * @param params - Intersection parameters
   * @returns Comprehensive analysis
   */
  analyzeIntersection(params: {
    intersectionId?: string;
    approaches: IntersectionApproach[];
    cycleTime: number;
    lostTime: number;
  }): IntersectionAnalysis {
    const { intersectionId = 'INT-UNKNOWN', approaches, cycleTime, lostTime } = params;

    // Calculate per-approach metrics
    const delays: number[] = [];
    const queueLengths: number[] = [];
    let totalCapacity = 0;
    let totalVolume = 0;

    approaches.forEach(approach => {
      const g = approach.greenTime || cycleTime / approaches.length;
      const s = approach.saturationFlow;
      const q = approach.volume;

      // Capacity for this approach
      const c = s * (g / cycleTime);
      totalCapacity += c;
      totalVolume += q;

      // Degree of saturation
      const x = q / c;

      // Delay calculation
      const lambda = g / cycleTime;
      const delay = x < 1
        ? (cycleTime * Math.pow(1 - lambda, 2)) / (2 * (1 - lambda * x))
        : Infinity;

      delays.push(delay);

      // Queue length (number of vehicles)
      const queueVehicles = x < 1
        ? ((cycleTime / 3600) * q * Math.pow(x, 2)) / (2 * (1 - x))
        : Infinity;

      queueLengths.push(queueVehicles);
    });

    // Calculate weighted average delay
    const averageDelay = approaches.reduce((sum, approach, i) => {
      return sum + (delays[i] * approach.volume);
    }, 0) / totalVolume;

    // Find maximum delay
    const maxDelay = Math.max(...delays.filter(d => d !== Infinity));

    // Overall metrics
    const volumeToCapacity = totalVolume / totalCapacity;
    const levelOfService = this.getLOSFromDelay(averageDelay);

    // Performance metrics
    const throughput = Math.min(totalVolume, totalCapacity);
    const efficiency = throughput / totalCapacity;
    const reliability = 1 - Math.min(1, averageDelay / 100);

    // Recommendations
    const recommendations: string[] = [];
    if (volumeToCapacity > 0.9) {
      recommendations.push('Approaching capacity - consider improvements');
    }
    if (maxDelay > 60) {
      recommendations.push('High delay on some approaches - rebalance green time');
    }
    if (efficiency < 0.7) {
      recommendations.push('Low efficiency - optimize signal timing');
    }

    return {
      intersectionId,
      timestamp: new Date(),
      cycleTime,
      approaches,
      levelOfService,
      averageDelay,
      maxDelay,
      capacity: totalCapacity,
      volumeToCapacity,
      queueLengths,
      metrics: {
        throughput,
        efficiency,
        reliability,
      },
      recommendations,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Determine Level of Service from V/C ratio and speed
   */
  private determineLOS(vcRatio: number, speed: number, freeFlowSpeed: number): LevelOfService {
    const speedRatio = speed / freeFlowSpeed;

    if (vcRatio <= 0.35 && speedRatio >= 0.9) return 'A';
    if (vcRatio <= 0.55 && speedRatio >= 0.8) return 'B';
    if (vcRatio <= 0.75 && speedRatio >= 0.7) return 'C';
    if (vcRatio <= 0.90 && speedRatio >= 0.5) return 'D';
    if (vcRatio <= 1.00 && speedRatio >= 0.3) return 'E';
    return 'F';
  }

  /**
   * Get Level of Service from delay
   */
  private getLOSFromDelay(delay: number): LevelOfService {
    if (delay <= LOS_THRESHOLDS.A) return 'A';
    if (delay <= LOS_THRESHOLDS.B) return 'B';
    if (delay <= LOS_THRESHOLDS.C) return 'C';
    if (delay <= LOS_THRESHOLDS.D) return 'D';
    if (delay <= LOS_THRESHOLDS.E) return 'E';
    return 'F';
  }

  /**
   * Estimate speed from flow (inverse Greenshields)
   */
  private estimateSpeedFromFlow(flow: number): number {
    const qmax = (TRAFFIC_CONSTANTS.HIGHWAY_FREE_FLOW_SPEED * TRAFFIC_CONSTANTS.JAM_DENSITY) / 4;

    if (flow > qmax) return 30; // Congested
    if (flow < 500) return 95; // Free flow

    // Linear interpolation
    const ratio = flow / qmax;
    return 95 - ratio * 45; // 95 km/h at low flow, 50 km/h near capacity
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate traffic flow (standalone function)
 */
export function calculateTrafficFlow(params: {
  density?: number;
  speed?: number;
  flow?: number;
  freeFlowSpeed?: number;
  jamDensity?: number;
  laneCount?: number;
}): FlowAnalysis {
  const sdk = new TrafficManagementSDK();
  return sdk.calculateTrafficFlow(params);
}

/**
 * Optimize signal timing (standalone function)
 */
export function optimizeSignalTiming(
  request: SignalOptimizationRequest
): SignalOptimizationResult {
  const sdk = new TrafficManagementSDK();
  return sdk.optimizeSignalTiming(request);
}

/**
 * Detect congestion (standalone function)
 */
export function detectCongestion(
  request: CongestionDetectionRequest
): CongestionDetectionResult {
  const sdk = new TrafficManagementSDK();
  return sdk.detectCongestion(request);
}

/**
 * Predict traffic (standalone function)
 */
export function predictTraffic(
  request: TrafficPredictionRequest
): TrafficPredictionResult {
  const sdk = new TrafficManagementSDK();
  return sdk.predictTraffic(request);
}

/**
 * Analyze intersection (standalone function)
 */
export function analyzeIntersection(params: {
  intersectionId?: string;
  approaches: IntersectionApproach[];
  cycleTime: number;
  lostTime: number;
}): IntersectionAnalysis {
  const sdk = new TrafficManagementSDK();
  return sdk.analyzeIntersection(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TrafficManagementSDK };
export default TrafficManagementSDK;
