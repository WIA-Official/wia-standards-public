/**
 * WIA-DEF-012: Space Surveillance SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Space Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for space surveillance including:
 * - Space object tracking and catalog management
 * - Orbital determination and propagation
 * - Conjunction assessment and collision risk analysis
 * - Debris tracking and characterization
 * - Threat detection and anomaly analysis
 */

import {
  SpaceObject,
  TrackingRequest,
  TrackingResult,
  ConjunctionRequest,
  ConjunctionResult,
  RiskAssessmentRequest,
  RiskAssessmentResult,
  RiskLevel,
  ManeuverPlan,
  CatalogSearchParams,
  CatalogUpdateRequest,
  DebrisAnalysisRequest,
  DebrisAnalysisResult,
  DecayPredictionRequest,
  DecayPredictionResult,
  KeplerianElements,
  StateVector,
  Vector3,
  TLE,
  CovarianceMatrix,
  SPACE_CONSTANTS,
  SpaceSurveillanceErrorCode,
  SpaceSurveillanceError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-012 Space Surveillance SDK
 */
export class SpaceSurveillanceSDK {
  private version = '1.0.0';
  private catalog: Map<string, SpaceObject> = new Map();

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
  // Space Object Tracking
  // ==========================================================================

  /**
   * Track a space object and update its orbital parameters
   *
   * @param request - Tracking request with observation data
   * @returns Updated tracking result
   */
  trackSpaceObject(request: TrackingRequest): TrackingResult {
    const {
      objectId,
      observationTime,
      position,
      velocity,
      uncertainty,
    } = request;

    // Validate inputs
    if (!objectId || !observationTime || !position || !velocity) {
      throw new SpaceSurveillanceError(
        SpaceSurveillanceErrorCode.INVALID_PARAMETERS,
        'Missing required tracking parameters'
      );
    }

    // Convert state vector to orbital elements
    const orbitalElements = this.stateVectorToKeplerian(
      position,
      velocity,
      observationTime
    );

    // Calculate derived parameters
    const altitude = this.calculateAltitude(orbitalElements.semiMajorAxis);
    const period = this.calculateOrbitalPeriod(orbitalElements.semiMajorAxis);
    const orbitType = this.classifyOrbit(altitude);

    // Calculate tracking quality
    const trackingQuality = this.assessTrackingQuality(
      position,
      velocity,
      uncertainty
    );

    // Update catalog if object exists
    if (this.catalog.has(objectId)) {
      const obj = this.catalog.get(objectId)!;
      obj.orbitalElements = orbitalElements;
      obj.period = period;
      obj.inclination = orbitalElements.inclination;
    }

    return {
      objectId,
      orbitalElements,
      altitude,
      period,
      inclination: orbitalElements.inclination,
      orbitType,
      nextUpdate: new Date(observationTime.getTime() + 24 * 3600 * 1000),
      covariance: uncertainty,
      trackingQuality,
    };
  }

  /**
   * Calculate conjunction between two space objects
   *
   * @param request - Conjunction request parameters
   * @returns Conjunction event details
   */
  calculateConjunction(request: ConjunctionRequest): ConjunctionResult {
    const {
      primary,
      secondary,
      timeWindow,
      minDistance,
      startTime = new Date(),
    } = request;

    // Get objects from catalog
    const obj1 = this.catalog.get(primary);
    const obj2 = this.catalog.get(secondary);

    if (!obj1 || !obj2) {
      throw new SpaceSurveillanceError(
        SpaceSurveillanceErrorCode.OBJECT_NOT_FOUND,
        `Object not found: ${!obj1 ? primary : secondary}`
      );
    }

    if (!obj1.orbitalElements || !obj2.orbitalElements) {
      throw new SpaceSurveillanceError(
        SpaceSurveillanceErrorCode.INSUFFICIENT_DATA,
        'Orbital elements not available for conjunction calculation'
      );
    }

    // Find time of closest approach
    const { tca, missDistance, relativePosition, relativeVelocity } =
      this.findClosestApproach(
        obj1.orbitalElements,
        obj2.orbitalElements,
        startTime,
        timeWindow
      );

    // Decompose relative position into RTN components
    const { radial, alongTrack, crossTrack } =
      this.decomposeRelativePosition(relativePosition, relativeVelocity);

    // Generate conjunction ID
    const conjunctionId = `CONJ-${Date.now()}-${Math.random().toString(36).substr(2, 6)}`;

    return {
      id: conjunctionId,
      primary,
      secondary,
      timeOfClosestApproach: tca,
      missDistance,
      relativeVelocity: this.vectorMagnitude(relativeVelocity),
      radialDistance: radial,
      alongTrackDistance: alongTrack,
      crossTrackDistance: crossTrack,
      relativePosition,
      relativeVelocityVector: relativeVelocity,
      status: missDistance < minDistance ? 'ACTIVE' : 'CLEARED',
    };
  }

  /**
   * Assess collision risk for a conjunction event
   *
   * @param request - Risk assessment parameters
   * @returns Risk assessment result with probability and recommendations
   */
  assessCollisionRisk(request: RiskAssessmentRequest): RiskAssessmentResult {
    const {
      primarySize,
      secondarySize,
      positionUncertainty,
      method = 'FOSTER',
    } = request;

    // Calculate combined hard-body radius
    const combinedRadius = (primarySize + secondarySize) / 2;

    // Calculate collision probability based on method
    let probability: number;

    switch (method) {
      case 'FOSTER':
        probability = this.calculatePcFoster(combinedRadius, positionUncertainty);
        break;
      case 'CHAN':
        probability = this.calculatePcChan(
          combinedRadius,
          positionUncertainty,
          request.primaryCovariance,
          request.secondaryCovariance
        );
        break;
      case 'MONTE_CARLO':
        probability = this.calculatePcMonteCarlo(
          combinedRadius,
          positionUncertainty,
          10000
        );
        break;
      default:
        probability = this.calculatePcFoster(combinedRadius, positionUncertainty);
    }

    // Classify risk level
    const riskLevel = this.classifyRiskLevel(probability);

    // Determine if maneuver is required
    const maneuverRequired = probability > SPACE_CONSTANTS.PC_THRESHOLD.LOW;

    // Estimate delta-V for avoidance maneuver
    const estimatedDeltaV = maneuverRequired
      ? this.estimateAvoidanceDeltaV(combinedRadius, positionUncertainty)
      : undefined;

    // Generate recommendation
    const recommendedAction = this.generateRiskRecommendation(
      riskLevel,
      probability,
      maneuverRequired
    );

    return {
      probability,
      riskLevel,
      recommendedAction,
      maneuverRequired,
      estimatedDeltaV,
      combinedRadius,
    };
  }

  /**
   * Update space catalog with new or modified objects
   *
   * @param request - Catalog update request
   * @returns Number of objects updated
   */
  updateSpaceCatalog(request: CatalogUpdateRequest): number {
    const { objects, updateType } = request;

    let updateCount = 0;

    for (const obj of objects) {
      switch (updateType) {
        case 'NEW':
        case 'MODIFIED':
          this.catalog.set(obj.objectId, obj);
          updateCount++;
          break;
        case 'DELETED':
          if (this.catalog.delete(obj.objectId)) {
            updateCount++;
          }
          break;
      }
    }

    return updateCount;
  }

  /**
   * Search space catalog with filters
   *
   * @param params - Search parameters
   * @returns Matching space objects
   */
  searchCatalog(params: CatalogSearchParams): SpaceObject[] {
    let results = Array.from(this.catalog.values());

    // Apply filters
    if (params.objectType && params.objectType.length > 0) {
      results = results.filter((obj) =>
        params.objectType!.includes(obj.objectType)
      );
    }

    if (params.status && params.status.length > 0) {
      results = results.filter((obj) => params.status!.includes(obj.status));
    }

    if (params.country && params.country.length > 0) {
      results = results.filter(
        (obj) => obj.country && params.country!.includes(obj.country)
      );
    }

    if (params.name) {
      const searchName = params.name.toLowerCase();
      results = results.filter(
        (obj) => obj.name && obj.name.toLowerCase().includes(searchName)
      );
    }

    if (params.minAltitude !== undefined || params.maxAltitude !== undefined) {
      results = results.filter((obj) => {
        if (!obj.perigee) return false;
        if (params.minAltitude && obj.perigee < params.minAltitude) return false;
        if (params.maxAltitude && obj.apogee && obj.apogee > params.maxAltitude)
          return false;
        return true;
      });
    }

    // Apply limit
    if (params.limit) {
      results = results.slice(0, params.limit);
    }

    return results;
  }

  /**
   * Analyze debris environment in specified orbit
   *
   * @param request - Debris analysis parameters
   * @returns Debris analysis results
   */
  analyzeDebris(request: DebrisAnalysisRequest): DebrisAnalysisResult {
    const { orbit, sizeRange } = request;

    // Get altitude range for orbit type
    const altitudeRange = this.getAltitudeRangeForOrbit(orbit);

    // Filter debris in specified orbit
    const debrisObjects = Array.from(this.catalog.values()).filter(
      (obj) =>
        obj.objectType === 'DEBRIS' &&
        obj.perigee &&
        obj.perigee >= altitudeRange.min &&
        obj.perigee <= altitudeRange.max
    );

    // Categorize by size
    const bySize = {
      large: debrisObjects.filter((obj) => obj.rcs === 'LARGE').length,
      medium: debrisObjects.filter((obj) => obj.rcs === 'MEDIUM').length,
      small: debrisObjects.filter((obj) => obj.rcs === 'SMALL').length,
    };

    // Create spatial distribution
    const distribution = this.calculateDebrisDistribution(debrisObjects);

    // Calculate collision risk index
    const collisionRiskIndex = this.calculateCollisionRiskIndex(
      debrisObjects.length,
      orbit
    );

    return {
      totalCount: debrisObjects.length,
      bySize,
      distribution,
      collisionRiskIndex,
    };
  }

  /**
   * Predict orbital decay and re-entry
   *
   * @param request - Decay prediction parameters
   * @returns Decay prediction results
   */
  predictDecay(request: DecayPredictionRequest): DecayPredictionResult {
    const { objectId, horizon } = request;

    const obj = this.catalog.get(objectId);
    if (!obj) {
      throw new SpaceSurveillanceError(
        SpaceSurveillanceErrorCode.OBJECT_NOT_FOUND,
        `Object not found: ${objectId}`
      );
    }

    if (!obj.perigee || !obj.orbitalElements) {
      throw new SpaceSurveillanceError(
        SpaceSurveillanceErrorCode.INSUFFICIENT_DATA,
        'Insufficient data for decay prediction'
      );
    }

    // Calculate decay rate (simplified model)
    const decayRate = this.calculateDecayRate(
      obj.perigee,
      obj.physical?.dragCoefficient || 2.2,
      obj.physical?.crossSectionalArea || 10,
      obj.physical?.mass || 1000
    );

    // Estimate lifetime
    const lifetimeEstimate = obj.perigee / Math.abs(decayRate);

    // Predict decay date
    const decayDate =
      lifetimeEstimate < horizon
        ? new Date(Date.now() + lifetimeEstimate * 24 * 3600 * 1000)
        : null;

    // Calculate uncertainty (10% of lifetime)
    const uncertainty = lifetimeEstimate * 0.1;

    return {
      objectId,
      decayDate,
      uncertainty,
      lifetimeEstimate,
      decayRate,
    };
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Convert state vector to Keplerian elements
   */
  private stateVectorToKeplerian(
    position: Vector3,
    velocity: Vector3,
    epoch: Date
  ): KeplerianElements {
    const r = this.vectorMagnitude(position);
    const v = this.vectorMagnitude(velocity);
    const mu = SPACE_CONSTANTS.EARTH_MU;

    // Calculate specific angular momentum
    const h = this.crossProduct(position, velocity);
    const hMag = this.vectorMagnitude(h);

    // Calculate eccentricity vector
    const eVec = this.subtract(
      this.scale(velocity, (v * v - mu / r) / mu),
      this.scale(position, this.dotProduct(position, velocity) / (mu * r))
    );
    const eccentricity = this.vectorMagnitude(eVec);

    // Calculate semi-major axis
    const energy = (v * v) / 2 - mu / r;
    const semiMajorAxis = -mu / (2 * energy);

    // Calculate inclination
    const inclination = (Math.acos(h.z / hMag) * 180) / Math.PI;

    // Calculate RAAN
    const nVec = this.crossProduct({ x: 0, y: 0, z: 1 }, h);
    const nMag = this.vectorMagnitude(nVec);
    let raan = (Math.acos(nVec.x / nMag) * 180) / Math.PI;
    if (nVec.y < 0) raan = 360 - raan;

    // Calculate argument of periapsis
    let argumentOfPerigee =
      (Math.acos(this.dotProduct(nVec, eVec) / (nMag * eccentricity)) * 180) /
      Math.PI;
    if (eVec.z < 0) argumentOfPerigee = 360 - argumentOfPerigee;

    // Calculate mean anomaly (simplified)
    const trueAnomaly =
      (Math.acos(
        this.dotProduct(eVec, position) / (eccentricity * r)
      ) * 180) /
      Math.PI;
    const eccentricAnomaly =
      2 *
      Math.atan(
        Math.sqrt((1 - eccentricity) / (1 + eccentricity)) *
          Math.tan((trueAnomaly * Math.PI) / 360)
      );
    const meanAnomaly =
      ((eccentricAnomaly - eccentricity * Math.sin(eccentricAnomaly)) * 180) /
      Math.PI;

    return {
      semiMajorAxis,
      eccentricity,
      inclination,
      raan,
      argumentOfPerigee,
      meanAnomaly,
      epoch,
    };
  }

  /**
   * Calculate altitude from semi-major axis
   */
  private calculateAltitude(semiMajorAxis: number): number {
    return (semiMajorAxis - SPACE_CONSTANTS.EARTH_RADIUS) / 1000; // km
  }

  /**
   * Calculate orbital period from semi-major axis
   */
  private calculateOrbitalPeriod(semiMajorAxis: number): number {
    const period =
      (2 * Math.PI * Math.sqrt(Math.pow(semiMajorAxis, 3) / SPACE_CONSTANTS.EARTH_MU)) /
      60;
    return period; // minutes
  }

  /**
   * Classify orbit type by altitude
   */
  private classifyOrbit(
    altitude: number
  ): 'LEO' | 'MEO' | 'GEO' | 'HEO' | 'UNKNOWN' {
    if (altitude < SPACE_CONSTANTS.LEO_ALTITUDE.max) return 'LEO';
    if (altitude < SPACE_CONSTANTS.MEO_ALTITUDE.max) return 'MEO';
    if (Math.abs(altitude - SPACE_CONSTANTS.GEO_ALTITUDE) < 100) return 'GEO';
    if (altitude > SPACE_CONSTANTS.MEO_ALTITUDE.max) return 'HEO';
    return 'UNKNOWN';
  }

  /**
   * Find closest approach between two orbits
   */
  private findClosestApproach(
    elem1: KeplerianElements,
    elem2: KeplerianElements,
    startTime: Date,
    timeWindow: number
  ): {
    tca: Date;
    missDistance: number;
    relativePosition: Vector3;
    relativeVelocity: Vector3;
  } {
    // Simplified closest approach calculation
    // In practice, would use numerical optimization

    let minDistance = Infinity;
    let tcaTime = startTime;
    let relPos: Vector3 = { x: 0, y: 0, z: 0 };
    let relVel: Vector3 = { x: 0, y: 0, z: 0 };

    const steps = 100;
    const dt = timeWindow / steps;

    for (let i = 0; i < steps; i++) {
      const t = new Date(startTime.getTime() + i * dt * 1000);

      // Propagate both orbits (simplified)
      const pos1 = this.propagateOrbit(elem1, t);
      const pos2 = this.propagateOrbit(elem2, t);
      const vel1 = this.calculateVelocity(elem1);
      const vel2 = this.calculateVelocity(elem2);

      const relPosition = this.subtract(pos2, pos1);
      const distance = this.vectorMagnitude(relPosition);

      if (distance < minDistance) {
        minDistance = distance;
        tcaTime = t;
        relPos = relPosition;
        relVel = this.subtract(vel2, vel1);
      }
    }

    return {
      tca: tcaTime,
      missDistance: minDistance,
      relativePosition: relPos,
      relativeVelocity: relVel,
    };
  }

  /**
   * Propagate orbit to specific time (simplified)
   */
  private propagateOrbit(elem: KeplerianElements, time: Date): Vector3 {
    // Simplified Keplerian propagation
    // In practice, would use SGP4/SDP4 or numerical integration

    const { semiMajorAxis, eccentricity, inclination, raan, argumentOfPerigee } = elem;

    const r = semiMajorAxis * (1 - eccentricity * eccentricity);
    const theta = 0; // Simplified - would calculate based on time

    // Convert to Cartesian (simplified)
    const x = r * Math.cos(theta);
    const y = r * Math.sin(theta);
    const z = 0;

    return { x, y, z };
  }

  /**
   * Calculate velocity from orbital elements (simplified)
   */
  private calculateVelocity(elem: KeplerianElements): Vector3 {
    const v = Math.sqrt(SPACE_CONSTANTS.EARTH_MU / elem.semiMajorAxis);
    return { x: 0, y: v, z: 0 }; // Simplified
  }

  /**
   * Decompose relative position into RTN components
   */
  private decomposeRelativePosition(
    relPosition: Vector3,
    relVelocity: Vector3
  ): { radial: number; alongTrack: number; crossTrack: number } {
    const rMag = this.vectorMagnitude(relPosition);
    const radial = rMag; // Simplified

    return {
      radial,
      alongTrack: 0, // Simplified
      crossTrack: 0, // Simplified
    };
  }

  /**
   * Calculate collision probability using Foster method
   */
  private calculatePcFoster(radius: number, sigma: number): number {
    return 1 - Math.exp(-(radius * radius) / (2 * sigma * sigma));
  }

  /**
   * Calculate collision probability using Chan method (2D)
   */
  private calculatePcChan(
    radius: number,
    sigma: number,
    cov1?: CovarianceMatrix,
    cov2?: CovarianceMatrix
  ): number {
    // Simplified 2D Chan method
    // In practice, would perform full 2D integration
    return this.calculatePcFoster(radius, sigma) * 0.8; // Approximation
  }

  /**
   * Calculate collision probability using Monte Carlo
   */
  private calculatePcMonteCarlo(
    radius: number,
    sigma: number,
    samples: number
  ): number {
    let collisions = 0;

    for (let i = 0; i < samples; i++) {
      // Sample from normal distribution
      const dx = this.randomNormal(0, sigma);
      const dy = this.randomNormal(0, sigma);
      const dz = this.randomNormal(0, sigma);

      const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

      if (distance < radius) {
        collisions++;
      }
    }

    return collisions / samples;
  }

  /**
   * Classify risk level from probability
   */
  private classifyRiskLevel(probability: number): RiskLevel {
    if (probability > SPACE_CONSTANTS.PC_THRESHOLD.HIGH) return 'CRITICAL';
    if (probability > SPACE_CONSTANTS.PC_THRESHOLD.MEDIUM) return 'HIGH';
    if (probability > SPACE_CONSTANTS.PC_THRESHOLD.LOW) return 'MEDIUM';
    if (probability > SPACE_CONSTANTS.PC_THRESHOLD.MINIMAL) return 'LOW';
    return 'MINIMAL';
  }

  /**
   * Estimate delta-V for collision avoidance
   */
  private estimateAvoidanceDeltaV(radius: number, uncertainty: number): number {
    // Simplified delta-V estimation
    // Typical avoidance maneuver: 0.5-5 m/s
    const baseDeltaV = 1.0; // m/s
    const scaleFactor = Math.min(radius / 1000, 5);
    return baseDeltaV * scaleFactor;
  }

  /**
   * Generate risk recommendation
   */
  private generateRiskRecommendation(
    riskLevel: RiskLevel,
    probability: number,
    maneuverRequired: boolean
  ): string {
    switch (riskLevel) {
      case 'CRITICAL':
        return 'IMMEDIATE MANEUVER REQUIRED - Execute collision avoidance within next orbit';
      case 'HIGH':
        return 'MANEUVER STRONGLY RECOMMENDED - Prepare for collision avoidance';
      case 'MEDIUM':
        return 'Monitor closely and prepare maneuver options';
      case 'LOW':
        return 'Continue monitoring - No immediate action required';
      case 'MINIMAL':
        return 'Routine tracking sufficient';
      default:
        return 'Unknown risk level';
    }
  }

  /**
   * Assess tracking quality
   */
  private assessTrackingQuality(
    position: Vector3,
    velocity: Vector3,
    uncertainty?: CovarianceMatrix
  ): number {
    // Simplified quality assessment
    if (!uncertainty) return 0.8;

    // Check position magnitude
    const posQuality = this.vectorMagnitude(position) > 0 ? 1.0 : 0.0;

    // Check velocity magnitude
    const velQuality = this.vectorMagnitude(velocity) > 0 ? 1.0 : 0.0;

    return (posQuality + velQuality) / 2;
  }

  /**
   * Get altitude range for orbit type
   */
  private getAltitudeRangeForOrbit(
    orbit: 'LEO' | 'MEO' | 'GEO'
  ): { min: number; max: number } {
    switch (orbit) {
      case 'LEO':
        return SPACE_CONSTANTS.LEO_ALTITUDE;
      case 'MEO':
        return SPACE_CONSTANTS.MEO_ALTITUDE;
      case 'GEO':
        return {
          min: SPACE_CONSTANTS.GEO_ALTITUDE - 100,
          max: SPACE_CONSTANTS.GEO_ALTITUDE + 100,
        };
    }
  }

  /**
   * Calculate debris spatial distribution
   */
  private calculateDebrisDistribution(
    debris: SpaceObject[]
  ): { altitude: number; count: number }[] {
    const bins = new Map<number, number>();

    for (const obj of debris) {
      if (!obj.perigee) continue;
      const bin = Math.floor(obj.perigee / 100) * 100;
      bins.set(bin, (bins.get(bin) || 0) + 1);
    }

    return Array.from(bins.entries())
      .map(([altitude, count]) => ({ altitude, count }))
      .sort((a, b) => a.altitude - b.altitude);
  }

  /**
   * Calculate collision risk index
   */
  private calculateCollisionRiskIndex(
    debrisCount: number,
    orbit: 'LEO' | 'MEO' | 'GEO'
  ): number {
    // Simplified risk index: 0-1 scale
    const baselineDebris = orbit === 'LEO' ? 20000 : orbit === 'MEO' ? 3000 : 2500;
    return Math.min(debrisCount / baselineDebris, 1.0);
  }

  /**
   * Calculate orbital decay rate
   */
  private calculateDecayRate(
    altitude: number,
    dragCoeff: number,
    area: number,
    mass: number
  ): number {
    // Simplified drag-based decay model
    // Actual implementation would use atmospheric density models

    if (altitude > 600) return -0.001; // km/day (very slow decay)
    if (altitude > 400) return -0.01; // km/day (slow decay)
    if (altitude > 250) return -0.1; // km/day (moderate decay)
    return -1.0; // km/day (rapid decay)
  }

  // ==========================================================================
  // Vector Math Utilities
  // ==========================================================================

  private vectorMagnitude(v: Vector3): number {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  private dotProduct(v1: Vector3, v2: Vector3): number {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
  }

  private crossProduct(v1: Vector3, v2: Vector3): Vector3 {
    return {
      x: v1.y * v2.z - v1.z * v2.y,
      y: v1.z * v2.x - v1.x * v2.z,
      z: v1.x * v2.y - v1.y * v2.x,
    };
  }

  private subtract(v1: Vector3, v2: Vector3): Vector3 {
    return {
      x: v1.x - v2.x,
      y: v1.y - v2.y,
      z: v1.z - v2.z,
    };
  }

  private scale(v: Vector3, s: number): Vector3 {
    return {
      x: v.x * s,
      y: v.y * s,
      z: v.z * s,
    };
  }

  private randomNormal(mean: number, stdDev: number): number {
    // Box-Muller transform
    const u1 = Math.random();
    const u2 = Math.random();
    const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    return z0 * stdDev + mean;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Track space object (standalone function)
 */
export function trackSpaceObject(request: TrackingRequest): TrackingResult {
  const sdk = new SpaceSurveillanceSDK();
  return sdk.trackSpaceObject(request);
}

/**
 * Calculate conjunction (standalone function)
 */
export function calculateConjunction(
  request: ConjunctionRequest
): ConjunctionResult {
  const sdk = new SpaceSurveillanceSDK();
  return sdk.calculateConjunction(request);
}

/**
 * Assess collision risk (standalone function)
 */
export function assessCollisionRisk(
  request: RiskAssessmentRequest
): RiskAssessmentResult {
  const sdk = new SpaceSurveillanceSDK();
  return sdk.assessCollisionRisk(request);
}

/**
 * Update space catalog (standalone function)
 */
export function updateSpaceCatalog(request: CatalogUpdateRequest): number {
  const sdk = new SpaceSurveillanceSDK();
  return sdk.updateSpaceCatalog(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SpaceSurveillanceSDK };
export default SpaceSurveillanceSDK;
