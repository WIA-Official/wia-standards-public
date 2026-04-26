/**
 * WIA-AUTO-015: Autonomous Ship SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Maritime Autonomy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive TypeScript SDK for autonomous maritime navigation,
 * collision avoidance, route planning, and remote operations.
 */

import {
  Position,
  ShipState,
  CollisionAssessment,
  Route,
  NavigationParams,
  AutonomyLevel,
  ShipConfig,
  SystemStatus,
  CollisionWarning,
  AvoidanceManeuver,
  WeatherConditions,
  MARITIME_CONSTANTS,
  AutoShipErrorCode,
  AutonomousShipError,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Navigation Functions
// ============================================================================

/**
 * Calculate great circle distance between two positions
 *
 * @param pos1 - First position
 * @param pos2 - Second position
 * @returns Distance in nautical miles
 *
 * @example
 * const distance = calculateDistance(
 *   { latitude: 35.6762, longitude: 139.6503 },
 *   { latitude: 1.2897, longitude: 103.8501 }
 * );
 * console.log(`Distance: ${distance} NM`);
 */
export function calculateDistance(pos1: Position, pos2: Position): number {
  const toRadians = (deg: number) => (deg * Math.PI) / 180;

  const φ1 = toRadians(pos1.latitude);
  const φ2 = toRadians(pos2.latitude);
  const Δφ = toRadians(pos2.latitude - pos1.latitude);
  const Δλ = toRadians(pos2.longitude - pos1.longitude);

  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return MARITIME_CONSTANTS.EARTH_RADIUS_NM * c;
}

/**
 * Calculate initial bearing from one position to another
 *
 * @param from - Starting position
 * @param to - Destination position
 * @returns Initial bearing in degrees (0-360)
 *
 * @example
 * const bearing = calculateBearing(
 *   { latitude: 35.6762, longitude: 139.6503 },
 *   { latitude: 1.2897, longitude: 103.8501 }
 * );
 * console.log(`Bearing: ${bearing}°`);
 */
export function calculateBearing(from: Position, to: Position): number {
  const toRadians = (deg: number) => (deg * Math.PI) / 180;
  const toDegrees = (rad: number) => (rad * 180) / Math.PI;

  const φ1 = toRadians(from.latitude);
  const φ2 = toRadians(to.latitude);
  const Δλ = toRadians(to.longitude - from.longitude);

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x =
    Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);

  const θ = Math.atan2(y, x);

  return (toDegrees(θ) + 360) % 360;
}

/**
 * Calculate cross-track error from planned route
 *
 * @param currentPos - Current ship position
 * @param routeStart - Route start position
 * @param routeEnd - Route end position
 * @returns Cross-track error in nautical miles (positive = right of track)
 */
export function calculateCrossTrackError(
  currentPos: Position,
  routeStart: Position,
  routeEnd: Position
): number {
  const toRadians = (deg: number) => (deg * Math.PI) / 180;

  const d13 = calculateDistance(routeStart, currentPos);
  const θ13 = toRadians(calculateBearing(routeStart, currentPos));
  const θ12 = toRadians(calculateBearing(routeStart, routeEnd));

  const xte =
    Math.asin(
      Math.sin(d13 / MARITIME_CONSTANTS.EARTH_RADIUS_NM) * Math.sin(θ13 - θ12)
    ) * MARITIME_CONSTANTS.EARTH_RADIUS_NM;

  return xte;
}

// ============================================================================
// Collision Avoidance
// ============================================================================

/**
 * Calculate collision risk assessment between two ships
 *
 * @param ownShip - Own ship state
 * @param target - Target ship state
 * @returns Collision assessment with CPA, TCPA, and risk level
 *
 * @example
 * const collision = calculateCollisionRisk(
 *   {
 *     position: { latitude: 35.6762, longitude: 139.6503 },
 *     heading: 90,
 *     speed: 15
 *   },
 *   {
 *     position: { latitude: 35.6850, longitude: 139.7500 },
 *     heading: 270,
 *     speed: 12
 *   }
 * );
 * console.log(`CPA: ${collision.cpa} NM, TCPA: ${collision.tcpa} minutes`);
 */
export function calculateCollisionRisk(
  ownShip: ShipState,
  target: ShipState
): CollisionAssessment {
  // Convert speeds from knots to m/s for calculation
  const ownSpeedMps = ownShip.speed * MARITIME_CONSTANTS.KNOTS_TO_MPS;
  const targetSpeedMps = target.speed * MARITIME_CONSTANTS.KNOTS_TO_MPS;

  // Convert headings to radians
  const toRadians = (deg: number) => (deg * Math.PI) / 180;
  const ownHeadingRad = toRadians(ownShip.heading);
  const targetHeadingRad = toRadians(target.heading);

  // Calculate velocity components
  const ownVx = ownSpeedMps * Math.sin(ownHeadingRad);
  const ownVy = ownSpeedMps * Math.cos(ownHeadingRad);
  const targetVx = targetSpeedMps * Math.sin(targetHeadingRad);
  const targetVy = targetSpeedMps * Math.cos(targetHeadingRad);

  // Relative velocity
  const relVx = targetVx - ownVx;
  const relVy = targetVy - ownVy;
  const relSpeed = Math.sqrt(relVx * relVx + relVy * relVy);

  // Relative position (approximate for short distances)
  const distance = calculateDistance(ownShip.position, target.position);
  const bearing = calculateBearing(ownShip.position, target.position);
  const bearingRad = toRadians(bearing);

  const relX = distance * MARITIME_CONSTANTS.NM_TO_METERS * Math.sin(bearingRad);
  const relY = distance * MARITIME_CONSTANTS.NM_TO_METERS * Math.cos(bearingRad);

  // CPA and TCPA calculation
  let tcpaSeconds = 0;
  let cpa = distance;

  if (relSpeed > 0.01) {
    // Avoid division by very small numbers
    tcpaSeconds = -(relX * relVx + relY * relVy) / (relSpeed * relSpeed);

    if (tcpaSeconds > 0) {
      const cpaX = relX + relVx * tcpaSeconds;
      const cpaY = relY + relVy * tcpaSeconds;
      cpa = Math.sqrt(cpaX * cpaX + cpaY * cpaY) / MARITIME_CONSTANTS.NM_TO_METERS;
    }
  }

  const tcpaMinutes = tcpaSeconds / 60;

  // Calculate relative bearing (from own ship perspective)
  const relativeBearing = (bearing - ownShip.heading + 360) % 360;

  // Determine COLREG situation
  let colregSituation: CollisionAssessment['colregSituation'] = 'unknown';
  let giveWayVessel: CollisionAssessment['giveWayVessel'] = 'none';
  let standOnVessel: CollisionAssessment['standOnVessel'] = 'none';

  if (cpa < MARITIME_CONSTANTS.SAFE_PASSING_DISTANCE) {
    if (relativeBearing >= 170 && relativeBearing <= 190) {
      colregSituation = 'head-on';
      giveWayVessel = 'both';
      standOnVessel = 'both';
    } else if (relativeBearing > 112.5 && relativeBearing < 247.5) {
      colregSituation = 'overtaking';
      giveWayVessel = 'own';
      standOnVessel = 'target';
    } else if (relativeBearing >= 5 && relativeBearing <= 112.5) {
      colregSituation = 'crossing';
      giveWayVessel = 'own';
      standOnVessel = 'target';
    } else {
      colregSituation = 'safe-passing';
    }
  } else {
    colregSituation = 'safe-passing';
  }

  // Calculate risk index
  const riskIndex =
    (MARITIME_CONSTANTS.SAFE_PASSING_DISTANCE / (cpa + 0.1)) *
    (20 / (Math.abs(tcpaMinutes) + 0.1));

  // Determine risk level
  let riskLevel: CollisionAssessment['riskLevel'];
  if (riskIndex < 0.3) {
    riskLevel = 'low';
  } else if (riskIndex < 0.7) {
    riskLevel = 'medium';
  } else if (riskIndex < 1.0) {
    riskLevel = 'high';
  } else {
    riskLevel = 'critical';
  }

  return {
    cpa: parseFloat(cpa.toFixed(2)),
    tcpa: parseFloat(tcpaMinutes.toFixed(1)),
    riskIndex: parseFloat(riskIndex.toFixed(2)),
    riskLevel,
    colregSituation,
    giveWayVessel,
    standOnVessel,
    relativeBearing: parseFloat(relativeBearing.toFixed(1)),
    relativeSpeed: parseFloat((relSpeed / MARITIME_CONSTANTS.KNOTS_TO_MPS).toFixed(1)),
  };
}

/**
 * Generate recommended avoidance maneuver based on collision assessment
 *
 * @param assessment - Collision assessment
 * @param ownHeading - Current own ship heading
 * @returns Recommended avoidance maneuver
 */
export function generateAvoidanceManeuver(
  assessment: CollisionAssessment,
  ownHeading: number
): AvoidanceManeuver {
  const now = new Date();

  if (assessment.riskLevel === 'low') {
    return {
      type: 'alter-course',
      newHeading: ownHeading,
      courseChange: 0,
      reason: 'No action required - risk is low',
      executionTime: now,
    };
  }

  // Determine maneuver based on COLREG situation
  if (assessment.colregSituation === 'head-on') {
    return {
      type: 'alter-course',
      courseChange: 15,
      newHeading: (ownHeading + 15) % 360,
      reason: 'COLREG Rule 14 - Head-on situation: Turn to starboard',
      colregRule: 'Rule 14',
      executionTime: now,
      duration: 60,
    };
  } else if (
    assessment.colregSituation === 'crossing' &&
    assessment.giveWayVessel === 'own'
  ) {
    const courseChange = assessment.riskLevel === 'critical' ? 30 : 20;
    return {
      type: 'alter-course',
      courseChange,
      newHeading: (ownHeading + courseChange) % 360,
      reason: 'COLREG Rule 15 - Crossing: Give way to vessel on starboard',
      colregRule: 'Rule 15',
      executionTime: now,
      duration: 90,
    };
  } else if (assessment.colregSituation === 'overtaking') {
    return {
      type: 'reduce-speed',
      speedChange: -3,
      reason: 'COLREG Rule 13 - Overtaking: Keep clear of overtaken vessel',
      colregRule: 'Rule 13',
      executionTime: now,
      duration: 120,
    };
  }

  // Default: reduce speed for high risk
  if (assessment.riskLevel === 'critical') {
    return {
      type: 'combined',
      courseChange: 30,
      newHeading: (ownHeading + 30) % 360,
      speedChange: -5,
      reason: 'Critical risk - emergency avoidance maneuver',
      executionTime: now,
      duration: 60,
    };
  }

  return {
    type: 'reduce-speed',
    speedChange: -3,
    reason: 'Precautionary speed reduction',
    executionTime: now,
    duration: 90,
  };
}

// ============================================================================
// Route Planning
// ============================================================================

/**
 * Plan basic route between two positions
 *
 * @param origin - Origin position
 * @param destination - Destination position
 * @param params - Navigation parameters
 * @returns Basic route plan
 *
 * @example
 * const route = await planRoute(
 *   { latitude: 35.6762, longitude: 139.6503 },
 *   { latitude: 1.2897, longitude: 103.8501 },
 *   { maxSpeed: 18, optimization: { objective: 'fuel-efficiency', weatherRouted: true } }
 * );
 */
export async function planRoute(
  origin: Position,
  destination: Position,
  params?: NavigationParams
): Promise<Route> {
  const distance = calculateDistance(origin, destination);
  const bearing = calculateBearing(origin, destination);

  // Generate simple waypoints (in production, this would use advanced algorithms)
  const numWaypoints = Math.ceil(distance / 500); // One waypoint every 500 NM
  const waypoints = [];

  for (let i = 1; i <= numWaypoints; i++) {
    const fraction = i / (numWaypoints + 1);
    const wpLat = origin.latitude + (destination.latitude - origin.latitude) * fraction;
    const wpLon =
      origin.longitude + (destination.longitude - origin.longitude) * fraction;

    waypoints.push({
      id: `WP${String(i).padStart(3, '0')}`,
      position: { latitude: wpLat, longitude: wpLon },
      speed: params?.maxSpeed || 16,
      turnRadius: 0.5,
    });
  }

  // Estimate voyage parameters
  const cruisingSpeed = params?.maxSpeed || 16;
  const estimatedDuration = distance / cruisingSpeed;
  const fuelRate = 8.5; // tons per day
  const estimatedFuel = (estimatedDuration / 24) * fuelRate;

  return {
    id: `RT-${new Date().toISOString().split('T')[0]}-${Math.floor(Math.random() * 1000).toString().padStart(3, '0')}`,
    name: params?.destinationPort
      ? `Route to ${params.destinationPort}`
      : 'Unnamed Route',
    origin,
    destination,
    waypoints,
    totalDistance: parseFloat(distance.toFixed(1)),
    estimatedFuel: parseFloat(estimatedFuel.toFixed(1)),
    estimatedDuration: parseFloat(estimatedDuration.toFixed(1)),
    optimization: params?.optimization,
    constraints: params?.constraints,
    created: new Date(),
  };
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Autonomous Ship Controller
 *
 * Main SDK class for controlling and monitoring autonomous ships
 *
 * @example
 * const ship = new AutonomousShipController({
 *   identification: {
 *     imo: '9876543',
 *     mmsi: '123456789',
 *     callSign: 'ABCD'
 *   },
 *   dimensions: {
 *     length: 300,
 *     beam: 48,
 *     draft: 15
 *   },
 *   maxAutonomyLevel: AutonomyLevel.REMOTE_WITHOUT_CREW
 * });
 *
 * await ship.startNavigation({
 *   destination: { latitude: 1.2897, longitude: 103.8501 }
 * });
 */
export class AutonomousShipController {
  private config: ShipConfig;
  private currentRoute?: Route;
  private autonomyLevel: AutonomyLevel = AutonomyLevel.MANUAL;

  constructor(config: ShipConfig) {
    this.config = config;
    this.autonomyLevel = AutonomyLevel.MANUAL;
  }

  /**
   * Start autonomous navigation to destination
   */
  async startNavigation(params: NavigationParams): Promise<void> {
    // Validate autonomy level
    if (this.autonomyLevel < AutonomyLevel.DECISION_SUPPORT) {
      throw new AutonomousShipError(
        AutoShipErrorCode.AUTONOMY_DEGRADED,
        'Cannot start autonomous navigation in manual mode'
      );
    }

    // Plan route
    const currentPosition = params.destination; // In production, get from GNSS
    this.currentRoute = await planRoute(currentPosition, params.destination, params);

    console.log(
      `Navigation started to ${params.destinationPort || 'destination'}`
    );
    console.log(`Route: ${this.currentRoute.waypoints.length} waypoints`);
    console.log(`Distance: ${this.currentRoute.totalDistance} NM`);
    console.log(`ETA: ${this.currentRoute.estimatedDuration.toFixed(1)} hours`);
  }

  /**
   * Stop autonomous navigation
   */
  async stopNavigation(): Promise<void> {
    this.currentRoute = undefined;
    console.log('Navigation stopped');
  }

  /**
   * Set ship course and speed
   */
  async setCourse(heading: number, speed: number): Promise<void> {
    if (heading < 0 || heading >= 360) {
      throw new AutonomousShipError(
        AutoShipErrorCode.NAVIGATION_FAILURE,
        'Invalid heading: must be 0-360 degrees'
      );
    }

    if (speed < 0 || speed > this.config.performance.maxSpeed) {
      throw new AutonomousShipError(
        AutoShipErrorCode.NAVIGATION_FAILURE,
        `Invalid speed: must be 0-${this.config.performance.maxSpeed} knots`
      );
    }

    console.log(`Course set to ${heading}° at ${speed} knots`);
  }

  /**
   * Get current route
   */
  getRoute(): Route | undefined {
    return this.currentRoute;
  }

  /**
   * Set autonomy level
   */
  async setAutonomyLevel(level: AutonomyLevel): Promise<void> {
    if (level > this.config.maxAutonomyLevel) {
      throw new AutonomousShipError(
        AutoShipErrorCode.AUTONOMY_DEGRADED,
        `Cannot set autonomy level ${level}: maximum authorized level is ${this.config.maxAutonomyLevel}`
      );
    }

    this.autonomyLevel = level;
    console.log(`Autonomy level set to ${level}`);
  }

  /**
   * Get current autonomy level
   */
  getAutonomyLevel(): AutonomyLevel {
    return this.autonomyLevel;
  }

  /**
   * Emergency stop
   */
  async emergencyStop(): Promise<void> {
    await this.stopNavigation();
    await this.setCourse(0, 0);
    console.log('EMERGENCY STOP EXECUTED');
  }
}

// ============================================================================
// Exports
// ============================================================================

export {
  calculateDistance,
  calculateBearing,
  calculateCrossTrackError,
  calculateCollisionRisk,
  generateAvoidanceManeuver,
  planRoute,
  AutonomousShipController,
};
