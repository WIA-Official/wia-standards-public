/**
 * WIA-QUA-015: Wormhole Navigation SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for wormhole physics and navigation including:
 * - Einstein-Rosen bridge analysis
 * - Morris-Thorne traversable wormhole calculations
 * - Exotic matter requirements
 * - Stability analysis
 * - Navigation trajectory planning
 * - Tidal force evaluation
 * - Safety assessment
 */

import {
  WormholeMetric,
  WormholeType,
  SpacetimeCoordinates,
  ProperCoordinates,
  MetricTensor,
  ShapeFunction,
  ExoticMatterRequirements,
  StabilityResult,
  TrajectoryParameters,
  NavigationTrajectory,
  TidalForces,
  TidalParameters,
  SafetyAssessment,
  RadiationAnalysis,
  CoordinateTransformation,
  WORMHOLE_CONSTANTS,
  SAFETY_LIMITS,
  DEFAULT_WORMHOLE,
  WormholeErrorCode,
  WormholeError,
  Velocity,
} from './types';

const { c, G, hbar, k_B, M_Sun, g_Earth } = WORMHOLE_CONSTANTS;

// ============================================================================
// Shape Functions
// ============================================================================

/**
 * Standard Morris-Thorne shape function
 */
export function morrisThornShapeFunction(throatRadius: number): ShapeFunction {
  return (l: number) => {
    if (l === 0) return throatRadius;
    return throatRadius * (1 + (throatRadius / l) ** 2) / 2;
  };
}

/**
 * Exponential shape function
 */
export function exponentialShapeFunction(throatRadius: number): ShapeFunction {
  return (l: number) => {
    return throatRadius * (1 - Math.exp(-Math.abs(l) / throatRadius));
  };
}

/**
 * Simple constant shape function (minimal wormhole)
 */
export function constantShapeFunction(throatRadius: number): ShapeFunction {
  return () => throatRadius;
}

// ============================================================================
// Metric Calculations
// ============================================================================

/**
 * Calculate metric tensor at given position
 */
export function calculateMetric(
  wormhole: WormholeMetric,
  coords: ProperCoordinates
): MetricTensor {
  const { l, theta } = coords;
  const b = wormhole.shapeFunction(l);
  const Phi = wormhole.redshiftFunction ? wormhole.redshiftFunction(l) : 0;

  // Circumferential radius
  const r = Math.sqrt(l * l + wormhole.throatRadius * wormhole.throatRadius);

  return {
    g_tt: -Math.exp(2 * Phi),
    g_rr: 1 / (1 - b / Math.abs(l || 1e-10)),
    g_thetatheta: r * r,
    g_phiphi: r * r * Math.sin(theta) ** 2,
  };
}

/**
 * Calculate proper distance through wormhole
 */
export function calculateProperDistance(
  wormhole: WormholeMetric,
  from: number,
  to: number
): number {
  const steps = 1000;
  const dl = (to - from) / steps;
  let distance = 0;

  for (let i = 0; i < steps; i++) {
    const l = from + i * dl;
    const b = wormhole.shapeFunction(l);
    const integrand = 1 / Math.sqrt(Math.abs(1 - b / (l || 1e-10)));
    distance += integrand * Math.abs(dl);
  }

  return distance;
}

/**
 * Calculate Schwarzschild radius
 */
export function schwarzschildRadius(mass: number): number {
  return (2 * G * mass) / (c * c);
}

// ============================================================================
// Exotic Matter
// ============================================================================

/**
 * Calculate exotic matter requirements
 */
export function calculateExoticMatter(
  wormhole: WormholeMetric
): ExoticMatterRequirements {
  const r0 = wormhole.throatRadius;

  // Energy density at throat (negative)
  // ρ = -(1 - b'(l₀))/(8πGl₀²)
  const dl = 1e-6;
  const bPrime =
    (wormhole.shapeFunction(r0 + dl) - wormhole.shapeFunction(r0 - dl)) / (2 * dl);

  const energyDensity = -(1 - bPrime) / (8 * Math.PI * G * r0 * r0);

  // Total exotic matter mass
  // M_exotic ≈ -c⁴r₀/(4G)
  const mass = (-c ** 4 * r0) / (4 * G);

  // Pressures
  const radialPressure = -energyDensity; // Approximation
  const tangentialPressure = -energyDensity / 2;

  // Null Energy Condition violation
  const nullEnergyViolation = energyDensity + radialPressure;

  // Distribution function
  const distribution = (l: number) => {
    const r = Math.sqrt(l * l + r0 * r0);
    return energyDensity * (r0 / r) ** 2;
  };

  return {
    mass,
    energyDensity,
    pressure: {
      radial: radialPressure,
      tangential: tangentialPressure,
    },
    nullEnergyViolation,
    distribution,
  };
}

/**
 * Check if exotic matter requirements are met
 */
export function checkExoticMatterRequirements(
  wormhole: WormholeMetric
): boolean {
  const exotic = calculateExoticMatter(wormhole);
  return (
    exotic.mass < 0 &&
    exotic.energyDensity < 0 &&
    exotic.nullEnergyViolation < 0 &&
    Math.abs(wormhole.exoticMatter) >= Math.abs(exotic.mass) * 0.9
  );
}

// ============================================================================
// Stability Analysis
// ============================================================================

/**
 * Check wormhole stability
 */
export function checkStability(wormhole: WormholeMetric): StabilityResult {
  const r0 = wormhole.throatRadius;
  const dl = 1e-6;

  // Calculate shape function derivative at throat
  const bPrime =
    (wormhole.shapeFunction(r0 + dl) - wormhole.shapeFunction(r0 - dl)) / (2 * dl);

  // Flare-out condition: b'(l₀) < 1
  const flareOutCondition = bPrime < 1;

  // Estimate throat change rate (simplified)
  const throatChangeRate = 0; // Assume static for now

  // Damping time for perturbations
  const dampingTime = (r0 / c) * Math.sqrt(1 / (1 - bPrime));

  // Overall stability
  const isStable =
    flareOutCondition &&
    Math.abs(throatChangeRate) < 0.01 &&
    checkExoticMatterRequirements(wormhole);

  // Stability margin
  const stabilityMargin = flareOutCondition ? 1 - bPrime : 0;

  return {
    isStable,
    throatRadius: r0,
    throatChangeRate,
    dampingTime,
    shapeFunctionDerivative: bPrime,
    flareOutCondition,
    stabilityMargin,
  };
}

/**
 * Estimate wormhole lifetime
 */
export function estimateLifetime(wormhole: WormholeMetric): number {
  // For traversable wormhole with sufficient exotic matter: indefinite
  if (checkStability(wormhole).isStable) {
    return Infinity;
  }

  // For unstable wormhole: collapse time
  const r0 = wormhole.throatRadius;
  return (Math.PI * r0) / c;
}

// ============================================================================
// Navigation
// ============================================================================

/**
 * Plan navigation trajectory through wormhole
 */
export function planTrajectory(
  params: TrajectoryParameters
): NavigationTrajectory {
  const { wormhole, entryPoint, velocity, mass } = params;
  const r0 = wormhole.throatRadius;

  // Check stability first
  const stability = checkStability(wormhole);
  if (!stability.isStable) {
    return {
      entry: entryPoint,
      exit: entryPoint,
      properTime: 0,
      coordinateTime: 0,
      worldline: [],
      velocityProfile: [],
      maxTidalAcceleration: Infinity,
      energyRequired: 0,
      success: false,
      warnings: ['Wormhole is unstable!'],
    };
  }

  // Calculate proper distance
  const distance = calculateProperDistance(wormhole, -10 * r0, 10 * r0);

  // Proper time = distance / velocity
  const properTime = distance / velocity;

  // Coordinate time (approximately same for slow velocities)
  const gamma = 1 / Math.sqrt(1 - (velocity / c) ** 2);
  const coordinateTime = properTime * gamma;

  // Generate worldline
  const steps = 100;
  const worldline: SpacetimeCoordinates[] = [];
  const velocityProfile: Velocity[] = [];

  for (let i = 0; i <= steps; i++) {
    const fraction = i / steps;
    const l = -10 * r0 + fraction * 20 * r0;
    const t = entryPoint.t + fraction * coordinateTime;

    worldline.push({
      t,
      r: Math.sqrt(l * l + r0 * r0),
      theta: entryPoint.theta,
      phi: entryPoint.phi,
    });

    velocityProfile.push({
      vr: velocity,
      vtheta: 0,
      vphi: 0,
    });
  }

  // Exit point
  const exit: SpacetimeCoordinates = {
    t: entryPoint.t + coordinateTime,
    r: Math.sqrt((10 * r0) ** 2 + r0 * r0),
    theta: entryPoint.theta,
    phi: entryPoint.phi,
  };

  // Calculate tidal forces
  const tidalParams: TidalParameters = {
    position: { t: 0, l: 0, theta: 0, phi: 0 },
    objectSize: 2, // 2 meters (human height)
    objectMass: mass,
    maxTolerance: SAFETY_LIMITS.MAX_ACCELERATION,
  };
  const tidal = calculateTidalForces(wormhole, tidalParams);

  // Energy required (kinetic energy)
  const energyRequired = 0.5 * mass * velocity ** 2;

  return {
    entry: entryPoint,
    exit,
    properTime,
    coordinateTime,
    worldline,
    velocityProfile,
    maxTidalAcceleration: tidal.maxAcceleration,
    energyRequired,
    success: true,
    warnings: tidal.humanSafe ? [] : ['Tidal forces may be unsafe for humans'],
  };
}

/**
 * Calculate optimal velocity for navigation
 */
export function calculateOptimalVelocity(wormhole: WormholeMetric): number {
  const r0 = wormhole.throatRadius;
  const a_max = SAFETY_LIMITS.MAX_ACCELERATION;

  // v_optimal = √(c × a_max × r₀)
  return Math.sqrt(c * a_max * r0);
}

// ============================================================================
// Tidal Forces
// ============================================================================

/**
 * Calculate tidal forces
 */
export function calculateTidalForces(
  wormhole: WormholeMetric,
  params: TidalParameters
): TidalForces {
  const { position, objectSize, objectMass } = params;
  const r0 = wormhole.throatRadius;
  const exotic = calculateExoticMatter(wormhole);

  // Tidal acceleration: Δa = c⁴h/(4|M_exotic|r₀²)
  const maxAcceleration =
    (c ** 4 * objectSize) / (4 * Math.abs(exotic.mass) * r0 * r0);

  // Gradient
  const gradient = maxAcceleration / objectSize;

  // Forces
  const radialForce = objectMass * maxAcceleration;
  const tangentialForce = radialForce / 2; // Approximation

  // Safety checks
  const humanSafe =
    maxAcceleration < SAFETY_LIMITS.MAX_ACCELERATION &&
    gradient < SAFETY_LIMITS.MAX_GRADIENT;

  const spacecraftSafe = maxAcceleration < SAFETY_LIMITS.MAX_ACCELERATION * 10;

  // Safety margin
  const safetyMargin = 1 - maxAcceleration / SAFETY_LIMITS.MAX_ACCELERATION;

  return {
    maxAcceleration,
    gradient,
    radialForce,
    tangentialForce,
    humanSafe,
    spacecraftSafe,
    safetyMargin: Math.max(0, safetyMargin),
  };
}

// ============================================================================
// Safety Assessment
// ============================================================================

/**
 * Perform comprehensive safety assessment
 */
export function assessSafety(
  wormhole: WormholeMetric,
  trajectory: NavigationTrajectory
): SafetyAssessment {
  const stability = checkStability(wormhole);
  const radiation = analyzeRadiation(wormhole);
  const tidalParams: TidalParameters = {
    position: { t: 0, l: 0, theta: 0, phi: 0 },
    objectSize: 2,
    objectMass: 1000,
    maxTolerance: SAFETY_LIMITS.MAX_ACCELERATION,
  };
  const tidal = calculateTidalForces(wormhole, tidalParams);

  // Individual checks
  const checks = {
    tidalForces: tidal.humanSafe,
    radiation: radiation.doseRate < SAFETY_LIMITS.MAX_RADIATION,
    stability: stability.isStable,
    structuralIntegrity: tidal.maxAcceleration < SAFETY_LIMITS.MAX_STRESS / 1e6,
    temperature: true, // Assume OK for now
  };

  // Overall safety
  const safe = Object.values(checks).every((check) => check);

  // Safety score (0-100)
  const safetyScore = (Object.values(checks).filter(Boolean).length / 5) * 100;

  // Recommendations
  const recommendations: string[] = [];
  if (!checks.tidalForces)
    recommendations.push('Increase throat radius to reduce tidal forces');
  if (!checks.radiation) recommendations.push('Add radiation shielding');
  if (!checks.stability) recommendations.push('Increase exotic matter to stabilize');

  // Warnings
  const warnings = trajectory.warnings;

  // Critical issues
  const criticalIssues: string[] = [];
  if (!checks.stability) criticalIssues.push('Wormhole unstable - do not traverse');
  if (tidal.maxAcceleration > SAFETY_LIMITS.MAX_ACCELERATION * 2)
    criticalIssues.push('Tidal forces lethal');

  return {
    safe,
    checks,
    safetyScore,
    recommendations,
    warnings,
    criticalIssues,
  };
}

/**
 * Analyze radiation hazards
 */
export function analyzeRadiation(wormhole: WormholeMetric): RadiationAnalysis {
  const M = Math.abs(wormhole.mass);

  // Hawking temperature: T = ℏc³/(8πGMk_B)
  const hawkingTemperature = (hbar * c ** 3) / (8 * Math.PI * G * M * k_B);

  // Hawking radiation power: P = ℏc⁶/(15360πG²M²)
  const radiationPower = (hbar * c ** 6) / (15360 * Math.PI * G ** 2 * M ** 2);

  // For wormhole with exotic matter, radiation is negligible
  const particleFlux = radiationPower / (hbar * c); // Rough estimate
  const doseRate = particleFlux * 1e-10; // Convert to mSv/h (very rough)

  const requiredShielding = doseRate > 0.1 ? 10 : 0; // cm lead

  const safeExposureTime = doseRate > 0 ? 1 / doseRate : Infinity;

  return {
    hawkingTemperature,
    radiationPower,
    particleFlux,
    doseRate,
    requiredShielding,
    safeExposureTime,
  };
}

// ============================================================================
// Coordinate Transformations
// ============================================================================

/**
 * Transform from standard to proper coordinates
 */
export function standardToProper(
  coords: SpacetimeCoordinates,
  wormhole: WormholeMetric
): ProperCoordinates {
  const { t, r, theta, phi } = coords;
  const r0 = wormhole.throatRadius;

  // l = ±√(r² - r₀²)
  const l = Math.sqrt(Math.max(0, r * r - r0 * r0));

  return { t, l, theta, phi };
}

/**
 * Transform from proper to standard coordinates
 */
export function properToStandard(
  coords: ProperCoordinates,
  wormhole: WormholeMetric
): SpacetimeCoordinates {
  const { t, l, theta, phi } = coords;
  const r0 = wormhole.throatRadius;

  // r = √(l² + r₀²)
  const r = Math.sqrt(l * l + r0 * r0);

  return { t, r, theta, phi };
}

/**
 * Transform coordinates through wormhole
 */
export function transformCoordinates(
  from: SpacetimeCoordinates | ProperCoordinates,
  wormhole: WormholeMetric,
  transformationType: 'standard-to-proper' | 'proper-to-standard' | 'entry-to-exit'
): CoordinateTransformation {
  let to: SpacetimeCoordinates | ProperCoordinates;

  if (transformationType === 'standard-to-proper') {
    to = standardToProper(from as SpacetimeCoordinates, wormhole);
  } else if (transformationType === 'proper-to-standard') {
    to = properToStandard(from as ProperCoordinates, wormhole);
  } else {
    // entry-to-exit: flip l coordinate
    const properCoords = from as ProperCoordinates;
    to = { ...properCoords, l: -properCoords.l };
  }

  return {
    from,
    to,
    transformationType,
  };
}

// ============================================================================
// Wormhole Creation Helpers
// ============================================================================

/**
 * Create Morris-Thorne wormhole
 */
export function createMorrisThorne(throatRadius: number): WormholeMetric {
  const exotic = calculateExoticMatter({
    type: 'MorrisThorne',
    throatRadius,
    shapeFunction: morrisThornShapeFunction(throatRadius),
    mass: DEFAULT_WORMHOLE.EXOTIC_MATTER_RATIO * throatRadius,
    exoticMatter: DEFAULT_WORMHOLE.EXOTIC_MATTER_RATIO * throatRadius,
  });

  return {
    type: 'MorrisThorne',
    throatRadius,
    shapeFunction: morrisThornShapeFunction(throatRadius),
    mass: Math.abs(exotic.mass),
    exoticMatter: exotic.mass,
    label: `Morris-Thorne Wormhole (r₀=${throatRadius}m)`,
  };
}

/**
 * Create Einstein-Rosen bridge
 */
export function createEinsteinRosen(mass: number): WormholeMetric {
  const rs = schwarzschildRadius(mass);

  return {
    type: 'EinsteinRosen',
    throatRadius: rs,
    shapeFunction: constantShapeFunction(rs),
    mass,
    exoticMatter: 0, // No exotic matter (non-traversable)
    label: `Einstein-Rosen Bridge (M=${mass}kg)`,
  };
}

// ============================================================================
// Validation
// ============================================================================

/**
 * Validate wormhole parameters
 */
export function validateWormhole(wormhole: WormholeMetric): boolean {
  if (wormhole.throatRadius <= 0) {
    throw new WormholeError(
      WormholeErrorCode.INVALID_THROAT_RADIUS,
      'Throat radius must be positive',
      { throatRadius: wormhole.throatRadius }
    );
  }

  if (wormhole.type === 'MorrisThorne' && wormhole.exoticMatter >= 0) {
    throw new WormholeError(
      WormholeErrorCode.INSUFFICIENT_EXOTIC_MATTER,
      'Morris-Thorne wormhole requires negative exotic matter',
      { exoticMatter: wormhole.exoticMatter }
    );
  }

  const stability = checkStability(wormhole);
  if (!stability.isStable && wormhole.type === 'MorrisThorne') {
    throw new WormholeError(
      WormholeErrorCode.UNSTABLE_WORMHOLE,
      'Wormhole is not stable',
      { stability }
    );
  }

  return true;
}

// ============================================================================
// Export All
// ============================================================================

export {
  // Shape functions
  morrisThornShapeFunction,
  exponentialShapeFunction,
  constantShapeFunction,

  // Metric
  calculateMetric,
  calculateProperDistance,
  schwarzschildRadius,

  // Exotic matter
  calculateExoticMatter,
  checkExoticMatterRequirements,

  // Stability
  checkStability,
  estimateLifetime,

  // Navigation
  planTrajectory,
  calculateOptimalVelocity,

  // Tidal forces
  calculateTidalForces,

  // Safety
  assessSafety,
  analyzeRadiation,

  // Transformations
  standardToProper,
  properToStandard,
  transformCoordinates,

  // Creation
  createMorrisThorne,
  createEinsteinRosen,

  // Validation
  validateWormhole,
};

// Re-export types
export * from './types';
