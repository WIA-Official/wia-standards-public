/**
 * WIA-TIME-003: Quantum Time Theory - SDK Implementation
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

import {
  QuantumTimeState,
  TemporalSuperposition,
  QuantumTimeStateConfig,
  QuantumEntanglement,
  CollapsedTimeline,
  DecoherenceRate,
  EnvironmentConfig,
  TunnelingResult,
  TemporalTunnelConfig,
  Observer,
  Complex,
  TimelineBranch,
  BranchingEvent,
  SchrodingersTimeline,
  TimelineState,
  WheelerDeWittState,
  Metric3D,
  PhysicalConstants,
  QuantumTimeError,
  ERROR_CODES,
  ValidationResult
} from './types';

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================

export const PHYSICAL_CONSTANTS: PhysicalConstants = {
  HBAR: 1.054571817e-34,           // J⋅s (reduced Planck constant)
  C: 299792458,                     // m/s (speed of light)
  BOLTZMANN: 1.380649e-23,          // J/K (Boltzmann constant)
  ELECTRON_MASS: 9.1093837015e-31,  // kg
  PROTON_MASS: 1.67262192369e-27,   // kg
  ELEMENTARY_CHARGE: 1.602176634e-19, // C
  EV_TO_JOULES: 1.602176634e-19,    // J (eV conversion)
  PLANCK_TIME: 5.391247e-44,        // s
  PLANCK_LENGTH: 1.616255e-35       // m
};

// Particle masses for convenience
const PARTICLE_MASSES: Record<string, number> = {
  electron: PHYSICAL_CONSTANTS.ELECTRON_MASS,
  proton: PHYSICAL_CONSTANTS.PROTON_MASS,
  neutron: 1.674927498e-27,  // kg
  photon: 0                   // kg (massless)
};

// Constants
const EPSILON = 1e-15;
const DECOHERENCE_THRESHOLD = 0.01;
const BASE_DECOHERENCE_RATE = 1e6; // Hz

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Generate unique ID
 */
function generateId(prefix: string = 'qt'): string {
  return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

/**
 * Complex number multiplication
 */
export function complexMultiply(a: Complex, b: Complex): Complex {
  return {
    real: a.real * b.real - a.imaginary * b.imaginary,
    imaginary: a.real * b.imaginary + a.imaginary * b.real
  };
}

/**
 * Complex number magnitude squared
 */
export function complexMagnitudeSquared(c: Complex): number {
  return c.real * c.real + c.imaginary * c.imaginary;
}

/**
 * Complex number magnitude
 */
export function complexMagnitude(c: Complex): number {
  return Math.sqrt(complexMagnitudeSquared(c));
}

/**
 * Complex conjugate
 */
export function complexConjugate(c: Complex): Complex {
  return {
    real: c.real,
    imaginary: -c.imaginary
  };
}

/**
 * Weighted random selection based on probabilities
 */
function weightedRandomSelect<T extends { probability: number }>(items: T[]): T {
  const rand = Math.random();
  let cumulative = 0;

  for (const item of items) {
    cumulative += item.probability;
    if (rand <= cumulative) {
      return item;
    }
  }

  return items[items.length - 1];
}

/**
 * Gaussian random number (Box-Muller transform)
 */
function gaussianRandom(mean: number = 0, stdDev: number = 1): number {
  const u1 = Math.random();
  const u2 = Math.random();
  const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return z0 * stdDev + mean;
}

// ============================================================================
// QUANTUM TIME STATE CREATION
// ============================================================================

/**
 * Create a quantum superposition of time states
 *
 * @param config - Configuration for quantum time state
 * @returns Quantum time state in superposition
 *
 * @example
 * ```typescript
 * const timeState = createQuantumTimeState({
 *   baseTimestamp: Date.now(),
 *   superpositionCount: 100,
 *   coherenceTime: 1000
 * });
 * ```
 */
export function createQuantumTimeState(
  config: QuantumTimeStateConfig
): QuantumTimeState {
  const {
    baseTimestamp,
    superpositionCount,
    coherenceTime,
    distribution = 'uniform',
    initialPhase = 0,
    timeRange
  } = config;

  // Validate input
  if (superpositionCount <= 0) {
    throw new QuantumTimeError(
      'Superposition count must be positive',
      ERROR_CODES.INVALID_STATE
    );
  }

  if (coherenceTime <= 0) {
    throw new QuantumTimeError(
      'Coherence time must be positive',
      ERROR_CODES.INVALID_STATE
    );
  }

  // Generate timelines
  const timelines: TemporalSuperposition[] = [];
  const startTime = timeRange?.start ?? baseTimestamp;
  const endTime = timeRange?.end ?? baseTimestamp + coherenceTime;

  for (let i = 0; i < superpositionCount; i++) {
    let timestamp: number;
    let rawProbability: number;

    // Generate timestamp based on distribution
    switch (distribution) {
      case 'gaussian':
        timestamp = gaussianRandom(baseTimestamp, coherenceTime / 6);
        rawProbability = 1 / Math.sqrt(2 * Math.PI) *
          Math.exp(-Math.pow((timestamp - baseTimestamp) / (coherenceTime / 6), 2) / 2);
        break;

      case 'exponential':
        const lambda = 1 / coherenceTime;
        const u = Math.random();
        timestamp = baseTimestamp - Math.log(1 - u) / lambda;
        rawProbability = lambda * Math.exp(-lambda * (timestamp - baseTimestamp));
        break;

      case 'uniform':
      default:
        timestamp = startTime + (endTime - startTime) * i / superpositionCount;
        rawProbability = 1;
        break;
    }

    // Generate complex amplitude
    const phase = initialPhase + (i * 2 * Math.PI / superpositionCount);
    const amplitude: Complex = {
      real: Math.sqrt(rawProbability) * Math.cos(phase),
      imaginary: Math.sqrt(rawProbability) * Math.sin(phase)
    };

    timelines.push({
      timelineId: generateId('timeline'),
      timestamp,
      amplitude,
      probability: complexMagnitudeSquared(amplitude),
      relativePhase: phase
    });
  }

  // Normalize
  const totalProbability = timelines.reduce((sum, t) => sum + t.probability, 0);
  const normFactor = Math.sqrt(totalProbability);

  const normalizedTimelines = timelines.map(t => ({
    ...t,
    amplitude: {
      real: t.amplitude.real / normFactor,
      imaginary: t.amplitude.imaginary / normFactor
    },
    probability: t.probability / totalProbability
  }));

  return {
    stateId: generateId('qstate'),
    baseTimestamp,
    timelines: normalizedTimelines,
    coherenceTime,
    createdAt: Date.now(),
    normalization: 1.0,
    globalPhase: initialPhase
  };
}

/**
 * Validate quantum time state normalization
 */
export function validateNormalization(state: QuantumTimeState): boolean {
  const total = state.timelines.reduce((sum, t) => sum + t.probability, 0);
  return Math.abs(total - 1.0) < EPSILON;
}

// ============================================================================
// QUANTUM ENTANGLEMENT ACROSS TIME
// ============================================================================

/**
 * Entangle two quantum time states across temporal separation
 *
 * @param state1 - First quantum time state
 * @param state2 - Second quantum time state or timestamp
 * @returns Quantum entanglement object
 *
 * @example
 * ```typescript
 * const entanglement = entangleAcrossTime(
 *   state1,
 *   { timestamp: Date.now() + 5000 }
 * );
 * ```
 */
export function entangleAcrossTime(
  state1: QuantumTimeState,
  state2: QuantumTimeState | { timestamp: number }
): QuantumEntanglement {
  // Determine time separation
  const t1 = state1.baseTimestamp;
  const t2 = 'baseTimestamp' in state2 ? state2.baseTimestamp : state2.timestamp;
  const timeSeparation = Math.abs(t2 - t1);

  // Select Bell state (randomize for now)
  const bellStates: Array<'phi_plus' | 'phi_minus' | 'psi_plus' | 'psi_minus'> = [
    'phi_plus', 'phi_minus', 'psi_plus', 'psi_minus'
  ];
  const bellState = bellStates[Math.floor(Math.random() * bellStates.length)];

  // Calculate entanglement strength (decays with time separation)
  const tau_entangle = state1.coherenceTime;
  const strength = Math.exp(-timeSeparation / tau_entangle);

  // Correlation function based on Bell state
  const correlationFunction = (time1: number, time2: number): number => {
    const dt = Math.abs(time2 - time1);

    switch (bellState) {
      case 'phi_plus':
        return Math.cos(dt / tau_entangle);
      case 'phi_minus':
        return -Math.cos(dt / tau_entangle);
      case 'psi_plus':
        return Math.sin(dt / tau_entangle);
      case 'psi_minus':
        return -Math.sin(dt / tau_entangle);
    }
  };

  return {
    entanglementId: generateId('entangle'),
    state1,
    state2,
    entanglementStrength: strength,
    bellState,
    timeSeparation,
    correlationFunction,
    createdAt: Date.now()
  };
}

// ============================================================================
// WAVEFUNCTION COLLAPSE
// ============================================================================

/**
 * Collapse quantum time state wavefunction to single timeline
 *
 * @param state - Quantum time state to collapse
 * @param observer - Observer performing measurement
 * @returns Collapsed timeline
 *
 * @example
 * ```typescript
 * const collapsed = collapseTimelineWavefunction(timeState, {
 *   position: { x: 0, y: 0, z: 0, t: 0 },
 *   referenceFrame: {
 *     velocity: { vx: 0, vy: 0, vz: 0 },
 *     properTime: 0,
 *     gamma: 1
 *   }
 * });
 * ```
 */
export function collapseTimelineWavefunction(
  state: QuantumTimeState,
  observer: Observer
): CollapsedTimeline {
  // Validate state
  if (!validateNormalization(state)) {
    throw new QuantumTimeError(
      'Cannot collapse unnormalized state',
      ERROR_CODES.COLLAPSE_ERROR
    );
  }

  // Account for relativistic effects (simplified)
  const gamma = observer.referenceFrame.gamma;
  const properTime = observer.position.t / gamma;

  // Select outcome based on Born rule (probabilistic)
  const selectedTimeline = weightedRandomSelect(state.timelines);

  // Get other branches
  const otherBranches = state.timelines.filter(
    t => t.timelineId !== selectedTimeline.timelineId
  );

  return {
    selectedTimeline,
    collapseTime: observer.position.t,
    observerFrame: observer.referenceFrame,
    otherBranches,
    collapseMechanism: 'measurement'
  };
}

// ============================================================================
// QUANTUM DECOHERENCE
// ============================================================================

/**
 * Calculate decoherence rate for quantum time state
 *
 * @param state - Quantum time state
 * @param environment - Environmental parameters
 * @returns Decoherence rate information
 *
 * @example
 * ```typescript
 * const rate = calculateDecoherenceRate(timeState, {
 *   temperature: 300,
 *   environmentalNoise: 0.1
 * });
 * ```
 */
export function calculateDecoherenceRate(
  state: QuantumTimeState,
  environment: EnvironmentConfig
): DecoherenceRate {
  const { temperature, environmentalNoise, gravitationalField = 9.81 } = environment;

  // Thermal decoherence: τ_thermal = ℏ / (kT)
  const tau_thermal = PHYSICAL_CONSTANTS.HBAR /
    (PHYSICAL_CONSTANTS.BOLTZMANN * temperature);
  const gamma_thermal = 1 / tau_thermal;

  // Environmental decoherence (phenomenological)
  const gamma_env = environmentalNoise * BASE_DECOHERENCE_RATE;

  // Gravitational decoherence (Penrose model, simplified)
  // Estimate energy difference from time spread
  const timeSpread = Math.max(
    ...state.timelines.map(t => Math.abs(t.timestamp - state.baseTimestamp))
  );
  const energyDifference = PHYSICAL_CONSTANTS.HBAR / (timeSpread / 1000); // Convert ms to s
  const tau_grav = PHYSICAL_CONSTANTS.HBAR / energyDifference;
  const gamma_grav = gravitationalField / (PHYSICAL_CONSTANTS.C ** 2) / tau_grav;

  // Informational decoherence (von Neumann entropy)
  const entropy = -state.timelines.reduce((sum, t) => {
    return sum + (t.probability > 0 ? t.probability * Math.log(t.probability) : 0);
  }, 0);
  const gamma_info = entropy * 1e3; // Hz

  // Total decoherence rate
  const gamma_total = gamma_thermal + gamma_env + gamma_grav + gamma_info;
  const tau_D = 1 / gamma_total;

  // Coherence function: C(t) = exp(-t / τ_D)
  const coherenceFunction = (t: number): number => {
    return Math.exp(-t / tau_D);
  };

  return {
    rate: gamma_total,
    decoherenceTime: tau_D,
    factors: {
      thermal: gamma_thermal,
      environmental: gamma_env,
      gravitational: gamma_grav,
      informational: gamma_info
    },
    coherenceFunction,
    decoherenceThreshold: DECOHERENCE_THRESHOLD
  };
}

/**
 * Evolve quantum state with decoherence
 */
export function evolveWithDecoherence(
  state: QuantumTimeState,
  timeElapsed: number,
  decoherenceRate: DecoherenceRate
): QuantumTimeState | CollapsedTimeline {
  const coherence = decoherenceRate.coherenceFunction(timeElapsed);

  if (coherence < decoherenceRate.decoherenceThreshold) {
    // Collapse to single timeline
    return collapseTimelineWavefunction(state, {
      position: { x: 0, y: 0, z: 0, t: timeElapsed },
      referenceFrame: {
        velocity: { vx: 0, vy: 0, vz: 0 },
        properTime: timeElapsed,
        gamma: 1
      }
    });
  }

  // Reduce coherence but maintain superposition
  return {
    ...state,
    timelines: state.timelines.map(t => ({
      ...t,
      probability: t.probability * coherence
    })),
    coherenceTime: state.coherenceTime * coherence
  };
}

// ============================================================================
// TEMPORAL QUANTUM TUNNELING
// ============================================================================

/**
 * Calculate quantum tunneling probability through temporal barrier
 *
 * @param config - Tunneling configuration
 * @returns Tunneling result with probability
 *
 * @example
 * ```typescript
 * const result = tunnelThroughTime({
 *   particle: 'electron',
 *   barrierHeight: 1.5,
 *   barrierWidth: 1e-9,
 *   particleEnergy: 1.0
 * });
 * ```
 */
export function tunnelThroughTime(
  config: TemporalTunnelConfig
): TunnelingResult {
  const {
    particle,
    mass: customMass,
    barrierHeight,
    barrierWidth,
    particleEnergy,
    temperature = 300,
    barrierShape = 'rectangular'
  } = config;

  // Get particle mass
  const m = customMass ?? PARTICLE_MASSES[particle];
  if (!m && particle === 'custom') {
    throw new QuantumTimeError(
      'Custom particle requires mass parameter',
      ERROR_CODES.TUNNELING_IMPOSSIBLE
    );
  }

  // Convert energies to Joules
  const E = particleEnergy * PHYSICAL_CONSTANTS.EV_TO_JOULES;
  const V0 = barrierHeight * PHYSICAL_CONSTANTS.EV_TO_JOULES;
  const a = barrierWidth;

  // Check if tunneling is needed
  if (E >= V0) {
    // Classical transmission
    return {
      transmissionProbability: 1.0,
      reflectionProbability: 0.0,
      tunnelingTime: 0,
      wavefunctionDecay: 1.0,
      success: true,
      details: {
        wkbApproximation: false,
        energyRatio: E / V0,
        penetrationDepth: Infinity
      }
    };
  }

  // Calculate wave vector inside barrier
  // k = sqrt(2m(V0 - E)) / ℏ
  const k = Math.sqrt(2 * m * (V0 - E)) / PHYSICAL_CONSTANTS.HBAR;

  // Calculate transmission coefficient based on barrier shape
  let T: number;
  let penetrationDepth: number;

  switch (barrierShape) {
    case 'rectangular':
      // WKB approximation: T ≈ exp(-2ka)
      T = Math.exp(-2 * k * a);
      penetrationDepth = 1 / k;
      break;

    case 'triangular':
      // Linear barrier: V(x) = V0(1 - x/a)
      const integral = (4 / 3) * Math.sqrt(2 * m * V0) * a / PHYSICAL_CONSTANTS.HBAR;
      T = Math.exp(-integral);
      penetrationDepth = a / 2;
      break;

    case 'parabolic':
      // Parabolic barrier: V(x) = V0(1 - (x/a)²)
      const integralParabolic = Math.PI * Math.sqrt(2 * m * V0) * a / (2 * PHYSICAL_CONSTANTS.HBAR);
      T = Math.exp(-integralParabolic);
      penetrationDepth = a / Math.sqrt(2);
      break;

    case 'delta':
      // Delta function barrier
      const strength = V0 * a;
      T = 1 / (1 + (m * strength ** 2) / (2 * PHYSICAL_CONSTANTS.HBAR ** 2 * E));
      penetrationDepth = PHYSICAL_CONSTANTS.HBAR / Math.sqrt(2 * m * E);
      break;

    default:
      T = Math.exp(-2 * k * a);
      penetrationDepth = 1 / k;
  }

  // Calculate tunneling time (Büttiker-Landauer time)
  const tau = (m * a) / (PHYSICAL_CONSTANTS.HBAR * k);
  const tunnelingTime = tau * 1e15; // Convert to femtoseconds

  // Wavefunction decay
  const wavefunctionDecay = Math.exp(-k * a);

  // Determine if tunneling succeeds (probabilistic)
  const success = Math.random() < T;

  return {
    transmissionProbability: T,
    reflectionProbability: 1 - T,
    tunnelingTime,
    wavefunctionDecay,
    success,
    barrierWaveVector: k,
    details: {
      wkbApproximation: true,
      energyRatio: E / V0,
      penetrationDepth
    }
  };
}

// ============================================================================
// MANY-WORLDS TIMELINE BRANCHING
// ============================================================================

/**
 * Branch timeline based on quantum event
 */
export function branchTimeline(
  state: QuantumTimeState,
  event: BranchingEvent
): QuantumTimeState {
  const newTimelines: TemporalSuperposition[] = [];

  for (const timeline of state.timelines) {
    // Each existing timeline branches into n outcomes
    for (let i = 0; i < event.outcomeCount; i++) {
      const branchFactor = 1 / Math.sqrt(event.outcomeCount);
      const branchAmplitude: Complex = {
        real: timeline.amplitude.real * branchFactor,
        imaginary: timeline.amplitude.imaginary * branchFactor
      };

      newTimelines.push({
        timelineId: `${timeline.timelineId}-branch-${i}`,
        timestamp: event.timestamp,
        amplitude: branchAmplitude,
        probability: complexMagnitudeSquared(branchAmplitude),
        relativePhase: timeline.relativePhase + (i * 2 * Math.PI / event.outcomeCount),
        divergencePoint: event.timestamp,
        metadata: {
          parentTimelineId: timeline.timelineId,
          branchOutcome: i,
          branchEvent: event.description
        }
      });
    }
  }

  return {
    ...state,
    timelines: newTimelines,
    normalization: 1.0
  };
}

// ============================================================================
// SCHRÖDINGER'S TIMELINE
// ============================================================================

/**
 * Create Schrödinger's timeline (superposition of two states)
 */
export function createSchrodingersTimeline(
  state1: TimelineState,
  state2: TimelineState,
  paradox: string
): SchrodingersTimeline {
  // Create equal superposition (1/√2)(|state1⟩ + |state2⟩)
  const amplitude1: Complex = {
    real: 1 / Math.sqrt(2),
    imaginary: 0
  };

  const amplitude2: Complex = {
    real: 0,
    imaginary: 1 / Math.sqrt(2)
  };

  return {
    state1,
    state2,
    amplitudes: {
      state1: amplitude1,
      state2: amplitude2
    },
    observed: false,
    paradox,
    createdAt: Date.now()
  };
}

/**
 * Observe Schrödinger's timeline (collapse superposition)
 */
export function observeSchrodingersTimeline(
  timeline: SchrodingersTimeline
): SchrodingersTimeline {
  if (timeline.observed) {
    return timeline; // Already collapsed
  }

  // Born rule: measure with probability |α|²
  const prob1 = complexMagnitudeSquared(timeline.amplitudes.state1);
  const rand = Math.random();
  const collapsedState = rand < prob1 ? timeline.state1 : timeline.state2;

  return {
    ...timeline,
    observed: true,
    collapsedState,
    amplitudes: {
      state1: collapsedState === timeline.state1
        ? { real: 1, imaginary: 0 }
        : { real: 0, imaginary: 0 },
      state2: collapsedState === timeline.state2
        ? { real: 1, imaginary: 0 }
        : { real: 0, imaginary: 0 }
    }
  };
}

// ============================================================================
// WHEELER-DEWITT FUNCTIONS
// ============================================================================

/**
 * Create Wheeler-DeWitt state (simplified)
 */
export function createWheelerDeWittState(
  metric: Metric3D,
  cosmologicalConstant: number = 0
): WheelerDeWittState {
  // Calculate Ricci scalar (simplified for flat space)
  const ricciScalar = 0; // Would require full GR calculation

  // Wavefunction in superspace (placeholder)
  const wavefunctionInSuperspace = (m: Metric3D): Complex => {
    // Simplified wavefunction
    const determinant = m.g11 * (m.g22 * m.g33 - m.g23 * m.g32) -
                        m.g12 * (m.g21 * m.g33 - m.g23 * m.g31) +
                        m.g13 * (m.g21 * m.g32 - m.g22 * m.g31);

    return {
      real: Math.cos(determinant),
      imaginary: Math.sin(determinant)
    };
  };

  // Energy constraint (should be 0 for Wheeler-DeWitt)
  const energyConstraint = 0;

  return {
    wavefunctionInSuperspace,
    spatialMetric: metric,
    ricciScalar,
    cosmologicalConstant,
    energyConstraint
  };
}

// ============================================================================
// VALIDATION
// ============================================================================

/**
 * Validate quantum time state
 */
export function validateQuantumState(state: QuantumTimeState): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  // Check normalization
  const totalProb = state.timelines.reduce((sum, t) => sum + t.probability, 0);
  if (Math.abs(totalProb - 1.0) > EPSILON) {
    errors.push(`Normalization failed: total probability = ${totalProb}`);
  }

  // Check probabilities
  for (const timeline of state.timelines) {
    if (timeline.probability < 0 || timeline.probability > 1) {
      errors.push(`Invalid probability for timeline ${timeline.timelineId}: ${timeline.probability}`);
    }

    const calcProb = complexMagnitudeSquared(timeline.amplitude);
    if (Math.abs(calcProb - timeline.probability) > EPSILON) {
      warnings.push(`Probability mismatch for timeline ${timeline.timelineId}`);
    }
  }

  // Check coherence time
  if (state.coherenceTime <= 0) {
    errors.push('Coherence time must be positive');
  }

  return {
    valid: errors.length === 0,
    errors,
    warnings
  };
}

// ============================================================================
// EXPORTS
// ============================================================================

export {
  // Types
  type QuantumTimeState,
  type TemporalSuperposition,
  type QuantumTimeStateConfig,
  type QuantumEntanglement,
  type CollapsedTimeline,
  type DecoherenceRate,
  type EnvironmentConfig,
  type TunnelingResult,
  type TemporalTunnelConfig,
  type Observer,
  type Complex,
  type SchrodingersTimeline,
  type TimelineState,
  type WheelerDeWittState,
  type Metric3D,
  QuantumTimeError,
  ERROR_CODES
};

// Default export
export default {
  createQuantumTimeState,
  entangleAcrossTime,
  collapseTimelineWavefunction,
  calculateDecoherenceRate,
  tunnelThroughTime,
  branchTimeline,
  createSchrodingersTimeline,
  observeSchrodingersTimeline,
  createWheelerDeWittState,
  validateQuantumState,
  validateNormalization,
  evolveWithDecoherence,
  PHYSICAL_CONSTANTS,
  complexMultiply,
  complexMagnitude,
  complexMagnitudeSquared,
  complexConjugate
};
