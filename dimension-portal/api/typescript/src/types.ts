/**
 * WIA-QUA-018: Dimension Portal - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Fundamental physics constants
 */
export const PHYSICS_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,

  /** Gravitational constant (m³/(kg·s²)) */
  GRAVITATIONAL: 6.67430e-11,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Planck length (m) */
  PLANCK_LENGTH: 1.616255e-35,

  /** Planck time (s) */
  PLANCK_TIME: 5.391247e-44,

  /** Planck energy (J) */
  PLANCK_ENERGY: 1.9561e9,

  /** Planck mass (kg) */
  PLANCK_MASS: 2.176434e-8,

  /** String coupling constant (typical) */
  STRING_COUPLING: 0.5,
} as const;

/**
 * Dimensional physics parameters
 */
export const DIMENSIONAL_CONSTANTS = {
  /** Superstring theory dimensions */
  SUPERSTRING_DIMENSIONS: 10,

  /** M-theory dimensions */
  M_THEORY_DIMENSIONS: 11,

  /** Observable spacetime dimensions */
  OBSERVABLE_DIMENSIONS: 4,

  /** Typical compactification radius (m) */
  COMPACTIFICATION_RADIUS: 1e-35,

  /** Portal threshold energy (eV) */
  PORTAL_THRESHOLD_ENERGY: 1e15,

  /** Minimum stability index */
  MIN_STABILITY_INDEX: 0.95,

  /** Maximum energy fluctuation */
  MAX_ENERGY_FLUCTUATION: 0.01,
} as const;

// ============================================================================
// Dimensional Coordinates
// ============================================================================

/**
 * 4D spacetime coordinates [ct, x, y, z]
 */
export type SpacetimeCoordinates = [number, number, number, number];

/**
 * Extra dimensional coordinates (6 or 7 dimensions)
 */
export type ExtraDimensionalCoordinates = number[];

/**
 * Full dimensional coordinates
 */
export interface DimensionalCoordinates {
  /** 4D spacetime position */
  spacetime: SpacetimeCoordinates;

  /** Extra dimensional position */
  extra: ExtraDimensionalCoordinates;

  /** Total number of dimensions */
  totalDimensions: number;
}

/**
 * Coordinate transformation matrix
 */
export interface CoordinateTransform {
  /** 4D Lorentz transformation */
  lorentzMatrix: number[][];

  /** Extra dimensional rotation */
  extraDimRotation: number[][];

  /** Spacetime offset */
  spacetimeOffset: SpacetimeCoordinates;

  /** Extra dimensional shift */
  extraDimShift: ExtraDimensionalCoordinates;
}

// ============================================================================
// Portal Configuration
// ============================================================================

/**
 * Portal aperture size categories
 */
export type PortalSize =
  | 'microscopic' // < 1 μm
  | 'mesoscopic' // 1 μm - 1 mm
  | 'macroscopic' // 1 mm - 10 cm
  | 'human-scale' // 10 cm - 10 m
  | 'large-scale'; // > 10 m

/**
 * Portal status
 */
export type PortalStatus =
  | 'initializing'
  | 'forming'
  | 'stabilizing'
  | 'active'
  | 'degrading'
  | 'collapsing'
  | 'closed'
  | 'error';

/**
 * Portal aperture shape
 */
export type ApertureShape = 'circular' | 'elliptical' | 'arbitrary';

/**
 * Portal configuration
 */
export interface PortalConfiguration {
  /** Number of dimensions (10 or 11) */
  dimensions: 10 | 11;

  /** Portal aperture diameter (meters) */
  apertureDiameter: number;

  /** Aperture shape */
  apertureShape?: ApertureShape;

  /** Compactification radius (meters) */
  compactificationRadius: number;

  /** Stabilization field strength (eV) */
  stabilizationField: number;

  /** Portal coordinates */
  coordinates: DimensionalCoordinates;

  /** Maximum portal lifetime (seconds) */
  maxLifetime?: number;

  /** Energy budget (joules) */
  energyBudget?: number;

  /** Safety margin (0-1) */
  safetyMargin?: number;
}

/**
 * Portal parameters
 */
export interface PortalParameters {
  /** Portal unique identifier */
  id: string;

  /** Configuration */
  config: PortalConfiguration;

  /** Current status */
  status: PortalStatus;

  /** Stability index (0-1) */
  stabilityIndex: number;

  /** Current energy (joules) */
  currentEnergy: number;

  /** Current aperture radius (meters) */
  currentAperture: number;

  /** Time since creation (seconds) */
  lifetime: number;

  /** Temperature (Kelvin) */
  temperature: number;

  /** Creation timestamp */
  createdAt: Date;

  /** Last update timestamp */
  updatedAt: Date;
}

// ============================================================================
// Kaluza-Klein Theory
// ============================================================================

/**
 * Kaluza-Klein mode
 */
export interface KaluzaKleinMode {
  /** KK level (n) */
  level: number;

  /** KK mass (GeV) */
  mass: number;

  /** Momentum (1/R units) */
  momentum: number;

  /** Excitation energy (eV) */
  energy: number;
}

/**
 * Compactification geometry
 */
export type CompactificationGeometry =
  | 'torus'
  | 'calabi-yau'
  | 'orbifold'
  | 'k3-surface'
  | 'custom';

/**
 * Compactification specification
 */
export interface CompactificationSpec {
  /** Geometry type */
  geometry: CompactificationGeometry;

  /** Number of compactified dimensions */
  numDimensions: number;

  /** Compactification radius (meters) */
  radius: number;

  /** Topology description */
  topology?: string;

  /** Euler characteristic */
  eulerCharacteristic?: number;

  /** Hodge numbers */
  hodgeNumbers?: { p: number; q: number }[];

  /** Moduli space dimension */
  moduliDimension?: number;
}

// ============================================================================
// Portal Mechanics
// ============================================================================

/**
 * Portal formation parameters
 */
export interface PortalFormationParams {
  /** Target dimensions */
  targetDimensions: number;

  /** Aperture diameter (meters) */
  apertureDiameter: number;

  /** Formation time (seconds) */
  formationTime: number;

  /** Energy input (joules) */
  energyInput: number;

  /** Field strength (eV) */
  fieldStrength: number;
}

/**
 * Portal formation result
 */
export interface PortalFormationResult {
  /** Success status */
  success: boolean;

  /** Formed portal parameters */
  portal?: PortalParameters;

  /** Energy consumed (joules) */
  energyConsumed: number;

  /** Formation duration (seconds) */
  duration: number;

  /** Final stability index */
  stabilityIndex: number;

  /** Error message if failed */
  error?: string;
}

/**
 * Stabilization configuration
 */
export interface StabilizationConfig {
  /** Target stability index (0-1) */
  targetStability: number;

  /** Stabilization duration (seconds) */
  duration: number;

  /** Maximum power (watts) */
  maxPower?: number;

  /** Field strength (eV) */
  fieldStrength: number;

  /** Coherence time requirement (seconds) */
  coherenceTime?: number;
}

/**
 * Stabilization result
 */
export interface StabilizationResult {
  /** Success status */
  success: boolean;

  /** Achieved stability index */
  stabilityIndex: number;

  /** Energy consumed (joules) */
  energyConsumed: number;

  /** Actual duration (seconds) */
  duration: number;

  /** Average power (watts) */
  averagePower: number;

  /** Fluctuation level (0-1) */
  fluctuation: number;
}

// ============================================================================
// Energy Calculations
// ============================================================================

/**
 * Energy barrier parameters
 */
export interface EnergyBarrier {
  /** Barrier height (eV) */
  height: number;

  /** Barrier width (meters) */
  width: number;

  /** Tunneling probability */
  tunnelingProbability: number;

  /** Classical crossing energy (eV) */
  classicalEnergy: number;

  /** Quantum correction (eV) */
  quantumCorrection: number;
}

/**
 * Energy budget breakdown
 */
export interface EnergyBudget {
  /** Portal formation energy (joules) */
  formation: number;

  /** Stabilization energy (joules) */
  stabilization: number;

  /** Transfer energy (joules) */
  transfer: number;

  /** Containment energy (joules) */
  containment: number;

  /** Safety margin energy (joules) */
  safetyMargin: number;

  /** Total energy (joules) */
  total: number;
}

// ============================================================================
// Navigation and Transfer
// ============================================================================

/**
 * Navigation path
 */
export interface NavigationPath {
  /** Source coordinates */
  source: DimensionalCoordinates;

  /** Target coordinates */
  target: DimensionalCoordinates;

  /** Geodesic path points */
  waypoints: DimensionalCoordinates[];

  /** Total path length (meters in all dimensions) */
  totalLength: number;

  /** Estimated travel time (seconds) */
  travelTime: number;

  /** Energy requirement (joules) */
  energyRequired: number;

  /** Path type */
  pathType: 'geodesic' | 'minimal-energy' | 'time-optimal';
}

/**
 * Matter transfer specification
 */
export interface MatterTransferSpec {
  /** Mass to transfer (kilograms) */
  mass: number;

  /** Velocity vector [vx, vy, vz] (m/s) */
  velocity: number[];

  /** Target coordinates */
  targetCoordinates: DimensionalCoordinates;

  /** Transfer mode */
  transferMode: 'particle' | 'bulk' | 'quantum';

  /** Fidelity requirement (0-1) */
  fidelityRequirement?: number;

  /** Maximum transfer time (seconds) */
  maxTransferTime?: number;
}

/**
 * Matter transfer result
 */
export interface MatterTransferResult {
  /** Success status */
  success: boolean;

  /** Transferred mass (kilograms) */
  transferredMass: number;

  /** Energy consumed (joules) */
  energyUsed: number;

  /** Transfer duration (seconds) */
  duration: number;

  /** Achieved fidelity (0-1) */
  fidelity: number;

  /** Final position */
  finalPosition: DimensionalCoordinates;

  /** Error message if failed */
  error?: string;
}

/**
 * Quantum state transfer
 */
export interface QuantumStateTransfer {
  /** Input quantum state */
  inputState: Complex[];

  /** Output quantum state */
  outputState: Complex[];

  /** Transfer fidelity */
  fidelity: number;

  /** Entanglement entropy */
  entropy: number;

  /** Transfer operator */
  transferOperator?: Complex[][];
}

// ============================================================================
// Safety and Monitoring
// ============================================================================

/**
 * Safety status
 */
export type SafetyStatus = 'nominal' | 'warning' | 'critical' | 'emergency';

/**
 * Containment field status
 */
export interface ContainmentField {
  /** Field type */
  type: 'electromagnetic' | 'gravitational' | 'quantum';

  /** Field strength (relative to nominal) */
  strength: number;

  /** Coverage (0-1) */
  coverage: number;

  /** Integrity (0-1) */
  integrity: number;

  /** Status */
  status: SafetyStatus;
}

/**
 * Portal diagnostics
 */
export interface PortalDiagnostics {
  /** Portal ID */
  portalId: string;

  /** Timestamp */
  timestamp: Date;

  /** Stability index */
  stabilityIndex: number;

  /** Energy fluctuation */
  energyFluctuation: number;

  /** Temperature (Kelvin) */
  temperature: number;

  /** Radiation level (arbitrary units) */
  radiationLevel: number;

  /** Containment fields */
  containmentFields: ContainmentField[];

  /** Overall safety status */
  safetyStatus: SafetyStatus;

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];
}

/**
 * Emergency shutdown configuration
 */
export interface EmergencyShutdownConfig {
  /** Shutdown priority level */
  priority: 1 | 2 | 3;

  /** Maximum shutdown time (seconds) */
  maxShutdownTime: number;

  /** Energy dump capacity (joules) */
  energyDumpCapacity: number;

  /** Notification recipients */
  notificationRecipients?: string[];

  /** Reason for shutdown */
  reason: string;
}

/**
 * Shutdown result
 */
export interface ShutdownResult {
  /** Success status */
  success: boolean;

  /** Shutdown duration (seconds) */
  duration: number;

  /** Energy vented (joules) */
  energyVented: number;

  /** Final portal status */
  finalStatus: PortalStatus;

  /** Incident report ID */
  incidentReportId: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Dimensional Mapping
// ============================================================================

/**
 * Dimensional scan parameters
 */
export interface DimensionalScanParams {
  /** Scan center coordinates */
  center: DimensionalCoordinates;

  /** Scan radius (meters) */
  scanRadius: number;

  /** Resolution */
  resolution: 'low' | 'medium' | 'high' | 'ultra';

  /** Scanned dimensions */
  scannedDimensions: number[];

  /** Scan duration (seconds) */
  scanDuration?: number;
}

/**
 * Topology measurement
 */
export interface TopologyMeasurement {
  /** Euler characteristic */
  eulerCharacteristic: number;

  /** Betti numbers */
  bettiNumbers: number[];

  /** Riemann curvature tensor components */
  riemannTensor?: number[][][][];

  /** Ricci tensor */
  ricciTensor?: number[][];

  /** Ricci scalar */
  ricciScalar?: number;

  /** Detected geometry type */
  geometryType: CompactificationGeometry;
}

/**
 * Dimensional map
 */
export interface DimensionalMap {
  /** Map ID */
  id: string;

  /** Scan parameters */
  scanParams: DimensionalScanParams;

  /** Topology measurements */
  topology: TopologyMeasurement;

  /** Metric tensor components */
  metricTensor: number[][];

  /** Compactification specification */
  compactification: CompactificationSpec;

  /** Scan timestamp */
  timestamp: Date;

  /** Resolution achieved */
  resolution: number;

  /** Confidence level (0-1) */
  confidence: number;
}

// ============================================================================
// Complex Numbers
// ============================================================================

/**
 * Complex number representation
 */
export interface Complex {
  /** Real part */
  real: number;

  /** Imaginary part */
  imag: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-018 error codes
 */
export enum DimensionPortalErrorCode {
  INVALID_DIMENSIONS = 'DP001',
  INSUFFICIENT_ENERGY = 'DP002',
  INSTABILITY_DETECTED = 'DP003',
  CONTAINMENT_BREACH = 'DP004',
  COORDINATE_OUT_OF_BOUNDS = 'DP005',
  TOPOLOGY_MISMATCH = 'DP006',
  TRANSFER_FAILED = 'DP007',
  BARRIER_IMPENETRABLE = 'DP008',
  SAFETY_VIOLATION = 'DP009',
  PORTAL_COLLAPSED = 'DP010',
}

/**
 * Dimension portal error
 */
export class DimensionPortalError extends Error {
  constructor(
    public code: DimensionPortalErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DimensionPortalError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Vector in n-dimensional space
 */
export type Vector = number[];

/**
 * Matrix (2D array)
 */
export type Matrix = number[][];

/**
 * Tensor (multi-dimensional array)
 */
export type Tensor = number[] | number[][] | number[][][] | number[][][][];

// ============================================================================
// Export All
// ============================================================================

export type {
  // Coordinates
  SpacetimeCoordinates,
  ExtraDimensionalCoordinates,
  DimensionalCoordinates,
  CoordinateTransform,

  // Portal
  PortalSize,
  PortalStatus,
  ApertureShape,
  PortalConfiguration,
  PortalParameters,
  PortalFormationParams,
  PortalFormationResult,
  StabilizationConfig,
  StabilizationResult,

  // Kaluza-Klein
  KaluzaKleinMode,
  CompactificationGeometry,
  CompactificationSpec,

  // Energy
  EnergyBarrier,
  EnergyBudget,

  // Navigation
  NavigationPath,
  MatterTransferSpec,
  MatterTransferResult,
  QuantumStateTransfer,

  // Safety
  SafetyStatus,
  ContainmentField,
  PortalDiagnostics,
  EmergencyShutdownConfig,
  ShutdownResult,

  // Mapping
  DimensionalScanParams,
  TopologyMeasurement,
  DimensionalMap,

  // Math
  Complex,
  Vector,
  Matrix,
  Tensor,
};

export { PHYSICS_CONSTANTS, DIMENSIONAL_CONSTANTS, DimensionPortalErrorCode, DimensionPortalError };
