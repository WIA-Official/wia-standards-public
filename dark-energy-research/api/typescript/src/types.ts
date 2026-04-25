/**
 * WIA-QUA-014: Dark Energy Research - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Dark Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cosmological Types
// ============================================================================

/**
 * Cosmological model types
 */
export type CosmologicalModel =
  | 'LCDM' // Lambda-CDM (cosmological constant)
  | 'wCDM' // Constant equation of state w
  | 'CPL' // Chevallier-Polarski-Linder (w0-wa)
  | 'Quintessence' // Scalar field dark energy
  | 'Phantom' // w < -1
  | 'DGP' // Dvali-Gabadadze-Porrati
  | 'fR'; // f(R) modified gravity

/**
 * Cosmological parameters for ΛCDM
 */
export interface LCDMParameters {
  /** Hubble constant (km/s/Mpc) */
  H0: number;

  /** Matter density parameter */
  OmegaM: number;

  /** Dark energy density parameter (Lambda) */
  OmegaLambda: number;

  /** Curvature density parameter (default: 0 for flat) */
  OmegaK?: number;

  /** Baryon density parameter (optional) */
  OmegaB?: number;

  /** Radiation density parameter (optional) */
  OmegaR?: number;
}

/**
 * Equation of state parameters
 */
export interface EquationOfStateParams {
  /** Model type */
  model: 'constant' | 'CPL' | 'linear' | 'quintessence';

  /** Present-day equation of state */
  w0: number;

  /** Evolution parameter (CPL) */
  wa?: number;

  /** Scalar field potential parameters (quintessence) */
  potential?: {
    type: 'power-law' | 'exponential' | 'PNGB';
    parameters: number[];
  };
}

/**
 * Hubble parameter measurement
 */
export interface HubbleMeasurement {
  /** Redshift */
  z: number;

  /** Hubble parameter (km/s/Mpc) */
  H: number;

  /** Uncertainty */
  errorH: number;

  /** Measurement method */
  method: 'CMB' | 'BAO' | 'SNeIa' | 'Cepheid' | 'TRGB' | 'GW';

  /** Reference/survey name */
  reference: string;
}

// ============================================================================
// Distance Measures
// ============================================================================

/**
 * Cosmological distances at a given redshift
 */
export interface CosmologicalDistances {
  /** Redshift */
  z: number;

  /** Comoving distance (Mpc) */
  comovingDistance: number;

  /** Luminosity distance (Mpc) */
  luminosityDistance: number;

  /** Angular diameter distance (Mpc) */
  angularDiameterDistance: number;

  /** Distance modulus (mag) */
  distanceModulus: number;

  /** Lookback time (Gyr) */
  lookbackTime: number;

  /** Age of universe at this z (Gyr) */
  age: number;
}

/**
 * BAO distance measurements
 */
export interface BAODistance {
  /** Redshift */
  z: number;

  /** D_V / r_s (spherically averaged) */
  DVOverRS?: number;

  /** D_M / r_s (comoving transverse) */
  DMOverRS?: number;

  /** D_H / r_s (Hubble distance) */
  DHOverRS?: number;

  /** Uncertainties */
  errors: {
    DVOverRS?: number;
    DMOverRS?: number;
    DHOverRS?: number;
  };

  /** Survey name */
  survey: string;
}

// ============================================================================
// Supernova Data
// ============================================================================

/**
 * Type Ia supernova observation
 */
export interface SupernovaObservation {
  /** Supernova identifier */
  id: string;

  /** Redshift */
  z: number;

  /** Redshift uncertainty */
  errorZ: number;

  /** Apparent magnitude (peak B-band) */
  mB: number;

  /** Apparent magnitude uncertainty */
  errorMB: number;

  /** Color parameter */
  c?: number;

  /** Stretch parameter */
  x1?: number;

  /** Host galaxy mass (log M_solar) */
  hostMass?: number;

  /** Survey/sample */
  survey: string;
}

/**
 * Supernova dataset
 */
export interface SupernovaDataset {
  /** Dataset name */
  name: string;

  /** Number of supernovae */
  count: number;

  /** Observations */
  data: SupernovaObservation[];

  /** Redshift range */
  redshiftRange: {
    min: number;
    max: number;
  };

  /** Systematic uncertainties */
  systematics?: {
    calibration?: number;
    hostMass?: number;
    color?: number;
    stretch?: number;
  };
}

/**
 * Supernova standardization parameters
 */
export interface SupernovaStandardization {
  /** Absolute magnitude */
  M: number;

  /** Color correction coefficient */
  alpha: number;

  /** Stretch correction coefficient */
  beta: number;

  /** Host mass step */
  deltaM?: number;
}

// ============================================================================
// CMB Data
// ============================================================================

/**
 * CMB power spectrum
 */
export interface CMBPowerSpectrum {
  /** Multipole moments */
  ell: number[];

  /** TT power spectrum (μK²) */
  CTT: number[];

  /** TE power spectrum (μK²) */
  CTE?: number[];

  /** EE power spectrum (μK²) */
  CEE?: number[];

  /** Uncertainties */
  errors: {
    CTT: number[];
    CTE?: number[];
    CEE?: number[];
  };

  /** Mission/experiment */
  experiment: 'Planck' | 'WMAP' | 'ACT' | 'SPT';
}

/**
 * CMB derived parameters
 */
export interface CMBParameters {
  /** Sound horizon at recombination (Mpc) */
  soundHorizon: number;

  /** Angular size of sound horizon (degrees) */
  thetaStar: number;

  /** Redshift of recombination */
  zStar: number;

  /** Angular diameter distance to last scattering (Mpc) */
  dAStar: number;

  /** Shift parameter */
  R: number;

  /** Acoustic scale */
  lA: number;
}

// ============================================================================
// Cosmological Fate
// ============================================================================

/**
 * Future scenario types
 */
export type CosmologicalFate =
  | 'big-freeze' // Continued expansion, T → 0
  | 'big-rip' // Phantom energy tears everything apart
  | 'big-crunch' // Re-collapse to singularity
  | 'de-sitter' // Exponential expansion (LCDM)
  | 'vacuum-decay'; // Metastable vacuum transition

/**
 * Future prediction
 */
export interface FuturePrediction {
  /** Scenario type */
  scenario: CosmologicalFate;

  /** Equation of state today */
  w0: number;

  /** Time until critical event (years, if applicable) */
  timeToEvent?: number;

  /** Age when galaxies exit horizon (years) */
  galaxyHorizonExit?: number;

  /** Age when star formation ends (years) */
  starFormationEnd?: number;

  /** Description */
  description: string;

  /** Milestones */
  milestones?: Array<{
    time: number; // years from now
    event: string;
  }>;
}

// ============================================================================
// Parameter Estimation
// ============================================================================

/**
 * Fitting configuration
 */
export interface FittingConfig {
  /** Cosmological model */
  model: CosmologicalModel;

  /** Parameters to fit */
  parameters: string[];

  /** Prior ranges */
  priors: {
    [param: string]: {
      min: number;
      max: number;
      type?: 'uniform' | 'gaussian';
      mean?: number;
      std?: number;
    };
  };

  /** Datasets to use */
  datasets: Array<'SNeIa' | 'BAO' | 'CMB' | 'H(z)' | 'WeakLensing'>;

  /** MCMC configuration */
  mcmc?: {
    nChains: number;
    nSamples: number;
    burnIn: number;
    thinning?: number;
  };
}

/**
 * Fitting result
 */
export interface FittingResult {
  /** Best-fit parameters */
  bestFit: {
    [param: string]: number;
  };

  /** Parameter uncertainties (1σ) */
  errors: {
    [param: string]: {
      lower: number;
      upper: number;
    };
  };

  /** Chi-squared statistic */
  chiSquared: number;

  /** Degrees of freedom */
  dof: number;

  /** Reduced chi-squared */
  reducedChiSquared: number;

  /** Bayesian Information Criterion */
  BIC: number;

  /** Akaike Information Criterion */
  AIC: number;

  /** Log likelihood */
  logLikelihood: number;

  /** Covariance matrix */
  covariance?: number[][];

  /** MCMC chains (if used) */
  chains?: number[][][]; // [chain][sample][parameter]

  /** Convergence diagnostics */
  convergence?: {
    gelmanRubin: number[];
    effectiveSampleSize: number[];
  };
}

/**
 * Model comparison result
 */
export interface ModelComparison {
  /** Models compared */
  models: string[];

  /** Chi-squared values */
  chiSquared: number[];

  /** BIC values */
  BIC: number[];

  /** Delta BIC (relative to best) */
  deltaBIC: number[];

  /** Bayes factors (if calculated) */
  bayesFactors?: number[][];

  /** Best model */
  bestModel: string;

  /** Evidence strength */
  evidenceStrength: 'weak' | 'moderate' | 'strong' | 'decisive';
}

// ============================================================================
// Growth of Structure
// ============================================================================

/**
 * Linear growth factor
 */
export interface GrowthFactor {
  /** Redshift */
  z: number;

  /** Growth factor D(z) normalized to D(0) = 1 */
  D: number;

  /** Growth rate f = d ln D / d ln a */
  f: number;

  /** fσ₈(z) observable */
  fsigma8?: number;
}

/**
 * Matter power spectrum
 */
export interface PowerSpectrum {
  /** Wavenumber (h/Mpc) */
  k: number[];

  /** Power spectrum P(k) (Mpc/h)³ */
  Pk: number[];

  /** Redshift */
  z: number;

  /** Linear or non-linear */
  type: 'linear' | 'nonlinear';
}

// ============================================================================
// Modified Gravity
// ============================================================================

/**
 * Modified gravity parameters
 */
export interface ModifiedGravityParams {
  /** Theory type */
  theory: 'fR' | 'DGP' | 'Horndeski' | 'scalar-tensor';

  /** Model-specific parameters */
  parameters: {
    [key: string]: number;
  };

  /** Effective equation of state */
  wEffective?: number;

  /** Modified growth index gamma */
  gamma?: number;
}

/**
 * GW170817 constraint
 */
export interface GWConstraint {
  /** Speed of gravity constraint */
  cGravityOverC: {
    value: number;
    error: number;
  };

  /** Ruled out theories */
  ruledOut: string[];

  /** Surviving theories */
  allowed: string[];
}

// ============================================================================
// Vacuum Energy
// ============================================================================

/**
 * Vacuum energy calculation
 */
export interface VacuumEnergy {
  /** Energy density (J/m³) */
  rho: number;

  /** Cutoff scale (GeV) */
  cutoff: number;

  /** Cutoff type */
  cutoffType: 'Planck' | 'SUSY' | 'TeV' | 'QCD';

  /** Observed value (J/m³) */
  observed: number;

  /** Discrepancy factor */
  discrepancy: number;
}

/**
 * Anthropic bound
 */
export interface AnthropicBound {
  /** Maximum |ρ_Λ| for structure formation */
  maxRhoLambda: number;

  /** Reasoning */
  reasoning: string;

  /** Fine-tuning level */
  fineTuning: number;
}

// ============================================================================
// Observational Programs
// ============================================================================

/**
 * Dark energy survey
 */
export interface DarkEnergySurvey {
  /** Survey name */
  name: string;

  /** Status */
  status: 'completed' | 'ongoing' | 'planned' | 'proposed';

  /** Start year */
  startYear: number;

  /** End year (if applicable) */
  endYear?: number;

  /** Sky coverage (deg²) */
  skyCoverage: number;

  /** Observables */
  observables: Array<'SNeIa' | 'WeakLensing' | 'BAO' | 'Clusters' | 'RSD'>;

  /** Expected constraints */
  expectedConstraints?: {
    w0: number;
    wa: number;
  };

  /** Facilities */
  facilities: string[];
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and cosmological constants
 */
export const DARK_ENERGY_CONSTANTS = {
  /** Speed of light (km/s) */
  SPEED_OF_LIGHT: 299792.458,

  /** Gravitational constant (m³/kg/s²) */
  G: 6.67430e-11,

  /** Planck mass (GeV/c²) */
  PLANCK_MASS: 1.220910e19,

  /** Critical density today (M_sun/Mpc³) */
  RHO_CRITICAL_0: 2.77536627e11,

  /** Hubble distance (Mpc) */
  HUBBLE_DISTANCE: 2997.92458, // c/H₀ for H₀=100

  /** CMB temperature (K) */
  T_CMB: 2.7255,

  /** Baryon-to-photon ratio */
  ETA_B: 6.1e-10,

  /** Number of relativistic species */
  N_EFF: 3.046,

  /** Neutrino mass sum upper bound (eV) */
  NEUTRINO_MASS_SUM: 0.12,

  /** Current best values */
  PLANCK_2018: {
    H0: 67.36,
    OmegaM: 0.3111,
    OmegaLambda: 0.6889,
    OmegaB: 0.0490,
    sigma8: 0.8111,
    ns: 0.9665,
  },

  /** SH0ES 2022 */
  SH0ES_2022: {
    H0: 73.04,
    error: 1.04,
  },

  /** Standard ruler (BAO) */
  SOUND_HORIZON: 147.09, // Mpc

  /** Recombination redshift */
  Z_STAR: 1089.80,

  /** Transition redshift (decel → accel) */
  Z_ACCELERATION: 0.67,

  /** Age of universe (Gyr) */
  AGE_UNIVERSE: 13.787,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-014 error codes
 */
export enum DarkEnergyErrorCode {
  INVALID_REDSHIFT = 'DE001',
  INVALID_PARAMETERS = 'DE002',
  CONVERGENCE_FAILED = 'DE003',
  DATA_NOT_FOUND = 'DE004',
  FIT_FAILED = 'DE005',
  INTEGRATION_ERROR = 'DE006',
  UNPHYSICAL_MODEL = 'DE007',
  INSUFFICIENT_DATA = 'DE008',
  INVALID_MODEL = 'DE009',
}

/**
 * Dark energy research error
 */
export class DarkEnergyError extends Error {
  constructor(
    public code: DarkEnergyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DarkEnergyError';
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
 * Redshift array
 */
export type RedshiftArray = number[];

/**
 * Data point with uncertainty
 */
export interface DataPoint {
  x: number;
  y: number;
  error: number;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  CosmologicalModel,
  LCDMParameters,
  EquationOfStateParams,
  HubbleMeasurement,
  CosmologicalDistances,
  BAODistance,
  SupernovaObservation,
  SupernovaDataset,
  SupernovaStandardization,
  CMBPowerSpectrum,
  CMBParameters,
  CosmologicalFate,
  FuturePrediction,
  FittingConfig,
  FittingResult,
  ModelComparison,
  GrowthFactor,
  PowerSpectrum,
  ModifiedGravityParams,
  GWConstraint,
  VacuumEnergy,
  AnthropicBound,
  DarkEnergySurvey,
  RedshiftArray,
  DataPoint,
};

export { DARK_ENERGY_CONSTANTS, DarkEnergyErrorCode, DarkEnergyError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
