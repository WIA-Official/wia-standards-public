/**
 * WIA-QUA-013: Dark Matter Detection - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Dark Matter Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Dark Matter Candidates
// ============================================================================

/**
 * Dark matter candidate types
 */
export type DarkMatterCandidate =
  | 'WIMP'
  | 'axion'
  | 'sterile-neutrino'
  | 'primordial-black-hole'
  | 'self-interacting';

/**
 * WIMP (Weakly Interacting Massive Particle) parameters
 */
export interface WIMPParameters {
  /** WIMP mass in GeV */
  mass: number;

  /** WIMP-nucleon cross section in cm² */
  crossSection: number;

  /** Spin-independent or spin-dependent */
  interaction: 'spin-independent' | 'spin-dependent';

  /** Theoretical model */
  model?: 'neutralino' | 'kaluza-klein' | 'inert-higgs' | 'generic';

  /** Velocity distribution */
  velocityDistribution?: VelocityDistribution;
}

/**
 * Velocity distribution parameters
 */
export interface VelocityDistribution {
  /** Type of distribution */
  type: 'maxwell-boltzmann' | 'som' | 'stream';

  /** Circular velocity (km/s) */
  v0: number;

  /** Escape velocity (km/s) */
  vesc: number;

  /** Earth velocity (km/s) */
  vearth: number;

  /** Modulation phase (days from Jan 1) */
  modulationPhase?: number;
}

/**
 * Axion parameters
 */
export interface AxionParameters {
  /** Axion mass in eV */
  mass: number;

  /** Photon coupling in GeV⁻¹ */
  coupling: number;

  /** Axion model type */
  model: 'QCD' | 'KSVZ' | 'DFSZ' | 'generic';

  /** Peccei-Quinn scale in GeV */
  pqScale?: number;

  /** Local dark matter density (GeV/cm³) */
  density?: number;
}

// ============================================================================
// Direct Detection
// ============================================================================

/**
 * Detector types for direct detection
 */
export type DetectorType =
  | 'xenon'
  | 'argon'
  | 'germanium'
  | 'silicon'
  | 'sodium-iodide'
  | 'calcium-tungstate'
  | 'cf4-gas'
  | 'cs2-gas';

/**
 * Direct detection event
 */
export interface DirectDetectionEvent {
  /** Event ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Recoil energy in keV */
  energy: number;

  /** Event type */
  type: 'nuclear-recoil' | 'electron-recoil' | 'unknown';

  /** Position in detector (x, y, z) in mm */
  position?: [number, number, number];

  /** S1 signal (prompt scintillation) */
  s1?: number;

  /** S2 signal (ionization) */
  s2?: number;

  /** Pulse shape parameters */
  pulseShape?: PulseShapeParameters;

  /** Reconstruction quality */
  quality?: number;
}

/**
 * Pulse shape parameters
 */
export interface PulseShapeParameters {
  /** Prompt fraction */
  promptFraction: number;

  /** Rise time (ns) */
  riseTime: number;

  /** Fall time (ns) */
  fallTime: number;

  /** Pulse width (ns) */
  width: number;

  /** Asymmetry parameter */
  asymmetry?: number;
}

/**
 * Noble liquid detector configuration
 */
export interface NobleLiquidDetectorConfig {
  /** Detector name */
  name: string;

  /** Target material */
  material: 'xenon' | 'argon';

  /** Total mass in kg */
  totalMass: number;

  /** Fiducial mass in kg */
  fiducialMass: number;

  /** Energy threshold in keV */
  threshold: number;

  /** Exposure time in days */
  exposure: number;

  /** Detection efficiency */
  efficiency: number;

  /** Background rate in events/keV/kg/day */
  backgroundRate: number;
}

/**
 * Cryogenic bolometer configuration
 */
export interface CryogenicBolometerConfig {
  /** Crystal material */
  material: 'germanium' | 'silicon' | 'cawo4';

  /** Crystal mass in kg */
  mass: number;

  /** Operating temperature in mK */
  temperature: number;

  /** Energy resolution in eV */
  resolution: number;

  /** Threshold in eV */
  threshold: number;

  /** Phonon sensor type */
  sensorType: 'NTD' | 'TES' | 'MMC';
}

/**
 * Direct detection search parameters
 */
export interface DirectDetectionSearchParams {
  /** Detector configuration */
  detector: NobleLiquidDetectorConfig | CryogenicBolometerConfig;

  /** WIMP parameters */
  wimp: WIMPParameters;

  /** Analysis method */
  method: 'profile-likelihood' | 'bayesian' | 'optimum-interval';

  /** Confidence level (e.g., 0.90 for 90%) */
  confidenceLevel: number;

  /** Background model */
  backgroundModel?: BackgroundModel;
}

/**
 * Background model
 */
export interface BackgroundModel {
  /** Expected background events in ROI */
  expectedEvents: number;

  /** Energy spectrum */
  spectrum?: Array<{ energy: number; rate: number }>;

  /** Systematic uncertainty */
  uncertainty: number;

  /** Components */
  components?: BackgroundComponent[];
}

/**
 * Background component
 */
export interface BackgroundComponent {
  /** Component name */
  name: string;

  /** Type */
  type: 'radioactive' | 'cosmogenic' | 'neutrino' | 'surface';

  /** Rate contribution */
  rate: number;

  /** Isotope (if radioactive) */
  isotope?: string;
}

/**
 * Direct detection result
 */
export interface DirectDetectionResult {
  /** Total observed events */
  totalEvents: number;

  /** Signal events (after cuts) */
  signalEvents: number;

  /** Expected background */
  expectedBackground: number;

  /** Statistical significance (sigma) */
  significance: number;

  /** Cross-section limit in cm² */
  crossSectionLimit: number;

  /** Confidence level */
  confidenceLevel: number;

  /** Discovery claimed? */
  discovery: boolean;

  /** p-value */
  pValue?: number;

  /** Best-fit cross section (if detected) */
  bestFitCrossSection?: number;
}

// ============================================================================
// Indirect Detection
// ============================================================================

/**
 * Indirect detection method
 */
export type IndirectDetectionMethod =
  | 'gamma-ray'
  | 'neutrino'
  | 'antimatter'
  | 'cosmic-ray';

/**
 * Gamma ray observation
 */
export interface GammaRayObservation {
  /** Target name */
  target: string;

  /** Target type */
  targetType: 'galactic-center' | 'dwarf-galaxy' | 'cluster' | 'halo';

  /** J-factor in GeV² cm⁻⁵ */
  jFactor: number;

  /** J-factor uncertainty */
  jFactorUncertainty: number;

  /** Instrument */
  instrument: 'fermi-lat' | 'hess' | 'veritas' | 'magic' | 'cta';

  /** Energy range in GeV */
  energyRange: [number, number];

  /** Observation time in hours */
  observationTime: number;

  /** Flux data */
  fluxData?: Array<{ energy: number; flux: number; error: number }>;
}

/**
 * Neutrino detection parameters
 */
export interface NeutrinoDetectionParams {
  /** Target (Sun or Earth) */
  target: 'sun' | 'earth' | 'galactic-center';

  /** Detector */
  detector: 'icecube' | 'super-kamiokande' | 'antares' | 'km3net';

  /** Energy threshold in GeV */
  threshold: number;

  /** Effective area in m² */
  effectiveArea: number;

  /** Angular resolution in degrees */
  angularResolution: number;

  /** Live time in days */
  liveTime: number;
}

/**
 * Antimatter search parameters
 */
export interface AntimatterSearchParams {
  /** Particle type */
  particle: 'positron' | 'antiproton' | 'antideuteron';

  /** Instrument */
  instrument: 'ams-02' | 'pamela' | 'fermi-lat';

  /** Energy range in GeV */
  energyRange: [number, number];

  /** Propagation model */
  propagationModel?: 'med' | 'min' | 'max';

  /** Solar modulation parameter */
  solarModulation?: number;
}

/**
 * Indirect detection result
 */
export interface IndirectDetectionResult {
  /** Method used */
  method: IndirectDetectionMethod;

  /** Annihilation cross section limit in cm³/s */
  crossSectionLimit: number;

  /** WIMP mass in GeV */
  wimpMass: number;

  /** Annihilation channel */
  channel: string;

  /** Significance (if detected) */
  significance?: number;

  /** Best-fit cross section (if detected) */
  bestFitCrossSection?: number;
}

// ============================================================================
// Axion Detection
// ============================================================================

/**
 * Axion search type
 */
export type AxionSearchType =
  | 'cavity-haloscope'
  | 'helioscope'
  | 'light-shining-through-wall'
  | 'polarization';

/**
 * Cavity haloscope parameters
 */
export interface CavityHaloscopeParams {
  /** Resonant frequency in GHz */
  frequency: number;

  /** Magnetic field in Tesla */
  magneticField: number;

  /** Cavity quality factor */
  qualityFactor: number;

  /** Cavity volume in m³ */
  volume: number;

  /** Form factor */
  formFactor?: number;

  /** Integration time in seconds */
  integrationTime: number;
}

/**
 * Axion search result
 */
export interface AxionSearchResult {
  /** Search type */
  type: AxionSearchType;

  /** Axion mass range in eV */
  massRange: [number, number];

  /** Coupling limit in GeV⁻¹ */
  couplingLimit: number;

  /** Confidence level */
  confidenceLevel: number;

  /** Signal detected? */
  detected: boolean;

  /** Best-fit coupling (if detected) */
  bestFitCoupling?: number;

  /** Best-fit mass (if detected) */
  bestFitMass?: number;
}

// ============================================================================
// Collider Searches
// ============================================================================

/**
 * Collider experiment
 */
export type ColliderExperiment = 'ATLAS' | 'CMS' | 'LHCb' | 'ALICE';

/**
 * Collider search channel
 */
export type ColliderChannel =
  | 'monojet'
  | 'monophoton'
  | 'mono-Z'
  | 'mono-Higgs'
  | 'mono-W'
  | 'jets-MET'
  | 'leptons-MET';

/**
 * Collider search parameters
 */
export interface ColliderSearchParams {
  /** Experiment */
  experiment: ColliderExperiment;

  /** Search channel */
  channel: ColliderChannel;

  /** Integrated luminosity in fb⁻¹ */
  luminosity: number;

  /** Center-of-mass energy in TeV */
  sqrtS: number;

  /** Missing ET threshold in GeV */
  metThreshold?: number;

  /** Mediator model */
  mediatorModel?: 'vector' | 'axial-vector' | 'scalar' | 'pseudoscalar';
}

/**
 * Collider search result
 */
export interface ColliderSearchResult {
  /** Channel searched */
  channel: ColliderChannel;

  /** Observed events */
  observedEvents: number;

  /** Expected background */
  expectedBackground: number;

  /** Mediator mass limit in GeV */
  mediatorMassLimit?: number;

  /** DM mass limit in GeV */
  dmMassLimit?: number;

  /** Effective field theory scale limit in GeV */
  eftScaleLimit?: number;
}

// ============================================================================
// Astrophysical Observations
// ============================================================================

/**
 * Gravitational lensing parameters
 */
export interface GravitationalLensingParams {
  /** Cluster or galaxy name */
  object: string;

  /** Redshift */
  redshift: number;

  /** Lensing type */
  type: 'strong' | 'weak' | 'microlensing';

  /** Mass reconstruction method */
  method?: 'parametric' | 'non-parametric' | 'free-form';

  /** Imaging data */
  imagingData?: MassMapData;
}

/**
 * Mass map data
 */
export interface MassMapData {
  /** RA, Dec grid */
  coordinates: Array<{ ra: number; dec: number }>;

  /** Surface mass density in M☉/pc² */
  surfaceDensity: number[][];

  /** Uncertainty */
  uncertainty: number[][];

  /** Resolution in arcsec */
  resolution: number;
}

/**
 * Gravitational lensing result
 */
export interface GravitationalLensingResult {
  /** Total mass in M☉ */
  totalMass: number;

  /** Total mass uncertainty */
  massUncertainty: number;

  /** Dark matter fraction */
  darkMatterFraction: number;

  /** Mass-to-light ratio in M☉/L☉ */
  massToLightRatio: number;

  /** Concentration parameter */
  concentration?: number;

  /** Virial radius in kpc */
  virialRadius?: number;
}

/**
 * Rotation curve data
 */
export interface RotationCurveData {
  /** Galaxy name */
  galaxy: string;

  /** Distance measurements in kpc */
  radius: number[];

  /** Velocity measurements in km/s */
  velocity: number[];

  /** Velocity uncertainties */
  velocityError: number[];

  /** HI or CO data */
  tracer: 'HI' | 'CO' | 'optical' | 'combined';
}

/**
 * Rotation curve fit result
 */
export interface RotationCurveFitResult {
  /** Dark matter profile */
  profile: 'NFW' | 'Einasto' | 'Burkert' | 'isothermal';

  /** Halo mass in M☉ */
  haloMass: number;

  /** Scale radius in kpc */
  scaleRadius: number;

  /** Concentration */
  concentration: number;

  /** Chi-squared */
  chiSquared: number;

  /** Degrees of freedom */
  dof: number;
}

// ============================================================================
// Signal Discrimination
// ============================================================================

/**
 * Discrimination method
 */
export type DiscriminationMethod =
  | 'pulse-shape'
  | 'charge-light-ratio'
  | 'neural-network'
  | 'boosted-decision-tree'
  | 'multiple-scatter';

/**
 * Machine learning classifier configuration
 */
export interface MLClassifierConfig {
  /** Model type */
  type: 'neural-network' | 'random-forest' | 'xgboost' | 'svm';

  /** Input features */
  features: string[];

  /** Training parameters */
  training?: {
    epochs?: number;
    batchSize?: number;
    learningRate?: number;
    validationSplit?: number;
  };

  /** Model file path (if pre-trained) */
  modelPath?: string;
}

/**
 * Event classification result
 */
export interface EventClassification {
  /** Event ID */
  eventId: string;

  /** Nuclear recoil probability */
  nrProbability: number;

  /** Electron recoil probability */
  erProbability: number;

  /** Classification (based on threshold) */
  classification: 'nuclear-recoil' | 'electron-recoil' | 'ambiguous';

  /** Confidence score */
  confidence: number;
}

// ============================================================================
// Statistical Analysis
// ============================================================================

/**
 * Statistical method
 */
export type StatisticalMethod =
  | 'profile-likelihood'
  | 'bayesian'
  | 'optimum-interval'
  | 'cls';

/**
 * Likelihood analysis parameters
 */
export interface LikelihoodAnalysisParams {
  /** Observed events */
  observed: number;

  /** Expected signal */
  expectedSignal: number;

  /** Expected background */
  expectedBackground: number;

  /** Systematic uncertainties */
  systematics?: {
    signalUncertainty: number;
    backgroundUncertainty: number;
  };

  /** Nuisance parameters */
  nuisanceParams?: Map<string, number>;
}

/**
 * Statistical result
 */
export interface StatisticalResult {
  /** Method used */
  method: StatisticalMethod;

  /** p-value */
  pValue: number;

  /** Significance in sigma */
  significance: number;

  /** Upper limit (90% CL) */
  upperLimit90: number;

  /** Upper limit (95% CL) */
  upperLimit95?: number;

  /** Best-fit value */
  bestFit?: number;

  /** Confidence interval */
  confidenceInterval?: [number, number];
}

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * Detector performance metrics
 */
export interface DetectorPerformance {
  /** Timestamp */
  timestamp: Date;

  /** Detector uptime (fraction) */
  uptime: number;

  /** Event rate (events/day) */
  eventRate: number;

  /** Background rate (events/day) */
  backgroundRate: number;

  /** Energy resolution at 1 keV (%) */
  energyResolution: number;

  /** Position resolution (mm) */
  positionResolution?: number;

  /** Discrimination power (ER rejection at 50% NR) */
  discriminationPower?: number;

  /** Trigger efficiency */
  triggerEfficiency: number;
}

/**
 * Experiment status
 */
export interface ExperimentStatus {
  /** Experiment name */
  name: string;

  /** Status */
  status: 'planning' | 'construction' | 'commissioning' | 'running' | 'completed';

  /** Location */
  location: string;

  /** Depth in m.w.e. */
  depth: number;

  /** Start date */
  startDate: Date;

  /** Projected/actual end date */
  endDate?: Date;

  /** Current exposure in kg·days */
  currentExposure: number;

  /** Target exposure */
  targetExposure: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-013 error codes
 */
export enum DarkMatterErrorCode {
  INVALID_PARAMETERS = 'DM001',
  INSUFFICIENT_EXPOSURE = 'DM002',
  HIGH_BACKGROUND = 'DM003',
  CALIBRATION_FAILED = 'DM004',
  DISCRIMINATION_FAILED = 'DM005',
  ANALYSIS_FAILED = 'DM006',
  DATA_QUALITY_ISSUE = 'DM007',
  SIMULATION_ERROR = 'DM008',
  LIMIT_CALCULATION_FAILED = 'DM009',
  UNKNOWN_ERROR = 'DM999',
}

/**
 * Dark matter detection error
 */
export class DarkMatterError extends Error {
  constructor(
    public code: DarkMatterErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DarkMatterError';
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

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for dark matter detection
 */
export const DARK_MATTER_CONSTANTS = {
  /** Local dark matter density (GeV/cm³) */
  LOCAL_DM_DENSITY: 0.3,

  /** Circular velocity (km/s) */
  V0_VELOCITY: 220,

  /** Escape velocity (km/s) */
  ESCAPE_VELOCITY: 544,

  /** Earth velocity (km/s) */
  EARTH_VELOCITY: 232,

  /** Speed of light (km/s) */
  SPEED_OF_LIGHT: 299792.458,

  /** GeV to erg */
  GEV_TO_ERG: 1.60218e-3,

  /** Planck mass (GeV) */
  PLANCK_MASS: 1.22e19,

  /** Weak scale (GeV) */
  WEAK_SCALE: 246,

  /** Thermal relic cross section (cm³/s) */
  THERMAL_RELIC_CROSS_SECTION: 3e-26,

  /** Nucleon mass (GeV) */
  NUCLEON_MASS: 0.939,

  /** Solar mass (g) */
  SOLAR_MASS: 1.989e33,

  /** kpc to cm */
  KPC_TO_CM: 3.086e21,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  DarkMatterCandidate,
  WIMPParameters,
  VelocityDistribution,
  AxionParameters,
  DetectorType,
  DirectDetectionEvent,
  PulseShapeParameters,
  NobleLiquidDetectorConfig,
  CryogenicBolometerConfig,
  DirectDetectionSearchParams,
  BackgroundModel,
  BackgroundComponent,
  DirectDetectionResult,
  IndirectDetectionMethod,
  GammaRayObservation,
  NeutrinoDetectionParams,
  AntimatterSearchParams,
  IndirectDetectionResult,
  AxionSearchType,
  CavityHaloscopeParams,
  AxionSearchResult,
  ColliderExperiment,
  ColliderChannel,
  ColliderSearchParams,
  ColliderSearchResult,
  GravitationalLensingParams,
  MassMapData,
  GravitationalLensingResult,
  RotationCurveData,
  RotationCurveFitResult,
  DiscriminationMethod,
  MLClassifierConfig,
  EventClassification,
  StatisticalMethod,
  LikelihoodAnalysisParams,
  StatisticalResult,
  DetectorPerformance,
  ExperimentStatus,
};

export { DARK_MATTER_CONSTANTS, DarkMatterErrorCode, DarkMatterError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
