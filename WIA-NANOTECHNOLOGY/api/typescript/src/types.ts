/**
 * WIA-NANOTECHNOLOGY TypeScript Type Definitions
 *
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive type system for nanomaterial data management,
 * synthesis planning, characterization analysis, and simulation.
 */

// ============================================================================
// Base Types
// ============================================================================

export type UUID = string;
export type ISO8601Timestamp = string;
export type URL = string;
export type DOI = string;

// ============================================================================
// Material Types
// ============================================================================

export enum MaterialType {
  CNT = 'CNT',
  Graphene = 'graphene',
  QuantumDot = 'quantum_dot',
  Nanoparticle = 'nanoparticle',
  Nanocomposite = 'nanocomposite',
  Nanofilm = 'nanofilm',
  Nanowire = 'nanowire',
  Nanotube = 'nanotube',
  Fullerene = 'fullerene',
  Nanosheet = 'nanosheet'
}

export enum NanoparticleType {
  Gold = 'gold',
  Silver = 'silver',
  TitaniumDioxide = 'titanium_dioxide',
  QuantumDot = 'quantum_dot',
  Iron = 'iron',
  Platinum = 'platinum',
  Palladium = 'palladium',
  Copper = 'copper',
  Zinc = 'zinc',
  Other = 'other'
}

export interface Element {
  symbol: string;
  atomicNumber: number;
  percentage: number;
}

export interface Composition {
  elements: Element[];
  formula: string;
  molecularWeight: number;
}

export interface Dimension {
  value: number;
  unit: 'nm' | 'μm' | 'mm';
  range?: {
    min: number;
    max: number;
  };
}

export interface Dimensions {
  length?: Dimension;
  width?: Dimension;
  height?: Dimension;
  diameter?: Dimension;
  aspectRatio?: number;
}

export enum StructureType {
  Crystalline = 'crystalline',
  Amorphous = 'amorphous',
  Polycrystalline = 'polycrystalline',
  SemiCrystalline = 'semi_crystalline'
}

export interface LatticeParameters {
  a: number;
  b: number;
  c: number;
  alpha: number;
  beta: number;
  gamma: number;
}

export interface Structure {
  type: StructureType;
  latticeParameters?: LatticeParameters;
  spaceGroup?: string;
}

export interface PhysicalProperties {
  density?: number;
  surfaceArea?: number;
  porosity?: number;
  crystallinity?: number;
}

export interface MechanicalProperties {
  youngsModulus?: number;
  tensileStrength?: number;
  hardness?: number;
  compressiveStrength?: number;
  shearModulus?: number;
  poissonsRatio?: number;
}

export interface ElectricalProperties {
  conductivity?: number;
  bandgap?: number;
  mobility?: number;
  resistivity?: number;
  dielectricConstant?: number;
}

export interface OpticalProperty {
  wavelength: number;
  intensity?: number;
  quantumYield?: number;
}

export interface OpticalProperties {
  absorptionPeaks?: OpticalProperty[];
  emissionPeaks?: OpticalProperty[];
  refractiveIndex?: number;
  extinctionCoefficient?: number;
}

export interface MagneticProperties {
  susceptibility?: number;
  coercivity?: number;
  saturationMagnetization?: number;
  curieTemeprature?: number;
}

export interface ThermalProperties {
  meltingPoint?: number;
  thermalConductivity?: number;
  specificHeat?: number;
  thermalExpansionCoefficient?: number;
}

export interface MaterialProperties {
  physical?: PhysicalProperties;
  mechanical?: MechanicalProperties;
  electrical?: ElectricalProperties;
  optical?: OpticalProperties;
  magnetic?: MagneticProperties;
  thermal?: ThermalProperties;
}

export interface MaterialMetadata {
  createdAt: ISO8601Timestamp;
  updatedAt: ISO8601Timestamp;
  source: string;
  doi?: DOI;
  references?: string[];
}

export interface Material {
  materialId: UUID;
  type: MaterialType;
  name: string;
  composition: Composition;
  dimensions: Dimensions;
  structure: Structure;
  properties: MaterialProperties;
  metadata: MaterialMetadata;
}

// ============================================================================
// Nanoparticle-Specific Types
// ============================================================================

export interface CoreShell {
  hasShell: boolean;
  coreComposition: string;
  shellComposition?: string;
  shellThickness?: number;
}

export interface Ligand {
  name: string;
  coverage: number;
  chainLength?: number;
}

export interface SurfaceModification {
  ligands?: Ligand[];
  functionalGroups?: string[];
}

export interface Dispersibility {
  solvent: string;
  concentration: number;
  zetaPotential?: number;
  hydrodynamicDiameter?: number;
}

export interface Nanoparticle extends Material {
  nanoparticleType: NanoparticleType;
  coreShell?: CoreShell;
  surfaceModification?: SurfaceModification;
  dispersibility?: Dispersibility;
}

// ============================================================================
// Characterization Types
// ============================================================================

export enum CharacterizationTechnique {
  TEM = 'TEM',
  SEM = 'SEM',
  AFM = 'AFM',
  XRD = 'XRD',
  XPS = 'XPS',
  Raman = 'Raman',
  FTIR = 'FTIR',
  DLS = 'DLS',
  BET = 'BET',
  UV_Vis = 'UV_Vis',
  PL = 'PL',
  EDX = 'EDX',
  EELS = 'EELS',
  NMR = 'NMR',
  ICP_MS = 'ICP_MS',
  Other = 'other'
}

export interface Instrument {
  model: string;
  manufacturer: string;
  serialNumber?: string;
}

export interface CharacterizationConditions {
  temperature?: number;
  pressure?: number;
  atmosphere?: string;
}

export interface CharacterizationBase {
  characterizationId: UUID;
  materialId: UUID;
  technique: CharacterizationTechnique;
  timestamp: ISO8601Timestamp;
  operator: string;
  instrument: Instrument;
  conditions?: CharacterizationConditions;
}

// TEM/SEM specific types
export interface ImageAnnotation {
  type: 'particle' | 'defect' | 'region';
  coordinates: { x: number; y: number };
  measurement?: {
    value: number;
    unit: string;
  };
}

export interface MicroscopyImage {
  url: URL;
  magnification: number;
  accelerationVoltage: number;
  resolution: number;
  annotations?: ImageAnnotation[];
}

export interface HistogramBin {
  binCenter: number;
  count: number;
}

export interface ParticleSizeDistribution {
  mean: number;
  median: number;
  standardDeviation: number;
  histogram?: HistogramBin[];
}

export interface TEMResults {
  images: MicroscopyImage[];
  particleSizeDistribution?: ParticleSizeDistribution;
}

export interface TEMCharacterization extends CharacterizationBase {
  technique: CharacterizationTechnique.TEM;
  results: TEMResults;
}

export interface SEMResults extends TEMResults {
  // SEM may have additional fields
}

export interface SEMCharacterization extends CharacterizationBase {
  technique: CharacterizationTechnique.SEM;
  results: SEMResults;
}

// XRD specific types
export interface XRDDataPoint {
  twoTheta: number;
  intensity: number;
}

export interface MillerIndices {
  h: number;
  k: number;
  l: number;
}

export interface XRDPeak {
  twoTheta: number;
  dSpacing: number;
  intensity: number;
  fwhm: number;
  millerIndices?: MillerIndices;
}

export interface Phase {
  name: string;
  percentage: number;
}

export interface XRDResults {
  pattern: XRDDataPoint[];
  peaks: XRDPeak[];
  crystalliteSize?: number;
  latticeParameters?: LatticeParameters;
  phases?: Phase[];
}

export interface XRDCharacterization extends CharacterizationBase {
  technique: CharacterizationTechnique.XRD;
  results: XRDResults;
}

// AFM specific types
export interface AFMScanSize {
  width: number;
  height: number;
}

export interface AFMResolution {
  x: number;
  y: number;
}

export interface AFMTopography {
  imageUrl: URL;
  dataMatrix?: number[][];
}

export interface RoughnessAnalysis {
  rms: number;
  ra: number;
  peakToPeak: number;
}

export interface AFMFeature {
  type: 'particle' | 'grain' | 'defect';
  height: number;
  lateralSize: number;
}

export interface AFMResults {
  scanSize: AFMScanSize;
  resolution: AFMResolution;
  topography: AFMTopography;
  roughnessAnalysis?: RoughnessAnalysis;
  features?: AFMFeature[];
}

export interface AFMCharacterization extends CharacterizationBase {
  technique: CharacterizationTechnique.AFM;
  results: AFMResults;
}

// Generic characterization
export interface Characterization extends CharacterizationBase {
  results: any;
}

// ============================================================================
// Synthesis Types
// ============================================================================

export enum SynthesisMethod {
  CVD = 'chemical_vapor_deposition',
  PVD = 'physical_vapor_deposition',
  SolGel = 'sol_gel',
  Hydrothermal = 'hydrothermal',
  Electrochemical = 'electrochemical',
  BallMilling = 'ball_milling',
  MBE = 'molecular_beam_epitaxy',
  Lithography = 'lithography',
  SelfAssembly = 'self_assembly',
  Coprecipitation = 'coprecipitation',
  Other = 'other'
}

export interface Amount {
  value: number;
  unit: 'g' | 'mL' | 'mol' | 'mmol' | 'mg' | 'μL';
}

export interface Precursor {
  name: string;
  formula: string;
  purity: number;
  amount: Amount;
  supplier?: string;
}

export interface Solvent {
  name: string;
  volume: number;
  purity: string;
}

export interface ProcessStep {
  stepNumber: number;
  description: string;
  duration: number;
  temperature?: number;
  pressure?: number;
  parameters?: Record<string, any>;
}

export interface Yield {
  mass: number;
  percentage: number;
}

export interface Quality {
  uniformity: number;
  purity: number;
  defectDensity?: number;
}

export interface SynthesisRecord {
  synthesisId: UUID;
  materialId: UUID;
  method: SynthesisMethod;
  timestamp: ISO8601Timestamp;
  operator: string;
  precursors: Precursor[];
  solvents?: Solvent[];
  processSteps: ProcessStep[];
  yield?: Yield;
  quality?: Quality;
}

// CVD specific types
export interface GasFlow {
  gas: string;
  flowRate: number;
}

export interface CVDSubstrate {
  material: string;
  pretreatment?: string;
}

export interface CVDParameters {
  gasFlow: GasFlow[];
  chamberPressure: number;
  temperature: number;
  rfPower?: number;
  substrate: CVDSubstrate;
  growthTime: number;
  coolingRate: number;
}

export interface CVDSynthesis extends SynthesisRecord {
  method: SynthesisMethod.CVD;
  cvdParameters: CVDParameters;
}

// ============================================================================
// Simulation Types
// ============================================================================

export enum SimulationType {
  MolecularDynamics = 'molecular_dynamics',
  DFT = 'density_functional_theory',
  FiniteElement = 'finite_element',
  MonteCarlo = 'monte_carlo',
  CoarseGrained = 'coarse_grained',
  QuantumMechanics = 'quantum_mechanics'
}

export enum SimulationStatus {
  Queued = 'queued',
  Running = 'running',
  Completed = 'completed',
  Failed = 'failed',
  Cancelled = 'cancelled'
}

export interface Software {
  name: string;
  version: string;
}

export interface SimulationSystem {
  materialId?: UUID;
  numberOfAtoms: number;
  dimensions: {
    x: number;
    y: number;
    z: number;
  };
  periodicBoundaryConditions: boolean;
}

export enum Ensemble {
  NVE = 'NVE',
  NVT = 'NVT',
  NPT = 'NPT',
  NpT = 'NpT'
}

export interface SimulationParameters {
  timeStep?: number;
  totalTime?: number;
  temperature?: number;
  ensemble?: Ensemble;
  potential?: string;
  [key: string]: any;
}

export interface EnergyResults {
  potential: number;
  kinetic: number;
  total: number;
}

export interface SimulationResults {
  energy?: EnergyResults;
  trajectory?: URL;
  properties?: Record<string, any>;
}

export interface ComputeResources {
  cores: number;
  memory: string;
  wallTime: string;
}

export interface ComputeStats {
  coreHours: number;
  peakMemory: string;
  wallTime: number;
}

export interface Simulation {
  simulationId: UUID;
  type: SimulationType;
  status: SimulationStatus;
  software: Software;
  system: SimulationSystem;
  parameters: SimulationParameters;
  results?: SimulationResults;
  computeResources?: ComputeResources;
  computeStats?: ComputeStats;
  queuePosition?: number;
  progress?: number;
  estimatedStartTime?: ISO8601Timestamp;
  estimatedCompletion?: ISO8601Timestamp;
  startedAt?: ISO8601Timestamp;
  completedAt?: ISO8601Timestamp;
}

// ============================================================================
// Safety Types
// ============================================================================

export interface GHSClassification {
  codes: string[];
}

export interface NFPARating {
  health: number;
  flammability: number;
  reactivity: number;
  special?: string;
}

export interface HazardClassification {
  ghs?: GHSClassification;
  nfpa?: NFPARating;
}

export interface ExposureLimits {
  pel?: number;
  rel?: number;
  tlv?: number;
}

export interface LD50Data {
  value: number;
  route: 'oral' | 'dermal' | 'inhalation';
  species: string;
}

export interface CytotoxicityData {
  cellLine: string;
  ic50: number;
  assay: string;
}

export interface EcotoxicityData {
  organism: string;
  lc50: number;
  duration: number;
}

export interface Toxicology {
  ld50?: LD50Data;
  cytotoxicity?: CytotoxicityData[];
  ecotoxicity?: EcotoxicityData[];
}

export interface Handling {
  ppe: string[];
  storageConditions?: string;
  disposalMethod?: string;
}

export interface SafetyData {
  safetyId: UUID;
  materialId: UUID;
  hazardClassification: HazardClassification;
  exposureLimits?: ExposureLimits;
  toxicology?: Toxicology;
  handling: Handling;
}

// ============================================================================
// Protocol Types
// ============================================================================

export interface Protocol {
  protocolId: UUID;
  name: string;
  version: string;
  type: 'synthesis' | 'characterization' | 'safety';
  description: string;
  steps: any[];
  validatedBy?: string;
}

// ============================================================================
// Search and Query Types
// ============================================================================

export interface SearchFilters {
  type?: MaterialType;
  elements?: string[];
  minSize?: number;
  maxSize?: number;
  properties?: Record<string, any>;
}

export interface Pagination {
  page: number;
  limit: number;
  total: number;
  pages: number;
}

export interface SearchResults<T> {
  items: T[];
  pagination: Pagination;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: any;
  timestamp: ISO8601Timestamp;
  requestId?: UUID;
}

export interface APIResponse<T> {
  data?: T;
  error?: APIError;
}

// ============================================================================
// Property Prediction Types
// ============================================================================

export enum PredictionMethod {
  ML = 'ML',
  DFT = 'DFT',
  Empirical = 'empirical',
  SemiEmpirical = 'semi_empirical'
}

export interface PropertyPrediction {
  value: number;
  unit: string;
  confidence: number;
  method: PredictionMethod;
}

export interface PredictionResults {
  predictionId: UUID;
  materialId: UUID;
  predictions: Record<string, PropertyPrediction>;
  timestamp: ISO8601Timestamp;
}

// ============================================================================
// Compliance Types
// ============================================================================

export interface RegulationStatus {
  regulation: string;
  status: 'compliant' | 'non_compliant' | 'pending' | 'unknown';
  registrationNumber?: string;
  expiresAt?: ISO8601Timestamp;
}

export interface ComplianceCheck {
  materialId: UUID;
  compliant: boolean;
  details: RegulationStatus[];
  warnings?: string[];
  requiredActions?: string[];
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  MaterialType,
  NanoparticleType,
  CharacterizationTechnique,
  SynthesisMethod,
  SimulationType,
  SimulationStatus,
  Ensemble,
  PredictionMethod,
  StructureType
};
