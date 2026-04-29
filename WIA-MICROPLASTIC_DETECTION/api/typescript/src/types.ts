/**
 * WIA-MICROPLASTIC_DETECTION TypeScript Type Definitions
 *
 * Comprehensive type system for microplastic detection, analysis, and monitoring
 *
 * @module @wia/microplastic-detection/types
 * @version 1.0.0
 * @philosophy 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Enumerations
// ============================================================================

/**
 * Polymer type classification codes
 */
export enum PolymerCode {
  PE = "PE",                       // Polyethylene
  PP = "PP",                       // Polypropylene
  PS = "PS",                       // Polystyrene
  PET = "PET",                     // Polyethylene terephthalate
  PVC = "PVC",                     // Polyvinyl chloride
  PA = "PA",                       // Polyamide (Nylon)
  PC = "PC",                       // Polycarbonate
  PMMA = "PMMA",                   // Polymethyl methacrylate (Acrylic)
  PU = "PU",                       // Polyurethane
  PTFE = "PTFE",                   // Polytetrafluoroethylene (Teflon)
  EVA = "EVA",                     // Ethylene-vinyl acetate
  ABS = "ABS",                     // Acrylonitrile butadiene styrene
  PEEK = "PEEK",                   // Polyether ether ketone
  UNKNOWN = "UNKNOWN",
  MIXED = "MIXED"
}

/**
 * Size classification for particles
 */
export enum SizeClass {
  NANOPLASTIC = "NANOPLASTIC",     // < 1 μm
  SMALL_MICROPLASTIC = "SMALL",    // 1-100 μm
  LARGE_MICROPLASTIC = "LARGE",    // 100-1000 μm
  MESOPLASTIC = "MESOPLASTIC",     // 1-5 mm
  MACROPLASTIC = "MACROPLASTIC"    // > 5 mm
}

/**
 * Shape categories for particles
 */
export enum ShapeCategory {
  FIBER = "FIBER",
  FRAGMENT = "FRAGMENT",
  FILM = "FILM",
  FOAM = "FOAM",
  BEAD = "BEAD",
  PELLET = "PELLET",
  FLAKE = "FLAKE",
  OTHER = "OTHER"
}

/**
 * Color categories
 */
export enum ColorCategory {
  TRANSPARENT = "TRANSPARENT",
  WHITE = "WHITE",
  BLACK = "BLACK",
  RED = "RED",
  BLUE = "BLUE",
  GREEN = "GREEN",
  YELLOW = "YELLOW",
  ORANGE = "ORANGE",
  BROWN = "BROWN",
  MULTICOLOR = "MULTICOLOR"
}

/**
 * Transparency levels
 */
export enum TransparencyLevel {
  TRANSPARENT = "TRANSPARENT",
  TRANSLUCENT = "TRANSLUCENT",
  OPAQUE = "OPAQUE"
}

/**
 * Environmental type classification
 */
export enum EnvironmentType {
  MARINE_SURFACE = "MARINE_SURFACE",
  MARINE_SUBSURFACE = "MARINE_SUBSURFACE",
  MARINE_SEDIMENT = "MARINE_SEDIMENT",
  FRESHWATER_SURFACE = "FRESHWATER_SURFACE",
  FRESHWATER_SUBSURFACE = "FRESHWATER_SUBSURFACE",
  FRESHWATER_SEDIMENT = "FRESHWATER_SEDIMENT",
  SOIL = "SOIL",
  AIR = "AIR",
  WASTEWATER = "WASTEWATER",
  DRINKING_WATER = "DRINKING_WATER",
  BIOTA = "BIOTA"
}

/**
 * Sampling methods
 */
export enum SamplingMethod {
  MANTA_TRAWL = "MANTA_TRAWL",
  NEUSTON_NET = "NEUSTON_NET",
  BULK_WATER = "BULK_WATER",
  PUMP_FILTRATION = "PUMP_FILTRATION",
  GRAB_SAMPLER = "GRAB_SAMPLER",
  SEDIMENT_CORE = "SEDIMENT_CORE",
  AIR_SAMPLER = "AIR_SAMPLER",
  MANUAL_COLLECTION = "MANUAL_COLLECTION"
}

/**
 * Detection methods
 */
export enum DetectionMethod {
  VISUAL_MICROSCOPY = "VISUAL_MICROSCOPY",
  RAMAN_SPECTROSCOPY = "RAMAN_SPECTROSCOPY",
  FTIR_SPECTROSCOPY = "FTIR_SPECTROSCOPY",
  FLUORESCENCE_IMAGING = "FLUORESCENCE_IMAGING",
  SEM_EDS = "SEM_EDS",
  PYROLYSIS_GC_MS = "PYROLYSIS_GC_MS",
  AUTOMATED_IMAGING = "AUTOMATED_IMAGING"
}

/**
 * Analysis methods
 */
export enum AnalysisMethod {
  AUTOMATED_IMAGING = "AUTOMATED_IMAGING",
  MANUAL_SORTING = "MANUAL_SORTING",
  SPECTROSCOPIC_ANALYSIS = "SPECTROSCOPIC_ANALYSIS",
  COMBINED_APPROACH = "COMBINED_APPROACH"
}

/**
 * Quality control status
 */
export enum QCStatus {
  PASSED = "PASSED",
  PASSED_WITH_NOTES = "PASSED_WITH_NOTES",
  FAILED = "FAILED",
  PENDING = "PENDING"
}

/**
 * Quality levels for spectroscopy
 */
export enum QualityLevel {
  EXCELLENT = "EXCELLENT",
  GOOD = "GOOD",
  ACCEPTABLE = "ACCEPTABLE",
  POOR = "POOR",
  FAILED = "FAILED"
}

/**
 * Job status
 */
export enum JobStatus {
  QUEUED = "QUEUED",
  IN_PROGRESS = "IN_PROGRESS",
  COMPLETED = "COMPLETED",
  FAILED = "FAILED",
  CANCELLED = "CANCELLED"
}

/**
 * Concentration units
 */
export enum ConcentrationUnit {
  PARTICLES_PER_LITER = "particles/L",
  PARTICLES_PER_CUBIC_METER = "particles/m³",
  PARTICLES_PER_KILOGRAM = "particles/kg",
  PARTICLES_PER_SQUARE_METER = "particles/m²"
}

/**
 * Sample type
 */
export enum SampleType {
  ENVIRONMENTAL = "ENVIRONMENTAL",
  BLANK = "BLANK",
  CONTROL = "CONTROL",
  REPLICATE = "REPLICATE",
  REFERENCE = "REFERENCE"
}

/**
 * Processing methods
 */
export enum ProcessingMethod {
  DENSITY_SEPARATION = "DENSITY_SEPARATION",
  HYDROGEN_PEROXIDE_DIGESTION = "HYDROGEN_PEROXIDE_DIGESTION",
  ENZYMATIC_DIGESTION = "ENZYMATIC_DIGESTION",
  FILTRATION = "FILTRATION",
  SIEVING = "SIEVING",
  COMBINED = "COMBINED"
}

/**
 * FTIR technique
 */
export enum FTIRTechnique {
  ATR = "ATR",
  TRANSMISSION = "TRANSMISSION",
  REFLECTANCE = "REFLECTANCE",
  MICRO_FTIR = "MICRO_FTIR"
}

/**
 * Additive function
 */
export enum AdditiveFunction {
  PLASTICIZER = "PLASTICIZER",
  STABILIZER = "STABILIZER",
  COLORANT = "COLORANT",
  FLAME_RETARDANT = "FLAME_RETARDANT",
  UV_ABSORBER = "UV_ABSORBER",
  FILLER = "FILLER",
  OTHER = "OTHER"
}

/**
 * Hazard level
 */
export enum HazardLevel {
  LOW = "LOW",
  MODERATE = "MODERATE",
  HIGH = "HIGH",
  SEVERE = "SEVERE"
}

/**
 * Degradation rate
 */
export enum DegradationRate {
  VERY_SLOW = "VERY_SLOW",       // > 1000 years
  SLOW = "SLOW",                 // 100-1000 years
  MODERATE = "MODERATE",         // 10-100 years
  FAST = "FAST",                 // < 10 years
  UNKNOWN = "UNKNOWN"
}

/**
 * Toxicity level
 */
export enum ToxicityLevel {
  NON_TOXIC = "NON_TOXIC",
  LOW = "LOW",
  MODERATE = "MODERATE",
  HIGH = "HIGH",
  VERY_HIGH = "VERY_HIGH"
}

/**
 * Bioaccumulation potential
 */
export enum BioaccumulationPotential {
  LOW = "LOW",
  MODERATE = "MODERATE",
  HIGH = "HIGH",
  VERY_HIGH = "VERY_HIGH"
}

/**
 * Measurement method
 */
export enum MeasurementMethod {
  MICROSCOPY_MANUAL = "MICROSCOPY_MANUAL",
  MICROSCOPY_AUTOMATED = "MICROSCOPY_AUTOMATED",
  IMAGE_ANALYSIS = "IMAGE_ANALYSIS",
  LASER_DIFFRACTION = "LASER_DIFFRACTION",
  DYNAMIC_LIGHT_SCATTERING = "DYNAMIC_LIGHT_SCATTERING"
}

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 date-time string
 */
export type ISO8601DateTime = string;

/**
 * 3D coordinate
 */
export interface Coordinate3D {
  x: number;
  y: number;
  z?: number;
}

/**
 * RGB color representation
 */
export interface RGBColor {
  r: number;  // 0-255
  g: number;  // 0-255
  b: number;  // 0-255
}

/**
 * HSV color representation
 */
export interface HSVColor {
  h: number;  // 0-360
  s: number;  // 0-100
  v: number;  // 0-100
}

/**
 * LAB color representation
 */
export interface LABColor {
  l: number;  // 0-100
  a: number;  // -128 to 127
  b: number;  // -128 to 127
}

/**
 * Image resolution
 */
export interface ImageResolution {
  width: number;
  height: number;
  unit: "pixels" | "micrometers";
}

/**
 * Bounding box for image regions
 */
export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

// ============================================================================
// Geographic Types
// ============================================================================

/**
 * Geographic location
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  depth?: number;
  siteName?: string;
  region?: string;
  country?: string;
  waterbody?: string;
  coordinateSystem: string;
  accuracy: number;
}

// ============================================================================
// Particle Types
// ============================================================================

/**
 * Particle size measurements
 */
export interface ParticleSize {
  length: number;
  width: number;
  height?: number;
  area: number;
  volume?: number;
  equivalentSphericalDiameter: number;
  sizeClass: SizeClass;
  measurementError: number;
  measurementMethod: MeasurementMethod;
}

/**
 * Particle shape characteristics
 */
export interface ParticleShape {
  primaryShape: ShapeCategory;
  aspectRatio: number;
  circularity: number;
  sphericity: number;
  roughness: number;
  perimeter: number;
  convexHullArea: number;
  solidity: number;
}

/**
 * Particle color information
 */
export interface ParticleColor {
  rgb: RGBColor;
  hsv: HSVColor;
  lab: LABColor;
  primaryColor: ColorCategory;
  transparency: TransparencyLevel;
  reflectanceSpectrum?: SpectralData;
}

/**
 * Spectral data
 */
export interface SpectralData {
  wavelengths: number[];
  values: number[];
}

/**
 * Chemical additive information
 */
export interface ChemicalAdditive {
  additiveName: string;
  casNumber: string;
  concentration?: number;
  purpose: AdditiveFunction;
  hazardLevel: HazardLevel;
}

/**
 * Polymer type information
 */
export interface PolymerType {
  polymerCode: PolymerCode;
  polymerName: string;
  chemicalFormula: string;
  casNumber?: string;
  density: number;
  meltingPoint?: number;
  glassTransitionTemp?: number;
  degradationRate: DegradationRate;
  toxicityLevel: ToxicityLevel;
  bioaccumulation: BioaccumulationPotential;
  commonSources: string[];
  typicalApplications: string[];
}

/**
 * Microplastic particle
 */
export interface MicroplasticParticle {
  particleId: string;
  sampleId: string;
  size: ParticleSize;
  shape: ParticleShape;
  color: ParticleColor;
  polymerType: PolymerCode;
  polymerConfidence: number;
  additives: ChemicalAdditive[];
  positionInSample: Coordinate3D;
  detectionMethod: DetectionMethod;
  detectedAt: ISO8601DateTime;
  detectedBy: string;
  verified: boolean;
  verifiedBy?: string;
  verificationNotes?: string;
  imageUrl?: string;
}

// ============================================================================
// Spectroscopy Types
// ============================================================================

/**
 * Raman spectrum
 */
export interface RamanSpectrum {
  spectrumId: string;
  particleId: string;
  wavenumbers: number[];
  intensities: number[];
  laserWavelength: number;
  laserPower: number;
  exposureTime: number;
  accumulations: number;
  peakPositions: number[];
  peakIntensities: number[];
  baselineCorrected: boolean;
  matchedPolymer?: PolymerCode;
  matchConfidence: number;
  spectralLibrary?: string;
  signalToNoiseRatio: number;
  spectrumQuality: QualityLevel;
}

/**
 * Functional group identification
 */
export interface FunctionalGroup {
  name: string;
  wavenumber: number;
  intensity: number;
}

/**
 * FTIR spectrum
 */
export interface FTIRSpectrum {
  spectrumId: string;
  particleId: string;
  wavenumbers: number[];
  absorbance: number[];
  transmittance?: number[];
  technique: FTIRTechnique;
  resolution: number;
  scans: number;
  functionalGroups: FunctionalGroup[];
  matchedPolymer?: PolymerCode;
  matchConfidence: number;
  signalToNoiseRatio: number;
  spectrumQuality: QualityLevel;
}

/**
 * Fluorescence data
 */
export interface FluorescenceData {
  dataId: string;
  particleId: string;
  excitationWavelength: number;
  emissionWavelength: number;
  intensity: number;
  fluorophore?: string;
  stainingProtocol?: string;
  imageUrl?: string;
  imageResolution?: ImageResolution;
}

// ============================================================================
// Sample Types
// ============================================================================

/**
 * Sample control information
 */
export interface SampleControl {
  controlType: "BLANK" | "POSITIVE" | "NEGATIVE" | "REFERENCE";
  particleCount: number;
  expectedParticleCount?: number;
  recoveryRate?: number;
  notes?: string;
}

/**
 * Microplastic sample
 */
export interface MicroplasticSample {
  sampleId: string;
  sampleName: string;
  collectionId?: string;
  externalId?: string;
  collectedAt: ISO8601DateTime;
  collectedBy: string;
  collectionMethod: SamplingMethod;
  location: GeoLocation;
  environmentType: EnvironmentType;
  sampleType: SampleType;
  sampleVolume?: number;
  sampleMass?: number;
  sampleArea?: number;
  temperature?: number;
  pH?: number;
  salinity?: number;
  turbidity?: number;
  processingDate?: ISO8601DateTime;
  processingMethod?: ProcessingMethod;
  filtrationSize?: number;
  totalParticleCount?: number;
  particleConcentration?: number;
  concentrationUnit?: ConcentrationUnit;
  dominantPolymer?: PolymerCode;
  blankControl?: SampleControl;
  recoveryRate?: number;
  qcStatus: QCStatus;
  status: "PENDING_ANALYSIS" | "ANALYZING" | "ANALYZED" | "APPROVED" | "REJECTED";
  createdAt?: ISO8601DateTime;
  updatedAt?: ISO8601DateTime;
}

// ============================================================================
// Analysis Types
// ============================================================================

/**
 * Size distribution bin
 */
export interface SizeBin {
  minSize: number;
  maxSize: number;
  count: number;
  percentage: number;
}

/**
 * Size distribution statistics
 */
export interface SizeDistribution {
  bins: SizeBin[];
  meanSize: number;
  medianSize: number;
  standardDeviation: number;
  sizeRange: [number, number];
}

/**
 * Polymer distribution entry
 */
export interface PolymerDistributionEntry {
  polymerType: PolymerCode;
  count: number;
  percentage: number;
}

/**
 * Polymer distribution
 */
export interface PolymerDistribution {
  polymers: PolymerDistributionEntry[];
}

/**
 * Shape distribution entry
 */
export interface ShapeDistributionEntry {
  shapeType: ShapeCategory;
  count: number;
  percentage: number;
}

/**
 * Shape distribution
 */
export interface ShapeDistribution {
  shapes: ShapeDistributionEntry[];
}

/**
 * Color distribution entry
 */
export interface ColorDistributionEntry {
  colorType: ColorCategory;
  count: number;
  percentage: number;
}

/**
 * Color distribution
 */
export interface ColorDistribution {
  colors: ColorDistributionEntry[];
}

/**
 * Concentration data
 */
export interface ConcentrationData {
  particlesPerUnit: number;
  unit: ConcentrationUnit;
  massPerUnit?: number;
  nanoplasticConcentration?: number;
  microplasticConcentration?: number;
  confidenceLevel: number;
  confidenceInterval: [number, number];
}

/**
 * Risk assessment
 */
export interface RiskAssessment {
  overallRisk: "LOW" | "MODERATE" | "HIGH" | "SEVERE";
  ecotoxicityRisk: number;
  bioaccumulationRisk: number;
  humanHealthRisk: number;
  ecosystemImpact: string;
  recommendations: string[];
}

/**
 * Detection result
 */
export interface DetectionResult {
  resultId: string;
  sampleId: string;
  analyzedAt: ISO8601DateTime;
  analyzedBy: string;
  analysisMethod: AnalysisMethod;
  particles: MicroplasticParticle[];
  totalParticles: number;
  sizeDistribution: SizeDistribution;
  polymerDistribution: PolymerDistribution;
  shapeDistribution: ShapeDistribution;
  colorDistribution: ColorDistribution;
  concentration: ConcentrationData;
  detectionLimit: number;
  falsePositiveRate?: number;
  falseNegativeRate?: number;
  riskAssessment?: RiskAssessment;
  qcStatus: QCStatus;
}

// ============================================================================
// Job Types
// ============================================================================

/**
 * Analysis job configuration
 */
export interface AnalysisJobConfig {
  analysisMethod: AnalysisMethod;
  techniques: DetectionMethod[];
  minParticleSize: number;
  maxParticleSize: number;
  minPolymerConfidence: number;
  priority: "LOW" | "NORMAL" | "HIGH" | "URGENT";
  notifications?: {
    email?: string;
    webhook?: string;
  };
}

/**
 * Analysis job
 */
export interface AnalysisJob {
  jobId: string;
  sampleId: string;
  status: JobStatus;
  progress: number;
  estimatedDuration?: number;
  queuePosition?: number;
  currentStage?: string;
  particlesAnalyzed?: number;
  totalParticles?: number;
  estimatedCompletion?: ISO8601DateTime;
  createdAt: ISO8601DateTime;
  startedAt?: ISO8601DateTime;
  completedAt?: ISO8601DateTime;
  errorMessage?: string;
}

// ============================================================================
// Sensor Types
// ============================================================================

/**
 * Water quality parameters
 */
export interface WaterQuality {
  temperature?: number;
  pH?: number;
  salinity?: number;
  turbidity?: number;
  dissolvedOxygen?: number;
  conductivity?: number;
}

/**
 * Sensor reading
 */
export interface SensorReading {
  readingId?: string;
  sensorId: string;
  timestamp: ISO8601DateTime;
  location: GeoLocation;
  particleCount: number;
  sizeDistribution?: {
    nano: number;
    micro: number;
    meso?: number;
  };
  waterQuality?: WaterQuality;
  status: "VALID" | "QUESTIONABLE" | "INVALID";
}

/**
 * Sensor metadata
 */
export interface SensorMetadata {
  sensorId: string;
  sensorName: string;
  sensorType: string;
  manufacturer?: string;
  model?: string;
  deploymentDate: ISO8601DateTime;
  location: GeoLocation;
  samplingInterval: number;
  calibrationDate?: ISO8601DateTime;
  status: "ACTIVE" | "INACTIVE" | "MAINTENANCE" | "DECOMMISSIONED";
}

// ============================================================================
// Report Types
// ============================================================================

/**
 * Report configuration
 */
export interface ReportConfig {
  reportType: "SAMPLE_ANALYSIS" | "COLLECTION_SUMMARY" | "MONITORING_REPORT" | "REGULATORY_COMPLIANCE";
  sampleIds?: string[];
  collectionId?: string;
  format: "PDF" | "HTML" | "DOCX" | "JSON";
  includeCharts: boolean;
  includeRawData: boolean;
  language: "en" | "ko" | "es" | "fr" | "de" | "ja" | "zh";
}

/**
 * Report metadata
 */
export interface ReportMetadata {
  reportId: string;
  reportType: string;
  generatedAt: ISO8601DateTime;
  generatedBy: string;
  format: string;
  status: "GENERATING" | "COMPLETED" | "FAILED";
  downloadUrl?: string;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  limit: number;
  offset: number;
  links: {
    next?: string;
    prev?: string;
  };
}

/**
 * API error
 */
export interface APIError {
  code: string;
  message: string;
  details?: string;
  timestamp: ISO8601DateTime;
  requestId?: string;
}

/**
 * API response
 */
export interface APIResponse<T> {
  data?: T;
  error?: APIError;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface SDKConfig {
  apiKey?: string;
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;
  enableWebSocket?: boolean;
  environment?: "production" | "staging" | "development";
}

/**
 * Webhook configuration
 */
export interface WebhookConfig {
  url: string;
  events: string[];
  secret?: string;
  headers?: Record<string, string>;
}

// ============================================================================
// Export All
// ============================================================================

export * from "./types";
