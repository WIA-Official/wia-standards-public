/**
 * WIA-IND-029: Additive Manufacturing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 3D position vector (mm)
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * 2D position (mm)
 */
export interface Vector2D {
  x: number;
  y: number;
}

/**
 * Timestamp with timezone
 */
export type Timestamp = Date | string;

/**
 * Print farm identifier
 */
export type FarmId = string;

/**
 * Printer identifier
 */
export type PrinterId = string;

/**
 * Job identifier
 */
export type JobId = string;

/**
 * Model identifier
 */
export type ModelId = string;

/**
 * Material spool identifier
 */
export type SpoolId = string;

// ============================================================================
// 3D Printing Technologies
// ============================================================================

/**
 * Additive manufacturing technology types
 */
export type PrintingTechnology =
  | 'FDM'           // Fused Deposition Modeling
  | 'FFF'           // Fused Filament Fabrication
  | 'SLA'           // Stereolithography
  | 'DLP'           // Digital Light Processing
  | 'MSLA'          // Masked Stereolithography
  | 'SLS'           // Selective Laser Sintering
  | 'MJF'           // Multi Jet Fusion
  | 'DMLS'          // Direct Metal Laser Sintering
  | 'SLM'           // Selective Laser Melting
  | 'EBM'           // Electron Beam Melting
  | 'PolyJet'       // Material Jetting
  | 'MultiJet'      // MultiJet Printing
  | 'BinderJet'     // Binder Jetting
  | 'DED'           // Direct Energy Deposition
  | 'LENS';         // Laser Engineered Net Shaping

/**
 * Material categories
 */
export type MaterialCategory =
  | 'thermoplastic'
  | 'photopolymer'
  | 'powder-nylon'
  | 'powder-metal'
  | 'resin'
  | 'composite'
  | 'ceramic'
  | 'concrete'
  | 'bio-ink';

/**
 * Common thermoplastic materials
 */
export type ThermoplasticMaterial =
  | 'PLA'
  | 'ABS'
  | 'PETG'
  | 'Nylon'
  | 'PA12'
  | 'TPU'
  | 'PC'
  | 'PEEK'
  | 'ULTEM'
  | 'ASA'
  | 'PP'
  | 'PVA'
  | 'HIPS';

/**
 * Metal materials for DMLS/SLM
 */
export type MetalMaterial =
  | '316L-stainless'
  | '17-4PH-stainless'
  | 'Ti64-titanium'
  | 'AlSi10Mg-aluminum'
  | 'Inconel-625'
  | 'Inconel-718'
  | 'CoCr-cobalt-chrome'
  | 'H13-tool-steel'
  | 'M2-tool-steel';

// ============================================================================
// File Formats
// ============================================================================

/**
 * Supported 3D file formats
 */
export type FileFormat = 'STL' | '3MF' | 'OBJ' | 'AMF' | 'STEP' | 'GCODE';

/**
 * Unit systems
 */
export type Units = 'mm' | 'cm' | 'inches';

/**
 * 3D Model upload configuration
 */
export interface ModelUpload {
  filePath: string;
  format: FileFormat;
  units?: Units;
  name?: string;
  description?: string;
}

/**
 * 3D Model metadata
 */
export interface Model {
  id: ModelId;
  name: string;
  format: FileFormat;
  units: Units;
  dimensions: Vector3D;
  volume: number;        // mm³
  surfaceArea: number;   // mm²
  triangleCount: number;
  uploadedAt: Timestamp;
  fileSize: number;      // bytes
  thumbnailUrl?: string;
}

// ============================================================================
// Slicing Configuration
// ============================================================================

/**
 * Infill patterns
 */
export type InfillPattern =
  | 'grid'
  | 'lines'
  | 'triangles'
  | 'tri-hexagon'
  | 'gyroid'
  | 'honeycomb'
  | 'cubic'
  | 'octet'
  | 'concentric'
  | 'hilbert-curve'
  | 'adaptive';

/**
 * Support structure types
 */
export type SupportType =
  | 'none'
  | 'standard'
  | 'tree'
  | 'tree-ai'
  | 'organic';

/**
 * Retraction settings
 */
export interface RetractionSettings {
  distance: number;        // mm
  speed: number;           // mm/s
  minimumDistance?: number; // mm
  zhop?: number;           // mm
}

/**
 * Slicing profile
 */
export interface SlicingProfile {
  technology: PrintingTechnology;
  layerHeight: number;              // mm
  wallThickness: number;            // mm
  wallLineCount?: number;
  infillDensity: number;            // percentage (0-100)
  infillPattern: InfillPattern;
  supportType: SupportType;
  supportDensity?: number;          // percentage
  supportAngle?: number;            // degrees
  brimWidth?: number;               // mm
  raftLayers?: number;
  printSpeed: number;               // mm/s
  travelSpeed: number;              // mm/s
  retraction?: RetractionSettings;
  enableIroning?: boolean;
  enableAdaptiveLayers?: boolean;
  minLayerHeight?: number;
  maxLayerHeight?: number;
}

/**
 * Sliced model result
 */
export interface SlicedModel {
  id: string;
  modelId: ModelId;
  profile: SlicingProfile;
  material: string;
  printer: PrinterId;
  estimatedTime: number;       // seconds
  materialUsage: number;       // grams
  supportMaterial?: number;    // grams
  layerCount: number;
  gcodeFileUrl: string;
  previewImageUrl?: string;
  slicedAt: Timestamp;
}

// ============================================================================
// Material Specifications
// ============================================================================

/**
 * Temperature range
 */
export interface TemperatureRange {
  min: number;
  max: number;
  optimal: number;
}

/**
 * Material properties
 */
export interface MaterialProperties {
  printTemperature: TemperatureRange;  // °C
  bedTemperature?: TemperatureRange;   // °C
  density: number;                      // g/cm³
  tensileStrength?: number;             // MPa
  elongationAtBreak?: number;           // percentage
  flexuralModulus?: number;             // MPa
  heatDeflectionTemp?: number;          // °C
  impactStrength?: number;              // kJ/m²
  printSpeed?: TemperatureRange;        // mm/s
}

/**
 * Material characteristics
 */
export interface MaterialCharacteristics {
  biodegradable?: boolean;
  foodSafe?: boolean;
  warpResistance?: 'low' | 'medium' | 'high';
  layerAdhesion?: 'poor' | 'good' | 'excellent';
  bridging?: 'poor' | 'good' | 'excellent';
  overhangAngle?: number;  // degrees
  moistureSensitive?: boolean;
  enclosureRequired?: boolean;
  recyclable?: boolean;
}

/**
 * Material specification
 */
export interface Material {
  name: string;
  category: MaterialCategory;
  properties: MaterialProperties;
  characteristics?: MaterialCharacteristics;
  applications?: string[];
  dryingRequired?: {
    temperature: number;  // °C
    time: number;         // hours
    humidity: string;     // target humidity
  };
}

/**
 * Material spool/inventory
 */
export interface MaterialSpool {
  spoolId: SpoolId;
  material: string;
  brand: string;
  color: string;
  weight: {
    total: number;       // grams
    remaining: number;   // grams
    unit: 'grams' | 'kg';
  };
  properties: {
    diameter: number;    // mm (1.75 or 2.85)
    tolerance: number;   // mm
  };
  location?: string;
  status: 'available' | 'in-use' | 'low-stock' | 'empty';
  printerId?: PrinterId;
  expiryDate?: string;
  lastUsed?: Timestamp;
  rfidTag?: string;
}

// ============================================================================
// Printer Configuration
// ============================================================================

/**
 * Build volume
 */
export interface BuildVolume {
  x: number;  // mm
  y: number;  // mm
  z: number;  // mm
}

/**
 * Printer status
 */
export type PrinterStatus =
  | 'idle'
  | 'printing'
  | 'paused'
  | 'error'
  | 'maintenance'
  | 'offline';

/**
 * Printer configuration
 */
export interface Printer {
  printerId: PrinterId;
  name: string;
  technology: PrintingTechnology;
  manufacturer: string;
  model: string;
  buildVolume: BuildVolume;
  nozzleDiameter: number[];        // mm
  maxHotendTemp: number;           // °C
  maxBedTemp: number;              // °C
  hasChamber?: boolean;
  maxChamberTemp?: number;         // °C
  numberOfExtruders: number;
  capabilities: {
    multiMaterial?: boolean;
    multiColor?: boolean;
    enclosure?: boolean;
    autoLeveling?: boolean;
    filamentSensor?: boolean;
    camera?: boolean;
  };
  status: PrinterStatus;
  currentJob?: JobId;
  location?: string;
  ipAddress?: string;
  apiEndpoint?: string;
}

// ============================================================================
// Print Job Management
// ============================================================================

/**
 * Job priority levels
 */
export type JobPriority = 'low' | 'normal' | 'high' | 'urgent';

/**
 * Job status
 */
export type JobStatus =
  | 'created'
  | 'queued'
  | 'running'
  | 'paused'
  | 'completed'
  | 'failed'
  | 'cancelled';

/**
 * Post-processing steps
 */
export type PostProcessingStep =
  | 'support-removal'
  | 'surface-finish'
  | 'sanding'
  | 'vapor-smooth'
  | 'painting'
  | 'annealing'
  | 'heat-treatment'
  | 'coating';

/**
 * Print job configuration
 */
export interface PrintJobConfig {
  slicedModelId: string;
  printerId: PrinterId;
  priority?: JobPriority;
  copies?: number;
  materialSpool: SpoolId;
  postProcessing?: PostProcessingStep[];
  notifyOnComplete?: boolean;
  notificationEmail?: string;
}

/**
 * Print job
 */
export interface PrintJob {
  jobId: JobId;
  name: string;
  status: JobStatus;
  priority: JobPriority;
  modelId: ModelId;
  slicedFileId: string;
  printerId: PrinterId;
  materialSpool: SpoolId;
  copies: number;
  currentCopy: number;
  estimatedTime: number;       // seconds
  elapsedTime: number;         // seconds
  estimatedMaterial: number;   // grams
  usedMaterial: number;        // grams
  progress: number;            // percentage
  currentLayer: number;
  totalLayers: number;
  queuePosition?: number;
  createdAt: Timestamp;
  startedAt?: Timestamp;
  completedAt?: Timestamp;
  estimatedCompletion?: Timestamp;
  errorMessage?: string;
}

/**
 * Print telemetry (real-time data)
 */
export interface PrintTelemetry {
  jobId: JobId;
  printerId: PrinterId;
  timestamp: Timestamp;
  temperatures: {
    hotend: { current: number; target: number };
    bed: { current: number; target: number };
    chamber?: { current: number; target: number };
  };
  position: {
    x: number;
    y: number;
    z: number;
    e: number;  // extruder
  };
  speeds: {
    print: number;
    fan: number;
  };
  progress: {
    percentage: number;
    layer: number;
    totalLayers: number;
  };
  estimates: {
    timeRemaining: number;      // seconds
    materialRemaining: number;  // grams
  };
}

// ============================================================================
// Quality Assurance
// ============================================================================

/**
 * Inspection types
 */
export type InspectionType =
  | 'dimensional'
  | 'visual'
  | 'mechanical'
  | 'surface-finish';

/**
 * Dimensional measurement
 */
export interface DimensionalMeasurement {
  dimension: string;
  nominal: number;
  tolerance: number;
  measured?: number;
  deviation?: number;
  status?: 'pass' | 'fail';
}

/**
 * Visual defect types
 */
export type DefectType =
  | 'warping'
  | 'layer-shift'
  | 'stringing'
  | 'gaps'
  | 'under-extrusion'
  | 'over-extrusion'
  | 'poor-adhesion';

/**
 * Visual defect detection
 */
export interface VisualDefect {
  type: DefectType;
  detected: boolean;
  severity?: number;  // 0-10
  location?: Vector2D;
  acceptable?: boolean;
}

/**
 * Quality inspection configuration
 */
export interface InspectionConfig {
  jobId: JobId;
  partNumber?: number;
  inspectionType: InspectionType;
  measurements?: DimensionalMeasurement[];
  visualChecks?: DefectType[];
  mechanicalTests?: string[];
}

/**
 * Inspection result
 */
export interface InspectionResult {
  inspectionId: string;
  jobId: JobId;
  inspectionType: InspectionType;
  timestamp: Timestamp;
  inspector?: string;
  passed: boolean;
  score?: number;
  measurements?: DimensionalMeasurement[];
  defects?: VisualDefect[];
  mechanicalResults?: any;
  notes?: string;
  images?: string[];
}

// ============================================================================
// Multi-Material Printing
// ============================================================================

/**
 * Material assignment for multi-material
 */
export interface MaterialAssignment {
  extruder: number;
  material: string;
  color: string;
  regions?: number[];
  infillDensity?: number;
  modifier?: 'color-change' | 'support' | 'interface';
}

/**
 * Multi-material job configuration
 */
export interface MultiMaterialConfig {
  modelId: ModelId;
  materials: MaterialAssignment[];
  printerId: PrinterId;
  wipeDistance?: number;
  purgeVolume?: number;
}

// ============================================================================
// Print Farm Management
// ============================================================================

/**
 * Print farm configuration
 */
export interface PrintFarm {
  farmId: FarmId;
  name: string;
  location: string;
  printers: Printer[];
  totalCapacity: number;
  activePrinters: number;
  queuedJobs: number;
  completedToday: number;
  utilization: number;  // percentage
  materialInventory: MaterialSpool[];
}

/**
 * Load balancing configuration
 */
export interface LoadBalancingConfig {
  algorithm: 'round-robin' | 'weighted-round-robin' | 'least-loaded' | 'ai-optimize';
  factors?: {
    printerCapability?: number;
    queueLength?: number;
    materialAvailability?: number;
    estimatedCompletion?: number;
  };
  constraints?: {
    technologyMatch?: boolean;
    materialMatch?: boolean;
    buildVolumeCheck?: boolean;
    maintenanceWindows?: boolean;
  };
}

/**
 * Farm optimization result
 */
export interface FarmOptimization {
  schedule: Array<{
    jobId: JobId;
    printerId: PrinterId;
    startTime: Timestamp;
    endTime: Timestamp;
  }>;
  makespan: number;        // hours
  utilization: number;     // percentage
  estimatedCompletion: Timestamp;
}

// ============================================================================
// Certification
// ============================================================================

/**
 * Quality standard
 */
export type QualityStandard =
  | 'ISO-9001'
  | 'AS9100'
  | 'ISO-13485'
  | 'FDA-21CFR'
  | 'NADCAP';

/**
 * Certification configuration
 */
export interface CertificationConfig {
  jobId: JobId;
  inspectionId: string;
  standard: QualityStandard;
  inspector: string;
  traceability?: {
    materialBatch?: string;
    machineId?: string;
    operatorId?: string;
    environmentalConditions?: {
      temperature: number;
      humidity: number;
    };
  };
}

/**
 * Certification record
 */
export interface Certification {
  certId: string;
  jobId: JobId;
  standard: QualityStandard;
  inspector: string;
  date: Timestamp;
  passed: boolean;
  traceability: any;
  testResults: any;
  certificateUrl?: string;
  blockchainHash?: string;
  status: 'pending' | 'certified' | 'rejected';
}

// ============================================================================
// AI Features
// ============================================================================

/**
 * AI optimization objectives
 */
export interface OptimizationObjectives {
  minimizePrintTime?: number;     // weight 0-1
  minimizeMaterial?: number;      // weight 0-1
  maximizeStrength?: number;      // weight 0-1
  minimizeSupport?: number;       // weight 0-1
}

/**
 * AI optimization constraints
 */
export interface OptimizationConstraints {
  maxPrintTime?: number;          // seconds
  minInfillDensity?: number;      // percentage
  maxSupportMaterial?: number;    // grams
  minStrength?: number;           // MPa
}

/**
 * AI slicing optimization
 */
export interface AISlicingOptimization {
  modelId: ModelId;
  objectives: OptimizationObjectives;
  constraints?: OptimizationConstraints;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration
 */
export interface SDKConfig {
  printFarmId: FarmId;
  apiEndpoint?: string;
  apiKey?: string;
  defaultPrinter?: PrinterId;
  defaultMaterial?: string;
  defaultProfile?: string;
  timeout?: number;
  retries?: number;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event types for real-time updates
 */
export type EventType =
  | 'job.created'
  | 'job.queued'
  | 'job.started'
  | 'job.progress'
  | 'job.paused'
  | 'job.resumed'
  | 'job.completed'
  | 'job.failed'
  | 'job.cancelled'
  | 'printer.status'
  | 'printer.error'
  | 'material.low'
  | 'quality.alert';

/**
 * Event payload
 */
export interface Event {
  type: EventType;
  timestamp: Timestamp;
  data: any;
}

/**
 * 弘益人間 (Benefit All Humanity)
 */
