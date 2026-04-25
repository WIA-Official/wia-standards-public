/**
 * WIA-SEMI-018 Semiconductor Material Standard - Type Definitions
 * @packageDocumentation
 */

/**
 * Silicon wafer specifications according to WIA-SEMI-018
 */
export interface WaferSpecification {
  /** Wafer diameter in millimeters (e.g., 300) */
  diameter: number;

  /** Wafer thickness in micrometers (e.g., 775) */
  thickness: number;

  /** Total Thickness Variation in micrometers */
  ttv: number;

  /** Bow in micrometers */
  bow: number;

  /** Warp in micrometers */
  warp: number;

  /** Defect density in defects per square centimeter */
  defectDensity: number;

  /** Surface roughness Ra in nanometers */
  surfaceRoughness: number;

  /** Silicon purity in 9s (e.g., 11 for 11-9s) */
  purity: number;

  /** Crystal orientation (e.g., "<100>", "<111>") */
  orientation: string;

  /** Resistivity in ohm-centimeters */
  resistivity: number;
}

/**
 * Photoresist material specifications
 */
export interface PhotoresistSpecification {
  /** Resist type (e.g., "EUV", "ArF", "KrF") */
  type: "EUV" | "ArF" | "KrF" | "i-line";

  /** Sensitivity in mJ/cm² */
  sensitivity: number;

  /** Line Edge Roughness (3σ) in nanometers */
  ler: number;

  /** Contrast value */
  contrast: number;

  /** Resolution capability in nanometers */
  resolution: number;

  /** Defect density in defects per square centimeter */
  defectDensity: number;

  /** Viscosity in centipoise */
  viscosity: number;

  /** Particle count per milliliter (>0.2 µm) */
  particleCount: number;

  /** Shelf life in months */
  shelfLife: number;
}

/**
 * Specialty gas specifications
 */
export interface GasSpecification {
  /** Gas name (e.g., "Phosphine", "Silane", "Nitrogen") */
  name: string;

  /** Chemical formula (e.g., "PH₃", "SiH₄", "N₂") */
  formula: string;

  /** Purity in nines (e.g., 6 for 99.9999%) */
  purity: number;

  /** Maximum impurity levels in ppm */
  impurities: Record<string, number>;

  /** UN hazard number */
  unNumber?: string;

  /** TLV-TWA in ppm (Threshold Limit Value - Time Weighted Average) */
  tlvTwa?: number;

  /** Whether the gas is pyrophoric */
  pyrophoric: boolean;

  /** Whether the gas is toxic */
  toxic: boolean;
}

/**
 * Quality test result
 */
export interface TestResult {
  /** Test parameter name */
  parameter: string;

  /** Measured value */
  measuredValue: number;

  /** Specification limit (lower) */
  lowerLimit: number;

  /** Specification limit (upper) */
  upperLimit: number;

  /** Pass/fail status */
  passed: boolean;

  /** Test method used */
  testMethod: string;

  /** Test date */
  testDate: Date;

  /** Operator ID */
  operatorId?: string;
}

/**
 * Material lot information
 */
export interface MaterialLot {
  /** Unique lot identifier */
  lotNumber: string;

  /** Material type */
  materialType: "wafer" | "photoresist" | "gas" | "chemical";

  /** Supplier name */
  supplier: string;

  /** Manufacturing date */
  manufacturingDate: Date;

  /** Receipt date */
  receiptDate: Date;

  /** Quantity */
  quantity: number;

  /** Quantity unit */
  quantityUnit: string;

  /** IQC status */
  iqcStatus: "pending" | "passed" | "failed" | "conditional";

  /** Test results */
  testResults: TestResult[];

  /** Expiration date (if applicable) */
  expirationDate?: Date;
}

/**
 * Supplier information
 */
export interface Supplier {
  /** Supplier name */
  name: string;

  /** Supplier ID */
  supplierId: string;

  /** ISO 9001 certified */
  iso9001Certified: boolean;

  /** Qualification status */
  qualified: boolean;

  /** Performance scorecard */
  scorecard: SupplierScorecard;

  /** Last audit date */
  lastAuditDate?: Date;
}

/**
 * Supplier performance scorecard
 */
export interface SupplierScorecard {
  /** Overall score (0-100) */
  overallScore: number;

  /** Quality score (0-40) */
  qualityScore: number;

  /** Delivery score (0-30) */
  deliveryScore: number;

  /** Service score (0-20) */
  serviceScore: number;

  /** Cost score (0-10) */
  costScore: number;

  /** Defect rate in ppm */
  defectRatePpm: number;

  /** On-time delivery percentage */
  ontimeDeliveryPercent: number;
}

/**
 * Statistical Process Control data
 */
export interface SPCData {
  /** Parameter name */
  parameter: string;

  /** Mean value */
  mean: number;

  /** Standard deviation */
  stdDev: number;

  /** Upper Control Limit */
  ucl: number;

  /** Lower Control Limit */
  lcl: number;

  /** Cpk value */
  cpk: number;

  /** In-control status */
  inControl: boolean;

  /** Data points */
  dataPoints: number[];

  /** Timestamps */
  timestamps: Date[];
}

/**
 * Defect information
 */
export interface Defect {
  /** Defect ID */
  defectId: string;

  /** Defect type */
  type: "particle" | "scratch" | "pit" | "crystal-defect" | "contamination";

  /** Size in micrometers */
  size: number;

  /** Location (x, y) in millimeters from wafer center */
  location: { x: number; y: number };

  /** Detection method */
  detectionMethod: string;

  /** Severity */
  severity: "critical" | "major" | "minor";
}

/**
 * Material traceability record
 */
export interface TraceabilityRecord {
  /** Material lot number */
  lotNumber: string;

  /** Forward tracing: devices produced */
  devicesProduced: string[];

  /** Backward tracing: source materials */
  sourceMaterials: string[];

  /** Process steps */
  processSteps: ProcessStep[];

  /** Quality events */
  qualityEvents: QualityEvent[];
}

/**
 * Process step information
 */
export interface ProcessStep {
  /** Step name */
  stepName: string;

  /** Tool ID */
  toolId: string;

  /** Recipe name */
  recipeName: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Operator ID */
  operatorId: string;
}

/**
 * Quality event information
 */
export interface QualityEvent {
  /** Event ID */
  eventId: string;

  /** Event type */
  type: "ncr" | "scar" | "deviation" | "improvement";

  /** Description */
  description: string;

  /** Date */
  date: Date;

  /** Resolution status */
  status: "open" | "in-progress" | "closed";

  /** Root cause */
  rootCause?: string;

  /** Corrective action */
  correctiveAction?: string;
}

/**
 * Configuration options for SDK
 */
export interface SDKConfig {
  /** API endpoint */
  apiEndpoint?: string;

  /** API key for authentication */
  apiKey?: string;

  /** Enable debug logging */
  debug?: boolean;

  /** Timeout in milliseconds */
  timeout?: number;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Whether validation passed */
  valid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];
}
