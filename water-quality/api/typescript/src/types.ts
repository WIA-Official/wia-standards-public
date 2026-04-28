/**
 * WIA Water Quality Standard Types
 * Comprehensive water quality monitoring and compliance types
 *
 * @module @wia/water-quality
 * @version 1.0.0
 */

// ============================================================================
// Water Quality Parameters
// ============================================================================

/**
 * Physical water quality parameters
 */
export interface PhysicalParameters {
  /** Temperature in degrees Celsius */
  temperature: number;
  /** Turbidity in NTU (Nephelometric Turbidity Units) */
  turbidity: number;
  /** Color in Pt-Co units */
  color?: number;
  /** Total Suspended Solids in mg/L */
  totalSuspendedSolids?: number;
  /** Total Dissolved Solids in mg/L */
  totalDissolvedSolids?: number;
  /** Conductivity in μS/cm */
  conductivity?: number;
}

/**
 * Chemical water quality parameters
 */
export interface ChemicalParameters {
  /** pH value (0-14) */
  pH: number;
  /** Dissolved Oxygen in mg/L */
  dissolvedOxygen: number;
  /** Biochemical Oxygen Demand in mg/L */
  BOD?: number;
  /** Chemical Oxygen Demand in mg/L */
  COD?: number;
  /** Total Organic Carbon in mg/L */
  TOC?: number;
  /** Alkalinity in mg/L as CaCO3 */
  alkalinity?: number;
  /** Hardness in mg/L as CaCO3 */
  hardness?: number;
  /** Chlorine (free) in mg/L */
  freeChlorine?: number;
  /** Chlorine (total) in mg/L */
  totalChlorine?: number;
  /** Ammonia as N in mg/L */
  ammonia?: number;
  /** Nitrate as N in mg/L */
  nitrate?: number;
  /** Nitrite as N in mg/L */
  nitrite?: number;
  /** Phosphate as P in mg/L */
  phosphate?: number;
  /** Sulfate in mg/L */
  sulfate?: number;
  /** Fluoride in mg/L */
  fluoride?: number;
}

/**
 * Biological water quality parameters
 */
export interface BiologicalParameters {
  /** Total Coliform in MPN/100mL or CFU/100mL */
  totalColiform?: number;
  /** Fecal Coliform in MPN/100mL or CFU/100mL */
  fecalColiform?: number;
  /** E. coli in MPN/100mL or CFU/100mL */
  eColi?: number;
  /** Enterococci in MPN/100mL or CFU/100mL */
  enterococci?: number;
  /** Algae count in cells/mL */
  algaeCount?: number;
  /** Chlorophyll-a in μg/L */
  chlorophyllA?: number;
}

/**
 * Heavy metals and trace elements
 */
export interface MetalParameters {
  /** Lead in mg/L */
  lead?: number;
  /** Copper in mg/L */
  copper?: number;
  /** Zinc in mg/L */
  zinc?: number;
  /** Iron in mg/L */
  iron?: number;
  /** Manganese in mg/L */
  manganese?: number;
  /** Arsenic in mg/L */
  arsenic?: number;
  /** Mercury in mg/L */
  mercury?: number;
  /** Cadmium in mg/L */
  cadmium?: number;
  /** Chromium in mg/L */
  chromium?: number;
  /** Nickel in mg/L */
  nickel?: number;
}

// ============================================================================
// Sampling and Monitoring
// ============================================================================

/**
 * Water sample types
 */
export type SampleType =
  | 'grab'           // Single point-in-time sample
  | 'composite'      // Combined samples over time
  | 'continuous'     // Real-time monitoring
  | 'integrated';    // Depth or width integrated

/**
 * Water source types
 */
export type WaterSourceType =
  | 'surface'        // Rivers, lakes, reservoirs
  | 'groundwater'    // Wells, aquifers
  | 'drinking'       // Treated drinking water
  | 'wastewater'     // Municipal or industrial wastewater
  | 'recreational'   // Swimming pools, beaches
  | 'industrial'     // Process water
  | 'agricultural';  // Irrigation water

/**
 * Sample location details
 */
export interface SampleLocation {
  /** Unique station identifier */
  stationId: string;
  /** Station name */
  stationName: string;
  /** Latitude */
  latitude: number;
  /** Longitude */
  longitude: number;
  /** Water body name */
  waterBody?: string;
  /** Source type */
  sourceType: WaterSourceType;
  /** Sampling depth in meters */
  depth?: number;
}

/**
 * Complete water quality sample
 */
export interface WaterSample {
  /** Unique sample identifier */
  sampleId: string;
  /** Sample collection timestamp */
  timestamp: Date;
  /** Sample location */
  location: SampleLocation;
  /** Sample type */
  sampleType: SampleType;
  /** Physical parameters */
  physical: PhysicalParameters;
  /** Chemical parameters */
  chemical: ChemicalParameters;
  /** Biological parameters */
  biological?: BiologicalParameters;
  /** Metal parameters */
  metals?: MetalParameters;
  /** Additional notes */
  notes?: string;
  /** Sampled by (person/organization) */
  sampledBy?: string;
  /** Lab analyzed by */
  analyzedBy?: string;
}

// ============================================================================
// Contaminants and Compliance
// ============================================================================

/**
 * Contaminant categories
 */
export type ContaminantCategory =
  | 'microbiological'
  | 'inorganic'
  | 'organic'
  | 'radionuclides'
  | 'disinfection-byproducts';

/**
 * Contaminant detection
 */
export interface Contaminant {
  /** Contaminant name */
  name: string;
  /** Category */
  category: ContaminantCategory;
  /** Detected concentration */
  concentration: number;
  /** Unit of measurement */
  unit: string;
  /** Detection method */
  method?: string;
  /** Detection limit */
  detectionLimit?: number;
}

/**
 * Regulatory compliance standards
 */
export type ComplianceStandard =
  | 'WHO'           // World Health Organization
  | 'EPA'           // US Environmental Protection Agency
  | 'EU'            // European Union
  | 'ISO'           // International Organization for Standardization
  | 'national'      // National standards
  | 'local';        // Local/municipal standards

/**
 * Compliance threshold
 */
export interface ComplianceThreshold {
  /** Parameter name */
  parameter: string;
  /** Maximum Contaminant Level (MCL) */
  maxLevel?: number;
  /** Minimum level (e.g., for DO) */
  minLevel?: number;
  /** Optimal range */
  optimalRange?: [number, number];
  /** Unit of measurement */
  unit: string;
  /** Compliance standard */
  standard: ComplianceStandard;
  /** Action level (triggers response) */
  actionLevel?: number;
}

/**
 * Compliance check result
 */
export interface ComplianceResult {
  /** Parameter checked */
  parameter: string;
  /** Measured value */
  measuredValue: number;
  /** Threshold applied */
  threshold: ComplianceThreshold;
  /** Is compliant */
  isCompliant: boolean;
  /** Violation severity if non-compliant */
  severity?: 'minor' | 'major' | 'critical';
  /** Recommended action */
  action?: string;
}

// ============================================================================
// Treatment and Monitoring
// ============================================================================

/**
 * Water treatment process types
 */
export type TreatmentProcessType =
  | 'coagulation'
  | 'flocculation'
  | 'sedimentation'
  | 'filtration'
  | 'disinfection'
  | 'aeration'
  | 'softening'
  | 'reverse-osmosis'
  | 'ion-exchange'
  | 'activated-carbon';

/**
 * Treatment process monitoring
 */
export interface TreatmentProcess {
  /** Process identifier */
  processId: string;
  /** Process type */
  processType: TreatmentProcessType;
  /** Inlet sample */
  inlet: Partial<WaterSample>;
  /** Outlet sample */
  outlet: Partial<WaterSample>;
  /** Process efficiency (%) */
  efficiency?: number;
  /** Operating parameters */
  operatingParams?: Record<string, any>;
  /** Process timestamp */
  timestamp: Date;
}

/**
 * Monitoring station configuration
 */
export interface MonitoringStation {
  /** Station identifier */
  stationId: string;
  /** Station name */
  name: string;
  /** Location details */
  location: SampleLocation;
  /** Parameters monitored */
  parametersMonitored: string[];
  /** Sampling frequency */
  samplingFrequency: 'continuous' | 'hourly' | 'daily' | 'weekly' | 'monthly';
  /** Is active */
  isActive: boolean;
  /** Installation date */
  installDate?: Date;
  /** Last maintenance date */
  lastMaintenance?: Date;
  /** Equipment details */
  equipment?: string[];
}

// ============================================================================
// Events and Alerts
// ============================================================================

/**
 * Water quality event types
 */
export type WaterQualityEventType =
  | 'sample-recorded'
  | 'compliance-violation'
  | 'contaminant-detected'
  | 'treatment-completed'
  | 'alert-triggered'
  | 'station-offline';

/**
 * Water quality event
 */
export interface WaterQualityEvent {
  /** Event type */
  type: WaterQualityEventType;
  /** Event timestamp */
  timestamp: Date;
  /** Related sample ID */
  sampleId?: string;
  /** Related station ID */
  stationId?: string;
  /** Event data */
  data: any;
  /** Severity level */
  severity?: 'info' | 'warning' | 'critical';
}

/**
 * Event handler function
 */
export type WaterQualityEventHandler = (event: WaterQualityEvent) => void;
