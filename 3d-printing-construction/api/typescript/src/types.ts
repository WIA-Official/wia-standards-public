/**
 * WIA-CITY-008: 3D Printing Construction Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
  elevation_m?: number;
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  elevation_m?: number;
  address: string;
  city: string;
  state?: string;
  country: string;
  postalCode?: string;
}

// ============================================================================
// Printer Types
// ============================================================================

/**
 * 3D Printer types for construction
 */
export enum PrinterType {
  GANTRY = 'GANTRY',                    // 갠트리 (고정 레일 시스템)
  ROBOTIC_ARM = 'ROBOTIC_ARM',          // 로봇 암
  WASP = 'WASP',                        // WASP 델타 프린터
  DELTA = 'DELTA',                      // 델타 프린터
  CRANE = 'CRANE',                      // 크레인 시스템
  MOBILE = 'MOBILE',                    // 이동식
}

/**
 * Printer manufacturer
 */
export enum PrinterManufacturer {
  APIS_COR = 'APIS_COR',                // 러시아
  ICON = 'ICON',                        // 미국
  COBOD = 'COBOD',                      // 덴마크
  WASP = 'WASP',                        // 이탈리아
  MIGHTY_BUILDINGS = 'MIGHTY_BUILDINGS', // 미국
  PERI = 'PERI',                        // 독일
  XIAOFANG = 'XIAOFANG',                // 중국
  CONTOUR_CRAFTING = 'CONTOUR_CRAFTING', // 미국
}

/**
 * 3D Printer specification
 */
export interface PrinterSpec {
  printerId: string;
  model: string;
  manufacturer: PrinterManufacturer;
  type: PrinterType;

  // Build volume
  buildVolume: {
    width_m: number;
    depth_m: number;
    height_m: number;
  };

  // Print parameters
  maxPrintSpeed_mm_s: number;
  layerHeight_mm: number;
  nozzleDiameter_mm: number;
  extrusionRate_kg_h: number;

  // Precision
  positioning_accuracy_mm: number;
  repeatability_mm: number;

  // Power & mobility
  powerRequirement_kW: number;
  mobile: boolean;

  // Status
  status: 'OPERATIONAL' | 'MAINTENANCE' | 'OFFLINE';
  lastCalibration?: Timestamp;
}

// ============================================================================
// Materials
// ============================================================================

/**
 * Construction material types
 */
export enum MaterialType {
  CONCRETE = 'CONCRETE',                // 콘크리트
  GEOPOLYMER = 'GEOPOLYMER',            // 지오폴리머
  CLAY = 'CLAY',                        // 점토
  MORTAR = 'MORTAR',                    // 모르타르
  FIBER_REINFORCED = 'FIBER_REINFORCED', // 섬유 보강
  RECYCLED = 'RECYCLED',                // 재활용 재료
  COMPOSITE = 'COMPOSITE',              // 복합 재료
}

/**
 * Material properties
 */
export interface MaterialProperties {
  materialId: string;
  name: string;
  type: MaterialType;

  // Composition
  composition: {
    cement_percent?: number;
    aggregate_percent?: number;
    water_percent?: number;
    additives?: Array<{
      name: string;
      percent: number;
    }>;
  };

  // Physical properties
  density_kg_m3: number;
  viscosity_Pa_s: number;
  workability_time_min: number;

  // Strength (at 28 days)
  compressive_strength_MPa: number;
  tensile_strength_MPa?: number;
  flexural_strength_MPa?: number;

  // Curing
  initial_setting_time_min: number;
  final_setting_time_min: number;
  curing_time_days: number;

  // Environmental
  co2_emissions_kg_per_m3?: number;
  recycled_content_percent?: number;

  // Print parameters
  recommended_layer_height_mm: number;
  recommended_extrusion_temp_C?: number;
  printability_index?: number; // 0-100
}

/**
 * Material batch
 */
export interface MaterialBatch {
  batchId: string;
  material: MaterialProperties;
  quantity_kg: number;
  production_date: Timestamp;
  expiry_date?: Timestamp;
  supplier: string;
  quality_control: {
    tested: boolean;
    test_date?: Timestamp;
    test_results?: Record<string, number>;
    certified: boolean;
  };
}

// ============================================================================
// Print Parameters
// ============================================================================

/**
 * Print settings
 */
export interface PrintParameters {
  // Layer settings
  layer_height_mm: number;
  first_layer_height_mm?: number;
  layer_width_mm: number;

  // Speed
  print_speed_mm_s: number;
  travel_speed_mm_s: number;
  first_layer_speed_mm_s?: number;

  // Extrusion
  extrusion_rate_kg_h: number;
  extrusion_multiplier: number; // 0.8-1.2

  // Temperature (if applicable)
  material_temp_C?: number;
  ambient_temp_C?: number;

  // Path planning
  wall_thickness_mm: number;
  infill_density_percent: number;
  infill_pattern?: 'RECTILINEAR' | 'HONEYCOMB' | 'TRIANGULAR' | 'NONE';

  // Support
  support_required: boolean;
  support_pattern?: string;

  // Timing
  pause_between_layers_s?: number;
}

// ============================================================================
// Building Design
// ============================================================================

/**
 * Building type
 */
export enum BuildingType {
  RESIDENTIAL = 'RESIDENTIAL',          // 주거용
  COMMERCIAL = 'COMMERCIAL',            // 상업용
  INDUSTRIAL = 'INDUSTRIAL',            // 산업용
  EMERGENCY_SHELTER = 'EMERGENCY_SHELTER', // 긴급 대피소
  AFFORDABLE_HOUSING = 'AFFORDABLE_HOUSING', // 저가 주택
  CUSTOM = 'CUSTOM',                    // 맞춤형
}

/**
 * Building design
 */
export interface BuildingDesign {
  designId: string;
  name: string;
  type: BuildingType;

  // Dimensions
  dimensions: {
    length_m: number;
    width_m: number;
    height_m: number;
    total_area_m2: number;
    living_area_m2?: number;
  };

  // Structure
  walls: {
    thickness_mm: number;
    material: MaterialType;
    reinforcement?: string;
  };

  roof: {
    type: 'FLAT' | 'PITCHED' | 'DOMED' | 'PRINTED' | 'TRADITIONAL';
    material?: string;
  };

  foundation: {
    type: 'SLAB' | 'PIER' | 'STRIP' | 'PRINTED';
    depth_m?: number;
  };

  // Features
  rooms: number;
  floors: number;
  windows: number;
  doors: number;

  // Files
  model_file_url: string; // 3D model (STL, OBJ, etc.)
  gcode_file_url?: string; // G-code for printer

  // Compliance
  building_code: string; // e.g., "IBC 2021", "KBC 2021"
  structural_certification?: string;

  // Estimates
  estimated_print_time_hours: number;
  estimated_material_kg: number;
  estimated_cost_USD?: number;
}

// ============================================================================
// Construction Project
// ============================================================================

/**
 * Project status
 */
export enum ProjectStatus {
  PLANNING = 'PLANNING',
  APPROVED = 'APPROVED',
  IN_PROGRESS = 'IN_PROGRESS',
  PAUSED = 'PAUSED',
  COMPLETED = 'COMPLETED',
  INSPECTED = 'INSPECTED',
  OCCUPIED = 'OCCUPIED',
  CANCELLED = 'CANCELLED',
}

/**
 * Construction project
 */
export interface ConstructionProject {
  projectId: string;
  name: string;
  design: BuildingDesign;
  location: Location;

  // Parties
  owner: string;
  contractor: string;
  architect?: string;
  engineer?: string;

  // Equipment
  printer: PrinterSpec;
  materials: MaterialBatch[];

  // Timeline
  start_date: Timestamp;
  estimated_completion_date: Timestamp;
  actual_completion_date?: Timestamp;

  // Progress
  status: ProjectStatus;
  completion_percent: number;

  // Print job
  print_job?: PrintJob;

  // Permits & compliance
  building_permit: string;
  environmental_permit?: string;
  safety_plan_url?: string;

  // Quality
  inspections?: Inspection[];
  structural_tests?: StructuralTest[];
}

/**
 * Print job details
 */
export interface PrintJob {
  jobId: string;
  projectId: string;

  // Parameters
  parameters: PrintParameters;

  // Files
  gcode_url: string;
  preview_image_url?: string;

  // Progress
  status: 'QUEUED' | 'PRINTING' | 'PAUSED' | 'COMPLETED' | 'FAILED';
  progress_percent: number;
  current_layer: number;
  total_layers: number;

  // Timing
  start_time?: Timestamp;
  end_time?: Timestamp;
  elapsed_time_hours?: number;
  remaining_time_hours?: number;

  // Materials used
  material_used_kg: number;
  material_wasted_kg?: number;

  // Quality metrics
  layer_adhesion_quality?: number; // 0-100
  dimensional_accuracy_mm?: number;
  surface_quality?: number; // 0-100

  // Monitoring
  temperature_log?: Array<{
    timestamp: Timestamp;
    material_temp_C: number;
    ambient_temp_C: number;
  }>;

  error_log?: Array<{
    timestamp: Timestamp;
    error_code: string;
    error_message: string;
    resolved: boolean;
  }>;
}

// ============================================================================
// Structural Integrity
// ============================================================================

/**
 * Structural test type
 */
export enum StructuralTestType {
  COMPRESSIVE_STRENGTH = 'COMPRESSIVE_STRENGTH',
  TENSILE_STRENGTH = 'TENSILE_STRENGTH',
  FLEXURAL_STRENGTH = 'FLEXURAL_STRENGTH',
  BOND_STRENGTH = 'BOND_STRENGTH',       // 층간 접착력
  WATER_ABSORPTION = 'WATER_ABSORPTION',
  FREEZE_THAW = 'FREEZE_THAW',
  SEISMIC = 'SEISMIC',
  LOAD_BEARING = 'LOAD_BEARING',
}

/**
 * Structural test result
 */
export interface StructuralTest {
  testId: string;
  projectId: string;
  type: StructuralTestType;

  // Test details
  test_date: Timestamp;
  specimen_age_days: number;
  test_standard: string; // e.g., "ASTM C39", "KS F 2405"

  // Results
  result_value: number;
  result_unit: string;
  required_value: number;
  passed: boolean;

  // Sample info
  sample_location: string;
  sample_id?: string;

  // Testing
  lab: string;
  inspector: string;
  certificate_url?: string;

  notes?: string;
}

/**
 * Building inspection
 */
export interface Inspection {
  inspectionId: string;
  projectId: string;

  // Type
  inspection_type: 'FOUNDATION' | 'STRUCTURAL' | 'ELECTRICAL' | 'PLUMBING' | 'FINAL';

  // Timing
  inspection_date: Timestamp;

  // Inspector
  inspector_name: string;
  inspector_license: string;
  inspector_organization: string;

  // Results
  passed: boolean;
  defects?: Array<{
    severity: 'MINOR' | 'MAJOR' | 'CRITICAL';
    location: string;
    description: string;
    photo_url?: string;
  }>;

  // Compliance
  building_code_compliance: boolean;
  safety_compliance: boolean;

  // Documentation
  report_url: string;
  certificate_url?: string;

  notes?: string;
}

// ============================================================================
// Building Codes & Compliance
// ============================================================================

/**
 * Building code standard
 */
export enum BuildingCodeStandard {
  IBC = 'IBC',                          // International Building Code
  KBC = 'KBC',                          // Korean Building Code
  EUROCODE = 'EUROCODE',                // European standards
  ASTM = 'ASTM',                        // ASTM standards
  ISO = 'ISO',                          // ISO standards
  LOCAL = 'LOCAL',                      // Local regulations
}

/**
 * Compliance checklist
 */
export interface ComplianceChecklist {
  projectId: string;

  // Building codes
  building_code: BuildingCodeStandard;
  code_version: string;

  // Structural requirements
  structural: {
    load_bearing_capacity_compliant: boolean;
    seismic_resistance_compliant: boolean;
    wind_resistance_compliant: boolean;
    fire_resistance_compliant: boolean;
  };

  // Material requirements
  material: {
    material_certification: boolean;
    strength_tests_passed: boolean;
    durability_tests_passed: boolean;
  };

  // Safety
  safety: {
    fall_protection: boolean;
    electrical_safety: boolean;
    fire_safety: boolean;
    emergency_exits: boolean;
  };

  // Environmental
  environmental: {
    environmental_impact_assessment: boolean;
    waste_management_plan: boolean;
    emissions_within_limits: boolean;
  };

  // Accessibility
  accessibility: {
    wheelchair_accessible: boolean;
    ada_compliant?: boolean; // US only
  };

  // Overall
  overall_compliance: boolean;
  approved_for_occupancy: boolean;
  occupancy_certificate?: string;
}

// ============================================================================
// Project Tracking
// ============================================================================

/**
 * Project milestone
 */
export interface ProjectMilestone {
  milestone_id: string;
  project_id: string;

  name: string;
  description?: string;

  planned_date: Timestamp;
  actual_date?: Timestamp;

  status: 'PENDING' | 'IN_PROGRESS' | 'COMPLETED' | 'DELAYED';

  dependencies?: string[]; // milestone IDs

  deliverables?: string[];
}

/**
 * Project timeline
 */
export interface ProjectTimeline {
  projectId: string;

  milestones: ProjectMilestone[];

  critical_path: string[]; // milestone IDs

  overall_status: 'ON_TRACK' | 'AT_RISK' | 'DELAYED' | 'COMPLETED';

  delay_days?: number;
}

/**
 * Cost tracking
 */
export interface CostTracking {
  projectId: string;

  // Budget
  budget: {
    materials_USD: number;
    labor_USD: number;
    equipment_USD: number;
    permits_USD: number;
    contingency_USD: number;
    total_USD: number;
  };

  // Actual costs
  actual: {
    materials_USD: number;
    labor_USD: number;
    equipment_USD: number;
    permits_USD: number;
    other_USD: number;
    total_USD: number;
  };

  // Variance
  variance_USD: number;
  variance_percent: number;

  status: 'UNDER_BUDGET' | 'ON_BUDGET' | 'OVER_BUDGET';
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Project registration request
 */
export interface ProjectRegistrationRequest {
  project: Omit<ConstructionProject, 'projectId' | 'status' | 'completion_percent'>;
}

/**
 * Project registration response
 */
export interface ProjectRegistrationResponse {
  projectId: string;
  registrationNumber: string;
  permitUrl: string;
  trackingUrl: string;
  qrCode?: string;
}

/**
 * Print job submission request
 */
export interface PrintJobSubmissionRequest {
  projectId: string;
  parameters: PrintParameters;
  gcode_url: string;
}

/**
 * Print job submission response
 */
export interface PrintJobSubmissionResponse {
  jobId: string;
  status: string;
  estimated_completion: Timestamp;
  monitoring_url: string;
}

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
    requestId?: string;
  };
}

// ============================================================================
// Statistics & Reporting
// ============================================================================

/**
 * Project statistics
 */
export interface ProjectStatistics {
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  // Projects
  total_projects: number;
  completed_projects: number;
  active_projects: number;

  // Buildings
  total_area_built_m2: number;
  total_buildings: number;
  by_type: Record<BuildingType, number>;

  // Materials
  total_material_used_kg: number;
  by_material_type: Record<MaterialType, number>;

  // Environmental impact
  total_co2_emissions_kg: number;
  recycled_material_percent: number;

  // Economics
  total_cost_USD: number;
  average_cost_per_m2_USD: number;
  cost_savings_vs_traditional_percent: number;

  // Quality
  average_completion_time_days: number;
  average_inspection_pass_rate_percent: number;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
