/**
 * WIA-IND-025: Quality Control Standard - TypeScript Type Definitions
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
 * Timestamp with timezone
 */
export type Timestamp = Date | string;

/**
 * Product identifier (SKU)
 */
export type ProductSKU = string;

/**
 * Employee identifier
 */
export type EmployeeId = string;

/**
 * Unique identifier
 */
export type UUID = string;

// ============================================================================
// Inspection Types
// ============================================================================

/**
 * Inspection type classification
 */
export type InspectionType =
  | 'receiving'      // Incoming materials
  | 'first-article'  // First production unit
  | 'in-process'     // During production
  | 'final'          // Finished product
  | 'patrol'         // Random sampling
  | 'audit';         // Quality audit

/**
 * Inspection result
 */
export type InspectionResult = 'pass' | 'fail' | 'conditional';

/**
 * Disposition decision
 */
export type Disposition =
  | 'accept'
  | 'rework'
  | 'scrap'
  | 'ncr'
  | 'use-as-is'
  | 'repair'
  | 'return-to-supplier';

/**
 * Complete inspection record
 */
export interface Inspection {
  inspectionId: string;
  inspectionType: InspectionType;
  timestamp: Timestamp;
  productSku: ProductSKU;
  batchNumber: string;
  lotNumber?: string;
  serialNumbers?: string[];
  quantity: number;
  inspector: Inspector;
  measurements: Measurement[];
  defects: Defect[];
  visualInspection?: VisualInspection;
  functionalTest?: FunctionalTest;
  result: InspectionResult;
  disposition?: Disposition;
  ncrId?: string;
  spcData?: SPCData;
  attachments?: string[];
  approvals?: Approval[];
  comments?: string;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Inspector information
 */
export interface Inspector {
  employeeId: EmployeeId;
  name: string;
  department?: string;
  certification?: string;
  certificationExpiry?: Timestamp;
  visionTestDate?: Timestamp;
}

/**
 * Dimensional or parametric measurement
 */
export interface Measurement {
  measurementId?: string;
  characteristicId: string;
  characteristic: string;
  nominal: number;
  measured: number;
  tolerance: number;
  unit: string;
  usl?: number; // Upper Spec Limit
  lsl?: number; // Lower Spec Limit
  status: 'pass' | 'fail';
  equipment?: string;
  equipmentId?: string;
  method?: string;
  timestamp?: Timestamp;
}

/**
 * Defect information
 */
export interface Defect {
  defectId: string;
  type: string;
  category: DefectCategory;
  severity: DefectSeverity;
  location: string;
  count: number;
  description: string;
  imageUrl?: string;
  detectionMethod?: 'visual' | 'automated' | 'ndt' | 'functional';
  aiConfidence?: number; // 0-1 for AI detection
}

/**
 * Defect category
 */
export type DefectCategory =
  | 'dimensional'
  | 'surface'
  | 'assembly'
  | 'functional'
  | 'material'
  | 'cosmetic';

/**
 * Defect severity levels
 */
export type DefectSeverity = 'critical' | 'major' | 'minor' | 'cosmetic';

/**
 * Visual inspection details
 */
export interface VisualInspection {
  passed: boolean;
  lighting: number; // lux
  magnification?: number;
  limitSamplesUsed?: string[];
  findings: string[];
}

/**
 * Functional test results
 */
export interface FunctionalTest {
  testId: string;
  testProcedure: string;
  passed: boolean;
  results: {
    parameter: string;
    expected: any;
    actual: any;
    status: 'pass' | 'fail';
  }[];
}

/**
 * Approval record
 */
export interface Approval {
  role: string;
  approver: string;
  approverId: EmployeeId;
  timestamp: Timestamp;
  approved: boolean;
  comments?: string;
  signature?: string;
}

// ============================================================================
// Statistical Process Control (SPC) Types
// ============================================================================

/**
 * SPC data and capability indices
 */
export interface SPCData {
  cp: number;           // Process Capability
  cpk: number;          // Process Capability Index
  pp?: number;          // Process Performance
  ppk?: number;         // Process Performance Index
  mean: number;         // Process mean
  stdDev: number;       // Standard deviation
  sigmaLevel: number;   // Six Sigma level
  dpmo: number;         // Defects Per Million Opportunities
  outOfControl: boolean;
  controlLimits?: ControlLimits;
  capability?: CapabilityAssessment;
}

/**
 * Control limits for control charts
 */
export interface ControlLimits {
  ucl: number;  // Upper Control Limit
  lcl: number;  // Lower Control Limit
  centerline: number;
  usl?: number; // Upper Spec Limit
  lsl?: number; // Lower Spec Limit
}

/**
 * Process capability assessment
 */
export interface CapabilityAssessment {
  cpk: number;
  rating: 'excellent' | 'capable' | 'marginal' | 'not-capable';
  recommendation: string;
}

/**
 * Control chart types
 */
export type ControlChartType =
  | 'xbar-r'    // X-bar and R chart (variables)
  | 'xbar-s'    // X-bar and S chart (variables)
  | 'i-mr'      // Individual and Moving Range
  | 'p-chart'   // Proportion defective (attributes)
  | 'np-chart'  // Number defective (attributes)
  | 'c-chart'   // Count of defects (attributes)
  | 'u-chart'   // Defects per unit (attributes)
  | 'cusum'     // Cumulative Sum
  | 'ewma';     // Exponentially Weighted Moving Average

/**
 * Control chart configuration
 */
export interface ControlChart {
  chartId: string;
  type: ControlChartType;
  characteristic: string;
  subgroupSize?: number;
  targetValue?: number;
  controlLimits: ControlLimits;
  dataPoints: ControlChartDataPoint[];
  outOfControlPoints: number[];
  violatedRules: WesternElectricRule[];
}

/**
 * Control chart data point
 */
export interface ControlChartDataPoint {
  sampleNumber: number;
  timestamp: Timestamp;
  value: number;
  range?: number;
  stdDev?: number;
  inControl: boolean;
}

/**
 * Western Electric rules
 */
export type WesternElectricRule =
  | 'beyond-3-sigma'
  | '9-consecutive-same-side'
  | '6-consecutive-increasing'
  | '14-alternating'
  | '2-of-3-beyond-2-sigma'
  | '4-of-5-beyond-1-sigma';

/**
 * SPC calculation request
 */
export interface SPCCalculationRequest {
  characteristic: string;
  measurements: number[];
  specification: {
    nominal: number;
    usl: number;
    lsl: number;
  };
  subgroupSize?: number;
}

// ============================================================================
// Non-Conformance Report (NCR) Types
// ============================================================================

/**
 * Non-Conformance Report status
 */
export type NCRStatus =
  | 'open'
  | 'investigation'
  | 'disposition'
  | 'verification'
  | 'closed';

/**
 * Detection point for non-conformance
 */
export type DetectionPoint =
  | 'receiving'
  | 'in-process'
  | 'final'
  | 'customer'
  | 'audit'
  | 'field';

/**
 * Complete Non-Conformance Report
 */
export interface NCR {
  ncrId: string;
  ncrNumber: string;
  status: NCRStatus;
  dateReported: Timestamp;
  reportedBy: Employee;
  productInfo: ProductInfo;
  issueDescription: string;
  detectionPoint: DetectionPoint;
  severity: DefectSeverity;
  defectType: string;
  rootCause?: RootCauseAnalysis;
  containmentAction: ContainmentAction;
  disposition: DispositionDecision;
  customerImpact: 'none' | 'potential' | 'actual';
  customerNotified: boolean;
  customerNotificationDate?: Timestamp;
  capaRequired: boolean;
  capaId?: string;
  attachments: string[];
  mrb?: MaterialReviewBoard;
  costImpact?: CostImpact;
  closedBy?: EmployeeId;
  closedDate?: Timestamp;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Employee information
 */
export interface Employee {
  employeeId: EmployeeId;
  name: string;
  department: string;
  email?: string;
  phone?: string;
}

/**
 * Product information for NCR
 */
export interface ProductInfo {
  sku: ProductSKU;
  description: string;
  batchNumber?: string;
  lotNumber?: string;
  serialNumbers?: string[];
  quantityAffected: number;
  productionDate?: Timestamp;
  supplier?: string;
}

/**
 * Root cause analysis methods
 */
export type RCAMethod = '5-why' | 'fishbone' | 'fta' | 'fmea' | '8d';

/**
 * Root cause analysis
 */
export interface RootCauseAnalysis {
  method: RCAMethod;
  analysis: string;
  findings?: string[];
  rootCause: string;
  contributingFactors?: string[];
  supportingData?: string[];
  performedBy: EmployeeId;
  performedDate: Timestamp;
}

/**
 * Containment action
 */
export interface ContainmentAction {
  description: string;
  implementedBy: EmployeeId;
  implementedDate: Timestamp;
  effective: boolean;
  verificationMethod?: string;
}

/**
 * Disposition decision
 */
export interface DispositionDecision {
  decision: Disposition;
  justification: string;
  approvedBy: EmployeeId;
  approvalDate: Timestamp;
  reworkProcedure?: string;
  reworkCompleted?: boolean;
  verifiedBy?: EmployeeId;
}

/**
 * Material Review Board
 */
export interface MaterialReviewBoard {
  reviewDate: Timestamp;
  members: EmployeeId[];
  chairperson: EmployeeId;
  decision: string;
  minutes?: string;
}

/**
 * Cost impact of non-conformance
 */
export interface CostImpact {
  scrapCost: number;
  reworkCost: number;
  laborCost: number;
  inspectionCost: number;
  shippingCost: number;
  opportunityCost: number;
  totalCost: number;
  currency: string;
}

// ============================================================================
// CAPA (Corrective and Preventive Action) Types
// ============================================================================

/**
 * CAPA type
 */
export type CAPAType = 'corrective' | 'preventive' | 'both';

/**
 * CAPA status
 */
export type CAPAStatus =
  | 'open'
  | 'in-progress'
  | 'verification'
  | 'effectiveness'
  | 'closed';

/**
 * CAPA source/trigger
 */
export type CAPASource =
  | 'ncr'
  | 'audit'
  | 'complaint'
  | 'management-review'
  | 'trend'
  | 'risk-assessment';

/**
 * Action status
 */
export type ActionStatus = 'planned' | 'in-progress' | 'completed';

/**
 * Complete CAPA record
 */
export interface CAPA {
  capaId: string;
  capaNumber: string;
  type: CAPAType;
  status: CAPAStatus;
  priority: 'low' | 'medium' | 'high' | 'critical';
  dateInitiated: Timestamp;
  initiator: Employee;
  source: CAPASource;
  linkedRecords: LinkedRecords;
  problemStatement: string;
  rootCauseAnalysis: RootCauseAnalysis;
  correctiveAction?: Action;
  preventiveAction?: Action;
  verification: Verification;
  effectivenessCheck: EffectivenessCheck;
  lessonsLearned?: string;
  closedBy?: EmployeeId;
  closedDate?: Timestamp;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Linked records (NCR, audit, etc.)
 */
export interface LinkedRecords {
  ncrIds?: string[];
  auditIds?: string[];
  complaintIds?: string[];
  otherReferences?: string[];
}

/**
 * Corrective or preventive action
 */
export interface Action {
  description: string;
  assignedTo: EmployeeId;
  assignedDate: Timestamp;
  dueDate: Timestamp;
  status: ActionStatus;
  completionDate?: Timestamp;
  evidence?: string[];
  resources?: string[];
  estimatedCost?: number;
}

/**
 * Verification of action implementation
 */
export interface Verification {
  method: string;
  responsible: EmployeeId;
  plannedDate: Timestamp;
  verificationDate?: Timestamp;
  verified: boolean;
  comments?: string;
  evidence?: string[];
}

/**
 * Effectiveness check
 */
export interface EffectivenessCheck {
  followUpPeriod: number; // days
  metricsMonitored: string[];
  checkDate?: Timestamp;
  effective: boolean;
  data?: EffectivenessData[];
  conclusion?: string;
}

/**
 * Effectiveness data (before/after comparison)
 */
export interface EffectivenessData {
  metric: string;
  before: number;
  after: number;
  improvement: number; // percentage
  target?: number;
  targetMet: boolean;
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Equipment criticality
 */
export type EquipmentCriticality = 'critical' | 'non-critical' | 'reference-standard';

/**
 * Calibration status
 */
export type CalibrationStatus = 'in-calibration' | 'due-soon' | 'overdue' | 'out-of-tolerance';

/**
 * Calibration record
 */
export interface CalibrationRecord {
  calibrationId: string;
  equipmentId: string;
  equipmentInfo: EquipmentInfo;
  calibrationDate: Timestamp;
  nextDueDate: Timestamp;
  interval: number; // days
  calibrationLab: CalibrationLab;
  calibratedBy: EmployeeId;
  standardsUsed: CalibrationStandard[];
  asFoundCondition: CalibrationCondition;
  adjustmentsMade: string;
  asLeftCondition: CalibrationCondition;
  uncertainty: Uncertainty;
  environmentalConditions: EnvironmentalConditions;
  status: CalibrationStatus;
  certificateNumber: string;
  certificateUrl?: string;
  approvedBy: EmployeeId;
  createdAt: Timestamp;
}

/**
 * Equipment information
 */
export interface EquipmentInfo {
  description: string;
  manufacturer: string;
  model: string;
  serialNumber: string;
  assetNumber?: string;
  location: string;
  department: string;
  criticality: EquipmentCriticality;
  accuracy?: string;
  range?: string;
}

/**
 * Calibration laboratory
 */
export interface CalibrationLab {
  name: string;
  accreditation?: string; // e.g., "ISO/IEC 17025"
  accreditationNumber?: string;
  certificate?: string;
  address?: string;
  contact?: string;
}

/**
 * Calibration standard (reference)
 */
export interface CalibrationStandard {
  standardId: string;
  description: string;
  traceability: string; // e.g., "NIST"
  certificateNumber: string;
  calibrationDate: Timestamp;
  expirationDate: Timestamp;
}

/**
 * Calibration condition (as-found or as-left)
 */
export interface CalibrationCondition {
  inTolerance: boolean;
  readings: CalibrationReading[];
  overallStatus: 'pass' | 'fail';
}

/**
 * Calibration reading at test point
 */
export interface CalibrationReading {
  point: string;
  expected: number;
  measured: number;
  tolerance: number;
  error: number;
  unit: string;
  status: 'pass' | 'fail';
}

/**
 * Measurement uncertainty
 */
export interface Uncertainty {
  value: number;
  unit: string;
  confidenceLevel: number; // e.g., 95
  coverageFactor?: number; // k-factor
}

/**
 * Environmental conditions during calibration
 */
export interface EnvironmentalConditions {
  temperature: number;
  humidity: number;
  pressure?: number;
  temperatureUnit: string;
  humidityUnit: string;
  pressureUnit?: string;
}

// ============================================================================
// Audit Types
// ============================================================================

/**
 * Audit type
 */
export type AuditType = 'internal' | 'external' | 'supplier' | 'customer' | 'regulatory';

/**
 * Audit status
 */
export type AuditStatus = 'planned' | 'in-progress' | 'reported' | 'capa' | 'closed';

/**
 * Finding severity
 */
export type FindingSeverity = 'major-nc' | 'minor-nc' | 'observation';

/**
 * Audit record
 */
export interface Audit {
  auditId: string;
  auditNumber: string;
  type: AuditType;
  standard: string; // e.g., "ISO-9001:2015"
  scope: string;
  auditDate: Timestamp;
  auditTeam: AuditTeam;
  auditee: Auditee;
  agenda: AuditAgendaItem[];
  findings: Finding[];
  observations: string[];
  strengths: string[];
  overallConclusion: string;
  reportIssueDate: Timestamp;
  reportUrl?: string;
  followUpRequired: boolean;
  followUpDate?: Timestamp;
  status: AuditStatus;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Audit team
 */
export interface AuditTeam {
  leadAuditor: EmployeeId;
  auditors: EmployeeId[];
  technicalExperts?: EmployeeId[];
}

/**
 * Auditee information
 */
export interface Auditee {
  organization: string;
  location: string;
  representatives: EmployeeId[];
  contact?: string;
}

/**
 * Audit agenda item
 */
export interface AuditAgendaItem {
  time: string;
  process: string;
  clause?: string;
  area?: string;
  auditor: EmployeeId;
  notes?: string;
}

/**
 * Audit finding
 */
export interface Finding {
  findingId: string;
  severity: FindingSeverity;
  clause: string;
  process: string;
  area?: string;
  description: string;
  objectiveEvidence: string;
  requirement: string;
  capaRequired: boolean;
  capaId?: string;
  capaDueDate?: Timestamp;
  capaStatus?: CAPAStatus;
  verificationDate?: Timestamp;
  verified: boolean;
  verifiedBy?: EmployeeId;
}

// ============================================================================
// Quality Metrics Types
// ============================================================================

/**
 * Quality metrics dashboard
 */
export interface QualityMetrics {
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  defectMetrics: DefectMetrics;
  processMetrics: ProcessMetrics;
  costMetrics: CostMetrics;
  supplierMetrics?: SupplierMetrics;
  trend: 'improving' | 'stable' | 'declining';
}

/**
 * Defect-related metrics
 */
export interface DefectMetrics {
  defectRate: number; // percentage
  ppm: number; // parts per million
  dpmo: number; // defects per million opportunities
  firstPassYield: number; // percentage
  finalYield: number; // percentage
  rolledThroughputYield: number; // percentage
  escapeRate: number; // percentage
}

/**
 * Process capability metrics
 */
export interface ProcessMetrics {
  averageCpk: number;
  sigmaLevel: number;
  oee: number; // Overall Equipment Effectiveness
  availability: number;
  performance: number;
  quality: number;
  processesInControl: number;
  totalProcesses: number;
}

/**
 * Cost of quality metrics
 */
export interface CostMetrics {
  totalCostOfQuality: number;
  internalFailureCost: number;
  externalFailureCost: number;
  appraisalCost: number;
  preventionCost: number;
  costOfQualityPercentage: number; // % of revenue
  currency: string;
}

/**
 * Supplier quality metrics
 */
export interface SupplierMetrics {
  incomingQualityRate: number; // percentage
  supplierPPM: number;
  onTimeDelivery: number; // percentage
  supplierDefects: number;
  rejectedLots: number;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * SDK Configuration
 */
export interface QualityControlConfig {
  apiKey: string;
  apiEndpoint?: string;
  certificationLevel?: CertificationLevel;
  industry?: Industry;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

/**
 * Certification level
 */
export type CertificationLevel =
  | 'ISO-9001'
  | 'ISO-13485'
  | 'IATF-16949'
  | 'AS9100'
  | 'GMP'
  | 'HACCP';

/**
 * Industry type
 */
export type Industry =
  | 'automotive'
  | 'aerospace'
  | 'medical-device'
  | 'pharmaceutical'
  | 'electronics'
  | 'food-beverage'
  | 'general-manufacturing';

// ============================================================================
// Response Types
// ============================================================================

/**
 * API Response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata?: ResponseMetadata;
}

/**
 * API Error
 */
export interface APIError {
  code: string;
  message: string;
  details?: any;
}

/**
 * Response metadata
 */
export interface ResponseMetadata {
  timestamp: Timestamp;
  version: string;
  requestId?: string;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

/**
 * Filter options for queries
 */
export interface FilterOptions {
  startDate?: Timestamp;
  endDate?: Timestamp;
  status?: string;
  productSku?: string;
  severity?: DefectSeverity;
  limit?: number;
  offset?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
