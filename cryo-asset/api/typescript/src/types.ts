/**
 * WIA Cryo Asset Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Asset Types
// ============================================================================

export interface WIACryoAssetProject {
  standard: 'WIA-CRYO-ASSET';
  version: string;
  metadata: ProjectMetadata;
  assets: CryoAsset[];
  storage: StorageConfiguration;
  valuation: ValuationFramework;
  custody: CustodyArrangement;
  insurance: InsuranceCoverage;
  monitoring: MonitoringSystem;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: ProjectType;
  organization: Organization;
  jurisdiction: string;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export type ProjectType =
  | 'biological'
  | 'tissue-banking'
  | 'reproductive'
  | 'research'
  | 'preservation-service';

export interface Organization {
  name: string;
  type: 'facility' | 'bank' | 'hospital' | 'research' | 'commercial';
  country: string;
  registrationNumber?: string;
  licenses?: License[];
  contact?: ContactInfo;
}

export interface License {
  type: string;
  number: string;
  issuer: string;
  validFrom: string;
  validTo: string;
  status: 'active' | 'expired' | 'suspended';
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
  address?: string;
}

export type ProjectStatus =
  | 'active'
  | 'pending'
  | 'suspended'
  | 'terminated';

// ============================================================================
// Cryo Asset
// ============================================================================

export interface CryoAsset {
  id: string;
  type: AssetType;
  category: AssetCategory;
  identifier: AssetIdentifier;
  specimen: SpecimenDetails;
  preservation: PreservationDetails;
  ownership: OwnershipRecord;
  storage: StorageAssignment;
  status: AssetStatus;
  value?: AssetValue;
  history: AssetEvent[];
}

export type AssetType =
  | 'cell'
  | 'tissue'
  | 'organ'
  | 'embryo'
  | 'gamete'
  | 'blood'
  | 'stem-cell'
  | 'dna'
  | 'protein'
  | 'other';

export type AssetCategory =
  | 'human'
  | 'animal'
  | 'plant'
  | 'microbial'
  | 'synthetic';

export interface AssetIdentifier {
  internalId: string;
  barcode?: string;
  rfid?: string;
  externalIds?: ExternalId[];
}

export interface ExternalId {
  system: string;
  id: string;
}

export interface SpecimenDetails {
  description: string;
  source: SpecimenSource;
  collection: CollectionInfo;
  processing: ProcessingInfo;
  quality: QualityAssessment;
  metadata?: Record<string, unknown>;
}

export interface SpecimenSource {
  type: 'donor' | 'patient' | 'research' | 'commercial' | 'unknown';
  donorId?: string;
  consent?: string;
  origin?: string;
  species?: string;
}

export interface CollectionInfo {
  date: string;
  location: string;
  method: string;
  collector: string;
  protocol?: string;
}

export interface ProcessingInfo {
  date: string;
  method: string;
  cryoprotectant?: string;
  coolingRate?: string;
  processor: string;
  protocol?: string;
  notes?: string;
}

export interface QualityAssessment {
  date: string;
  viability?: number;
  purity?: number;
  sterility?: boolean;
  tests: QualityTest[];
  grade?: string;
  assessor: string;
}

export interface QualityTest {
  name: string;
  result: string;
  reference?: string;
  passed: boolean;
}

export interface PreservationDetails {
  method: PreservationMethod;
  temperature: number;
  medium?: string;
  container: ContainerInfo;
  startDate: string;
  expectedDuration?: string;
}

export type PreservationMethod =
  | 'slow-freeze'
  | 'vitrification'
  | 'controlled-rate'
  | 'ultra-rapid';

export interface ContainerInfo {
  type: 'vial' | 'straw' | 'bag' | 'cassette' | 'cryobox' | 'other';
  size: string;
  material: string;
  volume?: number;
  count: number;
}

export interface OwnershipRecord {
  owner: Owner;
  beneficiaries?: Beneficiary[];
  restrictions?: string[];
  transferable: boolean;
  encumbrances?: string[];
}

export interface Owner {
  id: string;
  type: 'individual' | 'organization' | 'trust' | 'estate';
  name: string;
  contact?: ContactInfo;
}

export interface Beneficiary {
  id: string;
  name: string;
  relationship: string;
  share: number;
  conditions?: string[];
}

export interface StorageAssignment {
  facilityId: string;
  tankId: string;
  rackId?: string;
  boxId?: string;
  position?: string;
  assignedAt: string;
  assignedBy: string;
}

export type AssetStatus =
  | 'stored'
  | 'in-transit'
  | 'thawed'
  | 'distributed'
  | 'disposed'
  | 'lost'
  | 'quarantine';

export interface AssetValue {
  amount: number;
  currency: string;
  valuationDate: string;
  method: ValuationMethod;
  appraiser?: string;
}

export type ValuationMethod =
  | 'cost-based'
  | 'market-based'
  | 'replacement-cost'
  | 'scientific-value';

export interface AssetEvent {
  id: string;
  type: EventType;
  timestamp: string;
  description: string;
  actor: string;
  details?: Record<string, unknown>;
}

export type EventType =
  | 'collection'
  | 'processing'
  | 'storage'
  | 'transfer'
  | 'quality-check'
  | 'thaw'
  | 'distribution'
  | 'disposal'
  | 'audit';

// ============================================================================
// Storage Configuration
// ============================================================================

export interface StorageConfiguration {
  facilities: StorageFacility[];
  equipment: StorageEquipment[];
  protocols: StorageProtocol[];
  capacity: CapacitySummary;
}

export interface StorageFacility {
  id: string;
  name: string;
  type: FacilityType;
  location: Location;
  certifications: Certification[];
  capacity: FacilityCapacity;
  status: 'operational' | 'maintenance' | 'offline';
}

export type FacilityType =
  | 'primary'
  | 'backup'
  | 'disaster-recovery'
  | 'satellite';

export interface Location {
  address: string;
  city: string;
  country: string;
  coordinates?: { latitude: number; longitude: number };
  timezone: string;
}

export interface Certification {
  type: string;
  issuer: string;
  number: string;
  validFrom: string;
  validTo: string;
}

export interface FacilityCapacity {
  tanks: number;
  specimens: number;
  utilization: number;
}

export interface StorageEquipment {
  id: string;
  type: EquipmentType;
  model: string;
  manufacturer: string;
  location: string;
  capacity: number;
  temperature: TemperatureSpec;
  monitoring: MonitoringConfig;
  maintenance: MaintenanceSchedule;
  status: 'operational' | 'warning' | 'alarm' | 'offline';
}

export type EquipmentType =
  | 'ln2-tank'
  | 'ln2-freezer'
  | 'mechanical-freezer'
  | 'controlled-rate-freezer';

export interface TemperatureSpec {
  target: number;
  min: number;
  max: number;
  unit: 'celsius' | 'kelvin';
}

export interface MonitoringConfig {
  sensors: SensorConfig[];
  interval: number;
  alerts: AlertConfig[];
}

export interface SensorConfig {
  id: string;
  type: 'temperature' | 'level' | 'pressure';
  location: string;
  accuracy: number;
}

export interface AlertConfig {
  parameter: string;
  threshold: number;
  condition: 'above' | 'below';
  severity: 'warning' | 'critical';
  recipients: string[];
}

export interface MaintenanceSchedule {
  lastService: string;
  nextService: string;
  frequency: string;
  provider: string;
}

export interface StorageProtocol {
  id: string;
  name: string;
  version: string;
  assetTypes: AssetType[];
  procedure: string[];
  requirements: string[];
  approved: boolean;
  approvedBy: string;
  approvedAt: string;
}

export interface CapacitySummary {
  total: number;
  used: number;
  available: number;
  reserved: number;
}

// ============================================================================
// Valuation & Custody
// ============================================================================

export interface ValuationFramework {
  methodology: ValuationMethodology;
  schedule: ValuationSchedule;
  history: ValuationRecord[];
}

export interface ValuationMethodology {
  primary: ValuationMethod;
  secondary?: ValuationMethod;
  factors: ValuationFactor[];
  documentation: string;
}

export interface ValuationFactor {
  name: string;
  weight: number;
  description: string;
}

export interface ValuationSchedule {
  frequency: 'annual' | 'semi-annual' | 'quarterly' | 'on-demand';
  lastValuation: string;
  nextValuation: string;
}

export interface ValuationRecord {
  date: string;
  totalValue: number;
  currency: string;
  assets: { assetId: string; value: number }[];
  appraiser: string;
  report?: string;
}

export interface CustodyArrangement {
  custodian: Custodian;
  terms: CustodyTerms;
  responsibilities: CustodyResponsibility[];
  reporting: ReportingRequirements;
}

export interface Custodian {
  id: string;
  name: string;
  type: 'facility' | 'bank' | 'trust' | 'third-party';
  licenses: License[];
  contact: ContactInfo;
  insurance: string;
}

export interface CustodyTerms {
  startDate: string;
  endDate?: string;
  renewalTerms?: string;
  terminationConditions: string[];
  fees: CustodyFee[];
}

export interface CustodyFee {
  type: 'storage' | 'handling' | 'administrative' | 'insurance';
  amount: number;
  currency: string;
  frequency: 'monthly' | 'annual' | 'per-transaction';
}

export interface CustodyResponsibility {
  area: string;
  responsible: 'custodian' | 'owner' | 'shared';
  description: string;
}

export interface ReportingRequirements {
  inventory: { frequency: string; format: string };
  monitoring: { frequency: string; format: string };
  financial: { frequency: string; format: string };
}

// ============================================================================
// Insurance & Monitoring
// ============================================================================

export interface InsuranceCoverage {
  policies: InsurancePolicy[];
  totalCoverage: number;
  currency: string;
  gaps?: string[];
}

export interface InsurancePolicy {
  id: string;
  type: InsuranceType;
  provider: string;
  policyNumber: string;
  coverage: number;
  deductible: number;
  premium: number;
  currency: string;
  validFrom: string;
  validTo: string;
  exclusions?: string[];
  claims?: Claim[];
}

export type InsuranceType =
  | 'property'
  | 'liability'
  | 'business-interruption'
  | 'transit'
  | 'errors-omissions';

export interface Claim {
  id: string;
  date: string;
  amount: number;
  reason: string;
  status: 'filed' | 'processing' | 'approved' | 'denied' | 'paid';
}

export interface MonitoringSystem {
  realTime: RealTimeMonitoring;
  alerts: AlertManagement;
  reporting: ReportingConfig;
  audit: AuditConfig;
}

export interface RealTimeMonitoring {
  enabled: boolean;
  parameters: MonitoredParameter[];
  dashboard: string;
  retention: string;
}

export interface MonitoredParameter {
  id: string;
  name: string;
  unit: string;
  frequency: number;
  thresholds: { warning: number; critical: number };
}

export interface AlertManagement {
  channels: AlertChannel[];
  escalation: EscalationPolicy;
  acknowledgement: string;
}

export interface AlertChannel {
  type: 'email' | 'sms' | 'phone' | 'app' | 'api';
  recipients: string[];
  severity: ('warning' | 'critical')[];
}

export interface EscalationPolicy {
  levels: EscalationLevel[];
  timeout: number;
}

export interface EscalationLevel {
  level: number;
  delay: number;
  recipients: string[];
}

export interface ReportingConfig {
  automated: AutomatedReport[];
  onDemand: string[];
}

export interface AutomatedReport {
  name: string;
  frequency: string;
  format: 'pdf' | 'excel' | 'json';
  recipients: string[];
}

export interface AuditConfig {
  frequency: string;
  scope: string[];
  auditor: 'internal' | 'external' | 'both';
  lastAudit?: string;
  nextAudit?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
