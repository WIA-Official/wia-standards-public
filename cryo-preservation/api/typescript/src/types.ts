/**
 * WIA Cryo Preservation Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Preservation Types
// ============================================================================

export interface WIACryoPreservationProject {
  standard: 'WIA-CRYO-PRESERVATION';
  version: string;
  metadata: ProjectMetadata;
  specimens: Specimen[];
  protocols: PreservationProtocol[];
  storage: StorageSystem;
  quality: QualityManagement;
  monitoring: MonitoringSystem;
  recovery: RecoveryProcedures;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: PreservationType;
  organization: Organization;
  facility: FacilityInfo;
  createdAt: string;
  status: ProjectStatus;
}

export type PreservationType = 'cell' | 'tissue' | 'organ' | 'embryo' | 'gamete' | 'blood' | 'stem-cell' | 'whole-body';

export interface Organization { name: string; type: string; country: string; contact: ContactInfo; }
export interface ContactInfo { name: string; email: string; phone?: string; }
export interface FacilityInfo { id: string; name: string; location: string; certifications: string[]; }
export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

// ============================================================================
// Specimen Types
// ============================================================================

export interface Specimen {
  id: string;
  type: SpecimenType;
  identifier: SpecimenIdentifier;
  source: SpecimenSource;
  collection: CollectionInfo;
  processing: ProcessingRecord;
  preservation: PreservationRecord;
  storage: StorageLocation;
  quality: QualityRecord;
  status: SpecimenStatus;
  history: SpecimenEvent[];
}

export type SpecimenType = 'cell-line' | 'primary-cell' | 'tissue' | 'organ' | 'embryo' | 'oocyte' | 'sperm' | 'blood-product' | 'stem-cell';

export interface SpecimenIdentifier { internalId: string; barcode?: string; rfid?: string; externalIds?: { system: string; id: string }[]; }
export interface SpecimenSource { type: 'donor' | 'patient' | 'research' | 'commercial'; donorId?: string; species?: string; tissue?: string; }
export interface CollectionInfo { date: string; location: string; method: string; collector: string; volume?: number; unit?: string; }
export interface ProcessingRecord { date: string; protocol: string; operator: string; steps: ProcessingStep[]; notes?: string; }
export interface ProcessingStep { step: number; action: string; duration?: number; temperature?: number; reagents?: string[]; }
export interface PreservationRecord { date: string; protocol: string; method: PreservationMethod; operator: string; cryoprotectant?: string; coolingRate?: number; vitrification?: boolean; finalTemperature: number; }
export type PreservationMethod = 'slow-freeze' | 'vitrification' | 'controlled-rate' | 'ultra-rapid';
export interface StorageLocation { facilityId: string; tankId: string; rackId?: string; boxId?: string; position: string; temperature: number; }
export interface QualityRecord { preFreeze?: QualityAssessment; postThaw?: QualityAssessment; periodic?: QualityAssessment[]; }
export interface QualityAssessment { date: string; viability?: number; purity?: number; sterility?: boolean; morphology?: string; tests: QualityTest[]; grade?: string; assessor: string; }
export interface QualityTest { name: string; result: string; passed: boolean; reference?: string; }
export type SpecimenStatus = 'stored' | 'in-transit' | 'thawed' | 'distributed' | 'disposed' | 'quarantine' | 'pending-qa';
export interface SpecimenEvent { id: string; type: string; timestamp: string; actor: string; description: string; }

// ============================================================================
// Protocol Types
// ============================================================================

export interface PreservationProtocol {
  id: string;
  name: string;
  version: string;
  type: PreservationMethod;
  applicability: SpecimenType[];
  steps: ProtocolStep[];
  reagents: Reagent[];
  equipment: string[];
  qualityControl: QCRequirement[];
  validation: ValidationRecord;
  approval: ApprovalRecord;
  status: 'draft' | 'approved' | 'active' | 'deprecated';
}

export interface ProtocolStep { order: number; phase: string; action: string; duration?: string; temperature?: number; criticalParameters?: string[]; notes?: string; }
export interface Reagent { name: string; concentration: string; volume: string; preparation?: string; storage: string; expiry?: string; }
export interface QCRequirement { checkpoint: string; parameter: string; acceptance: string; method: string; }
export interface ValidationRecord { date: string; specimens: number; successRate: number; report?: string; validator: string; }
export interface ApprovalRecord { approvedBy: string; approvedAt: string; reviewDate: string; }

// ============================================================================
// Storage & Quality Types
// ============================================================================

export interface StorageSystem {
  facilities: StorageFacility[];
  equipment: StorageEquipment[];
  logistics: LogisticsConfig;
  inventory: InventoryConfig;
}

export interface StorageFacility { id: string; name: string; type: 'primary' | 'backup'; location: string; capacity: number; utilization: number; status: string; }
export interface StorageEquipment { id: string; type: 'ln2-tank' | 'ln2-freezer' | 'mechanical-freezer'; model: string; location: string; capacity: number; temperature: number; monitoring: boolean; status: string; }
export interface LogisticsConfig { shipping: ShippingConfig; receiving: ReceivingConfig; chain: ChainOfCustody; }
export interface ShippingConfig { containers: string[]; carriers: string[]; monitoring: boolean; documentation: string[]; }
export interface ReceivingConfig { verification: string[]; quarantine: boolean; documentation: string[]; }
export interface ChainOfCustody { required: boolean; tracking: string; documentation: string; }
export interface InventoryConfig { tracking: string; audits: string; reconciliation: string; }

export interface QualityManagement {
  standards: string[];
  procedures: QualityProcedure[];
  testing: TestingProgram;
  deviations: DeviationManagement;
  capa: CAPASystem;
}

export interface QualityProcedure { id: string; name: string; scope: string; frequency: string; responsible: string; }
export interface TestingProgram { viability: string[]; sterility: string[]; identity: string[]; potency?: string[]; }
export interface DeviationManagement { categories: string[]; investigation: string; timeline: string; }
export interface CAPASystem { enabled: boolean; workflow: string; tracking: boolean; }

export interface MonitoringSystem {
  parameters: MonitoredParameter[];
  alerts: AlertConfig;
  logging: LoggingConfig;
  reporting: ReportingConfig;
}

export interface MonitoredParameter { name: string; unit: string; frequency: number; thresholds: { warning: number; critical: number }; }
export interface AlertConfig { channels: string[]; escalation: { levels: number; timeout: number }; }
export interface LoggingConfig { retention: string; backup: string; integrity: boolean; }
export interface ReportingConfig { automated: { type: string; frequency: string; recipients: string[] }[]; onDemand: string[]; }

export interface RecoveryProcedures {
  thawing: ThawingProtocol[];
  validation: PostThawValidation;
  distribution: DistributionProcedures;
}

export interface ThawingProtocol { id: string; name: string; specimenTypes: SpecimenType[]; steps: ProtocolStep[]; timing: string; equipment: string[]; }
export interface PostThawValidation { required: boolean; tests: string[]; acceptance: string; timeline: string; }
export interface DistributionProcedures { packaging: string; documentation: string[]; tracking: boolean; }

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig { baseURL: string; apiKey?: string; timeout?: number; }
export interface ProjectResponse { id: string; name: string; status: ProjectStatus; createdAt: string; updatedAt?: string; }
export interface ValidationResult { valid: boolean; errors?: ValidationError[]; }
export interface ValidationError { path: string; message: string; value?: unknown; }
export interface PaginatedResponse<T> { data: T[]; pagination: { total: number; limit: number; offset: number; hasMore: boolean }; }
