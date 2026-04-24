/**
 * WIA Cryo Revival Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Revival Types
// ============================================================================

export interface WIACryoRevivalProject {
  standard: 'WIA-CRYO-REVIVAL';
  version: string;
  metadata: ProjectMetadata;
  subjects: RevivalSubject[];
  protocols: RevivalProtocol[];
  technology: TechnologyStack;
  ethics: EthicsFramework;
  medical: MedicalSupport;
  monitoring: MonitoringSystem;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: ProjectType;
  organization: Organization;
  facility: FacilityInfo;
  createdAt: string;
  status: ProjectStatus;
}

export type ProjectType = 'research' | 'clinical-trial' | 'emergency' | 'elective';
export interface Organization { name: string; type: string; country: string; contact: ContactInfo; }
export interface ContactInfo { name: string; email: string; phone?: string; }
export interface FacilityInfo { id: string; name: string; location: string; capabilities: string[]; }
export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

// ============================================================================
// Revival Subject Types
// ============================================================================

export interface RevivalSubject {
  id: string;
  identifier: SubjectIdentifier;
  preservation: PreservationRecord;
  health: HealthRecord;
  consent: ConsentRecord;
  revival: RevivalRecord;
  rehabilitation: RehabilitationPlan;
  status: SubjectStatus;
  history: SubjectEvent[];
}

export interface SubjectIdentifier { internalId: string; patientId?: string; preservationId: string; }
export interface PreservationRecord { date: string; method: string; facility: string; duration: number; conditions: string; }
export interface HealthRecord { prePreservation: HealthAssessment; currentCondition?: HealthAssessment; medicalHistory: string[]; medications?: string[]; }
export interface HealthAssessment { date: string; status: string; vitals?: Record<string, number>; conditions: string[]; assessor: string; }
export interface ConsentRecord { original: { date: string; scope: string[]; document: string }; revival?: { date: string; authorized: string; document: string }; }
export interface RevivalRecord { protocol: string; scheduledDate?: string; actualDate?: string; duration?: number; outcome?: RevivalOutcome; team: string[]; notes?: string; }
export interface RevivalOutcome { success: boolean; complications?: string[]; vitals?: Record<string, number>; consciousness?: string; assessment: string; }
export interface RehabilitationPlan { phases: RehabPhase[]; goals: string[]; timeline: string; specialists: string[]; }
export interface RehabPhase { name: string; duration: string; activities: string[]; milestones: string[]; }
export type SubjectStatus = 'preserved' | 'scheduled' | 'in-revival' | 'recovery' | 'rehabilitation' | 'completed' | 'failed';
export interface SubjectEvent { id: string; type: string; timestamp: string; actor: string; description: string; }

// ============================================================================
// Protocol & Technology Types
// ============================================================================

export interface RevivalProtocol {
  id: string;
  name: string;
  version: string;
  applicability: string[];
  phases: ProtocolPhase[];
  requirements: RequirementSpec;
  safety: SafetyProtocol;
  validation: ValidationRecord;
  status: 'draft' | 'approved' | 'active' | 'deprecated';
}

export interface ProtocolPhase { order: number; name: string; duration: string; steps: ProtocolStep[]; checkpoints: string[]; }
export interface ProtocolStep { step: number; action: string; parameters: Record<string, unknown>; monitoring: string[]; criticalPoints: string[]; }
export interface RequirementSpec { equipment: string[]; personnel: PersonnelRequirement[]; facilities: string[]; supplies: string[]; }
export interface PersonnelRequirement { role: string; count: number; qualifications: string[]; }
export interface SafetyProtocol { risks: Risk[]; mitigations: string[]; emergencyProcedures: string[]; abort: AbortCriteria; }
export interface Risk { type: string; severity: string; probability: string; mitigation: string; }
export interface AbortCriteria { triggers: string[]; procedure: string; recovery: string; }
export interface ValidationRecord { trials: number; successRate: number; lastValidation: string; approvedBy: string; }

export interface TechnologyStack {
  warming: WarmingTechnology;
  perfusion: PerfusionSystem;
  monitoring: MonitoringTech;
  support: LifeSupport;
  imaging: ImagingCapabilities;
}

export interface WarmingTechnology { method: string; rate: string; uniformity: string; monitoring: string[]; }
export interface PerfusionSystem { type: string; solutions: string[]; pressure: string; temperature: string; oxygenation: string; }
export interface MonitoringTech { vitals: string[]; imaging: string[]; biochemical: string[]; neural: string[]; }
export interface LifeSupport { respiratory: string; cardiovascular: string; temperature: string; nutrition: string; }
export interface ImagingCapabilities { modalities: string[]; realTime: boolean; ai: boolean; }

export interface EthicsFramework {
  principles: string[];
  consent: ConsentRequirements;
  review: EthicsReview;
  rights: SubjectRights;
}

export interface ConsentRequirements { prePreservation: string[]; revival: string[]; ongoing: string[]; withdrawal: string; }
export interface EthicsReview { board: string; approval: string; monitoring: string; reporting: string; }
export interface SubjectRights { information: string[]; autonomy: string[]; privacy: string[]; welfare: string[]; }

export interface MedicalSupport {
  team: MedicalTeam;
  facilities: MedicalFacility[];
  emergency: EmergencyProtocol;
  followUp: FollowUpCare;
}

export interface MedicalTeam { lead: string; specialists: { role: string; name: string }[]; support: string[]; }
export interface MedicalFacility { type: string; capabilities: string[]; proximity: string; }
export interface EmergencyProtocol { procedures: string[]; resources: string[]; contacts: string[]; }
export interface FollowUpCare { schedule: string; assessments: string[]; duration: string; }

export interface MonitoringSystem {
  realTime: RealTimeMonitoring;
  alerts: AlertConfig;
  documentation: DocumentationConfig;
  reporting: ReportingConfig;
}

export interface RealTimeMonitoring { parameters: string[]; frequency: number; dashboard: string; }
export interface AlertConfig { thresholds: { parameter: string; warning: number; critical: number }[]; channels: string[]; escalation: string; }
export interface DocumentationConfig { automated: boolean; retention: string; format: string; }
export interface ReportingConfig { internal: string; regulatory: string; research: string; }

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig { baseURL: string; apiKey?: string; timeout?: number; }
export interface ProjectResponse { id: string; name: string; status: ProjectStatus; createdAt: string; updatedAt?: string; }
export interface ValidationResult { valid: boolean; errors?: ValidationError[]; }
export interface ValidationError { path: string; message: string; value?: unknown; }
export interface PaginatedResponse<T> { data: T[]; pagination: { total: number; limit: number; offset: number; hasMore: boolean }; }
