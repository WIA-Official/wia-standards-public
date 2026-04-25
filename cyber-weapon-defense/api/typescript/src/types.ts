/**
 * WIA Cyber Weapon Defense Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cyber Weapon Defense Types
// ============================================================================

export interface WIACyberWeaponDefenseProject {
  standard: 'WIA-CYBER-WEAPON-DEFENSE';
  version: string;
  metadata: ProjectMetadata;
  threats: ThreatIntelligence;
  defenses: DefenseCapabilities;
  detection: DetectionSystem;
  response: ResponseFramework;
  attribution: AttributionCapabilities;
  compliance: ComplianceFramework;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  classification: ClassificationLevel;
  organization: Organization;
  scope: DefenseScope;
  createdAt: string;
  status: ProjectStatus;
}

export type ClassificationLevel = 'unclassified' | 'confidential' | 'secret' | 'top-secret';
export interface Organization { name: string; type: 'government' | 'military' | 'critical-infrastructure' | 'enterprise'; country: string; sector?: string; contact: ContactInfo; }
export interface ContactInfo { name: string; email: string; phone?: string; clearance?: string; }
export interface DefenseScope { assets: string[]; networks: string[]; criticality: 'low' | 'medium' | 'high' | 'critical'; }
export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'classified';

// ============================================================================
// Threat Intelligence Types
// ============================================================================

export interface ThreatIntelligence {
  sources: IntelSource[];
  actors: ThreatActor[];
  weapons: CyberWeapon[];
  campaigns: ThreatCampaign[];
  indicators: ThreatIndicator[];
}

export interface IntelSource { id: string; name: string; type: 'osint' | 'sigint' | 'humint' | 'techint' | 'commercial'; reliability: 'A' | 'B' | 'C' | 'D' | 'E' | 'F'; classification: ClassificationLevel; }
export interface ThreatActor { id: string; name: string; aliases?: string[]; type: ActorType; origin?: string; motivation: Motivation[]; capabilities: string[]; targets: string[]; ttps: string[]; confidence: 'low' | 'medium' | 'high'; }
export type ActorType = 'nation-state' | 'apt' | 'criminal' | 'hacktivist' | 'insider' | 'terrorist';
export type Motivation = 'espionage' | 'sabotage' | 'financial' | 'ideological' | 'destruction';
export interface CyberWeapon { id: string; name: string; type: WeaponType; category: WeaponCategory; capabilities: string[]; targets: string[]; indicators: string[]; mitigations: string[]; firstSeen?: string; attribution?: string; }
export type WeaponType = 'malware' | 'exploit' | 'implant' | 'destructive' | 'wiper' | 'ransomware' | 'apt-toolkit';
export type WeaponCategory = 'access' | 'persistence' | 'lateral-movement' | 'exfiltration' | 'destruction' | 'disruption';
export interface ThreatCampaign { id: string; name: string; actor?: string; startDate: string; endDate?: string; targets: string[]; weapons: string[]; objectives: string[]; status: 'active' | 'dormant' | 'concluded'; }
export interface ThreatIndicator { id: string; type: IOCType; value: string; confidence: number; firstSeen: string; lastSeen?: string; campaigns?: string[]; tags?: string[]; }
export type IOCType = 'ip' | 'domain' | 'url' | 'hash' | 'email' | 'mutex' | 'registry' | 'certificate' | 'yara';

// ============================================================================
// Defense & Detection Types
// ============================================================================

export interface DefenseCapabilities {
  layers: DefenseLayer[];
  controls: SecurityControl[];
  technologies: DefenseTechnology[];
  resilience: ResilienceCapabilities;
}

export interface DefenseLayer { name: string; scope: string; controls: string[]; monitoring: string[]; effectiveness: number; }
export interface SecurityControl { id: string; name: string; type: ControlType; implementation: 'technical' | 'administrative' | 'physical'; status: 'implemented' | 'planned' | 'partial'; effectiveness: number; }
export type ControlType = 'preventive' | 'detective' | 'corrective' | 'deterrent' | 'compensating';
export interface DefenseTechnology { id: string; name: string; category: string; vendor?: string; capabilities: string[]; coverage: string[]; integration: string[]; }
export interface ResilienceCapabilities { redundancy: string[]; recovery: RecoveryCapability; continuity: ContinuityPlan; isolation: IsolationCapabilities; }
export interface RecoveryCapability { rto: number; rpo: number; procedures: string[]; testing: string; }
export interface ContinuityPlan { scenarios: string[]; procedures: string[]; resources: string[]; testing: string; }
export interface IsolationCapabilities { segmentation: string[]; airGap: boolean; killSwitch: boolean; quarantine: string; }

export interface DetectionSystem {
  capabilities: DetectionCapability[];
  sensors: Sensor[];
  analytics: AnalyticsCapability[];
  correlation: CorrelationEngine;
  hunting: ThreatHunting;
}

export interface DetectionCapability { name: string; scope: string; technique: string; coverage: number; latency: string; }
export interface Sensor { id: string; type: string; location: string; coverage: string[]; status: 'active' | 'maintenance' | 'offline'; }
export interface AnalyticsCapability { type: 'signature' | 'behavioral' | 'ml' | 'anomaly'; scope: string; effectiveness: number; tuning: string; }
export interface CorrelationEngine { rules: number; sources: string[]; latency: string; automation: number; }
export interface ThreatHunting { frequency: string; methodologies: string[]; tools: string[]; team: string; }

// ============================================================================
// Response & Attribution Types
// ============================================================================

export interface ResponseFramework {
  playbooks: ResponsePlaybook[];
  team: ResponseTeam;
  escalation: EscalationPath;
  communication: CommunicationPlan;
  recovery: RecoveryProcedures;
}

export interface ResponsePlaybook { id: string; name: string; trigger: string; severity: string; steps: ResponseStep[]; automation: number; sla: string; }
export interface ResponseStep { order: number; action: string; responsible: string; tools?: string[]; criteria?: string; escalation?: string; }
export interface ResponseTeam { structure: string; roles: { role: string; responsibilities: string[] }[]; onCall: string; training: string; }
export interface EscalationPath { levels: { level: number; criteria: string; contacts: string[]; timeframe: string }[]; }
export interface CommunicationPlan { internal: { channels: string[]; templates: string[] }; external: { stakeholders: string[]; protocols: string[] }; authorities: { agencies: string[]; procedures: string[] }; }
export interface RecoveryProcedures { containment: string[]; eradication: string[]; recovery: string[]; validation: string[]; }

export interface AttributionCapabilities {
  methodology: AttributionMethodology;
  evidence: EvidenceCollection;
  analysis: AttributionAnalysis;
  confidence: ConfidenceFramework;
}

export interface AttributionMethodology { approaches: string[]; standards: string[]; legal: string[]; }
export interface EvidenceCollection { forensics: string[]; chain: string; preservation: string; }
export interface AttributionAnalysis { techniques: string[]; correlation: string[]; validation: string; }
export interface ConfidenceFramework { levels: { level: string; criteria: string[] }[]; reporting: string; }

export interface ComplianceFramework {
  regulations: string[];
  standards: string[];
  reporting: ComplianceReporting;
  audits: AuditProgram;
}

export interface ComplianceReporting { mandatory: { authority: string; frequency: string; format: string }[]; voluntary: string[]; }
export interface AuditProgram { internal: { frequency: string; scope: string[] }; external: { frequency: string; auditor: string }; }

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig { baseURL: string; apiKey?: string; timeout?: number; }
export interface ProjectResponse { id: string; name: string; status: ProjectStatus; createdAt: string; updatedAt?: string; }
export interface ValidationResult { valid: boolean; errors?: ValidationError[]; }
export interface ValidationError { path: string; message: string; value?: unknown; }
export interface PaginatedResponse<T> { data: T[]; pagination: { total: number; limit: number; offset: number; hasMore: boolean }; }
