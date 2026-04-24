# CRYO-REVIVAL Phase 1: Data Format Specification

## Overview

This document defines the data structures for revival procedures, team management, monitoring data, and outcome tracking.

## Core Interfaces

### Revival Procedure

```typescript
interface RevivalProcedure {
  procedureId: string;                   // "REVIVAL-2075-001"
  subjectId: string;                     // Reference to preserved subject
  identityId: string;                    // CRYO-IDENTITY reference
  consentId: string;                     // CRYO-CONSENT reference

  status: RevivalStatus;
  currentStage: RevivalStage;

  preparation: RevivalPreparation;
  stages: StageRecord[];
  monitoring: MonitoringData;
  team: RevivalTeam;
  outcome: RevivalOutcome;

  facility: FacilityInfo;
  startedAt: string;                     // ISO 8601
  completedAt?: string;
  lastUpdated: string;
}

type RevivalStatus =
  | 'SCHEDULED'
  | 'PREPARING'
  | 'IN_PROGRESS'
  | 'PAUSED'
  | 'COMPLETED'
  | 'FAILED'
  | 'ABORTED'
  | 'RE_PRESERVED';

type RevivalStage =
  | 'PRE_REVIVAL'
  | 'WARMING'
  | 'CRYOPROTECTANT_REMOVAL'
  | 'REHYDRATION'
  | 'CARDIOVASCULAR_RESTART'
  | 'RESPIRATORY_ACTIVATION'
  | 'NEUROLOGICAL_RESTORATION'
  | 'STABILIZATION'
  | 'POST_REVIVAL_CARE';
```

### Revival Preparation

```typescript
interface RevivalPreparation {
  assessmentId: string;
  scheduledDate: string;

  preRevivalAssessment: PreRevivalAssessment;
  facilityReadiness: FacilityReadiness;
  teamAssembly: TeamAssembly;
  equipmentCheck: EquipmentCheck;
  notifications: NotificationRecord[];

  approvals: {
    medicalDirector: Approval;
    facilityManager: Approval;
    guardianConsent: Approval;
  };

  readinessScore: number;                // 0-100
  readyForRevival: boolean;
}

interface PreRevivalAssessment {
  assessmentDate: string;
  assessedBy: string;

  preservationStatus: {
    duration: string;                    // Duration since preservation
    storageFacility: string;
    storageConditions: string;
    integrityScore: number;              // 0-100
    transferHistory: TransferRecord[];
  };

  tissueCondition: {
    overallScore: number;
    brainTissue: TissueAssessment;
    cardiacTissue: TissueAssessment;
    vascularSystem: TissueAssessment;
    otherOrgans: TissueAssessment[];
  };

  cryoprotectantStatus: {
    type: string;
    concentration: number;
    distribution: string;
    estimatedRemovalTime: string;
  };

  riskFactors: RiskFactor[];
  recommendation: 'PROCEED' | 'DELAY' | 'CONTRAINDICATED';
}

interface TissueAssessment {
  tissue: string;
  condition: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'CRITICAL';
  integrityScore: number;
  concerns: string[];
  specialConsiderations: string[];
}

interface FacilityReadiness {
  facilityId: string;
  facilityName: string;

  revivalSuite: {
    available: boolean;
    lastCalibration: string;
    temperatureControl: EquipmentStatus;
    perfusionSystem: EquipmentStatus;
    monitoring: EquipmentStatus;
  };

  operatingRoom: {
    available: boolean;
    sterilized: boolean;
    equipment: EquipmentStatus[];
  };

  icuBeds: {
    available: number;
    reserved: number;
  };

  powerSystems: {
    primary: PowerStatus;
    backup: PowerStatus;
    emergencyGenerator: PowerStatus;
  };

  readinessChecklist: ChecklistItem[];
  overallReady: boolean;
}

interface EquipmentStatus {
  equipmentId: string;
  name: string;
  status: 'OPERATIONAL' | 'STANDBY' | 'MAINTENANCE' | 'FAILED';
  lastCheck: string;
  calibrationValid: boolean;
  notes?: string;
}
```

### Revival Stages

```typescript
interface StageRecord {
  stage: RevivalStage;
  stageNumber: number;

  startedAt: string;
  completedAt?: string;
  duration?: string;

  status: 'PENDING' | 'IN_PROGRESS' | 'COMPLETED' | 'FAILED' | 'SKIPPED';

  parameters: StageParameters;
  monitoring: StageMonitoring;
  interventions: Intervention[];
  complications: Complication[];

  teamOnDuty: TeamMember[];
  notes: string[];

  outcome: StageOutcome;
}

interface WarmingStageParameters {
  stage: 'WARMING';

  targetTemperature: number;             // Celsius
  warmingRate: number;                   // Degrees per minute
  warmingMethod: 'PERFUSION' | 'EXTERNAL' | 'HYBRID';

  temperatureProfile: TemperaturePoint[];
  phaseTransitionMonitoring: boolean;
  iceCrystalDetection: SensorReading[];
}

interface CryoprotectantRemovalParameters {
  stage: 'CRYOPROTECTANT_REMOVAL';

  initialConcentration: number;          // Percentage
  targetConcentration: number;
  removalRate: number;                   // Per hour
  perfusionSolution: string;

  concentrationProfile: ConcentrationPoint[];
  osmoticPressure: number[];
  cellularIntegrity: number[];
}

interface CardiovascularRestartParameters {
  stage: 'CARDIOVASCULAR_RESTART';

  restartMethod: 'ELECTRICAL' | 'MECHANICAL' | 'PHARMACOLOGICAL' | 'COMBINED';
  defibrillationAttempts: DefibrillationAttempt[];
  pacingParameters?: PacingConfig;

  bloodPressureTarget: BloodPressureRange;
  heartRateTarget: HeartRateRange;
  rhythmTarget: 'SINUS' | 'PACED';

  vasoactiveAgents: Medication[];
  fluidResuscitation: FluidRecord[];
}

interface NeurologicalRestorationParameters {
  stage: 'NEUROLOGICAL_RESTORATION';

  eegMonitoring: EEGRecord[];
  pupilResponse: PupilAssessment[];
  reflexAssessment: ReflexAssessment[];

  consciousnessScale: ConsciousnessAssessment[];
  sedationLevel: SedationRecord[];

  neuroprotectiveAgents: Medication[];
  intracranialPressure?: number[];
}

type StageParameters =
  | WarmingStageParameters
  | CryoprotectantRemovalParameters
  | RehydrationParameters
  | CardiovascularRestartParameters
  | RespiratoryActivationParameters
  | NeurologicalRestorationParameters
  | StabilizationParameters
  | PostRevivalCareParameters;
```

### Monitoring Data

```typescript
interface MonitoringData {
  vitalSigns: VitalSignsRecord[];
  laboratoryResults: LabResult[];
  imagingStudies: ImagingStudy[];
  specializedMonitoring: SpecializedMonitor[];

  alerts: Alert[];
  trends: TrendAnalysis[];
}

interface VitalSignsRecord {
  timestamp: string;
  temperature: number;                   // Celsius
  heartRate?: number;                    // BPM
  bloodPressure?: {
    systolic: number;
    diastolic: number;
    mean: number;
  };
  respiratoryRate?: number;
  oxygenSaturation?: number;             // Percentage
  endTidalCO2?: number;
  centralVenousPressure?: number;
  cardiacOutput?: number;

  source: 'AUTOMATED' | 'MANUAL';
  verified: boolean;
}

interface LabResult {
  labId: string;
  timestamp: string;
  category: LabCategory;

  tests: LabTest[];
  interpretation: string;
  actionRequired: boolean;
  criticalValues: string[];
}

type LabCategory =
  | 'BLOOD_GAS'
  | 'CHEMISTRY'
  | 'HEMATOLOGY'
  | 'COAGULATION'
  | 'TOXICOLOGY'
  | 'BIOMARKERS';

interface LabTest {
  testName: string;
  value: number | string;
  unit: string;
  referenceRange: string;
  flag?: 'LOW' | 'HIGH' | 'CRITICAL_LOW' | 'CRITICAL_HIGH';
}

interface Alert {
  alertId: string;
  timestamp: string;
  severity: 'INFO' | 'WARNING' | 'CRITICAL' | 'EMERGENCY';
  category: string;
  message: string;
  source: string;

  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: string;

  resolved: boolean;
  resolvedBy?: string;
  resolution?: string;
}
```

### Revival Team

```typescript
interface RevivalTeam {
  teamId: string;
  assembledAt: string;

  director: TeamMember;
  members: TeamMember[];

  shifts: ShiftSchedule[];
  currentShift: string;

  communications: CommunicationLog[];
  briefings: BriefingRecord[];
}

interface TeamMember {
  memberId: string;
  name: string;
  role: TeamRole;
  specialization: string;

  credentials: {
    licenseNumber: string;
    wiaRevivalCertification: string;
    certificationExpiry: string;
    additionalCertifications: string[];
  };

  experience: {
    totalRevivalsPart ticipated: number;
    yearsExperience: number;
    specialtyExperience: string[];
  };

  contactInfo: EncryptedData;
  onSite: boolean;
  currentAssignment?: string;
}

type TeamRole =
  | 'REVIVAL_DIRECTOR'
  | 'CRYONICS_SPECIALIST'
  | 'CARDIOVASCULAR_SURGEON'
  | 'NEUROLOGIST'
  | 'ANESTHESIOLOGIST'
  | 'PERFUSIONIST'
  | 'ICU_NURSE'
  | 'MONITORING_SPECIALIST'
  | 'SUPPORT_STAFF';

interface ShiftSchedule {
  shiftId: string;
  shiftNumber: number;
  startTime: string;
  endTime: string;

  teamOnDuty: {
    memberId: string;
    role: TeamRole;
    responsibilities: string[];
  }[];

  handoverNotes?: string;
  incidentsDuringShift: string[];
}
```

### Revival Outcome

```typescript
interface RevivalOutcome {
  outcomeId: string;
  procedureId: string;

  result: RevivalResult;
  survivalStatus: SurvivalStatus;

  functionalAssessment: {
    neurologicalFunction: FunctionScore;
    cardiovascularFunction: FunctionScore;
    respiratoryFunction: FunctionScore;
    renalFunction: FunctionScore;
    hepaticFunction: FunctionScore;
    motorFunction: FunctionScore;
    cognitiveFunction: FunctionScore;
  };

  complications: ComplicationSummary[];
  interventionsRequired: InterventionSummary[];

  recoveryPrognosis: RecoveryPrognosis;
  followUpPlan: FollowUpPlan;

  qualityMetrics: QualityMetrics;
  documentedAt: string;
  documentedBy: string;
}

type RevivalResult =
  | 'FULL_SUCCESS'                       // All functions restored
  | 'PARTIAL_SUCCESS'                    // Some limitations
  | 'SURVIVAL_WITH_DEFICITS'             // Significant deficits
  | 'FAILURE_REPRESERVED'                // Failed, re-preserved
  | 'FAILURE_DEATH'                      // Failed, not recoverable
  | 'ABORTED';                           // Procedure stopped

type SurvivalStatus =
  | 'ALIVE_STABLE'
  | 'ALIVE_CRITICAL'
  | 'ALIVE_RECOVERING'
  | 'REPRESERVED'
  | 'DECEASED';

interface FunctionScore {
  category: string;
  score: number;                         // 0-100
  assessment: 'NORMAL' | 'MILD_IMPAIRMENT' | 'MODERATE' | 'SEVERE' | 'ABSENT';
  details: string;
  rehabilitationNeeded: boolean;
}

interface RecoveryPrognosis {
  overallPrognosis: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'GUARDED' | 'POOR';
  expectedRecoveryTime: string;

  neurologicalRecovery: {
    expected: string;
    timeline: string;
    uncertainties: string[];
  };

  physicalRecovery: {
    expected: string;
    timeline: string;
    rehabilitationRequired: string[];
  };

  psychologicalConsiderations: {
    adjustmentExpected: string;
    supportRequired: string[];
    riskFactors: string[];
  };
}

interface QualityMetrics {
  procedureDuration: string;
  timeToCardiacRestart: string;
  timeToConsciousness: string;
  complicationCount: number;
  interventionCount: number;

  protocolAdherence: number;             // Percentage
  documentationCompleteness: number;
  teamPerformanceScore: number;

  benchmarkComparison: {
    metric: string;
    thisCase: number;
    benchmark: number;
    deviation: number;
  }[];
}
```

### Complications and Interventions

```typescript
interface Complication {
  complicationId: string;
  timestamp: string;
  stage: RevivalStage;

  type: ComplicationType;
  severity: 'MILD' | 'MODERATE' | 'SEVERE' | 'LIFE_THREATENING';
  description: string;

  detectedBy: string;
  response: ComplicationResponse;

  resolved: boolean;
  resolutionTime?: string;
  residualEffects?: string[];
}

type ComplicationType =
  | 'CARDIAC_ARRHYTHMIA'
  | 'CARDIAC_ARREST'
  | 'HEMORRHAGE'
  | 'COAGULOPATHY'
  | 'CEREBRAL_EDEMA'
  | 'SEIZURE'
  | 'RESPIRATORY_FAILURE'
  | 'RENAL_FAILURE'
  | 'METABOLIC_DERANGEMENT'
  | 'INFECTION'
  | 'REPERFUSION_INJURY'
  | 'TISSUE_DAMAGE'
  | 'OTHER';

interface Intervention {
  interventionId: string;
  timestamp: string;
  stage: RevivalStage;

  type: InterventionType;
  indication: string;
  description: string;

  performedBy: string;
  assistedBy: string[];

  outcome: 'SUCCESSFUL' | 'PARTIALLY_SUCCESSFUL' | 'UNSUCCESSFUL';
  complications?: string[];
  notes: string;
}

type InterventionType =
  | 'MEDICATION_ADMINISTRATION'
  | 'DEFIBRILLATION'
  | 'CARDIOVERSION'
  | 'PACING'
  | 'INTUBATION'
  | 'MECHANICAL_VENTILATION'
  | 'SURGICAL_PROCEDURE'
  | 'BLOOD_TRANSFUSION'
  | 'DIALYSIS'
  | 'TEMPERATURE_INTERVENTION'
  | 'PERFUSION_ADJUSTMENT'
  | 'OTHER';
```

### Re-Preservation Protocol

```typescript
interface RePreservationDecision {
  decisionId: string;
  procedureId: string;
  timestamp: string;

  trigger: RePreservationTrigger;
  criteria: RePreservationCriteria;

  decisionMakers: {
    memberId: string;
    role: string;
    vote: 'PROCEED' | 'OPPOSE' | 'ABSTAIN';
  }[];

  decision: 'REPRESERVE' | 'CONTINUE' | 'TERMINATE';
  rationale: string;

  guardianNotification: {
    notified: boolean;
    notifiedAt: string;
    response?: string;
  };

  execution?: RePreservationExecution;
}

type RePreservationTrigger =
  | 'IRREVERSIBLE_CARDIAC_FAILURE'
  | 'IRREVERSIBLE_BRAIN_DAMAGE'
  | 'MULTIPLE_ORGAN_FAILURE'
  | 'SUBJECT_REQUEST'
  | 'GUARDIAN_REQUEST'
  | 'PROTOCOL_THRESHOLD';

interface RePreservationCriteria {
  criterionMet: {
    criterion: string;
    met: boolean;
    evidence: string;
  }[];

  overallScore: number;
  thresholdMet: boolean;
  timeConstraint: string;
}

interface RePreservationExecution {
  startedAt: string;
  completedAt?: string;

  coolingMethod: string;
  coolingRate: number;
  targetTemperature: number;

  cryoprotectantAdministration: {
    type: string;
    concentration: number;
    administrationTime: string;
  };

  outcome: 'SUCCESSFUL' | 'PARTIAL' | 'FAILED';
  notes: string;
}
```

## JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cryo-revival/v1",
  "title": "RevivalProcedure",
  "type": "object",
  "required": [
    "procedureId",
    "subjectId",
    "status",
    "currentStage"
  ],
  "properties": {
    "procedureId": {
      "type": "string",
      "pattern": "^REVIVAL-[0-9]{4}-[0-9]{3}$"
    },
    "status": {
      "type": "string",
      "enum": [
        "SCHEDULED",
        "PREPARING",
        "IN_PROGRESS",
        "PAUSED",
        "COMPLETED",
        "FAILED",
        "ABORTED",
        "RE_PRESERVED"
      ]
    },
    "currentStage": {
      "type": "string",
      "enum": [
        "PRE_REVIVAL",
        "WARMING",
        "CRYOPROTECTANT_REMOVAL",
        "REHYDRATION",
        "CARDIOVASCULAR_RESTART",
        "RESPIRATORY_ACTIVATION",
        "NEUROLOGICAL_RESTORATION",
        "STABILIZATION",
        "POST_REVIVAL_CARE"
      ]
    }
  }
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
