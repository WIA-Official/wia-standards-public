# Chapter 3: Clinical Decision Support Data Formats

## Healthcare Data Models and Interoperability Standards

### 3.1 Healthcare Data Standards Overview

Clinical decision support systems require seamless integration with healthcare data from multiple sources. This chapter defines the data formats, standards, and models that enable interoperability and effective clinical decision support.

```typescript
// Healthcare Data Standards Framework
interface HealthcareDataStandards {
  version: '1.0.0';

  interoperabilityStandards: {
    hl7Fhir: {
      version: 'R4 (4.0.1)';
      scope: 'Primary data exchange standard';
      resources: FHIRResource[];
      adoption: 'Mandated by US regulations (21st Century Cures)';
    };
    hl7V2: {
      version: '2.5.1+';
      scope: 'Legacy message exchange';
      usage: 'Lab results, ADT, orders';
      status: 'Still widely used, gradually migrating to FHIR';
    };
    cda: {
      version: 'R2';
      scope: 'Clinical document exchange';
      usage: 'CCDs, discharge summaries';
      status: 'Being superseded by FHIR documents';
    };
    dicom: {
      version: '2024a';
      scope: 'Medical imaging';
      usage: 'All imaging modalities, SR for reports';
    };
  };

  terminologyStandards: {
    snomedCt: {
      scope: 'Clinical terminology';
      coverage: 'Diagnoses, procedures, findings, organisms';
      concepts: '350,000+ concepts';
    };
    icd: {
      versions: ['ICD-10-CM', 'ICD-11'];
      scope: 'Diagnosis coding';
      usage: 'Billing, epidemiology';
    };
    loinc: {
      scope: 'Laboratory and clinical observations';
      codes: '90,000+ codes';
      usage: 'Lab tests, vital signs, documents';
    };
    rxnorm: {
      scope: 'Medications';
      coverage: 'Drug names, ingredients, forms, strengths';
      usage: 'Prescription data, drug databases';
    };
    cpt: {
      scope: 'Procedure coding';
      usage: 'Billing, service documentation';
    };
  };
}

// FHIR Resources for CDSS
const fhirResourcesForCDSS: FHIRResourceCoverage = {
  patientData: {
    Patient: {
      usage: 'Demographics, identifiers',
      cdssRelevance: 'Patient identification, age-based rules',
      keyElements: ['identifier', 'name', 'birthDate', 'gender', 'address']
    },
    Condition: {
      usage: 'Problems, diagnoses',
      cdssRelevance: 'Rule triggers, risk factors, contraindications',
      keyElements: ['code', 'clinicalStatus', 'verificationStatus', 'onset', 'severity']
    },
    MedicationRequest: {
      usage: 'Prescription orders',
      cdssRelevance: 'Drug interaction checking, dosing recommendations',
      keyElements: ['medication', 'dosageInstruction', 'status', 'intent']
    },
    MedicationStatement: {
      usage: 'Medication usage records',
      cdssRelevance: 'Active medication list for interaction checking',
      keyElements: ['medication', 'status', 'effectivePeriod', 'dosage']
    },
    AllergyIntolerance: {
      usage: 'Allergies and adverse reactions',
      cdssRelevance: 'Allergy alerts, contraindication checking',
      keyElements: ['code', 'clinicalStatus', 'type', 'category', 'criticality', 'reaction']
    },
    Observation: {
      usage: 'Lab results, vital signs, assessments',
      cdssRelevance: 'Rule triggers, trend analysis, risk scores',
      keyElements: ['code', 'value', 'effectiveDateTime', 'status', 'interpretation']
    },
    Procedure: {
      usage: 'Procedures performed',
      cdssRelevance: 'Care history, follow-up recommendations',
      keyElements: ['code', 'status', 'performedDateTime', 'outcome']
    },
    Immunization: {
      usage: 'Vaccination records',
      cdssRelevance: 'Immunization recommendations, due dates',
      keyElements: ['vaccineCode', 'occurrenceDateTime', 'status', 'doseQuantity']
    }
  },

  clinicalResources: {
    Encounter: {
      usage: 'Healthcare visits',
      cdssRelevance: 'Context for recommendations, visit type-specific rules',
      keyElements: ['status', 'class', 'type', 'period', 'reasonCode']
    },
    DiagnosticReport: {
      usage: 'Lab reports, imaging reports',
      cdssRelevance: 'Test interpretation support, abnormal flagging',
      keyElements: ['code', 'status', 'effectiveDateTime', 'conclusion', 'result']
    },
    CarePlan: {
      usage: 'Treatment plans',
      cdssRelevance: 'Care pathway adherence, intervention scheduling',
      keyElements: ['status', 'intent', 'category', 'activity', 'goal']
    },
    Goal: {
      usage: 'Clinical goals',
      cdssRelevance: 'Outcome tracking, goal-directed recommendations',
      keyElements: ['lifecycleStatus', 'description', 'target', 'achievementStatus']
    },
    RiskAssessment: {
      usage: 'Risk evaluations',
      cdssRelevance: 'Risk scores, predictive model outputs',
      keyElements: ['code', 'prediction', 'mitigation', 'note']
    }
  },

  cdssSpecificResources: {
    Library: {
      usage: 'Knowledge artifacts (CQL libraries)',
      cdssRelevance: 'Clinical logic definitions',
      keyElements: ['content', 'type', 'dataRequirement', 'parameter']
    },
    PlanDefinition: {
      usage: 'Protocols, order sets, guidelines',
      cdssRelevance: 'Computable clinical guidelines',
      keyElements: ['action', 'goal', 'trigger', 'condition', 'dynamicValue']
    },
    ActivityDefinition: {
      usage: 'Order templates',
      cdssRelevance: 'Recommended orders, activities',
      keyElements: ['code', 'timing', 'dosage', 'productReference']
    },
    RequestGroup: {
      usage: 'Grouped recommendations',
      cdssRelevance: 'CDSS output - bundled recommendations',
      keyElements: ['status', 'intent', 'action', 'priority']
    },
    GuidanceResponse: {
      usage: 'CDS response',
      cdssRelevance: 'Standard CDSS response format',
      keyElements: ['status', 'outputParameters', 'result', 'dataRequirement']
    }
  }
};
```

### 3.2 CDSS Data Models

```typescript
// Core CDSS Data Models
interface CDSSPatientData {
  demographics: PatientDemographics;
  problems: ProblemList;
  medications: MedicationList;
  allergies: AllergyList;
  vitals: VitalSignHistory;
  labs: LaboratoryResults;
  procedures: ProcedureHistory;
  immunizations: ImmunizationRecord;
  familyHistory: FamilyHistoryRecord;
  socialHistory: SocialHistoryRecord;
}

// Patient Demographics
interface PatientDemographics {
  patientId: string;
  mrn: string;
  name: HumanName;
  birthDate: Date;
  age: AgeValue;
  gender: 'male' | 'female' | 'other' | 'unknown';
  administrativeGender: string;
  race: CodeableConcept[];
  ethnicity: CodeableConcept[];
  addresses: Address[];
  telecom: ContactPoint[];
  maritalStatus: CodeableConcept;
  communication: Communication[];
  contacts: PatientContact[];
}

interface AgeValue {
  years: number;
  months?: number;
  days?: number;
  ageGroup: 'NEONATE' | 'INFANT' | 'CHILD' | 'ADOLESCENT' | 'ADULT' | 'ELDERLY';
}

// Problem List for Clinical Context
interface ProblemList {
  active: ClinicalProblem[];
  inactive: ClinicalProblem[];
  resolved: ClinicalProblem[];
}

interface ClinicalProblem {
  id: string;
  code: {
    system: 'http://snomed.info/sct' | 'http://hl7.org/fhir/sid/icd-10-cm';
    code: string;
    display: string;
  };
  clinicalStatus: 'active' | 'recurrence' | 'relapse' | 'inactive' | 'remission' | 'resolved';
  verificationStatus: 'unconfirmed' | 'provisional' | 'differential' | 'confirmed';
  severity: 'mild' | 'moderate' | 'severe';
  category: 'problem-list-item' | 'encounter-diagnosis';
  onset: {
    dateTime?: Date;
    age?: AgeValue;
    period?: Period;
    string?: string;
  };
  recordedDate: Date;
  note: string[];
}

// Medication Data Model
interface MedicationList {
  active: MedicationRecord[];
  completed: MedicationRecord[];
  discontinued: MedicationRecord[];
}

interface MedicationRecord {
  id: string;
  medication: {
    rxnormCode: string;
    display: string;
    ingredients: Ingredient[];
    drugClass: DrugClass[];
  };
  status: 'active' | 'completed' | 'entered-in-error' | 'intended' | 'stopped' | 'on-hold';
  intent: 'proposal' | 'plan' | 'order' | 'instance-order' | 'option';
  dosage: Dosage[];
  route: CodeableConcept;
  frequency: Frequency;
  duration: Duration;
  prescribedDate: Date;
  prescriber: Reference;
  indication: CodeableConcept[];
  substitution: SubstitutionInfo;
}

interface Dosage {
  sequence: number;
  text: string;
  timing: Timing;
  route: CodeableConcept;
  method: CodeableConcept;
  doseQuantity: Quantity;
  maxDosePerPeriod: Ratio;
  maxDosePerAdministration: Quantity;
  maxDosePerLifetime: Quantity;
}

// Allergy and Intolerance Data
interface AllergyList {
  allergies: AllergyRecord[];
}

interface AllergyRecord {
  id: string;
  clinicalStatus: 'active' | 'inactive' | 'resolved';
  verificationStatus: 'unconfirmed' | 'confirmed' | 'refuted';
  type: 'allergy' | 'intolerance';
  category: ('food' | 'medication' | 'environment' | 'biologic')[];
  criticality: 'low' | 'high' | 'unable-to-assess';
  code: {
    system: string;
    code: string;
    display: string;
  };
  reactions: AllergyReaction[];
  onsetDateTime: Date;
  recordedDate: Date;
  recorder: Reference;
  note: string[];
}

interface AllergyReaction {
  substance: CodeableConcept;
  manifestation: CodeableConcept[];
  severity: 'mild' | 'moderate' | 'severe';
  exposureRoute: CodeableConcept;
  note: string[];
}

// Laboratory Results Model
interface LaboratoryResults {
  recent: LabResult[];
  historical: LabResult[];
  panels: LabPanel[];
}

interface LabResult {
  id: string;
  code: {
    loinc: string;
    display: string;
    loincPartNumber?: string;
  };
  status: 'registered' | 'preliminary' | 'final' | 'amended' | 'corrected' | 'cancelled';
  category: string;
  effectiveDateTime: Date;
  issued: Date;
  value: LabValue;
  interpretation: {
    code: 'H' | 'L' | 'A' | 'AA' | 'HH' | 'LL' | 'N' | 'U' | 'D' | 'B' | 'W';
    display: string;
  };
  referenceRange: ReferenceRange[];
  specimen: SpecimenInfo;
  performer: Reference;
  note: string[];
}

type LabValue =
  | { type: 'quantity'; value: number; unit: string; system: string; code: string }
  | { type: 'string'; value: string }
  | { type: 'codeableConcept'; coding: Coding[] }
  | { type: 'ratio'; numerator: Quantity; denominator: Quantity }
  | { type: 'range'; low: Quantity; high: Quantity };

interface ReferenceRange {
  low?: Quantity;
  high?: Quantity;
  type?: CodeableConcept;
  appliesTo?: CodeableConcept[];
  age?: Range;
  text?: string;
}

// Vital Signs Model
interface VitalSignHistory {
  current: VitalSigns;
  history: VitalSignReading[];
  trends: VitalTrend[];
}

interface VitalSigns {
  timestamp: Date;
  bloodPressure?: {
    systolic: number;
    diastolic: number;
    position: 'sitting' | 'standing' | 'supine';
    cuffSize: string;
  };
  heartRate?: {
    value: number;
    rhythm: 'regular' | 'irregular';
    method: 'palpation' | 'auscultation' | 'device';
  };
  respiratoryRate?: {
    value: number;
  };
  temperature?: {
    value: number;
    unit: 'Cel' | '[degF]';
    site: 'oral' | 'tympanic' | 'axillary' | 'rectal' | 'temporal';
  };
  oxygenSaturation?: {
    value: number;
    supplementalOxygen: boolean;
    oxygenFlowRate?: number;
  };
  weight?: {
    value: number;
    unit: 'kg' | '[lb_av]';
  };
  height?: {
    value: number;
    unit: 'cm' | '[in_i]';
  };
  bmi?: number;
  painScore?: {
    value: number;
    scale: '0-10' | 'FACES' | 'FLACC';
  };
}
```

### 3.3 Clinical Knowledge Representation

```typescript
// Clinical Knowledge Formats
interface ClinicalKnowledgeFormats {
  guidelines: GuidelineFormat;
  alerts: AlertRuleFormat;
  orderSets: OrderSetFormat;
  calculators: ClinicalCalculatorFormat;
  evidenceSynthesis: EvidenceFormat;
}

// Guideline Representation (PlanDefinition)
interface GuidelineFormat {
  id: string;
  url: string;
  version: string;
  name: string;
  title: string;
  status: 'draft' | 'active' | 'retired' | 'unknown';
  experimental: boolean;
  date: Date;
  publisher: string;
  description: string;
  purpose: string;

  // Applicability
  useContext: UseContext[];
  jurisdiction: CodeableConcept[];

  // Timing
  effectivePeriod: Period;

  // Topics
  topic: CodeableConcept[];

  // Contributors
  author: ContactDetail[];
  editor: ContactDetail[];
  reviewer: ContactDetail[];
  endorser: ContactDetail[];

  // Related artifacts
  relatedArtifact: RelatedArtifact[];

  // Library references (CQL logic)
  library: string[];

  // Goals
  goal: GoalDefinition[];

  // Actions (the guideline steps)
  action: GuidelineAction[];
}

interface GuidelineAction {
  id: string;
  prefix: string;
  title: string;
  description: string;
  textEquivalent: string;
  priority: 'routine' | 'urgent' | 'asap' | 'stat';
  code: CodeableConcept[];
  reason: CodeableConcept[];
  documentation: RelatedArtifact[];

  // Applicability conditions
  condition: {
    kind: 'applicability' | 'start' | 'stop';
    expression: Expression;
  }[];

  // Triggers
  trigger: TriggerDefinition[];

  // Inputs/Outputs
  input: DataRequirement[];
  output: DataRequirement[];

  // Timing
  timing: Timing;

  // Participants
  participant: ActionParticipant[];

  // Related actions
  relatedAction: RelatedAction[];

  // Dynamic values (calculated)
  dynamicValue: DynamicValue[];

  // Nested actions
  action?: GuidelineAction[];

  // What to do
  definitionCanonical?: string;  // ActivityDefinition reference
  definitionUri?: string;

  // Selection behavior
  selectionBehavior: 'any' | 'all' | 'all-or-none' | 'exactly-one' | 'at-most-one' | 'one-or-more';
  requiredBehavior: 'must' | 'could' | 'must-unless-documented';
  precheckBehavior: 'yes' | 'no';
  cardinalityBehavior: 'single' | 'multiple';
}

// Alert Rule Format
interface AlertRuleFormat {
  ruleId: string;
  ruleName: string;
  version: string;
  category: AlertCategory;
  severity: AlertSeverity;
  status: 'active' | 'draft' | 'retired';

  // When the rule applies
  applicability: {
    patientCriteria: Expression;
    encounterCriteria?: Expression;
    settingCriteria?: string[];
  };

  // Trigger conditions
  triggers: AlertTrigger[];

  // Alert content
  alert: {
    titleTemplate: string;
    messageTemplate: string;
    detailTemplate?: string;
    recommendedAction: string;
    references: Reference[];
  };

  // Override options
  overrideOptions: {
    allowOverride: boolean;
    requireReason: boolean;
    acceptedReasons: OverrideReason[];
    requireDocumentation: boolean;
  };

  // Suppression rules
  suppression: {
    cooldownPeriod?: Duration;
    maxPerEncounter?: number;
    maxPerDay?: number;
    suppressIfAcknowledged?: boolean;
  };

  // Evidence and references
  evidence: {
    level: EvidenceLevel;
    sources: Citation[];
    lastReviewed: Date;
    nextReviewDue: Date;
  };
}

type AlertCategory =
  | 'DRUG_DRUG_INTERACTION'
  | 'DRUG_ALLERGY'
  | 'DRUG_DISEASE'
  | 'DRUG_AGE'
  | 'DRUG_PREGNANCY'
  | 'DRUG_RENAL'
  | 'DRUG_HEPATIC'
  | 'DRUG_DUPLICATE'
  | 'DOSE_ALERT'
  | 'LAB_CRITICAL'
  | 'LAB_ABNORMAL'
  | 'VITAL_SIGN_ALERT'
  | 'SCREENING_DUE'
  | 'IMMUNIZATION_DUE'
  | 'GUIDELINE_RECOMMENDATION'
  | 'RISK_ALERT'
  | 'DOCUMENTATION_REMINDER';

type AlertSeverity = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

// Order Set Format
interface OrderSetFormat {
  id: string;
  name: string;
  version: string;
  status: 'active' | 'draft' | 'retired';
  category: string[];
  indication: CodeableConcept[];
  description: string;

  // Order items
  orders: OrderItem[];

  // Selection behavior
  defaultSelections: string[];  // IDs of pre-selected items
  requiredItems: string[];  // IDs of required items
  mutuallyExclusive: string[][];  // Groups of mutually exclusive items

  // Evidence
  evidence: {
    guidelines: Reference[];
    lastReviewed: Date;
    institution: string;
  };
}

interface OrderItem {
  id: string;
  sequence: number;
  category: 'MEDICATION' | 'LAB' | 'IMAGING' | 'PROCEDURE' | 'CONSULT' | 'NURSING' | 'DIET' | 'ACTIVITY';
  selected: boolean;
  required: boolean;

  // Order details (varies by category)
  medication?: MedicationOrderTemplate;
  labOrder?: LabOrderTemplate;
  imagingOrder?: ImagingOrderTemplate;
  procedure?: ProcedureOrderTemplate;
  consult?: ConsultOrderTemplate;
  nursingOrder?: NursingOrderTemplate;

  // Conditions for inclusion
  condition?: Expression;

  // Notes
  comment?: string;
}

interface MedicationOrderTemplate {
  medication: {
    rxnormCode: string;
    display: string;
    generic: boolean;
  };
  dose: {
    value: number | string;  // Can be calculated
    unit: string;
  };
  route: CodeableConcept;
  frequency: CodeableConcept;
  duration?: Duration;
  prn: boolean;
  prnReason?: CodeableConcept[];
  instructions?: string;
  substitutionAllowed: boolean;
  maxDailyDose?: Quantity;
  monitoringRequired?: MonitoringRequirement[];
}

// Clinical Calculator Format
interface ClinicalCalculatorFormat {
  id: string;
  name: string;
  version: string;
  description: string;
  category: string[];
  indication: string;

  // Inputs
  inputs: CalculatorInput[];

  // Calculation logic
  formula: CalculatorFormula;

  // Output interpretation
  interpretation: ScoreInterpretation[];

  // Evidence
  validation: {
    originalStudy: Citation;
    validationStudies: Citation[];
    populations: string[];
    limitations: string[];
  };

  // Display
  displayOptions: {
    showFormula: boolean;
    showInterpretation: boolean;
    showReferences: boolean;
  };
}

interface CalculatorInput {
  id: string;
  name: string;
  description: string;
  type: 'numeric' | 'categorical' | 'boolean' | 'date';
  required: boolean;

  // For numeric
  numericConfig?: {
    unit: string;
    min: number;
    max: number;
    precision: number;
  };

  // For categorical
  categoricalConfig?: {
    options: { value: string | number; label: string; points?: number }[];
  };

  // Source (auto-populate from EHR)
  autoPopulate?: {
    fhirPath: string;
    loincCode?: string;
    defaultValue?: any;
  };
}

interface CalculatorFormula {
  type: 'expression' | 'lookup' | 'algorithm';

  // For expression (e.g., GFR calculation)
  expression?: string;

  // For lookup (e.g., body surface area charts)
  lookupTable?: LookupTable;

  // For algorithm (e.g., cardiovascular risk)
  algorithm?: string;  // CQL or JavaScript
}

// Example: CKD-EPI eGFR Calculator
const ckdEpiCalculator: ClinicalCalculatorFormat = {
  id: 'ckd-epi-egfr-2021',
  name: 'CKD-EPI eGFR (2021)',
  version: '2021',
  description: 'Estimates glomerular filtration rate using CKD-EPI 2021 equation (race-free)',
  category: ['Nephrology', 'Laboratory'],
  indication: 'Assessment of kidney function',

  inputs: [
    {
      id: 'serum_creatinine',
      name: 'Serum Creatinine',
      description: 'Serum creatinine level',
      type: 'numeric',
      required: true,
      numericConfig: {
        unit: 'mg/dL',
        min: 0.1,
        max: 20,
        precision: 2
      },
      autoPopulate: {
        fhirPath: "Observation.where(code.coding.where(system='http://loinc.org' and code='2160-0')).value.value",
        loincCode: '2160-0'
      }
    },
    {
      id: 'age',
      name: 'Age',
      description: 'Patient age in years',
      type: 'numeric',
      required: true,
      numericConfig: {
        unit: 'years',
        min: 18,
        max: 120,
        precision: 0
      },
      autoPopulate: {
        fhirPath: "Patient.birthDate.toAge()"
      }
    },
    {
      id: 'sex',
      name: 'Sex',
      description: 'Biological sex',
      type: 'categorical',
      required: true,
      categoricalConfig: {
        options: [
          { value: 'female', label: 'Female' },
          { value: 'male', label: 'Male' }
        ]
      },
      autoPopulate: {
        fhirPath: "Patient.gender"
      }
    }
  ],

  formula: {
    type: 'expression',
    expression: `
      // CKD-EPI 2021 equation (race-free)
      let kappa = sex == 'female' ? 0.7 : 0.9;
      let alpha = sex == 'female' ? -0.241 : -0.302;
      let sexCoef = sex == 'female' ? 1.012 : 1.0;

      let scrKappa = serum_creatinine / kappa;
      let term1 = Math.min(scrKappa, 1) ** alpha;
      let term2 = Math.max(scrKappa, 1) ** -1.200;

      return 142 * term1 * term2 * (0.9938 ** age) * sexCoef;
    `
  },

  interpretation: [
    {
      range: { min: 90, max: Infinity },
      stage: 'G1',
      description: 'Normal or high',
      recommendation: 'If no other evidence of kidney disease, may not indicate CKD'
    },
    {
      range: { min: 60, max: 89 },
      stage: 'G2',
      description: 'Mildly decreased',
      recommendation: 'May indicate early CKD if other markers present'
    },
    {
      range: { min: 45, max: 59 },
      stage: 'G3a',
      description: 'Mildly to moderately decreased',
      recommendation: 'CKD stage 3a - monitor and manage risk factors'
    },
    {
      range: { min: 30, max: 44 },
      stage: 'G3b',
      description: 'Moderately to severely decreased',
      recommendation: 'CKD stage 3b - nephrology referral recommended'
    },
    {
      range: { min: 15, max: 29 },
      stage: 'G4',
      description: 'Severely decreased',
      recommendation: 'CKD stage 4 - prepare for renal replacement therapy'
    },
    {
      range: { min: 0, max: 14 },
      stage: 'G5',
      description: 'Kidney failure',
      recommendation: 'CKD stage 5 - dialysis or transplant may be needed'
    }
  ],

  validation: {
    originalStudy: {
      authors: 'Inker LA, et al.',
      title: 'New Creatinine- and Cystatin C-Based Equations to Estimate GFR without Race',
      journal: 'N Engl J Med',
      year: 2021,
      doi: '10.1056/NEJMoa2102953'
    },
    validationStudies: [],
    populations: ['Adults ≥18 years'],
    limitations: [
      'Less accurate in acute kidney injury',
      'May be inaccurate with extremes of muscle mass',
      'Not validated in children or pregnant women'
    ]
  },

  displayOptions: {
    showFormula: true,
    showInterpretation: true,
    showReferences: true
  }
};
```

### 3.4 CDS Hooks Specification

```typescript
// CDS Hooks Standard Implementation
interface CDSHooksSpecification {
  version: '2.0';
  description: 'HL7 CDS Hooks for EHR integration';

  hooks: {
    patientView: PatientViewHook;
    orderSelect: OrderSelectHook;
    orderSign: OrderSignHook;
    appointmentBook: AppointmentBookHook;
    encounterStart: EncounterStartHook;
    encounterDischarge: EncounterDischargeHook;
  };
}

// CDS Hooks Request/Response
interface CDSHookRequest {
  hook: string;
  hookInstance: string;  // UUID
  fhirServer: string;
  fhirAuthorization?: FHIRAuthorization;
  context: HookContext;
  prefetch?: PrefetchData;
}

interface HookContext {
  userId: string;  // Practitioner/[id]
  patientId: string;
  encounterId?: string;

  // Hook-specific context
  selections?: string[];  // For order-select
  draftOrders?: Bundle;  // For order-sign
}

interface CDSHookResponse {
  cards: Card[];
  systemActions?: SystemAction[];
}

interface Card {
  uuid: string;
  summary: string;  // Max 140 chars
  detail?: string;  // Markdown
  indicator: 'info' | 'warning' | 'critical';
  source: CardSource;
  suggestions?: Suggestion[];
  selectionBehavior?: 'at-most-one';
  overrideReasons?: OverrideReason[];
  links?: Link[];
}

interface CardSource {
  label: string;  // Organization name
  url?: string;
  icon?: string;  // PNG 100x100
  topic?: Coding;
}

interface Suggestion {
  label: string;
  uuid?: string;
  isRecommended?: boolean;
  actions: SuggestionAction[];
}

interface SuggestionAction {
  type: 'create' | 'update' | 'delete';
  description: string;
  resource?: Resource;  // FHIR resource
  resourceId?: string;  // For update/delete
}

// CDS Hooks Service Implementation
class CDSHooksService {
  private ruleEngine: RuleEngine;
  private mlService: MLInferenceService;
  private knowledgeBase: ClinicalKnowledgeBase;

  async handleHook(request: CDSHookRequest): Promise<CDSHookResponse> {
    const startTime = Date.now();

    try {
      // Get patient data (from prefetch or fetch)
      const patientData = await this.resolvePatientData(request);

      // Determine applicable rules and services
      const applicableServices = await this.findApplicableServices(
        request.hook,
        patientData
      );

      // Execute services in parallel
      const results = await Promise.all(
        applicableServices.map(service =>
          this.executeService(service, patientData, request)
        )
      );

      // Aggregate and prioritize cards
      const cards = this.aggregateAndPrioritize(results);

      // Log for audit
      await this.auditLog(request, cards, Date.now() - startTime);

      return { cards };

    } catch (error) {
      console.error('CDS Hooks error:', error);
      return { cards: [] };  // Fail silently to not block clinician
    }
  }

  private async resolvePatientData(
    request: CDSHookRequest
  ): Promise<PatientData> {
    // Use prefetch if available
    if (request.prefetch) {
      return this.extractFromPrefetch(request.prefetch);
    }

    // Otherwise fetch from FHIR server
    return this.fetchPatientData(
      request.fhirServer,
      request.context.patientId,
      request.fhirAuthorization
    );
  }

  private aggregateAndPrioritize(
    results: ServiceResult[]
  ): Card[] {
    // Flatten all cards
    let allCards = results.flatMap(r => r.cards);

    // Remove duplicates (by knowledge source)
    allCards = this.deduplicateCards(allCards);

    // Sort by priority
    allCards.sort((a, b) => {
      const priority = { 'critical': 0, 'warning': 1, 'info': 2 };
      return priority[a.indicator] - priority[b.indicator];
    });

    // Apply maximum card limit
    const MAX_CARDS = 5;
    return allCards.slice(0, MAX_CARDS);
  }
}

// Example: Drug Interaction CDS Service
class DrugInteractionCDSService implements CDSService {
  serviceName = 'Drug-Drug Interaction Check';
  supportedHooks = ['order-select', 'order-sign'];

  async execute(
    patientData: PatientData,
    request: CDSHookRequest
  ): Promise<ServiceResult> {
    const cards: Card[] = [];

    // Get current medications
    const currentMeds = patientData.medications.active;

    // Get draft orders (medications only)
    const draftMeds = this.extractDraftMedications(request.context.draftOrders);

    // Check for interactions
    for (const draftMed of draftMeds) {
      for (const currentMed of currentMeds) {
        const interaction = await this.checkInteraction(draftMed, currentMed);

        if (interaction && interaction.severity !== 'NONE') {
          cards.push(this.createInteractionCard(draftMed, currentMed, interaction));
        }
      }

      // Also check draft-to-draft interactions
      for (const otherDraft of draftMeds) {
        if (otherDraft.id === draftMed.id) continue;

        const interaction = await this.checkInteraction(draftMed, otherDraft);
        if (interaction && interaction.severity !== 'NONE') {
          cards.push(this.createInteractionCard(draftMed, otherDraft, interaction));
        }
      }
    }

    return { cards };
  }

  private createInteractionCard(
    drug1: MedicationOrder,
    drug2: MedicationRecord,
    interaction: DrugInteraction
  ): Card {
    return {
      uuid: generateUUID(),
      summary: `${interaction.severity} interaction: ${drug1.display} + ${drug2.medication.display}`,
      detail: `**Effect:** ${interaction.effect}\n\n**Mechanism:** ${interaction.mechanism}\n\n**Management:** ${interaction.management}`,
      indicator: this.mapSeverityToIndicator(interaction.severity),
      source: {
        label: 'Drug Interaction Database',
        url: 'https://www.drugs.com/interactions.html'
      },
      suggestions: interaction.alternatives?.map(alt => ({
        label: `Consider ${alt.name} instead`,
        isRecommended: alt.recommended,
        actions: [{
          type: 'update',
          description: `Replace ${drug1.display} with ${alt.name}`,
          resource: this.createAlternativeMedication(drug1, alt)
        }]
      })),
      overrideReasons: [
        { code: 'patient-tolerated', display: 'Patient has tolerated this combination' },
        { code: 'benefit-outweighs-risk', display: 'Clinical benefit outweighs risk' },
        { code: 'monitoring-in-place', display: 'Appropriate monitoring in place' }
      ]
    };
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT Data Formats**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
