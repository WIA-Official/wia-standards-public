# 제3장: 임상의사결정지원 데이터 형식

## 의료 데이터 모델 및 상호운용성 표준

### 3.1 의료 데이터 표준 개요

임상의사결정지원 시스템은 여러 소스에서 의료 데이터와의 원활한 통합이 필요합니다. 이 장에서는 상호운용성과 효과적인 임상의사결정지원을 가능하게 하는 데이터 형식, 표준 및 모델을 정의합니다.

```typescript
// 의료 데이터 표준 프레임워크
interface HealthcareDataStandards {
  version: '1.0.0';

  interoperabilityStandards: {
    hl7Fhir: {
      version: 'R4 (4.0.1)';
      scope: '주요 데이터 교환 표준';
      resources: FHIRResource[];
      adoption: '미국 규제에 의해 의무화(21세기 치료법)';
    };
    hl7V2: {
      version: '2.5.1+';
      scope: '레거시 메시지 교환';
      usage: '검사 결과, ADT, 처방';
      status: '여전히 널리 사용, 점진적으로 FHIR로 이전';
    };
    cda: {
      version: 'R2';
      scope: '임상 문서 교환';
      usage: 'CCD, 퇴원 요약';
      status: 'FHIR 문서로 대체되는 중';
    };
    dicom: {
      version: '2024a';
      scope: '의료 영상';
      usage: '모든 영상 모달리티, 리포트용 SR';
    };
  };

  terminologyStandards: {
    snomedCt: {
      scope: '임상 용어';
      coverage: '진단, 처치, 소견, 유기체';
      concepts: '35만 개 이상 개념';
    };
    icd: {
      versions: ['ICD-10-CM', 'ICD-11'];
      scope: '진단 코딩';
      usage: '청구, 역학';
    };
    loinc: {
      scope: '검사 및 임상 관찰';
      codes: '9만 개 이상 코드';
      usage: '검사, 생체징후, 문서';
    };
    rxnorm: {
      scope: '약물';
      coverage: '약물명, 성분, 제형, 강도';
      usage: '처방 데이터, 약물 데이터베이스';
    };
    cpt: {
      scope: '처치 코딩';
      usage: '청구, 서비스 문서화';
    };
  };
}

// CDSS용 FHIR 리소스
const fhirResourcesForCDSS: FHIRResourceCoverage = {
  patientData: {
    Patient: {
      usage: '인구통계, 식별자',
      cdssRelevance: '환자 식별, 연령 기반 규칙',
      keyElements: ['identifier', 'name', 'birthDate', 'gender', 'address']
    },
    Condition: {
      usage: '문제, 진단',
      cdssRelevance: '규칙 트리거, 위험 요인, 금기사항',
      keyElements: ['code', 'clinicalStatus', 'verificationStatus', 'onset', 'severity']
    },
    MedicationRequest: {
      usage: '처방 오더',
      cdssRelevance: '약물 상호작용 검사, 용량 권장사항',
      keyElements: ['medication', 'dosageInstruction', 'status', 'intent']
    },
    MedicationStatement: {
      usage: '약물 사용 기록',
      cdssRelevance: '상호작용 검사를 위한 활성 약물 목록',
      keyElements: ['medication', 'status', 'effectivePeriod', 'dosage']
    },
    AllergyIntolerance: {
      usage: '알레르기 및 부작용',
      cdssRelevance: '알레르기 경보, 금기사항 검사',
      keyElements: ['code', 'clinicalStatus', 'type', 'category', 'criticality', 'reaction']
    },
    Observation: {
      usage: '검사 결과, 생체징후, 평가',
      cdssRelevance: '규칙 트리거, 트렌드 분석, 위험 점수',
      keyElements: ['code', 'value', 'effectiveDateTime', 'status', 'interpretation']
    },
    Procedure: {
      usage: '수행된 처치',
      cdssRelevance: '케어 이력, 후속 권장사항',
      keyElements: ['code', 'status', 'performedDateTime', 'outcome']
    },
    Immunization: {
      usage: '예방접종 기록',
      cdssRelevance: '예방접종 권장사항, 예정일',
      keyElements: ['vaccineCode', 'occurrenceDateTime', 'status', 'doseQuantity']
    }
  },

  clinicalResources: {
    Encounter: {
      usage: '의료 방문',
      cdssRelevance: '권장사항 컨텍스트, 방문 유형별 규칙',
      keyElements: ['status', 'class', 'type', 'period', 'reasonCode']
    },
    DiagnosticReport: {
      usage: '검사 리포트, 영상 리포트',
      cdssRelevance: '검사 해석 지원, 이상 플래깅',
      keyElements: ['code', 'status', 'effectiveDateTime', 'conclusion', 'result']
    },
    CarePlan: {
      usage: '치료 계획',
      cdssRelevance: '케어 경로 준수, 중재 일정',
      keyElements: ['status', 'intent', 'category', 'activity', 'goal']
    },
    Goal: {
      usage: '임상 목표',
      cdssRelevance: '결과 추적, 목표 지향 권장사항',
      keyElements: ['lifecycleStatus', 'description', 'target', 'achievementStatus']
    },
    RiskAssessment: {
      usage: '위험 평가',
      cdssRelevance: '위험 점수, 예측 모델 출력',
      keyElements: ['code', 'prediction', 'mitigation', 'note']
    }
  },

  cdssSpecificResources: {
    Library: {
      usage: '지식 아티팩트(CQL 라이브러리)',
      cdssRelevance: '임상 로직 정의',
      keyElements: ['content', 'type', 'dataRequirement', 'parameter']
    },
    PlanDefinition: {
      usage: '프로토콜, 처방세트, 지침',
      cdssRelevance: '컴퓨팅 가능 임상 지침',
      keyElements: ['action', 'goal', 'trigger', 'condition', 'dynamicValue']
    },
    ActivityDefinition: {
      usage: '처방 템플릿',
      cdssRelevance: '권장 처방, 활동',
      keyElements: ['code', 'timing', 'dosage', 'productReference']
    },
    RequestGroup: {
      usage: '그룹화된 권장사항',
      cdssRelevance: 'CDSS 출력 - 번들 권장사항',
      keyElements: ['status', 'intent', 'action', 'priority']
    },
    GuidanceResponse: {
      usage: 'CDS 응답',
      cdssRelevance: '표준 CDSS 응답 형식',
      keyElements: ['status', 'outputParameters', 'result', 'dataRequirement']
    }
  }
};
```

### 3.2 CDSS 데이터 모델

```typescript
// 핵심 CDSS 데이터 모델
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

// 환자 인구통계
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

// 임상 컨텍스트를 위한 문제 목록
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

// 약물 데이터 모델
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

// 알레르기 및 불내성 데이터
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

// 검사 결과 모델
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

// 생체징후 모델
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

### 3.3 임상 지식 표현

```typescript
// 임상 지식 형식
interface ClinicalKnowledgeFormats {
  guidelines: GuidelineFormat;
  alerts: AlertRuleFormat;
  orderSets: OrderSetFormat;
  calculators: ClinicalCalculatorFormat;
  evidenceSynthesis: EvidenceFormat;
}

// 지침 표현(PlanDefinition)
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

  // 적용 가능성
  useContext: UseContext[];
  jurisdiction: CodeableConcept[];

  // 타이밍
  effectivePeriod: Period;

  // 주제
  topic: CodeableConcept[];

  // 기여자
  author: ContactDetail[];
  editor: ContactDetail[];
  reviewer: ContactDetail[];
  endorser: ContactDetail[];

  // 관련 아티팩트
  relatedArtifact: RelatedArtifact[];

  // 라이브러리 참조(CQL 로직)
  library: string[];

  // 목표
  goal: GoalDefinition[];

  // 액션(지침 단계)
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

  // 적용 가능성 조건
  condition: {
    kind: 'applicability' | 'start' | 'stop';
    expression: Expression;
  }[];

  // 트리거
  trigger: TriggerDefinition[];

  // 입력/출력
  input: DataRequirement[];
  output: DataRequirement[];

  // 타이밍
  timing: Timing;

  // 참가자
  participant: ActionParticipant[];

  // 관련 액션
  relatedAction: RelatedAction[];

  // 동적 값(계산됨)
  dynamicValue: DynamicValue[];

  // 중첩 액션
  action?: GuidelineAction[];

  // 수행할 것
  definitionCanonical?: string;  // ActivityDefinition 참조
  definitionUri?: string;

  // 선택 동작
  selectionBehavior: 'any' | 'all' | 'all-or-none' | 'exactly-one' | 'at-most-one' | 'one-or-more';
  requiredBehavior: 'must' | 'could' | 'must-unless-documented';
  precheckBehavior: 'yes' | 'no';
  cardinalityBehavior: 'single' | 'multiple';
}

// 경보 규칙 형식
interface AlertRuleFormat {
  ruleId: string;
  ruleName: string;
  version: string;
  category: AlertCategory;
  severity: AlertSeverity;
  status: 'active' | 'draft' | 'retired';

  // 규칙 적용 시기
  applicability: {
    patientCriteria: Expression;
    encounterCriteria?: Expression;
    settingCriteria?: string[];
  };

  // 트리거 조건
  triggers: AlertTrigger[];

  // 경보 내용
  alert: {
    titleTemplate: string;
    messageTemplate: string;
    detailTemplate?: string;
    recommendedAction: string;
    references: Reference[];
  };

  // 무시 옵션
  overrideOptions: {
    allowOverride: boolean;
    requireReason: boolean;
    acceptedReasons: OverrideReason[];
    requireDocumentation: boolean;
  };

  // 억제 규칙
  suppression: {
    cooldownPeriod?: Duration;
    maxPerEncounter?: number;
    maxPerDay?: number;
    suppressIfAcknowledged?: boolean;
  };

  // 증거 및 참조
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

// 임상 계산기 형식
interface ClinicalCalculatorFormat {
  id: string;
  name: string;
  version: string;
  description: string;
  category: string[];
  indication: string;

  // 입력
  inputs: CalculatorInput[];

  // 계산 로직
  formula: CalculatorFormula;

  // 출력 해석
  interpretation: ScoreInterpretation[];

  // 증거
  validation: {
    originalStudy: Citation;
    validationStudies: Citation[];
    populations: string[];
    limitations: string[];
  };

  // 표시
  displayOptions: {
    showFormula: boolean;
    showInterpretation: boolean;
    showReferences: boolean;
  };
}

// 예: CKD-EPI eGFR 계산기
const ckdEpiCalculator: ClinicalCalculatorFormat = {
  id: 'ckd-epi-egfr-2021',
  name: 'CKD-EPI eGFR (2021)',
  version: '2021',
  description: 'CKD-EPI 2021 공식(인종 무관)을 사용하여 사구체여과율 추정',
  category: ['신장내과', '검사실'],
  indication: '신장 기능 평가',

  inputs: [
    {
      id: 'serum_creatinine',
      name: '혈청 크레아티닌',
      description: '혈청 크레아티닌 수치',
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
      name: '나이',
      description: '환자 나이(년)',
      type: 'numeric',
      required: true,
      numericConfig: {
        unit: '년',
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
      name: '성별',
      description: '생물학적 성별',
      type: 'categorical',
      required: true,
      categoricalConfig: {
        options: [
          { value: 'female', label: '여성' },
          { value: 'male', label: '남성' }
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
      // CKD-EPI 2021 공식(인종 무관)
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
      description: '정상 또는 높음',
      recommendation: '다른 신장 질환 증거가 없으면 CKD를 나타내지 않을 수 있음'
    },
    {
      range: { min: 60, max: 89 },
      stage: 'G2',
      description: '경도 감소',
      recommendation: '다른 마커가 있으면 초기 CKD를 나타낼 수 있음'
    },
    {
      range: { min: 45, max: 59 },
      stage: 'G3a',
      description: '경도-중등도 감소',
      recommendation: 'CKD 3a기 - 모니터링 및 위험 인자 관리'
    },
    {
      range: { min: 30, max: 44 },
      stage: 'G3b',
      description: '중등도-중증 감소',
      recommendation: 'CKD 3b기 - 신장내과 의뢰 권장'
    },
    {
      range: { min: 15, max: 29 },
      stage: 'G4',
      description: '중증 감소',
      recommendation: 'CKD 4기 - 신대체요법 준비'
    },
    {
      range: { min: 0, max: 14 },
      stage: 'G5',
      description: '신부전',
      recommendation: 'CKD 5기 - 투석 또는 이식 필요할 수 있음'
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
    populations: ['18세 이상 성인'],
    limitations: [
      '급성 신손상에서 정확도 낮음',
      '근육량 극단에서 부정확할 수 있음',
      '소아 또는 임산부에서 검증되지 않음'
    ]
  },

  displayOptions: {
    showFormula: true,
    showInterpretation: true,
    showReferences: true
  }
};
```

### 3.4 CDS Hooks 사양

```typescript
// CDS Hooks 표준 구현
interface CDSHooksSpecification {
  version: '2.0';
  description: 'EHR 통합을 위한 HL7 CDS Hooks';

  hooks: {
    patientView: PatientViewHook;
    orderSelect: OrderSelectHook;
    orderSign: OrderSignHook;
    appointmentBook: AppointmentBookHook;
    encounterStart: EncounterStartHook;
    encounterDischarge: EncounterDischargeHook;
  };
}

// CDS Hooks 요청/응답
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

  // Hook 특정 컨텍스트
  selections?: string[];  // order-select용
  draftOrders?: Bundle;  // order-sign용
}

interface CDSHookResponse {
  cards: Card[];
  systemActions?: SystemAction[];
}

interface Card {
  uuid: string;
  summary: string;  // 최대 140자
  detail?: string;  // 마크다운
  indicator: 'info' | 'warning' | 'critical';
  source: CardSource;
  suggestions?: Suggestion[];
  selectionBehavior?: 'at-most-one';
  overrideReasons?: OverrideReason[];
  links?: Link[];
}

// CDS Hooks 서비스 구현
class CDSHooksService {
  private ruleEngine: RuleEngine;
  private mlService: MLInferenceService;
  private knowledgeBase: ClinicalKnowledgeBase;

  async handleHook(request: CDSHookRequest): Promise<CDSHookResponse> {
    const startTime = Date.now();

    try {
      // 환자 데이터 가져오기(prefetch에서 또는 fetch)
      const patientData = await this.resolvePatientData(request);

      // 적용 가능한 규칙 및 서비스 결정
      const applicableServices = await this.findApplicableServices(
        request.hook,
        patientData
      );

      // 서비스 병렬 실행
      const results = await Promise.all(
        applicableServices.map(service =>
          this.executeService(service, patientData, request)
        )
      );

      // 카드 집계 및 우선순위 지정
      const cards = this.aggregateAndPrioritize(results);

      // 감사 로깅
      await this.auditLog(request, cards, Date.now() - startTime);

      return { cards };

    } catch (error) {
      console.error('CDS Hooks 오류:', error);
      return { cards: [] };  // 임상의를 차단하지 않도록 조용히 실패
    }
  }

  private aggregateAndPrioritize(
    results: ServiceResult[]
  ): Card[] {
    // 모든 카드 평탄화
    let allCards = results.flatMap(r => r.cards);

    // 중복 제거(지식 소스별)
    allCards = this.deduplicateCards(allCards);

    // 우선순위별 정렬
    allCards.sort((a, b) => {
      const priority = { 'critical': 0, 'warning': 1, 'info': 2 };
      return priority[a.indicator] - priority[b.indicator];
    });

    // 최대 카드 제한 적용
    const MAX_CARDS = 5;
    return allCards.slice(0, MAX_CARDS);
  }
}

// 예: 약물 상호작용 CDS 서비스
class DrugInteractionCDSService implements CDSService {
  serviceName = '약물-약물 상호작용 검사';
  supportedHooks = ['order-select', 'order-sign'];

  async execute(
    patientData: PatientData,
    request: CDSHookRequest
  ): Promise<ServiceResult> {
    const cards: Card[] = [];

    // 현재 약물 가져오기
    const currentMeds = patientData.medications.active;

    // 초안 처방(약물만) 가져오기
    const draftMeds = this.extractDraftMedications(request.context.draftOrders);

    // 상호작용 검사
    for (const draftMed of draftMeds) {
      for (const currentMed of currentMeds) {
        const interaction = await this.checkInteraction(draftMed, currentMed);

        if (interaction && interaction.severity !== 'NONE') {
          cards.push(this.createInteractionCard(draftMed, currentMed, interaction));
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
      summary: `${interaction.severity} 상호작용: ${drug1.display} + ${drug2.medication.display}`,
      detail: `**효과:** ${interaction.effect}\n\n**기전:** ${interaction.mechanism}\n\n**관리:** ${interaction.management}`,
      indicator: this.mapSeverityToIndicator(interaction.severity),
      source: {
        label: '약물 상호작용 데이터베이스',
        url: 'https://www.drugs.com/interactions.html'
      },
      suggestions: interaction.alternatives?.map(alt => ({
        label: `대신 ${alt.name} 고려`,
        isRecommended: alt.recommended,
        actions: [{
          type: 'update',
          description: `${drug1.display}를 ${alt.name}로 대체`,
          resource: this.createAlternativeMedication(drug1, alt)
        }]
      })),
      overrideReasons: [
        { code: 'patient-tolerated', display: '환자가 이 조합을 견뎠음' },
        { code: 'benefit-outweighs-risk', display: '임상적 이점이 위험을 능가함' },
        { code: 'monitoring-in-place', display: '적절한 모니터링 시행 중' }
      ]
    };
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT 데이터 형식**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
