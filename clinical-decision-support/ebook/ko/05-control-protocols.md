# 제5장: 임상의사결정지원 제어 프로토콜

## 추론 엔진 및 AI 알고리즘

### 5.1 CDSS 추론 아키텍처

WIA-CLINICAL-DECISION-SUPPORT 표준은 다양한 임상 시나리오 및 사용 사례를 지원하기 위해 여러 추론 엔진을 정의합니다.

```typescript
// CDSS 추론 프레임워크
interface CDSSInferenceFramework {
  version: '1.0.0';

  inferenceEngines: {
    ruleEngine: {
      type: '지식 기반';
      technology: 'Rete 알고리즘을 사용한 프로덕션 규칙';
      useCases: ['경보', '지침 준수', '처방 검사'];
      latency: '<100ms';
    };
    mlEngine: {
      type: '머신러닝';
      technology: '앙상블 모델, 그래디언트 부스팅';
      useCases: ['위험 예측', '진단 지원', '결과 예측'];
      latency: '<500ms';
    };
    dlEngine: {
      type: '딥러닝';
      technology: 'CNN, Transformers, GNN';
      useCases: ['영상 분석', 'NLP 추출', '시계열 분석'];
      latency: '<2s';
    };
    llmEngine: {
      type: '대규모 언어 모델';
      technology: '의료 조정 LLM';
      useCases: ['임상 요약', 'Q&A', '지침 해석'];
      latency: '<5s';
    };
    hybridEngine: {
      type: '하이브리드';
      description: '위 엔진들의 조합';
      useCases: ['복잡한 임상 시나리오'];
    };
  };

  orchestration: {
    pattern: '파이프라인 + 병렬';
    routing: '컨텍스트 기반';
    fallback: '우아한 성능 저하';
  };
}

// 규칙 엔진 구현
class ClinicalRuleEngine {
  private ruleBase: RuleBase;
  private workingMemory: WorkingMemory;
  private agenda: Agenda;

  constructor(ruleBase: RuleBase) {
    this.ruleBase = ruleBase;
    this.workingMemory = new WorkingMemory();
    this.agenda = new Agenda();
  }

  async evaluate(
    patientData: PatientData,
    context: ClinicalContext
  ): Promise<RuleEvaluationResult> {
    // 작업 메모리 초기화
    this.workingMemory.clear();

    // 팩트 주장
    this.assertFacts(patientData, context);

    // 매칭 실행
    const activatedRules = await this.match();

    // 충돌 해결 및 실행
    const results = await this.resolveAndExecute(activatedRules);

    return {
      activatedRules: results.length,
      recommendations: results.filter(r => r.type === 'RECOMMENDATION'),
      alerts: results.filter(r => r.type === 'ALERT'),
      actions: results.filter(r => r.type === 'ACTION'),
      executionTime: performance.now()
    };
  }

  private assertFacts(patientData: PatientData, context: ClinicalContext): void {
    // 환자 인구통계
    this.workingMemory.assert({
      type: 'Patient',
      age: patientData.demographics.age.years,
      gender: patientData.demographics.gender,
      ...patientData.demographics
    });

    // 활성 상태
    patientData.problems.active.forEach(problem => {
      this.workingMemory.assert({
        type: 'Condition',
        code: problem.code.code,
        system: problem.code.system,
        display: problem.code.display,
        severity: problem.severity,
        status: 'active'
      });
    });

    // 현재 약물
    patientData.medications.active.forEach(med => {
      this.workingMemory.assert({
        type: 'Medication',
        rxnorm: med.medication.rxnormCode,
        drugClass: med.medication.drugClass,
        status: 'active'
      });
    });

    // 알레르기
    patientData.allergies.allergies.forEach(allergy => {
      this.workingMemory.assert({
        type: 'Allergy',
        code: allergy.code.code,
        category: allergy.category,
        criticality: allergy.criticality,
        reactions: allergy.reactions
      });
    });

    // 최근 검사
    patientData.labs.recent.forEach(lab => {
      this.workingMemory.assert({
        type: 'LabResult',
        loinc: lab.code.loinc,
        value: lab.value,
        interpretation: lab.interpretation,
        dateTime: lab.effectiveDateTime
      });
    });

    // 생체징후
    this.workingMemory.assert({
      type: 'Vitals',
      ...patientData.vitals.current
    });
  }
}

// CQL(Clinical Quality Language) 규칙 엔진
class CQLEngine {
  private libraryManager: CQLLibraryManager;
  private terminologyService: TerminologyService;
  private dataProvider: FHIRDataProvider;

  async evaluateCQL(
    libraryId: string,
    patientId: string,
    parameters?: Record<string, unknown>
  ): Promise<CQLResult> {
    // 라이브러리 로드
    const library = await this.libraryManager.getLibrary(libraryId);

    // 환자 데이터 검색 설정
    const context = new PatientContext(
      patientId,
      this.dataProvider,
      this.terminologyService
    );

    // 표현식 평가
    const results: Record<string, unknown> = {};

    for (const expression of library.expressions) {
      try {
        const value = await this.evaluateExpression(expression, context, parameters);
        results[expression.name] = value;
      } catch (error) {
        results[expression.name] = { error: error.message };
      }
    }

    return {
      libraryId,
      patientId,
      timestamp: new Date(),
      results,
      dataRequirements: library.dataRequirements
    };
  }
}

// CQL 규칙 예시
const diabetesManagementCQL = `
library DiabetesManagement version '1.0.0'

using FHIR version '4.0.1'

include FHIRHelpers version '4.0.1'

codesystem "LOINC": 'http://loinc.org'
codesystem "SNOMED": 'http://snomed.info/sct'

// 값 집합
valueset "당뇨병 진단": 'http://example.org/fhir/ValueSet/diabetes-diagnoses'
valueset "HbA1c 검사": 'http://example.org/fhir/ValueSet/hba1c-tests'
valueset "혈당 검사": 'http://example.org/fhir/ValueSet/glucose-tests'

// 환자 컨텍스트
context Patient

// 정의
define "당뇨병 있음":
  exists([Condition: "당뇨병 진단"] C
    where C.clinicalStatus ~ 'active')

define "최근 HbA1c":
  Last([Observation: "HbA1c 검사"] O
    where O.status in {'final', 'amended'}
    sort by effective.value)

define "HbA1c 값":
  ("최근 HbA1c".value as Quantity).value

define "HbA1c 목표 초과":
  "HbA1c 값" > 7.0

define "HbA1c 검사 필요":
  "당뇨병 있음"
    and (
      "최근 HbA1c" is null
      or months between ("최근 HbA1c".effective as dateTime) and Today() >= 3
    )

define "당뇨 관리 권장사항":
  if "HbA1c 검사 필요" then
    'HbA1c 검사가 예정되어 있습니다 (마지막 검사: 3개월 이상 전)'
  else if "HbA1c 목표 초과" then
    'HbA1c가 ' + ToString("HbA1c 값") + '%입니다. 치료 강화를 고려하세요'
  else
    '당뇨병 관리가 목표에 있습니다'
`;
```

### 5.2 머신러닝 추론

```typescript
// ML 추론 엔진
class MLInferenceEngine {
  private modelRegistry: ModelRegistry;
  private featureExtractor: FeatureExtractor;
  private explainer: ModelExplainer;

  async predict(
    modelId: string,
    patientData: PatientData,
    options: PredictionOptions = {}
  ): Promise<MLPrediction> {
    // 모델 메타데이터 로드
    const modelMeta = await this.modelRegistry.getModel(modelId);

    // 특성 추출
    const features = await this.featureExtractor.extract(
      patientData,
      modelMeta.featureSpec
    );

    // 특성 검증
    const validation = this.validateFeatures(features, modelMeta);
    if (!validation.valid) {
      throw new InsufficientFeaturesError(validation.missing);
    }

    // 추론 실행
    const prediction = await this.runInference(modelMeta, features);

    // 설명 생성
    let explanation: ModelExplanation | undefined;
    if (options.includeExplanation) {
      explanation = await this.explainer.explain(modelMeta, features, prediction);
    }

    // 신뢰도 보정
    const calibratedPrediction = this.calibrate(prediction, modelMeta);

    return {
      modelId,
      modelVersion: modelMeta.version,
      timestamp: new Date(),
      prediction: calibratedPrediction,
      confidence: this.calculateConfidence(calibratedPrediction, features, modelMeta),
      explanation,
      metadata: {
        featuresUsed: Object.keys(features),
        missingFeatures: validation.missing || [],
        inferenceTime: prediction.latency
      }
    };
  }

  // SHAP 설명자
  private async explainWithSHAP(
    model: MLModel,
    features: FeatureVector,
    prediction: number
  ): Promise<SHAPExplanation> {
    // SHAP 값 계산
    const shapValues = await this.calculateSHAP(model, features);

    // 영향별 특성 순위
    const rankedFeatures = Object.entries(shapValues)
      .map(([feature, value]) => ({
        feature,
        shapValue: value,
        contribution: value > 0 ? 'positive' : 'negative',
        magnitude: Math.abs(value)
      }))
      .sort((a, b) => b.magnitude - a.magnitude);

    return {
      baseValue: model.expectedValue,
      outputValue: prediction,
      features: rankedFeatures,
      topPositive: rankedFeatures.filter(f => f.contribution === 'positive').slice(0, 5),
      topNegative: rankedFeatures.filter(f => f.contribution === 'negative').slice(0, 5)
    };
  }
}

// 패혈증 예측 모델 예시
class SepsisPredictionModel implements CDSSModel {
  modelId = 'sepsis-risk-v3';
  version = '3.2.1';
  modelType = 'gradient_boosting';

  featureSpec: FeatureSpecification = {
    required: [
      { name: 'heart_rate', type: 'numeric', source: 'vitals.current.heartRate' },
      { name: 'respiratory_rate', type: 'numeric', source: 'vitals.current.respiratoryRate' },
      { name: 'temperature', type: 'numeric', source: 'vitals.current.temperature' },
      { name: 'systolic_bp', type: 'numeric', source: 'vitals.current.bloodPressure.systolic' },
      { name: 'wbc_count', type: 'numeric', source: 'labs.recent.WBC' },
      { name: 'lactate', type: 'numeric', source: 'labs.recent.lactate' }
    ],
    optional: [
      { name: 'age', type: 'numeric', source: 'demographics.age.years' },
      { name: 'creatinine', type: 'numeric', source: 'labs.recent.creatinine' },
      { name: 'platelet_count', type: 'numeric', source: 'labs.recent.platelets' },
      { name: 'bilirubin', type: 'numeric', source: 'labs.recent.bilirubin' },
      { name: 'immunocompromised', type: 'boolean', source: 'derived.immunocompromised' }
    ],
    derived: [
      {
        name: 'shock_index',
        formula: 'heart_rate / systolic_bp',
        dependencies: ['heart_rate', 'systolic_bp']
      },
      {
        name: 'sofa_respiratory',
        formula: 'calculateSOFARespiratoryComponent(pao2_fio2_ratio)',
        dependencies: ['pao2_fio2_ratio']
      }
    ]
  };

  thresholds = {
    low: 0.1,      // <10% = 저위험
    moderate: 0.3, // 10-30% = 중등도 위험
    high: 0.5,     // 30-50% = 고위험
    critical: 0.7  // >70% = 위험
  };

  async predict(features: FeatureVector): Promise<SepsisPrediction> {
    const probability = await this.runModel(features);

    return {
      probability,
      riskCategory: this.categorizeRisk(probability),
      recommendation: this.getRecommendation(probability),
      earlyWarningScore: this.calculateEarlyWarning(features),
      timeToOnset: this.estimateTimeToOnset(probability, features)
    };
  }

  private getRecommendation(probability: number): SepsisRecommendation {
    if (probability >= this.thresholds.critical) {
      return {
        urgency: 'IMMEDIATE',
        actions: [
          '패혈증 번들 프로토콜 즉시 시작',
          '젖산, 혈액 배양 검사 획득',
          '광범위 항생제 투여',
          '수액 소생술 시작',
          '중환자 의뢰 고려'
        ],
        monitoring: '연속 모니터링',
        escalation: '주치의 및 신속대응팀 즉시 호출'
      };
    }
    // 추가 권장사항 수준...

    return {
      urgency: 'ROUTINE',
      actions: ['일상적인 모니터링 계속'],
      monitoring: '4시간마다',
      escalation: '해당 없음'
    };
  }
}
```

### 5.3 딥러닝 추론

```typescript
// 딥러닝 추론 엔진
class DeepLearningInferenceEngine {
  private onnxRuntime: ONNXRuntimeSession;
  private preprocessors: Map<string, Preprocessor>;
  private gpuDevice?: GPUDevice;

  async runImageAnalysis(
    imageData: MedicalImage,
    modelId: string
  ): Promise<ImageAnalysisResult> {
    const model = await this.loadModel(modelId);

    // 이미지 전처리
    const preprocessor = this.preprocessors.get(model.preprocessorId)!;
    const tensor = await preprocessor.process(imageData);

    // 추론 실행
    const outputs = await this.onnxRuntime.run(model.session, {
      [model.inputName]: tensor
    });

    // 후처리
    const result = this.postprocess(outputs, model);

    // 시각화 생성(GradCAM 등)
    const visualization = await this.generateVisualization(
      imageData,
      model,
      outputs
    );

    return {
      modelId,
      imageId: imageData.id,
      findings: result.findings,
      confidence: result.confidence,
      visualization,
      processingTime: performance.now()
    };
  }

  // GradCAM 시각화
  private async generateGradCAM(
    image: Tensor,
    model: DeepLearningModel,
    targetClass: number
  ): Promise<GradCAMResult> {
    // 그래디언트 및 활성화 계산
    const { gradients, activations } = await this.computeGradients(
      image,
      model,
      targetClass
    );

    // 그래디언트 평균 풀링
    const weights = this.globalAveragePooling(gradients);

    // 가중치 적용
    const cam = this.weightedCombination(activations, weights);

    // 원본 이미지 크기로 업샘플링
    const heatmap = this.upsample(cam, image.shape);

    // ReLU 적용 및 정규화
    const normalizedHeatmap = this.normalizeHeatmap(heatmap);

    return {
      heatmap: normalizedHeatmap,
      overlayImage: this.createOverlay(image, normalizedHeatmap),
      topRegions: this.extractTopRegions(normalizedHeatmap)
    };
  }
}

// 흉부 X-ray 분석 모델
class ChestXRayAnalysisModel {
  modelId = 'chest-xray-densenet-v2';

  findings = [
    'No Finding',
    'Atelectasis',
    'Cardiomegaly',
    'Effusion',
    'Infiltration',
    'Mass',
    'Nodule',
    'Pneumonia',
    'Pneumothorax',
    'Consolidation',
    'Edema',
    'Emphysema',
    'Fibrosis',
    'Pleural_Thickening',
    'Hernia'
  ];

  async analyze(image: DicomImage): Promise<ChestXRayResult> {
    // DICOM 전처리
    const processed = await this.preprocessDicom(image);

    // 멀티라벨 분류 실행
    const predictions = await this.runInference(processed);

    // 결과 구조화
    const results: Finding[] = [];

    for (let i = 0; i < this.findings.length; i++) {
      if (predictions[i] > 0.5 && this.findings[i] !== 'No Finding') {
        results.push({
          finding: this.findings[i],
          probability: predictions[i],
          confidence: this.getConfidenceLevel(predictions[i]),
          location: await this.localizeGradCAM(processed, i),
          clinicalSignificance: this.getClinicalSignificance(this.findings[i])
        });
      }
    }

    // 확률순 정렬
    results.sort((a, b) => b.probability - a.probability);

    return {
      imageId: image.sopInstanceUid,
      studyId: image.studyInstanceUid,
      findings: results,
      normalProbability: predictions[0],
      overallAssessment: this.generateAssessment(results),
      recommendations: this.generateRecommendations(results),
      qualityMetrics: await this.assessImageQuality(processed)
    };
  }

  private generateAssessment(findings: Finding[]): string {
    if (findings.length === 0) {
      return '식별된 소견 없이 검토를 위해 제시됨';
    }

    const criticalFindings = findings.filter(f =>
      ['Pneumothorax', 'Pneumonia', 'Mass'].includes(f.finding)
    );

    if (criticalFindings.length > 0) {
      return `주의 필요: ${criticalFindings.map(f => f.finding).join(', ')} - 즉각적인 영상의학과 검토 권장`;
    }

    return `소견: ${findings.map(f => f.finding).join(', ')} - 표준 검토`;
  }
}
```

### 5.4 의료 LLM 통합

```typescript
// 의료 LLM 서비스
class MedicalLLMService {
  private llmClient: LLMClient;
  private safetyFilter: MedicalSafetyFilter;
  private groundingService: EvidenceGroundingService;
  private auditLogger: LLMAuditLogger;

  async generateClinicalResponse(
    query: ClinicalQuery,
    context: ClinicalContext
  ): Promise<LLMClinicalResponse> {
    // 컨텍스트 준비
    const enrichedContext = await this.prepareContext(query, context);

    // 안전 시스템 프롬프트 구성
    const systemPrompt = this.buildSafetyPrompt(query.type);

    // LLM 호출
    const rawResponse = await this.llmClient.generate({
      system: systemPrompt,
      messages: [
        {
          role: 'user',
          content: this.formatClinicalQuery(query, enrichedContext)
        }
      ],
      temperature: 0.1,  // 의료용 저온도
      maxTokens: 2000
    });

    // 안전 필터링
    const filteredResponse = await this.safetyFilter.filter(rawResponse);

    // 증거로 그라운딩
    const groundedResponse = await this.groundingService.ground(
      filteredResponse,
      query.type
    );

    // 감사 로깅
    await this.auditLogger.log({
      query,
      context,
      response: groundedResponse,
      timestamp: new Date()
    });

    return groundedResponse;
  }

  private buildSafetyPrompt(queryType: ClinicalQueryType): string {
    const basePrompt = `당신은 의료 문서 및 임상 정보 검색 지원을 위해 설계된 전문 의료 AI 도우미입니다.

핵심 안전 지침:
1. 절대 직접적인 진단이나 치료 결정을 내리지 마세요
2. 모든 제안은 임상의 검토가 필요함을 항상 명시하세요
3. 불확실할 때는 불확실성을 인정하세요
4. 가능한 경우 근거 기반 출처를 인용하세요
5. 긴급 상황은 즉각적인 의료 지원으로 의뢰하세요
6. 환자 개인정보를 보호하고 민감한 정보를 저장하지 마세요

응답 형식:
- 명확하고 간결하게 작성
- 적절한 경우 구조화된 형식 사용
- 주요 포인트 강조
- 제한사항과 불확실성 포함`;

    const typeSpecificPrompts: Record<ClinicalQueryType, string> = {
      CLINICAL_SUMMARY: `
추가 지침:
- 관련 임상 정보를 중심으로 요약
- 시간순 구성
- 핵심 소견 및 문제 강조
- 감별 진단을 언급하되 확정 진단하지 않음`,

      DIFFERENTIAL_DIAGNOSIS: `
추가 지침:
- 확률순으로 감별 진단 목록 제공
- 각각에 대한 증거와 반대 증거 포함
- 권장 검사 제안
- 이것이 의사결정 지원용이며 확정 진단이 아님을 명시`,

      TREATMENT_OPTIONS: `
추가 지침:
- 치료 옵션을 근거 수준과 함께 제시
- 잠재적 부작용 및 금기사항 포함
- 환자별 고려사항 언급
- 약물 제안 시 용량 범위 제공하되 정확한 처방은 처방의에게 의뢰`,

      LAB_INTERPRETATION: `
추가 지침:
- 맥락과 함께 검사 결과 해석
- 가능한 원인 식별
- 추가 검사 제안
- 해석에 영향을 미칠 수 있는 한계 언급`
    };

    return basePrompt + (typeSpecificPrompts[queryType] || '');
  }

  // 환각 탐지
  private async detectHallucinations(
    response: string,
    context: ClinicalContext
  ): Promise<HallucinationCheck> {
    const checks: VerificationResult[] = [];

    // 약물명 확인
    const medications = this.extractMedications(response);
    for (const med of medications) {
      const exists = await this.drugDatabase.exists(med);
      checks.push({
        type: 'MEDICATION',
        value: med,
        verified: exists,
        source: exists ? 'RxNorm' : null
      });
    }

    // 진단 코드 확인
    const diagnoses = this.extractDiagnoses(response);
    for (const dx of diagnoses) {
      const valid = await this.icdDatabase.validate(dx);
      checks.push({
        type: 'DIAGNOSIS',
        value: dx,
        verified: valid,
        source: valid ? 'ICD-10' : null
      });
    }

    // 통계 주장 확인
    const statistics = this.extractStatistics(response);
    for (const stat of statistics) {
      const verified = await this.factChecker.verify(stat);
      checks.push({
        type: 'STATISTIC',
        value: stat,
        verified: verified.confirmed,
        source: verified.source
      });
    }

    const unverifiedCount = checks.filter(c => !c.verified).length;

    return {
      checks,
      overallRisk: unverifiedCount > 3 ? 'HIGH' :
                   unverifiedCount > 0 ? 'MEDIUM' : 'LOW',
      unverifiedClaims: checks.filter(c => !c.verified)
    };
  }
}

// 의료 안전 필터
class MedicalSafetyFilter {
  private dangerousPatterns: RegExp[];
  private requiresReviewPatterns: RegExp[];

  constructor() {
    this.dangerousPatterns = [
      /당신은\s*(반드시|해야)\s*(복용|주사|투여)/gi,
      /처방전\s*없이/gi,
      /자가\s*진단/gi,
      /즉시\s*중단/gi,
      /이것이\s*확정\s*진단입니다/gi
    ];

    this.requiresReviewPatterns = [
      /용량/gi,
      /처방/gi,
      /치료/gi,
      /투약/gi
    ];
  }

  async filter(response: LLMRawResponse): Promise<FilteredResponse> {
    let filteredText = response.text;

    // 위험한 패턴 제거
    for (const pattern of this.dangerousPatterns) {
      if (pattern.test(filteredText)) {
        filteredText = filteredText.replace(
          pattern,
          '[안전 필터에 의해 제거됨 - 의료 전문가와 상담하세요]'
        );
      }
    }

    // 면책 조항 추가
    const hasReviewPatterns = this.requiresReviewPatterns.some(p =>
      p.test(filteredText)
    );

    if (hasReviewPatterns) {
      filteredText += '\n\n**면책 조항**: 이 정보는 의사결정 지원용이며 전문적인 의학적 조언을 대체하지 않습니다. 모든 임상 결정은 자격을 갖춘 의료 전문가가 내려야 합니다.';
    }

    return {
      text: filteredText,
      safetyFlags: this.identifySafetyFlags(response.text),
      modified: filteredText !== response.text
    };
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT 제어 프로토콜**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
