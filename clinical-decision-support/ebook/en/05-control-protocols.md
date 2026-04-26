# Chapter 5: Clinical Decision Support Control Protocols

## Inference Engines, Reasoning Systems, and AI Models

### 5.1 CDSS Reasoning Architecture

The WIA-CLINICAL-DECISION-SUPPORT standard defines comprehensive control protocols for clinical reasoning systems, including rule engines, machine learning models, and hybrid approaches that combine multiple inference methods.

```typescript
// CDSS Reasoning Architecture
interface CDSSReasoningArchitecture {
  version: '1.0.0';

  reasoningLayers: {
    ruleBasedLayer: {
      description: 'Deterministic clinical rules';
      engines: ['Drools', 'OpenL Tablets', 'CQL Engine'];
      useCases: ['Alerts', 'Contraindications', 'Guidelines'];
    };
    probabilisticLayer: {
      description: 'Bayesian and probabilistic reasoning';
      methods: ['Bayesian networks', 'Markov models'];
      useCases: ['Diagnostic reasoning', 'Risk assessment'];
    };
    machineLearningLayer: {
      description: 'Statistical learning models';
      frameworks: ['scikit-learn', 'XGBoost', 'LightGBM'];
      useCases: ['Risk prediction', 'Classification', 'Anomaly detection'];
    };
    deepLearningLayer: {
      description: 'Neural network models';
      frameworks: ['TensorFlow', 'PyTorch', 'ONNX'];
      useCases: ['Image analysis', 'NLP', 'Sequence prediction'];
    };
    llmLayer: {
      description: 'Large language model reasoning';
      models: ['GPT-4', 'Med-PaLM', 'Clinical BERT'];
      useCases: ['Summarization', 'Q&A', 'Report generation'];
    };
  };

  orchestration: {
    type: 'Ensemble with confidence-weighted fusion';
    conflictResolution: 'Priority-based with clinical review';
    explainability: 'Multi-level explanation generation';
  };
}
```

### 5.2 Rule Engine Implementation

```typescript
// Clinical Rule Engine
interface ClinicalRuleEngine {
  ruleTypes: {
    alertRules: AlertRule[];
    recommendationRules: RecommendationRule[];
    guidelineRules: GuidelineRule[];
    qualityRules: QualityRule[];
  };

  executionModel: {
    forwardChaining: boolean;
    backwardChaining: boolean;
    conflictResolution: ConflictResolutionStrategy;
  };
}

// Rule Definition Using CQL (Clinical Quality Language)
const drugInteractionRule: CQLLibrary = {
  libraryId: 'DrugInteractionAlerts',
  version: '1.0.0',
  cql: `
library DrugInteractionAlerts version '1.0.0'

using FHIR version '4.0.1'

include FHIRHelpers version '4.0.1' called FHIRHelpers

codesystem "RxNorm": 'http://www.nlm.nih.gov/research/umls/rxnorm'

valueset "Warfarin": 'http://cts.nlm.nih.gov/ValueSet/warfarin'
valueset "NSAIDs": 'http://cts.nlm.nih.gov/ValueSet/nsaids'
valueset "SSRIs": 'http://cts.nlm.nih.gov/ValueSet/ssris'

context Patient

define "Active Warfarin":
  [MedicationRequest: "Warfarin"] MR
    where MR.status = 'active'

define "Active NSAIDs":
  [MedicationRequest: "NSAIDs"] MR
    where MR.status = 'active'

define "Active SSRIs":
  [MedicationRequest: "SSRIs"] MR
    where MR.status = 'active'

define "Warfarin NSAID Interaction":
  exists("Active Warfarin") and exists("Active NSAIDs")

define "Warfarin SSRI Interaction":
  exists("Active Warfarin") and exists("Active SSRIs")

define "Triple Therapy Risk":
  exists("Active Warfarin") and exists("Active NSAIDs") and exists("Active SSRIs")

define "Interaction Severity":
  case
    when "Triple Therapy Risk" then 'CRITICAL'
    when "Warfarin NSAID Interaction" then 'HIGH'
    when "Warfarin SSRI Interaction" then 'MODERATE'
    else 'NONE'
  end

define "Alert Message":
  case
    when "Triple Therapy Risk" then
      'CRITICAL: Triple therapy (Warfarin + NSAID + SSRI) significantly increases bleeding risk'
    when "Warfarin NSAID Interaction" then
      'HIGH: Concurrent Warfarin and NSAID use increases bleeding risk'
    when "Warfarin SSRI Interaction" then
      'MODERATE: Concurrent Warfarin and SSRI use may increase bleeding risk'
    else null
  end
  `
};

// Rule Engine Implementation
class ClinicalRuleEngineService {
  private cqlEngine: CQLEngine;
  private ruleRepository: RuleRepository;
  private auditLogger: AuditLogger;

  async evaluateRules(
    patientData: PatientData,
    context: RuleContext
  ): Promise<RuleEvaluationResult> {
    const startTime = Date.now();
    const results: RuleResult[] = [];

    // Get applicable rules
    const applicableRules = await this.getApplicableRules(context);

    // Execute rules in priority order
    for (const rule of applicableRules) {
      try {
        const result = await this.executeRule(rule, patientData, context);
        results.push(result);

        // Check for stop conditions
        if (result.triggered && rule.stopOnTrigger) {
          break;
        }
      } catch (error) {
        this.logRuleError(rule, error);
      }
    }

    // Apply conflict resolution
    const resolvedResults = this.resolveConflicts(results);

    // Audit log
    await this.auditLogger.logRuleExecution({
      context,
      rulesEvaluated: applicableRules.length,
      rulesTriggered: results.filter(r => r.triggered).length,
      executionTime: Date.now() - startTime
    });

    return {
      results: resolvedResults,
      metadata: {
        rulesEvaluated: applicableRules.length,
        executionTime: Date.now() - startTime
      }
    };
  }

  private async executeRule(
    rule: ClinicalRule,
    patientData: PatientData,
    context: RuleContext
  ): Promise<RuleResult> {
    // Convert patient data to FHIR bundle
    const fhirBundle = this.convertToFHIR(patientData);

    // Execute CQL
    const cqlResult = await this.cqlEngine.execute(
      rule.cqlLibrary,
      fhirBundle,
      {
        libraryName: rule.cqlLibrary,
        expressions: rule.expressions
      }
    );

    // Check if rule triggered
    const triggered = this.evaluateTriggerCondition(rule, cqlResult);

    if (triggered) {
      return {
        ruleId: rule.id,
        ruleName: rule.name,
        triggered: true,
        severity: cqlResult['Interaction Severity'] || rule.defaultSeverity,
        message: cqlResult['Alert Message'] || rule.defaultMessage,
        details: this.extractDetails(cqlResult),
        suggestions: rule.suggestions,
        references: rule.references
      };
    }

    return {
      ruleId: rule.id,
      ruleName: rule.name,
      triggered: false
    };
  }

  private resolveConflicts(results: RuleResult[]): RuleResult[] {
    // Group by conflict category
    const groups = this.groupByConflictCategory(results);

    const resolved: RuleResult[] = [];

    for (const [category, groupResults] of Object.entries(groups)) {
      if (groupResults.length === 1) {
        resolved.push(...groupResults);
      } else {
        // Apply conflict resolution strategy
        const winner = this.selectWinner(groupResults);
        resolved.push(winner);
      }
    }

    return resolved;
  }

  private selectWinner(results: RuleResult[]): RuleResult {
    // Priority order: CRITICAL > HIGH > MODERATE > LOW
    const severityOrder = { 'CRITICAL': 0, 'HIGH': 1, 'MODERATE': 2, 'LOW': 3 };

    return results.sort((a, b) =>
      severityOrder[a.severity] - severityOrder[b.severity]
    )[0];
  }
}

// Temporal Reasoning for Clinical Rules
class TemporalReasoningEngine {
  async evaluateTemporalConditions(
    patientData: PatientData,
    temporalRules: TemporalRule[]
  ): Promise<TemporalResult[]> {
    const results: TemporalResult[] = [];

    for (const rule of temporalRules) {
      const result = await this.evaluateTemporalRule(rule, patientData);
      if (result.satisfied) {
        results.push(result);
      }
    }

    return results;
  }

  private async evaluateTemporalRule(
    rule: TemporalRule,
    patientData: PatientData
  ): Promise<TemporalResult> {
    switch (rule.temporalOperator) {
      case 'BEFORE':
        return this.evaluateBefore(rule, patientData);
      case 'AFTER':
        return this.evaluateAfter(rule, patientData);
      case 'DURING':
        return this.evaluateDuring(rule, patientData);
      case 'WITHIN':
        return this.evaluateWithin(rule, patientData);
      case 'SEQUENCE':
        return this.evaluateSequence(rule, patientData);
      default:
        throw new Error(`Unknown temporal operator: ${rule.temporalOperator}`);
    }
  }

  // Example: Check if INR was measured within 7 days before warfarin prescription
  private async evaluateWithin(
    rule: TemporalRule,
    patientData: PatientData
  ): Promise<TemporalResult> {
    const events1 = this.getEvents(patientData, rule.event1);
    const events2 = this.getEvents(patientData, rule.event2);

    for (const e2 of events2) {
      const windowStart = this.subtractDuration(e2.timestamp, rule.duration);
      const windowEnd = e2.timestamp;

      const matchingE1 = events1.find(e1 =>
        e1.timestamp >= windowStart && e1.timestamp <= windowEnd
      );

      if (!matchingE1 && rule.requireMatch) {
        return {
          ruleId: rule.id,
          satisfied: true,  // Rule violation detected
          message: `${rule.event1.display} not found within ${rule.duration} before ${rule.event2.display}`,
          recommendation: rule.recommendation
        };
      }
    }

    return { ruleId: rule.id, satisfied: false };
  }
}
```

### 5.3 Machine Learning Inference

```typescript
// ML Inference Service
interface MLInferenceService {
  models: {
    riskPrediction: RiskPredictionModel[];
    diagnosticSupport: DiagnosticModel[];
    treatmentSelection: TreatmentModel[];
    prognosis: PrognosticModel[];
  };

  infrastructure: {
    modelRegistry: ModelRegistry;
    featureStore: FeatureStore;
    inferenceEngine: InferenceEngine;
    monitoringService: ModelMonitoringService;
  };
}

// Risk Prediction Model Interface
interface RiskPredictionModel {
  modelId: string;
  modelName: string;
  version: string;
  modelType: 'LOGISTIC_REGRESSION' | 'RANDOM_FOREST' | 'GRADIENT_BOOSTING' | 'NEURAL_NETWORK';
  targetOutcome: string;
  predictionHorizon: string;
  features: ModelFeature[];
  performance: ModelPerformance;
  calibration: CalibrationInfo;
}

// ML Inference Implementation
class MLInferenceEngine {
  private modelRegistry: ModelRegistry;
  private featureStore: FeatureStore;
  private modelServers: Map<string, ModelServer>;
  private explainer: ModelExplainer;

  async predict(
    modelId: string,
    patientData: PatientData,
    options?: PredictionOptions
  ): Promise<PredictionResult> {
    // Get model
    const model = await this.modelRegistry.getModel(modelId);
    if (!model) {
      throw new ModelNotFoundError(modelId);
    }

    // Extract features
    const features = await this.extractFeatures(model, patientData);

    // Validate features
    const validation = this.validateFeatures(model, features);
    if (!validation.valid) {
      return this.handleMissingFeatures(model, features, validation);
    }

    // Run inference
    const rawPrediction = await this.runInference(model, features);

    // Calibrate
    const calibratedPrediction = this.calibrate(model, rawPrediction);

    // Generate explanation
    const explanation = await this.explainer.explain(
      model,
      features,
      calibratedPrediction,
      options?.explanationLevel ?? 'STANDARD'
    );

    // Calculate confidence
    const confidence = this.calculateConfidence(
      model,
      features,
      calibratedPrediction,
      validation
    );

    return {
      modelId,
      modelName: model.modelName,
      modelVersion: model.version,
      prediction: {
        value: calibratedPrediction.probability,
        class: calibratedPrediction.class,
        threshold: model.decisionThreshold
      },
      confidence,
      explanation,
      features: this.summarizeFeatures(features),
      warnings: validation.warnings,
      timestamp: new Date()
    };
  }

  private async extractFeatures(
    model: MLModel,
    patientData: PatientData
  ): Promise<FeatureVector> {
    const features: FeatureVector = {};

    for (const featureDef of model.features) {
      const value = await this.extractFeature(featureDef, patientData);
      features[featureDef.name] = value;
    }

    // Apply transformations
    return this.applyTransformations(model, features);
  }

  private async extractFeature(
    featureDef: ModelFeature,
    patientData: PatientData
  ): Promise<any> {
    switch (featureDef.source) {
      case 'DEMOGRAPHICS':
        return this.extractDemographicFeature(featureDef, patientData.demographics);
      case 'LABS':
        return this.extractLabFeature(featureDef, patientData.labs);
      case 'VITALS':
        return this.extractVitalFeature(featureDef, patientData.vitals);
      case 'DIAGNOSES':
        return this.extractDiagnosisFeature(featureDef, patientData.problems);
      case 'MEDICATIONS':
        return this.extractMedicationFeature(featureDef, patientData.medications);
      case 'CALCULATED':
        return this.calculateFeature(featureDef, patientData);
      default:
        throw new Error(`Unknown feature source: ${featureDef.source}`);
    }
  }

  private async runInference(
    model: MLModel,
    features: FeatureVector
  ): Promise<RawPrediction> {
    const modelServer = this.modelServers.get(model.servingEndpoint);

    // Convert features to model input format
    const inputTensor = this.featuresToTensor(model, features);

    // Call model server
    const response = await modelServer.predict({
      modelId: model.modelId,
      modelVersion: model.version,
      inputs: inputTensor
    });

    return {
      probability: response.outputs[0],
      class: response.outputs[0] >= model.decisionThreshold ? 1 : 0,
      rawScores: response.outputs
    };
  }

  private calibrate(
    model: MLModel,
    prediction: RawPrediction
  ): CalibratedPrediction {
    if (!model.calibration) {
      return prediction;
    }

    // Apply Platt scaling or isotonic regression
    const calibratedProbability = this.applyCalibration(
      prediction.probability,
      model.calibration
    );

    return {
      ...prediction,
      probability: calibratedProbability,
      class: calibratedProbability >= model.decisionThreshold ? 1 : 0
    };
  }
}

// SHAP Explainer for Model Explanations
class SHAPExplainer implements ModelExplainer {
  async explain(
    model: MLModel,
    features: FeatureVector,
    prediction: CalibratedPrediction,
    level: ExplanationLevel
  ): Promise<Explanation> {
    // Calculate SHAP values
    const shapValues = await this.calculateSHAPValues(model, features);

    // Get feature importance ranking
    const featureImportance = this.rankFeatures(shapValues);

    // Generate explanation
    return {
      type: 'SHAP',

      summary: this.generateSummary(
        model,
        prediction,
        featureImportance,
        level
      ),

      featureContributions: featureImportance.slice(0, level === 'BRIEF' ? 3 : 10).map(f => ({
        feature: f.name,
        displayName: f.displayName,
        value: features[f.name],
        contribution: f.shapValue,
        direction: f.shapValue > 0 ? 'INCREASES_RISK' : 'DECREASES_RISK',
        magnitude: Math.abs(f.shapValue)
      })),

      baseline: model.baselineRisk,

      visualization: level === 'DETAILED' ? {
        type: 'WATERFALL',
        data: this.generateWaterfallData(featureImportance, model.baselineRisk)
      } : undefined,

      confidence: this.assessExplanationConfidence(shapValues)
    };
  }

  private generateSummary(
    model: MLModel,
    prediction: CalibratedPrediction,
    featureImportance: FeatureImportance[],
    level: ExplanationLevel
  ): string {
    const riskLevel = this.getRiskLevel(prediction.probability);
    const topFactors = featureImportance.slice(0, 3);

    const increasingFactors = topFactors.filter(f => f.shapValue > 0);
    const decreasingFactors = topFactors.filter(f => f.shapValue < 0);

    let summary = `${model.targetOutcome} risk is ${riskLevel} (${(prediction.probability * 100).toFixed(1)}%). `;

    if (increasingFactors.length > 0) {
      summary += `Key factors increasing risk: ${increasingFactors.map(f => f.displayName).join(', ')}. `;
    }

    if (decreasingFactors.length > 0) {
      summary += `Protective factors: ${decreasingFactors.map(f => f.displayName).join(', ')}.`;
    }

    return summary;
  }
}

// Example: Sepsis Early Warning Model
const sepsisEarlyWarningModel: RiskPredictionModel = {
  modelId: 'sepsis-ew-v3',
  modelName: 'Sepsis Early Warning Score',
  version: '3.2.1',
  modelType: 'GRADIENT_BOOSTING',
  targetOutcome: 'Sepsis onset within 6 hours',
  predictionHorizon: '6 hours',

  features: [
    {
      name: 'heart_rate',
      displayName: 'Heart Rate',
      source: 'VITALS',
      extraction: 'LATEST',
      required: true,
      validation: { min: 20, max: 250 }
    },
    {
      name: 'respiratory_rate',
      displayName: 'Respiratory Rate',
      source: 'VITALS',
      extraction: 'LATEST',
      required: true,
      validation: { min: 4, max: 60 }
    },
    {
      name: 'temperature',
      displayName: 'Temperature',
      source: 'VITALS',
      extraction: 'LATEST',
      required: true,
      validation: { min: 30, max: 45 }
    },
    {
      name: 'systolic_bp',
      displayName: 'Systolic Blood Pressure',
      source: 'VITALS',
      extraction: 'LATEST',
      required: true,
      validation: { min: 40, max: 300 }
    },
    {
      name: 'wbc',
      displayName: 'White Blood Cell Count',
      source: 'LABS',
      loincCode: '6690-2',
      extraction: 'LATEST_WITHIN_24H',
      required: false
    },
    {
      name: 'lactate',
      displayName: 'Lactate',
      source: 'LABS',
      loincCode: '2524-7',
      extraction: 'LATEST_WITHIN_12H',
      required: false
    },
    {
      name: 'age',
      displayName: 'Age',
      source: 'DEMOGRAPHICS',
      extraction: 'CALCULATED',
      required: true
    },
    {
      name: 'hr_trend',
      displayName: 'Heart Rate Trend (6h)',
      source: 'CALCULATED',
      calculation: 'linear_regression_slope(heart_rate, 6h)',
      required: false
    }
  ],

  performance: {
    auc: 0.89,
    sensitivity: 0.85,
    specificity: 0.82,
    ppv: 0.45,
    npv: 0.97,
    calibrationSlope: 1.02,
    calibrationIntercept: -0.01,
    validationCohort: 'Multi-center ICU (n=45,000)',
    validationDate: new Date('2024-06-15')
  },

  calibration: {
    method: 'isotonic',
    parameters: { /* isotonic regression mapping */ }
  }
};
```

### 5.4 Deep Learning for Clinical Images and NLP

```typescript
// Deep Learning Service
interface DeepLearningService {
  imagingModels: {
    radiology: RadiologyModel[];
    pathology: PathologyModel[];
    dermatology: DermatologyModel[];
    ophthalmology: OphthalmologyModel[];
  };

  nlpModels: {
    namedEntityRecognition: NERModel;
    relationExtraction: RelationModel;
    documentClassification: ClassificationModel;
    summarization: SummarizationModel;
  };
}

// Medical Image Analysis
class MedicalImageAnalyzer {
  private modelRegistry: ModelRegistry;
  private preprocessor: ImagePreprocessor;
  private postprocessor: ResultPostprocessor;

  async analyzeImage(
    image: MedicalImage,
    analysisType: ImageAnalysisType,
    options?: ImageAnalysisOptions
  ): Promise<ImageAnalysisResult> {
    // Select appropriate model
    const model = await this.selectModel(image.modality, analysisType);

    // Preprocess image
    const preprocessed = await this.preprocessor.process(image, model.inputSpec);

    // Run inference
    const rawOutput = await this.runInference(model, preprocessed);

    // Post-process results
    const findings = await this.postprocessor.process(
      rawOutput,
      model,
      analysisType
    );

    // Generate explanation
    const explanation = await this.generateExplanation(
      model,
      preprocessed,
      rawOutput,
      findings
    );

    return {
      imageId: image.id,
      studyId: image.studyId,
      modality: image.modality,
      analysisType,
      model: {
        id: model.id,
        name: model.name,
        version: model.version
      },
      findings,
      explanation,
      confidence: this.calculateOverallConfidence(findings),
      processingTime: Date.now() - startTime,
      disclaimers: this.getDisclaimers(model, analysisType)
    };
  }

  private async generateExplanation(
    model: ImagingModel,
    image: ProcessedImage,
    output: ModelOutput,
    findings: Finding[]
  ): Promise<ImageExplanation> {
    // Generate attention/saliency map
    const attentionMap = await this.generateAttentionMap(model, image);

    // Generate GradCAM visualization
    const gradCam = await this.generateGradCAM(model, image, output);

    return {
      type: 'IMAGE_ATTENTION',
      attentionMap: attentionMap,
      gradCam: gradCam,
      highlightedRegions: findings.map(f => f.location),
      textDescription: this.generateTextExplanation(findings)
    };
  }
}

// Example: Chest X-Ray Analysis Model
const chestXRayModel: RadiologyModel = {
  id: 'cxr-findings-v4',
  name: 'Chest X-Ray Findings Detector',
  version: '4.1.0',
  modality: 'CR',  // Computed Radiography
  bodyPart: 'CHEST',

  architecture: 'DenseNet-121 with multi-label head',
  inputSpec: {
    imageSize: [1024, 1024],
    channels: 1,
    normalization: 'ImageNet',
    preprocessing: ['CLAHE', 'resize', 'normalize']
  },

  findings: [
    { code: 'consolidation', display: 'Consolidation', icd10: 'R91.1' },
    { code: 'pneumothorax', display: 'Pneumothorax', icd10: 'J93.9' },
    { code: 'cardiomegaly', display: 'Cardiomegaly', icd10: 'I51.7' },
    { code: 'pleural_effusion', display: 'Pleural Effusion', icd10: 'J90' },
    { code: 'atelectasis', display: 'Atelectasis', icd10: 'J98.11' },
    { code: 'nodule', display: 'Pulmonary Nodule', icd10: 'R91.1' },
    { code: 'mass', display: 'Lung Mass', icd10: 'R91.8' },
    { code: 'fracture', display: 'Rib Fracture', icd10: 'S22.3' }
  ],

  performance: {
    overall_auc: 0.91,
    per_finding: {
      consolidation: { auc: 0.94, sensitivity: 0.89, specificity: 0.92 },
      pneumothorax: { auc: 0.97, sensitivity: 0.95, specificity: 0.96 },
      cardiomegaly: { auc: 0.92, sensitivity: 0.88, specificity: 0.90 },
      pleural_effusion: { auc: 0.95, sensitivity: 0.91, specificity: 0.93 },
      atelectasis: { auc: 0.86, sensitivity: 0.80, specificity: 0.85 },
      nodule: { auc: 0.88, sensitivity: 0.82, specificity: 0.87 },
      mass: { auc: 0.90, sensitivity: 0.85, specificity: 0.89 },
      fracture: { auc: 0.89, sensitivity: 0.84, specificity: 0.88 }
    },
    validation: 'NIH ChestX-ray14 + Internal (n=200,000)'
  },

  regulatory: {
    fdaClearance: '510(k) K221234',
    ceMarking: 'Class IIa',
    intendedUse: 'Aid to radiologist in detecting findings on chest X-rays'
  }
};

// Clinical NLP Service
class ClinicalNLPService {
  private nerModel: NERModel;
  private relationModel: RelationExtractor;
  private llmService: MedicalLLMService;

  async extractClinicalEntities(
    text: string,
    documentType: string
  ): Promise<ClinicalEntities> {
    // Named Entity Recognition
    const entities = await this.nerModel.extract(text);

    // Entity linking to standard terminologies
    const linkedEntities = await this.linkEntities(entities);

    // Relation extraction
    const relations = await this.relationModel.extract(text, linkedEntities);

    // Assertion detection (negation, uncertainty, etc.)
    const assertions = await this.detectAssertions(linkedEntities, text);

    return {
      entities: linkedEntities,
      relations,
      assertions,
      sections: await this.detectSections(text, documentType)
    };
  }

  async summarizeClinicalDocument(
    document: ClinicalDocument,
    options: SummarizationOptions
  ): Promise<DocumentSummary> {
    // Extract key information
    const entities = await this.extractClinicalEntities(
      document.text,
      document.type
    );

    // Generate summary using LLM
    const summary = await this.llmService.summarize(
      document,
      entities,
      options
    );

    return {
      documentId: document.id,
      documentType: document.type,
      summary: {
        brief: summary.oneSentence,
        standard: summary.paragraph,
        structured: summary.structured
      },
      keyFindings: summary.keyFindings,
      medications: entities.entities.filter(e => e.type === 'MEDICATION'),
      diagnoses: entities.entities.filter(e => e.type === 'DIAGNOSIS'),
      procedures: entities.entities.filter(e => e.type === 'PROCEDURE'),
      followUp: summary.followUpItems
    };
  }
}
```

### 5.5 Large Language Model Integration

```typescript
// Medical LLM Service
interface MedicalLLMService {
  models: {
    general: 'GPT-4' | 'Claude-3' | 'Gemini';
    medical: 'Med-PaLM 2' | 'Clinical-T5' | 'BioGPT';
  };

  capabilities: {
    questionAnswering: boolean;
    summarization: boolean;
    reasoning: boolean;
    codeGeneration: boolean;  // For CQL, etc.
    translation: boolean;  // Medical terminology
  };

  safeguards: {
    groundingRequired: boolean;
    citationRequired: boolean;
    uncertaintyEstimation: boolean;
    hallucinationDetection: boolean;
  };
}

// Medical LLM Implementation
class MedicalLLMEngine {
  private llmClient: LLMClient;
  private groundingService: MedicalGroundingService;
  private safetyChecker: LLMSafetyChecker;
  private promptTemplates: PromptTemplateRepository;

  async answerClinicalQuestion(
    question: ClinicalQuestion,
    context: ClinicalContext
  ): Promise<LLMClinicalResponse> {
    // Build grounded prompt
    const prompt = await this.buildGroundedPrompt(question, context);

    // Call LLM with medical system prompt
    const rawResponse = await this.llmClient.complete({
      model: 'gpt-4-turbo',
      systemPrompt: MEDICAL_SYSTEM_PROMPT,
      userPrompt: prompt.text,
      temperature: 0.3,  // Lower for clinical accuracy
      maxTokens: 2000
    });

    // Ground response in medical knowledge
    const groundedResponse = await this.groundingService.ground(
      rawResponse,
      context
    );

    // Safety check
    const safetyResult = await this.safetyChecker.check(groundedResponse);
    if (!safetyResult.safe) {
      return this.handleUnsafeResponse(safetyResult);
    }

    // Extract citations
    const citations = await this.extractCitations(groundedResponse);

    // Estimate uncertainty
    const uncertainty = await this.estimateUncertainty(
      question,
      groundedResponse
    );

    return {
      answer: groundedResponse.text,
      confidence: 1 - uncertainty.overall,
      citations,
      evidenceLevel: this.assessEvidenceLevel(citations),
      uncertaintyFactors: uncertainty.factors,
      disclaimers: this.generateDisclaimers(question, uncertainty),
      relatedQuestions: await this.suggestRelatedQuestions(question)
    };
  }

  private async buildGroundedPrompt(
    question: ClinicalQuestion,
    context: ClinicalContext
  ): Promise<GroundedPrompt> {
    // Retrieve relevant knowledge
    const relevantKnowledge = await this.retrieveRelevantKnowledge(question);

    // Get patient context if available
    const patientContext = context.patientId
      ? await this.formatPatientContext(context.patientId)
      : null;

    // Build prompt with RAG
    const template = await this.promptTemplates.get(question.type);

    return {
      text: template.format({
        question: question.text,
        context: patientContext,
        knowledge: relevantKnowledge,
        guidelines: await this.getRelevantGuidelines(question)
      }),
      sources: relevantKnowledge.sources
    };
  }

  async generateDifferentialDiagnosis(
    presentation: ClinicalPresentation,
    patientContext: PatientContext
  ): Promise<DifferentialDiagnosisResult> {
    const prompt = `
Given the following clinical presentation and patient context, generate a differential diagnosis list:

**Patient Context:**
${this.formatPatientContext(patientContext)}

**Presenting Symptoms:**
${presentation.symptoms.map(s => `- ${s.name} (${s.severity}, ${s.duration})`).join('\n')}

**Physical Exam Findings:**
${presentation.examFindings.map(f => `- ${f.finding}: ${f.value}`).join('\n')}

**Recent Labs:**
${presentation.recentLabs?.map(l => `- ${l.name}: ${l.value} ${l.unit} (${l.flag || 'normal'})`).join('\n') || 'Not available'}

Please provide:
1. Top 5 differential diagnoses ranked by likelihood
2. For each diagnosis:
   - Key supporting features
   - Key features against
   - Recommended workup
3. "Can't miss" diagnoses to rule out
4. Recommended next steps

Base your reasoning on current clinical evidence and guidelines.
`;

    const response = await this.llmClient.complete({
      model: 'gpt-4-turbo',
      systemPrompt: DIFFERENTIAL_DIAGNOSIS_SYSTEM_PROMPT,
      userPrompt: prompt,
      temperature: 0.2
    });

    // Parse structured response
    const parsed = await this.parseDifferentialResponse(response);

    // Ground in medical knowledge
    const grounded = await this.groundDifferentials(parsed);

    // Add ICD codes
    const withCodes = await this.addDiagnosisCodes(grounded);

    return {
      differentials: withCodes.diagnoses,
      cantMiss: withCodes.cantMiss,
      recommendedWorkup: withCodes.workup,
      reasoning: parsed.reasoning,
      confidence: this.assessDifferentialConfidence(parsed),
      citations: grounded.citations
    };
  }
}

// Safety and Grounding
const MEDICAL_SYSTEM_PROMPT = `
You are a clinical decision support assistant. Your role is to help healthcare professionals by providing accurate, evidence-based medical information.

IMPORTANT GUIDELINES:
1. Always cite sources for medical claims
2. Clearly state uncertainty when present
3. Never make definitive diagnoses - provide differential considerations
4. Include relevant warnings and contraindications
5. Recommend consulting specialists when appropriate
6. Do not provide advice that could harm patients
7. Acknowledge limitations of AI in clinical decision-making

When uncertain, say so explicitly. When information is incomplete, note what additional information would be helpful.

Your responses should support, not replace, clinical judgment.
`;
```

---

**WIA-CLINICAL-DECISION-SUPPORT Control Protocols**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
