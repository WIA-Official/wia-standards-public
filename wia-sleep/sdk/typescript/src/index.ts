/**
 * WIA-SLEEP SDK
 *
 * World Interoperability for Sleep Optimization
 *
 * A comprehensive TypeScript SDK for chronobiological sleep optimization,
 * circadian phase tracking, and personalized sleep interventions.
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 */

// ============================================================================
// Type Exports
// ============================================================================

// Sleep Record Types
export {
  TimeString,
  DurationMinutes,
  Percentage,
  SleepDataSource,
  ScoringMethod,
  SleepStage,
  SleepStagePercentages,
  SleepStageDurations,
  SleepCycle,
  Hypnogram,
  SleepMicrostructure,
  SleepQualityMetrics,
  SleepArchitectureSummary,
  SleepArchitecture,
  EnvironmentalFactors,
  SleepDisorderMetrics,
  SleepRecord,
  SleepRecordInput
} from './types/SleepRecord';

// Chronotype Types
export {
  ChronotypeClass,
  SleepSchedule,
  MCTQResponses,
  ActimetryData,
  DLMOMethod,
  ChronotypeBiologicalMarkers,
  TimeWindow,
  ChronotypeAssessmentInput,
  ChronotypeResult,
  ChronotypeProfile,
  ChronotypeRecommendation,
  ChronotypeInterpretation
} from './types/Chronotype';

// Circadian Marker Types
export {
  EntrainmentStatus,
  MelatoninSampleType,
  MelatoninMeasurement,
  MelatoninRhythm,
  CortisolAwakeningResponse,
  CortisolRhythm,
  TemperatureMeasurement,
  TemperatureRhythm,
  HRVMetrics,
  HRVCircadianModulation,
  HRVCircadianData,
  ActivityRhythm,
  CircadianPhase,
  AlertnessLevel,
  AlertnessWindow,
  CircadianPhaseEstimate,
  CircadianMarkersRecord,
  CircadianMarkersInput
} from './types/CircadianMarker';

// Intervention Types
export {
  InterventionType,
  OptimizationGoal,
  LightTimingReference,
  LightTherapySession,
  LightAvoidanceWindow,
  LightPrescription,
  LightPrescriptionRequest,
  TemperatureSchedule,
  MealTimingPlan,
  ExerciseTimingPlan,
  SleepHygieneIntervention,
  OptimizationOutcomes,
  TreatmentPhase,
  MonitoringPlan,
  PlanWarning,
  OptimizationRequest,
  CurrentSleepState,
  TargetSleepState,
  SleepOptimizationPlan,
  Intervention,
  SleepNeedRequest,
  SleepNeedPrediction
} from './types/Intervention';

// ============================================================================
// Service Exports
// ============================================================================

export { ChronotypeService } from './services/ChronotypeService';
export { SleepAnalyzer } from './services/SleepAnalyzer';
export { CircadianCalculator } from './services/CircadianCalculator';
export { OptimizationEngine } from './services/OptimizationEngine';

// ============================================================================
// Utility Exports
// ============================================================================

export {
  calculateDLMO,
  calculateDLMOff,
  calculateMelatoninDuration,
  estimateDLMOFromBedtime,
  calculatePhaseAngleToSleep,
  interpretPhaseAngle,
  MelatoninSample,
  DLMOResult
} from './utils/melatoninPhaseCalculator';

export {
  calculateStagePercentages,
  analyzeStageTransitions,
  calculateStageContinuity,
  validateHypnogram,
  getStageName,
  assessStageBalance,
  AASM_2020_RULES,
  SleepScoringRules,
  StageTransitions
} from './utils/sleepStageScorer';

export {
  estimateMelanopicEDI,
  calculateCircadianIlluminance,
  calculateLightDose,
  calculatePhaseShiftFromTiming,
  calculateCumulativeDose,
  recommendPhaseAdvanceTherapy,
  recommendPhaseDelayTherapy,
  getDailyLightRequirements,
  LightSource,
  LightExposure,
  LightDose,
  LightTherapyRecommendation,
  LightSpectrumType
} from './utils/lightDoseCalculator';

// ============================================================================
// SDK Version and Metadata
// ============================================================================

/**
 * WIA-SLEEP SDK version
 */
export const VERSION = '1.0.0';

/**
 * WIA-SLEEP Standard version
 */
export const STANDARD_VERSION = '1.0.0';

/**
 * SDK metadata
 */
export const SDK_INFO = {
  name: '@wia/sleep-sdk',
  version: VERSION,
  standardVersion: STANDARD_VERSION,
  description: 'WIA-SLEEP SDK - Chronobiological sleep optimization',
  repository: 'https://github.com/WIA-Official/wia-standards',
  documentation: 'https://wiastandards.com/wia-sleep',
  license: 'MIT'
};

// ============================================================================
// Convenience Factory Functions
// ============================================================================

/**
 * Create a new ChronotypeService instance
 */
export function createChronotypeService() {
  return ChronotypeService;
}

/**
 * Create a new SleepAnalyzer instance with default reference values
 */
export function createSleepAnalyzer() {
  return new SleepAnalyzer();
}

/**
 * Create a new CircadianCalculator instance
 */
export function createCircadianCalculator() {
  return CircadianCalculator;
}

/**
 * Create a new OptimizationEngine instance
 */
export function createOptimizationEngine() {
  return OptimizationEngine;
}

// ============================================================================
// Default Export
// ============================================================================

export default {
  VERSION,
  STANDARD_VERSION,
  SDK_INFO,
  ChronotypeService,
  SleepAnalyzer,
  CircadianCalculator,
  OptimizationEngine,
  createChronotypeService,
  createSleepAnalyzer,
  createCircadianCalculator,
  createOptimizationEngine
};
