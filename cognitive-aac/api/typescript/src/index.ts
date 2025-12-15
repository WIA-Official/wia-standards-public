/**
 * WIA Cognitive AAC - Main Entry Point
 * 인지 장애 사용자를 위한 적응형 AAC 표준 라이브러리
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 *
 * @packageDocumentation
 */

// Types
export * from './types';

// Engine
export {
  UIEngine,
  type AdaptiveUIEngine,
  type EngineConfig,
  ProfileAdapter,
  isAutismProfile,
  isDementiaProfile,
  COGNITIVE_LEVEL_PRESETS,
  AUTISM_SUPPORT_PRESETS,
  DEMENTIA_STAGE_PRESETS,
  SENSORY_ADJUSTMENTS,
  CELL_SIZE_PX,
  MIN_TOUCH_TARGET,
  getPresetForLevel,
  getAutismAdjustments,
  getDementiaAdjustments,
  mergeConfigurations,
} from './engine';

// Components
export {
  AdaptiveSymbol,
  type AdaptiveSymbolProps,
  AdaptiveGrid,
  type AdaptiveGridProps,
  AdaptiveButton,
  type AdaptiveButtonProps,
  type ButtonVariant,
  type ButtonSize,
  CognitiveLoadManagerProvider,
  useCognitiveLoadManager,
  CognitiveLoadIndicator,
  type CognitiveLoadState,
  type CognitiveLoadManagerContextValue,
  type LoadIndicatorProps,
} from './components';

// Hooks
export {
  useCognitiveProfile,
  type UseCognitiveProfileOptions,
  type UseCognitiveProfileReturn,
  useAdaptiveUI,
  type UseAdaptiveUIOptions,
  type UseAdaptiveUIReturn,
} from './hooks';

// Prediction (Phase 3)
export {
  PredictionEngine,
  PatternLearner,
  ContextDetector,
  PrivacyManager,
  type Context,
  type PredictedSymbol,
  type PredictionResult,
  type UsagePattern,
  type PrivacySettings,
  type PredictionEngineConfig,
  type RoutineBasedPrediction,
  type ReminiscenceBasedPrediction,
} from './prediction';

// Version
export const VERSION = '1.0.0';

// Default export
export { UIEngine as default } from './engine';
