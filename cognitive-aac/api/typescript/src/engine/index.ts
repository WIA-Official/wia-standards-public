/**
 * WIA Cognitive AAC - Engine Module Exports
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

export { UIEngine, type AdaptiveUIEngine, type EngineConfig } from './UIEngine';
export {
  ProfileAdapter,
  isAutismProfile,
  isDementiaProfile,
} from './ProfileAdapter';
export {
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
} from './Presets';
