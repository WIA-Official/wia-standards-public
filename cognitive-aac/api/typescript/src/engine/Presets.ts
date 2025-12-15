/**
 * WIA Cognitive AAC - UI Presets
 * 인지 수준별 UI 설정 프리셋
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import {
  UIConfiguration,
  CognitiveLevel,
  AutismUIAdjustments,
  DementiaUIAdjustments,
  DementiaStage,
} from '../types';

// ============================================================================
// Cognitive Level Presets
// ============================================================================

/**
 * 인지 수준별 기본 UI 프리셋
 */
export const COGNITIVE_LEVEL_PRESETS: Record<CognitiveLevel, UIConfiguration> = {
  // 심각한 인지 손상 (Level 1)
  [CognitiveLevel.PROFOUND]: {
    grid: {
      columns: 2,
      rows: 2,
      cellSize: 'xlarge',
      gap: 16,
    },
    symbols: {
      type: 'photos',
      labelPosition: 'none',
      fontSize: 24,
      highContrast: true,
      borderRadius: 16,
    },
    interaction: {
      dwellTime: 2000,
      confirmationRequired: true,
      audioFeedback: true,
      hapticFeedback: true,
      highlightOnHover: true,
    },
    cognitiveLoad: {
      maxVisibleItems: 4,
      progressiveDisclosure: false,
      distractionFilter: 1.0,
      simplificationLevel: 5,
    },
    theme: {
      colorScheme: 'high-contrast',
      primaryColor: '#000000',
      backgroundColor: '#FFFFFF',
      textColor: '#000000',
      accentColor: '#0066CC',
      reducedMotion: true,
    },
  },

  // 중증 인지 손상 (Level 2)
  [CognitiveLevel.SEVERE]: {
    grid: {
      columns: 3,
      rows: 2,
      cellSize: 'large',
      gap: 12,
    },
    symbols: {
      type: 'photos',
      labelPosition: 'below',
      fontSize: 20,
      highContrast: true,
      borderRadius: 12,
    },
    interaction: {
      dwellTime: 1500,
      confirmationRequired: true,
      audioFeedback: true,
      hapticFeedback: true,
      highlightOnHover: true,
    },
    cognitiveLoad: {
      maxVisibleItems: 6,
      progressiveDisclosure: true,
      distractionFilter: 0.8,
      simplificationLevel: 4,
    },
    theme: {
      colorScheme: 'high-contrast',
      primaryColor: '#1A1A1A',
      backgroundColor: '#FFFFFF',
      textColor: '#1A1A1A',
      accentColor: '#0066CC',
      reducedMotion: true,
    },
  },

  // 중등도 인지 손상 (Level 3)
  [CognitiveLevel.MODERATE]: {
    grid: {
      columns: 4,
      rows: 3,
      cellSize: 'medium',
      gap: 8,
    },
    symbols: {
      type: 'pcs',
      labelPosition: 'below',
      fontSize: 16,
      highContrast: false,
      borderRadius: 8,
    },
    interaction: {
      dwellTime: 1000,
      confirmationRequired: false,
      audioFeedback: true,
      hapticFeedback: false,
      highlightOnHover: true,
    },
    cognitiveLoad: {
      maxVisibleItems: 12,
      progressiveDisclosure: true,
      distractionFilter: 0.5,
      simplificationLevel: 3,
    },
    theme: {
      colorScheme: 'light',
      primaryColor: '#333333',
      backgroundColor: '#F5F5F5',
      textColor: '#333333',
      accentColor: '#007AFF',
      reducedMotion: false,
    },
  },

  // 경미한 인지 손상 (Level 4)
  [CognitiveLevel.MILD]: {
    grid: {
      columns: 5,
      rows: 4,
      cellSize: 'medium',
      gap: 6,
    },
    symbols: {
      type: 'pcs',
      labelPosition: 'below',
      fontSize: 14,
      highContrast: false,
      borderRadius: 8,
    },
    interaction: {
      dwellTime: 800,
      confirmationRequired: false,
      audioFeedback: false,
      hapticFeedback: false,
      highlightOnHover: true,
    },
    cognitiveLoad: {
      maxVisibleItems: 20,
      progressiveDisclosure: true,
      distractionFilter: 0.2,
      simplificationLevel: 2,
    },
    theme: {
      colorScheme: 'light',
      primaryColor: '#444444',
      backgroundColor: '#FAFAFA',
      textColor: '#444444',
      accentColor: '#007AFF',
      reducedMotion: false,
    },
  },

  // 정상 범위 (Level 5)
  [CognitiveLevel.TYPICAL]: {
    grid: {
      columns: 6,
      rows: 5,
      cellSize: 'small',
      gap: 4,
    },
    symbols: {
      type: 'pcs',
      labelPosition: 'below',
      fontSize: 12,
      highContrast: false,
      borderRadius: 6,
    },
    interaction: {
      dwellTime: 0,
      confirmationRequired: false,
      audioFeedback: false,
      hapticFeedback: false,
      highlightOnHover: true,
    },
    cognitiveLoad: {
      maxVisibleItems: 30,
      progressiveDisclosure: false,
      distractionFilter: 0,
      simplificationLevel: 1,
    },
    theme: {
      colorScheme: 'light',
      primaryColor: '#555555',
      backgroundColor: '#FFFFFF',
      textColor: '#555555',
      accentColor: '#007AFF',
      reducedMotion: false,
    },
  },
};

// ============================================================================
// Autism-specific Presets
// ============================================================================

/**
 * 자폐 스펙트럼 지원 수준별 UI 조정 프리셋
 */
export const AUTISM_SUPPORT_PRESETS: Record<1 | 2 | 3, AutismUIAdjustments> = {
  // Level 1: 지원 필요
  1: {
    sensoryAccommodations: {
      reducedAnimation: false,
      mutedColors: false,
      noFlashing: true,
      quietMode: false,
    },
    predictability: {
      consistentLayout: true,
      transitionWarnings: true,
      visualSchedule: false,
      progressIndicator: true,
    },
    specialInterests: {
      enabled: true,
      interests: [],
      integratedInUI: true,
    },
  },

  // Level 2: 상당한 지원 필요
  2: {
    sensoryAccommodations: {
      reducedAnimation: true,
      mutedColors: true,
      noFlashing: true,
      quietMode: false,
    },
    predictability: {
      consistentLayout: true,
      transitionWarnings: true,
      visualSchedule: true,
      progressIndicator: true,
    },
    specialInterests: {
      enabled: true,
      interests: [],
      integratedInUI: true,
    },
  },

  // Level 3: 매우 상당한 지원 필요
  3: {
    sensoryAccommodations: {
      reducedAnimation: true,
      mutedColors: true,
      noFlashing: true,
      quietMode: true,
    },
    predictability: {
      consistentLayout: true,
      transitionWarnings: true,
      visualSchedule: true,
      progressIndicator: true,
    },
    specialInterests: {
      enabled: true,
      interests: [],
      integratedInUI: true,
    },
  },
};

/**
 * 감각 반응성에 따른 UI 조정
 */
export const SENSORY_ADJUSTMENTS = {
  visual: {
    hyper: {
      reduceBrightness: true,
      mutedColors: true,
      noFlashing: true,
      reducedAnimation: true,
    },
    hypo: {
      highContrast: true,
      brighterColors: true,
      animatedFeedback: true,
    },
    typical: {},
    mixed: {
      // 사용자 맞춤 필요
      customizable: true,
    },
  },
  auditory: {
    hyper: {
      quietMode: true,
      reducedVolume: true,
      noSuddenSounds: true,
    },
    hypo: {
      louderFeedback: true,
      repeatAudio: true,
    },
    typical: {},
    mixed: {
      customizable: true,
    },
  },
};

// ============================================================================
// Dementia-specific Presets
// ============================================================================

/**
 * 치매 단계별 UI 조정 프리셋
 */
export const DEMENTIA_STAGE_PRESETS: Record<DementiaStage, DementiaUIAdjustments> = {
  // 전임상 단계
  preclinical: {
    familiarity: {
      usePersonalPhotos: false,
      familiarFaces: [],
      personalLocations: [],
      lifeStoryIntegration: false,
    },
    simplification: {
      minimalNavigation: false,
      largeText: false,
      reducedChoices: 20,
      clearLabeling: true,
    },
    temporalSupport: {
      dayNightAwareness: false,
      mealTimeReminders: false,
      routineBasedSuggestions: false,
      showDateTime: true,
    },
    errorPrevention: {
      confirmBeforeAction: false,
      undoSupport: true,
      noDestructiveActions: false,
    },
  },

  // 경도 인지장애 (MCI)
  mci: {
    familiarity: {
      usePersonalPhotos: true,
      familiarFaces: [],
      personalLocations: [],
      lifeStoryIntegration: false,
    },
    simplification: {
      minimalNavigation: false,
      largeText: true,
      reducedChoices: 16,
      clearLabeling: true,
    },
    temporalSupport: {
      dayNightAwareness: true,
      mealTimeReminders: false,
      routineBasedSuggestions: true,
      showDateTime: true,
    },
    errorPrevention: {
      confirmBeforeAction: false,
      undoSupport: true,
      noDestructiveActions: false,
    },
  },

  // 초기 치매
  early: {
    familiarity: {
      usePersonalPhotos: true,
      familiarFaces: [],
      personalLocations: [],
      lifeStoryIntegration: true,
    },
    simplification: {
      minimalNavigation: true,
      largeText: true,
      reducedChoices: 12,
      clearLabeling: true,
    },
    temporalSupport: {
      dayNightAwareness: true,
      mealTimeReminders: true,
      routineBasedSuggestions: true,
      showDateTime: true,
    },
    errorPrevention: {
      confirmBeforeAction: true,
      undoSupport: true,
      noDestructiveActions: true,
    },
  },

  // 중기 치매
  middle: {
    familiarity: {
      usePersonalPhotos: true,
      familiarFaces: [],
      personalLocations: [],
      lifeStoryIntegration: true,
    },
    simplification: {
      minimalNavigation: true,
      largeText: true,
      reducedChoices: 6,
      clearLabeling: true,
    },
    temporalSupport: {
      dayNightAwareness: true,
      mealTimeReminders: true,
      routineBasedSuggestions: true,
      showDateTime: true,
    },
    errorPrevention: {
      confirmBeforeAction: true,
      undoSupport: true,
      noDestructiveActions: true,
    },
  },

  // 말기 치매
  late: {
    familiarity: {
      usePersonalPhotos: true,
      familiarFaces: [],
      personalLocations: [],
      lifeStoryIntegration: true,
    },
    simplification: {
      minimalNavigation: true,
      largeText: true,
      reducedChoices: 4,
      clearLabeling: true,
    },
    temporalSupport: {
      dayNightAwareness: true,
      mealTimeReminders: true,
      routineBasedSuggestions: true,
      showDateTime: true,
    },
    errorPrevention: {
      confirmBeforeAction: true,
      undoSupport: true,
      noDestructiveActions: true,
    },
  },
};

// ============================================================================
// Cell Size Mappings
// ============================================================================

/**
 * 셀 크기별 픽셀 값 매핑
 */
export const CELL_SIZE_PX = {
  small: 80,
  medium: 120,
  large: 160,
  xlarge: 200,
} as const;

/**
 * 셀 크기별 최소 터치 영역 (접근성)
 */
export const MIN_TOUCH_TARGET = {
  small: 44,
  medium: 48,
  large: 56,
  xlarge: 64,
} as const;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * 인지 수준에 따른 기본 프리셋 반환
 */
export function getPresetForLevel(level: CognitiveLevel): UIConfiguration {
  return COGNITIVE_LEVEL_PRESETS[level];
}

/**
 * 자폐 지원 수준에 따른 조정 반환
 */
export function getAutismAdjustments(supportLevel: 1 | 2 | 3): AutismUIAdjustments {
  return AUTISM_SUPPORT_PRESETS[supportLevel];
}

/**
 * 치매 단계에 따른 조정 반환
 */
export function getDementiaAdjustments(stage: DementiaStage): DementiaUIAdjustments {
  return DEMENTIA_STAGE_PRESETS[stage];
}

/**
 * 두 UIConfiguration을 병합
 */
export function mergeConfigurations(
  base: UIConfiguration,
  overrides: Partial<UIConfiguration>
): UIConfiguration {
  return {
    grid: { ...base.grid, ...overrides.grid },
    symbols: { ...base.symbols, ...overrides.symbols },
    interaction: { ...base.interaction, ...overrides.interaction },
    cognitiveLoad: { ...base.cognitiveLoad, ...overrides.cognitiveLoad },
    theme: overrides.theme ? { ...base.theme, ...overrides.theme } : base.theme,
  };
}
