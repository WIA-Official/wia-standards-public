/**
 * WIA Cognitive AAC - Profile Adapter
 * 인지 프로파일을 UI 설정으로 변환하는 어댑터
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import {
  CognitiveProfile,
  AutismProfile,
  DementiaProfile,
  UIConfiguration,
  AutismUIAdjustments,
  DementiaUIAdjustments,
  CognitiveLevel,
  GridConfig,
  SymbolConfig,
  InteractionConfig,
  CognitiveLoadConfig,
  ThemeConfig,
  SensoryReactivity,
} from '../types';

import {
  COGNITIVE_LEVEL_PRESETS,
  AUTISM_SUPPORT_PRESETS,
  DEMENTIA_STAGE_PRESETS,
  SENSORY_ADJUSTMENTS,
  mergeConfigurations,
} from './Presets';

// ============================================================================
// Profile Type Guards
// ============================================================================

/**
 * AutismProfile 타입 가드
 */
export function isAutismProfile(profile: CognitiveProfile): profile is AutismProfile {
  return (profile as AutismProfile).profileType === 'autism';
}

/**
 * DementiaProfile 타입 가드
 */
export function isDementiaProfile(profile: CognitiveProfile): profile is DementiaProfile {
  return (profile as DementiaProfile).profileType === 'dementia';
}

// ============================================================================
// Profile Adapter Class
// ============================================================================

export class ProfileAdapter {
  /**
   * 인지 프로파일에서 UI 설정 생성
   */
  static generateUIConfiguration(profile: CognitiveProfile): UIConfiguration {
    // 1. 기본 프리셋 가져오기
    const baseConfig = COGNITIVE_LEVEL_PRESETS[profile.summary.overallLevel];

    // 2. 프로파일 유형에 따른 조정
    if (isAutismProfile(profile)) {
      return this.applyAutismAdjustments(baseConfig, profile);
    }

    if (isDementiaProfile(profile)) {
      return this.applyDementiaAdjustments(baseConfig, profile);
    }

    // 3. 일반 프로파일: 도메인 기반 미세 조정
    return this.applyDomainAdjustments(baseConfig, profile);
  }

  /**
   * 자폐 프로파일에 특화된 조정 적용
   */
  private static applyAutismAdjustments(
    baseConfig: UIConfiguration,
    profile: AutismProfile
  ): UIConfiguration {
    const supportLevel = profile.diagnosis.supportLevel;
    const autismAdjustments = AUTISM_SUPPORT_PRESETS[supportLevel];

    let config = { ...baseConfig };

    // 감각 처리 조정
    config = this.applySensoryAdjustments(config, profile.sensoryProcessing);

    // 예측 가능성 조정
    if (autismAdjustments.predictability.consistentLayout) {
      config.cognitiveLoad.progressiveDisclosure = false;
    }

    // 감각 과민 조정
    if (autismAdjustments.sensoryAccommodations.reducedAnimation) {
      config.theme = {
        ...config.theme!,
        reducedMotion: true,
      };
    }

    if (autismAdjustments.sensoryAccommodations.mutedColors) {
      config.theme = {
        ...config.theme!,
        colorScheme: 'light',
      };
    }

    // 루틴 고집 수준에 따른 조정
    if (profile.routineAdherence) {
      const rigidity = profile.routineAdherence.rigidityLevel;
      if (rigidity >= 7) {
        config.cognitiveLoad.distractionFilter = Math.max(
          config.cognitiveLoad.distractionFilter,
          0.8
        );
      }
    }

    // 전환 어려움에 따른 조정
    if (profile.routineAdherence?.transitionDifficulty >= 7) {
      config.interaction.confirmationRequired = true;
    }

    // 언어 상태에 따른 상징 조정
    switch (profile.diagnosis.languageStatus) {
      case 'nonverbal':
      case 'minimally_verbal':
        config.symbols.type = 'photos';
        config.symbols.labelPosition = 'none';
        break;
      case 'verbal_limited':
        config.symbols.type = 'pcs';
        config.symbols.labelPosition = 'below';
        break;
      case 'verbal_fluent':
        // 기본 설정 유지
        break;
    }

    return config;
  }

  /**
   * 치매 프로파일에 특화된 조정 적용
   */
  private static applyDementiaAdjustments(
    baseConfig: UIConfiguration,
    profile: DementiaProfile
  ): UIConfiguration {
    const stage = profile.diagnosis.stage;
    const dementiaAdjustments = DEMENTIA_STAGE_PRESETS[stage];

    let config = { ...baseConfig };

    // 단순화 조정
    if (dementiaAdjustments.simplification.minimalNavigation) {
      config.cognitiveLoad.progressiveDisclosure = false;
      config.cognitiveLoad.maxVisibleItems = dementiaAdjustments.simplification.reducedChoices;
    }

    if (dementiaAdjustments.simplification.largeText) {
      config.symbols.fontSize = Math.max(config.symbols.fontSize, 18);
    }

    // 친숙함 조정
    if (dementiaAdjustments.familiarity.usePersonalPhotos) {
      config.symbols.type = 'photos';
    }

    // 오류 방지 조정
    if (dementiaAdjustments.errorPrevention.confirmBeforeAction) {
      config.interaction.confirmationRequired = true;
    }

    // 단계별 그리드 조정
    config.grid = this.getGridForDementiaStage(stage);

    // 보존된 능력 활용
    if (profile.preservedAbilities?.readingAbility?.preserved) {
      config.symbols.labelPosition = 'below';
    }

    // 행동 증상 고려
    if (profile.behavioralSymptoms?.sundowning) {
      // 시간대별 설정 적용 (별도 로직 필요)
    }

    return config;
  }

  /**
   * 도메인 기반 미세 조정
   */
  private static applyDomainAdjustments(
    baseConfig: UIConfiguration,
    profile: CognitiveProfile
  ): UIConfiguration {
    let config = { ...baseConfig };
    const domains = profile.domains;

    // 기억력 조정
    if (domains.memory.workingMemory.level <= CognitiveLevel.MODERATE) {
      config.cognitiveLoad.maxVisibleItems = Math.min(
        config.cognitiveLoad.maxVisibleItems,
        domains.memory.immediateRecall.span || 6
      );
    }

    // 주의력 조정
    if (domains.attention.sustained.level <= CognitiveLevel.MODERATE) {
      const duration = domains.attention.sustained.durationMinutes || 5;
      // 집중 시간이 짧으면 더 단순한 인터페이스
      if (duration < 5) {
        config.cognitiveLoad.simplificationLevel = Math.max(
          config.cognitiveLoad.simplificationLevel,
          4
        );
      }
    }

    if (domains.attention.selective.distractibility) {
      const distractibility = domains.attention.selective.distractibility;
      if (distractibility >= 7) {
        config.cognitiveLoad.distractionFilter = Math.max(
          config.cognitiveLoad.distractionFilter,
          0.7
        );
      }
    }

    // 주의 전환 조정
    if (domains.attention.shifting.transitionTimeSeconds) {
      const transitionTime = domains.attention.shifting.transitionTimeSeconds;
      if (transitionTime > 3) {
        config.interaction.dwellTime = Math.max(config.interaction.dwellTime, 1500);
      }
    }

    // 언어 조정
    if (domains.language.receptive.level <= CognitiveLevel.SEVERE) {
      config.symbols.labelPosition = 'none';
      config.symbols.type = 'photos';
    }

    // 시공간 조정
    if (domains.visualSpatial.visualPerception.level <= CognitiveLevel.MODERATE) {
      config.grid.cellSize = this.increaseCellSize(config.grid.cellSize);
      config.symbols.highContrast = true;
    }

    // 실행 기능 조정
    if (domains.executive.planning.level <= CognitiveLevel.MODERATE) {
      config.cognitiveLoad.progressiveDisclosure = true;
    }

    return config;
  }

  /**
   * 감각 처리 프로파일에 따른 조정
   */
  private static applySensoryAdjustments(
    config: UIConfiguration,
    sensoryProcessing: AutismProfile['sensoryProcessing']
  ): UIConfiguration {
    let adjustedConfig = { ...config };

    // 시각 감각 조정
    const visual = sensoryProcessing.visual;
    if (visual.reactivity === 'hyper') {
      adjustedConfig.theme = {
        ...adjustedConfig.theme!,
        colorScheme: 'light',
        reducedMotion: true,
      };
      adjustedConfig.symbols.highContrast = false; // 너무 강한 대비 피함
    } else if (visual.reactivity === 'hypo') {
      adjustedConfig.symbols.highContrast = true;
      adjustedConfig.theme = {
        ...adjustedConfig.theme!,
        accentColor: '#FF6600', // 더 눈에 띄는 색상
      };
    }

    // 청각 감각 조정
    const auditory = sensoryProcessing.auditory;
    if (auditory.reactivity === 'hyper') {
      adjustedConfig.interaction.audioFeedback = false;
    } else if (auditory.reactivity === 'hypo') {
      adjustedConfig.interaction.audioFeedback = true;
    }

    // 촉각 감각 조정
    const tactile = sensoryProcessing.tactile;
    if (tactile.reactivity === 'hyper') {
      adjustedConfig.interaction.hapticFeedback = false;
    } else if (tactile.reactivity === 'hypo') {
      adjustedConfig.interaction.hapticFeedback = true;
    }

    return adjustedConfig;
  }

  /**
   * 치매 단계별 그리드 설정
   */
  private static getGridForDementiaStage(stage: DementiaProfile['diagnosis']['stage']): GridConfig {
    switch (stage) {
      case 'preclinical':
        return { columns: 5, rows: 4, cellSize: 'medium', gap: 6 };
      case 'mci':
        return { columns: 4, rows: 3, cellSize: 'medium', gap: 8 };
      case 'early':
        return { columns: 4, rows: 3, cellSize: 'large', gap: 10 };
      case 'middle':
        return { columns: 3, rows: 2, cellSize: 'large', gap: 12 };
      case 'late':
        return { columns: 2, rows: 2, cellSize: 'xlarge', gap: 16 };
      default:
        return { columns: 4, rows: 3, cellSize: 'medium', gap: 8 };
    }
  }

  /**
   * 셀 크기 한 단계 증가
   */
  private static increaseCellSize(
    current: GridConfig['cellSize']
  ): GridConfig['cellSize'] {
    const sizes: GridConfig['cellSize'][] = ['small', 'medium', 'large', 'xlarge'];
    const currentIndex = sizes.indexOf(current);
    return sizes[Math.min(currentIndex + 1, sizes.length - 1)];
  }

  /**
   * 자폐 프로파일용 특수 관심사 UI 요소 생성
   */
  static getSpecialInterestElements(profile: AutismProfile): {
    topics: string[];
    canUseForMotivation: boolean;
    suggestedIntegration: 'theme' | 'rewards' | 'vocabulary' | 'none';
  } {
    const interests = profile.specialInterests;

    if (!interests || !interests.usableForAAC) {
      return {
        topics: [],
        canUseForMotivation: false,
        suggestedIntegration: 'none',
      };
    }

    const functionalTopics = interests.topics
      .filter((t) => t.functionalUse)
      .map((t) => t.topic);

    const highIntensityTopics = interests.topics
      .filter((t) => t.intensity >= 7)
      .map((t) => t.topic);

    return {
      topics: [...new Set([...functionalTopics, ...highIntensityTopics])],
      canUseForMotivation: functionalTopics.length > 0,
      suggestedIntegration:
        highIntensityTopics.length > 0
          ? 'theme'
          : functionalTopics.length > 0
            ? 'rewards'
            : 'none',
    };
  }

  /**
   * 치매 프로파일용 회상 요소 생성
   */
  static getReminiscenceElements(profile: DementiaProfile): {
    familyPhotos: { name: string; relationship: string }[];
    preferredMusic: string[];
    accessibleMemoryPeriods: string[];
    suggestedActivities: string[];
  } {
    const history = profile.personalHistory;
    const preserved = profile.preservedAbilities;

    const familyPhotos = (history?.familyMembers || [])
      .filter((m) => m.recognitionLevel === 'full' || m.recognitionLevel === 'sometimes')
      .map((m) => ({ name: m.name, relationship: m.relationship }));

    return {
      familyPhotos,
      preferredMusic: preserved?.musicalAbility?.preferredMusic || [],
      accessibleMemoryPeriods: preserved?.remoteMemory?.accessiblePeriods || [],
      suggestedActivities: this.getSuggestedActivities(profile),
    };
  }

  /**
   * 치매 단계별 권장 활동
   */
  private static getSuggestedActivities(profile: DementiaProfile): string[] {
    const stage = profile.diagnosis.stage;
    const preserved = profile.preservedAbilities;

    const activities: string[] = [];

    if (preserved?.musicalAbility?.preserved) {
      activities.push('music_listening', 'singing');
    }

    if (preserved?.proceduralMemory?.preserved) {
      activities.push(...(preserved.proceduralMemory.examples || []));
    }

    switch (stage) {
      case 'preclinical':
      case 'mci':
        activities.push('reading', 'puzzles', 'conversation');
        break;
      case 'early':
        activities.push('photo_viewing', 'simple_games', 'reminiscence');
        break;
      case 'middle':
        activities.push('music', 'sensory_activities', 'comfort');
        break;
      case 'late':
        activities.push('presence', 'touch', 'familiar_voices');
        break;
    }

    return [...new Set(activities)];
  }
}

export default ProfileAdapter;
