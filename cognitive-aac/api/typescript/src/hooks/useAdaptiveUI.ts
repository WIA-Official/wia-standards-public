/**
 * WIA Cognitive AAC - useAdaptiveUI Hook
 * 적응형 UI 설정 관리 훅
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import { useState, useCallback, useMemo, useEffect, useRef } from 'react';
import {
  CognitiveProfile,
  AutismProfile,
  DementiaProfile,
  UIConfiguration,
  UIAdjustment,
  UserInteractionEvent,
  AutismUIAdjustments,
  DementiaUIAdjustments,
} from '../types';
import { UIEngine, EngineConfig } from '../engine/UIEngine';
import {
  ProfileAdapter,
  isAutismProfile,
  isDementiaProfile,
} from '../engine/ProfileAdapter';
import { COGNITIVE_LEVEL_PRESETS } from '../engine/Presets';

// ============================================================================
// Types
// ============================================================================

export interface UseAdaptiveUIOptions {
  profile: CognitiveProfile;
  engineConfig?: Partial<EngineConfig>;
  onConfigChange?: (config: UIConfiguration, adjustment: UIAdjustment | null) => void;
  autoStart?: boolean;
}

export interface UseAdaptiveUIReturn {
  // UI 설정
  config: UIConfiguration;
  baseConfig: UIConfiguration;

  // 특화 조정
  autismAdjustments: AutismUIAdjustments | null;
  dementiaAdjustments: DementiaUIAdjustments | null;

  // 특수 요소
  specialInterestElements: ReturnType<typeof ProfileAdapter.getSpecialInterestElements> | null;
  reminiscenceElements: ReturnType<typeof ProfileAdapter.getReminiscenceElements> | null;

  // 조정 기록
  adjustmentHistory: UIAdjustment[];

  // 엔진 상태
  isActive: boolean;
  loadLevel: number;

  // 액션
  recordInteraction: (event: Omit<UserInteractionEvent, 'timestamp'>) => void;
  applyAdjustment: (adjustment: Partial<UIConfiguration>) => void;
  resetConfig: () => void;
  simplify: () => void;
  restore: () => void;

  // 엔진 제어
  start: () => void;
  stop: () => void;
  pause: () => void;
  resume: () => void;
}

// ============================================================================
// Hook Implementation
// ============================================================================

export function useAdaptiveUI(options: UseAdaptiveUIOptions): UseAdaptiveUIReturn {
  const { profile, engineConfig, onConfigChange, autoStart = true } = options;

  // Engine ref
  const engineRef = useRef<UIEngine | null>(null);

  // 상태
  const [config, setConfig] = useState<UIConfiguration>(
    COGNITIVE_LEVEL_PRESETS[profile.summary.overallLevel]
  );
  const [baseConfig, setBaseConfig] = useState<UIConfiguration>(config);
  const [adjustmentHistory, setAdjustmentHistory] = useState<UIAdjustment[]>([]);
  const [isActive, setIsActive] = useState(autoStart);
  const [isPaused, setIsPaused] = useState(false);
  const [loadLevel, setLoadLevel] = useState(0);

  // Engine 초기화
  useEffect(() => {
    // 프로파일 유형에 따른 엔진 생성
    if (isAutismProfile(profile)) {
      engineRef.current = UIEngine.forAutism(profile as AutismProfile, engineConfig);
    } else if (isDementiaProfile(profile)) {
      engineRef.current = UIEngine.forDementia(profile as DementiaProfile, engineConfig);
    } else {
      engineRef.current = UIEngine.forGeneral(profile, engineConfig);
    }

    const initialConfig = engineRef.current.getCurrentConfig();
    if (initialConfig) {
      setConfig(initialConfig);
      setBaseConfig(initialConfig);
    }
  }, [profile, engineConfig]);

  // 자폐 특화 조정
  const autismAdjustments = useMemo(() => {
    if (isAutismProfile(profile)) {
      const autismProfile = profile as AutismProfile;
      return {
        sensoryAccommodations: {
          reducedAnimation:
            autismProfile.sensoryProcessing.visual.reactivity === 'hyper',
          mutedColors: autismProfile.sensoryProcessing.visual.reactivity === 'hyper',
          noFlashing: true,
          quietMode: autismProfile.sensoryProcessing.auditory.reactivity === 'hyper',
        },
        predictability: {
          consistentLayout: true,
          transitionWarnings: (autismProfile.routineAdherence?.transitionDifficulty ?? 0) >= 5,
          visualSchedule: autismProfile.diagnosis.supportLevel >= 2,
          progressIndicator: true,
        },
        specialInterests: {
          enabled: autismProfile.specialInterests?.usableForAAC ?? false,
          interests: autismProfile.specialInterests?.topics.map((t) => t.topic) ?? [],
          integratedInUI: true,
        },
      };
    }
    return null;
  }, [profile]);

  // 치매 특화 조정
  const dementiaAdjustments = useMemo(() => {
    if (isDementiaProfile(profile)) {
      const dementiaProfile = profile as DementiaProfile;
      return {
        familiarity: {
          usePersonalPhotos: true,
          familiarFaces:
            dementiaProfile.personalHistory?.familyMembers?.map((m) => ({
              name: m.name,
              relationship: m.relationship,
              recognitionLevel: m.recognitionLevel as 'full' | 'sometimes' | 'familiar' | 'none',
            })) ?? [],
          personalLocations: [],
          lifeStoryIntegration: dementiaProfile.diagnosis.stage !== 'preclinical',
        },
        simplification: {
          minimalNavigation: ['middle', 'late'].includes(dementiaProfile.diagnosis.stage),
          largeText: true,
          reducedChoices: getReducedChoicesForStage(dementiaProfile.diagnosis.stage),
          clearLabeling: true,
        },
        temporalSupport: {
          dayNightAwareness: true,
          mealTimeReminders: dementiaProfile.diagnosis.stage !== 'preclinical',
          routineBasedSuggestions: true,
          showDateTime: true,
        },
        errorPrevention: {
          confirmBeforeAction: ['early', 'middle', 'late'].includes(
            dementiaProfile.diagnosis.stage
          ),
          undoSupport: true,
          noDestructiveActions: true,
        },
      };
    }
    return null;
  }, [profile]);

  // 특수 관심사 요소
  const specialInterestElements = useMemo(() => {
    return engineRef.current?.getSpecialInterestElements() ?? null;
  }, [config]);

  // 회상 요소
  const reminiscenceElements = useMemo(() => {
    return engineRef.current?.getReminiscenceElements() ?? null;
  }, [config]);

  // 상호작용 기록
  const recordInteraction = useCallback(
    (eventData: Omit<UserInteractionEvent, 'timestamp'>) => {
      if (!engineRef.current || !isActive || isPaused) return;

      const event: UserInteractionEvent = {
        ...eventData,
        timestamp: Date.now(),
      };

      const adjustment = engineRef.current.adjustUI(event);

      // 부하 수준 업데이트
      const loadState = engineRef.current.getLoadState();
      setLoadLevel(loadState.currentLoad);

      // 조정 적용
      if (adjustment && adjustment.confidence > 0.5) {
        const newConfig = {
          ...config,
          ...adjustment.changes,
        } as UIConfiguration;

        setConfig(newConfig);
        setAdjustmentHistory((prev) => [...prev, adjustment]);
        onConfigChange?.(newConfig, adjustment);
      }
    },
    [config, isActive, isPaused, onConfigChange]
  );

  // 수동 조정 적용
  const applyAdjustment = useCallback(
    (adjustmentChanges: Partial<UIConfiguration>) => {
      const newConfig = { ...config, ...adjustmentChanges } as UIConfiguration;
      setConfig(newConfig);

      const adjustment: UIAdjustment = {
        type: 'cognitiveLoad',
        changes: adjustmentChanges,
        reason: 'Manual adjustment',
        confidence: 1,
      };

      setAdjustmentHistory((prev) => [...prev, adjustment]);
      onConfigChange?.(newConfig, adjustment);
    },
    [config, onConfigChange]
  );

  // 기본 설정으로 리셋
  const resetConfig = useCallback(() => {
    setConfig(baseConfig);
    setAdjustmentHistory([]);
    onConfigChange?.(baseConfig, null);
  }, [baseConfig, onConfigChange]);

  // 인터페이스 단순화
  const simplify = useCallback(() => {
    const currentItems = config.cognitiveLoad.maxVisibleItems;
    const newItems = Math.max(4, Math.floor(currentItems * 0.7));

    applyAdjustment({
      cognitiveLoad: {
        ...config.cognitiveLoad,
        maxVisibleItems: newItems,
        simplificationLevel: Math.min(5, config.cognitiveLoad.simplificationLevel + 1),
      },
      grid: {
        ...config.grid,
        columns: Math.max(2, config.grid.columns - 1),
        rows: Math.max(2, config.grid.rows - 1),
      },
    });
  }, [config, applyAdjustment]);

  // 기본 설정 복원
  const restore = useCallback(() => {
    if (adjustmentHistory.length === 0) return;

    // 마지막 조정 전 상태로 복원
    const previousConfig = adjustmentHistory.length > 1
      ? { ...baseConfig } // 히스토리가 있으면 기본으로
      : baseConfig;

    setConfig(previousConfig);
    setAdjustmentHistory((prev) => prev.slice(0, -1));
    onConfigChange?.(previousConfig, null);
  }, [adjustmentHistory, baseConfig, onConfigChange]);

  // 엔진 제어
  const start = useCallback(() => {
    setIsActive(true);
    setIsPaused(false);
  }, []);

  const stop = useCallback(() => {
    setIsActive(false);
    setIsPaused(false);
  }, []);

  const pause = useCallback(() => {
    setIsPaused(true);
  }, []);

  const resume = useCallback(() => {
    setIsPaused(false);
  }, []);

  return {
    config,
    baseConfig,
    autismAdjustments,
    dementiaAdjustments,
    specialInterestElements,
    reminiscenceElements,
    adjustmentHistory,
    isActive,
    loadLevel,
    recordInteraction,
    applyAdjustment,
    resetConfig,
    simplify,
    restore,
    start,
    stop,
    pause,
    resume,
  };
}

// ============================================================================
// Helper Functions
// ============================================================================

function getReducedChoicesForStage(stage: DementiaProfile['diagnosis']['stage']): number {
  switch (stage) {
    case 'preclinical':
      return 20;
    case 'mci':
      return 16;
    case 'early':
      return 12;
    case 'middle':
      return 6;
    case 'late':
      return 4;
    default:
      return 12;
  }
}

export default useAdaptiveUI;
