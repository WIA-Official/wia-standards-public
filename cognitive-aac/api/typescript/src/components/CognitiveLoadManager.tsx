/**
 * WIA Cognitive AAC - Cognitive Load Manager Component
 * 인지 부하를 모니터링하고 UI를 자동 조정하는 관리 컴포넌트
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import React, {
  createContext,
  useContext,
  useCallback,
  useEffect,
  useRef,
  useState,
  ReactNode,
} from 'react';

import {
  CognitiveProfile,
  UIConfiguration,
  UIAdjustment,
  UserInteractionEvent,
  CognitiveLevel,
} from '../types';
import { UIEngine, EngineConfig } from '../engine/UIEngine';
import { COGNITIVE_LEVEL_PRESETS } from '../engine/Presets';

// ============================================================================
// Types
// ============================================================================

export interface CognitiveLoadState {
  currentLoad: number;
  status: 'optimal' | 'elevated' | 'high' | 'overloaded';
  errorRate: number;
  averageResponseTime: number;
  sessionDuration: number;
  adjustmentsMade: number;
}

export interface CognitiveLoadManagerContextValue {
  // 상태
  loadState: CognitiveLoadState;
  currentConfig: UIConfiguration;
  isMonitoring: boolean;

  // 액션
  recordEvent: (event: Omit<UserInteractionEvent, 'timestamp'>) => void;
  recordSelection: (itemId: string, duration: number, successful: boolean) => void;
  recordError: (errorType?: string) => void;
  recordHesitation: () => void;
  recordTimeout: () => void;

  // 수동 조정
  simplifyInterface: () => void;
  resetToDefault: () => void;
  setConfig: (config: Partial<UIConfiguration>) => void;

  // 모니터링 제어
  startMonitoring: () => void;
  stopMonitoring: () => void;
  pauseMonitoring: () => void;
  resumeMonitoring: () => void;
}

interface CognitiveLoadManagerProps {
  profile: CognitiveProfile;
  engineConfig?: Partial<EngineConfig>;
  onConfigChange?: (config: UIConfiguration, adjustment: UIAdjustment | null) => void;
  onLoadStateChange?: (state: CognitiveLoadState) => void;
  onOverload?: () => void;
  children: ReactNode;
}

// ============================================================================
// Context
// ============================================================================

const CognitiveLoadManagerContext = createContext<CognitiveLoadManagerContextValue | null>(null);

// ============================================================================
// Hook
// ============================================================================

export function useCognitiveLoadManager(): CognitiveLoadManagerContextValue {
  const context = useContext(CognitiveLoadManagerContext);
  if (!context) {
    throw new Error(
      'useCognitiveLoadManager must be used within a CognitiveLoadManagerProvider'
    );
  }
  return context;
}

// ============================================================================
// Provider Component
// ============================================================================

export const CognitiveLoadManagerProvider: React.FC<CognitiveLoadManagerProps> = ({
  profile,
  engineConfig,
  onConfigChange,
  onLoadStateChange,
  onOverload,
  children,
}) => {
  // Engine 인스턴스
  const engineRef = useRef<UIEngine | null>(null);

  // 상태
  const [currentConfig, setCurrentConfig] = useState<UIConfiguration>(
    COGNITIVE_LEVEL_PRESETS[profile.summary.overallLevel]
  );
  const [loadState, setLoadState] = useState<CognitiveLoadState>({
    currentLoad: 0,
    status: 'optimal',
    errorRate: 0,
    averageResponseTime: 0,
    sessionDuration: 0,
    adjustmentsMade: 0,
  });
  const [isMonitoring, setIsMonitoring] = useState(false);
  const [isPaused, setIsPaused] = useState(false);

  // 이벤트 추적
  const eventCountRef = useRef({ total: 0, errors: 0, selections: 0 });
  const selectionTimesRef = useRef<number[]>([]);
  const sessionStartRef = useRef<number>(0);

  // Engine 초기화
  useEffect(() => {
    engineRef.current = new UIEngine(engineConfig);
    const initialConfig = engineRef.current.generateUI(profile);
    setCurrentConfig(initialConfig);
  }, [profile, engineConfig]);

  // 부하 상태 계산
  const calculateLoadStatus = useCallback(
    (load: number): CognitiveLoadState['status'] => {
      if (load < 0.3) return 'optimal';
      if (load < 0.5) return 'elevated';
      if (load < 0.7) return 'high';
      return 'overloaded';
    },
    []
  );

  // 이벤트 기록 및 조정 처리
  const processEvent = useCallback(
    (event: UserInteractionEvent) => {
      if (!engineRef.current || isPaused) return;

      // 이벤트 카운트 업데이트
      eventCountRef.current.total++;
      if (event.type === 'error') eventCountRef.current.errors++;
      if (event.type === 'selection') eventCountRef.current.selections++;

      // 엔진에 이벤트 전달
      const adjustment = engineRef.current.adjustUI(event);

      // 부하 상태 업데이트
      const engineLoadState = engineRef.current.getLoadState();
      const newLoadState: CognitiveLoadState = {
        currentLoad: engineLoadState.currentLoad,
        status: calculateLoadStatus(engineLoadState.currentLoad),
        errorRate:
          eventCountRef.current.total > 0
            ? eventCountRef.current.errors / eventCountRef.current.total
            : 0,
        averageResponseTime: engineLoadState.averageSelectionTime,
        sessionDuration: Date.now() - sessionStartRef.current,
        adjustmentsMade: loadState.adjustmentsMade + (adjustment ? 1 : 0),
      };

      setLoadState(newLoadState);
      onLoadStateChange?.(newLoadState);

      // 조정이 필요한 경우 설정 업데이트
      if (adjustment && adjustment.confidence > 0.5) {
        const newConfig = {
          ...currentConfig,
          ...adjustment.changes,
        } as UIConfiguration;

        setCurrentConfig(newConfig);
        onConfigChange?.(newConfig, adjustment);
      }

      // 과부하 알림
      if (newLoadState.status === 'overloaded') {
        onOverload?.();
      }
    },
    [
      currentConfig,
      isPaused,
      loadState.adjustmentsMade,
      calculateLoadStatus,
      onConfigChange,
      onLoadStateChange,
      onOverload,
    ]
  );

  // 이벤트 기록 함수들
  const recordEvent = useCallback(
    (eventData: Omit<UserInteractionEvent, 'timestamp'>) => {
      if (!isMonitoring) return;

      const event: UserInteractionEvent = {
        ...eventData,
        timestamp: Date.now(),
      };
      processEvent(event);
    },
    [isMonitoring, processEvent]
  );

  const recordSelection = useCallback(
    (itemId: string, duration: number, successful: boolean) => {
      selectionTimesRef.current.push(duration);
      recordEvent({
        type: 'selection',
        targetId: itemId,
        duration,
        successful,
      });
    },
    [recordEvent]
  );

  const recordError = useCallback(
    (errorType?: string) => {
      recordEvent({
        type: 'error',
        metadata: errorType ? { errorType } : undefined,
      });
    },
    [recordEvent]
  );

  const recordHesitation = useCallback(() => {
    recordEvent({ type: 'hesitation' });
  }, [recordEvent]);

  const recordTimeout = useCallback(() => {
    recordEvent({ type: 'timeout' });
  }, [recordEvent]);

  // 수동 조정 함수들
  const simplifyInterface = useCallback(() => {
    const currentLevel = profile.summary.overallLevel;
    const simplerLevel = Math.max(CognitiveLevel.PROFOUND, currentLevel - 1) as CognitiveLevel;
    const simplerConfig = COGNITIVE_LEVEL_PRESETS[simplerLevel];

    setCurrentConfig(simplerConfig);
    onConfigChange?.(simplerConfig, {
      type: 'cognitiveLoad',
      changes: simplerConfig,
      reason: 'Manual simplification requested',
      confidence: 1,
    });
  }, [profile.summary.overallLevel, onConfigChange]);

  const resetToDefault = useCallback(() => {
    if (engineRef.current) {
      const defaultConfig = engineRef.current.generateUI(profile);
      setCurrentConfig(defaultConfig);
      onConfigChange?.(defaultConfig, null);
    }
  }, [profile, onConfigChange]);

  const setConfigManual = useCallback(
    (configChanges: Partial<UIConfiguration>) => {
      const newConfig = { ...currentConfig, ...configChanges } as UIConfiguration;
      setCurrentConfig(newConfig);
      onConfigChange?.(newConfig, null);
    },
    [currentConfig, onConfigChange]
  );

  // 모니터링 제어
  const startMonitoring = useCallback(() => {
    sessionStartRef.current = Date.now();
    eventCountRef.current = { total: 0, errors: 0, selections: 0 };
    selectionTimesRef.current = [];
    setIsMonitoring(true);
    setIsPaused(false);
    setLoadState({
      currentLoad: 0,
      status: 'optimal',
      errorRate: 0,
      averageResponseTime: 0,
      sessionDuration: 0,
      adjustmentsMade: 0,
    });
  }, []);

  const stopMonitoring = useCallback(() => {
    setIsMonitoring(false);
    setIsPaused(false);
  }, []);

  const pauseMonitoring = useCallback(() => {
    setIsPaused(true);
  }, []);

  const resumeMonitoring = useCallback(() => {
    setIsPaused(false);
  }, []);

  // Context 값
  const contextValue: CognitiveLoadManagerContextValue = {
    loadState,
    currentConfig,
    isMonitoring,
    recordEvent,
    recordSelection,
    recordError,
    recordHesitation,
    recordTimeout,
    simplifyInterface,
    resetToDefault,
    setConfig: setConfigManual,
    startMonitoring,
    stopMonitoring,
    pauseMonitoring,
    resumeMonitoring,
  };

  return (
    <CognitiveLoadManagerContext.Provider value={contextValue}>
      {children}
    </CognitiveLoadManagerContext.Provider>
  );
};

// ============================================================================
// Visual Load Indicator Component
// ============================================================================

export interface LoadIndicatorProps {
  showLabel?: boolean;
  size?: 'small' | 'medium' | 'large';
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';
}

export const CognitiveLoadIndicator: React.FC<LoadIndicatorProps> = ({
  showLabel = true,
  size = 'medium',
  position = 'top-right',
}) => {
  const { loadState } = useCognitiveLoadManager();

  const sizeMap = {
    small: { width: 60, height: 8, fontSize: 10 },
    medium: { width: 100, height: 12, fontSize: 12 },
    large: { width: 150, height: 16, fontSize: 14 },
  };

  const colorMap: Record<CognitiveLoadState['status'], string> = {
    optimal: '#34C759',
    elevated: '#FFD60A',
    high: '#FF9500',
    overloaded: '#FF3B30',
  };

  const positionMap = {
    'top-left': { top: 16, left: 16 },
    'top-right': { top: 16, right: 16 },
    'bottom-left': { bottom: 16, left: 16 },
    'bottom-right': { bottom: 16, right: 16 },
  };

  const config = sizeMap[size];

  const containerStyle: React.CSSProperties = {
    position: 'fixed',
    ...positionMap[position],
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'flex-end',
    gap: '4px',
    zIndex: 9999,
  };

  const barContainerStyle: React.CSSProperties = {
    width: config.width,
    height: config.height,
    backgroundColor: '#E0E0E0',
    borderRadius: config.height / 2,
    overflow: 'hidden',
  };

  const barStyle: React.CSSProperties = {
    width: `${loadState.currentLoad * 100}%`,
    height: '100%',
    backgroundColor: colorMap[loadState.status],
    transition: 'width 0.3s ease, background-color 0.3s ease',
  };

  const labelStyle: React.CSSProperties = {
    fontSize: config.fontSize,
    color: '#666666',
    fontWeight: 500,
  };

  return (
    <div style={containerStyle}>
      {showLabel && (
        <span style={labelStyle}>
          {loadState.status.charAt(0).toUpperCase() + loadState.status.slice(1)}
        </span>
      )}
      <div style={barContainerStyle}>
        <div style={barStyle} />
      </div>
    </div>
  );
};

// ============================================================================
// Exports
// ============================================================================

export default CognitiveLoadManagerProvider;
