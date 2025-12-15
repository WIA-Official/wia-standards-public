/**
 * WIA Cognitive AAC - Adaptive Symbol Component
 * 인지 프로파일 기반 적응형 심볼 컴포넌트
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import React, { useCallback, useState, useRef, useEffect } from 'react';
import { SymbolItem, SymbolConfig, InteractionConfig, CellSize } from '../types';
import { CELL_SIZE_PX, MIN_TOUCH_TARGET } from '../engine/Presets';

// ============================================================================
// Types
// ============================================================================

export interface AdaptiveSymbolProps {
  item: SymbolItem;
  symbolConfig: SymbolConfig;
  interactionConfig: InteractionConfig;
  cellSize: CellSize;
  isSelected?: boolean;
  isHighlighted?: boolean;
  onSelect?: (item: SymbolItem) => void;
  onHover?: (item: SymbolItem | null) => void;
  onDwellComplete?: (item: SymbolItem) => void;
  className?: string;
  style?: React.CSSProperties;
}

// ============================================================================
// Component
// ============================================================================

export const AdaptiveSymbol: React.FC<AdaptiveSymbolProps> = ({
  item,
  symbolConfig,
  interactionConfig,
  cellSize,
  isSelected = false,
  isHighlighted = false,
  onSelect,
  onHover,
  onDwellComplete,
  className = '',
  style = {},
}) => {
  const [isDwelling, setIsDwelling] = useState(false);
  const [dwellProgress, setDwellProgress] = useState(0);
  const dwellTimerRef = useRef<NodeJS.Timeout | null>(null);
  const dwellStartRef = useRef<number>(0);
  const animationFrameRef = useRef<number>(0);

  // Dwell 애니메이션 업데이트
  const updateDwellProgress = useCallback(() => {
    if (!isDwelling || interactionConfig.dwellTime <= 0) return;

    const elapsed = Date.now() - dwellStartRef.current;
    const progress = Math.min(elapsed / interactionConfig.dwellTime, 1);
    setDwellProgress(progress);

    if (progress < 1) {
      animationFrameRef.current = requestAnimationFrame(updateDwellProgress);
    }
  }, [isDwelling, interactionConfig.dwellTime]);

  // Dwell 시작
  const startDwell = useCallback(() => {
    if (interactionConfig.dwellTime <= 0) return;

    setIsDwelling(true);
    dwellStartRef.current = Date.now();
    setDwellProgress(0);

    animationFrameRef.current = requestAnimationFrame(updateDwellProgress);

    dwellTimerRef.current = setTimeout(() => {
      setIsDwelling(false);
      setDwellProgress(0);
      onDwellComplete?.(item);

      if (interactionConfig.audioFeedback) {
        playAudioFeedback('dwell_complete');
      }
      if (interactionConfig.hapticFeedback) {
        triggerHapticFeedback();
      }
    }, interactionConfig.dwellTime);
  }, [interactionConfig, item, onDwellComplete, updateDwellProgress]);

  // Dwell 취소
  const cancelDwell = useCallback(() => {
    if (dwellTimerRef.current) {
      clearTimeout(dwellTimerRef.current);
      dwellTimerRef.current = null;
    }
    if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current);
    }
    setIsDwelling(false);
    setDwellProgress(0);
  }, []);

  // 클릭 핸들러
  const handleClick = useCallback(() => {
    onSelect?.(item);

    if (interactionConfig.audioFeedback) {
      playAudioFeedback('select');
    }
    if (interactionConfig.hapticFeedback) {
      triggerHapticFeedback();
    }
  }, [item, onSelect, interactionConfig]);

  // 호버 핸들러
  const handleMouseEnter = useCallback(() => {
    onHover?.(item);
    if (interactionConfig.dwellTime > 0) {
      startDwell();
    }
  }, [item, onHover, interactionConfig.dwellTime, startDwell]);

  const handleMouseLeave = useCallback(() => {
    onHover?.(null);
    cancelDwell();
  }, [onHover, cancelDwell]);

  // 터치 핸들러
  const handleTouchStart = useCallback(() => {
    if (interactionConfig.dwellTime > 0) {
      startDwell();
    }
  }, [interactionConfig.dwellTime, startDwell]);

  const handleTouchEnd = useCallback(() => {
    cancelDwell();
    handleClick();
  }, [cancelDwell, handleClick]);

  // 컴포넌트 언마운트 시 정리
  useEffect(() => {
    return () => {
      if (dwellTimerRef.current) {
        clearTimeout(dwellTimerRef.current);
      }
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, []);

  // 스타일 계산
  const sizePx = CELL_SIZE_PX[cellSize];
  const minTouch = MIN_TOUCH_TARGET[cellSize];

  const containerStyle: React.CSSProperties = {
    width: sizePx,
    height: sizePx,
    minWidth: minTouch,
    minHeight: minTouch,
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: symbolConfig.labelPosition === 'none' ? 'center' : 'flex-start',
    padding: '8px',
    borderRadius: symbolConfig.borderRadius || 8,
    border: `2px solid ${isSelected ? '#007AFF' : isHighlighted ? '#FFD700' : 'transparent'}`,
    backgroundColor: isSelected ? '#E3F2FD' : isHighlighted ? '#FFFDE7' : '#FFFFFF',
    boxShadow: isSelected ? '0 2px 8px rgba(0, 122, 255, 0.3)' : '0 1px 3px rgba(0,0,0,0.1)',
    cursor: 'pointer',
    transition: 'all 0.15s ease',
    position: 'relative',
    overflow: 'hidden',
    userSelect: 'none',
    ...style,
  };

  const imageStyle: React.CSSProperties = {
    width: symbolConfig.labelPosition === 'none' ? '80%' : '70%',
    height: symbolConfig.labelPosition === 'none' ? '80%' : '60%',
    objectFit: 'contain',
    filter: symbolConfig.highContrast ? 'contrast(1.2)' : 'none',
  };

  const labelStyle: React.CSSProperties = {
    fontSize: symbolConfig.fontSize,
    fontWeight: symbolConfig.highContrast ? 600 : 400,
    color: symbolConfig.highContrast ? '#000000' : '#333333',
    textAlign: 'center',
    marginTop: symbolConfig.labelPosition === 'below' ? '4px' : '0',
    marginBottom: symbolConfig.labelPosition === 'above' ? '4px' : '0',
    maxWidth: '100%',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
  };

  const dwellIndicatorStyle: React.CSSProperties = {
    position: 'absolute',
    bottom: 0,
    left: 0,
    height: '4px',
    width: `${dwellProgress * 100}%`,
    backgroundColor: '#007AFF',
    transition: 'width 0.05s linear',
  };

  return (
    <div
      className={`adaptive-symbol ${className}`}
      style={containerStyle}
      onClick={handleClick}
      onMouseEnter={interactionConfig.highlightOnHover ? handleMouseEnter : undefined}
      onMouseLeave={interactionConfig.highlightOnHover ? handleMouseLeave : undefined}
      onTouchStart={handleTouchStart}
      onTouchEnd={handleTouchEnd}
      role="button"
      tabIndex={0}
      aria-label={item.label}
      aria-pressed={isSelected}
    >
      {/* Dwell 진행 표시 */}
      {isDwelling && interactionConfig.dwellTime > 0 && (
        <div style={dwellIndicatorStyle} />
      )}

      {/* 레이블 (상단) */}
      {symbolConfig.labelPosition === 'above' && (
        <span style={labelStyle}>{item.label}</span>
      )}

      {/* 이미지 */}
      {item.imageUrl && (
        <img
          src={item.imageUrl}
          alt={item.label}
          style={imageStyle}
          draggable={false}
        />
      )}

      {/* 텍스트 전용 심볼 */}
      {!item.imageUrl && symbolConfig.type === 'text' && (
        <span style={{ ...labelStyle, fontSize: symbolConfig.fontSize * 1.5 }}>
          {item.label}
        </span>
      )}

      {/* 레이블 (하단) */}
      {symbolConfig.labelPosition === 'below' && (
        <span style={labelStyle}>{item.label}</span>
      )}
    </div>
  );
};

// ============================================================================
// Helper Functions
// ============================================================================

function playAudioFeedback(type: 'select' | 'dwell_complete' | 'error'): void {
  // Web Audio API 또는 HTMLAudioElement로 피드백 재생
  // 실제 구현에서는 오디오 파일 로드 필요
  if ('AudioContext' in window || 'webkitAudioContext' in window) {
    try {
      const audioContext = new (window.AudioContext || (window as any).webkitAudioContext)();
      const oscillator = audioContext.createOscillator();
      const gainNode = audioContext.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(audioContext.destination);

      // 피드백 유형별 소리 설정
      switch (type) {
        case 'select':
          oscillator.frequency.value = 880;
          gainNode.gain.value = 0.1;
          break;
        case 'dwell_complete':
          oscillator.frequency.value = 660;
          gainNode.gain.value = 0.15;
          break;
        case 'error':
          oscillator.frequency.value = 220;
          gainNode.gain.value = 0.1;
          break;
      }

      oscillator.start();
      oscillator.stop(audioContext.currentTime + 0.1);
    } catch (e) {
      // 오디오 재생 실패 시 무시
    }
  }
}

function triggerHapticFeedback(): void {
  if ('vibrate' in navigator) {
    navigator.vibrate(50);
  }
}

export default AdaptiveSymbol;
