/**
 * WIA Cognitive AAC - Adaptive Button Component
 * 인지 프로파일 기반 적응형 버튼 컴포넌트
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import React, { useCallback, useState, useRef, useEffect } from 'react';
import { InteractionConfig, CellSize, ThemeConfig } from '../types';
import { CELL_SIZE_PX, MIN_TOUCH_TARGET } from '../engine/Presets';

// ============================================================================
// Types
// ============================================================================

export type ButtonVariant = 'primary' | 'secondary' | 'danger' | 'success' | 'neutral';
export type ButtonSize = 'small' | 'medium' | 'large';

export interface AdaptiveButtonProps {
  label: string;
  icon?: React.ReactNode;
  variant?: ButtonVariant;
  size?: ButtonSize;
  interactionConfig?: Partial<InteractionConfig>;
  theme?: Partial<ThemeConfig>;
  disabled?: boolean;
  loading?: boolean;
  fullWidth?: boolean;
  onClick?: () => void;
  onDwellComplete?: () => void;
  className?: string;
  style?: React.CSSProperties;
  'aria-label'?: string;
}

// ============================================================================
// Default Configurations
// ============================================================================

const DEFAULT_INTERACTION: InteractionConfig = {
  dwellTime: 0,
  confirmationRequired: false,
  audioFeedback: false,
  hapticFeedback: false,
  highlightOnHover: true,
};

const SIZE_MAP: Record<ButtonSize, { height: number; fontSize: number; padding: string }> = {
  small: { height: 36, fontSize: 14, padding: '8px 16px' },
  medium: { height: 48, fontSize: 16, padding: '12px 24px' },
  large: { height: 64, fontSize: 20, padding: '16px 32px' },
};

const VARIANT_COLORS: Record<ButtonVariant, { bg: string; text: string; hover: string }> = {
  primary: { bg: '#007AFF', text: '#FFFFFF', hover: '#0056CC' },
  secondary: { bg: '#E0E0E0', text: '#333333', hover: '#CCCCCC' },
  danger: { bg: '#FF3B30', text: '#FFFFFF', hover: '#CC2F26' },
  success: { bg: '#34C759', text: '#FFFFFF', hover: '#28A745' },
  neutral: { bg: '#F5F5F5', text: '#333333', hover: '#E8E8E8' },
};

// ============================================================================
// Component
// ============================================================================

export const AdaptiveButton: React.FC<AdaptiveButtonProps> = ({
  label,
  icon,
  variant = 'primary',
  size = 'medium',
  interactionConfig = {},
  theme = {},
  disabled = false,
  loading = false,
  fullWidth = false,
  onClick,
  onDwellComplete,
  className = '',
  style = {},
  'aria-label': ariaLabel,
}) => {
  const [isHovered, setIsHovered] = useState(false);
  const [isPressed, setIsPressed] = useState(false);
  const [isDwelling, setIsDwelling] = useState(false);
  const [dwellProgress, setDwellProgress] = useState(0);

  const dwellTimerRef = useRef<NodeJS.Timeout | null>(null);
  const dwellStartRef = useRef<number>(0);
  const animationFrameRef = useRef<number>(0);

  const config = { ...DEFAULT_INTERACTION, ...interactionConfig };
  const sizeConfig = SIZE_MAP[size];
  const colorConfig = VARIANT_COLORS[variant];

  // Dwell 진행 업데이트
  const updateDwellProgress = useCallback(() => {
    if (!isDwelling || config.dwellTime <= 0) return;

    const elapsed = Date.now() - dwellStartRef.current;
    const progress = Math.min(elapsed / config.dwellTime, 1);
    setDwellProgress(progress);

    if (progress < 1) {
      animationFrameRef.current = requestAnimationFrame(updateDwellProgress);
    }
  }, [isDwelling, config.dwellTime]);

  // Dwell 시작
  const startDwell = useCallback(() => {
    if (config.dwellTime <= 0 || disabled || loading) return;

    setIsDwelling(true);
    dwellStartRef.current = Date.now();
    setDwellProgress(0);

    animationFrameRef.current = requestAnimationFrame(updateDwellProgress);

    dwellTimerRef.current = setTimeout(() => {
      setIsDwelling(false);
      setDwellProgress(0);
      onDwellComplete?.();
      onClick?.();

      if (config.audioFeedback) {
        playClickSound();
      }
      if (config.hapticFeedback && 'vibrate' in navigator) {
        navigator.vibrate(50);
      }
    }, config.dwellTime);
  }, [config, disabled, loading, onDwellComplete, onClick, updateDwellProgress]);

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
    if (disabled || loading) return;

    onClick?.();

    if (config.audioFeedback) {
      playClickSound();
    }
    if (config.hapticFeedback && 'vibrate' in navigator) {
      navigator.vibrate(50);
    }
  }, [disabled, loading, onClick, config]);

  // 정리
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
  const buttonStyle: React.CSSProperties = {
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '8px',
    height: sizeConfig.height,
    minWidth: MIN_TOUCH_TARGET.medium,
    padding: sizeConfig.padding,
    fontSize: sizeConfig.fontSize,
    fontWeight: 600,
    color: disabled ? '#999999' : colorConfig.text,
    backgroundColor: disabled
      ? '#E0E0E0'
      : isPressed
        ? colorConfig.hover
        : isHovered
          ? colorConfig.hover
          : colorConfig.bg,
    border: 'none',
    borderRadius: theme.colorScheme === 'high-contrast' ? '4px' : '8px',
    cursor: disabled || loading ? 'not-allowed' : 'pointer',
    opacity: disabled ? 0.6 : 1,
    transition: theme.reducedMotion ? 'none' : 'all 0.15s ease',
    position: 'relative',
    overflow: 'hidden',
    width: fullWidth ? '100%' : 'auto',
    userSelect: 'none',
    ...style,
  };

  const dwellIndicatorStyle: React.CSSProperties = {
    position: 'absolute',
    bottom: 0,
    left: 0,
    height: '4px',
    width: `${dwellProgress * 100}%`,
    backgroundColor: 'rgba(255, 255, 255, 0.5)',
    transition: 'width 0.05s linear',
  };

  const loadingSpinnerStyle: React.CSSProperties = {
    width: sizeConfig.fontSize,
    height: sizeConfig.fontSize,
    border: `2px solid ${colorConfig.text}`,
    borderTopColor: 'transparent',
    borderRadius: '50%',
    animation: theme.reducedMotion ? 'none' : 'spin 1s linear infinite',
  };

  return (
    <button
      className={`adaptive-button ${className}`}
      style={buttonStyle}
      onClick={handleClick}
      onMouseEnter={() => {
        setIsHovered(true);
        if (config.highlightOnHover && config.dwellTime > 0) {
          startDwell();
        }
      }}
      onMouseLeave={() => {
        setIsHovered(false);
        cancelDwell();
      }}
      onMouseDown={() => setIsPressed(true)}
      onMouseUp={() => setIsPressed(false)}
      onTouchStart={() => {
        setIsPressed(true);
        if (config.dwellTime > 0) {
          startDwell();
        }
      }}
      onTouchEnd={() => {
        setIsPressed(false);
        cancelDwell();
      }}
      disabled={disabled || loading}
      aria-label={ariaLabel || label}
      aria-disabled={disabled}
      aria-busy={loading}
    >
      {/* Dwell 진행 표시 */}
      {isDwelling && config.dwellTime > 0 && <div style={dwellIndicatorStyle} />}

      {/* 로딩 스피너 */}
      {loading && <div style={loadingSpinnerStyle} />}

      {/* 아이콘 */}
      {icon && !loading && icon}

      {/* 레이블 */}
      {!loading && label}
    </button>
  );
};

// ============================================================================
// Helper Functions
// ============================================================================

function playClickSound(): void {
  if ('AudioContext' in window || 'webkitAudioContext' in window) {
    try {
      const audioContext = new (window.AudioContext || (window as any).webkitAudioContext)();
      const oscillator = audioContext.createOscillator();
      const gainNode = audioContext.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(audioContext.destination);

      oscillator.frequency.value = 880;
      gainNode.gain.value = 0.1;

      oscillator.start();
      oscillator.stop(audioContext.currentTime + 0.05);
    } catch (e) {
      // 오디오 재생 실패 시 무시
    }
  }
}

// CSS 애니메이션 (로딩 스피너)
if (typeof document !== 'undefined') {
  const styleSheet = document.createElement('style');
  styleSheet.textContent = `
    @keyframes spin {
      from { transform: rotate(0deg); }
      to { transform: rotate(360deg); }
    }
  `;
  document.head.appendChild(styleSheet);
}

export default AdaptiveButton;
