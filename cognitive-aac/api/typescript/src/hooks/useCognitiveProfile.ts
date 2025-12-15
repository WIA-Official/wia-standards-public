/**
 * WIA Cognitive AAC - useCognitiveProfile Hook
 * 인지 프로파일 상태 관리 훅
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import { useState, useCallback, useMemo, useEffect } from 'react';
import {
  CognitiveProfile,
  AutismProfile,
  DementiaProfile,
  CognitiveLevel,
} from '../types';
import { isAutismProfile, isDementiaProfile } from '../engine/ProfileAdapter';

// ============================================================================
// Types
// ============================================================================

export interface UseCognitiveProfileOptions {
  initialProfile?: CognitiveProfile | null;
  onProfileChange?: (profile: CognitiveProfile) => void;
  persistKey?: string;
}

export interface UseCognitiveProfileReturn {
  // 프로파일 상태
  profile: CognitiveProfile | null;
  isAutism: boolean;
  isDementia: boolean;
  cognitiveLevel: CognitiveLevel;

  // 프로파일 관리
  setProfile: (profile: CognitiveProfile) => void;
  updateProfile: (updates: Partial<CognitiveProfile>) => void;
  clearProfile: () => void;

  // 특화 프로파일 접근
  getAutismProfile: () => AutismProfile | null;
  getDementiaProfile: () => DementiaProfile | null;

  // 유틸리티
  getStrengths: () => string[];
  getChallenges: () => string[];
  getDomainLevel: (domain: keyof CognitiveProfile['domains']) => CognitiveLevel;
}

// ============================================================================
// Storage Helper
// ============================================================================

const storage = {
  get: (key: string): CognitiveProfile | null => {
    if (typeof window === 'undefined') return null;
    try {
      const stored = localStorage.getItem(key);
      return stored ? JSON.parse(stored) : null;
    } catch {
      return null;
    }
  },
  set: (key: string, value: CognitiveProfile): void => {
    if (typeof window === 'undefined') return;
    try {
      localStorage.setItem(key, JSON.stringify(value));
    } catch {
      // Storage full or unavailable
    }
  },
  remove: (key: string): void => {
    if (typeof window === 'undefined') return;
    try {
      localStorage.removeItem(key);
    } catch {
      // Storage unavailable
    }
  },
};

// ============================================================================
// Hook Implementation
// ============================================================================

export function useCognitiveProfile(
  options: UseCognitiveProfileOptions = {}
): UseCognitiveProfileReturn {
  const { initialProfile = null, onProfileChange, persistKey } = options;

  // 초기값 로드
  const getInitialProfile = (): CognitiveProfile | null => {
    if (persistKey) {
      const stored = storage.get(persistKey);
      if (stored) return stored;
    }
    return initialProfile;
  };

  const [profile, setProfileState] = useState<CognitiveProfile | null>(
    getInitialProfile
  );

  // 프로파일 유형 확인
  const isAutism = useMemo(
    () => profile !== null && isAutismProfile(profile),
    [profile]
  );

  const isDementia = useMemo(
    () => profile !== null && isDementiaProfile(profile),
    [profile]
  );

  // 전체 인지 수준
  const cognitiveLevel = useMemo(
    () => profile?.summary.overallLevel ?? CognitiveLevel.TYPICAL,
    [profile]
  );

  // 프로파일 설정
  const setProfile = useCallback(
    (newProfile: CognitiveProfile) => {
      setProfileState(newProfile);
      onProfileChange?.(newProfile);

      if (persistKey) {
        storage.set(persistKey, newProfile);
      }
    },
    [onProfileChange, persistKey]
  );

  // 프로파일 부분 업데이트
  const updateProfile = useCallback(
    (updates: Partial<CognitiveProfile>) => {
      if (!profile) return;

      const updatedProfile: CognitiveProfile = {
        ...profile,
        ...updates,
        updatedAt: new Date().toISOString(),
      };

      setProfile(updatedProfile);
    },
    [profile, setProfile]
  );

  // 프로파일 초기화
  const clearProfile = useCallback(() => {
    setProfileState(null);

    if (persistKey) {
      storage.remove(persistKey);
    }
  }, [persistKey]);

  // 자폐 프로파일 접근
  const getAutismProfile = useCallback((): AutismProfile | null => {
    if (profile && isAutismProfile(profile)) {
      return profile as AutismProfile;
    }
    return null;
  }, [profile]);

  // 치매 프로파일 접근
  const getDementiaProfile = useCallback((): DementiaProfile | null => {
    if (profile && isDementiaProfile(profile)) {
      return profile as DementiaProfile;
    }
    return null;
  }, [profile]);

  // 강점 목록
  const getStrengths = useCallback((): string[] => {
    return profile?.summary.strengths ?? [];
  }, [profile]);

  // 도전 영역 목록
  const getChallenges = useCallback((): string[] => {
    return profile?.summary.challenges ?? [];
  }, [profile]);

  // 특정 도메인 수준 가져오기
  const getDomainLevel = useCallback(
    (domain: keyof CognitiveProfile['domains']): CognitiveLevel => {
      if (!profile) return CognitiveLevel.TYPICAL;

      const domainData = profile.domains[domain];

      // 각 도메인의 대표 레벨 반환
      switch (domain) {
        case 'memory':
          return domainData.workingMemory.level;
        case 'attention':
          return (domainData as typeof profile.domains.attention).sustained.level;
        case 'language':
          return (domainData as typeof profile.domains.language).receptive.level;
        case 'executive':
          return (domainData as typeof profile.domains.executive).planning.level;
        case 'visualSpatial':
          return (domainData as typeof profile.domains.visualSpatial).visualPerception.level;
        case 'socialCognition':
          return (domainData as typeof profile.domains.socialCognition).emotionRecognition.level;
        default:
          return CognitiveLevel.TYPICAL;
      }
    },
    [profile]
  );

  // persistKey 변경 시 프로파일 로드
  useEffect(() => {
    if (persistKey) {
      const stored = storage.get(persistKey);
      if (stored) {
        setProfileState(stored);
      }
    }
  }, [persistKey]);

  return {
    profile,
    isAutism,
    isDementia,
    cognitiveLevel,
    setProfile,
    updateProfile,
    clearProfile,
    getAutismProfile,
    getDementiaProfile,
    getStrengths,
    getChallenges,
    getDomainLevel,
  };
}

export default useCognitiveProfile;
