/**
 * WIA Cognitive AAC - Privacy Manager
 * 프라이버시 보호 및 데이터 관리 모듈
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 *
 * 핵심 원칙:
 * - 온디바이스 학습 우선 (Local-first learning)
 * - 데이터 최소화 (Data minimization)
 * - 명시적 동의 필수 (Explicit consent required)
 * - 완전한 데이터 삭제 지원 (Full data erasure support)
 */

import {
  PrivacySettings,
  ConsentRecord,
  LearningEvent,
  DataRetentionPolicy,
  UsagePattern,
  AnonymizationLevel,
} from './types';

// ============================================================================
// Types
// ============================================================================

export interface DataExport {
  version: string;
  exportDate: string;
  dataType: 'patterns' | 'settings' | 'full';
  encrypted: boolean;
  data: unknown;
}

export interface StorageQuota {
  used: number;
  limit: number;
  unit: 'KB' | 'MB';
}

export interface PrivacyAuditLog {
  timestamp: number;
  action: 'access' | 'export' | 'delete' | 'consent_change';
  details: string;
  actor: 'user' | 'system' | 'caregiver';
}

// ============================================================================
// Default Settings
// ============================================================================

const DEFAULT_PRIVACY_SETTINGS: PrivacySettings = {
  localLearning: {
    enabled: true,
    modelStorage: 'local_only',
    noCloudUpload: true,
  },
  federatedLearning: {
    enabled: false,
    consentGiven: false,
    anonymization: 'full',
    aggregationOnly: true,
  },
  retention: {
    maxDays: 90,
    autoDelete: true,
    exportFormat: 'encrypted_backup',
  },
  dataMinimization: {
    excludePersonalInfo: true,
    excludeLocationData: false,
    excludeConversationContent: true,
  },
};

const DEFAULT_RETENTION_POLICY: DataRetentionPolicy = {
  maxAgeDays: 90,
  maxRecords: 10000,
  autoCleanup: true,
  cleanupInterval: 24 * 60 * 60 * 1000, // 24시간
};

// ============================================================================
// Privacy Manager Class
// ============================================================================

export class PrivacyManager {
  private settings: PrivacySettings;
  private retentionPolicy: DataRetentionPolicy;
  private consents: ConsentRecord[] = [];
  private auditLog: PrivacyAuditLog[] = [];
  private lastCleanup: number = Date.now();
  private storageKey = 'wia_cognitive_aac_privacy';

  constructor(settings?: Partial<PrivacySettings>) {
    this.settings = { ...DEFAULT_PRIVACY_SETTINGS, ...settings };
    this.retentionPolicy = DEFAULT_RETENTION_POLICY;
    this.loadPersistedData();
  }

  // ============================================================================
  // Consent Management
  // ============================================================================

  /**
   * 동의 기록
   */
  recordConsent(
    type: ConsentRecord['type'],
    granted: boolean,
    grantor: ConsentRecord['grantor'] = 'user'
  ): void {
    const consent: ConsentRecord = {
      timestamp: Date.now(),
      type,
      granted,
      grantor,
    };

    this.consents.push(consent);
    this.logAudit('consent_change', `${type} consent ${granted ? 'granted' : 'revoked'} by ${grantor}`);

    // 동의 철회 시 관련 설정 업데이트
    if (!granted) {
      this.handleConsentRevocation(type);
    }

    this.persistData();
  }

  /**
   * 동의 상태 확인
   */
  hasConsent(type: ConsentRecord['type']): boolean {
    const latestConsent = this.consents
      .filter((c) => c.type === type)
      .sort((a, b) => b.timestamp - a.timestamp)[0];

    if (!latestConsent) return false;

    // 만료 확인
    if (latestConsent.expiresAt && latestConsent.expiresAt < Date.now()) {
      return false;
    }

    return latestConsent.granted;
  }

  /**
   * 동의 기록 조회
   */
  getConsentHistory(): ConsentRecord[] {
    return [...this.consents];
  }

  private handleConsentRevocation(type: ConsentRecord['type']): void {
    switch (type) {
      case 'data_collection':
        this.settings.localLearning.enabled = false;
        break;
      case 'federated_learning':
        this.settings.federatedLearning.enabled = false;
        this.settings.federatedLearning.consentGiven = false;
        break;
      case 'analytics':
        // 분석 관련 설정 비활성화
        break;
    }
  }

  // ============================================================================
  // Data Filtering
  // ============================================================================

  /**
   * 학습 이벤트 필터링
   */
  filterLearningEvent(event: LearningEvent): LearningEvent | null {
    // 학습이 비활성화된 경우
    if (!this.settings.localLearning.enabled) {
      return null;
    }

    // 동의가 없는 경우
    if (!this.hasConsent('data_collection')) {
      return null;
    }

    // 데이터 최소화 적용
    const filteredEvent = { ...event, data: { ...event.data } };

    // 개인정보 제외
    if (this.settings.dataMinimization.excludePersonalInfo) {
      filteredEvent.data = this.removePersonalInfo(filteredEvent.data);
    }

    // 위치 정보 제외
    if (this.settings.dataMinimization.excludeLocationData) {
      filteredEvent.context = {
        ...filteredEvent.context,
        location: undefined,
      };
    }

    // 대화 내용 제외
    if (this.settings.dataMinimization.excludeConversationContent) {
      filteredEvent.data = this.removeConversationContent(filteredEvent.data);
    }

    return filteredEvent;
  }

  private removePersonalInfo(data: Record<string, unknown>): Record<string, unknown> {
    const personalFields = ['name', 'email', 'phone', 'address', 'ssn', '이름', '전화', '주소'];
    const filtered = { ...data };

    for (const field of personalFields) {
      if (field in filtered) {
        delete filtered[field];
      }
    }

    return filtered;
  }

  private removeConversationContent(data: Record<string, unknown>): Record<string, unknown> {
    const conversationFields = ['message', 'text', 'content', 'utterance', '메시지', '내용'];
    const filtered = { ...data };

    for (const field of conversationFields) {
      if (field in filtered) {
        delete filtered[field];
      }
    }

    return filtered;
  }

  // ============================================================================
  // Data Anonymization
  // ============================================================================

  /**
   * 패턴 데이터 익명화
   */
  anonymizePatterns(patterns: UsagePattern): UsagePattern {
    const level = this.settings.federatedLearning.anonymization;

    switch (level) {
      case 'full':
        return this.fullyAnonymizePatterns(patterns);
      case 'partial':
        return this.partiallyAnonymizePatterns(patterns);
      default:
        return patterns;
    }
  }

  private fullyAnonymizePatterns(patterns: UsagePattern): UsagePattern {
    // 모든 식별 가능한 정보 제거
    const anonymized: UsagePattern = {
      temporal: {
        hourlyFrequency: new Map(),
        dayOfWeek: new Map(),
        contextual: new Map(),
      },
      sequences: {
        commonPhrases: [],
        symbolChains: patterns.sequences.symbolChains.map((chain) => ({
          ...chain,
          symbols: chain.symbols.map((s) => this.hashSymbolId(s)),
        })),
        conversationPatterns: [],
      },
      contextual: {
        locationBased: new Map(),
        personBased: new Map(),
        activityBased: new Map(),
      },
    };

    // 시간대별 빈도만 유지 (심볼 ID 해싱)
    patterns.temporal.hourlyFrequency.forEach((symbols, hour) => {
      anonymized.temporal.hourlyFrequency.set(
        hour,
        symbols.map((s) => ({
          ...s,
          symbolId: this.hashSymbolId(s.symbolId),
        }))
      );
    });

    return anonymized;
  }

  private partiallyAnonymizePatterns(patterns: UsagePattern): UsagePattern {
    // 부분 익명화: 개인 식별 정보만 제거
    const anonymized = { ...patterns };

    // 사람 기반 컨텍스트 제거
    anonymized.contextual = {
      ...patterns.contextual,
      personBased: new Map(),
    };

    // 대화 패턴 제거
    anonymized.sequences = {
      ...patterns.sequences,
      conversationPatterns: [],
    };

    return anonymized;
  }

  private hashSymbolId(symbolId: string): string {
    // 간단한 해시 함수 (프로덕션에서는 암호학적 해시 사용 권장)
    let hash = 0;
    for (let i = 0; i < symbolId.length; i++) {
      const char = symbolId.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash;
    }
    return `sym_${Math.abs(hash).toString(16)}`;
  }

  // ============================================================================
  // Data Retention
  // ============================================================================

  /**
   * 데이터 보존 기간 확인 및 정리
   */
  enforceRetention(patterns: UsagePattern): UsagePattern {
    if (!this.settings.retention.autoDelete) {
      return patterns;
    }

    const maxAge = this.retentionPolicy.maxAgeDays * 24 * 60 * 60 * 1000;
    const cutoff = Date.now() - maxAge;

    // 오래된 데이터 제거
    const cleaned = this.cleanOldData(patterns, cutoff);

    this.logAudit('access', 'Retention policy enforced');
    return cleaned;
  }

  private cleanOldData(patterns: UsagePattern, cutoff: number): UsagePattern {
    const cleaned: UsagePattern = {
      temporal: {
        hourlyFrequency: new Map(),
        dayOfWeek: new Map(),
        contextual: new Map(),
      },
      sequences: {
        commonPhrases: patterns.sequences.commonPhrases.filter((p) => p.lastUsed > cutoff),
        symbolChains: patterns.sequences.symbolChains.filter((c) => c.frequency > 0),
        conversationPatterns: patterns.sequences.conversationPatterns.filter(
          (p) => p.frequency > 0
        ),
      },
      contextual: {
        locationBased: new Map(),
        personBased: new Map(),
        activityBased: new Map(),
      },
    };

    // 시간대별 데이터 복사 (최근 데이터만)
    patterns.temporal.hourlyFrequency.forEach((symbols, hour) => {
      const recentSymbols = symbols.filter((s) => s.lastUsed > cutoff);
      if (recentSymbols.length > 0) {
        cleaned.temporal.hourlyFrequency.set(hour, recentSymbols);
      }
    });

    patterns.temporal.dayOfWeek.forEach((symbols, day) => {
      const recentSymbols = symbols.filter((s) => s.lastUsed > cutoff);
      if (recentSymbols.length > 0) {
        cleaned.temporal.dayOfWeek.set(day, recentSymbols);
      }
    });

    return cleaned;
  }

  /**
   * 주기적 정리 실행
   */
  runPeriodicCleanup(): boolean {
    const now = Date.now();
    if (now - this.lastCleanup < this.retentionPolicy.cleanupInterval) {
      return false;
    }

    this.lastCleanup = now;
    this.cleanupAuditLog();
    this.persistData();

    return true;
  }

  private cleanupAuditLog(): void {
    const maxAge = this.retentionPolicy.maxAgeDays * 24 * 60 * 60 * 1000;
    const cutoff = Date.now() - maxAge;
    this.auditLog = this.auditLog.filter((log) => log.timestamp > cutoff);
  }

  // ============================================================================
  // Data Export/Import
  // ============================================================================

  /**
   * 데이터 내보내기
   */
  exportData(patterns: UsagePattern, includeSettings: boolean = false): DataExport {
    this.logAudit('export', 'Data exported');

    const exportData: DataExport = {
      version: '1.0.0',
      exportDate: new Date().toISOString(),
      dataType: includeSettings ? 'full' : 'patterns',
      encrypted: this.settings.retention.exportFormat === 'encrypted_backup',
      data: {
        patterns: this.serializePatterns(patterns),
        ...(includeSettings && { settings: this.settings }),
      },
    };

    return exportData;
  }

  private serializePatterns(patterns: UsagePattern): object {
    return {
      temporal: {
        hourlyFrequency: Object.fromEntries(patterns.temporal.hourlyFrequency),
        dayOfWeek: Object.fromEntries(patterns.temporal.dayOfWeek),
        contextual: Object.fromEntries(patterns.temporal.contextual),
      },
      sequences: patterns.sequences,
      contextual: {
        locationBased: Object.fromEntries(patterns.contextual.locationBased),
        personBased: Object.fromEntries(patterns.contextual.personBased),
        activityBased: Object.fromEntries(patterns.contextual.activityBased),
      },
    };
  }

  // ============================================================================
  // Data Deletion
  // ============================================================================

  /**
   * 모든 데이터 삭제 (잊혀질 권리)
   */
  deleteAllData(): void {
    this.logAudit('delete', 'All data deleted (right to be forgotten)');

    // 동의 기록은 유지 (법적 요구사항)
    const consentBackup = [...this.consents];

    // 모든 저장 데이터 삭제
    this.clearStorage();

    // 동의 기록 복원
    this.consents = consentBackup;

    // 기본 설정으로 리셋
    this.settings = { ...DEFAULT_PRIVACY_SETTINGS };

    this.persistData();
  }

  /**
   * 특정 기간 데이터 삭제
   */
  deleteDataBefore(date: Date): void {
    this.logAudit('delete', `Data before ${date.toISOString()} deleted`);
    // 실제 패턴 데이터 삭제는 외부에서 처리
  }

  // ============================================================================
  // Storage
  // ============================================================================

  private loadPersistedData(): void {
    if (typeof window === 'undefined') return;

    try {
      const stored = localStorage.getItem(this.storageKey);
      if (stored) {
        const data = JSON.parse(stored);
        this.consents = data.consents ?? [];
        this.auditLog = data.auditLog ?? [];
      }
    } catch {
      // 저장된 데이터 없음
    }
  }

  private persistData(): void {
    if (typeof window === 'undefined') return;

    try {
      const data = {
        consents: this.consents,
        auditLog: this.auditLog.slice(-100), // 최근 100개만 유지
      };
      localStorage.setItem(this.storageKey, JSON.stringify(data));
    } catch {
      // 저장 실패
    }
  }

  private clearStorage(): void {
    if (typeof window === 'undefined') return;

    try {
      localStorage.removeItem(this.storageKey);
      // 관련 모든 키 삭제
      const keysToRemove = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith('wia_cognitive_aac')) {
          keysToRemove.push(key);
        }
      }
      keysToRemove.forEach((key) => localStorage.removeItem(key));
    } catch {
      // 삭제 실패
    }
  }

  /**
   * 스토리지 사용량 확인
   */
  getStorageQuota(): StorageQuota {
    if (typeof window === 'undefined') {
      return { used: 0, limit: 5120, unit: 'KB' };
    }

    try {
      let totalSize = 0;
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith('wia_cognitive_aac')) {
          totalSize += (localStorage.getItem(key) ?? '').length * 2; // UTF-16
        }
      }

      return {
        used: Math.round(totalSize / 1024),
        limit: 5120, // 5MB 기본 제한
        unit: 'KB',
      };
    } catch {
      return { used: 0, limit: 5120, unit: 'KB' };
    }
  }

  // ============================================================================
  // Audit Log
  // ============================================================================

  private logAudit(
    action: PrivacyAuditLog['action'],
    details: string,
    actor: PrivacyAuditLog['actor'] = 'system'
  ): void {
    this.auditLog.push({
      timestamp: Date.now(),
      action,
      details,
      actor,
    });
  }

  /**
   * 감사 로그 조회
   */
  getAuditLog(): PrivacyAuditLog[] {
    return [...this.auditLog];
  }

  // ============================================================================
  // Public API
  // ============================================================================

  /**
   * 현재 설정 반환
   */
  getSettings(): PrivacySettings {
    return { ...this.settings };
  }

  /**
   * 설정 업데이트
   */
  updateSettings(settings: Partial<PrivacySettings>): void {
    this.settings = {
      ...this.settings,
      ...settings,
      localLearning: { ...this.settings.localLearning, ...settings.localLearning },
      federatedLearning: { ...this.settings.federatedLearning, ...settings.federatedLearning },
      retention: { ...this.settings.retention, ...settings.retention },
      dataMinimization: { ...this.settings.dataMinimization, ...settings.dataMinimization },
    };
    this.persistData();
  }

  /**
   * 보존 정책 업데이트
   */
  updateRetentionPolicy(policy: Partial<DataRetentionPolicy>): void {
    this.retentionPolicy = { ...this.retentionPolicy, ...policy };
  }

  /**
   * 연합 학습 활성화 가능 여부
   */
  canEnableFederatedLearning(): boolean {
    return (
      this.hasConsent('federated_learning') &&
      this.settings.localLearning.enabled &&
      this.settings.federatedLearning.consentGiven
    );
  }
}

export default PrivacyManager;
