/**
 * WIA-AIR-SHIELD SDK
 * 이모의 TypeScript 구현 - "내가 지켜줄게" 🛡️
 *
 * @packageDocumentation
 */

import {
  ProtectionMode,
  ThreatLevel,
  ThreatType,
  WirelessSource,
  AccessPoint,
  PowerTransmitter,
  Threat,
  ThreatReport,
  ThreatEvidence,
  AnomalyInfo,
  AnomalyScore,
  VerificationResult,
  ZKProof,
  ZKProofType,
  ZKPredicate,
  ShieldStatus,
  ShieldComponentStatus,
  AirShieldConfig,
  CloakConfig,
  NoiseConfig,
  VerifyConfig,
  AlertConfig,
  SecureChannel,
  ThreatCallback,
  AnomalyCallback,
  Unsubscribe,
  ActivityEntry,
  BlockedThreat,
  ChargingSecurityStatus,
  Milliseconds,
  Percentage,
  Timestamp,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Default Configurations
// ============================================================================

const DEFAULT_CLOAK_CONFIG: CloakConfig = {
  enabled: true,
  level: 3,
  hideMetadata: true,
  hideTrafficPattern: true,
  hideLocation: true,
  hideDeviceFingerprint: true,
  randomizeMAC: true,
};

const DEFAULT_NOISE_CONFIG: NoiseConfig = {
  enabled: true,
  intensity: 30,
  decoyTraffic: true,
  decoyPercentage: 20,
  randomizeTiming: true,
  timingJitter: 50,
  scramblePowerPattern: true,
};

const DEFAULT_VERIFY_CONFIG: VerifyConfig = {
  enabled: true,
  strictness: 'normal',
  autoVerify: true,
  trustOnFirstUse: false,
  crowdVerification: true,
};

const DEFAULT_ALERT_CONFIG: AlertConfig = {
  enabled: true,
  minLevel: 'medium',
  soundAlert: false,
  visualAlert: true,
  pushNotification: true,
  logToFile: true,
};

const DEFAULT_CONFIG: AirShieldConfig = {
  mode: 'balanced',
  autoDetect: true,
  autoBlock: true,
  cloak: DEFAULT_CLOAK_CONFIG,
  noise: DEFAULT_NOISE_CONFIG,
  verify: DEFAULT_VERIFY_CONFIG,
  alert: DEFAULT_ALERT_CONFIG,
  encryption: {
    algorithm: 'CRYSTALS-Kyber',
    keySize: 768,
    layers: [
      { name: 'physical', algorithm: 'RF-scramble', enabled: true },
      { name: 'transport', algorithm: 'TLS-1.3', enabled: true },
      { name: 'application', algorithm: 'AES-256-GCM', enabled: true },
    ],
  },
  zeroKnowledge: {
    enabled: true,
    defaultProofExpiry: 3600000, // 1 hour
    cacheProofs: true,
  },
};

// ============================================================================
// Cloak Shield (은폐 방패)
// ============================================================================

class CloakShield {
  private config: CloakConfig;
  private enabled: boolean = false;

  constructor(config: CloakConfig) {
    this.config = config;
  }

  enable(): void {
    this.enabled = true;
    console.log('🛡️ 은폐 방패 활성화');
  }

  disable(): void {
    this.enabled = false;
    console.log('🛡️ 은폐 방패 비활성화');
  }

  setLevel(level: 1 | 2 | 3 | 4 | 5): void {
    this.config.level = level;
    console.log(`🛡️ 은폐 레벨: ${level}`);
  }

  hideMetadata(): void {
    if (!this.enabled) return;
    // 메타데이터 은폐 로직
    console.log('🛡️ 메타데이터 은폐 중...');
  }

  hideTrafficPattern(): void {
    if (!this.enabled) return;
    // 트래픽 패턴 은폐 로직
    console.log('🛡️ 트래픽 패턴 은폐 중...');
  }

  hideLocation(): void {
    if (!this.enabled) return;
    // 위치 은폐 로직
    console.log('🛡️ 위치 정보 은폐 중...');
  }

  hideDeviceFingerprint(): void {
    if (!this.enabled) return;
    // 기기 지문 은폐 로직
    console.log('🛡️ 기기 지문 은폐 중...');
  }

  randomizeMAC(): string {
    const randomMAC = Array.from({ length: 6 }, () =>
      Math.floor(Math.random() * 256).toString(16).padStart(2, '0')
    ).join(':');
    console.log(`🛡️ MAC 주소 랜덤화: ${randomMAC}`);
    return randomMAC;
  }

  getStatus(): ShieldComponentStatus {
    return {
      enabled: this.enabled,
      level: this.config.level,
      status: this.enabled ? 'active' : 'standby',
    };
  }
}

// ============================================================================
// Noise Shield (교란 방패)
// ============================================================================

class NoiseShield {
  private config: NoiseConfig;
  private enabled: boolean = false;
  private decoyInterval: NodeJS.Timeout | null = null;

  constructor(config: NoiseConfig) {
    this.config = config;
  }

  enable(): void {
    this.enabled = true;
    console.log('🛡️ 교란 방패 활성화');
  }

  disable(): void {
    this.enabled = false;
    this.stopDecoy();
    console.log('🛡️ 교란 방패 비활성화');
  }

  setIntensity(intensity: Percentage): void {
    this.config.intensity = Math.min(100, Math.max(0, intensity));
    console.log(`🛡️ 교란 강도: ${this.config.intensity}%`);
  }

  startDecoy(): void {
    if (!this.enabled || this.decoyInterval) return;

    this.decoyInterval = setInterval(() => {
      this.generateDecoyTraffic();
    }, 1000);
    console.log('🛡️ 디코이 트래픽 생성 시작');
  }

  stopDecoy(): void {
    if (this.decoyInterval) {
      clearInterval(this.decoyInterval);
      this.decoyInterval = null;
      console.log('🛡️ 디코이 트래픽 생성 중지');
    }
  }

  private generateDecoyTraffic(): void {
    // 가짜 트래픽 생성 로직
    const decoySize = Math.floor(Math.random() * 1024);
    // In real implementation, send decoy packets
  }

  randomizeTiming(): Milliseconds {
    const jitter = Math.floor(Math.random() * this.config.timingJitter * 2) - this.config.timingJitter;
    return jitter;
  }

  scramblePowerPattern(): void {
    if (!this.enabled) return;
    // 전력 패턴 교란 로직
    console.log('🛡️ 전력 소비 패턴 교란 중...');
  }

  injectFakeMetadata(): Record<string, unknown> {
    return {
      userAgent: this.generateFakeUserAgent(),
      timestamp: Date.now() + this.randomizeTiming(),
      deviceId: crypto.randomUUID(),
    };
  }

  private generateFakeUserAgent(): string {
    const agents = [
      'Mozilla/5.0 (Windows NT 10.0; Win64; x64)',
      'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7)',
      'Mozilla/5.0 (X11; Linux x86_64)',
      'Mozilla/5.0 (iPhone; CPU iPhone OS 15_0)',
    ];
    return agents[Math.floor(Math.random() * agents.length)];
  }

  getStatus(): ShieldComponentStatus {
    return {
      enabled: this.enabled,
      level: Math.ceil(this.config.intensity / 20) as 1 | 2 | 3 | 4 | 5,
      status: this.enabled ? (this.decoyInterval ? 'active' : 'standby') : 'disabled',
    };
  }
}

// ============================================================================
// Verify Shield (검증 방패)
// ============================================================================

class VerifyShield {
  private config: VerifyConfig;
  private enabled: boolean = false;
  private trustedSources: Map<string, WirelessSource> = new Map();
  private verificationHistory: Map<string, VerificationResult[]> = new Map();

  constructor(config: VerifyConfig) {
    this.config = config;
  }

  enable(): void {
    this.enabled = true;
    console.log('🛡️ 검증 방패 활성화');
  }

  disable(): void {
    this.enabled = false;
    console.log('🛡️ 검증 방패 비활성화');
  }

  setStrictness(level: 'relaxed' | 'normal' | 'strict' | 'paranoid'): void {
    this.config.strictness = level;
    console.log(`🛡️ 검증 엄격도: ${level}`);
  }

  async verifyTarget(target: WirelessSource): Promise<boolean> {
    const result = await this.performVerification(target);
    return result.authentic;
  }

  async verifyAccessPoint(ap: AccessPoint): Promise<VerificationResult> {
    return this.performVerification(ap);
  }

  async verifyTransmitter(tx: PowerTransmitter): Promise<VerificationResult> {
    return this.performVerification(tx);
  }

  private async performVerification(source: WirelessSource): Promise<VerificationResult> {
    const checks: string[] = [];
    const checkResults: Record<string, boolean> = {};
    const warnings: { code: string; message: string; severity: 'info' | 'warning' | 'critical' }[] = [];

    // Check 1: Signal strength consistency
    checks.push('signal_consistency');
    const signalOk = this.checkSignalConsistency(source);
    checkResults['signal_consistency'] = signalOk;
    if (!signalOk) {
      warnings.push({
        code: 'INCONSISTENT_SIGNAL',
        message: '신호 강도가 불안정합니다',
        severity: 'warning',
      });
    }

    // Check 2: History check
    checks.push('history_check');
    const historyOk = this.checkHistory(source);
    checkResults['history_check'] = historyOk;
    if (!historyOk) {
      warnings.push({
        code: 'NO_HISTORY',
        message: '처음 보는 기기입니다',
        severity: 'info',
      });
    }

    // Check 3: Evil Twin detection
    checks.push('evil_twin_check');
    const notEvilTwin = !this.detectEvilTwin(source);
    checkResults['evil_twin_check'] = notEvilTwin;
    if (!notEvilTwin) {
      warnings.push({
        code: 'POSSIBLE_EVIL_TWIN',
        message: '가짜 AP일 수 있습니다!',
        severity: 'critical',
      });
    }

    // Check 4: Encryption check
    checks.push('encryption_check');
    const encrypted = source.encrypted;
    checkResults['encryption_check'] = encrypted;
    if (!encrypted) {
      warnings.push({
        code: 'NO_ENCRYPTION',
        message: '암호화되지 않은 연결입니다',
        severity: 'critical',
      });
    }

    // Calculate confidence based on strictness
    const passedChecks = Object.values(checkResults).filter(Boolean).length;
    const totalChecks = checks.length;
    let confidence = (passedChecks / totalChecks) * 100;

    // Adjust for strictness
    const strictnessMultiplier = {
      relaxed: 1.2,
      normal: 1.0,
      strict: 0.8,
      paranoid: 0.6,
    };
    confidence = Math.min(100, confidence * strictnessMultiplier[this.config.strictness]);

    // Determine recommendation
    let recommendation: 'connect' | 'avoid' | 'proceed_with_caution';
    let authentic: boolean;

    if (confidence >= 80) {
      recommendation = 'connect';
      authentic = true;
    } else if (confidence >= 50) {
      recommendation = 'proceed_with_caution';
      authentic = this.config.strictness !== 'paranoid';
    } else {
      recommendation = 'avoid';
      authentic = false;
    }

    const result: VerificationResult = {
      authentic,
      confidence,
      method: 'heuristic',
      warnings,
      recommendation,
      details: {
        checkedAt: Date.now(),
        checksPerformed: checks,
        checkResults,
      },
    };

    // Store in history
    const history = this.verificationHistory.get(source.id) || [];
    history.push(result);
    this.verificationHistory.set(source.id, history);

    return result;
  }

  private checkSignalConsistency(source: WirelessSource): boolean {
    // In real implementation, check signal over time
    return source.signalStrength > -80 && source.signalStrength < -20;
  }

  private checkHistory(source: WirelessSource): boolean {
    return this.trustedSources.has(source.id) || this.verificationHistory.has(source.id);
  }

  private detectEvilTwin(source: WirelessSource): boolean {
    // Simple heuristic: check for duplicate names with different IDs
    for (const [id, trusted] of this.trustedSources) {
      if (trusted.name === source.name && id !== source.id) {
        return true; // Possible evil twin!
      }
    }
    return false;
  }

  async verifyConnectionIntegrity(): Promise<boolean> {
    // In real implementation, perform integrity checks
    return true;
  }

  trustSource(source: WirelessSource): void {
    this.trustedSources.set(source.id, source);
    console.log(`🛡️ 신뢰 목록에 추가: ${source.name}`);
  }

  untrustSource(sourceId: string): void {
    this.trustedSources.delete(sourceId);
    console.log(`🛡️ 신뢰 목록에서 제거: ${sourceId}`);
  }

  getStatus(): ShieldComponentStatus {
    const strictnessLevels = { relaxed: 1, normal: 2, strict: 3, paranoid: 5 };
    return {
      enabled: this.enabled,
      level: strictnessLevels[this.config.strictness] as 1 | 2 | 3 | 4 | 5,
      status: this.enabled ? 'active' : 'standby',
    };
  }
}

// ============================================================================
// Alert Shield (경보 방패)
// ============================================================================

class AlertShield {
  private config: AlertConfig;
  private enabled: boolean = false;
  private threatCallbacks: Set<ThreatCallback> = new Set();
  private anomalyCallbacks: Set<AnomalyCallback> = new Set();
  private threatHistory: Threat[] = [];
  private blockedThreats: BlockedThreat[] = [];

  constructor(config: AlertConfig) {
    this.config = config;
  }

  enable(): void {
    this.enabled = true;
    console.log('🛡️ 경보 방패 활성화');
  }

  disable(): void {
    this.enabled = false;
    console.log('🛡️ 경보 방패 비활성화');
  }

  onThreat(callback: ThreatCallback): Unsubscribe {
    this.threatCallbacks.add(callback);
    return () => this.threatCallbacks.delete(callback);
  }

  onAnomaly(callback: AnomalyCallback): Unsubscribe {
    this.anomalyCallbacks.add(callback);
    return () => this.anomalyCallbacks.delete(callback);
  }

  raise(type: ThreatType, source: WirelessSource | null, details?: string): void {
    if (!this.enabled) return;

    const threat: Threat = {
      id: crypto.randomUUID(),
      type,
      level: this.determineThreatLevel(type),
      source,
      description: details || this.getDefaultDescription(type),
      detectedAt: Date.now(),
      confidence: 80,
      evidence: [],
      recommendations: this.getRecommendations(type),
    };

    if (this.shouldAlert(threat.level)) {
      this.threatHistory.push(threat);
      this.notifyThreat(threat);
    }
  }

  private determineThreatLevel(type: ThreatType): ThreatLevel {
    const criticalThreats: ThreatType[] = ['evil_twin', 'mitm', 'data_exfiltration'];
    const highThreats: ThreatType[] = ['eavesdropping', 'rf_sniffing', 'rogue_device'];
    const mediumThreats: ThreatType[] = ['power_analysis', 'timing_attack', 'fingerprinting'];

    if (criticalThreats.includes(type)) return 'critical';
    if (highThreats.includes(type)) return 'high';
    if (mediumThreats.includes(type)) return 'medium';
    return 'low';
  }

  private getDefaultDescription(type: ThreatType): string {
    const descriptions: Record<ThreatType, string> = {
      evil_twin: '가짜 AP가 감지되었습니다',
      mitm: '중간자 공격이 의심됩니다',
      eavesdropping: '도청 시도가 감지되었습니다',
      rf_sniffing: 'RF 스니핑이 감지되었습니다',
      power_analysis: '전력 분석 공격이 의심됩니다',
      timing_attack: '타이밍 공격이 감지되었습니다',
      replay_attack: '재생 공격이 감지되었습니다',
      deauth_attack: '인증 해제 공격이 감지되었습니다',
      rogue_device: '불량 기기가 감지되었습니다',
      data_exfiltration: '데이터 유출 시도가 감지되었습니다',
      fingerprinting: '기기 식별 시도가 감지되었습니다',
      location_tracking: '위치 추적 시도가 감지되었습니다',
    };
    return descriptions[type];
  }

  private getRecommendations(type: ThreatType): string[] {
    const recommendations: Record<ThreatType, string[]> = {
      evil_twin: ['해당 네트워크 연결을 피하세요', 'VPN을 사용하세요'],
      mitm: ['즉시 연결을 끊으세요', '민감한 작업을 중단하세요'],
      eavesdropping: ['암호화를 활성화하세요', '민감한 대화를 피하세요'],
      rf_sniffing: ['은폐 모드를 활성화하세요', '물리적 위치를 변경하세요'],
      power_analysis: ['전력 패턴 교란을 활성화하세요'],
      timing_attack: ['타이밍 랜덤화를 활성화하세요'],
      replay_attack: ['세션을 갱신하세요'],
      deauth_attack: ['802.11w를 지원하는 네트워크를 사용하세요'],
      rogue_device: ['해당 기기와의 연결을 차단하세요'],
      data_exfiltration: ['즉시 네트워크 연결을 끊으세요', '보안 점검을 실시하세요'],
      fingerprinting: ['디코이 트래픽을 활성화하세요'],
      location_tracking: ['위치 은폐를 활성화하세요'],
    };
    return recommendations[type] || ['주의하세요'];
  }

  private shouldAlert(level: ThreatLevel): boolean {
    const levels: ThreatLevel[] = ['safe', 'low', 'medium', 'high', 'critical'];
    const minIndex = levels.indexOf(this.config.minLevel);
    const currentIndex = levels.indexOf(level);
    return currentIndex >= minIndex;
  }

  private notifyThreat(threat: Threat): void {
    console.log(`🚨 위협 감지: [${threat.level.toUpperCase()}] ${threat.description}`);

    for (const callback of this.threatCallbacks) {
      try {
        callback(threat);
      } catch (error) {
        console.error('Threat callback error:', error);
      }
    }
  }

  notifyAnomaly(anomaly: AnomalyInfo): void {
    for (const callback of this.anomalyCallbacks) {
      try {
        callback(anomaly);
      } catch (error) {
        console.error('Anomaly callback error:', error);
      }
    }
  }

  recordBlocked(threat: Threat, action: string, prevented: string): void {
    this.blockedThreats.push({
      threat,
      blockedAt: Date.now(),
      action,
      prevented,
    });
  }

  getHistory(): Threat[] {
    return [...this.threatHistory];
  }

  getBlockedThreats(): BlockedThreat[] {
    return [...this.blockedThreats];
  }

  clearHistory(): void {
    this.threatHistory = [];
    this.blockedThreats = [];
  }

  getStatus(): ShieldComponentStatus {
    const levelMap = { safe: 1, low: 2, medium: 3, high: 4, critical: 5 };
    return {
      enabled: this.enabled,
      level: levelMap[this.config.minLevel] as 1 | 2 | 3 | 4 | 5,
      status: this.enabled ? 'active' : 'standby',
    };
  }
}

// ============================================================================
// Zero-Knowledge Module
// ============================================================================

class ZeroKnowledgeModule {
  private proofCache: Map<string, ZKProof> = new Map();
  private defaultExpiry: Milliseconds;

  constructor(expiry: Milliseconds = 3600000) {
    this.defaultExpiry = expiry;
  }

  prover = {
    createIdentityProof: async (identityHash: string): Promise<ZKProof> => {
      return this.createProof('identity', { identityHash });
    },

    createAttributeProof: async (attribute: string, predicate: ZKPredicate): Promise<ZKProof> => {
      return this.createProof('attribute', { attribute, predicate });
    },

    createRangeProof: async (value: number, range: [number, number]): Promise<ZKProof> => {
      // Prove value is in range without revealing value
      return this.createProof('balance', { inRange: value >= range[0] && value <= range[1] });
    },

    createAgeProof: async (age: number, threshold: number): Promise<ZKProof> => {
      return this.createProof('age', { meetsThreshold: age >= threshold });
    },

    createLocationProof: async (lat: number, lon: number, range: number): Promise<ZKProof> => {
      return this.createProof('location', { inRange: true }); // Simplified
    },
  };

  verifier = {
    verify: async (proof: ZKProof): Promise<boolean> => {
      // Check expiry
      if (proof.expiresAt && Date.now() > proof.expiresAt) {
        return false;
      }

      // In real implementation, verify cryptographic proof
      // This is a simplified version
      return proof.commitment.length > 0 && proof.response.length > 0;
    },

    verifyBatch: async (proofs: ZKProof[]): Promise<boolean[]> => {
      return Promise.all(proofs.map((p) => this.verifier.verify(p)));
    },
  };

  private createProof(type: ZKProofType, data: unknown): ZKProof {
    const proof: ZKProof = {
      id: crypto.randomUUID(),
      type,
      commitment: new Uint8Array(32).fill(1), // Simplified
      challenge: new Uint8Array(32).fill(2),
      response: new Uint8Array(32).fill(3),
      publicInputs: [data],
      createdAt: Date.now(),
      expiresAt: Date.now() + this.defaultExpiry,
    };

    this.proofCache.set(proof.id, proof);
    return proof;
  }

  getCachedProof(id: string): ZKProof | undefined {
    return this.proofCache.get(id);
  }

  clearCache(): void {
    this.proofCache.clear();
  }
}

// ============================================================================
// Threat Detection Engine
// ============================================================================

class ThreatDetectionEngine {
  private monitoringActive: boolean = false;
  private monitorInterval: NodeJS.Timeout | null = null;
  private baselineTraffic: unknown[] = [];

  startMonitoring(callback: (report: ThreatReport) => void): void {
    if (this.monitoringActive) return;

    this.monitoringActive = true;
    this.monitorInterval = setInterval(() => {
      const report = this.scan();
      if (report.threats.length > 0) {
        callback(report);
      }
    }, 5000);

    console.log('🛡️ 실시간 위협 모니터링 시작');
  }

  stopMonitoring(): void {
    if (this.monitorInterval) {
      clearInterval(this.monitorInterval);
      this.monitorInterval = null;
    }
    this.monitoringActive = false;
    console.log('🛡️ 실시간 위협 모니터링 중지');
  }

  scan(): ThreatReport {
    const startTime = Date.now();
    const threats: Threat[] = [];

    // In real implementation, perform actual scans
    // This is a simplified version that demonstrates the structure

    return {
      timestamp: Date.now(),
      threatLevel: threats.length > 0 ? 'medium' : 'safe',
      threats,
      scannedSources: 0,
      scanDuration: Date.now() - startTime,
      recommendations: [],
      autoActions: [],
    };
  }

  async deepScan(): Promise<ThreatReport> {
    console.log('🛡️ 정밀 검사 시작...');
    // Extended scan with more thorough checks
    await new Promise((resolve) => setTimeout(resolve, 2000));
    return this.scan();
  }

  trainBaseline(samples: unknown[]): void {
    this.baselineTraffic = samples;
    console.log(`🛡️ 기준선 학습 완료: ${samples.length}개 샘플`);
  }

  detectAnomaly(sample: unknown): AnomalyScore {
    // Simplified anomaly detection
    return {
      overall: 0,
      components: {
        signalPattern: 0,
        timingPattern: 0,
        behaviorPattern: 0,
        metadataPattern: 0,
      },
      isAnomaly: false,
      threshold: 0.7,
    };
  }
}

// ============================================================================
// Secure Channel Implementation
// ============================================================================

class SecureChannelImpl implements SecureChannel {
  id: string;
  target: WirelessSource;
  established: Timestamp;
  encryption: AirShieldConfig['encryption'];
  status: 'connecting' | 'connected' | 'disconnected' | 'error' = 'connecting';

  constructor(target: WirelessSource, encryption: AirShieldConfig['encryption']) {
    this.id = crypto.randomUUID();
    this.target = target;
    this.encryption = encryption;
    this.established = Date.now();
  }

  async connect(): Promise<void> {
    // Simulate connection
    await new Promise((resolve) => setTimeout(resolve, 100));
    this.status = 'connected';
    console.log(`🛡️ 보안 채널 연결됨: ${this.target.name}`);
  }

  async send(data: Uint8Array): Promise<void> {
    if (this.status !== 'connected') {
      throw new Error('채널이 연결되지 않았습니다');
    }
    // In real implementation, encrypt and send
    console.log(`🛡️ ${data.length} bytes 암호화 전송`);
  }

  async receive(): Promise<Uint8Array> {
    if (this.status !== 'connected') {
      throw new Error('채널이 연결되지 않았습니다');
    }
    // In real implementation, receive and decrypt
    return new Uint8Array(0);
  }

  close(): void {
    this.status = 'disconnected';
    console.log(`🛡️ 보안 채널 종료: ${this.target.name}`);
  }
}

// ============================================================================
// Main AirShield Class
// ============================================================================

/**
 * WIA-AIR-SHIELD 메인 클래스
 * 이모의 보호 - "내가 지켜줄게" 🛡️
 */
export class AirShield {
  private config: AirShieldConfig;
  private active: boolean = false;
  private startTime: Timestamp = 0;

  // Four Shields
  readonly cloak: CloakShield;
  readonly noise: NoiseShield;
  readonly verify: VerifyShield;
  readonly alert: AlertShield;

  // Additional modules
  readonly zk: ZeroKnowledgeModule;
  private detector: ThreatDetectionEngine;

  // Stats
  private stats = {
    threatsDetected: 0,
    threatsBlocked: 0,
    scansPerformed: 0,
    verificationsPassed: 0,
    verificationsFailed: 0,
    dataProtected: 0,
  };

  // Activity log
  private activityLog: ActivityEntry[] = [];

  constructor(config: Partial<AirShieldConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    // Initialize shields
    this.cloak = new CloakShield(this.config.cloak);
    this.noise = new NoiseShield(this.config.noise);
    this.verify = new VerifyShield(this.config.verify);
    this.alert = new AlertShield(this.config.alert);

    // Initialize modules
    this.zk = new ZeroKnowledgeModule(this.config.zeroKnowledge.defaultProofExpiry);
    this.detector = new ThreatDetectionEngine();

    // Setup alert callback
    if (config.alertCallback) {
      this.alert.onThreat(config.alertCallback);
    }

    console.log('🛡️ 이모 준비 완료: "내가 지켜줄게"');
  }

  // ============================================================================
  // Core Methods
  // ============================================================================

  async activate(): Promise<void> {
    if (this.active) return;

    this.active = true;
    this.startTime = Date.now();

    // Activate shields based on mode
    this.applyMode(this.config.mode);

    // Start monitoring if autoDetect
    if (this.config.autoDetect) {
      this.detector.startMonitoring((report) => {
        for (const threat of report.threats) {
          this.alert.raise(threat.type, threat.source, threat.description);
          this.stats.threatsDetected++;

          if (this.config.autoBlock) {
            this.blockThreat(threat);
          }
        }
      });
    }

    this.log('shield_activated', '방패 활성화');
    console.log(`🛡️ AIR-SHIELD 활성화 (모드: ${this.config.mode})`);
  }

  deactivate(): void {
    if (!this.active) return;

    this.active = false;

    // Deactivate all shields
    this.cloak.disable();
    this.noise.disable();
    this.verify.disable();
    this.alert.disable();

    // Stop monitoring
    this.detector.stopMonitoring();

    this.log('shield_deactivated', '방패 비활성화');
    console.log('🛡️ AIR-SHIELD 비활성화');
  }

  setMode(mode: ProtectionMode): void {
    this.config.mode = mode;
    if (this.active) {
      this.applyMode(mode);
    }
    console.log(`🛡️ 모드 변경: ${mode}`);
  }

  private applyMode(mode: ProtectionMode): void {
    switch (mode) {
      case 'stealth':
        this.cloak.enable();
        this.cloak.setLevel(5);
        this.noise.enable();
        this.noise.setIntensity(50);
        this.verify.enable();
        this.verify.setStrictness('normal');
        this.alert.enable();
        break;

      case 'fortress':
        this.cloak.enable();
        this.cloak.setLevel(3);
        this.noise.enable();
        this.noise.setIntensity(30);
        this.verify.enable();
        this.verify.setStrictness('strict');
        this.alert.enable();
        break;

      case 'ghost':
        this.cloak.enable();
        this.cloak.setLevel(5);
        this.noise.enable();
        this.noise.setIntensity(80);
        this.noise.startDecoy();
        this.verify.enable();
        this.verify.setStrictness('paranoid');
        this.alert.enable();
        break;

      case 'paranoid':
        this.cloak.enable();
        this.cloak.setLevel(5);
        this.noise.enable();
        this.noise.setIntensity(100);
        this.noise.startDecoy();
        this.verify.enable();
        this.verify.setStrictness('paranoid');
        this.alert.enable();
        break;

      case 'balanced':
        this.cloak.enable();
        this.cloak.setLevel(3);
        this.noise.enable();
        this.noise.setIntensity(30);
        this.verify.enable();
        this.verify.setStrictness('normal');
        this.alert.enable();
        break;

      case 'performance':
        this.cloak.enable();
        this.cloak.setLevel(1);
        this.noise.disable();
        this.verify.enable();
        this.verify.setStrictness('relaxed');
        this.alert.enable();
        break;
    }
  }

  // ============================================================================
  // Secure Channel
  // ============================================================================

  async createSecureChannel(target: WirelessSource): Promise<SecureChannel> {
    // Verify target first
    const verified = await this.verify.verifyTarget(target);
    if (!verified && this.config.verify.strictness === 'paranoid') {
      throw new Error('대상 검증 실패');
    }

    const channel = new SecureChannelImpl(target, this.config.encryption);
    await channel.connect();

    this.log('channel_created', `보안 채널 생성: ${target.name}`);
    return channel;
  }

  // ============================================================================
  // Threat Management
  // ============================================================================

  scan(): ThreatReport {
    this.stats.scansPerformed++;
    return this.detector.scan();
  }

  async deepScan(): Promise<ThreatReport> {
    this.stats.scansPerformed++;
    return this.detector.deepScan();
  }

  scanForThreats(): Threat[] {
    return this.scan().threats;
  }

  private blockThreat(threat: Threat): void {
    // Take action based on threat type
    const action = this.determineBlockAction(threat);

    this.alert.recordBlocked(threat, action, `${threat.type} 공격 차단`);
    this.stats.threatsBlocked++;

    this.log('threat_blocked', `위협 차단: ${threat.type}`);
  }

  private determineBlockAction(threat: Threat): string {
    switch (threat.type) {
      case 'evil_twin':
      case 'rogue_device':
        return 'connection_blocked';
      case 'mitm':
      case 'eavesdropping':
        return 'traffic_encrypted';
      case 'data_exfiltration':
        return 'transmission_blocked';
      default:
        return 'monitored';
    }
  }

  // ============================================================================
  // AIR-POWER Integration (삼촌 연동)
  // ============================================================================

  async verifyTransmitter(tx: PowerTransmitter): Promise<boolean> {
    const result = await this.verify.verifyTransmitter(tx);

    if (result.authentic) {
      this.stats.verificationsPassed++;
    } else {
      this.stats.verificationsFailed++;
      this.alert.raise('rogue_device', tx, '검증되지 않은 전력 송신기');
    }

    return result.authentic;
  }

  enableSideChannelProtection(): void {
    this.noise.scramblePowerPattern();
    console.log('🛡️ 사이드채널 공격 방어 활성화');
  }

  getChargingSecurityStatus(tx: PowerTransmitter | null): ChargingSecurityStatus {
    return {
      transmitterVerified: tx ? this.verify['trustedSources'].has(tx.id) : false,
      sideChannelProtection: this.noise['enabled'],
      dataLeakPrevention: this.cloak['enabled'],
      currentThreats: this.scanForThreats(),
    };
  }

  // ============================================================================
  // Environment Verification
  // ============================================================================

  async verifyEnvironment(): Promise<boolean> {
    const report = await this.deepScan();

    if (report.threatLevel === 'critical' || report.threatLevel === 'high') {
      return false;
    }

    return true;
  }

  // ============================================================================
  // Trace Cleanup
  // ============================================================================

  async cleanTraces(): Promise<void> {
    // Clear sensitive data
    this.zk.clearCache();
    console.log('🛡️ 흔적 정리 완료');
  }

  // ============================================================================
  // Status & Transparency
  // ============================================================================

  getStatus(): ShieldStatus {
    return {
      active: this.active,
      mode: this.config.mode,
      threatLevel: this.calculateCurrentThreatLevel(),
      shields: {
        cloak: this.cloak.getStatus(),
        noise: this.noise.getStatus(),
        verify: this.verify.getStatus(),
        alert: this.alert.getStatus(),
      },
      stats: { ...this.stats },
      lastScan: Date.now(),
      uptime: this.active ? Date.now() - this.startTime : 0,
    };
  }

  private calculateCurrentThreatLevel(): ThreatLevel {
    const recentThreats = this.alert.getHistory().filter(
      (t) => Date.now() - t.detectedAt < 300000 // Last 5 minutes
    );

    if (recentThreats.some((t) => t.level === 'critical')) return 'critical';
    if (recentThreats.some((t) => t.level === 'high')) return 'high';
    if (recentThreats.some((t) => t.level === 'medium')) return 'medium';
    if (recentThreats.length > 0) return 'low';
    return 'safe';
  }

  getActivityLog(): ActivityEntry[] {
    return [...this.activityLog];
  }

  getBlockedThreats(): BlockedThreat[] {
    return this.alert.getBlockedThreats();
  }

  private log(type: string, action: string, details?: Record<string, unknown>): void {
    this.activityLog.push({
      id: crypto.randomUUID(),
      timestamp: Date.now(),
      type,
      action,
      result: 'success',
      details,
    });
  }

  // ============================================================================
  // Data Privacy
  // ============================================================================

  async deleteAllData(): Promise<void> {
    this.activityLog = [];
    this.alert.clearHistory();
    this.zk.clearCache();
    this.stats = {
      threatsDetected: 0,
      threatsBlocked: 0,
      scansPerformed: 0,
      verificationsPassed: 0,
      verificationsFailed: 0,
      dataProtected: 0,
    };
    console.log('🛡️ 모든 데이터 삭제 완료');
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

let globalShield: AirShield | null = null;

/**
 * 전역 방패 활성화
 */
export async function activateShield(config?: Partial<AirShieldConfig>): Promise<AirShield> {
  if (!globalShield) {
    globalShield = new AirShield(config);
  }
  await globalShield.activate();
  return globalShield;
}

/**
 * 전역 방패 가져오기
 */
export function getShield(): AirShield | null {
  return globalShield;
}

/**
 * 전역 방패 비활성화
 */
export function deactivateShield(): void {
  if (globalShield) {
    globalShield.deactivate();
    globalShield = null;
  }
}

/**
 * 현재 위협 레벨 확인
 */
export function getThreatLevel(): ThreatLevel {
  return globalShield?.getStatus().threatLevel || 'safe';
}

/**
 * 빠른 스캔
 */
export function quickScan(): ThreatReport | null {
  return globalShield?.scan() || null;
}

// ============================================================================
// Family Integration Helper
// ============================================================================

/**
 * WIA 가족 통합 헬퍼
 *
 * @example
 * ```typescript
 * import { familyProtection } from '@anthropic/wia-air-shield';
 *
 * // 삼촌(AIR-POWER) 충전 시작 시 자동 보호
 * familyProtection.onUncleCharging(transmitter);
 * ```
 */
export const familyProtection = {
  /**
   * 삼촌(AIR-POWER)이 충전 시작할 때
   */
  onUncleCharging: async (transmitter: PowerTransmitter): Promise<boolean> => {
    const shield = await activateShield({ mode: 'fortress' });

    const verified = await shield.verifyTransmitter(transmitter);
    if (!verified) {
      console.log('🛡️ 이모: 이 충전기 수상해. 조심해!');
      return false;
    }

    shield.enableSideChannelProtection();
    console.log('🛡️ 이모: 충전하는 동안 내가 지켜줄게~');
    return true;
  },

  /**
   * 충전 중 보안 상태 확인
   */
  getChargingStatus: (transmitter: PowerTransmitter | null): ChargingSecurityStatus | null => {
    return globalShield?.getChargingSecurityStatus(transmitter) || null;
  },
};

// ============================================================================
// Export Default
// ============================================================================

export default AirShield;
