/**
 * WIA-AIR-SHIELD Type Definitions
 * 이모의 타입 정의 - "내가 지켜줄게"
 *
 * @packageDocumentation
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = number;
export type Milliseconds = number;
export type Percentage = number; // 0-100
export type DeviceId = string;
export type ShieldId = string;

// ============================================================================
// Protection Modes
// ============================================================================

/**
 * 보호 모드
 */
export type ProtectionMode =
  | 'stealth'      // 최대 은폐
  | 'fortress'     // 최대 방어
  | 'ghost'        // 흔적 없는 통신
  | 'paranoid'     // 모든 방패 최대치
  | 'balanced'     // 균형 모드 (기본값)
  | 'performance'; // 성능 우선

/**
 * 위협 레벨
 */
export type ThreatLevel =
  | 'safe'      // 녹색
  | 'low'       // 파란색
  | 'medium'    // 노란색
  | 'high'      // 주황색
  | 'critical'; // 빨간색

/**
 * 위협 유형
 */
export type ThreatType =
  | 'evil_twin'           // 가짜 AP/송신기
  | 'mitm'                // 중간자 공격
  | 'eavesdropping'       // 도청
  | 'rf_sniffing'         // RF 스니핑
  | 'power_analysis'      // 전력 분석 공격
  | 'timing_attack'       // 타이밍 공격
  | 'replay_attack'       // 재생 공격
  | 'deauth_attack'       // 인증 해제 공격
  | 'rogue_device'        // 불량 기기
  | 'data_exfiltration'   // 데이터 유출 시도
  | 'fingerprinting'      // 기기 식별 시도
  | 'location_tracking';  // 위치 추적 시도

// ============================================================================
// Wireless Sources
// ============================================================================

/**
 * 무선 소스 유형
 */
export type WirelessSourceType =
  | 'wifi_ap'
  | 'bluetooth'
  | 'power_transmitter'
  | 'cellular'
  | 'nfc'
  | 'unknown';

/**
 * 무선 소스 (AP, 송신기 등)
 */
export interface WirelessSource {
  id: string;
  type: WirelessSourceType;
  name: string;
  signalStrength: number; // dBm
  frequency?: number;
  encrypted: boolean;
  firstSeen: Timestamp;
  lastSeen: Timestamp;
  metadata?: Record<string, unknown>;
}

/**
 * Access Point
 */
export interface AccessPoint extends WirelessSource {
  type: 'wifi_ap';
  ssid: string;
  bssid: string;
  channel: number;
  security: 'open' | 'wep' | 'wpa' | 'wpa2' | 'wpa3';
  hidden: boolean;
}

/**
 * 전력 송신기 (삼촌 연동)
 */
export interface PowerTransmitter extends WirelessSource {
  type: 'power_transmitter';
  powerClass: 'class_a' | 'class_b' | 'class_c' | 'class_d' | 'class_e';
  maxPower: number; // Watts
  currentPower: number;
  efficiency: Percentage;
}

// ============================================================================
// Threats
// ============================================================================

/**
 * 위협 정보
 */
export interface Threat {
  id: string;
  type: ThreatType;
  level: ThreatLevel;
  source: WirelessSource | null;
  description: string;
  detectedAt: Timestamp;
  confidence: Percentage;
  evidence: ThreatEvidence[];
  recommendations: string[];
  autoActionTaken?: string;
}

/**
 * 위협 증거
 */
export interface ThreatEvidence {
  type: string;
  data: unknown;
  timestamp: Timestamp;
  weight: number; // 증거의 중요도
}

/**
 * 위협 리포트
 */
export interface ThreatReport {
  timestamp: Timestamp;
  threatLevel: ThreatLevel;
  threats: Threat[];
  scannedSources: number;
  scanDuration: Milliseconds;
  recommendations: string[];
  autoActions: AutoAction[];
}

/**
 * 자동 조치
 */
export interface AutoAction {
  type: 'block' | 'disconnect' | 'encrypt' | 'alert' | 'log';
  target?: string;
  timestamp: Timestamp;
  reason: string;
}

// ============================================================================
// Anomaly Detection
// ============================================================================

/**
 * 이상 징후 정보
 */
export interface AnomalyInfo {
  id: string;
  type: string;
  score: number; // 0-1, 높을수록 이상
  baseline: unknown;
  observed: unknown;
  deviation: number;
  detectedAt: Timestamp;
}

/**
 * 이상 점수
 */
export interface AnomalyScore {
  overall: number;
  components: {
    signalPattern: number;
    timingPattern: number;
    behaviorPattern: number;
    metadataPattern: number;
  };
  isAnomaly: boolean;
  threshold: number;
}

// ============================================================================
// Verification
// ============================================================================

/**
 * 검증 결과
 */
export interface VerificationResult {
  authentic: boolean;
  confidence: Percentage;
  method: VerificationMethod;
  warnings: Warning[];
  recommendation: 'connect' | 'avoid' | 'proceed_with_caution';
  details: VerificationDetails;
}

export type VerificationMethod =
  | 'certificate'
  | 'signature'
  | 'history'
  | 'crowd'
  | 'ml_model'
  | 'heuristic';

export interface VerificationDetails {
  checkedAt: Timestamp;
  checksPerformed: string[];
  checkResults: Record<string, boolean>;
}

export interface Warning {
  code: string;
  message: string;
  severity: 'info' | 'warning' | 'critical';
}

// ============================================================================
// Zero-Knowledge
// ============================================================================

/**
 * Zero-Knowledge 증명
 */
export interface ZKProof {
  id: string;
  type: ZKProofType;
  commitment: Uint8Array;
  challenge: Uint8Array;
  response: Uint8Array;
  publicInputs: unknown[];
  createdAt: Timestamp;
  expiresAt?: Timestamp;
}

export type ZKProofType =
  | 'identity'
  | 'age'
  | 'balance'
  | 'location'
  | 'membership'
  | 'attribute';

/**
 * ZK 술어 (predicate)
 */
export interface ZKPredicate {
  type: 'equals' | 'greater_than' | 'less_than' | 'in_range' | 'in_set';
  value?: unknown;
  range?: [number, number];
  set?: unknown[];
}

// ============================================================================
// Encryption
// ============================================================================

/**
 * 암호화 설정
 */
export interface EncryptionConfig {
  algorithm: PQAlgorithm;
  keySize: 512 | 768 | 1024;
  layers: EncryptionLayer[];
}

export type PQAlgorithm =
  | 'CRYSTALS-Kyber'
  | 'NTRU'
  | 'SABER'
  | 'Classic-McEliece';

export interface EncryptionLayer {
  name: string;
  algorithm: string;
  enabled: boolean;
}

// ============================================================================
// Shield Status
// ============================================================================

/**
 * 방패 상태
 */
export interface ShieldStatus {
  active: boolean;
  mode: ProtectionMode;
  threatLevel: ThreatLevel;
  shields: {
    cloak: ShieldComponentStatus;
    noise: ShieldComponentStatus;
    verify: ShieldComponentStatus;
    alert: ShieldComponentStatus;
  };
  stats: ShieldStats;
  lastScan: Timestamp;
  uptime: Milliseconds;
}

export interface ShieldComponentStatus {
  enabled: boolean;
  level: number; // 1-5
  status: 'active' | 'standby' | 'disabled' | 'error';
}

export interface ShieldStats {
  threatsDetected: number;
  threatsBlocked: number;
  scansPerformed: number;
  verificationsPassed: number;
  verificationsFailed: number;
  dataProtected: number; // bytes
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * AirShield 설정
 */
export interface AirShieldConfig {
  mode: ProtectionMode;
  autoDetect: boolean;
  autoBlock: boolean;
  alertCallback?: (threat: Threat) => void;

  cloak: CloakConfig;
  noise: NoiseConfig;
  verify: VerifyConfig;
  alert: AlertConfig;

  encryption: EncryptionConfig;
  zeroKnowledge: ZKConfig;
}

export interface CloakConfig {
  enabled: boolean;
  level: 1 | 2 | 3 | 4 | 5;
  hideMetadata: boolean;
  hideTrafficPattern: boolean;
  hideLocation: boolean;
  hideDeviceFingerprint: boolean;
  randomizeMAC: boolean;
}

export interface NoiseConfig {
  enabled: boolean;
  intensity: Percentage;
  decoyTraffic: boolean;
  decoyPercentage: Percentage;
  randomizeTiming: boolean;
  timingJitter: Milliseconds;
  scramblePowerPattern: boolean;
}

export interface VerifyConfig {
  enabled: boolean;
  strictness: 'relaxed' | 'normal' | 'strict' | 'paranoid';
  autoVerify: boolean;
  trustOnFirstUse: boolean;
  crowdVerification: boolean;
}

export interface AlertConfig {
  enabled: boolean;
  minLevel: ThreatLevel;
  soundAlert: boolean;
  visualAlert: boolean;
  pushNotification: boolean;
  logToFile: boolean;
}

export interface ZKConfig {
  enabled: boolean;
  defaultProofExpiry: Milliseconds;
  cacheProofs: boolean;
}

// ============================================================================
// Secure Channel
// ============================================================================

/**
 * 보안 채널
 */
export interface SecureChannel {
  id: string;
  target: WirelessSource;
  established: Timestamp;
  encryption: EncryptionConfig;
  status: 'connecting' | 'connected' | 'disconnected' | 'error';

  send(data: Uint8Array): Promise<void>;
  receive(): Promise<Uint8Array>;
  close(): void;
}

// ============================================================================
// Events
// ============================================================================

/**
 * 이벤트 타입
 */
export type ShieldEventType =
  | 'threat_detected'
  | 'threat_blocked'
  | 'anomaly_detected'
  | 'verification_failed'
  | 'evil_twin_detected'
  | 'mode_changed'
  | 'shield_activated'
  | 'shield_deactivated';

export interface ShieldEvent {
  type: ShieldEventType;
  timestamp: Timestamp;
  data: unknown;
}

export type ThreatCallback = (threat: Threat) => void;
export type AnomalyCallback = (anomaly: AnomalyInfo) => void;
export type EventCallback = (event: ShieldEvent) => void;
export type Unsubscribe = () => void;

// ============================================================================
// Activity Log
// ============================================================================

/**
 * 활동 로그 항목
 */
export interface ActivityEntry {
  id: string;
  timestamp: Timestamp;
  type: string;
  action: string;
  target?: string;
  result: 'success' | 'failure' | 'blocked';
  details?: Record<string, unknown>;
}

/**
 * 차단된 위협
 */
export interface BlockedThreat {
  threat: Threat;
  blockedAt: Timestamp;
  action: string;
  prevented: string; // 무엇을 막았는지
}

// ============================================================================
// AIR-POWER Integration (삼촌 연동)
// ============================================================================

/**
 * AIR-POWER 연동 설정
 */
export interface AirPowerIntegration {
  enabled: boolean;
  verifyTransmitters: boolean;
  protectDuringCharging: boolean;
  sideChannelDefense: boolean;
}

/**
 * 충전 보안 상태
 */
export interface ChargingSecurityStatus {
  transmitterVerified: boolean;
  sideChannelProtection: boolean;
  dataLeakPrevention: boolean;
  currentThreats: Threat[];
}
