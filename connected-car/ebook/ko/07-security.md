# 제7장: 커넥티드카 보안

## 사이버보안 프레임워크와 프라이버시 보호

본 장에서는 커넥티드 차량 시스템의 포괄적인 보안 요구사항을 설정하며, 글로벌 시장에서 사이버 위협, 데이터 보호 및 규제 준수를 다룹니다.

---

## 자동차 사이버보안 프레임워크

### 보안 아키텍처 개요

```typescript
// WIA 커넥티드카 보안 프레임워크
// 포괄적 사이버보안 구현

/**
 * 자동차 사이버보안 아키텍처
 * ISO/SAE 21434 및 UNECE WP.29 기반
 */
interface AutomotiveCybersecurityArchitecture {
  securityGovernance: SecurityGovernance;      // 보안 거버넌스
  threatAnalysis: TARA;                         // 위협 분석
  securityControls: SecurityControlSet;         // 보안 통제
  incidentResponse: IncidentResponsePlan;       // 사고 대응 계획
  continuousMonitoring: MonitoringSystem;       // 지속적 모니터링
}

interface SecurityGovernance {
  securityPolicy: SecurityPolicy;               // 보안 정책
  riskManagement: RiskManagementFramework;      // 위험 관리
  complianceRequirements: ComplianceRequirement[];  // 규제 준수
  securityOrganization: SecurityOrganization;   // 보안 조직
  auditProgram: AuditProgram;                   // 감사 프로그램
}

interface SecurityPolicy {
  version: string;                    // 버전
  effectiveDate: Date;                // 시행일
  approvalAuthority: string;          // 승인 권한
  scope: PolicyScope[];               // 적용 범위
  principles: SecurityPrinciple[];    // 보안 원칙
  requirements: PolicyRequirement[];  // 요구사항
  exceptions: PolicyException[];      // 예외 사항
  reviewCycle: number;                // 검토 주기 (개월)
}

type PolicyScope =
  | "VEHICLE_SYSTEMS"           // 차량 시스템
  | "BACKEND_INFRASTRUCTURE"    // 백엔드 인프라
  | "MOBILE_APPLICATIONS"       // 모바일 앱
  | "TELEMATICS_UNITS"          // 텔레매틱스 유닛
  | "THIRD_PARTY_INTEGRATIONS"; // 서드파티 연동

interface SecurityPrinciple {
  id: string;                   // 원칙 ID
  name: string;                 // 원칙명
  description: string;          // 설명
  implementation: string[];     // 구현 방법
}

/**
 * 위협 분석 및 위험 평가 (TARA)
 * ISO/SAE 21434 준수 방법론
 */
interface TARA {
  assetIdentification: Asset[];           // 자산 식별
  threatScenarios: ThreatScenario[];      // 위협 시나리오
  vulnerabilityAssessment: Vulnerability[];  // 취약점 평가
  riskEvaluation: RiskAssessment[];       // 위험 평가
  treatmentPlan: RiskTreatment[];         // 처리 계획
}

interface Asset {
  id: string;                           // 자산 ID
  name: string;                         // 자산명
  type: AssetType;                      // 자산 유형
  location: AssetLocation;              // 위치
  securityProperties: SecurityProperty[];  // 보안 속성
  owner: string;                        // 소유자
  value: AssetValue;                    // 가치
  dependencies: string[];               // 의존성
}

type AssetType =
  | "DATA"          // 데이터
  | "FUNCTION"      // 기능
  | "COMPONENT"     // 컴포넌트
  | "COMMUNICATION" // 통신
  | "INTERFACE"     // 인터페이스
  | "KEY_MATERIAL"; // 키 자료

type AssetLocation =
  | "ECU"               // 전자제어유닛
  | "TCU"               // 텔레매틱스 제어유닛
  | "GATEWAY"           // 게이트웨이
  | "CLOUD"             // 클라우드
  | "MOBILE_APP"        // 모바일 앱
  | "EXTERNAL_INTERFACE";  // 외부 인터페이스

interface SecurityProperty {
  property:
    | "CONFIDENTIALITY"   // 기밀성
    | "INTEGRITY"         // 무결성
    | "AVAILABILITY"      // 가용성
    | "AUTHENTICITY"      // 진정성
    | "NON_REPUDIATION";  // 부인 방지
  level: SecurityLevel;
  rationale: string;       // 근거
}

type SecurityLevel = "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";

interface ThreatScenario {
  id: string;                      // 시나리오 ID
  name: string;                    // 시나리오명
  category: ThreatCategory;        // 위협 카테고리
  attackVector: AttackVector;      // 공격 벡터
  targetAssets: string[];          // 대상 자산
  threatActors: ThreatActor[];     // 위협 행위자
  likelihood: LikelihoodLevel;     // 발생 가능성
  impact: ImpactAssessment;        // 영향 평가
  attackSteps: AttackStep[];       // 공격 단계
  mitigations: string[];           // 완화 조치
}

// STRIDE 모델 기반 위협 카테고리
type ThreatCategory =
  | "SPOOFING"              // 스푸핑 (위장)
  | "TAMPERING"             // 변조
  | "REPUDIATION"           // 부인
  | "INFORMATION_DISCLOSURE"  // 정보 유출
  | "DENIAL_OF_SERVICE"     // 서비스 거부
  | "ELEVATION_OF_PRIVILEGE";  // 권한 상승

type AttackVector =
  | "NETWORK"           // 네트워크
  | "PHYSICAL"          // 물리적
  | "LOCAL"             // 로컬
  | "ADJACENT_NETWORK"  // 인접 네트워크
  | "USER";             // 사용자

interface ThreatActor {
  type: ThreatActorType;      // 행위자 유형
  motivation: string;         // 동기
  capability: CapabilityLevel;  // 능력 수준
  resources: ResourceLevel;   // 자원 수준
}

type ThreatActorType =
  | "NATION_STATE"      // 국가
  | "ORGANIZED_CRIME"   // 조직 범죄
  | "HACKTIVIST"        // 핵티비스트
  | "INSIDER"           // 내부자
  | "OPPORTUNISTIC"     // 기회주의자
  | "RESEARCHER";       // 연구자

type CapabilityLevel = "BASIC" | "INTERMEDIATE" | "ADVANCED" | "EXPERT";
type ResourceLevel = "LIMITED" | "MODERATE" | "SIGNIFICANT" | "EXTENSIVE";
type LikelihoodLevel = "RARE" | "UNLIKELY" | "POSSIBLE" | "LIKELY" | "ALMOST_CERTAIN";

interface ImpactAssessment {
  safety: ImpactLevel;       // 안전 영향
  financial: ImpactLevel;    // 재무 영향
  operational: ImpactLevel;  // 운영 영향
  privacy: ImpactLevel;      // 프라이버시 영향
  reputation: ImpactLevel;   // 평판 영향
  overall: ImpactLevel;      // 전체 영향
}

type ImpactLevel = "NEGLIGIBLE" | "MINOR" | "MODERATE" | "MAJOR" | "SEVERE";

interface AttackStep {
  sequence: number;          // 순서
  action: string;            // 행동
  requiredAccess: string;    // 필요 접근권한
  tools: string[];           // 필요 도구
  detectability: DetectabilityLevel;  // 탐지 가능성
}

type DetectabilityLevel = "EASY" | "MODERATE" | "DIFFICULT" | "VERY_DIFFICULT";

interface Vulnerability {
  id: string;                    // 취약점 ID
  cveId?: string;                // CVE ID
  description: string;           // 설명
  affectedAssets: string[];      // 영향 자산
  severity: CVSSSeverity;        // CVSS 심각도
  cvssScore: number;             // CVSS 점수
  exploitability: ExploitabilityMetrics;  // 악용 가능성
  remediationStatus: RemediationStatus;   // 조치 상태
}

interface CVSSSeverity {
  baseScore: number;           // 기본 점수
  temporalScore: number;       // 시간적 점수
  environmentalScore: number;  // 환경적 점수
  overallSeverity: "NONE" | "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";
}

interface ExploitabilityMetrics {
  attackVector: string;         // 공격 벡터
  attackComplexity: string;     // 공격 복잡도
  privilegesRequired: string;   // 필요 권한
  userInteraction: string;      // 사용자 상호작용
  scope: string;                // 범위
}

type RemediationStatus =
  | "OPEN"          // 미해결
  | "IN_PROGRESS"   // 진행 중
  | "REMEDIATED"    // 조치 완료
  | "ACCEPTED"      // 수용
  | "TRANSFERRED";  // 이전

interface RiskAssessment {
  threatId: string;           // 위협 ID
  vulnerabilityId: string;    // 취약점 ID
  riskLevel: RiskLevel;       // 위험 수준
  riskScore: number;          // 위험 점수
  treatmentRequired: boolean; // 조치 필요 여부
  residualRisk: RiskLevel;    // 잔여 위험
}

type RiskLevel = "VERY_LOW" | "LOW" | "MEDIUM" | "HIGH" | "VERY_HIGH";

interface RiskTreatment {
  riskId: string;               // 위험 ID
  strategy: TreatmentStrategy;  // 전략
  controls: string[];           // 통제 수단
  owner: string;                // 담당자
  deadline: Date;               // 기한
  status: TreatmentStatus;      // 상태
  effectiveness: EffectivenessLevel;  // 효과성
}

type TreatmentStrategy =
  | "AVOID"      // 회피
  | "MITIGATE"   // 완화
  | "TRANSFER"   // 이전
  | "ACCEPT";    // 수용

type TreatmentStatus =
  | "PLANNED"      // 계획됨
  | "IN_PROGRESS"  // 진행 중
  | "IMPLEMENTED"  // 구현됨
  | "VERIFIED";    // 검증됨

type EffectivenessLevel =
  | "NOT_EFFECTIVE"       // 효과 없음
  | "PARTIALLY_EFFECTIVE" // 부분 효과
  | "EFFECTIVE"           // 효과적
  | "HIGHLY_EFFECTIVE";   // 매우 효과적

/**
 * 보안 통제 구현
 */
class VehicleSecurityController {
  private hsm: HardwareSecurityModule;           // 하드웨어 보안 모듈
  private keyManager: KeyManagementService;      // 키 관리 서비스
  private cryptoEngine: CryptoEngine;            // 암호화 엔진
  private ids: IntrusionDetectionSystem;         // 침입 탐지 시스템
  private accessControl: AccessControlManager;   // 접근 제어 관리자

  constructor(config: SecurityConfig) {
    this.hsm = new HardwareSecurityModule(config.hsmConfig);
    this.keyManager = new KeyManagementService(this.hsm, config.keyConfig);
    this.cryptoEngine = new CryptoEngine(this.hsm);
    this.ids = new IntrusionDetectionSystem(config.idsConfig);
    this.accessControl = new AccessControlManager(config.accessConfig);
  }

  /**
   * 보안 부팅 검증
   */
  async verifySecureBoot(): Promise<SecureBootResult> {
    const measurements: BootMeasurement[] = [];

    // 부트로더 검증
    const bootloaderMeasurement = await this.hsm.measureComponent("BOOTLOADER");
    measurements.push(bootloaderMeasurement);

    if (!this.verifyMeasurement(bootloaderMeasurement, "BOOTLOADER")) {
      return {
        success: false,
        failedStage: "BOOTLOADER",
        measurements
      };
    }

    // 커널 검증
    const kernelMeasurement = await this.hsm.measureComponent("KERNEL");
    measurements.push(kernelMeasurement);

    if (!this.verifyMeasurement(kernelMeasurement, "KERNEL")) {
      return {
        success: false,
        failedStage: "KERNEL",
        measurements
      };
    }

    // 중요 애플리케이션 검증
    const appMeasurement = await this.hsm.measureComponent("CRITICAL_APPS");
    measurements.push(appMeasurement);

    if (!this.verifyMeasurement(appMeasurement, "CRITICAL_APPS")) {
      return {
        success: false,
        failedStage: "CRITICAL_APPS",
        measurements
      };
    }

    // PCR 확장 (신뢰 체인 기록)
    await this.hsm.extendPCR(measurements);

    return {
      success: true,
      measurements,
      attestationReport: await this.generateAttestationReport(measurements)
    };
  }

  /**
   * 외부 엔티티 인증
   */
  async authenticate(
    request: AuthenticationRequest
  ): Promise<AuthenticationResult> {
    // 요청 무결성 검증
    if (!await this.validateRequestSignature(request)) {
      return {
        success: false,
        error: "유효하지 않은 요청 서명"
      };
    }

    // 인증서 유효성 검증
    const certValidation = await this.validateCertificate(request.certificate);
    if (!certValidation.valid) {
      return {
        success: false,
        error: `인증서 검증 실패: ${certValidation.reason}`
      };
    }

    // 챌린지-응답 검증
    const challengeResult = await this.verifyChallengeResponse(
      request.challengeResponse,
      request.certificate.publicKey
    );

    if (!challengeResult.valid) {
      return {
        success: false,
        error: "챌린지 검증 실패"
      };
    }

    // 권한 확인
    const authzResult = await this.accessControl.checkAuthorization(
      request.entityId,
      request.requestedPermissions
    );

    if (!authzResult.authorized) {
      return {
        success: false,
        error: `권한 없음: ${authzResult.reason}`
      };
    }

    // 세션 토큰 생성
    const sessionToken = await this.createSecureSession(
      request.entityId,
      authzResult.grantedPermissions
    );

    return {
      success: true,
      sessionToken,
      permissions: authzResult.grantedPermissions,
      expiresAt: sessionToken.expiresAt
    };
  }

  /**
   * 안전한 전송을 위한 데이터 암호화
   */
  async encryptData(
    plaintext: Buffer,
    recipient: string,
    context: EncryptionContext
  ): Promise<EncryptedData> {
    // 수신자 공개키 획득
    const recipientKey = await this.keyManager.getPublicKey(recipient);

    // ECDH용 임시 키 쌍 생성
    const ephemeralKeyPair = await this.cryptoEngine.generateECDHKeyPair();

    // 공유 비밀 도출
    const sharedSecret = await this.cryptoEngine.deriveSharedSecret(
      ephemeralKeyPair.privateKey,
      recipientKey
    );

    // HKDF를 사용한 암호화 키 도출
    const encryptionKey = await this.cryptoEngine.hkdf(
      sharedSecret,
      context.salt,
      context.info,
      32  // 256비트 키
    );

    // 논스 생성
    const nonce = await this.cryptoEngine.generateSecureRandom(12);

    // AES-GCM 암호화
    const ciphertext = await this.cryptoEngine.aesGcmEncrypt(
      encryptionKey,
      nonce,
      plaintext,
      context.aad  // 추가 인증 데이터
    );

    return {
      ciphertext: ciphertext.encrypted,
      authTag: ciphertext.authTag,
      nonce,
      ephemeralPublicKey: ephemeralKeyPair.publicKey,
      algorithm: "ECDH-ES+A256GCM",
      keyId: recipientKey.keyId
    };
  }

  /**
   * 펌웨어 업데이트 서명 검증
   */
  async verifyFirmwareUpdate(
    update: FirmwareUpdate
  ): Promise<FirmwareVerificationResult> {
    // 메타데이터 서명 검증
    const metadataValid = await this.cryptoEngine.verifySignature(
      update.metadata,
      update.metadataSignature,
      await this.keyManager.getOEMSigningKey()
    );

    if (!metadataValid) {
      return {
        valid: false,
        error: "유효하지 않은 메타데이터 서명"
      };
    }

    // 업데이트 패키지 해시 검증
    const packageHash = await this.cryptoEngine.sha256(update.package);
    if (!packageHash.equals(update.metadata.packageHash)) {
      return {
        valid: false,
        error: "패키지 해시 불일치"
      };
    }

    // 코드 서명 검증
    const codeSignatureValid = await this.cryptoEngine.verifyCodeSignature(
      update.package,
      update.codeSignature,
      await this.keyManager.getCodeSigningCertificate()
    );

    if (!codeSignatureValid) {
      return {
        valid: false,
        error: "유효하지 않은 코드 서명"
      };
    }

    // 버전 롤백 방지 검증
    const currentVersion = await this.getCurrentFirmwareVersion();
    if (this.compareVersions(update.metadata.version, currentVersion) <= 0) {
      return {
        valid: false,
        error: "펌웨어 버전 롤백 허용 안됨"
      };
    }

    return {
      valid: true,
      metadata: update.metadata,
      approvedForInstallation: true
    };
  }

  // 헬퍼 메서드 (스텁)
  private verifyMeasurement(measurement: BootMeasurement, component: string): boolean {
    return true;
  }
  private async generateAttestationReport(measurements: BootMeasurement[]): Promise<any> {
    return {};
  }
  private async validateRequestSignature(request: AuthenticationRequest): Promise<boolean> {
    return true;
  }
  private async validateCertificate(cert: any): Promise<{ valid: boolean; reason?: string }> {
    return { valid: true };
  }
  private async verifyChallengeResponse(response: any, publicKey: any): Promise<{ valid: boolean }> {
    return { valid: true };
  }
  private async createSecureSession(entityId: string, permissions: string[]): Promise<SessionToken> {
    return { token: "", expiresAt: new Date() };
  }
  private async getCurrentFirmwareVersion(): Promise<string> {
    return "1.0.0";
  }
  private compareVersions(a: string, b: string): number {
    return 1;
  }
}

// 인터페이스 정의
interface SecurityConfig {
  hsmConfig: any;
  keyConfig: any;
  idsConfig: any;
  accessConfig: any;
}

interface SecureBootResult {
  success: boolean;
  failedStage?: string;
  measurements: BootMeasurement[];
  attestationReport?: any;
}

interface BootMeasurement {
  component: string;
  hash: Buffer;
  timestamp: Date;
}

interface AuthenticationRequest {
  entityId: string;
  certificate: any;
  challengeResponse: any;
  requestedPermissions: string[];
}

interface AuthenticationResult {
  success: boolean;
  error?: string;
  sessionToken?: SessionToken;
  permissions?: string[];
  expiresAt?: Date;
}

interface SessionToken {
  token: string;
  expiresAt: Date;
}

interface EncryptionContext {
  salt: Buffer;
  info: Buffer;
  aad: Buffer;
}

interface EncryptedData {
  ciphertext: Buffer;
  authTag: Buffer;
  nonce: Buffer;
  ephemeralPublicKey: Buffer;
  algorithm: string;
  keyId: string;
}

interface FirmwareUpdate {
  metadata: FirmwareMetadata;
  metadataSignature: Buffer;
  package: Buffer;
  codeSignature: Buffer;
}

interface FirmwareMetadata {
  version: string;
  packageHash: Buffer;
  releaseDate: Date;
  targetHardware: string[];
}

interface FirmwareVerificationResult {
  valid: boolean;
  error?: string;
  metadata?: FirmwareMetadata;
  approvedForInstallation?: boolean;
}

// 지원 클래스 (스텁)
class HardwareSecurityModule {
  constructor(config: any) {}
  async measureComponent(component: string): Promise<BootMeasurement> {
    return { component, hash: Buffer.alloc(32), timestamp: new Date() };
  }
  async extendPCR(measurements: BootMeasurement[]): Promise<void> {}
}

class KeyManagementService {
  constructor(hsm: HardwareSecurityModule, config: any) {}
  async getPublicKey(recipient: string): Promise<any> { return {}; }
  async getOEMSigningKey(): Promise<any> { return {}; }
  async getCodeSigningCertificate(): Promise<any> { return {}; }
}

class CryptoEngine {
  constructor(hsm: HardwareSecurityModule) {}
  async generateECDHKeyPair(): Promise<any> { return {}; }
  async deriveSharedSecret(privateKey: any, publicKey: any): Promise<Buffer> { return Buffer.alloc(32); }
  async hkdf(secret: Buffer, salt: Buffer, info: Buffer, length: number): Promise<Buffer> { return Buffer.alloc(length); }
  async generateSecureRandom(length: number): Promise<Buffer> { return Buffer.alloc(length); }
  async aesGcmEncrypt(key: Buffer, nonce: Buffer, plaintext: Buffer, aad: Buffer): Promise<any> {
    return { encrypted: Buffer.alloc(0), authTag: Buffer.alloc(16) };
  }
  async verifySignature(data: any, signature: Buffer, key: any): Promise<boolean> { return true; }
  async sha256(data: Buffer): Promise<Buffer> { return Buffer.alloc(32); }
  async verifyCodeSignature(data: Buffer, signature: Buffer, cert: any): Promise<boolean> { return true; }
}

class IntrusionDetectionSystem {
  constructor(config: any) {}
}

class AccessControlManager {
  constructor(config: any) {}
  async checkAuthorization(entityId: string, permissions: string[]): Promise<any> {
    return { authorized: true, grantedPermissions: permissions };
  }
}
```

---

## 침입 탐지 시스템

```typescript
/**
 * 자동차 침입 탐지 시스템 (IDS)
 * 커넥티드 차량을 위한 다계층 위협 탐지
 */
class AutomotiveIDS {
  private networkMonitor: NetworkIDSModule;       // 네트워크 IDS 모듈
  private canBusMonitor: CANBusIDSModule;        // CAN 버스 IDS 모듈
  private applicationMonitor: ApplicationIDSModule;  // 애플리케이션 IDS
  private behavioralAnalyzer: BehavioralAnalysisEngine;  // 행위 분석 엔진
  private alertManager: AlertManager;             // 경보 관리자
  private mlEngine: MLDetectionEngine;           // ML 탐지 엔진

  constructor(config: IDSConfig) {
    this.networkMonitor = new NetworkIDSModule(config.network);
    this.canBusMonitor = new CANBusIDSModule(config.canBus);
    this.applicationMonitor = new ApplicationIDSModule(config.application);
    this.behavioralAnalyzer = new BehavioralAnalysisEngine(config.behavioral);
    this.alertManager = new AlertManager(config.alerting);
    this.mlEngine = new MLDetectionEngine(config.ml);
  }

  /**
   * IDS 모니터링 시작
   */
  async start(): Promise<void> {
    // 모든 모니터링 모듈 시작
    await Promise.all([
      this.networkMonitor.start(),
      this.canBusMonitor.start(),
      this.applicationMonitor.start(),
      this.behavioralAnalyzer.start()
    ]);

    // 이벤트 상관관계 분석 설정
    this.setupEventCorrelation();

    console.log("자동차 IDS 시작됨");
  }

  /**
   * 네트워크 트래픽 처리
   */
  processNetworkTraffic(packet: NetworkPacket): DetectionResult[] {
    const results: DetectionResult[] = [];

    // 시그니처 기반 탐지
    const signatureHits = this.networkMonitor.matchSignatures(packet);
    results.push(...signatureHits);

    // 프로토콜 이상 탐지
    const protocolAnomalies = this.networkMonitor.detectProtocolAnomalies(packet);
    results.push(...protocolAnomalies);

    // ML 기반 탐지
    const mlResults = this.mlEngine.analyzeNetworkTraffic(packet);
    results.push(...mlResults);

    // 탐지에 대한 경보 생성
    for (const result of results) {
      if (result.confidence >= result.threshold) {
        this.alertManager.createAlert(result);
      }
    }

    return results;
  }

  /**
   * CAN 버스 메시지 처리
   */
  processCANMessage(message: CANMessage): DetectionResult[] {
    const results: DetectionResult[] = [];

    // 메시지 빈도 분석
    const frequencyAnomaly = this.canBusMonitor.checkMessageFrequency(message);
    if (frequencyAnomaly) {
      results.push(frequencyAnomaly);
    }

    // 메시지 내용 검증
    const contentAnomaly = this.canBusMonitor.validateMessageContent(message);
    if (contentAnomaly) {
      results.push(contentAnomaly);
    }

    // 인젝션 탐지
    const injectionDetection = this.canBusMonitor.detectInjection(message);
    if (injectionDetection) {
      results.push(injectionDetection);
    }

    // 타이밍 분석
    const timingAnomaly = this.canBusMonitor.analyzeMessageTiming(message);
    if (timingAnomaly) {
      results.push(timingAnomaly);
    }

    return results;
  }

  private setupEventCorrelation(): void {
    // 여러 소스의 이벤트 상관관계 분석
    this.networkMonitor.on("detection", (result) => {
      this.correlateEvent(result, "NETWORK");
    });

    this.canBusMonitor.on("detection", (result) => {
      this.correlateEvent(result, "CAN_BUS");
    });

    this.applicationMonitor.on("detection", (result) => {
      this.correlateEvent(result, "APPLICATION");
    });
  }

  private correlateEvent(result: DetectionResult, source: string): void {
    // 이벤트 상관관계 로직
    this.behavioralAnalyzer.addEvent({
      ...result,
      source,
      timestamp: new Date()
    });
  }
}

interface IDSConfig {
  network: NetworkIDSConfig;
  canBus: CANBusIDSConfig;
  application: ApplicationIDSConfig;
  behavioral: BehavioralConfig;
  alerting: AlertConfig;
  ml: MLConfig;
}

interface CANBusIDSConfig {
  buses: string[];                      // 모니터링할 CAN 버스
  baselineProfile: CANBaselineProfile;  // 기준 프로파일
  injectionDetection: InjectionDetectionConfig;  // 인젝션 탐지 설정
}

interface CANBaselineProfile {
  messageFrequencies: Map<number, FrequencyRange>;  // 메시지 빈도
  expectedPayloads: Map<number, PayloadPattern>;    // 예상 페이로드
  messageTiming: Map<number, TimingProfile>;        // 메시지 타이밍
}

interface FrequencyRange {
  min: number;     // 초당 최소 메시지
  max: number;     // 초당 최대 메시지
  mean: number;    // 평균
  stdDev: number;  // 표준편차
}

interface PayloadPattern {
  mask: Buffer;                // 마스크
  expectedValues: Buffer[];    // 예상 값
  valueRanges?: ValueRange[];  // 값 범위
}

interface ValueRange {
  startBit: number;   // 시작 비트
  length: number;     // 길이
  min: number;        // 최소값
  max: number;        // 최대값
}

interface TimingProfile {
  expectedInterval: number;  // 예상 간격 (밀리초)
  jitterTolerance: number;   // 지터 허용치
  burstThreshold: number;    // 버스트 임계값
}

interface DetectionResult {
  id: string;                  // 탐지 ID
  type: DetectionType;         // 탐지 유형
  severity: AlertSeverity;     // 심각도
  confidence: number;          // 신뢰도
  threshold: number;           // 임계값
  description: string;         // 설명
  evidence: Evidence;          // 증거
  recommendation: string;      // 권장 조치
  timestamp: Date;             // 타임스탬프
}

type DetectionType =
  | "SIGNATURE_MATCH"     // 시그니처 일치
  | "PROTOCOL_ANOMALY"    // 프로토콜 이상
  | "FREQUENCY_ANOMALY"   // 빈도 이상
  | "CONTENT_ANOMALY"     // 내용 이상
  | "TIMING_ANOMALY"      // 타이밍 이상
  | "INJECTION_ATTEMPT"   // 인젝션 시도
  | "BEHAVIORAL_ANOMALY"  // 행위 이상
  | "ML_DETECTION";       // ML 탐지

type AlertSeverity = "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";

interface Evidence {
  rawData: Buffer;                   // 원시 데이터
  context: Record<string, any>;      // 컨텍스트
  relatedEvents: string[];           // 관련 이벤트
}

interface AlertConfig {
  channels: AlertChannel[];           // 알림 채널
  escalationPolicy: EscalationPolicy; // 에스컬레이션 정책
}

interface AlertChannel {
  type: "LOG" | "SIEM" | "CLOUD" | "LOCAL_DISPLAY";
  config: any;
}

interface EscalationPolicy {
  levels: EscalationLevel[];
}

interface EscalationLevel {
  severity: AlertSeverity;
  notifyAfter: number;   // 알림 시간 (초)
  contacts: string[];    // 연락처
}

interface MLConfig {
  models: MLModel[];
  inferenceMode: "LOCAL" | "EDGE" | "CLOUD";
}

interface MLModel {
  name: string;
  type: "ANOMALY_DETECTION" | "CLASSIFICATION";
  path: string;
  threshold: number;
}
```

---

## 프라이버시 보호 프레임워크

```typescript
/**
 * 프라이버시 보호 프레임워크
 * GDPR, CCPA 및 자동차 특화 프라이버시 요구사항
 */
interface PrivacyFramework {
  dataClassification: DataClassificationScheme;     // 데이터 분류
  consentManagement: ConsentManagementSystem;       // 동의 관리
  dataMinimization: DataMinimizationPolicy;         // 데이터 최소화
  anonymization: AnonymizationEngine;               // 익명화
  rightsFulfillment: DataSubjectRights;             // 정보주체 권리
  dataRetention: RetentionPolicy;                   // 보존 정책
  crossBorderTransfer: TransferMechanisms;          // 국경간 이전
}

/**
 * 차량 데이터 분류 체계
 */
interface DataCategory {
  id: string;
  name: string;
  description: string;
  sensitivityLevel: SensitivityLevel;
  legalBasis: LegalBasis[];
  retentionPeriod: RetentionPeriod;
  examples: string[];
}

type SensitivityLevel =
  | "PUBLIC"            // 공개
  | "INTERNAL"          // 내부
  | "CONFIDENTIAL"      // 기밀
  | "RESTRICTED"        // 제한
  | "HIGHLY_RESTRICTED";  // 고도 제한

type LegalBasis =
  | "CONSENT"            // 동의
  | "CONTRACT"           // 계약
  | "LEGAL_OBLIGATION"   // 법적 의무
  | "VITAL_INTEREST"     // 중대한 이익
  | "PUBLIC_TASK"        // 공공 업무
  | "LEGITIMATE_INTEREST";  // 정당한 이익

// 차량 데이터 분류 예시
const vehicleDataClassification: DataCategory[] = [
  {
    id: "PII_DIRECT",
    name: "직접 개인식별정보",
    description: "개인을 직접 식별할 수 있는 데이터",
    sensitivityLevel: "HIGHLY_RESTRICTED",
    legalBasis: ["CONSENT", "CONTRACT"],
    retentionPeriod: { duration: 36, unit: "MONTHS", basis: "LAST_ACTIVITY" },
    examples: ["운전자 이름", "이메일", "전화번호", "차량번호판"]
  },
  {
    id: "LOCATION_PRECISE",
    name: "정밀 위치 데이터",
    description: "GPS 좌표 및 정밀 위치 이력",
    sensitivityLevel: "RESTRICTED",
    legalBasis: ["CONSENT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 12, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["GPS 좌표", "주행 이력", "주차 위치"]
  },
  {
    id: "DRIVING_BEHAVIOR",
    name: "운전 행동 데이터",
    description: "운전 패턴과 행동을 나타내는 데이터",
    sensitivityLevel: "CONFIDENTIAL",
    legalBasis: ["CONSENT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 24, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["속도 패턴", "제동 행동", "가속 프로파일"]
  },
  {
    id: "VEHICLE_TELEMETRY",
    name: "차량 텔레메트리",
    description: "개인식별자 없는 기술적 차량 데이터",
    sensitivityLevel: "INTERNAL",
    legalBasis: ["CONTRACT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 60, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["배터리 상태", "타이어 압력", "엔진 진단"]
  },
  {
    id: "BIOMETRIC",
    name: "생체 데이터",
    description: "운전자 모니터링의 생체 식별자",
    sensitivityLevel: "HIGHLY_RESTRICTED",
    legalBasis: ["CONSENT"],
    retentionPeriod: { duration: 6, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["얼굴 이미지", "음성 지문", "운전자 주의 데이터"]
  }
];

interface RetentionPeriod {
  duration: number;
  unit: "DAYS" | "MONTHS" | "YEARS";
  basis: "COLLECTION" | "LAST_ACTIVITY" | "CONTRACT_END" | "EVENT";
}

/**
 * 동의 관리 서비스
 */
class ConsentManagementService {
  private consentStore: ConsentStore;
  private purposeRegistry: PurposeRegistry;

  constructor(
    consentStore: ConsentStore,
    purposeRegistry: PurposeRegistry
  ) {
    this.consentStore = consentStore;
    this.purposeRegistry = purposeRegistry;
  }

  /**
   * 사용자 동의 기록
   */
  async recordConsent(
    userId: string,
    vehicleId: string,
    consents: ConsentGrant[]
  ): Promise<ConsentRecord> {
    // 목적 검증
    for (const consent of consents) {
      const purpose = await this.purposeRegistry.getPurpose(consent.purposeId);
      if (!purpose) {
        throw new Error(`유효하지 않은 목적: ${consent.purposeId}`);
      }
    }

    // 동의 기록 생성
    const record: ConsentRecord = {
      id: crypto.randomUUID(),
      userId,
      vehicleId,
      consents,
      timestamp: new Date(),
      source: "USER_INTERFACE",
      version: "1.0",
      ipAddress: consents[0].metadata?.ipAddress,
      userAgent: consents[0].metadata?.userAgent
    };

    // 동의 저장
    await this.consentStore.store(record);

    // 활성 동의 상태 업데이트
    await this.updateActiveConsents(userId, vehicleId);

    return record;
  }

  /**
   * 특정 목적에 대한 동의 여부 확인
   */
  async hasConsent(
    userId: string,
    vehicleId: string,
    purposeId: string
  ): Promise<ConsentCheckResult> {
    const activeConsents = await this.consentStore.getActiveConsents(
      userId,
      vehicleId
    );

    const consent = activeConsents.find(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );

    if (!consent) {
      return { hasConsent: false, reason: "NO_CONSENT_RECORD" };
    }

    const grant = consent.consents.find(g => g.purposeId === purposeId);

    // 만료 확인
    if (grant?.expiresAt && new Date() > grant.expiresAt) {
      return { hasConsent: false, reason: "CONSENT_EXPIRED" };
    }

    return {
      hasConsent: true,
      consentId: consent.id,
      grantedAt: consent.timestamp,
      expiresAt: grant?.expiresAt
    };
  }

  /**
   * 동의 철회
   */
  async withdrawConsent(
    userId: string,
    vehicleId: string,
    purposeIds: string[]
  ): Promise<void> {
    const record: ConsentRecord = {
      id: crypto.randomUUID(),
      userId,
      vehicleId,
      consents: purposeIds.map(purposeId => ({
        purposeId,
        granted: false,
        withdrawnAt: new Date()
      })),
      timestamp: new Date(),
      source: "USER_INTERFACE",
      version: "1.0"
    };

    await this.consentStore.store(record);
    await this.updateActiveConsents(userId, vehicleId);

    // 필요시 데이터 삭제 워크플로우 트리거
    await this.handleConsentWithdrawal(userId, vehicleId, purposeIds);
  }

  /**
   * 사용자 동의 기본 설정 조회
   */
  async getConsentPreferences(
    userId: string,
    vehicleId: string
  ): Promise<ConsentPreferences> {
    const purposes = await this.purposeRegistry.getAllPurposes();
    const activeConsents = await this.consentStore.getActiveConsents(
      userId,
      vehicleId
    );

    return {
      userId,
      vehicleId,
      purposes: purposes.map(purpose => ({
        purposeId: purpose.id,
        name: purpose.name,
        description: purpose.description,
        required: purpose.required,
        granted: this.isConsentGranted(activeConsents, purpose.id),
        grantedAt: this.getConsentGrantDate(activeConsents, purpose.id),
        dataCategories: purpose.dataCategories
      })),
      lastUpdated: this.getLastUpdateDate(activeConsents)
    };
  }

  private async updateActiveConsents(userId: string, vehicleId: string): Promise<void> {
    // 구현
  }

  private async handleConsentWithdrawal(
    userId: string,
    vehicleId: string,
    purposeIds: string[]
  ): Promise<void> {
    // 철회된 목적에 대한 데이터 삭제 트리거
  }

  private isConsentGranted(consents: ConsentRecord[], purposeId: string): boolean {
    return consents.some(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );
  }

  private getConsentGrantDate(consents: ConsentRecord[], purposeId: string): Date | undefined {
    const record = consents.find(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );
    return record?.timestamp;
  }

  private getLastUpdateDate(consents: ConsentRecord[]): Date | undefined {
    if (consents.length === 0) return undefined;
    return consents.reduce((latest, c) =>
      c.timestamp > latest ? c.timestamp : latest,
      consents[0].timestamp
    );
  }
}

interface ConsentGrant {
  purposeId: string;
  granted: boolean;
  expiresAt?: Date;
  withdrawnAt?: Date;
  metadata?: Record<string, any>;
}

interface ConsentRecord {
  id: string;
  userId: string;
  vehicleId: string;
  consents: ConsentGrant[];
  timestamp: Date;
  source: ConsentSource;
  version: string;
  ipAddress?: string;
  userAgent?: string;
}

type ConsentSource =
  | "USER_INTERFACE"   // 사용자 인터페이스
  | "MOBILE_APP"       // 모바일 앱
  | "VEHICLE_HMI"      // 차량 HMI
  | "API"              // API
  | "PAPER_FORM";      // 종이 양식

interface ConsentCheckResult {
  hasConsent: boolean;
  reason?: string;
  consentId?: string;
  grantedAt?: Date;
  expiresAt?: Date;
}

interface ConsentPreferences {
  userId: string;
  vehicleId: string;
  purposes: PurposePreference[];
  lastUpdated?: Date;
}

interface PurposePreference {
  purposeId: string;
  name: string;
  description: string;
  required: boolean;
  granted: boolean;
  grantedAt?: Date;
  dataCategories: string[];
}
```

---

## 규제 준수

```typescript
/**
 * 자동차 규제 준수 프레임워크
 * UNECE WP.29, ISO/SAE 21434, 지역별 요구사항
 */
interface RegulatoryComplianceFramework {
  uneceWP29: UNECEWP29Compliance;
  isoSae21434: ISO21434Compliance;
  regionalCompliance: RegionalRequirement[];
}

/**
 * UN 규정 155 - 사이버보안 준수
 */
interface UNR155CybersecurityCompliance {
  csmsRequirements: CSMSRequirement[];     // CSMS 요구사항
  vehicleTypeApproval: VTARequirements;    // 차량형식 승인
  threatMitigation: ThreatMitigationEvidence[];  // 위협 완화 증거
}

interface CSMSRequirement {
  id: string;
  category: CSMSCategory;
  requirement: string;
  evidence: string[];
  status: ComplianceStatus;
}

type CSMSCategory =
  | "RISK_MANAGEMENT"      // 위험 관리
  | "VEHICLE_DEVELOPMENT"  // 차량 개발
  | "PRODUCTION"           // 생산
  | "POST_PRODUCTION"      // 생산 후
  | "INCIDENT_RESPONSE"    // 사고 대응
  | "SUPPLY_CHAIN";        // 공급망

type ComplianceStatus =
  | "COMPLIANT"            // 준수
  | "PARTIALLY_COMPLIANT"  // 부분 준수
  | "NON_COMPLIANT"        // 미준수
  | "NOT_APPLICABLE";      // 해당 없음

// UN R155 요구사항 예시
const unr155Requirements: CSMSRequirement[] = [
  {
    id: "UNR155-7.2.2.2-a",
    category: "RISK_MANAGEMENT",
    requirement: "차량 유형에 대한 사이버 위험 식별 프로세스",
    evidence: [
      "TARA 문서",
      "위험 평가 보고서",
      "위협 인텔리전스 통합"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.2-b",
    category: "RISK_MANAGEMENT",
    requirement: "식별된 위험 평가 프로세스",
    evidence: [
      "위험 점수 방법론",
      "영향 평가 기준",
      "위험 우선순위 프레임워크"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.3",
    category: "VEHICLE_DEVELOPMENT",
    requirement: "차량 개발에서 보안 설계 원칙",
    evidence: [
      "보안 요구사항 명세서",
      "보안 코딩 표준",
      "보안 테스트 결과"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.4",
    category: "SUPPLY_CHAIN",
    requirement: "공급망 사이버보안 위험 관리",
    evidence: [
      "공급업체 보안 평가",
      "계약의 보안 요구사항",
      "컴포넌트 보안 검증"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.5",
    category: "INCIDENT_RESPONSE",
    requirement: "사이버 공격 탐지 및 대응",
    evidence: [
      "IDS/SOC 역량",
      "사고 대응 절차",
      "사고 추적 기록"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.6",
    category: "POST_PRODUCTION",
    requirement: "새로운 위협 모니터링 및 대응",
    evidence: [
      "위협 인텔리전스 프로그램",
      "취약점 모니터링",
      "보안 패치 관리"
    ],
    status: "COMPLIANT"
  }
];

/**
 * UN 규정 156 - 소프트웨어 업데이트 준수
 */
interface UNR156SoftwareUpdateCompliance {
  sumsRequirements: SUMSRequirement[];     // SUMS 요구사항
  rxswinManagement: RXSWINManagement;      // RXSWIN 관리
  updateProcessVerification: UpdateProcessEvidence[];  // 업데이트 프로세스
}

interface RXSWINManagement {
  currentRxswin: string;              // 현재 RXSWIN
  managementProcess: string;          // 관리 프로세스
  updateHistory: RXSWINUpdate[];      // 업데이트 이력
}

interface RXSWINUpdate {
  previousRxswin: string;
  newRxswin: string;
  updateDate: Date;
  reason: string;
  approvalReference: string;
}

/**
 * 지역별 규제 요구사항
 */
const regionalRequirements: RegionalRequirement[] = [
  {
    region: "EUROPEAN_UNION",
    regulations: [
      {
        name: "GDPR",
        requirements: [
          "차량 데이터에 대한 데이터 보호 영향 평가",
          "커넥티드카 기능의 프라이버시 설계 원칙",
          "정보주체 권리 구현",
          "72시간 내 데이터 침해 통지"
        ]
      },
      {
        name: "UNECE WP.29 (UN R155/R156)",
        requirements: [
          "차량 유형별 CSMS 인증",
          "OTA 업데이트용 SUMS 인증",
          "사이버보안 형식 승인"
        ]
      },
      {
        name: "EU 사이버 복원력법",
        requirements: [
          "커넥티드 제품 CE 마킹",
          "취약점 처리 프로세스",
          "보안 업데이트 의무"
        ]
      }
    ],
    effectiveDate: new Date("2024-07-01"),
    applicability: ["2024년 7월부터 신규 차량 유형", "2026년 7월부터 모든 차량"]
  },
  {
    region: "UNITED_STATES",
    regulations: [
      {
        name: "CCPA/CPRA",
        requirements: [
          "차량 데이터에 대한 소비자 프라이버시 권리",
          "데이터 판매 옵트아웃",
          "프라이버시 정책 공개"
        ]
      },
      {
        name: "NHTSA 사이버보안 모범 사례",
        requirements: [
          "계층적 보안 접근법",
          "사고 대응 역량",
          "자체 감사 프로그램"
        ]
      }
    ],
    effectiveDate: new Date("2023-01-01"),
    applicability: ["미국 시장에서 판매되는 모든 차량"]
  },
  {
    region: "KOREA",
    regulations: [
      {
        name: "개인정보보호법",
        requirements: [
          "개인정보 처리 동의",
          "개인정보 영향평가",
          "정보주체 권리 보장"
        ]
      },
      {
        name: "자동차관리법",
        requirements: [
          "자율주행차 안전기준",
          "커넥티드카 보안 요구사항"
        ]
      }
    ],
    effectiveDate: new Date("2023-01-01"),
    applicability: ["대한민국 시장에서 판매되는 모든 차량"]
  },
  {
    region: "CHINA",
    regulations: [
      {
        name: "GB/T 자동차 표준",
        requirements: [
          "데이터 현지화 요구사항",
          "국경간 데이터 이전 제한",
          "데이터 수출 보안 평가"
        ]
      },
      {
        name: "사이버보안법 / PIPL",
        requirements: [
          "중요 정보 인프라 보호",
          "개인정보 보호",
          "데이터 분류 및 등급화"
        ]
      }
    ],
    effectiveDate: new Date("2023-01-01"),
    applicability: ["중국 시장에서 판매되는 모든 차량"]
  }
];

interface RegionalRequirement {
  region: string;
  regulations: RegulationDetail[];
  effectiveDate: Date;
  applicability: string[];
}

interface RegulationDetail {
  name: string;
  requirements: string[];
}
```

---

## 요약

| 보안 영역 | 핵심 통제 | 표준 |
|-----------|----------|------|
| **보안 부팅** | TPM, 코드 서명, 측정 부팅 | ISO 21434 |
| **통신** | TLS 1.3, mTLS, 암호화 | AUTOSAR SecOC |
| **인증** | PKI, 챌린지-응답, MFA | IEEE 1609.2 |
| **침입 탐지** | CAN IDS, 네트워크 모니터링 | UN R155 |
| **프라이버시** | 동의 관리, 익명화 | GDPR, CCPA |
| **규제 준수** | CSMS, SUMS, 형식 승인 | UN R155/R156 |

---

**다음 장:** [제8장: 구현](./08-implementation.md) - 배포, 테스트 및 인증.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
