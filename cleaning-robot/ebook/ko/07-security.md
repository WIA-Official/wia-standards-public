# 제7장: 청소 로봇 보안

## 안전, 개인정보 보호 및 사이버보안 프레임워크

### 7.1 보안 아키텍처 개요

WIA-CLEANING-ROBOT 표준은 주거 및 상업 환경에서 운영되는 자율 청소 로봇을 위한 물리적 안전, 데이터 프라이버시 및 사이버보안을 다루는 포괄적인 보안 조치를 정의합니다.

```typescript
// 보안 아키텍처 정의
interface CleaningRobotSecurityArchitecture {
  version: '1.0.0';

  securityDomains: {
    physicalSafety: {
      description: '인간-로봇 상호작용 안전';
      standards: ['ISO 13482', 'IEC 62443', 'UL 3300'];
      scope: ['충돌 방지', '비상 정지', '안전 운영 모드'];
    };
    cybersecurity: {
      description: '사이버 위협으로부터 보호';
      standards: ['IEC 62443', 'NIST 사이버보안 프레임워크', 'ETSI EN 303 645'];
      scope: ['인증', '암호화', '접근 제어', '보안 업데이트'];
    };
    dataPrivacy: {
      description: '개인 데이터 보호';
      regulations: ['GDPR', 'CCPA', 'PIPL'];
      scope: ['지도 데이터', '사용 패턴', '카메라 이미지', '음성 녹음'];
    };
    operationalSecurity: {
      description: '안전한 로봇 운영';
      scope: ['플릿 관리', '원격 접속', '감사 로깅'];
    };
  };

  threatModel: {
    threatActors: [
      '악의적 해커',
      '경쟁업체',
      '불만을 품은 직원',
      '국가 행위자',
      '기회주의적 공격자'
    ];
    attackVectors: [
      '네트워크 침입',
      '물리적 접근',
      '공급망 침해',
      '사회공학',
      '악성 펌웨어'
    ];
    assetCategories: [
      '로봇 하드웨어',
      '펌웨어 및 소프트웨어',
      '사용자 데이터',
      '지도 데이터',
      'API 자격증명',
      '클라우드 인프라'
    ];
  };
}

// 보안 관리자 구현
class RobotSecurityManager {
  private authenticationService: AuthenticationService;
  private encryptionService: EncryptionService;
  private auditLogger: AuditLogger;
  private intrusionDetector: IntrusionDetectionSystem;

  async initializeSecurity(robot: CleaningRobot): Promise<void> {
    // 보안 부팅 초기화
    await this.verifySecureBoot(robot);

    // 인증 설정
    await this.setupAuthentication(robot);

    // 암호화 구성
    await this.configureEncryption(robot);

    // 침입 탐지 시작
    await this.startIntrusionDetection(robot);

    // 감사 로깅 초기화
    await this.initializeAuditLogging(robot);
  }

  private async verifySecureBoot(robot: CleaningRobot): Promise<void> {
    // 펌웨어 서명 검증
    const firmwareVerification = await this.verifyFirmwareSignature(robot);
    if (!firmwareVerification.valid) {
      throw new SecurityError('펌웨어 서명 검증 실패');
    }

    // 부팅 체인 검증
    const bootChainVerification = await this.verifyBootChain(robot);
    if (!bootChainVerification.valid) {
      throw new SecurityError('부팅 체인 검증 실패');
    }

    this.auditLogger.log({
      event: 'SECURE_BOOT_VERIFIED',
      robotId: robot.id,
      details: {
        firmwareVersion: robot.firmwareVersion,
        bootChainValid: true
      }
    });
  }
}
```

### 7.2 물리적 안전 시스템

```typescript
// 물리적 안전 프로토콜
interface PhysicalSafetyProtocol {
  safetyLevels: {
    normal: NormalOperationSafety;
    cautious: CautiousOperationSafety;
    emergency: EmergencyProtocol;
  };

  humanDetection: HumanDetectionConfig;
  obstacleAvoidance: ObstacleAvoidanceConfig;
  emergencyStop: EmergencyStopConfig;
}

interface NormalOperationSafety {
  maxSpeed: number;               // m/s
  maxAcceleration: number;        // m/s²
  minObstacleDistance: number;    // 미터
  sensorCheckFrequency: number;   // Hz
}

interface CautiousOperationSafety {
  maxSpeed: number;               // 감속
  reducedSuction: boolean;
  enhancedDetection: boolean;
  humanPresenceResponse: 'SLOW' | 'PAUSE' | 'AVOID';
}

interface EmergencyProtocol {
  triggers: EmergencyTrigger[];
  response: EmergencyResponse;
  recoveryProcedure: RecoveryProcedure;
}

// 안전 시스템 구현
class PhysicalSafetySystem {
  private config: PhysicalSafetyProtocol;
  private sensorManager: SensorManager;
  private motorController: MotorController;
  private aiDetector: AIDetectionSystem;

  async monitorSafety(): Promise<void> {
    // 지속적인 안전 모니터링 루프
    while (true) {
      const sensorData = await this.sensorManager.readAllSensors();

      // 비상 상태 확인
      const emergencyCondition = this.checkEmergencyConditions(sensorData);
      if (emergencyCondition) {
        await this.triggerEmergencyStop(emergencyCondition);
        continue;
      }

      // 사람 감지
      const humanDetection = await this.detectHumans(sensorData);
      if (humanDetection.detected) {
        await this.handleHumanPresence(humanDetection);
      }

      // 장애물 근접도 확인
      const obstacleProximity = this.checkObstacleProximity(sensorData);
      if (obstacleProximity.tooClose) {
        await this.handleObstacleProximity(obstacleProximity);
      }

      // 안전 상태 업데이트
      await this.updateSafetyState(sensorData);

      await sleep(10);  // 100Hz 안전 루프
    }
  }

  private checkEmergencyConditions(
    sensorData: SensorData
  ): EmergencyCondition | null {
    // 낭떠러지 센서 확인
    for (const cliff of sensorData.cliffSensors) {
      if (cliff.triggered) {
        return {
          type: 'CLIFF_DETECTED',
          sensor: cliff.position,
          severity: 'CRITICAL'
        };
      }
    }

    // 휠 멈춤 확인
    for (const wheel of sensorData.wheelEncoders) {
      if (wheel.stalled && this.motorController.isPowered(wheel.id)) {
        return {
          type: 'WHEEL_STALL',
          wheel: wheel.id,
          severity: 'HIGH'
        };
      }
    }

    // 범퍼 센서로 강한 충돌 확인
    for (const bumper of sensorData.bumperSensors) {
      if (bumper.force > this.config.emergencyStop.maxBumperForce) {
        return {
          type: 'HARD_COLLISION',
          sensor: bumper.position,
          force: bumper.force,
          severity: 'CRITICAL'
        };
      }
    }

    // 배터리 위험 수준 확인
    if (sensorData.battery.voltage < this.config.emergencyStop.criticalVoltage) {
      return {
        type: 'CRITICAL_BATTERY',
        voltage: sensorData.battery.voltage,
        severity: 'HIGH'
      };
    }

    // 모터 온도 확인
    for (const motor of sensorData.motorTemperatures) {
      if (motor.temperature > this.config.emergencyStop.maxMotorTemp) {
        return {
          type: 'MOTOR_OVERHEAT',
          motor: motor.id,
          temperature: motor.temperature,
          severity: 'HIGH'
        };
      }
    }

    return null;
  }

  private async triggerEmergencyStop(
    condition: EmergencyCondition
  ): Promise<void> {
    // 즉시 모터 정지
    await this.motorController.emergencyStop();

    // 모든 청소 기능 중지
    await this.cleaningController.emergencyStop();

    // 비상 이벤트 로깅
    this.auditLogger.logEmergency({
      condition,
      timestamp: new Date(),
      robotState: await this.getRobotState()
    });

    // 사용자 알림
    await this.notificationService.sendAlert({
      type: 'EMERGENCY_STOP',
      severity: condition.severity,
      message: this.getEmergencyMessage(condition),
      robotId: this.robotId
    });

    // 안전 모드 진입
    await this.enterSafeMode(condition);
  }

  private async detectHumans(sensorData: SensorData): Promise<HumanDetection> {
    // 카메라 기반 AI 감지 사용
    if (sensorData.cameras) {
      const aiResult = await this.aiDetector.detectHumans(sensorData.cameras);
      if (aiResult.detected) {
        return {
          detected: true,
          confidence: aiResult.confidence,
          distance: aiResult.estimatedDistance,
          direction: aiResult.direction,
          type: this.classifyHuman(aiResult)
        };
      }
    }

    // LiDAR로 다리 감지
    if (sensorData.lidar) {
      const legDetection = this.detectLegs(sensorData.lidar);
      if (legDetection.detected) {
        return {
          detected: true,
          confidence: legDetection.confidence,
          distance: legDetection.distance,
          direction: legDetection.direction,
          type: 'UNKNOWN'
        };
      }
    }

    return { detected: false };
  }

  private async handleHumanPresence(detection: HumanDetection): Promise<void> {
    const response = this.config.safetyLevels.cautious.humanPresenceResponse;

    switch (response) {
      case 'SLOW':
        await this.motorController.setSpeedLimit(
          this.config.safetyLevels.cautious.maxSpeed
        );
        break;

      case 'PAUSE':
        await this.motorController.pause();
        await this.waitForHumanClear(detection);
        await this.motorController.resume();
        break;

      case 'AVOID':
        await this.pathPlanner.avoidArea(detection.direction, detection.distance);
        break;
    }

    // 어린이와 반려동물 특별 처리
    if (detection.type === 'CHILD' || detection.type === 'PET') {
      await this.enterCautiousMode();
      await this.increaseDetectionSensitivity();
    }
  }

  private classifyHuman(aiResult: AIDetectionResult): HumanType {
    if (aiResult.classification === 'CHILD' || aiResult.heightEstimate < 1.2) {
      return 'CHILD';
    }
    if (aiResult.classification === 'PET') {
      return 'PET';
    }
    return 'ADULT';
  }
}

// 비상 대응 시스템
class EmergencyResponseSystem {
  private safetySystem: PhysicalSafetySystem;
  private communicator: RobotCommunicator;

  async handleEmergency(emergency: EmergencyCondition): Promise<void> {
    // 1단계: 즉각 대응
    await this.immediateResponse(emergency);

    // 2단계: 상황 평가
    const assessment = await this.assessSituation(emergency);

    // 3단계: 복구 또는 에스컬레이션
    if (assessment.canRecover) {
      await this.initiateRecovery(assessment);
    } else {
      await this.escalateEmergency(emergency, assessment);
    }
  }

  private async immediateResponse(emergency: EmergencyCondition): Promise<void> {
    switch (emergency.type) {
      case 'CLIFF_DETECTED':
        // 약간 후진 후 정지
        await this.motorController.reverse(0.05);  // 5cm
        await this.motorController.stop();
        break;

      case 'HARD_COLLISION':
        // 완전 정지
        await this.motorController.emergencyStop();
        break;

      case 'WHEEL_STALL':
        // 손상 방지를 위해 모터 전원 차단
        await this.motorController.disableMotor(emergency.wheel);
        break;

      case 'MOTOR_OVERHEAT':
        // 정지 및 냉각
        await this.motorController.stop();
        await this.cleaningController.stop();
        break;

      case 'CRITICAL_BATTERY':
        // 청소 중지, 도킹 시도
        await this.cleaningController.stop();
        if (await this.canReachDock()) {
          await this.returnToDock();
        } else {
          await this.motorController.stop();
        }
        break;
    }
  }

  private async assessSituation(
    emergency: EmergencyCondition
  ): Promise<SituationAssessment> {
    // 센서 상태 확인
    const sensorStatus = await this.safetySystem.checkSensorHealth();

    // 환경 확인
    const environment = await this.safetySystem.assessEnvironment();

    // 복구 가능성 결정
    const canRecover = this.determineRecoverability(emergency, sensorStatus, environment);

    return {
      emergency,
      sensorStatus,
      environment,
      canRecover,
      recoverySteps: canRecover ? this.planRecovery(emergency) : null,
      timestamp: new Date()
    };
  }
}
```

### 7.3 사이버보안 프레임워크

```typescript
// 사이버보안 프레임워크
interface CybersecurityFramework {
  authentication: AuthenticationConfig;
  encryption: EncryptionConfig;
  accessControl: AccessControlConfig;
  networkSecurity: NetworkSecurityConfig;
  firmwareSecurity: FirmwareSecurityConfig;
}

// 인증 서비스
class AuthenticationService {
  private tokenService: TokenService;
  private certificateManager: CertificateManager;
  private mfaService: MFAService;

  async authenticateUser(credentials: UserCredentials): Promise<AuthResult> {
    // 자격증명 검증
    const user = await this.validateCredentials(credentials);
    if (!user) {
      await this.logFailedAttempt(credentials.identifier);
      return { success: false, reason: '유효하지 않은 자격증명' };
    }

    // MFA 활성화된 경우 확인
    if (user.mfaEnabled) {
      const mfaResult = await this.mfaService.verify(user.id, credentials.mfaToken);
      if (!mfaResult.valid) {
        return { success: false, reason: 'MFA 검증 실패' };
      }
    }

    // 토큰 생성
    const tokens = await this.generateTokens(user);

    // 성공적인 인증 로깅
    await this.logSuccessfulAuth(user.id);

    return {
      success: true,
      accessToken: tokens.accessToken,
      refreshToken: tokens.refreshToken,
      expiresIn: tokens.expiresIn
    };
  }

  async authenticateRobot(robotCredentials: RobotCredentials): Promise<AuthResult> {
    // 로봇 인증서 검증
    const certValid = await this.certificateManager.verifyCertificate(
      robotCredentials.certificate
    );
    if (!certValid) {
      return { success: false, reason: '유효하지 않은 인증서' };
    }

    // 로봇 신원 검증
    const robot = await this.verifyRobotIdentity(robotCredentials);
    if (!robot) {
      return { success: false, reason: '등록되지 않은 로봇' };
    }

    // 로봇 토큰 생성
    const tokens = await this.generateRobotTokens(robot);

    return {
      success: true,
      accessToken: tokens.accessToken,
      refreshToken: tokens.refreshToken,
      expiresIn: tokens.expiresIn
    };
  }

  private async generateTokens(user: User): Promise<TokenPair> {
    const accessToken = await this.tokenService.createToken({
      type: 'access',
      subject: user.id,
      claims: {
        email: user.email,
        roles: user.roles,
        permissions: user.permissions
      },
      expiresIn: '15m'
    });

    const refreshToken = await this.tokenService.createToken({
      type: 'refresh',
      subject: user.id,
      expiresIn: '7d'
    });

    return {
      accessToken,
      refreshToken,
      expiresIn: 900  // 15분
    };
  }

  async validateToken(token: string): Promise<TokenValidation> {
    try {
      const decoded = await this.tokenService.verifyToken(token);

      // 토큰 폐기 확인
      if (await this.isTokenRevoked(decoded.jti)) {
        return { valid: false, reason: '토큰이 폐기됨' };
      }

      return {
        valid: true,
        claims: decoded
      };
    } catch (error) {
      return { valid: false, reason: error.message };
    }
  }
}

// 암호화 서비스
class EncryptionService {
  private keyManager: KeyManagementService;

  async encryptData(data: Buffer, context: EncryptionContext): Promise<EncryptedData> {
    // 적절한 키 가져오기
    const key = await this.keyManager.getEncryptionKey(context.keyId);

    // IV 생성
    const iv = crypto.randomBytes(16);

    // AES-256-GCM으로 암호화
    const cipher = crypto.createCipheriv('aes-256-gcm', key.data, iv);
    const encrypted = Buffer.concat([
      cipher.update(data),
      cipher.final()
    ]);
    const authTag = cipher.getAuthTag();

    return {
      ciphertext: encrypted,
      iv,
      authTag,
      keyId: context.keyId,
      algorithm: 'AES-256-GCM'
    };
  }

  async decryptData(encrypted: EncryptedData): Promise<Buffer> {
    // 키 가져오기
    const key = await this.keyManager.getEncryptionKey(encrypted.keyId);

    // 복호화
    const decipher = crypto.createDecipheriv(
      'aes-256-gcm',
      key.data,
      encrypted.iv
    );
    decipher.setAuthTag(encrypted.authTag);

    return Buffer.concat([
      decipher.update(encrypted.ciphertext),
      decipher.final()
    ]);
  }

  async encryptMapData(map: RobotMap): Promise<EncryptedMap> {
    // 지도 직렬화
    const serialized = Buffer.from(JSON.stringify(map));

    // 지도 전용 키로 암호화
    const encrypted = await this.encryptData(serialized, {
      keyId: `map-${map.robotId}`,
      purpose: 'MAP_STORAGE'
    });

    return {
      mapId: map.id,
      robotId: map.robotId,
      encrypted,
      metadata: {
        encryptedAt: new Date(),
        algorithm: encrypted.algorithm
      }
    };
  }
}

// 접근 제어 서비스
class AccessControlService {
  private policyEngine: PolicyEngine;
  private roleManager: RoleManager;

  async checkPermission(
    principal: Principal,
    resource: Resource,
    action: string
  ): Promise<PermissionResult> {
    // 주체의 역할 가져오기
    const roles = await this.roleManager.getRoles(principal);

    // 정책 평가
    const policyResult = await this.policyEngine.evaluate({
      principal,
      roles,
      resource,
      action,
      context: this.buildContext(principal, resource)
    });

    // 접근 시도 로깅
    await this.logAccessAttempt({
      principal,
      resource,
      action,
      result: policyResult.allowed,
      reason: policyResult.reason
    });

    return policyResult;
  }

  async grantAccess(
    robotId: string,
    userId: string,
    permissions: RobotPermission[]
  ): Promise<void> {
    // 사용자가 로봇 소유 또는 관리 권한 확인
    const hasAuthority = await this.verifyAuthority(userId, robotId);
    if (!hasAuthority) {
      throw new AccessDeniedError('사용자에게 접근 권한 부여 권한 없음');
    }

    // 접근 권한 생성
    await this.createAccessGrant({
      robotId,
      userId,
      permissions,
      grantedBy: userId,
      grantedAt: new Date(),
      expiresAt: null  // 기본적으로 만료 없음
    });
  }

  async defineRobotPermissions(): Promise<void> {
    // 표준 로봇 권한 정의
    const permissions: RobotPermission[] = [
      {
        name: 'robot:view',
        description: '로봇 상태 및 정보 조회',
        category: 'READ'
      },
      {
        name: 'robot:control',
        description: '로봇 제어 (시작, 정지, 도킹)',
        category: 'CONTROL'
      },
      {
        name: 'robot:schedule',
        description: '청소 일정 관리',
        category: 'MANAGE'
      },
      {
        name: 'robot:map:view',
        description: '로봇 지도 조회',
        category: 'READ'
      },
      {
        name: 'robot:map:edit',
        description: '지도, 방, 구역 편집',
        category: 'MANAGE'
      },
      {
        name: 'robot:settings',
        description: '로봇 설정 변경',
        category: 'MANAGE'
      },
      {
        name: 'robot:admin',
        description: '완전한 관리자 접근',
        category: 'ADMIN'
      }
    ];

    await this.policyEngine.registerPermissions(permissions);
  }
}

// 펌웨어 보안
class FirmwareSecurityService {
  private signingService: CodeSigningService;
  private updateService: SecureUpdateService;

  async verifyFirmware(firmware: FirmwarePackage): Promise<VerificationResult> {
    // 디지털 서명 검증
    const signatureValid = await this.signingService.verify(
      firmware.binary,
      firmware.signature,
      firmware.signingCertificate
    );

    if (!signatureValid) {
      return {
        valid: false,
        reason: '유효하지 않은 펌웨어 서명'
      };
    }

    // 인증서 체인 검증
    const certValid = await this.signingService.verifyCertificateChain(
      firmware.signingCertificate
    );

    if (!certValid) {
      return {
        valid: false,
        reason: '유효하지 않은 인증서 체인'
      };
    }

    // 펌웨어 해시 검증
    const hash = crypto.createHash('sha256').update(firmware.binary).digest('hex');
    if (hash !== firmware.expectedHash) {
      return {
        valid: false,
        reason: '펌웨어 해시 불일치'
      };
    }

    // 버전 확인 (다운그레이드 방지)
    const currentVersion = await this.getCurrentFirmwareVersion();
    if (this.compareVersions(firmware.version, currentVersion) <= 0) {
      return {
        valid: false,
        reason: '펌웨어 다운그레이드 허용되지 않음'
      };
    }

    return { valid: true };
  }

  async applySecureUpdate(
    robotId: string,
    firmware: FirmwarePackage
  ): Promise<UpdateResult> {
    // 펌웨어 검증
    const verification = await this.verifyFirmware(firmware);
    if (!verification.valid) {
      throw new SecurityError(`펌웨어 검증 실패: ${verification.reason}`);
    }

    // 업데이트 트랜잭션 생성
    const transaction = await this.updateService.createTransaction(robotId, firmware);

    try {
      // 현재 펌웨어 백업
      await this.updateService.backupCurrentFirmware(robotId, transaction.id);

      // 업데이트 적용
      await this.updateService.applyUpdate(robotId, firmware, transaction.id);

      // 설치 검증
      const installValid = await this.updateService.verifyInstallation(
        robotId,
        firmware.version
      );

      if (!installValid) {
        // 롤백
        await this.updateService.rollback(robotId, transaction.id);
        throw new UpdateError('펌웨어 설치 검증 실패');
      }

      // 트랜잭션 커밋
      await this.updateService.commitTransaction(transaction.id);

      return {
        success: true,
        newVersion: firmware.version,
        transactionId: transaction.id
      };

    } catch (error) {
      // 오류 발생 시 롤백
      await this.updateService.rollback(robotId, transaction.id);
      throw error;
    }
  }
}
```

### 7.4 데이터 프라이버시 보호

```typescript
// 데이터 프라이버시 프레임워크
interface DataPrivacyFramework {
  dataCategories: DataCategory[];
  retentionPolicies: RetentionPolicy[];
  anonymizationRules: AnonymizationRule[];
  consentManagement: ConsentManagementConfig;
  dataSubjectRights: DataSubjectRightsConfig;
}

// 프라이버시 관리자 구현
class PrivacyManager {
  private consentService: ConsentService;
  private anonymizer: DataAnonymizer;
  private retentionManager: RetentionManager;
  private auditLogger: PrivacyAuditLogger;

  async processDataWithPrivacy(
    data: RobotData,
    purpose: DataPurpose
  ): Promise<ProcessedData> {
    // 동의 확인
    const consent = await this.consentService.checkConsent(
      data.userId,
      purpose
    );

    if (!consent.granted) {
      throw new ConsentError(`동의가 없는 목적: ${purpose}`);
    }

    // 데이터 최소화 적용
    const minimizedData = this.applyDataMinimization(data, purpose);

    // 필요시 익명화 적용
    const processedData = consent.requiresAnonymization
      ? await this.anonymizer.anonymize(minimizedData)
      : minimizedData;

    // 처리 로깅
    await this.auditLogger.logProcessing({
      userId: data.userId,
      purpose,
      dataCategories: this.identifyDataCategories(data),
      timestamp: new Date(),
      consentId: consent.id
    });

    return processedData;
  }

  private applyDataMinimization(
    data: RobotData,
    purpose: DataPurpose
  ): RobotData {
    const allowedFields = this.getFieldsForPurpose(purpose);

    // 필요한 필드만 필터링
    const minimized: any = {};
    for (const field of allowedFields) {
      if (data[field] !== undefined) {
        minimized[field] = data[field];
      }
    }

    return minimized;
  }

  async handleDataSubjectRequest(
    request: DataSubjectRequest
  ): Promise<RequestResult> {
    switch (request.type) {
      case 'ACCESS':
        return this.handleAccessRequest(request);

      case 'DELETION':
        return this.handleDeletionRequest(request);

      case 'PORTABILITY':
        return this.handlePortabilityRequest(request);

      case 'RECTIFICATION':
        return this.handleRectificationRequest(request);

      case 'RESTRICTION':
        return this.handleRestrictionRequest(request);

      default:
        throw new Error(`알 수 없는 요청 유형: ${request.type}`);
    }
  }

  private async handleAccessRequest(
    request: DataSubjectRequest
  ): Promise<AccessRequestResult> {
    const userId = request.userId;

    // 모든 사용자 데이터 수집
    const userData = await this.collectUserData(userId);

    // 사람이 읽을 수 있는 보고서 생성
    const report = await this.generateDataReport(userData);

    // 요청 로깅
    await this.auditLogger.logDataSubjectRequest(request, 'COMPLETED');

    return {
      success: true,
      data: userData,
      report,
      generatedAt: new Date()
    };
  }

  private async handleDeletionRequest(
    request: DataSubjectRequest
  ): Promise<DeletionRequestResult> {
    const userId = request.userId;

    // 삭제할 데이터 식별
    const dataInventory = await this.inventoryUserData(userId);

    // 보존 요구사항 확인
    const retentionBlocks = await this.checkRetentionRequirements(dataInventory);

    // 차단되지 않은 데이터 삭제
    const deletionResults = [];
    for (const dataItem of dataInventory) {
      if (!retentionBlocks.has(dataItem.id)) {
        const result = await this.deleteData(dataItem);
        deletionResults.push(result);
      }
    }

    // 삭제 로깅
    await this.auditLogger.logDataSubjectRequest(request, 'COMPLETED', {
      itemsDeleted: deletionResults.filter(r => r.success).length,
      itemsRetained: retentionBlocks.size
    });

    return {
      success: true,
      itemsDeleted: deletionResults.filter(r => r.success).length,
      itemsRetained: Array.from(retentionBlocks.values()),
      retentionReasons: this.explainRetention(retentionBlocks)
    };
  }

  async collectUserData(userId: string): Promise<UserDataCollection> {
    const collection: UserDataCollection = {
      userId,
      collectedAt: new Date(),
      categories: {}
    };

    // 로봇 데이터 수집
    const robots = await this.robotService.getUserRobots(userId);
    collection.categories.robots = robots.map(r => ({
      id: r.id,
      nickname: r.nickname,
      registeredAt: r.registration.registeredAt
    }));

    // 지도 데이터 수집
    const maps = [];
    for (const robot of robots) {
      const robotMaps = await this.mapService.getMaps(robot.id);
      maps.push(...robotMaps.map(m => ({
        id: m.id,
        robotId: robot.id,
        name: m.name,
        roomCount: m.rooms.length,
        createdAt: m.metadata.createdAt
      })));
    }
    collection.categories.maps = maps;

    // 청소 이력 수집
    const history = [];
    for (const robot of robots) {
      const robotHistory = await this.historyService.getHistory(robot.id, { limit: 100 });
      history.push(...robotHistory.items.map(h => ({
        sessionId: h.id,
        robotId: robot.id,
        startedAt: h.startedAt,
        duration: h.summary.duration,
        area: h.summary.area
      })));
    }
    collection.categories.cleaningHistory = history;

    // 일정 수집
    const schedules = [];
    for (const robot of robots) {
      const robotSchedules = await this.scheduleService.getSchedules(robot.id);
      schedules.push(...robotSchedules);
    }
    collection.categories.schedules = schedules;

    return collection;
  }
}

// 지도 데이터 익명화
class MapDataAnonymizer {
  async anonymizeMap(map: RobotMap): Promise<AnonymizedMap> {
    // 정확한 위치 데이터 제거
    const anonymizedMap = this.removeLocationData(map);

    // 방 이름 일반화
    anonymizedMap.rooms = map.rooms.map(room => ({
      ...room,
      name: `Room_${room.id.substring(0, 4)}`,
      furniture: []  // 가구 세부정보 제거
    }));

    // 사용자 지정 구역 이름 제거
    anonymizedMap.zones = map.zones.map(zone => ({
      ...zone,
      name: `Zone_${zone.type}`
    }));

    // 랜드마크 제거
    anonymizedMap.landmarks = [];

    return anonymizedMap;
  }

  private removeLocationData(map: RobotMap): RobotMap {
    // 지오로케이션 방지를 위한 좌표 변환 적용
    const transform = this.generateRandomTransform();

    return {
      ...map,
      metadata: {
        ...map.metadata,
        coordinateSystem: {
          origin: { x: 0, y: 0 },  // 원점 재설정
          rotation: 0,
          scale: map.metadata.coordinateSystem.scale
        }
      },
      rooms: map.rooms.map(room => ({
        ...room,
        geometry: {
          ...room.geometry,
          boundary: this.transformPolygon(room.geometry.boundary, transform),
          centroid: this.transformPoint(room.geometry.centroid, transform)
        }
      }))
    };
  }
}
```

### 7.5 네트워크 보안

```typescript
// 네트워크 보안 구현
class NetworkSecurityService {
  private firewallManager: FirewallManager;
  private tlsConfig: TLSConfiguration;
  private intrusionDetector: NetworkIntrusionDetector;

  async configureNetworkSecurity(robot: CleaningRobot): Promise<void> {
    // TLS 구성
    await this.configureTLS(robot);

    // 방화벽 규칙 구성
    await this.configureFirewall(robot);

    // 네트워크 모니터링 시작
    await this.startNetworkMonitoring(robot);
  }

  private async configureTLS(robot: CleaningRobot): Promise<void> {
    const tlsConfig: TLSConfiguration = {
      minVersion: 'TLSv1.3',
      cipherSuites: [
        'TLS_AES_256_GCM_SHA384',
        'TLS_CHACHA20_POLY1305_SHA256',
        'TLS_AES_128_GCM_SHA256'
      ],
      certificateValidation: true,
      pinning: {
        enabled: true,
        pins: await this.getCertificatePins()
      }
    };

    await this.applyTLSConfig(robot.id, tlsConfig);
  }

  private async configureFirewall(robot: CleaningRobot): Promise<void> {
    const rules: FirewallRule[] = [
      // 클라우드로 아웃바운드 HTTPS 허용
      {
        direction: 'OUTBOUND',
        protocol: 'TCP',
        port: 443,
        destination: 'api.wia-robot.io',
        action: 'ALLOW'
      },
      // 아웃바운드 MQTTS 허용
      {
        direction: 'OUTBOUND',
        protocol: 'TCP',
        port: 8883,
        destination: 'mqtt.wia-robot.io',
        action: 'ALLOW'
      },
      // 로컬 API 허용
      {
        direction: 'INBOUND',
        protocol: 'TCP',
        port: 8080,
        source: 'LOCAL_NETWORK',
        action: 'ALLOW'
      },
      // mDNS 허용
      {
        direction: 'BOTH',
        protocol: 'UDP',
        port: 5353,
        action: 'ALLOW'
      },
      // 기타 모든 인바운드 차단
      {
        direction: 'INBOUND',
        protocol: 'ALL',
        action: 'DENY'
      }
    ];

    await this.firewallManager.applyRules(robot.id, rules);
  }

  async detectNetworkThreats(
    robot: CleaningRobot
  ): Promise<ThreatDetectionResult[]> {
    const threats: ThreatDetectionResult[] = [];

    // 불법 액세스 포인트 확인
    const wifiNetworks = await this.scanWifiNetworks(robot);
    const rogueAPs = this.detectRogueAccessPoints(wifiNetworks);
    if (rogueAPs.length > 0) {
      threats.push({
        type: 'ROGUE_ACCESS_POINT',
        severity: 'HIGH',
        details: rogueAPs
      });
    }

    // 의심스러운 연결 확인
    const connections = await this.getActiveConnections(robot);
    const suspicious = this.analyzeSuspiciousConnections(connections);
    if (suspicious.length > 0) {
      threats.push({
        type: 'SUSPICIOUS_CONNECTION',
        severity: 'MEDIUM',
        details: suspicious
      });
    }

    // 포트 스캔 확인
    const portScanDetected = await this.intrusionDetector.detectPortScan(robot.id);
    if (portScanDetected) {
      threats.push({
        type: 'PORT_SCAN_DETECTED',
        severity: 'MEDIUM',
        details: portScanDetected
      });
    }

    return threats;
  }
}
```

### 7.6 보안 모범 사례 요약

```yaml
청소 로봇 보안 모범 사례:

  물리적 안전:
    - 인간 감지 시스템 필수 구현
    - 다단계 비상 정지 메커니즘
    - 어린이 및 반려동물 특별 보호 모드
    - 정기적인 안전 센서 교정

  사이버보안:
    - 모든 통신에 TLS 1.3 적용
    - 다단계 인증 지원
    - 정기적인 펌웨어 보안 업데이트
    - 침입 탐지 시스템 운영

  데이터 프라이버시:
    - GDPR/CCPA 준수 데이터 처리
    - 지도 데이터 암호화 저장
    - 사용자 동의 기반 데이터 수집
    - 데이터 주체 권리 존중

  네트워크 보안:
    - 최소 권한 방화벽 정책
    - 인증서 피닝 적용
    - 정기적인 보안 감사
    - 이상 행위 모니터링

  弘益人間 보안 철학:
    - 사용자 안전이 최우선
    - 프라이버시 존중
    - 투명한 보안 정책
    - 지속적인 보안 개선
```

---

**WIA-CLEANING-ROBOT 보안**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
