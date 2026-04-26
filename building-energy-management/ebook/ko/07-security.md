# 제7장: 보안 프레임워크

## 서론

빌딩 에너지 관리 시스템(BEMS)은 IT(Information Technology)와 OT(Operational Technology)가 융합된 환경에서 운영됩니다. 이러한 융합 환경은 새로운 보안 위협에 노출되며, 사이버 공격이 물리적 피해로 이어질 수 있습니다. 본 장에서는 WIA-BEMS의 보안 프레임워크, 접근 제어, 데이터 보호, 그리고 사고 대응 절차를 상세히 다룹니다.

---

## 7.1 IT/OT 보안 아키텍처

### 7.1.1 심층 방어 전략

```typescript
// 심층 방어 보안 아키텍처
interface DefenseInDepthArchitecture {
  layers: {
    perimeter: {
      name: '경계 보안';
      description: '외부 위협으로부터 첫 번째 방어선';
      controls: [
        '차세대 방화벽 (NGFW)',
        '웹 애플리케이션 방화벽 (WAF)',
        'DDoS 보호',
        'VPN 게이트웨이'
      ];
      placement: '네트워크 경계';
    };

    network: {
      name: '네트워크 보안';
      description: '내부 네트워크 세분화 및 트래픽 제어';
      controls: [
        '네트워크 세그먼테이션',
        'VLAN 분리',
        '침입 탐지/방지 시스템 (IDS/IPS)',
        '네트워크 접근 제어 (NAC)'
      ];
      zones: ['Enterprise Zone', 'DMZ', 'Control Zone', 'Field Zone'];
    };

    application: {
      name: '애플리케이션 보안';
      description: '애플리케이션 레벨 보호';
      controls: [
        'API 보안',
        '입력 검증',
        '세션 관리',
        '보안 코딩 표준'
      ];
    };

    data: {
      name: '데이터 보안';
      description: '데이터 기밀성 및 무결성 보호';
      controls: [
        '암호화 (전송 중/저장 시)',
        '데이터 마스킹',
        '접근 제어',
        '감사 로깅'
      ];
    };

    endpoint: {
      name: '엔드포인트 보안';
      description: '장치 레벨 보안';
      controls: [
        '엔드포인트 보호 플랫폼 (EPP)',
        '패치 관리',
        '기기 인증',
        'USB 제어'
      ];
    };

    physical: {
      name: '물리 보안';
      description: '물리적 접근 제어';
      controls: [
        '출입 통제',
        'CCTV',
        '환경 모니터링',
        '장비 잠금'
      ];
    };
  };
}

// 보안 구역 정의
interface SecurityZones {
  enterpriseZone: {
    name: '엔터프라이즈 존';
    securityLevel: 3;
    systems: [
      '사용자 워크스테이션',
      '오피스 애플리케이션',
      '이메일 서버',
      '인트라넷'
    ];
    accessPolicy: '일반 기업 보안 정책';
    firewallRules: 'Enterprise ↔ DMZ 허용';
  };

  dmz: {
    name: 'DMZ';
    securityLevel: 4;
    systems: [
      'API 게이트웨이',
      '웹 서버',
      '외부 연동 서버',
      'VPN 집중 장치'
    ];
    accessPolicy: '외부 접근 제한, 내부 연결 제어';
    firewallRules: 'Internet ↔ DMZ 제한적 허용, DMZ ↔ Control Zone 제한적 허용';
  };

  controlZone: {
    name: '제어 존';
    securityLevel: 5;
    systems: [
      'BEMS 서버',
      '분석 서버',
      '히스토리안',
      'SCADA/HMI'
    ];
    accessPolicy: '승인된 사용자만 접근, MFA 필수';
    firewallRules: 'DMZ → Control Zone (특정 포트만), Control Zone → Field Zone (제어 프로토콜)';
  };

  fieldZone: {
    name: '필드 존';
    securityLevel: 6;
    systems: [
      'DDC 컨트롤러',
      'PLC',
      '센서',
      '액추에이터',
      '게이트웨이'
    ];
    accessPolicy: '물리적 접근 제한, 원격 접근 최소화';
    firewallRules: 'Control Zone → Field Zone 허용, Field Zone → Internet 차단';
  };
}

// 네트워크 세그먼테이션 구현
class NetworkSegmentation {
  private firewallManager: FirewallManager;
  private vlanManager: VLANManager;

  async implementSegmentation(config: SegmentationConfig): Promise<void> {
    // 1. VLAN 구성
    await this.configureVLANs(config.vlans);

    // 2. 방화벽 규칙 설정
    await this.configureFirewallRules(config.firewallRules);

    // 3. 접근 제어 목록 설정
    await this.configureACLs(config.acls);

    // 4. 검증
    await this.validateSegmentation();
  }

  private async configureVLANs(vlans: VLANConfig[]): Promise<void> {
    for (const vlan of vlans) {
      await this.vlanManager.createVLAN({
        id: vlan.id,
        name: vlan.name,
        subnet: vlan.subnet,
        gateway: vlan.gateway,
        dhcpEnabled: vlan.dhcpEnabled,
        securityZone: vlan.securityZone
      });
    }

    // VLAN 간 라우팅 구성 (필요한 경우만)
    await this.vlanManager.configureInterVLANRouting(vlans);
  }

  private async configureFirewallRules(rules: FirewallRule[]): Promise<void> {
    // 기본 거부 정책
    await this.firewallManager.setDefaultPolicy('deny');

    // 명시적 허용 규칙 추가
    for (const rule of rules) {
      await this.firewallManager.addRule({
        name: rule.name,
        source: rule.source,
        destination: rule.destination,
        service: rule.service,
        action: rule.action,
        logging: rule.logging,
        schedule: rule.schedule
      });
    }

    // 중요 규칙 예시
    const criticalRules: FirewallRule[] = [
      // DMZ → Control Zone: BEMS API만 허용
      {
        name: 'DMZ_to_Control_BEMS_API',
        source: 'dmz_subnet',
        destination: 'control_zone_subnet',
        service: 'tcp/443',
        action: 'allow',
        logging: true
      },
      // Control Zone → Field Zone: BACnet, Modbus 허용
      {
        name: 'Control_to_Field_BACnet',
        source: 'control_zone_subnet',
        destination: 'field_zone_subnet',
        service: 'udp/47808',
        action: 'allow',
        logging: true
      },
      {
        name: 'Control_to_Field_Modbus',
        source: 'control_zone_subnet',
        destination: 'field_zone_subnet',
        service: 'tcp/502',
        action: 'allow',
        logging: true
      },
      // Field Zone → Internet: 차단
      {
        name: 'Block_Field_to_Internet',
        source: 'field_zone_subnet',
        destination: 'any',
        service: 'any',
        action: 'deny',
        logging: true
      }
    ];

    for (const rule of criticalRules) {
      await this.firewallManager.addRule(rule);
    }
  }

  async validateSegmentation(): Promise<SegmentationValidationResult> {
    const results: ValidationResult[] = [];

    // 1. 구역 간 연결 테스트
    const connectivityTests = await this.testInterZoneConnectivity();
    results.push(...connectivityTests);

    // 2. 차단 규칙 확인
    const blockTests = await this.testBlockedConnections();
    results.push(...blockTests);

    // 3. 의도치 않은 경로 탐지
    const pathAnalysis = await this.analyzeNetworkPaths();
    results.push(...pathAnalysis);

    return {
      passed: results.every(r => r.passed),
      results,
      recommendations: this.generateRecommendations(results)
    };
  }
}
```

### 7.1.2 제로 트러스트 아키텍처

```typescript
// 제로 트러스트 보안 모델
interface ZeroTrustArchitecture {
  principles: {
    neverTrust: '아무도, 어떤 것도 기본적으로 신뢰하지 않음';
    alwaysVerify: '모든 접근 요청을 매번 검증';
    leastPrivilege: '최소 권한 원칙 적용';
    assumeBreach: '침해가 이미 발생했다고 가정';
  };

  components: {
    identityProvider: {
      role: '신원 확인 및 인증';
      technologies: ['Active Directory', 'Azure AD', 'Okta', 'LDAP'];
      features: ['MFA', 'SSO', 'Conditional Access', 'Risk-based Authentication'];
    };

    policyEngine: {
      role: '접근 정책 평가 및 결정';
      inputs: ['사용자 ID', '기기 상태', '위치', '시간', '행위 패턴'];
      outputs: ['허용', '거부', '추가 인증 요구', '세션 종료'];
    };

    policyEnforcement: {
      role: '정책 적용 및 실행';
      points: ['API Gateway', 'Reverse Proxy', 'Network Gateway', 'Application'];
    };

    continuousMonitoring: {
      role: '지속적인 모니터링 및 분석';
      capabilities: ['세션 모니터링', '이상 탐지', '위험 점수 계산', '자동 대응'];
    };
  };
}

// 제로 트러스트 정책 엔진
class ZeroTrustPolicyEngine {
  private identityService: IdentityService;
  private deviceService: DeviceService;
  private riskEngine: RiskAssessmentEngine;
  private policyStore: PolicyStore;

  async evaluateAccess(request: AccessRequest): Promise<AccessDecision> {
    // 1. 신원 검증
    const identityVerification = await this.verifyIdentity(request.credentials);
    if (!identityVerification.valid) {
      return { decision: 'deny', reason: 'Identity verification failed' };
    }

    // 2. 기기 상태 검증
    const deviceHealth = await this.checkDeviceHealth(request.deviceId);
    if (!deviceHealth.compliant) {
      return {
        decision: 'require_remediation',
        reason: 'Device not compliant',
        remediationSteps: deviceHealth.issues
      };
    }

    // 3. 위험 평가
    const riskScore = await this.assessRisk(request);
    if (riskScore > this.config.riskThreshold) {
      return {
        decision: 'require_additional_auth',
        reason: 'High risk score',
        requiredFactors: this.determineRequiredFactors(riskScore)
      };
    }

    // 4. 정책 평가
    const policyEvaluation = await this.evaluatePolicies(request);
    if (!policyEvaluation.allowed) {
      return {
        decision: 'deny',
        reason: policyEvaluation.reason,
        violatedPolicies: policyEvaluation.violatedPolicies
      };
    }

    // 5. 권한 부여
    const permissions = await this.determinePermissions(request);

    return {
      decision: 'allow',
      sessionToken: await this.createSession(request, permissions),
      permissions,
      expiresAt: new Date(Date.now() + this.config.sessionDuration)
    };
  }

  private async assessRisk(request: AccessRequest): Promise<number> {
    const factors: RiskFactor[] = [];

    // 위치 기반 위험
    const locationRisk = await this.riskEngine.assessLocation(request.location);
    factors.push(locationRisk);

    // 시간 기반 위험
    const timeRisk = this.riskEngine.assessTime(request.timestamp);
    factors.push(timeRisk);

    // 행위 기반 위험
    const behaviorRisk = await this.riskEngine.assessBehavior(
      request.userId,
      request.requestedResource
    );
    factors.push(behaviorRisk);

    // 기기 기반 위험
    const deviceRisk = await this.riskEngine.assessDevice(request.deviceId);
    factors.push(deviceRisk);

    // 리소스 민감도
    const resourceSensitivity = await this.getResourceSensitivity(request.requestedResource);
    factors.push({ factor: 'resource_sensitivity', score: resourceSensitivity });

    // 종합 위험 점수 계산
    return this.riskEngine.calculateCompositeScore(factors);
  }

  private async evaluatePolicies(request: AccessRequest): Promise<PolicyEvaluation> {
    const applicablePolicies = await this.policyStore.getPolicies({
      userId: request.userId,
      resource: request.requestedResource,
      action: request.action
    });

    const violations: PolicyViolation[] = [];

    for (const policy of applicablePolicies) {
      const evaluation = await this.evaluatePolicy(policy, request);
      if (!evaluation.satisfied) {
        violations.push({
          policyId: policy.id,
          policyName: policy.name,
          reason: evaluation.reason
        });
      }
    }

    return {
      allowed: violations.length === 0,
      violatedPolicies: violations,
      reason: violations.length > 0 ?
        `정책 위반: ${violations.map(v => v.policyName).join(', ')}` : undefined
    };
  }
}

// 위험 평가 엔진
class RiskAssessmentEngine {
  private mlModel: RiskMLModel;
  private historyStore: AccessHistoryStore;

  async assessBehavior(userId: string, resource: string): Promise<RiskFactor> {
    // 사용자의 과거 접근 패턴 분석
    const accessHistory = await this.historyStore.getUserHistory(userId);

    // ML 모델로 현재 접근의 이상 정도 평가
    const features = this.extractFeatures(userId, resource, accessHistory);
    const anomalyScore = await this.mlModel.predict(features);

    // 이상 점수를 위험 점수로 변환
    const riskScore = this.normalizeScore(anomalyScore);

    return {
      factor: 'behavior',
      score: riskScore,
      details: {
        typicalAccessTime: accessHistory.typicalAccessTime,
        currentTime: new Date().getHours(),
        frequentResources: accessHistory.frequentResources,
        requestedResource: resource
      }
    };
  }

  assessLocation(location: GeoLocation): RiskFactor {
    // 알려진 위치인지 확인
    const isKnownLocation = this.checkKnownLocation(location);

    // 국가 위험도
    const countryRisk = this.getCountryRiskScore(location.country);

    // 불가능한 이동 (Impossible Travel) 감지
    const impossibleTravel = this.checkImpossibleTravel(location);

    let score = 0;
    if (!isKnownLocation) score += 0.3;
    if (countryRisk > 0.5) score += countryRisk * 0.4;
    if (impossibleTravel) score += 0.5;

    return {
      factor: 'location',
      score: Math.min(score, 1),
      details: {
        isKnownLocation,
        countryRisk,
        impossibleTravel
      }
    };
  }

  calculateCompositeScore(factors: RiskFactor[]): number {
    // 가중 평균 계산
    const weights: Record<string, number> = {
      behavior: 0.3,
      location: 0.25,
      time: 0.1,
      device: 0.2,
      resource_sensitivity: 0.15
    };

    let totalScore = 0;
    let totalWeight = 0;

    for (const factor of factors) {
      const weight = weights[factor.factor] || 0.1;
      totalScore += factor.score * weight;
      totalWeight += weight;
    }

    return totalScore / totalWeight;
  }
}
```

---

## 7.2 접근 제어

### 7.2.1 다중 인증 (MFA)

```typescript
// 다중 인증 시스템
interface MFAConfiguration {
  factors: {
    knowledge: {
      name: '지식 기반';
      methods: ['비밀번호', 'PIN', '보안 질문'];
      strength: 'medium';
    };
    possession: {
      name: '소유 기반';
      methods: ['OTP 앱', 'SMS 인증', '하드웨어 토큰', '스마트 카드'];
      strength: 'high';
    };
    inherence: {
      name: '생체 기반';
      methods: ['지문', '얼굴', '홍채', '음성'];
      strength: 'very_high';
    };
    location: {
      name: '위치 기반';
      methods: ['GPS', 'IP 지오로케이션', '네트워크 위치'];
      strength: 'low';
    };
  };

  policies: {
    standard: {
      requiredFactors: 2;
      allowedCombinations: [
        ['password', 'totp'],
        ['password', 'sms'],
        ['password', 'hardware_token']
      ];
    };
    high_security: {
      requiredFactors: 3;
      allowedCombinations: [
        ['password', 'totp', 'fingerprint'],
        ['smart_card', 'pin', 'face']
      ];
    };
    critical_systems: {
      requiredFactors: 3;
      mandatoryFactors: ['hardware_token', 'biometric'];
      supervisorApproval: true;
    };
  };
}

// MFA 서비스 구현
class MFAService {
  private factorProviders: Map<string, FactorProvider>;
  private sessionStore: SessionStore;
  private config: MFAConfiguration;

  constructor(config: MFAConfiguration) {
    this.config = config;
    this.initializeProviders();
  }

  async initiateAuthentication(
    userId: string,
    requiredLevel: 'standard' | 'high_security' | 'critical_systems'
  ): Promise<AuthenticationSession> {
    const policy = this.config.policies[requiredLevel];
    const userFactors = await this.getUserAvailableFactors(userId);

    // 사용 가능한 인증 조합 결정
    const availableCombinations = policy.allowedCombinations.filter(combo =>
      combo.every(factor => userFactors.includes(factor))
    );

    if (availableCombinations.length === 0) {
      throw new AuthenticationError('사용 가능한 인증 방법이 없습니다');
    }

    // 세션 생성
    const session = await this.sessionStore.createSession({
      userId,
      requiredLevel,
      requiredFactors: policy.requiredFactors,
      completedFactors: [],
      availableCombinations,
      expiresAt: new Date(Date.now() + 5 * 60 * 1000) // 5분
    });

    return {
      sessionId: session.id,
      availableMethods: availableCombinations[0], // 기본 조합 제안
      requiredCount: policy.requiredFactors
    };
  }

  async verifyFactor(
    sessionId: string,
    factorType: string,
    credential: FactorCredential
  ): Promise<FactorVerificationResult> {
    const session = await this.sessionStore.getSession(sessionId);

    if (!session || session.expiresAt < new Date()) {
      throw new AuthenticationError('세션이 만료되었습니다');
    }

    // 이미 완료된 인증 요소인지 확인
    if (session.completedFactors.includes(factorType)) {
      return { success: false, reason: '이미 완료된 인증 요소입니다' };
    }

    // 인증 요소 검증
    const provider = this.factorProviders.get(factorType);
    if (!provider) {
      throw new AuthenticationError('지원하지 않는 인증 방식입니다');
    }

    const verification = await provider.verify(session.userId, credential);

    if (!verification.success) {
      // 실패 횟수 증가
      await this.handleFailedAttempt(sessionId, factorType);
      return {
        success: false,
        reason: verification.reason,
        remainingAttempts: this.config.maxAttempts - session.failedAttempts - 1
      };
    }

    // 성공: 완료된 요소 추가
    session.completedFactors.push(factorType);
    await this.sessionStore.updateSession(session);

    // 모든 필수 요소 완료 여부 확인
    if (session.completedFactors.length >= session.requiredFactors) {
      const authToken = await this.issueAuthToken(session);
      return {
        success: true,
        authenticationComplete: true,
        authToken
      };
    }

    return {
      success: true,
      authenticationComplete: false,
      remainingFactors: session.requiredFactors - session.completedFactors.length
    };
  }

  private initializeProviders(): void {
    this.factorProviders = new Map([
      ['password', new PasswordProvider()],
      ['totp', new TOTPProvider()],
      ['sms', new SMSProvider()],
      ['hardware_token', new HardwareTokenProvider()],
      ['fingerprint', new BiometricProvider('fingerprint')],
      ['face', new BiometricProvider('face')],
      ['smart_card', new SmartCardProvider()]
    ]);
  }
}

// TOTP 제공자
class TOTPProvider implements FactorProvider {
  private secretStore: SecretStore;

  async verify(userId: string, credential: TOTPCredential): Promise<VerificationResult> {
    const secret = await this.secretStore.getTOTPSecret(userId);

    if (!secret) {
      return { success: false, reason: 'TOTP가 설정되지 않았습니다' };
    }

    // TOTP 검증 (시간 오차 허용)
    const isValid = this.verifyTOTP(credential.code, secret, {
      window: 1 // 앞뒤 30초 허용
    });

    if (!isValid) {
      return { success: false, reason: '잘못된 인증 코드입니다' };
    }

    // 재사용 방지
    const isReused = await this.checkCodeReuse(userId, credential.code);
    if (isReused) {
      return { success: false, reason: '이미 사용된 코드입니다' };
    }

    await this.markCodeUsed(userId, credential.code);

    return { success: true };
  }

  private verifyTOTP(code: string, secret: string, options: TOTPOptions): boolean {
    const { window = 0 } = options;
    const time = Math.floor(Date.now() / 30000);

    for (let i = -window; i <= window; i++) {
      const expectedCode = this.generateTOTP(secret, time + i);
      if (code === expectedCode) {
        return true;
      }
    }

    return false;
  }

  private generateTOTP(secret: string, time: number): string {
    const hmac = crypto.createHmac('sha1', Buffer.from(secret, 'base32'));
    const timeBuffer = Buffer.alloc(8);
    timeBuffer.writeBigInt64BE(BigInt(time));
    hmac.update(timeBuffer);

    const hash = hmac.digest();
    const offset = hash[hash.length - 1] & 0x0f;
    const code = (hash.readUInt32BE(offset) & 0x7fffffff) % 1000000;

    return code.toString().padStart(6, '0');
  }
}
```

### 7.2.2 속성 기반 접근 제어 (ABAC)

```typescript
// ABAC 정책 프레임워크
interface ABACFramework {
  attributes: {
    subject: {
      userId: string;
      roles: string[];
      department: string;
      clearanceLevel: number;
      certifications: string[];
    };
    resource: {
      resourceType: string;
      resourceId: string;
      classification: string;
      owner: string;
      location: string;
    };
    action: {
      type: 'read' | 'write' | 'execute' | 'delete' | 'admin';
    };
    environment: {
      time: Date;
      location: GeoLocation;
      deviceType: string;
      networkZone: string;
      threatLevel: string;
    };
  };

  policyStructure: {
    target: 'Subject, Resource, Action 조합 매칭';
    condition: '환경 조건 평가';
    effect: 'Permit | Deny';
    obligations: '부가 작업 (로깅, 알림 등)';
  };
}

// ABAC 정책 엔진
class ABACPolicyEngine {
  private policyStore: PolicyStore;
  private attributeService: AttributeService;

  async evaluate(request: AccessRequest): Promise<ABACDecision> {
    // 1. 속성 수집
    const subjectAttributes = await this.attributeService.getSubjectAttributes(request.subjectId);
    const resourceAttributes = await this.attributeService.getResourceAttributes(request.resourceId);
    const environmentAttributes = await this.attributeService.getEnvironmentAttributes(request);

    const context: ABACContext = {
      subject: subjectAttributes,
      resource: resourceAttributes,
      action: { type: request.action },
      environment: environmentAttributes
    };

    // 2. 적용 가능한 정책 조회
    const applicablePolicies = await this.policyStore.findApplicablePolicies(context);

    if (applicablePolicies.length === 0) {
      // 기본 거부
      return { decision: 'deny', reason: 'No applicable policy' };
    }

    // 3. 정책 평가
    const evaluations: PolicyEvaluation[] = [];

    for (const policy of applicablePolicies) {
      const evaluation = await this.evaluatePolicy(policy, context);
      evaluations.push(evaluation);
    }

    // 4. 정책 결합 (기본: deny-overrides)
    const finalDecision = this.combineDecisions(evaluations);

    // 5. Obligations 실행
    if (finalDecision.obligations) {
      await this.executeObligations(finalDecision.obligations, context);
    }

    return finalDecision;
  }

  private async evaluatePolicy(
    policy: ABACPolicy,
    context: ABACContext
  ): Promise<PolicyEvaluation> {
    // Target 매칭
    const targetMatch = this.matchTarget(policy.target, context);
    if (!targetMatch) {
      return { policyId: policy.id, applicable: false };
    }

    // Condition 평가
    const conditionResult = await this.evaluateCondition(policy.condition, context);

    return {
      policyId: policy.id,
      applicable: true,
      effect: conditionResult ? policy.effect : 'not_applicable',
      obligations: conditionResult ? policy.obligations : undefined
    };
  }

  private matchTarget(target: PolicyTarget, context: ABACContext): boolean {
    // Subject 매칭
    if (target.subject) {
      if (target.subject.roles && !target.subject.roles.some(r => context.subject.roles.includes(r))) {
        return false;
      }
      if (target.subject.department && context.subject.department !== target.subject.department) {
        return false;
      }
    }

    // Resource 매칭
    if (target.resource) {
      if (target.resource.type && context.resource.resourceType !== target.resource.type) {
        return false;
      }
      if (target.resource.classification && context.resource.classification !== target.resource.classification) {
        return false;
      }
    }

    // Action 매칭
    if (target.action && !target.action.includes(context.action.type)) {
      return false;
    }

    return true;
  }

  private async evaluateCondition(
    condition: PolicyCondition,
    context: ABACContext
  ): Promise<boolean> {
    // 조건식 평가 (간단한 예시)
    switch (condition.type) {
      case 'time_range':
        return this.evaluateTimeCondition(condition, context.environment.time);

      case 'location':
        return this.evaluateLocationCondition(condition, context.environment.location);

      case 'clearance':
        return context.subject.clearanceLevel >= condition.requiredLevel;

      case 'network_zone':
        return condition.allowedZones.includes(context.environment.networkZone);

      case 'composite':
        return this.evaluateCompositeCondition(condition, context);

      default:
        return true;
    }
  }

  private combineDecisions(evaluations: PolicyEvaluation[]): ABACDecision {
    // Deny-overrides: 하나라도 deny면 deny
    const applicableEvaluations = evaluations.filter(e => e.applicable);

    if (applicableEvaluations.length === 0) {
      return { decision: 'deny', reason: 'No applicable policy' };
    }

    const denyEvaluation = applicableEvaluations.find(e => e.effect === 'deny');
    if (denyEvaluation) {
      return {
        decision: 'deny',
        reason: `Policy ${denyEvaluation.policyId} denied`,
        obligations: denyEvaluation.obligations
      };
    }

    const permitEvaluation = applicableEvaluations.find(e => e.effect === 'permit');
    if (permitEvaluation) {
      return {
        decision: 'permit',
        obligations: this.collectObligations(applicableEvaluations)
      };
    }

    return { decision: 'deny', reason: 'No permit decision' };
  }
}

// BEMS 접근 제어 정책 예시
const bemsAccessPolicies: ABACPolicy[] = [
  // 정책 1: 에너지 매니저는 근무 시간에 에너지 데이터 읽기 가능
  {
    id: 'BEMS-POLICY-001',
    name: '에너지 매니저 데이터 읽기',
    target: {
      subject: { roles: ['energy_manager'] },
      resource: { type: 'energy_data' },
      action: ['read']
    },
    condition: {
      type: 'time_range',
      startHour: 6,
      endHour: 22
    },
    effect: 'permit',
    obligations: [
      { type: 'audit_log', level: 'info' }
    ]
  },

  // 정책 2: 제어 명령은 운영자가 Control Zone에서만 실행 가능
  {
    id: 'BEMS-POLICY-002',
    name: '제어 명령 실행',
    target: {
      subject: { roles: ['operator', 'admin'] },
      resource: { type: 'control_command' },
      action: ['execute']
    },
    condition: {
      type: 'composite',
      operator: 'and',
      conditions: [
        { type: 'network_zone', allowedZones: ['control_zone'] },
        { type: 'mfa_verified', required: true }
      ]
    },
    effect: 'permit',
    obligations: [
      { type: 'audit_log', level: 'warn' },
      { type: 'notify', recipients: ['security_team'] }
    ]
  },

  // 정책 3: 민감 데이터는 암호화된 연결에서만 접근
  {
    id: 'BEMS-POLICY-003',
    name: '민감 데이터 접근 제한',
    target: {
      resource: { classification: 'confidential' },
      action: ['read', 'write']
    },
    condition: {
      type: 'composite',
      operator: 'and',
      conditions: [
        { type: 'encryption', required: 'tls_1_3' },
        { type: 'device_compliance', required: true }
      ]
    },
    effect: 'permit',
    obligations: [
      { type: 'audit_log', level: 'info' },
      { type: 'data_mask', fields: ['personal_info'] }
    ]
  }
];
```

---

## 7.3 데이터 보호

### 7.3.1 암호화 전략

```typescript
// 데이터 암호화 프레임워크
interface EncryptionFramework {
  dataAtRest: {
    databases: {
      method: 'TDE (Transparent Data Encryption)';
      algorithm: 'AES-256';
      keyManagement: 'HSM 또는 클라우드 KMS';
    };
    fileStorage: {
      method: '파일 시스템 암호화';
      algorithm: 'AES-256-GCM';
      keyPerFile: boolean;
    };
    backups: {
      method: '백업 암호화';
      algorithm: 'AES-256';
      keyRotation: '분기별';
    };
  };

  dataInTransit: {
    externalAPI: {
      protocol: 'TLS 1.3';
      cipherSuites: ['TLS_AES_256_GCM_SHA384', 'TLS_CHACHA20_POLY1305_SHA256'];
      certificateType: 'EV SSL';
    };
    internalAPI: {
      protocol: 'mTLS';
      certificateManagement: '자동 갱신';
    };
    fieldCommunication: {
      protocol: 'BACnet/SC 또는 TLS over TCP';
    };
  };

  dataInUse: {
    memoryProtection: {
      method: '메모리 암호화';
      technology: 'Intel SGX, AMD SEV';
    };
    secureComputation: {
      method: '동형암호화 (선택적)';
      useCase: '민감 데이터 분석';
    };
  };
}

// 암호화 서비스 구현
class EncryptionService {
  private keyManager: KeyManagementService;
  private config: EncryptionConfig;

  constructor(config: EncryptionConfig) {
    this.config = config;
    this.keyManager = new KeyManagementService(config.kmsConfig);
  }

  // 봉투 암호화 (Envelope Encryption)
  async encryptData(
    plaintext: Buffer,
    context: EncryptionContext
  ): Promise<EncryptedData> {
    // 1. 데이터 암호화 키 (DEK) 생성
    const dek = crypto.randomBytes(32);

    // 2. DEK로 데이터 암호화
    const iv = crypto.randomBytes(12);
    const cipher = crypto.createCipheriv('aes-256-gcm', dek, iv);

    const encrypted = Buffer.concat([
      cipher.update(plaintext),
      cipher.final()
    ]);
    const authTag = cipher.getAuthTag();

    // 3. KEK (Key Encryption Key)로 DEK 암호화
    const encryptedDek = await this.keyManager.encrypt(dek, context.keyId);

    // 4. 결과 패키징
    return {
      version: 1,
      keyId: context.keyId,
      encryptedDek: encryptedDek.toString('base64'),
      iv: iv.toString('base64'),
      authTag: authTag.toString('base64'),
      ciphertext: encrypted.toString('base64'),
      algorithm: 'AES-256-GCM',
      timestamp: new Date().toISOString()
    };
  }

  async decryptData(
    encryptedData: EncryptedData
  ): Promise<Buffer> {
    // 1. DEK 복호화
    const encryptedDek = Buffer.from(encryptedData.encryptedDek, 'base64');
    const dek = await this.keyManager.decrypt(encryptedDek, encryptedData.keyId);

    // 2. 데이터 복호화
    const iv = Buffer.from(encryptedData.iv, 'base64');
    const authTag = Buffer.from(encryptedData.authTag, 'base64');
    const ciphertext = Buffer.from(encryptedData.ciphertext, 'base64');

    const decipher = crypto.createDecipheriv('aes-256-gcm', dek, iv);
    decipher.setAuthTag(authTag);

    return Buffer.concat([
      decipher.update(ciphertext),
      decipher.final()
    ]);
  }

  // 필드 레벨 암호화
  async encryptField(
    value: string,
    fieldType: 'pii' | 'financial' | 'credential'
  ): Promise<string> {
    const keyId = this.config.fieldKeys[fieldType];
    const plaintext = Buffer.from(value, 'utf-8');

    const encrypted = await this.encryptData(plaintext, { keyId });

    return `ENC:${Buffer.from(JSON.stringify(encrypted)).toString('base64')}`;
  }

  async decryptField(encryptedValue: string): Promise<string> {
    if (!encryptedValue.startsWith('ENC:')) {
      return encryptedValue; // 암호화되지 않은 값
    }

    const encryptedData = JSON.parse(
      Buffer.from(encryptedValue.slice(4), 'base64').toString('utf-8')
    );

    const plaintext = await this.decryptData(encryptedData);
    return plaintext.toString('utf-8');
  }
}

// 키 관리 서비스
class KeyManagementService {
  private kmsClient: CloudKMSClient;
  private keyCache: Map<string, CachedKey>;

  constructor(config: KMSConfig) {
    this.kmsClient = new CloudKMSClient(config);
    this.keyCache = new Map();
  }

  async encrypt(plaintext: Buffer, keyId: string): Promise<Buffer> {
    const key = await this.getKey(keyId);
    return this.kmsClient.encrypt(keyId, plaintext);
  }

  async decrypt(ciphertext: Buffer, keyId: string): Promise<Buffer> {
    return this.kmsClient.decrypt(keyId, ciphertext);
  }

  async rotateKey(keyId: string): Promise<KeyRotationResult> {
    // 1. 새 키 버전 생성
    const newVersion = await this.kmsClient.createKeyVersion(keyId);

    // 2. 새 버전을 기본으로 설정
    await this.kmsClient.setDefaultKeyVersion(keyId, newVersion);

    // 3. 이전 키 버전은 복호화 전용으로 유지 (일정 기간)
    // 4. 캐시 무효화
    this.keyCache.delete(keyId);

    return {
      keyId,
      previousVersion: (await this.getKey(keyId)).version - 1,
      newVersion: newVersion,
      rotatedAt: new Date()
    };
  }

  private async getKey(keyId: string): Promise<CachedKey> {
    const cached = this.keyCache.get(keyId);
    if (cached && cached.expiresAt > new Date()) {
      return cached;
    }

    const keyInfo = await this.kmsClient.getKeyInfo(keyId);
    const cachedKey: CachedKey = {
      ...keyInfo,
      expiresAt: new Date(Date.now() + 60 * 60 * 1000) // 1시간 캐시
    };

    this.keyCache.set(keyId, cachedKey);
    return cachedKey;
  }
}
```

---

## 7.4 보안 모니터링 및 사고 대응

### 7.4.1 SIEM 통합

```typescript
// SIEM 통합 아키텍처
interface SIEMIntegration {
  logSources: {
    bems: ['API 로그', '인증 로그', '제어 명령 로그', 'FDD 이벤트'];
    network: ['방화벽 로그', 'IDS/IPS 알림', 'NetFlow'];
    endpoint: ['EDR 이벤트', '시스템 로그', '애플리케이션 로그'];
    bas: ['BAS 이벤트', '알람 로그', '접근 로그'];
  };

  correlationRules: {
    bruteForce: '단시간 다수 인증 실패';
    privilegeEscalation: '권한 상승 시도';
    dataExfiltration: '대량 데이터 전송';
    anomalousControl: '비정상 제어 명령';
    lateralMovement: '내부 이동 패턴';
  };

  alertPriority: {
    critical: '즉시 대응 필요';
    high: '4시간 내 대응';
    medium: '24시간 내 검토';
    low: '주간 검토';
  };
}

// SIEM 통합 서비스
class SIEMIntegrationService {
  private syslogClient: SyslogClient;
  private eventFormatter: EventFormatter;
  private alertRules: Map<string, AlertRule>;

  constructor(config: SIEMConfig) {
    this.syslogClient = new SyslogClient(config.syslogServer);
    this.eventFormatter = new EventFormatter(config.format);
    this.loadAlertRules();
  }

  async sendSecurityEvent(event: SecurityEvent): Promise<void> {
    // CEF (Common Event Format) 형식으로 변환
    const cefEvent = this.eventFormatter.toCEF(event);

    // SIEM으로 전송
    await this.syslogClient.send(cefEvent);

    // 로컬 상관 분석
    await this.correlateEvent(event);
  }

  private async correlateEvent(event: SecurityEvent): Promise<void> {
    for (const [ruleName, rule] of this.alertRules) {
      const match = await this.evaluateRule(rule, event);

      if (match) {
        await this.raiseAlert({
          ruleName,
          severity: rule.severity,
          event,
          description: rule.description,
          timestamp: new Date()
        });
      }
    }
  }

  private loadAlertRules(): void {
    this.alertRules = new Map([
      ['BEMS_BRUTE_FORCE', {
        name: 'BEMS 인증 무차별 공격',
        description: '5분 내 동일 계정 5회 이상 인증 실패',
        severity: 'high',
        conditions: {
          eventType: 'authentication_failure',
          threshold: 5,
          timeWindow: 300, // 초
          groupBy: 'user_id'
        }
      }],
      ['BEMS_ANOMALOUS_CONTROL', {
        name: '비정상 제어 명령',
        description: '비근무 시간 제어 명령 실행',
        severity: 'critical',
        conditions: {
          eventType: 'control_command',
          timeCondition: { outsideHours: [6, 22] },
          excludeRoles: ['admin', 'emergency_operator']
        }
      }],
      ['BEMS_DATA_EXFIL', {
        name: '대량 데이터 내보내기',
        description: '1시간 내 100MB 이상 데이터 내보내기',
        severity: 'high',
        conditions: {
          eventType: 'data_export',
          threshold: { bytes: 100 * 1024 * 1024 },
          timeWindow: 3600
        }
      }],
      ['BEMS_PRIVILEGE_ESCALATION', {
        name: '권한 상승 시도',
        description: '권한 변경 후 즉시 민감 리소스 접근',
        severity: 'critical',
        conditions: {
          sequence: [
            { eventType: 'role_change' },
            { eventType: 'sensitive_resource_access', within: 300 }
          ]
        }
      }]
    ]);
  }
}

// 이벤트 포맷터
class EventFormatter {
  toCEF(event: SecurityEvent): string {
    // CEF:Version|Device Vendor|Device Product|Device Version|Signature ID|Name|Severity|Extension
    const cef = [
      'CEF:0',
      'WIA',
      'BEMS',
      '1.0',
      event.signatureId,
      event.name,
      this.mapSeverity(event.severity),
      this.formatExtension(event)
    ].join('|');

    return cef;
  }

  private formatExtension(event: SecurityEvent): string {
    const extensions: string[] = [];

    if (event.sourceAddress) extensions.push(`src=${event.sourceAddress}`);
    if (event.destinationAddress) extensions.push(`dst=${event.destinationAddress}`);
    if (event.userId) extensions.push(`suser=${event.userId}`);
    if (event.resourceId) extensions.push(`cs1=${event.resourceId} cs1Label=ResourceID`);
    if (event.outcome) extensions.push(`outcome=${event.outcome}`);
    if (event.message) extensions.push(`msg=${event.message}`);

    return extensions.join(' ');
  }

  private mapSeverity(severity: string): number {
    const mapping: Record<string, number> = {
      'critical': 10,
      'high': 7,
      'medium': 5,
      'low': 3,
      'info': 1
    };
    return mapping[severity] || 5;
  }
}
```

### 7.4.2 사고 대응 절차

```typescript
// 사고 대응 프레임워크
interface IncidentResponseFramework {
  phases: {
    preparation: {
      name: '준비';
      activities: [
        '사고 대응 계획 수립',
        '대응팀 구성 및 교육',
        '도구 및 리소스 확보',
        '연락처 목록 유지',
        '정기 훈련 실시'
      ];
    };
    detection: {
      name: '탐지 및 분석';
      activities: [
        '이상 징후 모니터링',
        '알림 분류 및 우선순위화',
        '초기 영향 평가',
        '사고 여부 결정'
      ];
    };
    containment: {
      name: '억제';
      activities: [
        '단기 억제 (확산 방지)',
        '장기 억제 (시스템 격리)',
        '증거 보존',
        '영향 받는 시스템 식별'
      ];
    };
    eradication: {
      name: '제거';
      activities: [
        '근본 원인 제거',
        '악성코드 제거',
        '취약점 패치',
        '시스템 강화'
      ];
    };
    recovery: {
      name: '복구';
      activities: [
        '시스템 복원',
        '정상 운영 재개',
        '모니터링 강화',
        '검증 테스트'
      ];
    };
    lessonsLearned: {
      name: '교훈';
      activities: [
        '사고 보고서 작성',
        '개선 사항 도출',
        '프로세스 업데이트',
        '훈련 계획 수정'
      ];
    };
  };

  escalationMatrix: {
    severity1: {
      definition: '운영 중단, 데이터 유출, 안전 위협';
      responseTime: '즉시';
      notification: ['CISO', 'CEO', '법무팀', '규제 기관'];
    };
    severity2: {
      definition: '부분 서비스 장애, 보안 침해 징후';
      responseTime: '1시간';
      notification: ['보안팀장', 'IT 매니저'];
    };
    severity3: {
      definition: '비정상 활동, 정책 위반';
      responseTime: '4시간';
      notification: ['보안팀'];
    };
  };
}

// 사고 대응 시스템
class IncidentResponseSystem {
  private incidentStore: IncidentStore;
  private notificationService: NotificationService;
  private playbookEngine: PlaybookEngine;
  private forensicService: ForensicService;

  async handleSecurityAlert(alert: SecurityAlert): Promise<void> {
    // 1. 사고 생성
    const incident = await this.createIncident(alert);

    // 2. 초기 분류
    const classification = await this.classifyIncident(incident);

    // 3. 에스컬레이션
    await this.escalate(incident, classification);

    // 4. 플레이북 실행
    await this.executePlaybook(incident, classification);
  }

  private async createIncident(alert: SecurityAlert): Promise<Incident> {
    const incident: Incident = {
      id: generateIncidentId(),
      status: 'open',
      createdAt: new Date(),
      source: alert,
      severity: alert.severity,
      title: alert.name,
      description: alert.description,
      affectedAssets: await this.identifyAffectedAssets(alert),
      timeline: [{
        timestamp: new Date(),
        action: 'incident_created',
        actor: 'system',
        details: `보안 알림으로부터 사고 생성: ${alert.id}`
      }]
    };

    await this.incidentStore.save(incident);
    return incident;
  }

  private async classifyIncident(incident: Incident): Promise<IncidentClassification> {
    // 자동 분류 규칙 적용
    const classification: IncidentClassification = {
      category: this.determineCategory(incident),
      severity: this.assessSeverity(incident),
      impactScope: await this.assessImpactScope(incident),
      dataInvolved: await this.assessDataInvolvement(incident),
      attackVector: this.identifyAttackVector(incident)
    };

    return classification;
  }

  private async executePlaybook(
    incident: Incident,
    classification: IncidentClassification
  ): Promise<void> {
    // 분류에 따른 플레이북 선택
    const playbook = await this.playbookEngine.selectPlaybook(classification);

    if (!playbook) {
      await this.notifyNoPlaybook(incident);
      return;
    }

    // 플레이북 단계 실행
    for (const step of playbook.steps) {
      try {
        await this.executePlaybookStep(incident, step);

        // 타임라인 업데이트
        await this.updateTimeline(incident, {
          action: 'playbook_step_completed',
          actor: 'automation',
          details: `플레이북 단계 완료: ${step.name}`
        });
      } catch (error) {
        // 자동화 실패 시 수동 개입 요청
        await this.requestManualIntervention(incident, step, error);
        break;
      }
    }
  }

  private async executePlaybookStep(
    incident: Incident,
    step: PlaybookStep
  ): Promise<void> {
    switch (step.type) {
      case 'isolate_host':
        await this.isolateHost(step.targetHost);
        break;

      case 'block_ip':
        await this.blockIP(step.ipAddress);
        break;

      case 'disable_user':
        await this.disableUser(step.userId);
        break;

      case 'collect_evidence':
        await this.forensicService.collectEvidence(incident, step.evidenceTypes);
        break;

      case 'notify':
        await this.notificationService.send(step.recipients, step.template, incident);
        break;

      case 'manual':
        // 수동 단계: 담당자에게 알림
        await this.requestManualAction(incident, step);
        break;
    }
  }

  private async isolateHost(hostId: string): Promise<void> {
    // 네트워크에서 호스트 격리
    console.log(`호스트 격리: ${hostId}`);

    // 1. 방화벽 규칙 추가 (모든 트래픽 차단)
    // 2. NAC에서 격리 VLAN으로 이동
    // 3. 에이전트에 격리 명령 전송
  }

  async closeIncident(
    incidentId: string,
    resolution: IncidentResolution
  ): Promise<void> {
    const incident = await this.incidentStore.get(incidentId);

    incident.status = 'closed';
    incident.closedAt = new Date();
    incident.resolution = resolution;

    await this.updateTimeline(incident, {
      action: 'incident_closed',
      actor: resolution.closedBy,
      details: resolution.summary
    });

    await this.incidentStore.save(incident);

    // 사후 분석 트리거
    await this.triggerPostIncidentReview(incident);
  }
}
```

---

## 7.5 장 요약

### 보안 프레임워크 핵심 요소

| 영역 | 핵심 통제 | 기술/표준 |
|------|----------|----------|
| 네트워크 | 세그먼테이션, 방화벽 | VLAN, NGFW, IDS/IPS |
| 접근 제어 | MFA, ABAC | OAuth 2.0, SAML, XACML |
| 데이터 보호 | 암호화 | AES-256, TLS 1.3, KMS |
| 모니터링 | SIEM, 로깅 | CEF, Syslog, SOAR |
| 사고 대응 | 플레이북 | NIST SP 800-61 |

### 제로 트러스트 원칙 적용

- **신뢰하지 않음**: 내부/외부 구분 없이 모든 요청 검증
- **항상 검증**: 매 접근 시 인증/인가 수행
- **최소 권한**: 필요한 최소한의 권한만 부여
- **침해 가정**: 이미 침해되었다고 가정하고 설계

### 규제 준수

- **개인정보보호법**: 개인정보 암호화, 접근 기록
- **ISMS-P**: 정보보호 관리체계 인증
- **IEC 62443**: 산업 제어 시스템 보안

### 다음 장 미리보기

제8장에서는 WIA-BEMS 구현 가이드 및 모범 사례에 대해 다룹니다. 프로젝트 계획, 단계별 구현 전략, 커미셔닝, 운영 절차를 학습합니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
