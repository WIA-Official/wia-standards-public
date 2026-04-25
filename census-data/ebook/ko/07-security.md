# 제7장: 인구조사 데이터 보안 및 프라이버시

## 통계적 기밀성을 위한 포괄적인 보안 프레임워크

### 7.1 보안 아키텍처 개요

WIA-CENSUS-DATA 표준은 통계 분석 및 연구 접근을 가능하게 하면서 개인 기밀성을 보호하는 포괄적인 보안 프레임워크를 수립합니다. 이 장에서는 인구조사 데이터 보호에 필수적인 다층 보안 접근 방식을 다룹니다.

```typescript
// 인구조사 보안 아키텍처
interface CensusSecurityArchitecture {
  version: '1.0.0';

  securityLayers: {
    perimeter: {
      description: '네트워크 경계 보호';
      controls: [
        '웹 애플리케이션 방화벽',
        'DDoS 보호',
        '침입 탐지/방지',
        '네트워크 세분화'
      ];
    };
    transport: {
      description: '전송 중 데이터 보호';
      controls: [
        'TLS 1.3 암호화',
        '인증서 고정',
        '서비스용 상호 TLS',
        '관리 접근용 VPN'
      ];
    };
    application: {
      description: '애플리케이션 수준 보안';
      controls: [
        '인증 및 권한 부여',
        '입력 검증',
        '출력 인코딩',
        '세션 관리'
      ];
    };
    data: {
      description: '데이터 보호 제어';
      controls: [
        '저장 시 암호화',
        '키 관리',
        '데이터 마스킹',
        '공개 제어'
      ];
    };
    operational: {
      description: '운영 보안';
      controls: [
        '감사 로깅',
        '모니터링 및 경고',
        '사고 대응',
        '백업 및 복구'
      ];
    };
  };

  complianceFrameworks: [
    'ISO 27001',
    'NIST 사이버보안 프레임워크',
    'GDPR (EU)',
    '국가 통계 기밀성 법률'
  ];
}

// 보안 서비스 구현
class CensusSecurityService {
  private authService: AuthenticationService;
  private authzService: AuthorizationService;
  private encryptionService: EncryptionService;
  private auditService: AuditService;

  constructor(config: SecurityConfig) {
    this.authService = new AuthenticationService(config.auth);
    this.authzService = new AuthorizationService(config.authz);
    this.encryptionService = new EncryptionService(config.encryption);
    this.auditService = new AuditService(config.audit);
  }

  async authenticateRequest(
    credentials: Credentials
  ): Promise<AuthenticationResult> {
    // 자격 증명 검증
    const validation = await this.authService.validate(credentials);

    if (!validation.valid) {
      await this.auditService.log({
        event: 'AUTH_FAILURE',
        details: { reason: validation.reason },
        severity: 'WARNING'
      });

      return {
        authenticated: false,
        reason: validation.reason
      };
    }

    // 세션 생성
    const session = await this.authService.createSession(validation.identity);

    await this.auditService.log({
      event: 'AUTH_SUCCESS',
      identity: validation.identity.id,
      sessionId: session.id
    });

    return {
      authenticated: true,
      identity: validation.identity,
      session
    };
  }

  async authorizeAccess(
    identity: Identity,
    resource: Resource,
    action: Action
  ): Promise<AuthorizationResult> {
    // 접근 정책 평가
    const decision = await this.authzService.evaluate({
      subject: identity,
      resource,
      action,
      context: this.buildContext()
    });

    await this.auditService.log({
      event: 'AUTHZ_DECISION',
      identity: identity.id,
      resource: resource.id,
      action,
      decision: decision.allowed ? 'PERMIT' : 'DENY'
    });

    return decision;
  }
}
```

### 7.2 인증 및 접근 제어

```typescript
// 인증 프레임워크
interface AuthenticationFramework {
  methods: {
    apiKey: {
      useCase: '공개 API 접근';
      strength: 'BASIC';
      rateLimit: '낮은 계층';
    };
    oauth2: {
      useCase: '애플리케이션 및 사용자 인증';
      flows: ['Authorization Code', 'Client Credentials'];
      strength: 'STANDARD';
    };
    saml2: {
      useCase: '엔터프라이즈 SSO 통합';
      strength: 'STANDARD';
    };
    mutualTls: {
      useCase: '서비스 간, 관리';
      strength: 'HIGH';
    };
    multiFactor: {
      useCase: '민감한 데이터 접근';
      factors: ['비밀번호', 'TOTP', '하드웨어 키'];
      strength: 'HIGHEST';
    };
  };
}

// 역할 기반 접근 제어
interface RBACModel {
  roles: {
    PUBLIC_USER: {
      description: '익명 또는 등록된 공개 사용자';
      permissions: [
        'READ_PUBLIC_DATA',
        'ACCESS_PUBLIC_API'
      ];
      dataAccessLevel: 'AGGREGATE_ONLY';
    };
    RESEARCHER: {
      description: '승인된 연구 사용자';
      permissions: [
        'READ_PUBLIC_DATA',
        'READ_RESEARCH_MICRODATA',
        'SUBMIT_ANALYSIS_REQUESTS'
      ];
      dataAccessLevel: 'RESEARCH_MICRODATA';
      requirements: ['승인된 연구 제안서', '윤리 승인'];
    };
    STATISTICIAN: {
      description: '내부 통계 직원';
      permissions: [
        'READ_ALL_DATA',
        'PROCESS_DATA',
        'GENERATE_OUTPUTS',
        'APPLY_DISCLOSURE_CONTROLS'
      ];
      dataAccessLevel: 'FULL_MICRODATA';
    };
    DATA_ADMINISTRATOR: {
      description: '시스템 및 데이터 관리자';
      permissions: [
        'MANAGE_USERS',
        'MANAGE_ACCESS',
        'AUDIT_REVIEW',
        'SYSTEM_CONFIG'
      ];
      dataAccessLevel: 'ADMINISTRATIVE';
    };
  };
}

// 속성 기반 접근 제어 정책
class ABACPolicyEngine {
  private policyStore: PolicyStore;

  async evaluate(request: AccessRequest): Promise<PolicyDecision> {
    // 적용 가능한 정책 가져오기
    const policies = await this.policyStore.findPolicies({
      resourceType: request.resource.type,
      action: request.action
    });

    // 각 정책 평가
    const decisions: PolicyEvaluation[] = [];

    for (const policy of policies) {
      const evaluation = await this.evaluatePolicy(policy, request);
      decisions.push(evaluation);
    }

    // 결정 결합
    return this.combineDecisions(decisions);
  }

  private async evaluateCondition(
    condition: PolicyCondition,
    request: AccessRequest
  ): Promise<boolean> {
    const attributeValue = this.getAttributeValue(
      condition.attribute,
      request
    );

    switch (condition.operator) {
      case 'EQUALS':
        return attributeValue === condition.value;

      case 'IN':
        return (condition.value as any[]).includes(attributeValue);

      case 'GREATER_THAN':
        return attributeValue > condition.value;

      case 'TIME_BETWEEN':
        const now = new Date();
        const [start, end] = condition.value as [string, string];
        return now >= new Date(start) && now <= new Date(end);

      default:
        return false;
    }
  }
}
```

### 7.3 통계적 공개 제어

```typescript
// 통계적 공개 제어 프레임워크
interface DisclosureControlFramework {
  methods: {
    suppression: {
      description: '관측치가 적은 셀 제거';
      techniques: ['1차 억제', '2차 억제'];
      useCase: '표 데이터 보호';
    };
    perturbation: {
      description: '데이터 값에 노이즈 추가';
      techniques: ['무작위 반올림', '제어 반올림', '셀 섭동'];
      useCase: '집계 통계';
    };
    generalization: {
      description: '정밀도 또는 상세도 감소';
      techniques: ['상하 코딩', '재코딩', '데이터 스와핑'];
      useCase: '마이크로데이터 보호';
    };
    syntheticData: {
      description: '유사한 속성을 가진 인공 데이터 생성';
      techniques: ['완전 합성', '부분 합성', '하이브리드'];
      useCase: '공개 사용 파일, 테스트';
    };
    differentialPrivacy: {
      description: '수학적으로 증명 가능한 프라이버시 보장';
      techniques: ['라플라스 메커니즘', '가우시안 메커니즘', '지수 메커니즘'];
      useCase: '대화형 쿼리, 공식 통계';
    };
  };
}

// 공개 제어 서비스
class DisclosureControlService {
  private disclosureRules: DisclosureRules;
  private perturbationEngine: PerturbationEngine;
  private syntheticGenerator: SyntheticDataGenerator;

  async applyDisclosureControl(
    data: CensusData,
    outputType: OutputType
  ): Promise<ProtectedData> {
    switch (outputType) {
      case 'AGGREGATE_TABLE':
        return this.protectAggregateTable(data);

      case 'MICRODATA_FILE':
        return this.protectMicrodata(data);

      case 'INTERACTIVE_QUERY':
        return this.protectInteractiveQuery(data);

      case 'SYNTHETIC_FILE':
        return this.generateSyntheticData(data);

      default:
        throw new Error(`지원되지 않는 출력 유형: ${outputType}`);
    }
  }

  private async protectAggregateTable(
    data: CensusData
  ): Promise<ProtectedData> {
    const table = data as AggregateTable;
    const protectedTable = { ...table };

    // 1차 억제 적용
    const primarySuppressions = this.identifyPrimarySuppression(table);

    // 보완 억제 적용
    const allSuppressions = await this.applyComplementarySuppression(
      table,
      primarySuppressions
    );

    // 셀 억제
    for (const cellKey of allSuppressions) {
      protectedTable.cells[cellKey] = {
        ...protectedTable.cells[cellKey],
        value: null,
        suppressed: true,
        suppressionReason: primarySuppressions.has(cellKey)
          ? 'PRIMARY'
          : 'COMPLEMENTARY'
      };
    }

    return {
      data: protectedTable,
      disclosureMethod: 'SUPPRESSION',
      cellsAffected: allSuppressions.size,
      protectionApplied: new Date().toISOString()
    };
  }

  private identifyPrimarySuppression(
    table: AggregateTable
  ): Set<string> {
    const suppressions = new Set<string>();

    for (const [cellKey, cell] of Object.entries(table.cells)) {
      // 빈도 규칙: 수가 임계값 미만이면 억제
      if (cell.count < this.disclosureRules.minimumFrequency) {
        suppressions.add(cellKey);
        continue;
      }

      // 지배 규칙: 상위 기여자가 지배하면 억제
      if (cell.contributorShares) {
        const topShare = cell.contributorShares[0];
        if (topShare > this.disclosureRules.dominanceThreshold) {
          suppressions.add(cellKey);
          continue;
        }
      }
    }

    return suppressions;
  }
}

// 차등 프라이버시 구현
class DifferentialPrivacyService {
  private epsilon: number; // 프라이버시 예산
  private delta: number; // 실패 확률

  constructor(config: DPConfig) {
    this.epsilon = config.epsilon;
    this.delta = config.delta;
  }

  async queryWithDP(
    query: StatisticalQuery,
    dataset: CensusDataset
  ): Promise<DPQueryResult> {
    // 쿼리 민감도 계산
    const sensitivity = this.calculateSensitivity(query);

    // 노이즈 스케일 결정
    const noiseScale = this.calculateNoiseScale(sensitivity);

    // 실제 쿼리 실행
    const trueResult = await this.executeQuery(query, dataset);

    // 보정된 노이즈 추가
    const noisyResult = this.addNoise(trueResult, noiseScale);

    // 프라이버시 예산 업데이트
    await this.updatePrivacyBudget(query, this.epsilon);

    return {
      value: noisyResult,
      epsilon: this.epsilon,
      mechanism: 'LAPLACE',
      confidenceInterval: this.calculateConfidenceInterval(noisyResult, noiseScale),
      privacyGuarantee: `(${this.epsilon}, ${this.delta})-차등 프라이버시`
    };
  }

  private calculateSensitivity(query: StatisticalQuery): number {
    switch (query.type) {
      case 'COUNT':
        return 1; // 한 사람 추가/제거가 카운트를 1 변경

      case 'SUM':
        return query.bounds.max - query.bounds.min;

      case 'MEAN':
        return (query.bounds.max - query.bounds.min) / query.minGroupSize;

      case 'HISTOGRAM':
        return 2; // 한 사람이 두 개의 빈에 영향

      default:
        throw new Error(`알 수 없는 쿼리 유형: ${query.type}`);
    }
  }

  private addNoise(value: number, scale: number): number {
    // 라플라스 분포에서 샘플링
    const u = Math.random() - 0.5;
    const noise = -scale * Math.sign(u) * Math.log(1 - 2 * Math.abs(u));
    return value + noise;
  }
}
```

### 7.4 암호화 및 키 관리

```typescript
// 암호화 프레임워크
interface EncryptionFramework {
  atRest: {
    algorithm: 'AES-256-GCM';
    keyLength: 256;
    keyRotation: '90일';
    scope: ['데이터베이스 열', '파일 스토리지', '백업'];
  };

  inTransit: {
    protocol: 'TLS 1.3';
    cipherSuites: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ];
    certificateAuthority: '정부 승인 CA';
  };
}

// 키 관리 서비스
class KeyManagementService {
  private hsm: HSMInterface;
  private keyStore: SecureKeyStore;

  async generateDataEncryptionKey(
    purpose: string
  ): Promise<DataEncryptionKey> {
    // HSM에서 DEK 생성
    const dekId = await this.hsm.generateKey({
      algorithm: 'AES',
      keyLength: 256,
      extractable: true
    });

    // 래핑용 KEK 가져오기
    const kek = await this.getKeyEncryptionKey();

    // KEK로 DEK 래핑
    const wrappedDek = await this.hsm.wrapKey(dekId, kek.id);

    // 래핑된 DEK 저장
    const dek: DataEncryptionKey = {
      id: crypto.randomUUID(),
      purpose,
      wrappedKey: wrappedDek,
      kekId: kek.id,
      createdAt: new Date().toISOString(),
      expiresAt: this.calculateExpiration(),
      status: 'ACTIVE'
    };

    await this.keyStore.store(dek);

    return dek;
  }

  async encryptData(
    data: Buffer,
    keyId: string
  ): Promise<EncryptedData> {
    // 키 가져오기
    const dek = await this.keyStore.get(keyId);

    if (!dek || dek.status !== 'ACTIVE') {
      throw new Error(`유효하지 않거나 비활성 키: ${keyId}`);
    }

    // DEK 언래핑
    const unwrappedKey = await this.hsm.unwrapKey(dek.wrappedKey, dek.kekId);

    // IV 생성
    const iv = crypto.randomBytes(12);

    // 데이터 암호화
    const cipher = crypto.createCipheriv('aes-256-gcm', unwrappedKey, iv);
    const encrypted = Buffer.concat([
      cipher.update(data),
      cipher.final()
    ]);
    const authTag = cipher.getAuthTag();

    // 메모리에서 언래핑된 키 제거
    unwrappedKey.fill(0);

    return {
      ciphertext: encrypted,
      iv,
      authTag,
      keyId: dek.id,
      algorithm: 'AES-256-GCM'
    };
  }

  async decryptData(
    encryptedData: EncryptedData
  ): Promise<Buffer> {
    // 키 가져오기
    const dek = await this.keyStore.get(encryptedData.keyId);

    if (!dek) {
      throw new Error(`키를 찾을 수 없음: ${encryptedData.keyId}`);
    }

    // DEK 언래핑
    const unwrappedKey = await this.hsm.unwrapKey(dek.wrappedKey, dek.kekId);

    // 데이터 복호화
    const decipher = crypto.createDecipheriv(
      'aes-256-gcm',
      unwrappedKey,
      encryptedData.iv
    );
    decipher.setAuthTag(encryptedData.authTag);

    const decrypted = Buffer.concat([
      decipher.update(encryptedData.ciphertext),
      decipher.final()
    ]);

    // 메모리에서 언래핑된 키 제거
    unwrappedKey.fill(0);

    return decrypted;
  }
}
```

### 7.5 감사 및 규정 준수

```typescript
// 감사 프레임워크
interface AuditFramework {
  auditableEvents: {
    authentication: ['로그인', '로그아웃', '실패한 로그인', '비밀번호 변경'];
    authorization: ['접근 허용', '접근 거부', '권한 변경'];
    dataAccess: ['쿼리', '다운로드', '내보내기', '인쇄'];
    dataModification: ['생성', '업데이트', '삭제', '가져오기'];
    administration: ['사용자 관리', '역할 변경', '구성 변경'];
    security: ['키 순환', '인증서 갱신', '보안 경고'];
  };

  retentionPolicy: {
    securityEvents: '7년';
    dataAccessEvents: '10년';
    administrativeEvents: '7년';
  };

  auditLogProtection: {
    integrity: '해시 체인';
    confidentiality: '저장 시 암호화';
    availability: '지리적 복제';
  };
}

// 감사 서비스 구현
class AuditService {
  private auditStore: AuditStore;
  private hashChain: HashChain;
  private alertService: AlertService;

  async log(event: AuditEvent): Promise<void> {
    // 이벤트 보강
    const enrichedEvent = {
      ...event,
      eventId: crypto.randomUUID(),
      timestamp: new Date().toISOString(),
      serverInstance: this.getServerInstance(),
      clientInfo: this.getClientInfo()
    };

    // 해시 계산
    const previousHash = await this.hashChain.getLastHash();
    const eventHash = this.calculateHash(enrichedEvent, previousHash);

    enrichedEvent.previousHash = previousHash;
    enrichedEvent.eventHash = eventHash;

    // 이벤트 저장
    await this.auditStore.store(enrichedEvent);

    // 해시 체인 업데이트
    await this.hashChain.append(eventHash);

    // 경고 조건 확인
    await this.checkAlertConditions(enrichedEvent);
  }

  async query(
    criteria: AuditQueryCriteria
  ): Promise<AuditQueryResult> {
    // 쿼리 권한 검증
    // 권한이 있는 인원만 감사 로그 쿼리 가능

    const events = await this.auditStore.query(criteria);

    // 반환된 이벤트의 무결성 검증
    const integrityResults = await this.verifyIntegrity(events);

    return {
      events,
      total: events.length,
      integrityVerified: integrityResults.allValid,
      integrityIssues: integrityResults.issues
    };
  }

  async generateComplianceReport(
    period: DateRange,
    framework: ComplianceFramework
  ): Promise<ComplianceReport> {
    // 관련 감사 이벤트 쿼리
    const events = await this.query({
      startDate: period.start,
      endDate: period.end
    });

    // 규정 준수 요구사항에 대해 분석
    const findings: ComplianceFinding[] = [];

    for (const requirement of framework.requirements) {
      const finding = await this.assessRequirement(
        requirement,
        events.events
      );
      findings.push(finding);
    }

    return {
      period,
      framework: framework.name,
      overallStatus: this.calculateOverallStatus(findings),
      findings,
      generatedAt: new Date().toISOString()
    };
  }
}
```

### 7.6 사고 대응

```typescript
// 사고 대응 프레임워크
class IncidentResponseService {
  private alertManager: AlertManager;
  private notificationService: NotificationService;
  private containmentService: ContainmentService;

  async handleSecurityIncident(
    incident: SecurityIncident
  ): Promise<IncidentResponse> {
    // 초기 평가
    const assessment = await this.assessIncident(incident);

    // 필요한 경우 에스컬레이션
    if (assessment.severity >= IncidentSeverity.HIGH) {
      await this.escalate(incident, assessment);
    }

    // 활성 위협인 경우 격리
    if (assessment.activeTheat) {
      await this.containmentService.contain(incident, assessment);
    }

    // 사고 기록 생성
    const incidentRecord = await this.createIncidentRecord(
      incident,
      assessment
    );

    // 대응 워크플로우 시작
    await this.initiateResponseWorkflow(incidentRecord);

    return {
      incidentId: incidentRecord.id,
      status: 'RESPONSE_INITIATED',
      severity: assessment.severity,
      containmentActions: assessment.activeTheat
        ? await this.containmentService.getActions(incidentRecord.id)
        : [],
      nextSteps: this.determineNextSteps(assessment)
    };
  }

  private calculateSeverity(incident: SecurityIncident): IncidentSeverity {
    let score = 0;

    // 데이터 민감도
    if (incident.dataInvolved?.includes('MICRODATA')) score += 3;
    if (incident.dataInvolved?.includes('IDENTIFIABLE')) score += 4;

    // 범위
    if (incident.recordsAffected > 1000000) score += 3;
    else if (incident.recordsAffected > 10000) score += 2;
    else if (incident.recordsAffected > 100) score += 1;

    // 위협 행위자
    if (incident.threatActor === 'NATION_STATE') score += 3;
    if (incident.threatActor === 'ORGANIZED_CRIME') score += 2;

    // 심각도 수준 결정
    if (score >= 8) return IncidentSeverity.CRITICAL;
    if (score >= 5) return IncidentSeverity.HIGH;
    if (score >= 3) return IncidentSeverity.MEDIUM;
    return IncidentSeverity.LOW;
  }
}

enum IncidentSeverity {
  LOW = 1,
  MEDIUM = 2,
  HIGH = 3,
  CRITICAL = 4
}
```

---

**WIA-CENSUS-DATA 보안 및 프라이버시**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
