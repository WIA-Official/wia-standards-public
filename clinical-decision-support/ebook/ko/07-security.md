# 제7장: 임상의사결정지원 보안

## 환자 안전, 개인정보 보호 및 규제 준수

### 7.1 CDSS 보안 아키텍처

WIA-CLINICAL-DECISION-SUPPORT 표준은 의료 환경에서 환자 안전, 데이터 개인정보 보호, 규제 준수 및 시스템 무결성을 다루는 임상의사결정지원 시스템을 위한 포괄적인 보안 조치를 정의합니다.

```typescript
// CDSS 보안 아키텍처
interface CDSSSecurityArchitecture {
  version: '1.0.0';

  securityDomains: {
    patientSafety: {
      description: 'CDSS가 환자에게 해를 끼치지 않도록 보장';
      scope: ['알고리즘 안전성', '경보 신뢰성', '안전장치 설계'];
      standards: ['ISO 14971', 'IEC 62304', 'FDA SaMD 지침'];
    };
    dataPrivacy: {
      description: '환자 건강 정보 보호';
      regulations: ['HIPAA', 'GDPR', 'PIPEDA', 'HITECH'];
      scope: ['PHI 보호', '동의 관리', '비식별화'];
    };
    systemSecurity: {
      description: 'CDSS 인프라 보호';
      standards: ['NIST 사이버보안 프레임워크', 'HITRUST CSF'];
      scope: ['접근 제어', '암호화', '감사', '네트워크 보안'];
    };
    regulatoryCompliance: {
      description: '규제 요구사항 충족';
      frameworks: ['FDA SaMD', 'EU MDR', 'ISO 13485'];
      scope: ['분류', '승인', '시판 후 감시'];
    };
  };

  riskManagement: {
    framework: 'ISO 14971';
    process: [
      '위험 식별',
      '위험 분석',
      '위험 평가',
      '위험 통제',
      '잔여 위험 평가',
      '위험 관리 보고서'
    ];
  };
}

// 보안 관리자 구현
class CDSSSecurityManager {
  private accessControl: AccessControlService;
  private encryption: EncryptionService;
  private auditLogger: AuditLogger;
  private riskManager: RiskManagementService;
  private complianceChecker: ComplianceService;

  async validateSecurityPosture(): Promise<SecurityPostureReport> {
    const assessments = await Promise.all([
      this.assessAccessControls(),
      this.assessEncryption(),
      this.assessAuditCapabilities(),
      this.assessComplianceStatus(),
      this.assessVulnerabilities()
    ]);

    return {
      timestamp: new Date(),
      overallScore: this.calculateOverallScore(assessments),
      accessControl: assessments[0],
      encryption: assessments[1],
      audit: assessments[2],
      compliance: assessments[3],
      vulnerabilities: assessments[4],
      recommendations: this.generateRecommendations(assessments)
    };
  }
}
```

### 7.2 환자 안전 프레임워크

```typescript
// CDSS 환자 안전 프레임워크
interface PatientSafetyFramework {
  designPrinciples: {
    failSafe: {
      description: '시스템 장애 시 환자에게 해가 되지 않아야 함';
      implementations: [
        '우아한 성능 저하',
        '보수적인 권장사항으로 기본 설정',
        '인간 무시 기능',
        '장애 시 경보 보존'
      ];
    };
    transparency: {
      description: '임상의가 AI 추론을 이해해야 함';
      implementations: [
        '설명 가능한 권장사항',
        '신뢰도 수준 표시',
        '데이터 소스 표시',
        '제한사항 전달'
      ];
    };
    validation: {
      description: 'AI 정확도의 지속적 검증';
      implementations: [
        '배포 전 검증',
        '실세계 성능 모니터링',
        '결과 추적',
        '편향 탐지'
      ];
    };
  };
}

// 알고리즘 안전 서비스
class AlgorithmSafetyService {
  private validationEngine: ValidationEngine;
  private performanceMonitor: PerformanceMonitor;
  private biasDetector: BiasDetector;
  private driftDetector: ModelDriftDetector;

  async validateAlgorithm(
    algorithm: CDSSAlgorithm
  ): Promise<AlgorithmValidationReport> {
    // 임상 검증
    const clinicalValidation = await this.validateClinicalPerformance(algorithm);

    // 안전성 검증
    const safetyValidation = await this.validateSafety(algorithm);

    // 편향 평가
    const biasAssessment = await this.assessBias(algorithm);

    // 경계 케이스 테스트
    const edgeCaseTesting = await this.testEdgeCases(algorithm);

    return {
      algorithmId: algorithm.id,
      algorithmVersion: algorithm.version,
      validationDate: new Date(),
      clinicalValidation,
      safetyValidation,
      biasAssessment,
      edgeCaseTesting,
      overallStatus: this.determineOverallStatus(
        clinicalValidation,
        safetyValidation,
        biasAssessment,
        edgeCaseTesting
      ),
      recommendations: this.generateSafetyRecommendations(
        clinicalValidation,
        safetyValidation,
        biasAssessment
      )
    };
  }

  private async assessBias(
    algorithm: CDSSAlgorithm
  ): Promise<BiasAssessmentResult> {
    const biasResults: BiasResult[] = [];

    // 인구통계적 편향 평가
    const demographicGroups = ['age', 'sex', 'race', 'ethnicity'];
    for (const dimension of demographicGroups) {
      const bias = await this.biasDetector.assessDemographicBias(
        algorithm,
        dimension
      );
      biasResults.push(bias);
    }

    // 사회경제적 편향
    const socioeconomicBias = await this.biasDetector.assessSocioeconomicBias(
      algorithm
    );
    biasResults.push(socioeconomicBias);

    // 지리적 편향
    const geographicBias = await this.biasDetector.assessGeographicBias(
      algorithm
    );
    biasResults.push(geographicBias);

    return {
      biasResults,
      overallBiasRisk: this.calculateOverallBiasRisk(biasResults),
      mitigationRecommendations: this.generateBiasMitigations(biasResults)
    };
  }

  async monitorRealTimePerformance(
    algorithmId: string
  ): Promise<RealTimePerformanceReport> {
    const recentPredictions = await this.getRecentPredictions(algorithmId, 24);
    const outcomes = await this.getLinkedOutcomes(recentPredictions);

    // 실시간 메트릭 계산
    const metrics = this.calculateRealTimeMetrics(recentPredictions, outcomes);

    // 드리프트 탐지
    const driftAnalysis = await this.driftDetector.analyze(
      algorithmId,
      metrics
    );

    // 베이스라인과 비교
    const comparison = this.compareToBaseline(algorithmId, metrics);

    // 필요시 경보 생성
    if (driftAnalysis.significantDrift || comparison.degraded) {
      await this.generatePerformanceAlert(algorithmId, driftAnalysis, comparison);
    }

    return {
      algorithmId,
      reportPeriod: { start: recentPredictions[0].timestamp, end: new Date() },
      predictionsAnalyzed: recentPredictions.length,
      metrics,
      driftAnalysis,
      baselineComparison: comparison,
      status: this.determinePerformanceStatus(driftAnalysis, comparison)
    };
  }
}

// 안전장치 시스템
class FailSafeSystem {
  private healthChecker: SystemHealthChecker;
  private fallbackService: FallbackService;
  private alertService: CriticalAlertService;

  async handleSystemFailure(failure: SystemFailure): Promise<FailureResponse> {
    // 장애 로깅
    await this.logFailure(failure);

    // 장애 심각도 결정
    const severity = this.assessFailureSeverity(failure);

    // 안전장치 절차 실행
    switch (severity) {
      case 'CRITICAL':
        return this.handleCriticalFailure(failure);
      case 'HIGH':
        return this.handleHighSeverityFailure(failure);
      case 'MEDIUM':
        return this.handleMediumSeverityFailure(failure);
      default:
        return this.handleLowSeverityFailure(failure);
    }
  }

  private async handleCriticalFailure(
    failure: SystemFailure
  ): Promise<FailureResponse> {
    // 영향받는 CDSS 기능 비활성화
    await this.disableAffectedFunctions(failure);

    // 임상 직원에게 알림
    await this.alertService.sendCriticalAlert({
      type: 'CDSS_CRITICAL_FAILURE',
      message: '임상의사결정지원 시스템이 일시적으로 사용 불가합니다',
      affectedFunctions: failure.affectedFunctions,
      recommendation: '수동 임상 판단을 사용하세요. 수정되면 CDSS가 재개됩니다.'
    });

    // 폴백 모드 활성화
    await this.fallbackService.enableFallbackMode(failure.affectedFunctions);

    // IT/지원팀에 알림
    await this.alertService.notifySupport(failure);

    return {
      status: 'FAIL_SAFE_ACTIVATED',
      fallbackMode: true,
      disabledFunctions: failure.affectedFunctions,
      estimatedRecovery: await this.estimateRecoveryTime(failure)
    };
  }

  async ensureAlertDelivery(alert: CriticalClinicalAlert): Promise<void> {
    // 중요 경보를 위한 다중 전달 채널
    const deliveryChannels = [
      this.deliverViaEHR(alert),
      this.deliverViaPager(alert),
      this.deliverViaSMS(alert),
      this.deliverViaPhone(alert)
    ];

    // 최소 하나의 성공적인 전달 대기
    const results = await Promise.allSettled(deliveryChannels);
    const successfulDeliveries = results.filter(r => r.status === 'fulfilled');

    if (successfulDeliveries.length === 0) {
      // 백업 연락처로 에스컬레이션
      await this.escalateAlert(alert);
    }

    // 전달 상태 로깅
    await this.logAlertDelivery(alert, results);
  }
}
```

### 7.3 데이터 개인정보 보호 및 HIPAA 준수

```typescript
// HIPAA 준수 프레임워크
interface HIPAAComplianceFramework {
  privacyRule: {
    minimumNecessary: MinimumNecessaryPolicy;
    authorizedDisclosure: DisclosurePolicy;
    patientRights: PatientRightsPolicy;
  };

  securityRule: {
    administrative: AdministrativeSafeguards;
    physical: PhysicalSafeguards;
    technical: TechnicalSafeguards;
  };

  breachNotification: {
    detection: BreachDetectionPolicy;
    assessment: RiskAssessmentPolicy;
    notification: NotificationPolicy;
  };
}

// 개인정보 보호 서비스 구현
class CDSSPrivacyService {
  private accessLogger: PHIAccessLogger;
  private consentManager: ConsentManager;
  private deidentifier: DeidentificationService;
  private encryptionService: EncryptionService;

  async accessPatientData(
    request: DataAccessRequest
  ): Promise<DataAccessResult> {
    // 권한 확인
    const authorization = await this.verifyAuthorization(request);
    if (!authorization.authorized) {
      await this.logUnauthorizedAccess(request);
      throw new UnauthorizedAccessError(authorization.reason);
    }

    // 동의 확인
    const consent = await this.consentManager.checkConsent(
      request.patientId,
      request.purpose
    );
    if (!consent.granted) {
      throw new ConsentRequiredError();
    }

    // 최소 필요 원칙 적용
    const allowedData = await this.applyMinimumNecessary(
      request.requestedData,
      request.purpose,
      request.userRole
    );

    // 접근 로깅
    await this.accessLogger.logAccess({
      userId: request.userId,
      patientId: request.patientId,
      dataAccessed: allowedData,
      purpose: request.purpose,
      timestamp: new Date(),
      consent: consent.consentId
    });

    return {
      authorized: true,
      allowedData,
      restrictions: this.getDataRestrictions(request.purpose)
    };
  }

  private async applyMinimumNecessary(
    requestedData: string[],
    purpose: string,
    userRole: string
  ): Promise<string[]> {
    // 역할 기반 데이터 접근 정책 가져오기
    const rolePolicy = await this.getRoleDataPolicy(userRole);

    // 목적 기반 데이터 요구사항 가져오기
    const purposeRequirements = await this.getPurposeRequirements(purpose);

    // 요청된, 역할 허용, 목적 필요의 교집합
    return requestedData.filter(
      d => rolePolicy.allowedData.includes(d) &&
           purposeRequirements.requiredData.includes(d)
    );
  }

  async deidentifyForResearch(
    patientData: PatientData,
    method: 'SAFE_HARBOR' | 'EXPERT_DETERMINATION'
  ): Promise<DeidentifiedData> {
    if (method === 'SAFE_HARBOR') {
      return this.applySafeHarborDeidentification(patientData);
    } else {
      return this.applyExpertDetermination(patientData);
    }
  }

  private async applySafeHarborDeidentification(
    data: PatientData
  ): Promise<DeidentifiedData> {
    const deidentified = { ...data };

    // 18개 HIPAA 식별자 제거
    // 1. 이름
    delete deidentified.name;

    // 2. 주보다 작은 지리적 데이터
    if (deidentified.address) {
      deidentified.address = {
        state: deidentified.address.state,
        // 인구 > 20,000인 경우 우편번호 첫 3자리만 유지
        zipCode: this.truncateZipCode(deidentified.address.zipCode)
      };
    }

    // 3. 개인 관련 날짜(연도 제외)
    deidentified.birthDate = this.generalizeDate(deidentified.birthDate);

    // 4-17. 기타 식별자 제거
    delete deidentified.ssn;
    delete deidentified.mrn;
    delete deidentified.phone;
    delete deidentified.fax;
    delete deidentified.email;
    delete deidentified.healthPlanId;
    delete deidentified.accountNumber;
    delete deidentified.licenseNumber;
    delete deidentified.vehicleIds;
    delete deidentified.deviceIds;
    delete deidentified.urls;
    delete deidentified.ipAddress;
    delete deidentified.biometrics;
    delete deidentified.photos;

    // 18. 기타 고유 식별 번호
    deidentified.id = this.generateDeidentifiedId();

    return {
      data: deidentified,
      method: 'SAFE_HARBOR',
      attestation: {
        method: 'Safe Harbor',
        identifiersRemoved: 18,
        date: new Date()
      }
    };
  }
}

// PHI 접근 로깅
class PHIAccessLogger {
  private logStorage: SecureLogStorage;

  async logAccess(access: PHIAccessEvent): Promise<void> {
    const logEntry: PHIAccessLog = {
      id: generateUUID(),
      timestamp: new Date(),
      userId: access.userId,
      userRole: access.userRole,
      patientId: access.patientId,
      dataType: access.dataAccessed,
      purpose: access.purpose,
      action: access.action,
      sourceIP: access.sourceIP,
      application: access.application,
      consentId: access.consent,
      outcome: access.outcome
    };

    // 변조 방지와 함께 안전하게 저장
    await this.logStorage.store(logEntry);

    // 의심스러운 패턴 확인
    await this.detectSuspiciousAccess(logEntry);
  }

  private async detectSuspiciousAccess(log: PHIAccessLog): Promise<void> {
    // 정규 근무 시간 외 접근 확인
    const isOutsideHours = this.isOutsideWorkingHours(log.timestamp);

    // 비정상적인 양 확인
    const recentAccessCount = await this.getRecentAccessCount(
      log.userId,
      60 // 분
    );
    const isUnusualVolume = recentAccessCount > 50;

    // 비상 접근 확인
    const isBreakTheGlass = log.purpose === 'EMERGENCY_OVERRIDE';

    // 자신의 기록 접근 확인
    const isSelfAccess = await this.checkSelfAccess(log.userId, log.patientId);

    if (isOutsideHours || isUnusualVolume || isBreakTheGlass || isSelfAccess) {
      await this.flagForReview({
        logEntry: log,
        flags: {
          outsideHours: isOutsideHours,
          unusualVolume: isUnusualVolume,
          breakTheGlass: isBreakTheGlass,
          selfAccess: isSelfAccess
        }
      });
    }
  }

  async generateAccessReport(
    patientId: string,
    dateRange: DateRange
  ): Promise<AccessReport> {
    const accessLogs = await this.logStorage.query({
      patientId,
      startDate: dateRange.start,
      endDate: dateRange.end
    });

    return {
      patientId,
      reportPeriod: dateRange,
      generatedAt: new Date(),
      totalAccesses: accessLogs.length,
      accessesByUser: this.groupByUser(accessLogs),
      accessesByPurpose: this.groupByPurpose(accessLogs),
      flaggedAccesses: accessLogs.filter(l => l.flagged),
      timeline: this.generateTimeline(accessLogs)
    };
  }
}
```

### 7.4 규제 준수

```typescript
// 규제 준수 프레임워크
interface RegulatoryComplianceFramework {
  fdaSaMD: {
    classification: SaMDClassification;
    premarket: PremarketRequirements;
    postmarket: PostmarketRequirements;
    qms: QualityManagementSystem;
  };

  euMDR: {
    classification: MDRClassification;
    conformityAssessment: ConformityAssessment;
    notifiedBody: NotifiedBodyRequirements;
    udi: UDIRequirements;
  };
}

// FDA SaMD 준수
class FDASaMDCompliance {
  async classifySoftware(
    software: CDSSSoftware
  ): Promise<SaMDClassificationResult> {
    // 의료 상황 상태 결정
    const healthcareSituation = this.assessHealthcareSituation(software);

    // 제공되는 정보의 중요성 결정
    const informationSignificance = this.assessInformationSignificance(software);

    // IEC 62304 분류 매트릭스 적용
    const classification = this.applyClassificationMatrix(
      healthcareSituation,
      informationSignificance
    );

    return {
      healthcareSituation,
      informationSignificance,
      classificationLevel: classification,
      regulatoryPathway: this.determineRegulatoryPathway(classification),
      requirements: this.getRequirements(classification)
    };
  }

  private assessHealthcareSituation(
    software: CDSSSoftware
  ): HealthcareSituation {
    // Critical: 생명 위협 또는 불가역적
    // Serious: 심각한 치료 개입을 초래할 수 있음
    // Non-serious: 기타 모든 상황

    if (software.intendedUse.includes('life-threatening') ||
        software.intendedUse.includes('ICU') ||
        software.intendedUse.includes('emergency')) {
      return 'CRITICAL';
    }

    if (software.intendedUse.includes('diagnosis') ||
        software.intendedUse.includes('treatment')) {
      return 'SERIOUS';
    }

    return 'NON_SERIOUS';
  }

  private determineRegulatoryPathway(
    classification: string
  ): RegulatoryPathway {
    switch (classification) {
      case 'CLASS_III':
        return {
          pathway: 'PMA',
          description: '시판 전 승인',
          requirements: ['임상시험', '전체 검토', '패널 회의 가능']
        };
      case 'CLASS_II':
        return {
          pathway: '510(k)',
          description: '시판 전 통지',
          requirements: ['실질적 동등성', '성능 테스트', '설계 관리']
        };
      case 'CLASS_I':
        return {
          pathway: 'EXEMPT',
          description: 'Class I 면제(일반 관리와 함께)',
          requirements: ['등록', '목록화', '일반 관리']
        };
      default:
        return {
          pathway: 'DETERMINATION_NEEDED',
          description: '사전 제출 권장',
          requirements: ['Q-Submission', 'FDA 지침 상담']
        };
    }
  }
}

// 시판 후 감시
class PostMarketSurveillance {
  private incidentReporter: IncidentReporter;
  private performanceMonitor: PerformanceMonitor;
  private complaintManager: ComplaintManager;

  async reportAdverseEvent(
    event: AdverseEvent
  ): Promise<AdverseEventReport> {
    // 보고 가능성 평가
    const assessment = this.assessReportability(event);

    // 보고 가능한 경우 MDR 생성
    if (assessment.reportable) {
      const mdr = await this.createMDR(event);

      // 요구되는 기간 내에 FDA에 제출
      if (assessment.expedited) {
        await this.submitExpeditedMDR(mdr); // 5일
      } else {
        await this.submitMDR(mdr); // 30일
      }

      return {
        event,
        assessment,
        mdrNumber: mdr.number,
        submissionDate: mdr.submissionDate,
        followUpRequired: assessment.followUpRequired
      };
    }

    // 추세 파악을 위해 비보고 이벤트 로깅
    await this.logNonReportableEvent(event);

    return {
      event,
      assessment,
      logged: true
    };
  }

  private assessReportability(event: AdverseEvent): ReportabilityAssessment {
    // FDA 보고 요구 사항:
    // 1. 사망
    // 2. 심각한 부상
    // 3. 사망/심각한 부상을 유발할 수 있는 오작동

    const isDeath = event.outcome === 'DEATH';
    const isSeriousInjury = event.outcome === 'SERIOUS_INJURY';
    const isMalfunction = event.type === 'MALFUNCTION' &&
                          event.potentialForSerious;

    const reportable = isDeath || isSeriousInjury || isMalfunction;
    const expedited = isDeath;

    return {
      reportable,
      expedited,
      reason: this.getReportabilityReason(isDeath, isSeriousInjury, isMalfunction),
      followUpRequired: reportable
    };
  }

  async conductPeriodicReview(): Promise<PeriodicSafetyReport> {
    // 마지막 검토 이후 데이터 수집
    const reportingPeriod = await this.getCurrentReportingPeriod();

    const [
      complaints,
      adverseEvents,
      performanceMetrics,
      algorithmUpdates,
      correctionActions
    ] = await Promise.all([
      this.complaintManager.getComplaints(reportingPeriod),
      this.incidentReporter.getEvents(reportingPeriod),
      this.performanceMonitor.getMetrics(reportingPeriod),
      this.getAlgorithmUpdates(reportingPeriod),
      this.getCAPAs(reportingPeriod)
    ]);

    return {
      reportingPeriod,
      generatedDate: new Date(),
      executiveSummary: this.generateExecutiveSummary(
        complaints,
        adverseEvents,
        performanceMetrics
      ),
      complaints: {
        total: complaints.length,
        byCategory: this.categorizeComplaints(complaints),
        trends: this.analyzeComplaintTrends(complaints),
        serious: complaints.filter(c => c.serious)
      },
      adverseEvents: {
        total: adverseEvents.length,
        deaths: adverseEvents.filter(e => e.outcome === 'DEATH').length,
        seriousInjuries: adverseEvents.filter(e => e.outcome === 'SERIOUS_INJURY').length,
        malfunctions: adverseEvents.filter(e => e.type === 'MALFUNCTION').length,
        analysis: this.analyzeAdverseEvents(adverseEvents)
      },
      performance: {
        metrics: performanceMetrics,
        trends: this.analyzePerformanceTrends(performanceMetrics),
        deviations: this.identifyDeviations(performanceMetrics)
      },
      algorithmUpdates: {
        total: algorithmUpdates.length,
        details: algorithmUpdates
      },
      capas: correctionActions,
      riskAssessment: await this.updateRiskAssessment(
        complaints,
        adverseEvents,
        performanceMetrics
      ),
      conclusions: this.generateConclusions(
        complaints,
        adverseEvents,
        performanceMetrics
      )
    };
  }
}
```

### 7.5 접근 제어 및 인증

```typescript
// CDSS 접근 제어 시스템
class CDSSAccessControlSystem {
  private roleManager: RoleManager;
  private permissionEngine: PermissionEngine;
  private mfaService: MFAService;
  private sessionManager: SessionManager;

  async authenticateUser(
    credentials: UserCredentials
  ): Promise<AuthenticationResult> {
    // 기본 인증
    const primaryAuth = await this.primaryAuthenticate(credentials);
    if (!primaryAuth.success) {
      await this.logFailedAttempt(credentials.username);
      return { success: false, reason: primaryAuth.reason };
    }

    // MFA 필요 여부 확인
    const user = primaryAuth.user;
    if (this.requiresMFA(user)) {
      return {
        success: false,
        requiresMFA: true,
        mfaMethods: await this.mfaService.getAvailableMethods(user.id),
        sessionToken: await this.createPendingMFASession(user)
      };
    }

    // 인증된 세션 생성
    const session = await this.sessionManager.createSession(user);

    return {
      success: true,
      user,
      session,
      permissions: await this.permissionEngine.getPermissions(user)
    };
  }

  async authorizeAction(
    session: Session,
    action: CDSSAction,
    context: ActionContext
  ): Promise<AuthorizationResult> {
    // 세션 확인
    const sessionValid = await this.sessionManager.validateSession(session);
    if (!sessionValid) {
      return { authorized: false, reason: '유효하지 않거나 만료된 세션' };
    }

    // 사용자 권한 가져오기
    const permissions = await this.permissionEngine.getPermissions(session.user);

    // 액션 권한 확인
    const hasPermission = this.checkPermission(permissions, action, context);
    if (!hasPermission) {
      await this.logUnauthorizedAction(session, action, context);
      return { authorized: false, reason: '권한 부족' };
    }

    // 컨텍스트별 권한 확인
    const contextAuth = await this.checkContextAuthorization(
      session.user,
      action,
      context
    );
    if (!contextAuth.authorized) {
      return contextAuth;
    }

    // 권한 있는 액션 로깅
    await this.logAuthorizedAction(session, action, context);

    return { authorized: true };
  }

  async defineRoles(): Promise<void> {
    const roles: CDSSRole[] = [
      {
        name: 'PHYSICIAN',
        description: '면허 의사',
        permissions: [
          'cdss:recommendations:view',
          'cdss:recommendations:accept',
          'cdss:recommendations:reject',
          'cdss:alerts:view',
          'cdss:alerts:acknowledge',
          'cdss:alerts:override',
          'cdss:patient:view',
          'cdss:guidelines:view',
          'cdss:calculators:execute'
        ]
      },
      {
        name: 'NURSE',
        description: '등록 간호사',
        permissions: [
          'cdss:recommendations:view',
          'cdss:alerts:view',
          'cdss:alerts:acknowledge',
          'cdss:patient:view',
          'cdss:guidelines:view',
          'cdss:calculators:execute'
        ]
      },
      {
        name: 'PHARMACIST',
        description: '임상 약사',
        permissions: [
          'cdss:recommendations:view',
          'cdss:recommendations:accept',
          'cdss:alerts:view',
          'cdss:alerts:acknowledge',
          'cdss:alerts:override',
          'cdss:patient:view',
          'cdss:drug:view',
          'cdss:drug:interaction_check'
        ]
      },
      {
        name: 'CDSS_ADMIN',
        description: 'CDSS 시스템 관리자',
        permissions: [
          'cdss:rules:manage',
          'cdss:guidelines:manage',
          'cdss:alerts:configure',
          'cdss:reports:view',
          'cdss:audit:view',
          'cdss:users:manage'
        ]
      }
    ];

    for (const role of roles) {
      await this.roleManager.createRole(role);
    }
  }
}
```

### 7.6 감사 및 규정 준수 모니터링

```typescript
// 포괄적 감사 시스템
class CDSSAuditSystem {
  private auditStorage: ImmutableAuditStorage;
  private complianceMonitor: ComplianceMonitor;
  private reportGenerator: AuditReportGenerator;

  async logCDSSEvent(event: CDSSAuditEvent): Promise<void> {
    const auditEntry: AuditEntry = {
      id: generateUUID(),
      timestamp: new Date(),
      eventType: event.type,
      actor: {
        userId: event.userId,
        userRole: event.userRole,
        ipAddress: event.ipAddress,
        sessionId: event.sessionId
      },
      action: event.action,
      resource: event.resource,
      patient: event.patientId ? { id: event.patientId } : undefined,
      details: event.details,
      outcome: event.outcome,
      hash: ''  // 불변 저장소에서 설정됨
    };

    // 변조 방지 로그에 저장
    await this.auditStorage.store(auditEntry);
  }

  async generateComplianceReport(
    framework: 'HIPAA' | 'GDPR' | 'SOC2',
    period: DateRange
  ): Promise<ComplianceReport> {
    const requirements = this.getComplianceRequirements(framework);
    const assessments: ComplianceAssessment[] = [];

    for (const requirement of requirements) {
      const assessment = await this.assessCompliance(requirement, period);
      assessments.push(assessment);
    }

    const overallScore = this.calculateComplianceScore(assessments);

    return {
      framework,
      period,
      generatedDate: new Date(),
      overallScore,
      status: overallScore >= 0.95 ? 'COMPLIANT' : 'NON_COMPLIANT',
      assessments,
      findings: assessments.filter(a => !a.compliant),
      remediationPlan: this.generateRemediationPlan(
        assessments.filter(a => !a.compliant)
      )
    };
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT 보안**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
