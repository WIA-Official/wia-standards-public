# Chapter 7: Clinical Decision Support Security

## Patient Safety, Privacy, and Regulatory Compliance

### 7.1 CDSS Security Architecture

The WIA-CLINICAL-DECISION-SUPPORT standard defines comprehensive security measures for clinical decision support systems, addressing patient safety, data privacy, regulatory compliance, and system integrity in healthcare environments.

```typescript
// CDSS Security Architecture
interface CDSSSecurityArchitecture {
  version: '1.0.0';

  securityDomains: {
    patientSafety: {
      description: 'Ensuring CDSS does not harm patients';
      scope: ['Algorithm safety', 'Alert reliability', 'Fail-safe design'];
      standards: ['ISO 14971', 'IEC 62304', 'FDA SaMD guidance'];
    };
    dataPrivacy: {
      description: 'Protecting patient health information';
      regulations: ['HIPAA', 'GDPR', 'PIPEDA', 'HITECH'];
      scope: ['PHI protection', 'Consent management', 'De-identification'];
    };
    systemSecurity: {
      description: 'Protecting CDSS infrastructure';
      standards: ['NIST Cybersecurity Framework', 'HITRUST CSF'];
      scope: ['Access control', 'Encryption', 'Audit', 'Network security'];
    };
    regulatoryCompliance: {
      description: 'Meeting regulatory requirements';
      frameworks: ['FDA SaMD', 'EU MDR', 'ISO 13485'];
      scope: ['Classification', 'Approval', 'Post-market surveillance'];
    };
  };

  riskManagement: {
    framework: 'ISO 14971';
    process: [
      'Risk identification',
      'Risk analysis',
      'Risk evaluation',
      'Risk control',
      'Residual risk assessment',
      'Risk management report'
    ];
  };
}

// Security Manager Implementation
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

### 7.2 Patient Safety Framework

```typescript
// Patient Safety Framework for CDSS
interface PatientSafetyFramework {
  designPrinciples: {
    failSafe: {
      description: 'System failures should not cause patient harm';
      implementations: [
        'Graceful degradation',
        'Default to conservative recommendations',
        'Human override capabilities',
        'Alert preservation during failures'
      ];
    };
    transparency: {
      description: 'Clinicians understand AI reasoning';
      implementations: [
        'Explainable recommendations',
        'Confidence levels displayed',
        'Data sources shown',
        'Limitations communicated'
      ];
    };
    validation: {
      description: 'Continuous validation of AI accuracy';
      implementations: [
        'Pre-deployment validation',
        'Real-world performance monitoring',
        'Outcome tracking',
        'Bias detection'
      ];
    };
  };
}

// Algorithm Safety Service
class AlgorithmSafetyService {
  private validationEngine: ValidationEngine;
  private performanceMonitor: PerformanceMonitor;
  private biasDetector: BiasDetector;
  private driftDetector: ModelDriftDetector;

  async validateAlgorithm(
    algorithm: CDSSAlgorithm
  ): Promise<AlgorithmValidationReport> {
    // Clinical validation
    const clinicalValidation = await this.validateClinicalPerformance(algorithm);

    // Safety validation
    const safetyValidation = await this.validateSafety(algorithm);

    // Bias assessment
    const biasAssessment = await this.assessBias(algorithm);

    // Edge case testing
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

  private async validateClinicalPerformance(
    algorithm: CDSSAlgorithm
  ): Promise<ClinicalValidationResult> {
    // Load validation dataset
    const validationData = await this.loadValidationDataset(algorithm.id);

    // Run algorithm on validation data
    const predictions = await this.runPredictions(algorithm, validationData);

    // Calculate performance metrics
    const metrics = this.calculateMetrics(predictions, validationData.outcomes);

    // Compare to acceptance criteria
    const meetsAcceptanceCriteria = this.checkAcceptanceCriteria(
      metrics,
      algorithm.acceptanceCriteria
    );

    return {
      metrics: {
        sensitivity: metrics.sensitivity,
        specificity: metrics.specificity,
        ppv: metrics.ppv,
        npv: metrics.npv,
        auc: metrics.auc,
        calibration: metrics.calibration
      },
      acceptanceCriteria: algorithm.acceptanceCriteria,
      meetsAcceptanceCriteria,
      subgroupPerformance: await this.analyzeSubgroupPerformance(
        predictions,
        validationData
      )
    };
  }

  private async assessBias(
    algorithm: CDSSAlgorithm
  ): Promise<BiasAssessmentResult> {
    const biasResults: BiasResult[] = [];

    // Demographic bias assessment
    const demographicGroups = ['age', 'sex', 'race', 'ethnicity'];
    for (const dimension of demographicGroups) {
      const bias = await this.biasDetector.assessDemographicBias(
        algorithm,
        dimension
      );
      biasResults.push(bias);
    }

    // Socioeconomic bias
    const socioeconomicBias = await this.biasDetector.assessSocioeconomicBias(
      algorithm
    );
    biasResults.push(socioeconomicBias);

    // Geographic bias
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

    // Calculate real-time metrics
    const metrics = this.calculateRealTimeMetrics(recentPredictions, outcomes);

    // Detect drift
    const driftAnalysis = await this.driftDetector.analyze(
      algorithmId,
      metrics
    );

    // Compare to baseline
    const comparison = this.compareToBaseline(algorithmId, metrics);

    // Generate alerts if needed
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

// Fail-Safe System
class FailSafeSystem {
  private healthChecker: SystemHealthChecker;
  private fallbackService: FallbackService;
  private alertService: CriticalAlertService;

  async handleSystemFailure(failure: SystemFailure): Promise<FailureResponse> {
    // Log failure
    await this.logFailure(failure);

    // Determine failure severity
    const severity = this.assessFailureSeverity(failure);

    // Execute fail-safe procedures
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
    // Disable affected CDSS functions
    await this.disableAffectedFunctions(failure);

    // Notify clinical staff
    await this.alertService.sendCriticalAlert({
      type: 'CDSS_CRITICAL_FAILURE',
      message: 'Clinical Decision Support System is temporarily unavailable',
      affectedFunctions: failure.affectedFunctions,
      recommendation: 'Please use manual clinical judgment. CDSS will resume when fixed.'
    });

    // Enable fallback mode
    await this.fallbackService.enableFallbackMode(failure.affectedFunctions);

    // Notify IT/support
    await this.alertService.notifySupport(failure);

    return {
      status: 'FAIL_SAFE_ACTIVATED',
      fallbackMode: true,
      disabledFunctions: failure.affectedFunctions,
      estimatedRecovery: await this.estimateRecoveryTime(failure)
    };
  }

  async ensureAlertDelivery(alert: CriticalClinicalAlert): Promise<void> {
    // Multiple delivery channels for critical alerts
    const deliveryChannels = [
      this.deliverViaEHR(alert),
      this.deliverViaPager(alert),
      this.deliverViaSMS(alert),
      this.deliverViaPhone(alert)
    ];

    // Wait for at least one successful delivery
    const results = await Promise.allSettled(deliveryChannels);
    const successfulDeliveries = results.filter(r => r.status === 'fulfilled');

    if (successfulDeliveries.length === 0) {
      // Escalate to backup contacts
      await this.escalateAlert(alert);
    }

    // Log delivery status
    await this.logAlertDelivery(alert, results);
  }
}
```

### 7.3 Data Privacy and HIPAA Compliance

```typescript
// HIPAA Compliance Framework
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

// Privacy Service Implementation
class CDSSPrivacyService {
  private accessLogger: PHIAccessLogger;
  private consentManager: ConsentManager;
  private deidentifier: DeidentificationService;
  private encryptionService: EncryptionService;

  async accessPatientData(
    request: DataAccessRequest
  ): Promise<DataAccessResult> {
    // Verify authorization
    const authorization = await this.verifyAuthorization(request);
    if (!authorization.authorized) {
      await this.logUnauthorizedAccess(request);
      throw new UnauthorizedAccessError(authorization.reason);
    }

    // Check consent
    const consent = await this.consentManager.checkConsent(
      request.patientId,
      request.purpose
    );
    if (!consent.granted) {
      throw new ConsentRequiredError();
    }

    // Apply minimum necessary
    const allowedData = await this.applyMinimumNecessary(
      request.requestedData,
      request.purpose,
      request.userRole
    );

    // Log access
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
    // Get role-based data access policy
    const rolePolicy = await this.getRoleDataPolicy(userRole);

    // Get purpose-based data requirements
    const purposeRequirements = await this.getPurposeRequirements(purpose);

    // Intersection of requested, role-allowed, and purpose-required
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

    // Remove 18 HIPAA identifiers
    // 1. Names
    delete deidentified.name;

    // 2. Geographic data smaller than state
    if (deidentified.address) {
      deidentified.address = {
        state: deidentified.address.state,
        // Keep only first 3 digits of ZIP if population > 20,000
        zipCode: this.truncateZipCode(deidentified.address.zipCode)
      };
    }

    // 3. Dates (except year) for dates related to individual
    deidentified.birthDate = this.generalizeDate(deidentified.birthDate);

    // 4-17. Remove other identifiers
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

    // 18. Any other unique identifying number
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

// PHI Access Logging
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

    // Store securely with tamper protection
    await this.logStorage.store(logEntry);

    // Check for suspicious patterns
    await this.detectSuspiciousAccess(logEntry);
  }

  private async detectSuspiciousAccess(log: PHIAccessLog): Promise<void> {
    // Check for access outside normal hours
    const isOutsideHours = this.isOutsideWorkingHours(log.timestamp);

    // Check for unusual volume
    const recentAccessCount = await this.getRecentAccessCount(
      log.userId,
      60 // minutes
    );
    const isUnusualVolume = recentAccessCount > 50;

    // Check for break-the-glass access
    const isBreakTheGlass = log.purpose === 'EMERGENCY_OVERRIDE';

    // Check for accessing own record
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

### 7.4 Regulatory Compliance

```typescript
// Regulatory Compliance Framework
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

// FDA SaMD Compliance
class FDASaMDCompliance {
  async classifySoftware(
    software: CDSSSoftware
  ): Promise<SaMDClassificationResult> {
    // Determine state of healthcare situation
    const healthcareSituation = this.assessHealthcareSituation(software);

    // Determine significance of information provided
    const informationSignificance = this.assessInformationSignificance(software);

    // Apply IEC 62304 classification matrix
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
    // Critical: Life-threatening or irreversible
    // Serious: Could result in significant treatment intervention
    // Non-serious: All other situations

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
          description: 'Premarket Approval',
          requirements: ['Clinical trials', 'Full review', 'Panel meeting possible']
        };
      case 'CLASS_II':
        return {
          pathway: '510(k)',
          description: 'Premarket Notification',
          requirements: ['Substantial equivalence', 'Performance testing', 'Design controls']
        };
      case 'CLASS_I':
        return {
          pathway: 'EXEMPT',
          description: 'Class I Exempt (with general controls)',
          requirements: ['Registration', 'Listing', 'General controls']
        };
      default:
        return {
          pathway: 'DETERMINATION_NEEDED',
          description: 'Pre-submission recommended',
          requirements: ['Q-Submission', 'FDA guidance consultation']
        };
    }
  }

  async generatePresubmission(
    software: CDSSSoftware
  ): Promise<PresubmissionDocument> {
    const classification = await this.classifySoftware(software);

    return {
      deviceDescription: this.generateDeviceDescription(software),
      intendedUse: software.intendedUse,
      indications: software.indications,
      classification: classification,
      proposedRegulatoryPathway: classification.regulatoryPathway,
      clinicalDataPlan: this.generateClinicalDataPlan(software, classification),
      performanceTestingPlan: this.generatePerformanceTestingPlan(software),
      questions: this.generateFDAQuestions(software, classification)
    };
  }
}

// Post-Market Surveillance
class PostMarketSurveillance {
  private incidentReporter: IncidentReporter;
  private performanceMonitor: PerformanceMonitor;
  private complaintManager: ComplaintManager;

  async reportAdverseEvent(
    event: AdverseEvent
  ): Promise<AdverseEventReport> {
    // Assess reportability
    const assessment = this.assessReportability(event);

    // Create MDR if reportable
    if (assessment.reportable) {
      const mdr = await this.createMDR(event);

      // Submit to FDA within required timeframe
      if (assessment.expedited) {
        await this.submitExpeditedMDR(mdr); // 5 days
      } else {
        await this.submitMDR(mdr); // 30 days
      }

      return {
        event,
        assessment,
        mdrNumber: mdr.number,
        submissionDate: mdr.submissionDate,
        followUpRequired: assessment.followUpRequired
      };
    }

    // Log non-reportable event for trending
    await this.logNonReportableEvent(event);

    return {
      event,
      assessment,
      logged: true
    };
  }

  private assessReportability(event: AdverseEvent): ReportabilityAssessment {
    // FDA requires reporting of:
    // 1. Deaths
    // 2. Serious injuries
    // 3. Malfunctions that could cause death/serious injury

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
    // Collect data since last review
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

### 7.5 Access Control and Authentication

```typescript
// CDSS Access Control System
class CDSSAccessControlSystem {
  private roleManager: RoleManager;
  private permissionEngine: PermissionEngine;
  private mfaService: MFAService;
  private sessionManager: SessionManager;

  async authenticateUser(
    credentials: UserCredentials
  ): Promise<AuthenticationResult> {
    // Primary authentication
    const primaryAuth = await this.primaryAuthenticate(credentials);
    if (!primaryAuth.success) {
      await this.logFailedAttempt(credentials.username);
      return { success: false, reason: primaryAuth.reason };
    }

    // Check if MFA required
    const user = primaryAuth.user;
    if (this.requiresMFA(user)) {
      return {
        success: false,
        requiresMFA: true,
        mfaMethods: await this.mfaService.getAvailableMethods(user.id),
        sessionToken: await this.createPendingMFASession(user)
      };
    }

    // Create authenticated session
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
    // Verify session
    const sessionValid = await this.sessionManager.validateSession(session);
    if (!sessionValid) {
      return { authorized: false, reason: 'Invalid or expired session' };
    }

    // Get user permissions
    const permissions = await this.permissionEngine.getPermissions(session.user);

    // Check action permission
    const hasPermission = this.checkPermission(permissions, action, context);
    if (!hasPermission) {
      await this.logUnauthorizedAction(session, action, context);
      return { authorized: false, reason: 'Insufficient permissions' };
    }

    // Check context-specific authorization
    const contextAuth = await this.checkContextAuthorization(
      session.user,
      action,
      context
    );
    if (!contextAuth.authorized) {
      return contextAuth;
    }

    // Log authorized action
    await this.logAuthorizedAction(session, action, context);

    return { authorized: true };
  }

  private async checkContextAuthorization(
    user: User,
    action: CDSSAction,
    context: ActionContext
  ): Promise<AuthorizationResult> {
    // Patient-level authorization
    if (context.patientId) {
      const hasPatientAccess = await this.checkPatientAccess(
        user,
        context.patientId
      );
      if (!hasPatientAccess) {
        return {
          authorized: false,
          reason: 'No access to patient record'
        };
      }
    }

    // Department-level authorization
    if (context.departmentId) {
      const hasDeptAccess = await this.checkDepartmentAccess(
        user,
        context.departmentId
      );
      if (!hasDeptAccess) {
        return {
          authorized: false,
          reason: 'No access to department'
        };
      }
    }

    // Time-based restrictions
    if (action.requiresWorkingHours) {
      const isWorkingHours = this.isWithinWorkingHours(user);
      if (!isWorkingHours) {
        return {
          authorized: false,
          reason: 'Action restricted to working hours'
        };
      }
    }

    return { authorized: true };
  }

  async defineRoles(): Promise<void> {
    const roles: CDSSRole[] = [
      {
        name: 'PHYSICIAN',
        description: 'Licensed physician',
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
        description: 'Registered nurse',
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
        description: 'Clinical pharmacist',
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
        description: 'CDSS system administrator',
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

### 7.6 Audit and Compliance Monitoring

```typescript
// Comprehensive Audit System
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
      hash: ''  // Will be set by immutable storage
    };

    // Store in tamper-evident log
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

  private async assessCompliance(
    requirement: ComplianceRequirement,
    period: DateRange
  ): Promise<ComplianceAssessment> {
    switch (requirement.type) {
      case 'ACCESS_CONTROL':
        return this.assessAccessControlCompliance(requirement, period);
      case 'AUDIT_LOGGING':
        return this.assessAuditLoggingCompliance(requirement, period);
      case 'DATA_ENCRYPTION':
        return this.assessEncryptionCompliance(requirement);
      case 'INCIDENT_RESPONSE':
        return this.assessIncidentResponseCompliance(requirement, period);
      case 'TRAINING':
        return this.assessTrainingCompliance(requirement, period);
      default:
        throw new Error(`Unknown requirement type: ${requirement.type}`);
    }
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT Security**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
