# Chapter 6: Clinical Decision Support Integration

## EHR Integration, Workflow Embedding, and Interoperability

### 6.1 Integration Architecture Overview

The WIA-CLINICAL-DECISION-SUPPORT standard defines comprehensive integration patterns for embedding clinical decision support into healthcare workflows. Effective integration is critical for CDSS adoption and impact.

```typescript
// CDSS Integration Architecture
interface CDSSIntegrationArchitecture {
  version: '1.0.0';

  integrationPatterns: {
    ehrIntegration: {
      standards: ['SMART on FHIR', 'CDS Hooks', 'HL7 FHIR'];
      approaches: ['Native', 'App-based', 'API'];
    };
    workflowIntegration: {
      patterns: ['Synchronous', 'Asynchronous', 'Background'];
      triggers: ['User-initiated', 'Event-driven', 'Scheduled'];
    };
    dataIntegration: {
      sources: ['EHR', 'Lab systems', 'Imaging', 'Devices'];
      methods: ['Real-time', 'Batch', 'Streaming'];
    };
  };

  integrationLayers: {
    presentation: 'UI components embedded in EHR';
    service: 'API-based CDSS services';
    data: 'FHIR-based data exchange';
    knowledge: 'Shareable knowledge artifacts';
  };
}
```

### 6.2 SMART on FHIR Integration

```typescript
// SMART on FHIR Application Framework
interface SMARTOnFHIRIntegration {
  appManifest: SMARTAppManifest;
  launchContext: LaunchContext;
  authorization: SMARTAuthorization;
  dataAccess: FHIRDataAccess;
}

interface SMARTAppManifest {
  client_name: string;
  client_uri: string;
  logo_uri: string;
  redirect_uris: string[];
  scope: string;
  grant_types: string[];
  response_types: string[];
  token_endpoint_auth_method: string;

  // SMART specific
  launch_uri: string;
  fhir_versions: string[];
  smart_capabilities: SMARTCapability[];
}

type SMARTCapability =
  | 'launch-ehr'
  | 'launch-standalone'
  | 'client-public'
  | 'client-confidential-symmetric'
  | 'sso-openid-connect'
  | 'context-ehr-patient'
  | 'context-ehr-encounter'
  | 'context-standalone-patient'
  | 'permission-patient'
  | 'permission-user'
  | 'permission-offline';

// SMART Launch Handler
class SMARTLaunchHandler {
  async handleEHRLaunch(launchParams: LaunchParams): Promise<LaunchResult> {
    // Validate launch parameters
    this.validateLaunchParams(launchParams);

    // Exchange authorization code for tokens
    const tokens = await this.exchangeCodeForTokens(
      launchParams.code,
      launchParams.state
    );

    // Extract launch context
    const context = this.extractLaunchContext(tokens);

    // Initialize FHIR client
    const fhirClient = this.initializeFHIRClient(
      tokens.access_token,
      context.fhirServer
    );

    // Load patient context
    const patientContext = await this.loadPatientContext(
      fhirClient,
      context.patient
    );

    return {
      tokens,
      context,
      patientContext,
      fhirClient
    };
  }

  private async loadPatientContext(
    fhirClient: FHIRClient,
    patientId: string
  ): Promise<PatientContext> {
    // Parallel fetch of patient data
    const [patient, conditions, medications, allergies, labs, vitals] = await Promise.all([
      fhirClient.read({ resourceType: 'Patient', id: patientId }),
      fhirClient.search({
        resourceType: 'Condition',
        params: { patient: patientId, 'clinical-status': 'active' }
      }),
      fhirClient.search({
        resourceType: 'MedicationRequest',
        params: { patient: patientId, status: 'active' }
      }),
      fhirClient.search({
        resourceType: 'AllergyIntolerance',
        params: { patient: patientId, 'clinical-status': 'active' }
      }),
      fhirClient.search({
        resourceType: 'Observation',
        params: {
          patient: patientId,
          category: 'laboratory',
          _sort: '-date',
          _count: 100
        }
      }),
      fhirClient.search({
        resourceType: 'Observation',
        params: {
          patient: patientId,
          category: 'vital-signs',
          _sort: '-date',
          _count: 50
        }
      })
    ]);

    return {
      patient: this.transformPatient(patient),
      conditions: this.transformConditions(conditions.entry),
      medications: this.transformMedications(medications.entry),
      allergies: this.transformAllergies(allergies.entry),
      recentLabs: this.transformLabs(labs.entry),
      vitals: this.transformVitals(vitals.entry)
    };
  }
}

// SMART App Component (React)
const CDSSSmartApp: React.FC = () => {
  const { tokens, context, fhirClient } = useSMARTLaunch();
  const [recommendations, setRecommendations] = useState<Recommendation[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function loadRecommendations() {
      if (!context.patient) return;

      setLoading(true);
      try {
        // Get patient data
        const patientData = await loadPatientData(fhirClient, context.patient);

        // Request recommendations from CDSS
        const cdssResponse = await cdssService.getRecommendations({
          patientId: context.patient,
          encounterId: context.encounter,
          patientData,
          queryType: 'COMPREHENSIVE_REVIEW'
        });

        setRecommendations(cdssResponse.recommendations);
      } catch (error) {
        console.error('Failed to load recommendations:', error);
      } finally {
        setLoading(false);
      }
    }

    loadRecommendations();
  }, [context.patient, fhirClient]);

  if (loading) {
    return <LoadingSpinner />;
  }

  return (
    <div className="cdss-app">
      <PatientHeader patient={context.patient} />
      <RecommendationList
        recommendations={recommendations}
        onAction={handleRecommendationAction}
      />
      <AlertPanel
        patientId={context.patient}
        encounterId={context.encounter}
      />
    </div>
  );
};
```

### 6.3 CDS Hooks Integration

```typescript
// CDS Hooks Service Registration
interface CDSHooksServiceRegistration {
  services: CDSService[];
  discovery: ServiceDiscovery;
  prefetch: PrefetchTemplates;
}

// CDS Service Definition
const drugInteractionService: CDSService = {
  hook: 'order-sign',
  title: 'Drug Interaction Checker',
  description: 'Checks for clinically significant drug-drug interactions',
  id: 'drug-interaction-checker',
  usageRequirements: 'Requires active medication list access',

  prefetch: {
    patient: 'Patient/{{context.patientId}}',
    activeMedications: 'MedicationRequest?patient={{context.patientId}}&status=active',
    allergies: 'AllergyIntolerance?patient={{context.patientId}}&clinical-status=active'
  }
};

// CDS Hooks Server Implementation
class CDSHooksServer {
  private services: Map<string, CDSServiceHandler> = new Map();

  constructor() {
    this.registerService('drug-interaction-checker', new DrugInteractionService());
    this.registerService('dosing-checker', new DosingCheckerService());
    this.registerService('guideline-reminder', new GuidelineReminderService());
    this.registerService('risk-assessment', new RiskAssessmentService());
  }

  @Get('/cds-services')
  getDiscovery(): CDSDiscoveryResponse {
    return {
      services: Array.from(this.services.values()).map(s => s.getServiceDefinition())
    };
  }

  @Post('/cds-services/:serviceId')
  async handleHook(
    @Param('serviceId') serviceId: string,
    @Body() request: CDSHookRequest
  ): Promise<CDSHookResponse> {
    const service = this.services.get(serviceId);
    if (!service) {
      throw new NotFoundException(`Service ${serviceId} not found`);
    }

    // Validate request
    this.validateRequest(request);

    // Execute service
    const startTime = Date.now();
    const response = await service.handle(request);

    // Log execution
    await this.logExecution(serviceId, request, response, Date.now() - startTime);

    return response;
  }
}

// Drug Interaction CDS Service
class DrugInteractionService implements CDSServiceHandler {
  private interactionDatabase: DrugInteractionDatabase;
  private alertRules: AlertRuleEngine;

  async handle(request: CDSHookRequest): Promise<CDSHookResponse> {
    const cards: Card[] = [];

    // Extract medications from context
    const draftMedications = this.extractDraftMedications(request.context.draftOrders);
    const activeMedications = this.extractActiveMedications(request.prefetch?.activeMedications);
    const allergies = this.extractAllergies(request.prefetch?.allergies);

    // Check drug-drug interactions
    for (const draftMed of draftMedications) {
      // Check against active medications
      for (const activeMed of activeMedications) {
        const interaction = await this.interactionDatabase.checkInteraction(
          draftMed.rxnormCode,
          activeMed.rxnormCode
        );

        if (interaction && this.isSignificant(interaction)) {
          cards.push(this.createInteractionCard(draftMed, activeMed, interaction));
        }
      }

      // Check drug-allergy interactions
      for (const allergy of allergies) {
        const allergyInteraction = await this.checkAllergyInteraction(
          draftMed,
          allergy
        );

        if (allergyInteraction) {
          cards.push(this.createAllergyCard(draftMed, allergy, allergyInteraction));
        }
      }
    }

    // Sort by severity
    cards.sort((a, b) => this.severityOrder(b.indicator) - this.severityOrder(a.indicator));

    return { cards };
  }

  private createInteractionCard(
    drug1: DraftMedication,
    drug2: ActiveMedication,
    interaction: DrugInteraction
  ): Card {
    return {
      uuid: generateUUID(),
      summary: `${interaction.severity}: ${drug1.name} + ${drug2.name}`,
      detail: this.formatInteractionDetail(interaction),
      indicator: this.mapSeverityToIndicator(interaction.severity),
      source: {
        label: 'Drug Interaction Database',
        url: 'https://dailymed.nlm.nih.gov'
      },
      suggestions: this.generateSuggestions(drug1, interaction),
      overrideReasons: [
        {
          code: 'patient-tolerated',
          display: 'Patient has tolerated this combination previously'
        },
        {
          code: 'benefit-outweighs-risk',
          display: 'Clinical benefit outweighs potential risk'
        },
        {
          code: 'will-monitor',
          display: 'Will monitor patient closely'
        },
        {
          code: 'short-term',
          display: 'Short-term use only'
        }
      ],
      links: [
        {
          label: 'View interaction details',
          url: `https://www.drugs.com/interactions/${drug1.name}+${drug2.name}`,
          type: 'absolute'
        }
      ]
    };
  }

  private generateSuggestions(
    drug: DraftMedication,
    interaction: DrugInteraction
  ): Suggestion[] {
    const suggestions: Suggestion[] = [];

    // Alternative medications
    if (interaction.alternatives && interaction.alternatives.length > 0) {
      for (const alt of interaction.alternatives) {
        suggestions.push({
          label: `Use ${alt.name} instead`,
          uuid: generateUUID(),
          isRecommended: alt.preferred,
          actions: [{
            type: 'update',
            description: `Replace ${drug.name} with ${alt.name}`,
            resource: this.createMedicationRequest(alt, drug)
          }]
        });
      }
    }

    // Dose adjustment
    if (interaction.doseAdjustment) {
      suggestions.push({
        label: `Reduce ${drug.name} dose to ${interaction.doseAdjustment.recommendedDose}`,
        uuid: generateUUID(),
        actions: [{
          type: 'update',
          description: 'Adjust dose per interaction recommendation',
          resource: this.adjustDose(drug, interaction.doseAdjustment)
        }]
      });
    }

    // Monitoring
    if (interaction.monitoringRequired) {
      suggestions.push({
        label: `Add monitoring: ${interaction.monitoringRequired.parameters.join(', ')}`,
        uuid: generateUUID(),
        actions: interaction.monitoringRequired.orders.map(order => ({
          type: 'create',
          description: `Order ${order.display}`,
          resource: order
        }))
      });
    }

    return suggestions;
  }
}
```

### 6.4 Workflow Integration Patterns

```typescript
// Clinical Workflow Integration
interface WorkflowIntegrationPatterns {
  syncPatterns: {
    orderEntry: OrderEntryCDSS;
    documentation: DocumentationCDSS;
    review: ChartReviewCDSS;
  };

  asyncPatterns: {
    backgroundAnalysis: BackgroundCDSS;
    batchProcessing: BatchCDSS;
    scheduledReview: ScheduledCDSS;
  };

  eventDrivenPatterns: {
    labResultTrigger: LabResultCDSS;
    admissionTrigger: AdmissionCDSS;
    dischargeTrigger: DischargeCDSS;
  };
}

// Order Entry Workflow Integration
class OrderEntryWorkflowIntegration {
  private cdsHooksClient: CDSHooksClient;
  private alertManager: AlertManager;

  async onOrderSelect(
    order: DraftOrder,
    context: OrderContext
  ): Promise<OrderSelectResult> {
    // Call order-select hook
    const hookResponse = await this.cdsHooksClient.callHook('order-select', {
      context: {
        userId: context.userId,
        patientId: context.patientId,
        encounterId: context.encounterId,
        selections: [order.id],
        draftOrders: { resourceType: 'Bundle', entry: [{ resource: order }] }
      }
    });

    return {
      cards: hookResponse.cards,
      shouldBlock: hookResponse.cards.some(c => c.indicator === 'critical'),
      suggestions: this.extractSuggestions(hookResponse.cards)
    };
  }

  async onOrderSign(
    orders: DraftOrder[],
    context: OrderContext
  ): Promise<OrderSignResult> {
    // Call order-sign hook
    const hookResponse = await this.cdsHooksClient.callHook('order-sign', {
      context: {
        userId: context.userId,
        patientId: context.patientId,
        encounterId: context.encounterId,
        draftOrders: {
          resourceType: 'Bundle',
          entry: orders.map(o => ({ resource: o }))
        }
      }
    });

    // Categorize alerts
    const criticalAlerts = hookResponse.cards.filter(c => c.indicator === 'critical');
    const warningAlerts = hookResponse.cards.filter(c => c.indicator === 'warning');
    const infoAlerts = hookResponse.cards.filter(c => c.indicator === 'info');

    return {
      canProceed: criticalAlerts.length === 0,
      requiresOverride: criticalAlerts.length > 0 || warningAlerts.length > 0,
      criticalAlerts,
      warningAlerts,
      infoAlerts,
      systemActions: hookResponse.systemActions
    };
  }

  async handleAlertOverride(
    alert: Card,
    overrideReason: OverrideReason,
    context: OrderContext
  ): Promise<OverrideResult> {
    // Validate override reason
    if (!alert.overrideReasons?.find(r => r.code === overrideReason.code)) {
      throw new InvalidOverrideReasonError();
    }

    // Record override
    const overrideRecord = await this.alertManager.recordOverride({
      alertId: alert.uuid,
      alertType: alert.source.topic?.code,
      severity: alert.indicator,
      overrideReason: overrideReason,
      userId: context.userId,
      patientId: context.patientId,
      encounterId: context.encounterId,
      timestamp: new Date()
    });

    // Trigger any required follow-up
    if (this.requiresFollowUp(alert, overrideReason)) {
      await this.scheduleFollowUp(alert, overrideRecord, context);
    }

    return {
      success: true,
      overrideId: overrideRecord.id,
      followUpRequired: this.requiresFollowUp(alert, overrideReason)
    };
  }
}

// Background Analysis Service
class BackgroundAnalysisService {
  private analysisQueue: AnalysisQueue;
  private patientAnalyzer: PatientAnalyzer;
  private notificationService: NotificationService;

  async schedulePatientAnalysis(
    patientId: string,
    analysisTypes: AnalysisType[]
  ): Promise<AnalysisJob> {
    return this.analysisQueue.enqueue({
      patientId,
      analysisTypes,
      priority: 'normal',
      scheduledTime: new Date()
    });
  }

  @Process('patient-analysis')
  async processAnalysis(job: AnalysisJob): Promise<AnalysisResult> {
    const patientData = await this.loadPatientData(job.patientId);
    const results: AnalysisResult[] = [];

    for (const analysisType of job.analysisTypes) {
      const result = await this.runAnalysis(analysisType, patientData);
      results.push(result);

      // Check for actionable findings
      if (result.requiresAction) {
        await this.handleActionableResult(result, patientData);
      }
    }

    return {
      jobId: job.id,
      patientId: job.patientId,
      completedAt: new Date(),
      results
    };
  }

  private async handleActionableResult(
    result: AnalysisResult,
    patientData: PatientData
  ): Promise<void> {
    // Create clinical alert
    const alert = await this.createBackgroundAlert(result, patientData);

    // Notify appropriate care team members
    const careTeam = await this.getCareTeam(patientData.patientId);
    await this.notificationService.notifyCareTeam(careTeam, alert);

    // Add to patient's alert list
    await this.alertManager.addAlert(alert);
  }
}

// Event-Driven Integration
class EventDrivenCDSSIntegration {
  private eventBus: EventBus;
  private cdssServices: Map<string, CDSSService>;

  constructor() {
    this.registerEventHandlers();
  }

  private registerEventHandlers() {
    // Lab result events
    this.eventBus.subscribe('lab.result.final', async (event: LabResultEvent) => {
      await this.handleLabResult(event);
    });

    // Admission events
    this.eventBus.subscribe('admission.complete', async (event: AdmissionEvent) => {
      await this.handleAdmission(event);
    });

    // Vital sign events
    this.eventBus.subscribe('vital.recorded', async (event: VitalSignEvent) => {
      await this.handleVitalSign(event);
    });

    // Medication administration events
    this.eventBus.subscribe('medication.administered', async (event: MedAdminEvent) => {
      await this.handleMedicationAdministration(event);
    });
  }

  private async handleLabResult(event: LabResultEvent): Promise<void> {
    const { patientId, result, encounterId } = event;

    // Check for critical values
    if (this.isCriticalValue(result)) {
      await this.handleCriticalValue(result, patientId, encounterId);
    }

    // Check for trending abnormalities
    const trend = await this.checkLabTrend(result, patientId);
    if (trend.significant) {
      await this.handleLabTrend(trend, patientId, encounterId);
    }

    // Trigger relevant CDSS analyses
    const applicableAnalyses = this.getApplicableAnalyses(result);
    for (const analysis of applicableAnalyses) {
      await this.cdssServices.get(analysis)?.analyze(patientId, result);
    }
  }

  private async handleCriticalValue(
    result: LabResult,
    patientId: string,
    encounterId?: string
  ): Promise<void> {
    // Create critical alert
    const alert: CriticalAlert = {
      id: generateUUID(),
      type: 'CRITICAL_LAB_VALUE',
      severity: 'CRITICAL',
      patientId,
      encounterId,
      title: `Critical ${result.code.display} Value`,
      message: `${result.code.display}: ${result.value} ${result.unit} (Critical Range: ${result.referenceRange})`,
      value: result.value,
      referenceRange: result.referenceRange,
      timestamp: new Date(),
      requiresAcknowledgment: true
    };

    // Send immediate notifications
    await this.sendCriticalNotifications(alert, patientId);

    // Log for compliance
    await this.logCriticalValue(alert);
  }

  private async handleVitalSign(event: VitalSignEvent): Promise<void> {
    const { patientId, vital, encounterId } = event;

    // Calculate early warning score
    const recentVitals = await this.getRecentVitals(patientId);
    const ewsScore = this.calculateEWS([...recentVitals, vital]);

    // Check for deterioration
    if (ewsScore.total >= this.ewsThreshold) {
      await this.handleDeteriorationAlert(ewsScore, patientId, encounterId);
    }

    // Update real-time monitoring
    await this.updateRealTimeMonitoring(patientId, vital, ewsScore);
  }
}
```

### 6.5 EHR-Specific Integration

```typescript
// Epic EHR Integration
class EpicIntegration implements EHRIntegration {
  private epicClient: EpicFHIRClient;
  private smartAuth: SMARTAuth;

  async initialize(config: EpicConfig): Promise<void> {
    // Configure Epic FHIR endpoint
    this.epicClient = new EpicFHIRClient({
      baseUrl: config.fhirEndpoint,
      clientId: config.clientId,
      privateKey: config.privateKey
    });

    // Set up SMART on FHIR
    this.smartAuth = new SMARTAuth({
      authEndpoint: config.authEndpoint,
      tokenEndpoint: config.tokenEndpoint,
      clientId: config.clientId,
      scopes: [
        'patient/*.read',
        'user/*.read',
        'launch',
        'openid',
        'fhirUser'
      ]
    });
  }

  async launchFromEHR(launchToken: string, issuer: string): Promise<LaunchContext> {
    // Exchange launch token
    const tokens = await this.smartAuth.exchangeLaunchToken(launchToken);

    // Extract context from token
    const context = this.extractContext(tokens.id_token);

    // Initialize FHIR client with access token
    this.epicClient.setAccessToken(tokens.access_token);

    return {
      patientId: context.patient,
      encounterId: context.encounter,
      userId: context.fhirUser,
      fhirClient: this.epicClient
    };
  }

  // Epic-specific: Best Practice Alerts integration
  async registerBestPracticeAlert(
    alertDefinition: BestPracticeAlertDefinition
  ): Promise<string> {
    // Epic BPA registration through App Orchard
    return await this.epicClient.createCustomResource(
      'BestPracticeAlert',
      alertDefinition
    );
  }

  // Epic-specific: Access patient via MyChart proxy
  async getPatientProxyAccess(
    patientId: string
  ): Promise<ProxyAccess[]> {
    const response = await this.epicClient.search({
      resourceType: 'RelatedPerson',
      params: { patient: patientId }
    });

    return response.entry?.map(e => this.transformProxyAccess(e.resource)) || [];
  }
}

// Cerner (Oracle Health) Integration
class CernerIntegration implements EHRIntegration {
  private cernerClient: CernerFHIRClient;

  async initialize(config: CernerConfig): Promise<void> {
    this.cernerClient = new CernerFHIRClient({
      baseUrl: config.fhirEndpoint,
      clientId: config.clientId,
      clientSecret: config.clientSecret
    });
  }

  // Cerner-specific: MPages integration
  async launchMPage(mpageConfig: MPageConfig): Promise<MPageContext> {
    return {
      patientId: mpageConfig.patientId,
      encounterId: mpageConfig.encounterId,
      positionId: mpageConfig.positionId,
      personnelId: mpageConfig.personnelId
    };
  }

  // Cerner-specific: PowerChart integration
  async getPowerChartContext(): Promise<PowerChartContext> {
    // Integration with PowerChart via DiscernReportingFramework
    const context = await this.cernerClient.getContext();
    return {
      patientId: context.patient,
      encounterId: context.encounter,
      organizationId: context.organization,
      facility: context.facility
    };
  }
}

// Meditech Integration
class MeditechIntegration implements EHRIntegration {
  private meditechClient: MeditechClient;

  async initialize(config: MeditechConfig): Promise<void> {
    // Meditech uses different integration approach
    this.meditechClient = new MeditechClient({
      environment: config.environment,
      apiKey: config.apiKey
    });
  }

  // Meditech Expanse FHIR R4 integration
  async launchFromExpanse(context: ExpanseContext): Promise<LaunchContext> {
    const fhirClient = new MeditechFHIRClient({
      baseUrl: context.fhirEndpoint,
      accessToken: context.accessToken
    });

    return {
      patientId: context.patientId,
      encounterId: context.visitId,
      userId: context.userId,
      fhirClient
    };
  }
}
```

### 6.6 Integration Testing and Monitoring

```typescript
// Integration Testing Framework
class CDSSIntegrationTester {
  private testSuites: Map<string, TestSuite> = new Map();

  async runIntegrationTests(
    ehrSystem: string,
    environment: 'sandbox' | 'staging' | 'production'
  ): Promise<TestResults> {
    const testSuite = this.testSuites.get(ehrSystem);
    if (!testSuite) {
      throw new Error(`No test suite for ${ehrSystem}`);
    }

    const results: TestResult[] = [];

    // Authentication tests
    results.push(await this.testAuthentication(testSuite));

    // Data access tests
    results.push(await this.testDataAccess(testSuite));

    // CDS Hooks tests
    results.push(await this.testCDSHooks(testSuite));

    // Performance tests
    results.push(await this.testPerformance(testSuite));

    // Error handling tests
    results.push(await this.testErrorHandling(testSuite));

    return {
      ehrSystem,
      environment,
      timestamp: new Date(),
      results,
      overallStatus: this.calculateOverallStatus(results)
    };
  }

  private async testCDSHooks(testSuite: TestSuite): Promise<TestResult> {
    const hookTests = [
      this.testPatientViewHook(testSuite),
      this.testOrderSelectHook(testSuite),
      this.testOrderSignHook(testSuite)
    ];

    const hookResults = await Promise.all(hookTests);

    return {
      category: 'CDS Hooks',
      tests: hookResults,
      passed: hookResults.every(r => r.passed),
      duration: hookResults.reduce((sum, r) => sum + r.duration, 0)
    };
  }

  private async testOrderSignHook(testSuite: TestSuite): Promise<TestCase> {
    const startTime = Date.now();

    try {
      // Test with known drug interaction
      const response = await testSuite.client.callHook('order-sign', {
        context: {
          userId: testSuite.testUser,
          patientId: testSuite.testPatient,
          draftOrders: testSuite.testOrdersWithInteraction
        }
      });

      // Verify interaction detected
      const interactionCard = response.cards.find(
        c => c.source.topic?.code === 'drug-interaction'
      );

      return {
        name: 'order-sign hook - drug interaction detection',
        passed: interactionCard !== undefined,
        duration: Date.now() - startTime,
        details: interactionCard ? 'Interaction correctly detected' : 'Interaction not detected'
      };

    } catch (error) {
      return {
        name: 'order-sign hook - drug interaction detection',
        passed: false,
        duration: Date.now() - startTime,
        error: error.message
      };
    }
  }
}

// Integration Monitoring
class IntegrationMonitor {
  private metrics: MetricsCollector;
  private alerting: AlertingService;

  async monitorIntegration(integrationId: string): Promise<void> {
    // Collect metrics
    const metrics = await this.collectIntegrationMetrics(integrationId);

    // Check thresholds
    await this.checkThresholds(integrationId, metrics);

    // Record metrics
    await this.metrics.record(integrationId, metrics);
  }

  private async collectIntegrationMetrics(
    integrationId: string
  ): Promise<IntegrationMetrics> {
    return {
      // Availability
      availability: await this.measureAvailability(integrationId),

      // Latency
      latency: {
        p50: await this.measureLatencyP50(integrationId),
        p95: await this.measureLatencyP95(integrationId),
        p99: await this.measureLatencyP99(integrationId)
      },

      // Throughput
      requestsPerMinute: await this.measureThroughput(integrationId),

      // Error rate
      errorRate: await this.measureErrorRate(integrationId),

      // Hook-specific metrics
      hookMetrics: {
        'patient-view': await this.measureHookMetrics(integrationId, 'patient-view'),
        'order-select': await this.measureHookMetrics(integrationId, 'order-select'),
        'order-sign': await this.measureHookMetrics(integrationId, 'order-sign')
      }
    };
  }

  private async checkThresholds(
    integrationId: string,
    metrics: IntegrationMetrics
  ): Promise<void> {
    // Availability threshold
    if (metrics.availability < 0.999) {
      await this.alerting.sendAlert({
        severity: 'HIGH',
        integration: integrationId,
        metric: 'availability',
        value: metrics.availability,
        threshold: 0.999
      });
    }

    // Latency threshold
    if (metrics.latency.p95 > 500) {
      await this.alerting.sendAlert({
        severity: 'MEDIUM',
        integration: integrationId,
        metric: 'latency_p95',
        value: metrics.latency.p95,
        threshold: 500
      });
    }

    // Error rate threshold
    if (metrics.errorRate > 0.01) {
      await this.alerting.sendAlert({
        severity: 'HIGH',
        integration: integrationId,
        metric: 'error_rate',
        value: metrics.errorRate,
        threshold: 0.01
      });
    }
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT Integration**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
