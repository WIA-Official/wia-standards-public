# Chapter 4: Clinical Decision Support API Interface

## Service APIs and Integration Patterns

### 4.1 CDSS API Architecture

The WIA-CLINICAL-DECISION-SUPPORT standard defines comprehensive APIs for integrating clinical decision support into healthcare workflows. These APIs support real-time decision support, asynchronous analysis, and bidirectional communication with clinical systems.

```typescript
// CDSS API Architecture Definition
interface CDSSAPIArchitecture {
  version: '1.0.0';

  apiStyles: {
    rest: {
      description: 'RESTful APIs for synchronous operations';
      useCases: ['Point-of-care queries', 'Alert processing', 'Configuration'];
    };
    cdsHooks: {
      description: 'HL7 CDS Hooks for EHR integration';
      useCases: ['Order entry support', 'Patient chart review'];
    };
    fhir: {
      description: 'FHIR-native operations';
      useCases: ['Knowledge artifacts', 'Clinical data access'];
    };
    graphql: {
      description: 'GraphQL for flexible queries';
      useCases: ['Dashboard data', 'Complex aggregations'];
    };
    streaming: {
      description: 'Real-time event streaming';
      useCases: ['Continuous monitoring', 'Early warning systems'];
    };
  };

  authentication: {
    oauth2: 'Primary authentication mechanism';
    smartOnFhir: 'SMART on FHIR for EHR launch';
    mtls: 'Mutual TLS for service-to-service';
    apiKeys: 'For backend service integration';
  };

  rateLimit: {
    standard: '1000 requests/minute';
    burst: '100 requests/second';
    premium: '10000 requests/minute';
  };
}
```

### 4.2 Core CDSS REST API

```typescript
// Core CDSS API Specification
interface CDSSRestAPI {
  baseUrl: 'https://api.cdss.wia.org/v1';

  endpoints: {
    recommendations: RecommendationEndpoints;
    alerts: AlertEndpoints;
    knowledge: KnowledgeEndpoints;
    analytics: AnalyticsEndpoints;
    audit: AuditEndpoints;
  };
}

// Recommendation API Endpoints
interface RecommendationEndpoints {
  // Get clinical recommendations
  getRecommendation: {
    method: 'POST';
    path: '/recommendations';
    description: 'Request clinical decision support recommendation';
  };

  // Get recommendation by ID
  getRecommendationById: {
    method: 'GET';
    path: '/recommendations/{recommendationId}';
    description: 'Retrieve a specific recommendation';
  };

  // Update recommendation status
  updateRecommendationStatus: {
    method: 'PATCH';
    path: '/recommendations/{recommendationId}/status';
    description: 'Record action taken on recommendation';
  };
}

// Recommendation Request
interface RecommendationRequest {
  // Patient identification
  patient: {
    id: string;
    identifiers?: Identifier[];
  };

  // Clinical context
  context: {
    encounterId?: string;
    encounterType?: string;
    department?: string;
    careTeam?: Reference[];
    urgency: 'ROUTINE' | 'URGENT' | 'EMERGENT';
  };

  // What kind of support is needed
  query: {
    type: QueryType;
    focus?: CodeableConcept;
    additionalContext?: string;
  };

  // User making the request
  requestor: {
    userId: string;
    role: string;
    specialty?: string;
  };

  // Data to include
  dataScope?: {
    includeMedications: boolean;
    includeProblems: boolean;
    includeAllergies: boolean;
    includeLabs: boolean;
    includeVitals: boolean;
    labLookbackDays?: number;
    vitalLookbackHours?: number;
  };

  // Response preferences
  preferences?: {
    explanationLevel: 'BRIEF' | 'STANDARD' | 'DETAILED';
    maxRecommendations?: number;
    includeAlternatives: boolean;
    includeReferences: boolean;
  };
}

type QueryType =
  | 'DIAGNOSIS_DIFFERENTIAL'
  | 'TREATMENT_OPTIONS'
  | 'DRUG_SELECTION'
  | 'DOSING_RECOMMENDATION'
  | 'SCREENING_DUE'
  | 'RISK_ASSESSMENT'
  | 'LAB_INTERPRETATION'
  | 'IMAGING_INTERPRETATION'
  | 'GUIDELINE_COMPLIANCE'
  | 'COMPREHENSIVE_REVIEW';

// Recommendation Response
interface RecommendationResponse {
  recommendationId: string;
  status: 'COMPLETE' | 'PARTIAL' | 'ERROR';
  timestamp: string;

  // Query echo
  query: {
    type: QueryType;
    focus?: CodeableConcept;
  };

  // The recommendations
  recommendations: Recommendation[];

  // Overall confidence
  confidence: {
    score: number;  // 0-1
    level: 'HIGH' | 'MEDIUM' | 'LOW';
    factors: ConfidenceFactor[];
  };

  // Data quality assessment
  dataQuality: {
    completeness: number;
    timeliness: number;
    missingCriticalData?: string[];
  };

  // Explanation
  explanation: {
    summary: string;
    reasoning?: string;
    keyFactors: string[];
  };

  // References
  references?: Reference[];

  // Warnings/caveats
  warnings?: Warning[];

  // Validity
  validUntil: string;

  // Links
  _links: {
    self: string;
    patient: string;
    feedback: string;
  };
}

interface Recommendation {
  id: string;
  rank: number;
  type: RecommendationType;

  // What is recommended
  action: {
    description: string;
    code?: CodeableConcept;
    resource?: Resource;  // FHIR resource (e.g., MedicationRequest)
  };

  // Strength
  strength: 'STRONG' | 'MODERATE' | 'WEAK' | 'CONDITIONAL';
  urgency: 'IMMEDIATE' | 'WITHIN_HOURS' | 'WITHIN_DAYS' | 'ROUTINE';

  // Rationale
  rationale: string;
  keyEvidence: string[];

  // Evidence level
  evidenceLevel: {
    grade: 'A' | 'B' | 'C' | 'D' | 'I';
    description: string;
  };

  // Contraindications (if any)
  contraindications?: string[];

  // Monitoring
  monitoring?: {
    parameters: string[];
    frequency: string;
    duration: string;
  };

  // Alternatives
  alternatives?: Alternative[];
}

// CDSS API Implementation
class CDSSAPIService {
  private recommendationEngine: RecommendationEngine;
  private dataService: PatientDataService;
  private auditService: AuditService;

  @Post('/recommendations')
  @UseGuards(AuthGuard, RateLimitGuard)
  @ApiOperation({ summary: 'Get clinical recommendation' })
  async getRecommendation(
    @Body() request: RecommendationRequest,
    @CurrentUser() user: User
  ): Promise<RecommendationResponse> {
    const startTime = Date.now();

    try {
      // Validate request
      this.validateRequest(request);

      // Get patient data
      const patientData = await this.dataService.getPatientData(
        request.patient.id,
        request.dataScope
      );

      // Generate recommendations
      const result = await this.recommendationEngine.generateRecommendations(
        patientData,
        request.query,
        request.context
      );

      // Build response
      const response = this.buildResponse(result, request);

      // Audit log
      await this.auditService.logRecommendationRequest(
        request,
        response,
        user,
        Date.now() - startTime
      );

      return response;

    } catch (error) {
      await this.auditService.logError(request, error, user);
      throw this.handleError(error);
    }
  }

  @Patch('/recommendations/:id/status')
  @ApiOperation({ summary: 'Update recommendation status' })
  async updateStatus(
    @Param('id') recommendationId: string,
    @Body() update: StatusUpdate,
    @CurrentUser() user: User
  ): Promise<StatusUpdateResponse> {
    // Record the action taken
    const result = await this.recommendationEngine.recordAction(
      recommendationId,
      update,
      user
    );

    // This feedback can be used for learning
    if (update.outcome) {
      await this.feedbackService.recordOutcome(
        recommendationId,
        update.outcome
      );
    }

    return result;
  }
}

interface StatusUpdate {
  action: 'ACCEPTED' | 'MODIFIED' | 'REJECTED' | 'DEFERRED';
  reason?: string;
  modifications?: string;
  outcome?: {
    outcomeType: string;
    outcomeDate?: string;
    notes?: string;
  };
}
```

### 4.3 Alert API

```typescript
// Alert API Specification
interface AlertAPI {
  // Process potential alert trigger
  processAlert: {
    method: 'POST';
    path: '/alerts/evaluate';
  };

  // Get active alerts for patient
  getAlerts: {
    method: 'GET';
    path: '/alerts/patient/{patientId}';
  };

  // Acknowledge alert
  acknowledgeAlert: {
    method: 'POST';
    path: '/alerts/{alertId}/acknowledge';
  };

  // Override alert
  overrideAlert: {
    method: 'POST';
    path: '/alerts/{alertId}/override';
  };

  // Get alert statistics
  getAlertStats: {
    method: 'GET';
    path: '/alerts/statistics';
  };
}

// Alert Evaluation Request
interface AlertEvaluationRequest {
  trigger: {
    type: AlertTriggerType;
    source: string;
    data: any;
  };
  patient: {
    id: string;
  };
  context: {
    encounterId?: string;
    userId: string;
    userRole: string;
  };
}

type AlertTriggerType =
  | 'MEDICATION_ORDER'
  | 'LAB_RESULT'
  | 'VITAL_SIGN'
  | 'DIAGNOSIS_ENTRY'
  | 'ORDER_ENTRY'
  | 'DOCUMENTATION'
  | 'SCHEDULED_CHECK';

// Alert Response
interface AlertEvaluationResponse {
  alerts: Alert[];
  evaluationId: string;
  timestamp: string;
  processingTime: number;
}

interface Alert {
  alertId: string;
  category: string;
  severity: 'INFO' | 'WARNING' | 'HIGH' | 'CRITICAL';
  title: string;
  message: string;
  detail?: string;

  // Source
  source: {
    system: string;
    rule: string;
    version: string;
  };

  // Patient
  patient: {
    id: string;
    mrn?: string;
  };

  // Triggered by
  trigger: {
    type: string;
    resource?: Resource;
  };

  // Actions
  recommendedAction?: string;
  suggestions?: AlertSuggestion[];

  // Override options
  overrideAllowed: boolean;
  overrideReasons?: OverrideReason[];

  // Links
  links?: {
    moreInfo?: string;
    guideline?: string;
    similar?: string;
  };

  // Timing
  createdAt: string;
  expiresAt?: string;

  // Status
  status: 'ACTIVE' | 'ACKNOWLEDGED' | 'OVERRIDDEN' | 'EXPIRED' | 'AUTO_RESOLVED';
}

// Alert Service Implementation
class AlertAPIService {
  private alertEngine: AlertEngine;
  private alertRepository: AlertRepository;

  @Post('/alerts/evaluate')
  async evaluateAlerts(
    @Body() request: AlertEvaluationRequest
  ): Promise<AlertEvaluationResponse> {
    const startTime = Date.now();

    // Get patient context
    const patientContext = await this.getPatientContext(request.patient.id);

    // Evaluate alert rules
    const alerts = await this.alertEngine.evaluate(
      request.trigger,
      patientContext,
      request.context
    );

    // Apply suppression rules
    const filteredAlerts = await this.applySuppressionRules(
      alerts,
      request.patient.id,
      request.context.userId
    );

    // Store alerts
    await this.alertRepository.storeAlerts(filteredAlerts);

    return {
      alerts: filteredAlerts,
      evaluationId: generateUUID(),
      timestamp: new Date().toISOString(),
      processingTime: Date.now() - startTime
    };
  }

  @Post('/alerts/:alertId/override')
  async overrideAlert(
    @Param('alertId') alertId: string,
    @Body() override: AlertOverride,
    @CurrentUser() user: User
  ): Promise<OverrideResponse> {
    // Validate override is allowed
    const alert = await this.alertRepository.getAlert(alertId);
    if (!alert.overrideAllowed) {
      throw new ForbiddenException('This alert cannot be overridden');
    }

    // Validate reason
    if (alert.overrideReasons) {
      const validReason = alert.overrideReasons.find(
        r => r.code === override.reasonCode
      );
      if (!validReason) {
        throw new BadRequestException('Invalid override reason');
      }
    }

    // Record override
    const result = await this.alertRepository.recordOverride(
      alertId,
      override,
      user
    );

    // Audit log (important for safety)
    await this.auditService.logAlertOverride(
      alert,
      override,
      user
    );

    return result;
  }

  @Get('/alerts/statistics')
  async getAlertStatistics(
    @Query() params: AlertStatsParams
  ): Promise<AlertStatistics> {
    return {
      period: params.period,
      totalAlerts: await this.getAlertCount(params),
      bySeverity: await this.getAlertsBySeverity(params),
      byCategory: await this.getAlertsByCategory(params),
      overrideRate: await this.calculateOverrideRate(params),
      acknowledgeTime: await this.calculateAcknowledgeTime(params),
      topAlertTypes: await this.getTopAlertTypes(params),
      alertFatigueTrend: await this.calculateAlertFatigueTrend(params)
    };
  }
}

interface AlertStatistics {
  period: DateRange;
  totalAlerts: number;
  bySeverity: {
    critical: number;
    high: number;
    warning: number;
    info: number;
  };
  byCategory: { [category: string]: number };
  overrideRate: {
    overall: number;
    bySeverity: { [severity: string]: number };
    byCategory: { [category: string]: number };
  };
  acknowledgeTime: {
    median: number;
    p90: number;
    bySeverity: { [severity: string]: number };
  };
  topAlertTypes: { type: string; count: number; overrideRate: number }[];
  alertFatigueTrend: { date: string; alertsPerUser: number }[];
}
```

### 4.4 Knowledge Management API

```typescript
// Knowledge Management API
interface KnowledgeAPI {
  // Guidelines
  listGuidelines: {
    method: 'GET';
    path: '/knowledge/guidelines';
  };
  getGuideline: {
    method: 'GET';
    path: '/knowledge/guidelines/{guidelineId}';
  };
  evaluateGuideline: {
    method: 'POST';
    path: '/knowledge/guidelines/{guidelineId}/evaluate';
  };

  // Alert Rules
  listAlertRules: {
    method: 'GET';
    path: '/knowledge/alert-rules';
  };
  getAlertRule: {
    method: 'GET';
    path: '/knowledge/alert-rules/{ruleId}';
  };
  createAlertRule: {
    method: 'POST';
    path: '/knowledge/alert-rules';
  };
  updateAlertRule: {
    method: 'PUT';
    path: '/knowledge/alert-rules/{ruleId}';
  };

  // Order Sets
  listOrderSets: {
    method: 'GET';
    path: '/knowledge/order-sets';
  };
  getOrderSet: {
    method: 'GET';
    path: '/knowledge/order-sets/{orderSetId}';
  };

  // Calculators
  listCalculators: {
    method: 'GET';
    path: '/knowledge/calculators';
  };
  executeCalculator: {
    method: 'POST';
    path: '/knowledge/calculators/{calculatorId}/execute';
  };
}

// Guideline Evaluation
interface GuidelineEvaluationRequest {
  patientId: string;
  guidelineId: string;
  evaluationContext?: {
    encounterId?: string;
    assessmentDate?: string;
  };
  returnActions: boolean;
}

interface GuidelineEvaluationResponse {
  guidelineId: string;
  guidelineTitle: string;
  evaluationTimestamp: string;
  patientId: string;

  // Applicability
  applicable: boolean;
  applicabilityReason?: string;

  // Compliance assessment
  compliance: {
    overall: 'COMPLIANT' | 'PARTIALLY_COMPLIANT' | 'NON_COMPLIANT' | 'NOT_APPLICABLE';
    score: number;  // 0-100
    gaps: ComplianceGap[];
  };

  // Recommended actions
  recommendedActions?: GuidelineAction[];

  // Evidence
  evidence: {
    dataUsed: string[];
    assumptions: string[];
  };
}

interface ComplianceGap {
  recommendation: string;
  status: 'MET' | 'NOT_MET' | 'PARTIALLY_MET' | 'NOT_ASSESSED';
  reason?: string;
  suggestedAction?: string;
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
}

// Calculator Execution
interface CalculatorExecutionRequest {
  calculatorId: string;
  inputs: { [inputId: string]: any };
  patientId?: string;  // For auto-population
}

interface CalculatorExecutionResponse {
  calculatorId: string;
  calculatorName: string;
  executionTimestamp: string;

  // Inputs used
  inputs: {
    id: string;
    name: string;
    value: any;
    source: 'PROVIDED' | 'AUTO_POPULATED' | 'DEFAULT';
  }[];

  // Result
  result: {
    value: number | string;
    unit?: string;
    formattedValue: string;
  };

  // Interpretation
  interpretation?: {
    category: string;
    description: string;
    recommendation?: string;
    riskLevel?: 'LOW' | 'MODERATE' | 'HIGH' | 'VERY_HIGH';
  };

  // Confidence
  confidence?: {
    level: 'HIGH' | 'MEDIUM' | 'LOW';
    limitations: string[];
  };

  // References
  references?: Reference[];
}

// Knowledge Service Implementation
class KnowledgeAPIService {
  private guidelineRepository: GuidelineRepository;
  private guidelineEngine: GuidelineEngine;
  private calculatorService: CalculatorService;

  @Get('/knowledge/guidelines')
  async listGuidelines(
    @Query() params: GuidelineSearchParams
  ): Promise<GuidelineListResponse> {
    const guidelines = await this.guidelineRepository.search({
      category: params.category,
      specialty: params.specialty,
      status: params.status ?? 'active',
      search: params.search,
      pagination: {
        page: params.page ?? 1,
        pageSize: params.pageSize ?? 20
      }
    });

    return {
      guidelines: guidelines.items.map(g => ({
        id: g.id,
        title: g.title,
        version: g.version,
        category: g.category,
        publisher: g.publisher,
        lastUpdated: g.lastUpdated,
        status: g.status
      })),
      pagination: {
        page: guidelines.page,
        pageSize: guidelines.pageSize,
        total: guidelines.total,
        totalPages: Math.ceil(guidelines.total / guidelines.pageSize)
      }
    };
  }

  @Post('/knowledge/guidelines/:id/evaluate')
  async evaluateGuideline(
    @Param('id') guidelineId: string,
    @Body() request: GuidelineEvaluationRequest
  ): Promise<GuidelineEvaluationResponse> {
    // Get guideline
    const guideline = await this.guidelineRepository.getById(guidelineId);
    if (!guideline) {
      throw new NotFoundException('Guideline not found');
    }

    // Get patient data
    const patientData = await this.patientDataService.getPatientData(
      request.patientId
    );

    // Evaluate
    const evaluation = await this.guidelineEngine.evaluate(
      guideline,
      patientData,
      request.evaluationContext
    );

    return evaluation;
  }

  @Post('/knowledge/calculators/:id/execute')
  async executeCalculator(
    @Param('id') calculatorId: string,
    @Body() request: CalculatorExecutionRequest
  ): Promise<CalculatorExecutionResponse> {
    // Get calculator definition
    const calculator = await this.calculatorRepository.getById(calculatorId);
    if (!calculator) {
      throw new NotFoundException('Calculator not found');
    }

    // Auto-populate inputs if patient provided
    let inputs = { ...request.inputs };
    if (request.patientId) {
      inputs = await this.autoPopulateInputs(
        calculator,
        request.patientId,
        inputs
      );
    }

    // Validate inputs
    this.validateCalculatorInputs(calculator, inputs);

    // Execute
    const result = await this.calculatorService.execute(calculator, inputs);

    return result;
  }
}
```

### 4.5 Streaming API for Real-Time CDSS

```typescript
// Real-Time Streaming API
interface CDSSStreamingAPI {
  protocols: {
    websocket: 'wss://api.cdss.wia.org/v1/stream';
    sse: 'https://api.cdss.wia.org/v1/events';
    mqtt: 'mqtts://mqtt.cdss.wia.org';
  };

  channels: {
    patientAlerts: PatientAlertChannel;
    vitalSignMonitoring: VitalSignChannel;
    earlyWarning: EarlyWarningChannel;
    orderRecommendations: OrderRecommendationChannel;
  };
}

// WebSocket Implementation
class CDSSWebSocketService {
  private connections: Map<string, WebSocket> = new Map();
  private subscriptions: Map<string, Set<string>> = new Map();

  @WebSocketGateway('/stream')
  handleConnection(client: WebSocket, request: IncomingMessage) {
    const userId = this.authenticateConnection(request);
    this.connections.set(userId, client);

    client.on('message', (message) => this.handleMessage(userId, message));
    client.on('close', () => this.handleDisconnect(userId));
  }

  private handleMessage(userId: string, message: WebSocket.Data) {
    const parsed = JSON.parse(message.toString());

    switch (parsed.type) {
      case 'SUBSCRIBE':
        this.handleSubscribe(userId, parsed);
        break;
      case 'UNSUBSCRIBE':
        this.handleUnsubscribe(userId, parsed);
        break;
      case 'ACKNOWLEDGE':
        this.handleAcknowledge(userId, parsed);
        break;
    }
  }

  private handleSubscribe(userId: string, subscription: SubscriptionRequest) {
    const channelKey = this.buildChannelKey(subscription);

    if (!this.subscriptions.has(channelKey)) {
      this.subscriptions.set(channelKey, new Set());
    }
    this.subscriptions.get(channelKey)!.add(userId);

    // Start monitoring if patient subscription
    if (subscription.channel === 'PATIENT_ALERTS') {
      this.startPatientMonitoring(subscription.patientId);
    }

    this.sendToUser(userId, {
      type: 'SUBSCRIBED',
      channel: subscription.channel,
      subscriptionId: generateUUID()
    });
  }

  // Broadcast alert to subscribed users
  async broadcastAlert(alert: Alert) {
    const channelKey = `PATIENT_ALERTS:${alert.patient.id}`;
    const subscribers = this.subscriptions.get(channelKey);

    if (subscribers) {
      const message: AlertMessage = {
        type: 'ALERT',
        alert,
        timestamp: new Date().toISOString()
      };

      for (const userId of subscribers) {
        await this.sendToUser(userId, message);
      }
    }
  }

  private async sendToUser(userId: string, message: any) {
    const connection = this.connections.get(userId);
    if (connection && connection.readyState === WebSocket.OPEN) {
      connection.send(JSON.stringify(message));
    }
  }
}

// Early Warning System Streaming
interface EarlyWarningChannel {
  description: 'Real-time early warning scores and alerts';

  subscriptionRequest: {
    channel: 'EARLY_WARNING';
    patientId: string;
    thresholds?: {
      newsScore?: number;
      customScores?: { [scoreName: string]: number };
    };
  };

  events: {
    SCORE_UPDATE: {
      patientId: string;
      scoreType: string;
      score: number;
      previousScore?: number;
      trend: 'IMPROVING' | 'STABLE' | 'WORSENING';
      timestamp: string;
      components: ScoreComponent[];
    };
    THRESHOLD_BREACH: {
      patientId: string;
      scoreType: string;
      score: number;
      threshold: number;
      severity: 'WARNING' | 'CRITICAL';
      timestamp: string;
      recommendedActions: string[];
    };
    RAPID_DETERIORATION: {
      patientId: string;
      indicators: string[];
      confidence: number;
      timeToEvent?: string;
      timestamp: string;
    };
  };
}

// Early Warning Score Service
class EarlyWarningService {
  private scoreCalculators: Map<string, ScoreCalculator> = new Map();
  private monitoredPatients: Map<string, PatientMonitor> = new Map();

  async startMonitoring(patientId: string, config: MonitoringConfig) {
    // Create patient monitor
    const monitor = new PatientMonitor(patientId, config);

    // Subscribe to vital sign stream
    monitor.subscribeToVitals(async (vitals) => {
      // Calculate scores
      const scores = await this.calculateScores(patientId, vitals);

      // Check for threshold breaches
      for (const score of scores) {
        if (score.value >= config.thresholds[score.type]) {
          await this.handleThresholdBreach(patientId, score, config);
        }
      }

      // Emit score updates
      await this.emitScoreUpdate(patientId, scores);
    });

    this.monitoredPatients.set(patientId, monitor);
  }

  private async calculateScores(
    patientId: string,
    vitals: VitalSigns
  ): Promise<CalculatedScore[]> {
    const scores: CalculatedScore[] = [];

    // NEWS2 (National Early Warning Score 2)
    const news2 = await this.calculateNEWS2(vitals);
    scores.push({
      type: 'NEWS2',
      value: news2.total,
      components: news2.components,
      interpretation: this.interpretNEWS2(news2.total)
    });

    // MEWS (Modified Early Warning Score)
    const mews = await this.calculateMEWS(vitals);
    scores.push({
      type: 'MEWS',
      value: mews.total,
      components: mews.components
    });

    return scores;
  }

  private calculateNEWS2(vitals: VitalSigns): NEWS2Score {
    const components: ScoreComponent[] = [];

    // Respiratory rate
    const rrScore = this.scoreRespiratoryRate(vitals.respiratoryRate?.value);
    components.push({ name: 'Respiratory Rate', value: vitals.respiratoryRate?.value, score: rrScore });

    // SpO2 (two scales based on O2 supplementation)
    const spo2Score = this.scoreSpO2(
      vitals.oxygenSaturation?.value,
      vitals.oxygenSaturation?.supplementalOxygen
    );
    components.push({ name: 'SpO2', value: vitals.oxygenSaturation?.value, score: spo2Score });

    // Air or oxygen
    const airScore = vitals.oxygenSaturation?.supplementalOxygen ? 2 : 0;
    components.push({ name: 'Air or Oxygen', value: vitals.oxygenSaturation?.supplementalOxygen ? 'Oxygen' : 'Air', score: airScore });

    // Systolic BP
    const sbpScore = this.scoreSystolicBP(vitals.bloodPressure?.systolic);
    components.push({ name: 'Systolic BP', value: vitals.bloodPressure?.systolic, score: sbpScore });

    // Heart rate
    const hrScore = this.scoreHeartRate(vitals.heartRate?.value);
    components.push({ name: 'Heart Rate', value: vitals.heartRate?.value, score: hrScore });

    // Consciousness (AVPU or GCS)
    const consciousnessScore = this.scoreConsciousness(vitals.consciousness);
    components.push({ name: 'Consciousness', value: vitals.consciousness, score: consciousnessScore });

    // Temperature
    const tempScore = this.scoreTemperature(vitals.temperature?.value);
    components.push({ name: 'Temperature', value: vitals.temperature?.value, score: tempScore });

    const total = components.reduce((sum, c) => sum + c.score, 0);

    return { total, components };
  }

  private interpretNEWS2(score: number): NEWS2Interpretation {
    if (score >= 7) {
      return {
        level: 'HIGH',
        clinicalRisk: 'High',
        response: 'Emergency response team review',
        frequency: 'Continuous monitoring',
        color: 'RED'
      };
    } else if (score >= 5) {
      return {
        level: 'MEDIUM',
        clinicalRisk: 'Medium',
        response: 'Urgent review by clinician',
        frequency: 'Minimum 1-hourly',
        color: 'ORANGE'
      };
    } else if (score >= 1) {
      return {
        level: 'LOW_MEDIUM',
        clinicalRisk: 'Low-Medium',
        response: 'Assessment by ward nurse',
        frequency: 'Minimum 4-6 hourly',
        color: 'YELLOW'
      };
    } else {
      return {
        level: 'LOW',
        clinicalRisk: 'Low',
        response: 'Continue routine monitoring',
        frequency: 'Minimum 12-hourly',
        color: 'GREEN'
      };
    }
  }
}
```

### 4.6 GraphQL API for Complex Queries

```typescript
// GraphQL Schema for CDSS
const cdssGraphQLSchema = `
  type Query {
    # Patient queries
    patient(id: ID!): Patient
    patientRecommendations(patientId: ID!, types: [RecommendationType!]): [Recommendation!]!

    # Alert queries
    alerts(
      patientId: ID
      severity: [AlertSeverity!]
      status: [AlertStatus!]
      dateRange: DateRangeInput
    ): AlertConnection!

    # Knowledge queries
    guidelines(
      category: String
      specialty: String
      search: String
    ): GuidelineConnection!

    calculators(category: String): [Calculator!]!

    # Analytics queries
    alertStatistics(
      dateRange: DateRangeInput!
      groupBy: StatisticsGroupBy
    ): AlertStatistics!

    complianceReport(
      guidelineId: ID!
      patientId: ID
      departmentId: ID
    ): ComplianceReport!
  }

  type Mutation {
    # Recommendation actions
    acknowledgeRecommendation(
      recommendationId: ID!
      action: RecommendationAction!
      reason: String
    ): RecommendationActionResult!

    # Alert actions
    acknowledgeAlert(alertId: ID!): Alert!
    overrideAlert(alertId: ID!, reason: OverrideReasonInput!): Alert!

    # Feedback
    submitFeedback(input: FeedbackInput!): FeedbackResult!
  }

  type Subscription {
    # Real-time alerts
    patientAlerts(patientId: ID!): Alert!
    departmentAlerts(departmentId: ID!): Alert!

    # Early warning
    earlyWarningScore(patientId: ID!): EarlyWarningEvent!
  }

  type Patient {
    id: ID!
    mrn: String
    name: HumanName!
    birthDate: Date!
    age: Int!
    gender: Gender!

    # Clinical data
    conditions: [Condition!]!
    medications: [Medication!]!
    allergies: [Allergy!]!
    recentLabs(days: Int = 30): [LabResult!]!
    vitals: VitalSigns

    # CDSS data
    activeAlerts: [Alert!]!
    recommendations: [Recommendation!]!
    riskScores: [RiskScore!]!
    guidelineCompliance: [GuidelineCompliance!]!
  }

  type Recommendation {
    id: ID!
    type: RecommendationType!
    title: String!
    description: String!
    strength: RecommendationStrength!
    urgency: Urgency!
    evidenceLevel: EvidenceLevel!
    rationale: String!
    actions: [RecommendedAction!]!
    alternatives: [Alternative!]
    references: [Reference!]
    createdAt: DateTime!
    expiresAt: DateTime
    status: RecommendationStatus!
  }

  type Alert {
    id: ID!
    category: AlertCategory!
    severity: AlertSeverity!
    title: String!
    message: String!
    detail: String
    patient: Patient!
    trigger: AlertTrigger!
    recommendedAction: String
    overrideAllowed: Boolean!
    overrideReasons: [OverrideReason!]
    status: AlertStatus!
    createdAt: DateTime!
    acknowledgedAt: DateTime
    acknowledgedBy: User
    overriddenAt: DateTime
    overriddenBy: User
    overrideReason: String
  }

  enum RecommendationType {
    DIAGNOSIS_DIFFERENTIAL
    TREATMENT_RECOMMENDATION
    DRUG_SELECTION
    DOSING_RECOMMENDATION
    SCREENING
    RISK_ASSESSMENT
    LAB_INTERPRETATION
    GUIDELINE_COMPLIANCE
  }

  enum AlertSeverity {
    INFO
    WARNING
    HIGH
    CRITICAL
  }

  enum AlertStatus {
    ACTIVE
    ACKNOWLEDGED
    OVERRIDDEN
    EXPIRED
    AUTO_RESOLVED
  }
`;

// GraphQL Resolvers
class CDSSGraphQLResolvers {
  @Query('patient')
  async getPatient(@Args('id') id: string): Promise<Patient> {
    return this.patientService.getPatient(id);
  }

  @Query('patientRecommendations')
  async getPatientRecommendations(
    @Args('patientId') patientId: string,
    @Args('types') types?: RecommendationType[]
  ): Promise<Recommendation[]> {
    return this.recommendationService.getPatientRecommendations(
      patientId,
      types
    );
  }

  @Query('alerts')
  async getAlerts(
    @Args() args: AlertQueryArgs
  ): Promise<AlertConnection> {
    return this.alertService.queryAlerts(args);
  }

  @Mutation('acknowledgeAlert')
  async acknowledgeAlert(
    @Args('alertId') alertId: string,
    @CurrentUser() user: User
  ): Promise<Alert> {
    return this.alertService.acknowledge(alertId, user);
  }

  @Subscription('patientAlerts')
  subscribeToPatientAlerts(
    @Args('patientId') patientId: string
  ): AsyncIterator<Alert> {
    return this.pubSub.asyncIterator(`PATIENT_ALERTS:${patientId}`);
  }

  @ResolveField('Patient.recommendations')
  async resolveRecommendations(
    @Parent() patient: Patient
  ): Promise<Recommendation[]> {
    return this.recommendationService.getActiveRecommendations(patient.id);
  }

  @ResolveField('Patient.riskScores')
  async resolveRiskScores(
    @Parent() patient: Patient
  ): Promise<RiskScore[]> {
    return this.riskService.calculateRiskScores(patient.id);
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT API Interface**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
