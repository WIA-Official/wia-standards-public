# 제6장: 임상의사결정지원 통합

## EHR 통합, 워크플로우 임베딩 및 상호운용성

### 6.1 통합 아키텍처 개요

WIA-CLINICAL-DECISION-SUPPORT 표준은 임상 워크플로우에 임상의사결정지원을 임베딩하기 위한 포괄적인 통합 패턴을 정의합니다. 효과적인 통합은 CDSS 채택과 영향에 중요합니다.

```typescript
// CDSS 통합 아키텍처
interface CDSSIntegrationArchitecture {
  version: '1.0.0';

  integrationPatterns: {
    ehrIntegration: {
      standards: ['SMART on FHIR', 'CDS Hooks', 'HL7 FHIR'];
      approaches: ['네이티브', '앱 기반', 'API'];
    };
    workflowIntegration: {
      patterns: ['동기', '비동기', '백그라운드'];
      triggers: ['사용자 시작', '이벤트 구동', '예약'];
    };
    dataIntegration: {
      sources: ['EHR', '검사 시스템', '영상', '기기'];
      methods: ['실시간', '배치', '스트리밍'];
    };
  };

  integrationLayers: {
    presentation: 'EHR에 임베딩된 UI 컴포넌트';
    service: 'API 기반 CDSS 서비스';
    data: 'FHIR 기반 데이터 교환';
    knowledge: '공유 가능한 지식 아티팩트';
  };
}
```

### 6.2 SMART on FHIR 통합

```typescript
// SMART on FHIR 애플리케이션 프레임워크
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

  // SMART 특정
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

// SMART 런치 핸들러
class SMARTLaunchHandler {
  async handleEHRLaunch(launchParams: LaunchParams): Promise<LaunchResult> {
    // 런치 파라미터 검증
    this.validateLaunchParams(launchParams);

    // 인증 코드를 토큰으로 교환
    const tokens = await this.exchangeCodeForTokens(
      launchParams.code,
      launchParams.state
    );

    // 런치 컨텍스트 추출
    const context = this.extractLaunchContext(tokens);

    // FHIR 클라이언트 초기화
    const fhirClient = this.initializeFHIRClient(
      tokens.access_token,
      context.fhirServer
    );

    // 환자 컨텍스트 로드
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
    // 환자 데이터 병렬 조회
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

// SMART 앱 컴포넌트 (React)
const CDSSSmartApp: React.FC = () => {
  const { tokens, context, fhirClient } = useSMARTLaunch();
  const [recommendations, setRecommendations] = useState<Recommendation[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function loadRecommendations() {
      if (!context.patient) return;

      setLoading(true);
      try {
        // 환자 데이터 가져오기
        const patientData = await loadPatientData(fhirClient, context.patient);

        // CDSS에서 권장사항 요청
        const cdssResponse = await cdssService.getRecommendations({
          patientId: context.patient,
          encounterId: context.encounter,
          patientData,
          queryType: 'COMPREHENSIVE_REVIEW'
        });

        setRecommendations(cdssResponse.recommendations);
      } catch (error) {
        console.error('권장사항 로드 실패:', error);
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

### 6.3 CDS Hooks 통합

```typescript
// CDS Hooks 서비스 등록
interface CDSHooksServiceRegistration {
  services: CDSService[];
  discovery: ServiceDiscovery;
  prefetch: PrefetchTemplates;
}

// CDS 서비스 정의
const drugInteractionService: CDSService = {
  hook: 'order-sign',
  title: '약물 상호작용 검사',
  description: '임상적으로 중요한 약물-약물 상호작용 검사',
  id: 'drug-interaction-checker',
  usageRequirements: '활성 약물 목록 접근 필요',

  prefetch: {
    patient: 'Patient/{{context.patientId}}',
    activeMedications: 'MedicationRequest?patient={{context.patientId}}&status=active',
    allergies: 'AllergyIntolerance?patient={{context.patientId}}&clinical-status=active'
  }
};

// CDS Hooks 서버 구현
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
      throw new NotFoundException(`서비스 ${serviceId}를 찾을 수 없습니다`);
    }

    // 요청 검증
    this.validateRequest(request);

    // 서비스 실행
    const startTime = Date.now();
    const response = await service.handle(request);

    // 실행 로깅
    await this.logExecution(serviceId, request, response, Date.now() - startTime);

    return response;
  }
}

// 약물 상호작용 CDS 서비스
class DrugInteractionService implements CDSServiceHandler {
  private interactionDatabase: DrugInteractionDatabase;
  private alertRules: AlertRuleEngine;

  async handle(request: CDSHookRequest): Promise<CDSHookResponse> {
    const cards: Card[] = [];

    // 컨텍스트에서 약물 추출
    const draftMedications = this.extractDraftMedications(request.context.draftOrders);
    const activeMedications = this.extractActiveMedications(request.prefetch?.activeMedications);
    const allergies = this.extractAllergies(request.prefetch?.allergies);

    // 약물-약물 상호작용 검사
    for (const draftMed of draftMedications) {
      // 활성 약물과 비교
      for (const activeMed of activeMedications) {
        const interaction = await this.interactionDatabase.checkInteraction(
          draftMed.rxnormCode,
          activeMed.rxnormCode
        );

        if (interaction && this.isSignificant(interaction)) {
          cards.push(this.createInteractionCard(draftMed, activeMed, interaction));
        }
      }

      // 약물-알레르기 상호작용 검사
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

    // 심각도별 정렬
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
        label: '약물 상호작용 데이터베이스',
        url: 'https://dailymed.nlm.nih.gov'
      },
      suggestions: this.generateSuggestions(drug1, interaction),
      overrideReasons: [
        {
          code: 'patient-tolerated',
          display: '환자가 이 조합을 이전에 견뎠음'
        },
        {
          code: 'benefit-outweighs-risk',
          display: '임상적 이점이 잠재적 위험을 능가함'
        },
        {
          code: 'will-monitor',
          display: '환자를 면밀히 모니터링할 것임'
        },
        {
          code: 'short-term',
          display: '단기 사용만 해당'
        }
      ],
      links: [
        {
          label: '상호작용 상세 보기',
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

    // 대체 약물
    if (interaction.alternatives && interaction.alternatives.length > 0) {
      for (const alt of interaction.alternatives) {
        suggestions.push({
          label: `대신 ${alt.name} 사용`,
          uuid: generateUUID(),
          isRecommended: alt.preferred,
          actions: [{
            type: 'update',
            description: `${drug.name}을(를) ${alt.name}(으)로 대체`,
            resource: this.createMedicationRequest(alt, drug)
          }]
        });
      }
    }

    // 용량 조절
    if (interaction.doseAdjustment) {
      suggestions.push({
        label: `${drug.name} 용량을 ${interaction.doseAdjustment.recommendedDose}로 감량`,
        uuid: generateUUID(),
        actions: [{
          type: 'update',
          description: '상호작용 권장사항에 따라 용량 조절',
          resource: this.adjustDose(drug, interaction.doseAdjustment)
        }]
      });
    }

    // 모니터링
    if (interaction.monitoringRequired) {
      suggestions.push({
        label: `모니터링 추가: ${interaction.monitoringRequired.parameters.join(', ')}`,
        uuid: generateUUID(),
        actions: interaction.monitoringRequired.orders.map(order => ({
          type: 'create',
          description: `${order.display} 오더`,
          resource: order
        }))
      });
    }

    return suggestions;
  }
}
```

### 6.4 워크플로우 통합 패턴

```typescript
// 임상 워크플로우 통합
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

// 처방 입력 워크플로우 통합
class OrderEntryWorkflowIntegration {
  private cdsHooksClient: CDSHooksClient;
  private alertManager: AlertManager;

  async onOrderSelect(
    order: DraftOrder,
    context: OrderContext
  ): Promise<OrderSelectResult> {
    // order-select 훅 호출
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
    // order-sign 훅 호출
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

    // 경보 분류
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
    // 무시 사유 검증
    if (!alert.overrideReasons?.find(r => r.code === overrideReason.code)) {
      throw new InvalidOverrideReasonError();
    }

    // 무시 기록
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

    // 필요시 후속 조치 트리거
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

// 백그라운드 분석 서비스
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

      // 조치 가능한 소견 확인
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
    // 임상 경보 생성
    const alert = await this.createBackgroundAlert(result, patientData);

    // 적절한 케어 팀원에게 알림
    const careTeam = await this.getCareTeam(patientData.patientId);
    await this.notificationService.notifyCareTeam(careTeam, alert);

    // 환자 경보 목록에 추가
    await this.alertManager.addAlert(alert);
  }
}

// 이벤트 구동 통합
class EventDrivenCDSSIntegration {
  private eventBus: EventBus;
  private cdssServices: Map<string, CDSSService>;

  constructor() {
    this.registerEventHandlers();
  }

  private registerEventHandlers() {
    // 검사 결과 이벤트
    this.eventBus.subscribe('lab.result.final', async (event: LabResultEvent) => {
      await this.handleLabResult(event);
    });

    // 입원 이벤트
    this.eventBus.subscribe('admission.complete', async (event: AdmissionEvent) => {
      await this.handleAdmission(event);
    });

    // 생체징후 이벤트
    this.eventBus.subscribe('vital.recorded', async (event: VitalSignEvent) => {
      await this.handleVitalSign(event);
    });

    // 약물 투여 이벤트
    this.eventBus.subscribe('medication.administered', async (event: MedAdminEvent) => {
      await this.handleMedicationAdministration(event);
    });
  }

  private async handleLabResult(event: LabResultEvent): Promise<void> {
    const { patientId, result, encounterId } = event;

    // 위험 수치 확인
    if (this.isCriticalValue(result)) {
      await this.handleCriticalValue(result, patientId, encounterId);
    }

    // 추세 이상 확인
    const trend = await this.checkLabTrend(result, patientId);
    if (trend.significant) {
      await this.handleLabTrend(trend, patientId, encounterId);
    }

    // 관련 CDSS 분석 트리거
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
    // 위험 경보 생성
    const alert: CriticalAlert = {
      id: generateUUID(),
      type: 'CRITICAL_LAB_VALUE',
      severity: 'CRITICAL',
      patientId,
      encounterId,
      title: `위험 ${result.code.display} 수치`,
      message: `${result.code.display}: ${result.value} ${result.unit} (위험 범위: ${result.referenceRange})`,
      value: result.value,
      referenceRange: result.referenceRange,
      timestamp: new Date(),
      requiresAcknowledgment: true
    };

    // 즉시 알림 전송
    await this.sendCriticalNotifications(alert, patientId);

    // 규정 준수를 위한 로깅
    await this.logCriticalValue(alert);
  }

  private async handleVitalSign(event: VitalSignEvent): Promise<void> {
    const { patientId, vital, encounterId } = event;

    // 조기경보점수 계산
    const recentVitals = await this.getRecentVitals(patientId);
    const ewsScore = this.calculateEWS([...recentVitals, vital]);

    // 악화 확인
    if (ewsScore.total >= this.ewsThreshold) {
      await this.handleDeteriorationAlert(ewsScore, patientId, encounterId);
    }

    // 실시간 모니터링 업데이트
    await this.updateRealTimeMonitoring(patientId, vital, ewsScore);
  }
}
```

### 6.5 EHR별 통합

```typescript
// Epic EHR 통합
class EpicIntegration implements EHRIntegration {
  private epicClient: EpicFHIRClient;
  private smartAuth: SMARTAuth;

  async initialize(config: EpicConfig): Promise<void> {
    // Epic FHIR 엔드포인트 구성
    this.epicClient = new EpicFHIRClient({
      baseUrl: config.fhirEndpoint,
      clientId: config.clientId,
      privateKey: config.privateKey
    });

    // SMART on FHIR 설정
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
    // 런치 토큰 교환
    const tokens = await this.smartAuth.exchangeLaunchToken(launchToken);

    // 토큰에서 컨텍스트 추출
    const context = this.extractContext(tokens.id_token);

    // 액세스 토큰으로 FHIR 클라이언트 초기화
    this.epicClient.setAccessToken(tokens.access_token);

    return {
      patientId: context.patient,
      encounterId: context.encounter,
      userId: context.fhirUser,
      fhirClient: this.epicClient
    };
  }

  // Epic 특정: Best Practice Alerts 통합
  async registerBestPracticeAlert(
    alertDefinition: BestPracticeAlertDefinition
  ): Promise<string> {
    // App Orchard를 통한 Epic BPA 등록
    return await this.epicClient.createCustomResource(
      'BestPracticeAlert',
      alertDefinition
    );
  }

  // Epic 특정: MyChart 프록시를 통한 환자 접근
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

// Cerner (Oracle Health) 통합
class CernerIntegration implements EHRIntegration {
  private cernerClient: CernerFHIRClient;

  async initialize(config: CernerConfig): Promise<void> {
    this.cernerClient = new CernerFHIRClient({
      baseUrl: config.fhirEndpoint,
      clientId: config.clientId,
      clientSecret: config.clientSecret
    });
  }

  // Cerner 특정: MPages 통합
  async launchMPage(mpageConfig: MPageConfig): Promise<MPageContext> {
    return {
      patientId: mpageConfig.patientId,
      encounterId: mpageConfig.encounterId,
      positionId: mpageConfig.positionId,
      personnelId: mpageConfig.personnelId
    };
  }

  // Cerner 특정: PowerChart 통합
  async getPowerChartContext(): Promise<PowerChartContext> {
    // DiscernReportingFramework를 통한 PowerChart 통합
    const context = await this.cernerClient.getContext();
    return {
      patientId: context.patient,
      encounterId: context.encounter,
      organizationId: context.organization,
      facility: context.facility
    };
  }
}

// Meditech 통합
class MeditechIntegration implements EHRIntegration {
  private meditechClient: MeditechClient;

  async initialize(config: MeditechConfig): Promise<void> {
    // Meditech는 다른 통합 접근 방식 사용
    this.meditechClient = new MeditechClient({
      environment: config.environment,
      apiKey: config.apiKey
    });
  }

  // Meditech Expanse FHIR R4 통합
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

### 6.6 통합 테스트 및 모니터링

```typescript
// 통합 테스트 프레임워크
class CDSSIntegrationTester {
  private testSuites: Map<string, TestSuite> = new Map();

  async runIntegrationTests(
    ehrSystem: string,
    environment: 'sandbox' | 'staging' | 'production'
  ): Promise<TestResults> {
    const testSuite = this.testSuites.get(ehrSystem);
    if (!testSuite) {
      throw new Error(`${ehrSystem}에 대한 테스트 스위트 없음`);
    }

    const results: TestResult[] = [];

    // 인증 테스트
    results.push(await this.testAuthentication(testSuite));

    // 데이터 접근 테스트
    results.push(await this.testDataAccess(testSuite));

    // CDS Hooks 테스트
    results.push(await this.testCDSHooks(testSuite));

    // 성능 테스트
    results.push(await this.testPerformance(testSuite));

    // 오류 처리 테스트
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
      // 알려진 약물 상호작용으로 테스트
      const response = await testSuite.client.callHook('order-sign', {
        context: {
          userId: testSuite.testUser,
          patientId: testSuite.testPatient,
          draftOrders: testSuite.testOrdersWithInteraction
        }
      });

      // 상호작용 탐지 확인
      const interactionCard = response.cards.find(
        c => c.source.topic?.code === 'drug-interaction'
      );

      return {
        name: 'order-sign 훅 - 약물 상호작용 탐지',
        passed: interactionCard !== undefined,
        duration: Date.now() - startTime,
        details: interactionCard ? '상호작용이 올바르게 탐지됨' : '상호작용이 탐지되지 않음'
      };

    } catch (error) {
      return {
        name: 'order-sign 훅 - 약물 상호작용 탐지',
        passed: false,
        duration: Date.now() - startTime,
        error: error.message
      };
    }
  }
}

// 통합 모니터링
class IntegrationMonitor {
  private metrics: MetricsCollector;
  private alerting: AlertingService;

  async monitorIntegration(integrationId: string): Promise<void> {
    // 메트릭 수집
    const metrics = await this.collectIntegrationMetrics(integrationId);

    // 임계값 확인
    await this.checkThresholds(integrationId, metrics);

    // 메트릭 기록
    await this.metrics.record(integrationId, metrics);
  }

  private async collectIntegrationMetrics(
    integrationId: string
  ): Promise<IntegrationMetrics> {
    return {
      // 가용성
      availability: await this.measureAvailability(integrationId),

      // 지연 시간
      latency: {
        p50: await this.measureLatencyP50(integrationId),
        p95: await this.measureLatencyP95(integrationId),
        p99: await this.measureLatencyP99(integrationId)
      },

      // 처리량
      requestsPerMinute: await this.measureThroughput(integrationId),

      // 오류율
      errorRate: await this.measureErrorRate(integrationId),

      // Hook 특정 메트릭
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
    // 가용성 임계값
    if (metrics.availability < 0.999) {
      await this.alerting.sendAlert({
        severity: 'HIGH',
        integration: integrationId,
        metric: 'availability',
        value: metrics.availability,
        threshold: 0.999
      });
    }

    // 지연 시간 임계값
    if (metrics.latency.p95 > 500) {
      await this.alerting.sendAlert({
        severity: 'MEDIUM',
        integration: integrationId,
        metric: 'latency_p95',
        value: metrics.latency.p95,
        threshold: 500
      });
    }

    // 오류율 임계값
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

**WIA-CLINICAL-DECISION-SUPPORT 통합**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
