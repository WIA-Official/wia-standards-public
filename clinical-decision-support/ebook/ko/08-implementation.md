# 제8장: 임상의사결정지원 구현

## 배포, 운영 및 최적화

### 8.1 구현 아키텍처

WIA-CLINICAL-DECISION-SUPPORT 표준은 인프라 요구사항, 배포 전략 및 운영 모범 사례를 포함하여 의료 환경에 임상의사결정지원 시스템을 배포하기 위한 포괄적인 지침을 제공합니다.

```typescript
// CDSS 구현 아키텍처
interface CDSSImplementationArchitecture {
  version: '1.0.0';

  deploymentModels: {
    cloudNative: {
      description: '완전 클라우드 호스팅 CDSS';
      providers: ['AWS', 'Azure', 'GCP'];
      benefits: ['확장성', '관리형 서비스', '글로벌 가용성'];
      considerations: ['데이터 상주', 'HIPAA BAA', '지연 시간'];
    };
    onPremise: {
      description: '의료 기관 내 자체 호스팅';
      benefits: ['데이터 통제', '낮은 지연 시간', '통합 단순성'];
      considerations: ['인프라 비용', '유지보수 부담', '확장'];
    };
    hybrid: {
      description: '클라우드와 온프레미스 컴포넌트 혼합';
      benefits: ['유연성', '데이터 주권', '비용 최적화'];
      considerations: ['복잡성', '네트워크 요구사항', '데이터 동기화'];
    };
  };

  implementationPhases: [
    '발견 및 평가',
    '아키텍처 및 설계',
    '개발 및 통합',
    '검증 및 테스트',
    '파일럿 배포',
    '프로덕션 출시',
    '최적화 및 모니터링'
  ];
}

// 구현 관리자
class CDSSImplementationManager {
  private projectManager: ProjectManager;
  private technicalArchitect: TechnicalArchitect;
  private clinicalLeader: ClinicalLeader;
  private integrationSpecialist: IntegrationSpecialist;

  async executeImplementation(
    project: CDSSImplementationProject
  ): Promise<ImplementationResult> {
    const results: PhaseResult[] = [];

    // 1단계: 발견
    const discovery = await this.executeDiscoveryPhase(project);
    results.push({ phase: 'Discovery', result: discovery });

    // 2단계: 설계
    const design = await this.executeDesignPhase(project, discovery);
    results.push({ phase: 'Design', result: design });

    // 3단계: 빌드
    const build = await this.executeBuildPhase(project, design);
    results.push({ phase: 'Build', result: build });

    // 4단계: 테스트
    const test = await this.executeTestPhase(project, build);
    results.push({ phase: 'Test', result: test });

    if (!test.passed) {
      return {
        success: false,
        failedPhase: 'Test',
        issues: test.failures
      };
    }

    // 5단계: 파일럿
    const pilot = await this.executePilotPhase(project, build);
    results.push({ phase: 'Pilot', result: pilot });

    // 6단계: 출시
    const rollout = await this.executeRolloutPhase(project, pilot);
    results.push({ phase: 'Rollout', result: rollout });

    return {
      success: true,
      phases: results,
      metrics: await this.collectImplementationMetrics(project)
    };
  }

  private async executeDiscoveryPhase(
    project: CDSSImplementationProject
  ): Promise<DiscoveryResult> {
    return {
      stakeholderAnalysis: await this.analyzeStakeholders(project),
      currentStateAssessment: await this.assessCurrentState(project),
      requirementsGathering: await this.gatherRequirements(project),
      technicalReadiness: await this.assessTechnicalReadiness(project),
      clinicalReadiness: await this.assessClinicalReadiness(project),
      riskAssessment: await this.assessProjectRisks(project)
    };
  }

  private async executeDesignPhase(
    project: CDSSImplementationProject,
    discovery: DiscoveryResult
  ): Promise<DesignResult> {
    // 기술 아키텍처
    const architecture = await this.technicalArchitect.designArchitecture(
      project,
      discovery
    );

    // 통합 설계
    const integration = await this.integrationSpecialist.designIntegration(
      project,
      discovery.currentStateAssessment
    );

    // 임상 워크플로우 설계
    const workflow = await this.clinicalLeader.designWorkflows(
      project,
      discovery.requirementsGathering
    );

    return {
      technicalArchitecture: architecture,
      integrationDesign: integration,
      workflowDesign: workflow,
      dataFlowDesign: await this.designDataFlows(architecture, integration),
      securityDesign: await this.designSecurity(architecture),
      testStrategy: await this.designTestStrategy(project)
    };
  }
}
```

### 8.2 인프라 및 배포

```typescript
// 인프라 구성
interface CDSSInfrastructure {
  compute: ComputeInfrastructure;
  database: DatabaseInfrastructure;
  networking: NetworkingInfrastructure;
  security: SecurityInfrastructure;
  monitoring: MonitoringInfrastructure;
}

// Kubernetes 배포 구성
const cdssKubernetesDeployment = {
  apiVersion: 'apps/v1',
  kind: 'Deployment',
  metadata: {
    name: 'cdss-api',
    namespace: 'clinical-decision-support',
    labels: {
      app: 'cdss',
      component: 'api',
      version: 'v1.0.0'
    }
  },
  spec: {
    replicas: 3,
    selector: {
      matchLabels: { app: 'cdss', component: 'api' }
    },
    template: {
      metadata: {
        labels: {
          app: 'cdss',
          component: 'api',
          version: 'v1.0.0'
        },
        annotations: {
          'prometheus.io/scrape': 'true',
          'prometheus.io/port': '9090'
        }
      },
      spec: {
        serviceAccountName: 'cdss-api',
        securityContext: {
          runAsNonRoot: true,
          runAsUser: 1000,
          fsGroup: 1000
        },
        containers: [{
          name: 'cdss-api',
          image: 'cdss-registry.example.com/cdss-api:v1.0.0',
          ports: [
            { name: 'http', containerPort: 8080 },
            { name: 'metrics', containerPort: 9090 }
          ],
          resources: {
            requests: { cpu: '500m', memory: '1Gi' },
            limits: { cpu: '2000m', memory: '4Gi' }
          },
          livenessProbe: {
            httpGet: { path: '/health/live', port: 'http' },
            initialDelaySeconds: 30,
            periodSeconds: 10
          },
          readinessProbe: {
            httpGet: { path: '/health/ready', port: 'http' },
            initialDelaySeconds: 5,
            periodSeconds: 5
          },
          env: [
            { name: 'DATABASE_URL', valueFrom: { secretKeyRef: { name: 'cdss-secrets', key: 'database-url' } } },
            { name: 'FHIR_SERVER_URL', valueFrom: { configMapKeyRef: { name: 'cdss-config', key: 'fhir-server-url' } } },
            { name: 'LOG_LEVEL', value: 'INFO' }
          ],
          volumeMounts: [
            { name: 'config', mountPath: '/app/config', readOnly: true },
            { name: 'tls-certs', mountPath: '/app/certs', readOnly: true }
          ]
        }],
        volumes: [
          { name: 'config', configMap: { name: 'cdss-config' } },
          { name: 'tls-certs', secret: { secretName: 'cdss-tls' } }
        ]
      }
    }
  }
};

// 배포 서비스
class CDSSDeploymentService {
  private kubernetesClient: KubernetesClient;
  private configManager: ConfigurationManager;
  private healthChecker: HealthChecker;

  async deployVersion(
    version: string,
    environment: Environment
  ): Promise<DeploymentResult> {
    // 배포 전제 조건 검증
    const validation = await this.validatePrerequisites(version, environment);
    if (!validation.passed) {
      throw new DeploymentValidationError(validation.issues);
    }

    // 배포 전략 생성
    const strategy = this.determineDeploymentStrategy(environment);

    // 배포 실행
    let result: DeploymentResult;
    switch (strategy.type) {
      case 'ROLLING':
        result = await this.executeRollingDeployment(version, environment);
        break;
      case 'BLUE_GREEN':
        result = await this.executeBlueGreenDeployment(version, environment);
        break;
      case 'CANARY':
        result = await this.executeCanaryDeployment(version, environment);
        break;
      default:
        throw new Error(`알 수 없는 배포 전략: ${strategy.type}`);
    }

    // 배포 상태 확인
    const health = await this.verifyDeploymentHealth(version, environment);
    if (!health.healthy) {
      await this.rollback(version, environment);
      throw new DeploymentHealthError(health.issues);
    }

    return result;
  }

  private async executeCanaryDeployment(
    version: string,
    environment: Environment
  ): Promise<DeploymentResult> {
    // 카나리 배포 (트래픽 10%)
    await this.deployCanary(version, environment, 10);

    // 카나리 메트릭 모니터링
    const canaryMetrics = await this.monitorCanary(version, environment, 30); // 30분

    if (!canaryMetrics.acceptable) {
      await this.rollbackCanary(version, environment);
      throw new CanaryFailureError(canaryMetrics.issues);
    }

    // 점진적 출시: 25% -> 50% -> 100%
    for (const percentage of [25, 50, 100]) {
      await this.updateCanaryPercentage(version, environment, percentage);
      const metrics = await this.monitorCanary(version, environment, 15);

      if (!metrics.acceptable) {
        await this.rollbackCanary(version, environment);
        throw new CanaryFailureError(metrics.issues);
      }
    }

    // 배포 완료
    return this.finalizeDeployment(version, environment);
  }

  private async monitorCanary(
    version: string,
    environment: Environment,
    durationMinutes: number
  ): Promise<CanaryMetrics> {
    const startTime = Date.now();
    const endTime = startTime + durationMinutes * 60 * 1000;

    const metrics: MetricSample[] = [];

    while (Date.now() < endTime) {
      const sample = await this.collectMetrics(version, environment);
      metrics.push(sample);

      // 즉각적인 실패 확인
      if (sample.errorRate > 0.05) {
        return {
          acceptable: false,
          issues: ['오류율이 5% 임계값 초과'],
          metrics
        };
      }

      await sleep(30000); // 30초마다 샘플링
    }

    // 수집된 메트릭 분석
    return this.analyzeCanaryMetrics(metrics);
  }
}

// 데이터베이스 마이그레이션 서비스
class CDSSDatabaseMigrationService {
  private migrationRunner: MigrationRunner;
  private validator: SchemaValidator;
  private backupService: BackupService;

  async executeMigration(
    migration: DatabaseMigration
  ): Promise<MigrationResult> {
    // 백업 생성
    const backup = await this.backupService.createBackup();

    // 마이그레이션 검증
    const validation = await this.validator.validateMigration(migration);
    if (!validation.valid) {
      throw new MigrationValidationError(validation.errors);
    }

    try {
      // 마이그레이션 실행
      const result = await this.migrationRunner.run(migration);

      // 스키마 검증
      const schemaValid = await this.validator.validateSchema();
      if (!schemaValid) {
        throw new SchemaValidationError('마이그레이션 후 스키마 검증 실패');
      }

      // 데이터 무결성 검증
      const dataValid = await this.verifyDataIntegrity();
      if (!dataValid) {
        throw new DataIntegrityError('데이터 무결성 검사 실패');
      }

      return {
        success: true,
        migration: migration.version,
        duration: result.duration,
        changes: result.changes
      };

    } catch (error) {
      // 백업에서 복원
      await this.backupService.restore(backup.id);
      throw error;
    }
  }
}
```

### 8.3 임상 검증 및 테스트

```typescript
// 임상 검증 프레임워크
interface ClinicalValidationFramework {
  validationTypes: {
    retrospective: RetrospectiveValidation;
    prospective: ProspectiveValidation;
    silentMode: SilentModeValidation;
    abTesting: ABTestingValidation;
  };

  metrics: {
    accuracy: AccuracyMetrics;
    usability: UsabilityMetrics;
    impact: ClinicalImpactMetrics;
    adoption: AdoptionMetrics;
  };
}

// 임상 검증 서비스
class ClinicalValidationService {
  private dataExtractor: ClinicalDataExtractor;
  private metricsCalculator: MetricsCalculator;
  private statistician: StatisticalAnalyzer;

  async conductRetrospectiveValidation(
    algorithm: CDSSAlgorithm,
    config: ValidationConfig
  ): Promise<RetrospectiveValidationResult> {
    // 과거 데이터 추출
    const dataset = await this.dataExtractor.extractValidationDataset(
      config.dateRange,
      config.patientCriteria
    );

    // 과거 케이스에 알고리즘 실행
    const predictions = await this.runAlgorithmOnDataset(algorithm, dataset);

    // 알려진 결과와 비교
    const comparison = await this.compareToOutcomes(predictions, dataset.outcomes);

    // 성능 메트릭 계산
    const metrics = this.metricsCalculator.calculatePerformanceMetrics(comparison);

    // 하위 그룹 분석 수행
    const subgroupAnalysis = await this.analyzeSubgroups(comparison, config.subgroups);

    // 통계 분석
    const statistics = await this.statistician.analyze(comparison);

    return {
      algorithm: algorithm.id,
      version: algorithm.version,
      validationDate: new Date(),
      dataset: {
        size: dataset.cases.length,
        dateRange: config.dateRange,
        criteria: config.patientCriteria
      },
      performance: metrics,
      subgroupAnalysis,
      statistics,
      conclusion: this.generateConclusion(metrics, statistics)
    };
  }

  async conductSilentModeValidation(
    algorithm: CDSSAlgorithm,
    config: SilentModeConfig
  ): Promise<SilentModeValidationResult> {
    // 임상의에게 표시하지 않고 백그라운드에서 알고리즘 실행
    const silentRun = await this.startSilentMode(algorithm, config);

    // 예측 수집
    const predictions: SilentPrediction[] = [];

    while (await this.shouldContinueSilentMode(silentRun, config)) {
      const batch = await this.collectSilentPredictions(silentRun);
      predictions.push(...batch);
      await sleep(config.collectionInterval);
    }

    // 실제 임상 결정과 비교
    const comparison = await this.compareToActualDecisions(predictions);

    // 일치 메트릭 계산
    const agreement = this.calculateAgreement(comparison);

    // 불일치 분석
    const discordanceAnalysis = await this.analyzeDiscordances(
      comparison.discordant
    );

    return {
      algorithm: algorithm.id,
      duration: config.duration,
      predictions: predictions.length,
      agreement,
      discordanceAnalysis,
      recommendations: this.generateSilentModeRecommendations(
        agreement,
        discordanceAnalysis
      )
    };
  }

  async conductABTest(
    controlAlgorithm: CDSSAlgorithm,
    testAlgorithm: CDSSAlgorithm,
    config: ABTestConfig
  ): Promise<ABTestResult> {
    // 환자/임상의 무작위 배정
    const randomization = await this.performRandomization(config);

    // 테스트 실행
    const testRun = await this.startABTest(
      controlAlgorithm,
      testAlgorithm,
      randomization,
      config
    );

    // 결과 수집
    const outcomes = await this.collectABTestOutcomes(testRun, config.duration);

    // 결과 분석
    const analysis = await this.statistician.analyzeABTest(outcomes);

    return {
      controlAlgorithm: controlAlgorithm.id,
      testAlgorithm: testAlgorithm.id,
      duration: config.duration,
      sampleSize: {
        control: outcomes.control.length,
        test: outcomes.test.length
      },
      primaryOutcome: analysis.primaryOutcome,
      secondaryOutcomes: analysis.secondaryOutcomes,
      safetyAnalysis: analysis.safety,
      conclusion: analysis.conclusion,
      recommendation: this.generateABTestRecommendation(analysis)
    };
  }
}

// 사용성 테스트
class UsabilityTestingService {
  private taskRecorder: TaskRecorder;
  private surveyService: SurveyService;
  private observationService: ObservationService;

  async conductUsabilityTest(
    cdssFeature: CDSSFeature,
    participants: Participant[],
    tasks: UsabilityTask[]
  ): Promise<UsabilityTestResult> {
    const sessionResults: SessionResult[] = [];

    for (const participant of participants) {
      const session = await this.conductSession(participant, cdssFeature, tasks);
      sessionResults.push(session);
    }

    // 작업 완료 분석
    const taskAnalysis = this.analyzeTaskCompletion(sessionResults, tasks);

    // 작업 시간 분석
    const timeAnalysis = this.analyzeTimeOnTask(sessionResults, tasks);

    // 오류 분석
    const errorAnalysis = this.analyzeErrors(sessionResults);

    // 만족도 분석 (SUS 점수)
    const satisfactionAnalysis = await this.analyzeSatisfaction(sessionResults);

    return {
      feature: cdssFeature.id,
      participantCount: participants.length,
      taskAnalysis,
      timeAnalysis,
      errorAnalysis,
      satisfaction: {
        susScore: satisfactionAnalysis.susScore,
        interpretation: this.interpretSUSScore(satisfactionAnalysis.susScore),
        detailedFeedback: satisfactionAnalysis.feedback
      },
      recommendations: this.generateUsabilityRecommendations(
        taskAnalysis,
        timeAnalysis,
        errorAnalysis,
        satisfactionAnalysis
      )
    };
  }

  private async conductSession(
    participant: Participant,
    feature: CDSSFeature,
    tasks: UsabilityTask[]
  ): Promise<SessionResult> {
    const taskResults: TaskResult[] = [];

    for (const task of tasks) {
      // 작업 실행 기록
      const recording = await this.taskRecorder.startRecording(participant);

      // 작업 지시 제시
      await this.presentTask(participant, task);

      // 완료 또는 타임아웃 대기
      const completion = await this.waitForTaskCompletion(task, recording);

      taskResults.push({
        task: task.id,
        completed: completion.completed,
        timeToComplete: completion.time,
        errors: completion.errors,
        observations: completion.observations
      });
    }

    // 세션 후 설문 실시
    const survey = await this.surveyService.conductSUS(participant);

    return {
      participant: participant.id,
      participantRole: participant.role,
      taskResults,
      survey,
      feedback: await this.collectOpenFeedback(participant)
    };
  }
}
```

### 8.4 운영 및 모니터링

```typescript
// CDSS 운영 서비스
class CDSSOperationsService {
  private monitoringService: MonitoringService;
  private alertManager: OperationsAlertManager;
  private performanceOptimizer: PerformanceOptimizer;
  private incidentManager: IncidentManager;

  async monitorCDSS(): Promise<void> {
    // 지속적 모니터링 루프
    while (true) {
      const metrics = await this.collectOperationalMetrics();

      // 서비스 상태 확인
      const healthStatus = await this.checkServiceHealth();
      if (!healthStatus.healthy) {
        await this.handleUnhealthyService(healthStatus);
      }

      // 성능 확인
      const performanceStatus = await this.checkPerformance(metrics);
      if (performanceStatus.degraded) {
        await this.handlePerformanceDegradation(performanceStatus);
      }

      // 경보 전달 확인
      const alertDeliveryStatus = await this.checkAlertDelivery();
      if (alertDeliveryStatus.issues.length > 0) {
        await this.handleAlertDeliveryIssues(alertDeliveryStatus);
      }

      // 메트릭 저장
      await this.storeMetrics(metrics);

      await sleep(30000); // 30초 간격
    }
  }

  private async collectOperationalMetrics(): Promise<OperationalMetrics> {
    return {
      timestamp: new Date(),

      // 서비스 메트릭
      requestsPerMinute: await this.getRequestRate(),
      errorRate: await this.getErrorRate(),
      latency: {
        p50: await this.getLatencyP50(),
        p95: await this.getLatencyP95(),
        p99: await this.getLatencyP99()
      },

      // CDSS 특정 메트릭
      recommendationsGenerated: await this.getRecommendationCount(),
      alertsFired: await this.getAlertCount(),
      alertOverrideRate: await this.getOverrideRate(),
      alertAcknowledgeTime: await this.getAcknowledgeTime(),

      // 리소스 메트릭
      cpuUtilization: await this.getCPUUtilization(),
      memoryUtilization: await this.getMemoryUtilization(),
      databaseConnections: await this.getDatabaseConnections(),

      // 통합 메트릭
      ehrIntegrationHealth: await this.getEHRIntegrationHealth(),
      fhirServerHealth: await this.getFHIRServerHealth(),
      cdsHooksHealth: await this.getCDSHooksHealth()
    };
  }

  async handleIncident(incident: Incident): Promise<IncidentResponse> {
    // 인시던트 기록 생성
    const incidentRecord = await this.incidentManager.createIncident(incident);

    // 심각도 분류
    const severity = this.classifyIncidentSeverity(incident);

    // 대응 계획 실행
    switch (severity) {
      case 'SEV1':
        return this.executeSev1Response(incident, incidentRecord);
      case 'SEV2':
        return this.executeSev2Response(incident, incidentRecord);
      case 'SEV3':
        return this.executeSev3Response(incident, incidentRecord);
      default:
        return this.executeStandardResponse(incident, incidentRecord);
    }
  }

  private async executeSev1Response(
    incident: Incident,
    record: IncidentRecord
  ): Promise<IncidentResponse> {
    // 온콜 팀 호출
    await this.alertManager.pageOnCall(incident, 'SEV1');

    // 워룸 생성
    const warRoom = await this.createWarRoom(incident);

    // 이해관계자 알림
    await this.notifyStakeholders(incident, ['leadership', 'clinical', 'it']);

    // CDSS 가용성 영향 시 폴백 모드 활성화
    if (this.affectsCDSSAvailability(incident)) {
      await this.enableFallbackMode();
    }

    // 상태 페이지 업데이트
    await this.updateStatusPage(incident, 'INVESTIGATING');

    return {
      incidentId: record.id,
      severity: 'SEV1',
      warRoom,
      fallbackEnabled: this.affectsCDSSAvailability(incident),
      stakeholdersNotified: true
    };
  }
}

// 성능 최적화
class CDSSPerformanceOptimizer {
  private queryOptimizer: QueryOptimizer;
  private cacheManager: CacheManager;
  private loadBalancer: LoadBalancer;

  async optimizePerformance(
    metrics: PerformanceMetrics
  ): Promise<OptimizationResult> {
    const optimizations: Optimization[] = [];

    // 병목 현상 식별
    const bottlenecks = this.identifyBottlenecks(metrics);

    for (const bottleneck of bottlenecks) {
      const optimization = await this.addressBottleneck(bottleneck);
      optimizations.push(optimization);
    }

    return {
      bottlenecksIdentified: bottlenecks.length,
      optimizationsApplied: optimizations,
      expectedImprovement: this.calculateExpectedImprovement(optimizations)
    };
  }

  async optimizeRuleEngine(): Promise<RuleEngineOptimization> {
    // 규칙 실행 패턴 분석
    const executionStats = await this.analyzeRuleExecutionPatterns();

    // 느린 규칙 식별
    const slowRules = executionStats.filter(r => r.avgExecutionTime > 100);

    // 규칙 순서 최적화
    const ruleOrdering = await this.optimizeRuleOrdering(executionStats);

    // 규칙 캐싱 구현
    const cachingStrategy = await this.designRuleCaching(executionStats);

    return {
      slowRulesIdentified: slowRules.length,
      ruleOrderingOptimized: true,
      cachingImplemented: true,
      expectedImprovement: '규칙 평가 시간 40% 감소'
    };
  }

  async optimizeMLInference(): Promise<MLOptimization> {
    // 추론 패턴 분석
    const inferenceStats = await this.analyzeMLInferencePatterns();

    // 모델 로딩 최적화
    await this.optimizeModelLoading();

    // 배치 처리 구현
    await this.implementBatchInference();

    // GPU 가속 활성화 (가능한 경우)
    if (await this.gpuAvailable()) {
      await this.enableGPUAcceleration();
    }

    return {
      modelLoadingOptimized: true,
      batchingEnabled: true,
      gpuAcceleration: await this.gpuAvailable(),
      expectedLatencyReduction: '60%'
    };
  }
}

// 대시보드 및 보고
class CDSSDashboardService {
  async generateOperationalDashboard(): Promise<OperationalDashboard> {
    return {
      timestamp: new Date(),

      serviceHealth: {
        overall: await this.getOverallHealth(),
        components: await this.getComponentHealth()
      },

      performance: {
        latency: await this.getLatencyMetrics(),
        throughput: await this.getThroughputMetrics(),
        errorRate: await this.getErrorRateMetrics()
      },

      cdssMetrics: {
        recommendations: await this.getRecommendationMetrics(),
        alerts: await this.getAlertMetrics(),
        overrides: await this.getOverrideMetrics(),
        outcomes: await this.getOutcomeMetrics()
      },

      usage: {
        activeUsers: await this.getActiveUserCount(),
        requestsByService: await this.getRequestsByService(),
        topCDSSFunctions: await this.getTopFunctions()
      },

      trends: {
        hourly: await this.getHourlyTrends(),
        daily: await this.getDailyTrends(),
        weekly: await this.getWeeklyTrends()
      }
    };
  }
}
```

### 8.5 교육 및 변화 관리

```typescript
// CDSS 교육 프로그램
class CDSSTrainingProgram {
  private trainingContentManager: TrainingContentManager;
  private assessmentService: AssessmentService;
  private completionTracker: CompletionTracker;

  async designTrainingProgram(
    cdssSystem: CDSSSystem,
    audience: TrainingAudience[]
  ): Promise<TrainingProgram> {
    const modules: TrainingModule[] = [];

    // 모든 사용자를 위한 핵심 모듈
    modules.push(
      await this.createModule('CDSS 개요', 'core', 30),
      await this.createModule('AI 권장사항 이해하기', 'core', 45),
      await this.createModule('경보 해석 및 대응', 'core', 60),
      await this.createModule('무시 문서화', 'core', 30)
    );

    // 역할별 모듈
    for (const role of audience) {
      const roleModules = await this.createRoleSpecificModules(role, cdssSystem);
      modules.push(...roleModules);
    }

    // 실습 교육
    modules.push(
      await this.createModule('시뮬레이션 환자 시나리오', 'practical', 120),
      await this.createModule('임상 워크플로우와의 통합', 'practical', 90)
    );

    return {
      id: generateUUID(),
      name: `${cdssSystem.name} 교육 프로그램`,
      version: '1.0',
      modules,
      totalDuration: modules.reduce((sum, m) => sum + m.durationMinutes, 0),
      assessments: await this.createAssessments(modules),
      certification: await this.defineCertification(modules)
    };
  }

  async trackCompletion(
    userId: string,
    programId: string
  ): Promise<CompletionStatus> {
    const completedModules = await this.completionTracker.getCompletedModules(
      userId,
      programId
    );
    const program = await this.getProgram(programId);

    const assessmentScores = await this.assessmentService.getScores(
      userId,
      programId
    );

    return {
      userId,
      programId,
      completedModules: completedModules.length,
      totalModules: program.modules.length,
      completionPercentage: (completedModules.length / program.modules.length) * 100,
      assessmentScores,
      certified: this.meetsCertificationCriteria(completedModules, assessmentScores, program)
    };
  }
}

// 변화 관리
class CDSSChangeManagement {
  private communicationService: CommunicationService;
  private feedbackCollector: FeedbackCollector;
  private adoptionTracker: AdoptionTracker;

  async executeChangeManagement(
    cdssImplementation: CDSSImplementation
  ): Promise<ChangeManagementResult> {
    // 구현 전
    await this.conductPreImplementation(cdssImplementation);

    // 구현 중
    await this.supportImplementation(cdssImplementation);

    // 구현 후
    await this.conductPostImplementation(cdssImplementation);

    return {
      communicationsSent: await this.getCommunicationMetrics(),
      trainingsCompleted: await this.getTrainingMetrics(),
      adoptionRate: await this.adoptionTracker.getAdoptionRate(),
      satisfactionScore: await this.getSatisfactionScore(),
      feedbackSummary: await this.summarizeFeedback()
    };
  }

  private async conductPreImplementation(
    implementation: CDSSImplementation
  ): Promise<void> {
    // 이해관계자 분석
    const stakeholders = await this.identifyStakeholders(implementation);

    // 커뮤니케이션 계획
    const commPlan = await this.createCommunicationPlan(stakeholders);
    await this.executeCommunicationPlan(commPlan);

    // 준비도 평가
    const readiness = await this.assessReadiness(stakeholders);

    // 우려사항 해결
    const concerns = await this.collectConcerns(stakeholders);
    await this.addressConcerns(concerns);

    // 교육 일정
    await this.scheduleTraining(stakeholders);
  }
}
```

---

**WIA-CLINICAL-DECISION-SUPPORT 구현**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
