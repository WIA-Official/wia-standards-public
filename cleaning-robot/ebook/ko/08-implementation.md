# 제8장: 청소 로봇 구현

## 배포, 플릿 운영 및 유지보수

### 8.1 구현 아키텍처

WIA-CLEANING-ROBOT 표준은 주거 및 상업 환경에서 자율 청소 로봇을 배포하기 위한 포괄적인 지침을 제공하며, 플릿 관리, 유지보수 프로토콜 및 운영 모범 사례를 포함합니다.

```typescript
// 구현 아키텍처 정의
interface CleaningRobotImplementation {
  version: '1.0.0';

  deploymentTypes: {
    residential: {
      scale: '1-3대 로봇';
      complexity: 'LOW';
      management: '사용자 셀프 서비스';
      support: '소비자 지원';
    };
    smallCommercial: {
      scale: '3-10대 로봇';
      complexity: 'MEDIUM';
      management: '기본 플릿 관리';
      support: '비즈니스 지원';
    };
    enterprise: {
      scale: '10-100대 이상 로봇';
      complexity: 'HIGH';
      management: '완전 플릿 오케스트레이션';
      support: '엔터프라이즈 SLA';
    };
  };

  implementationPhases: [
    '평가 및 계획',
    '인프라 준비',
    '로봇 배포',
    '매핑 및 구성',
    '통합 설정',
    '테스트 및 검증',
    '교육 및 인계',
    '운영 및 최적화'
  ];
}

// 배포 관리자
class RobotDeploymentManager {
  private assessmentService: AssessmentService;
  private configurationService: ConfigurationService;
  private mappingService: MappingService;
  private validationService: ValidationService;

  async executeDeployment(
    deployment: DeploymentPlan
  ): Promise<DeploymentResult> {
    const results: PhaseResult[] = [];

    // 1단계: 배포 전 평가
    const assessment = await this.assessmentService.assessSite(deployment.site);
    results.push({ phase: 'Assessment', result: assessment });

    if (!assessment.passed) {
      return {
        success: false,
        failedPhase: 'Assessment',
        issues: assessment.issues
      };
    }

    // 2단계: 인프라 준비
    const infraPrep = await this.prepareInfrastructure(deployment, assessment);
    results.push({ phase: 'Infrastructure', result: infraPrep });

    // 3단계: 로봇 배포
    const robotDeploy = await this.deployRobots(deployment);
    results.push({ phase: 'RobotDeployment', result: robotDeploy });

    // 4단계: 매핑
    const mapping = await this.executeMapping(deployment, robotDeploy.robots);
    results.push({ phase: 'Mapping', result: mapping });

    // 5단계: 구성
    const config = await this.configureRobots(deployment, mapping.maps);
    results.push({ phase: 'Configuration', result: config });

    // 6단계: 통합
    const integration = await this.setupIntegrations(deployment);
    results.push({ phase: 'Integration', result: integration });

    // 7단계: 검증
    const validation = await this.validateDeployment(deployment);
    results.push({ phase: 'Validation', result: validation });

    return {
      success: validation.passed,
      phases: results,
      deploymentId: deployment.id,
      completedAt: new Date()
    };
  }

  private async prepareInfrastructure(
    deployment: DeploymentPlan,
    assessment: SiteAssessment
  ): Promise<InfrastructureResult> {
    const tasks: InfrastructureTask[] = [];

    // WiFi 커버리지
    if (assessment.wifi.needsImprovement) {
      tasks.push({
        type: 'WIFI_IMPROVEMENT',
        locations: assessment.wifi.weakSpots,
        recommendation: '액세스 포인트 또는 익스텐더 설치'
      });
    }

    // 도킹 스테이션 배치
    const dockLocations = this.optimizeDockLocations(
      deployment.site,
      deployment.robots.length
    );
    tasks.push({
      type: 'DOCK_INSTALLATION',
      locations: dockLocations,
      requirements: this.getDockRequirements(deployment.robots)
    });

    // 바닥 준비
    if (assessment.floor.issues.length > 0) {
      tasks.push({
        type: 'FLOOR_PREPARATION',
        issues: assessment.floor.issues,
        recommendation: assessment.floor.recommendations
      });
    }

    // 인프라 작업 실행
    for (const task of tasks) {
      await this.executeInfrastructureTask(task);
    }

    return {
      success: true,
      tasksCompleted: tasks.length,
      dockLocations
    };
  }

  private optimizeDockLocations(
    site: Site,
    robotCount: number
  ): Position2D[] {
    const floorPlan = site.floorPlan;
    const zones = site.cleaningZones;

    // 시설 레이아웃 최적화 사용
    const optimizer = new DockPlacementOptimizer();

    return optimizer.optimize({
      floorPlan,
      zones,
      robotCount,
      constraints: {
        nearPowerOutlet: true,
        minimumClearance: 1.0,  // 미터
        accessibleFromAllZones: true,
        avoidHighTraffic: true
      }
    });
  }
}
```

### 8.2 현장 평가 및 계획

```typescript
// 현장 평가 서비스
class SiteAssessmentService {
  async assessSite(site: Site): Promise<SiteAssessment> {
    const assessments = await Promise.all([
      this.assessFloorConditions(site),
      this.assessWifiCoverage(site),
      this.assessObstacleEnvironment(site),
      this.assessPowerAvailability(site),
      this.assessTrafficPatterns(site),
      this.assessSpecialRequirements(site)
    ]);

    const overallScore = this.calculateOverallScore(assessments);
    const recommendations = this.generateRecommendations(assessments);
    const robotSizing = this.calculateRobotSizing(site, assessments);

    return {
      site,
      assessmentDate: new Date(),
      overallScore,
      passed: overallScore >= 70,
      categories: {
        floor: assessments[0],
        wifi: assessments[1],
        obstacles: assessments[2],
        power: assessments[3],
        traffic: assessments[4],
        special: assessments[5]
      },
      recommendations,
      robotSizing,
      estimatedROI: this.calculateROIEstimate(site, robotSizing)
    };
  }

  private async assessFloorConditions(site: Site): Promise<FloorAssessment> {
    const issues: FloorIssue[] = [];

    // 바닥 유형 확인
    const floorTypes = await this.identifyFloorTypes(site);
    for (const floor of floorTypes) {
      if (floor.type === 'CARPET' && floor.pileHeight > 25) {
        issues.push({
          type: 'HIGH_PILE_CARPET',
          location: floor.area,
          severity: 'MEDIUM',
          recommendation: '고파일 카펫 기능이 있는 로봇 고려'
        });
      }

      if (floor.type === 'UNEVEN') {
        issues.push({
          type: 'UNEVEN_SURFACE',
          location: floor.area,
          severity: 'HIGH',
          recommendation: '배포 전 바닥 평탄화 권장'
        });
      }
    }

    // 바닥 전환부 확인
    const transitions = await this.identifyFloorTransitions(site);
    for (const transition of transitions) {
      if (transition.height > 20) {  // mm
        issues.push({
          type: 'HIGH_THRESHOLD',
          location: transition.location,
          height: transition.height,
          severity: transition.height > 25 ? 'HIGH' : 'MEDIUM',
          recommendation: '문턱 경사로 설치 또는 높은 등반 능력의 로봇 선택'
        });
      }
    }

    // 문제 영역 식별
    const problematic = await this.identifyProblematicAreas(site);
    issues.push(...problematic);

    return {
      score: this.calculateFloorScore(issues),
      floorTypes,
      transitions,
      issues,
      recommendations: this.generateFloorRecommendations(issues)
    };
  }

  private async assessWifiCoverage(site: Site): Promise<WifiAssessment> {
    // WiFi 조사 수행
    const surveyPoints = await this.generateSurveyPoints(site);
    const measurements: WifiMeasurement[] = [];

    for (const point of surveyPoints) {
      const measurement = await this.measureWifiAtPoint(point);
      measurements.push(measurement);
    }

    // 커버리지 분석
    const coverage = this.analyzeWifiCoverage(measurements);

    // 취약 지점 식별
    const weakSpots = measurements.filter(m => m.signalStrength < -70);

    // 간섭 확인
    const interference = await this.checkWifiInterference(site);

    return {
      score: this.calculateWifiScore(coverage, weakSpots, interference),
      averageSignal: coverage.averageSignal,
      coverage: coverage.coveragePercentage,
      weakSpots: weakSpots.map(w => w.location),
      interference,
      needsImprovement: weakSpots.length > 0 || coverage.coveragePercentage < 90,
      recommendations: this.generateWifiRecommendations(coverage, weakSpots)
    };
  }

  private calculateRobotSizing(
    site: Site,
    assessments: any[]
  ): RobotSizing {
    const totalArea = site.totalCleanableArea;
    const complexity = this.calculateSiteComplexity(assessments);

    // 기본 계산: 복잡도에 따라 500-1000sqm당 1대 로봇
    const areaPerRobot = complexity === 'HIGH' ? 500 : complexity === 'MEDIUM' ? 750 : 1000;
    const baseCount = Math.ceil(totalArea / areaPerRobot);

    // 운영 요구사항에 따른 조정
    const operationalHours = site.requiredCleaningFrequency;
    const adjustedCount = this.adjustForOperations(baseCount, operationalHours, site);

    // 상업용 배포에 여유분 추가
    const finalCount = site.type === 'COMMERCIAL'
      ? Math.ceil(adjustedCount * 1.2)  // 20% 여유
      : adjustedCount;

    return {
      recommendedRobotCount: finalCount,
      minimumRobotCount: baseCount,
      robotType: this.recommendRobotType(site, assessments),
      dockStationsNeeded: Math.ceil(finalCount / 2),  // 로봇 2대당 도크 1개
      estimatedCoverage: this.estimateCoverage(site, finalCount),
      estimatedDailyCleaningCapacity: this.estimateDailyCapacity(site, finalCount)
    };
  }
}

// 배포 플래너
class DeploymentPlanner {
  async createDeploymentPlan(
    site: Site,
    assessment: SiteAssessment,
    requirements: DeploymentRequirements
  ): Promise<DeploymentPlan> {
    // 로봇 선택
    const robots = await this.selectRobots(assessment.robotSizing, requirements);

    // 도크 위치 계획
    const dockLocations = await this.planDockLocations(site, robots);

    // 구역 할당 계획
    const zoneAssignments = await this.planZoneAssignments(site, robots);

    // 스케줄 생성
    const schedules = await this.createCleaningSchedules(site, robots, requirements);

    // 통합 계획
    const integrations = await this.planIntegrations(site, requirements);

    // 타임라인 생성
    const timeline = this.createImplementationTimeline(site, robots);

    return {
      id: generateId(),
      site,
      robots,
      dockLocations,
      zoneAssignments,
      schedules,
      integrations,
      timeline,
      estimatedCost: this.calculateDeploymentCost(robots, integrations),
      estimatedDuration: this.calculateDeploymentDuration(timeline)
    };
  }

  private async selectRobots(
    sizing: RobotSizing,
    requirements: DeploymentRequirements
  ): Promise<RobotSelection[]> {
    const selections: RobotSelection[] = [];
    const availableModels = await this.getAvailableModels(requirements.vendor);

    // 요구사항을 로봇 기능과 매칭
    const matchingModels = availableModels.filter(model =>
      this.modelMeetsRequirements(model, requirements)
    );

    // 최적 모델 선택
    const optimalModel = this.selectOptimalModel(matchingModels, requirements);

    for (let i = 0; i < sizing.recommendedRobotCount; i++) {
      selections.push({
        model: optimalModel,
        quantity: 1,
        configuration: this.generateRobotConfiguration(optimalModel, requirements)
      });
    }

    return selections;
  }

  private createImplementationTimeline(
    site: Site,
    robots: RobotSelection[]
  ): ImplementationTimeline {
    const phases: TimelinePhase[] = [];

    // 1단계: 인프라 준비
    phases.push({
      name: '인프라 준비',
      duration: this.estimateInfraDuration(site),
      tasks: [
        'WiFi 평가 및 업그레이드',
        '전원 콘센트 설치',
        '바닥 준비',
        '도크 위치 표시'
      ],
      dependencies: []
    });

    // 2단계: 로봇 배송 및 설정
    phases.push({
      name: '로봇 설정',
      duration: robots.length * 0.5,  // 로봇당 0.5일
      tasks: [
        '로봇 개봉 및 검사',
        '펌웨어 업데이트',
        '초기 구성',
        '도크 설치'
      ],
      dependencies: ['인프라 준비']
    });

    // 3단계: 매핑
    phases.push({
      name: '매핑',
      duration: Math.ceil(site.totalArea / 1000),  // 1000sqm당 1일
      tasks: [
        '초기 매핑 실행',
        '지도 검증',
        '방 구분',
        '구역 구성'
      ],
      dependencies: ['로봇 설정']
    });

    // 4단계: 구성 및 테스트
    phases.push({
      name: '구성 및 테스트',
      duration: robots.length * 0.5 + 2,
      tasks: [
        '스케줄 구성',
        '통합 설정',
        '테스트 청소 실행',
        '성능 검증'
      ],
      dependencies: ['매핑']
    });

    // 5단계: 교육 및 인계
    phases.push({
      name: '교육 및 인계',
      duration: 2,
      tasks: [
        '직원 교육',
        '문서 인계',
        '지원 연락처 설정',
        '가동 시작'
      ],
      dependencies: ['구성 및 테스트']
    });

    return {
      phases,
      totalDuration: phases.reduce((sum, p) => sum + p.duration, 0),
      criticalPath: this.calculateCriticalPath(phases)
    };
  }
}
```

### 8.3 플릿 운영 관리

```typescript
// 플릿 운영 서비스
class FleetOperationsService {
  private fleetMonitor: FleetMonitor;
  private performanceAnalyzer: PerformanceAnalyzer;
  private alertManager: AlertManager;
  private workloadBalancer: WorkloadBalancer;

  async monitorFleet(fleetId: string): Promise<FleetStatus> {
    const robots = await this.getRobots(fleetId);

    const statuses = await Promise.all(
      robots.map(r => this.getRobotStatus(r.id))
    );

    const metrics = await this.calculateFleetMetrics(statuses);
    const alerts = await this.checkFleetAlerts(statuses);
    const recommendations = await this.generateOperationalRecommendations(metrics);

    return {
      fleetId,
      timestamp: new Date(),
      robotCount: robots.length,
      activeRobots: statuses.filter(s => s.state === 'CLEANING').length,
      chargingRobots: statuses.filter(s => s.state === 'CHARGING').length,
      idleRobots: statuses.filter(s => s.state === 'IDLE').length,
      errorRobots: statuses.filter(s => s.state === 'ERROR').length,
      metrics,
      alerts,
      recommendations
    };
  }

  private async calculateFleetMetrics(
    statuses: RobotStatus[]
  ): Promise<FleetMetrics> {
    const now = new Date();
    const dayStart = new Date(now.setHours(0, 0, 0, 0));

    // 활용도 메트릭
    const utilization = await this.calculateUtilization(statuses, dayStart);

    // 커버리지 메트릭
    const coverage = await this.calculateCoverageMetrics(statuses, dayStart);

    // 효율성 메트릭
    const efficiency = await this.calculateEfficiencyMetrics(statuses, dayStart);

    // 상태 메트릭
    const health = await this.calculateHealthMetrics(statuses);

    return {
      utilization: {
        overall: utilization.overall,
        byRobot: utilization.byRobot,
        trend: utilization.trend
      },
      coverage: {
        todayTotal: coverage.today,
        weeklyTotal: coverage.weekly,
        monthlyTotal: coverage.monthly,
        byZone: coverage.byZone
      },
      efficiency: {
        averageAreaPerHour: efficiency.areaPerHour,
        averageBatteryEfficiency: efficiency.batteryEfficiency,
        completionRate: efficiency.completionRate
      },
      health: {
        averageHealthScore: health.averageScore,
        robotsNeedingAttention: health.needsAttention,
        upcomingMaintenance: health.upcomingMaintenance
      }
    };
  }

  async optimizeFleetOperations(
    fleetId: string,
    period: DateRange
  ): Promise<OptimizationResult> {
    // 과거 성과 분석
    const historicalData = await this.getHistoricalData(fleetId, period);

    // 최적화 기회 식별
    const opportunities = await this.identifyOptimizations(historicalData);

    // 최적화 권장사항 생성
    const recommendations: OptimizationRecommendation[] = [];

    // 스케줄 최적화
    const scheduleOpt = await this.optimizeSchedules(fleetId, historicalData);
    if (scheduleOpt.improvement > 5) {
      recommendations.push({
        type: 'SCHEDULE_OPTIMIZATION',
        description: '교통 패턴에 따른 청소 스케줄 조정',
        currentValue: scheduleOpt.current,
        proposedValue: scheduleOpt.proposed,
        estimatedImprovement: `${scheduleOpt.improvement}% 효율성 향상`,
        implementation: scheduleOpt.implementation
      });
    }

    // 구역 할당 최적화
    const zoneOpt = await this.optimizeZoneAssignments(fleetId, historicalData);
    if (zoneOpt.improvement > 5) {
      recommendations.push({
        type: 'ZONE_OPTIMIZATION',
        description: '더 나은 커버리지를 위한 구역 할당 재조정',
        currentValue: zoneOpt.current,
        proposedValue: zoneOpt.proposed,
        estimatedImprovement: `${zoneOpt.improvement}% 커버리지 향상`,
        implementation: zoneOpt.implementation
      });
    }

    // 청소 설정 최적화
    const settingsOpt = await this.optimizeCleaningSettings(fleetId, historicalData);
    if (settingsOpt.recommendations.length > 0) {
      recommendations.push({
        type: 'SETTINGS_OPTIMIZATION',
        description: '바닥 유형 및 오염 패턴에 따른 청소 설정 조정',
        recommendations: settingsOpt.recommendations,
        estimatedImprovement: settingsOpt.estimatedImprovement
      });
    }

    return {
      fleetId,
      analysisDate: new Date(),
      period,
      recommendations,
      estimatedOverallImprovement: this.calculateOverallImprovement(recommendations),
      implementationPlan: this.createImplementationPlan(recommendations)
    };
  }

  private async optimizeSchedules(
    fleetId: string,
    historicalData: HistoricalData
  ): Promise<ScheduleOptimization> {
    // 교통 패턴 분석
    const trafficPatterns = this.analyzeTrafficPatterns(historicalData);

    // 최적 청소 시간대 식별
    const optimalWindows = this.identifyOptimalWindows(trafficPatterns);

    // 오염 축적 패턴 분석
    const dirtPatterns = this.analyzeDirtPatterns(historicalData);

    // 최적화된 스케줄 생성
    const currentSchedule = await this.getCurrentSchedule(fleetId);
    const optimizedSchedule = this.generateOptimizedSchedule(
      currentSchedule,
      optimalWindows,
      dirtPatterns
    );

    // 개선도 계산
    const improvement = this.calculateScheduleImprovement(
      currentSchedule,
      optimizedSchedule,
      historicalData
    );

    return {
      current: currentSchedule,
      proposed: optimizedSchedule,
      improvement,
      implementation: {
        steps: [
          '제안된 스케줄 변경 검토',
          '영향받는 직원에게 알림',
          '플릿 관리에서 로봇 스케줄 업데이트',
          '2주간 성능 모니터링',
          '필요시 조정'
        ],
        estimatedTime: '1-2시간',
        riskLevel: 'LOW'
      }
    };
  }
}

// 성능 분석 서비스
class PerformanceAnalyticsService {
  async generatePerformanceReport(
    fleetId: string,
    period: DateRange
  ): Promise<PerformanceReport> {
    const data = await this.collectPerformanceData(fleetId, period);

    return {
      fleetId,
      period,
      generatedAt: new Date(),

      summary: {
        totalCleaningSessions: data.sessions.length,
        totalAreaCleaned: data.totalArea,
        totalOperatingHours: data.operatingHours,
        averageDailyArea: data.totalArea / data.dayCount
      },

      robotPerformance: await this.analyzeRobotPerformance(data),

      coverageAnalysis: {
        overallCoverage: data.coverage.overall,
        coverageByZone: data.coverage.byZone,
        missedAreas: data.coverage.missed,
        coverageTrend: this.calculateTrend(data.coverage.daily)
      },

      efficiencyMetrics: {
        areaPerBatteryPercent: data.efficiency.areaPerBattery,
        areaPerHour: data.efficiency.areaPerHour,
        pathEfficiency: data.efficiency.pathEfficiency,
        efficiencyTrend: this.calculateTrend(data.efficiency.daily)
      },

      reliabilityMetrics: {
        completionRate: data.reliability.completionRate,
        errorRate: data.reliability.errorRate,
        mtbf: data.reliability.mtbf,
        availability: data.reliability.availability
      },

      issueAnalysis: {
        totalIssues: data.issues.length,
        issuesByType: this.groupByType(data.issues),
        issuesTrend: this.calculateTrend(data.issues),
        topIssueLocations: this.identifyProblemAreas(data.issues)
      },

      recommendations: await this.generateRecommendations(data),

      benchmarks: await this.compareToBenchmarks(data)
    };
  }

  private async analyzeRobotPerformance(
    data: PerformanceData
  ): Promise<RobotPerformanceAnalysis[]> {
    const analyses: RobotPerformanceAnalysis[] = [];

    for (const robot of data.robots) {
      const robotData = data.sessions.filter(s => s.robotId === robot.id);

      analyses.push({
        robotId: robot.id,
        robotName: robot.nickname,

        metrics: {
          totalSessions: robotData.length,
          totalArea: robotData.reduce((sum, s) => sum + s.area, 0),
          totalOperatingTime: robotData.reduce((sum, s) => sum + s.duration, 0),
          averageSessionArea: this.average(robotData.map(s => s.area)),
          averageSessionDuration: this.average(robotData.map(s => s.duration)),
          averageCoverage: this.average(robotData.map(s => s.coverage))
        },

        efficiency: {
          areaPerHour: this.calculateAreaPerHour(robotData),
          batteryEfficiency: this.calculateBatteryEfficiency(robotData),
          pathEfficiency: this.calculatePathEfficiency(robotData)
        },

        reliability: {
          completionRate: robotData.filter(s => s.completed).length / robotData.length,
          errorCount: robotData.filter(s => s.hadError).length,
          stuckCount: robotData.filter(s => s.gotStuck).length
        },

        health: {
          currentHealth: robot.healthScore,
          batteryHealth: robot.batteryHealth,
          maintenanceStatus: robot.maintenanceStatus
        },

        ranking: {
          efficiencyRank: 0,  // 모든 로봇 후 계산
          reliabilityRank: 0,
          overallRank: 0
        }
      });
    }

    // 순위 계산
    this.calculateRankings(analyses);

    return analyses;
  }
}
```

### 8.4 유지보수 관리

```typescript
// 유지보수 관리 서비스
class MaintenanceManagementService {
  private componentTracker: ComponentTracker;
  private maintenanceScheduler: MaintenanceScheduler;
  private partsInventory: PartsInventory;
  private predictiveAnalytics: PredictiveMaintenanceAnalytics;

  async getMaintenanceStatus(robotId: string): Promise<MaintenanceStatus> {
    const robot = await this.getRobot(robotId);
    const components = await this.componentTracker.getComponentStatus(robotId);

    const maintenanceNeeds = this.assessMaintenanceNeeds(components);
    const predictions = await this.predictiveAnalytics.predictFailures(robotId);
    const scheduledMaintenance = await this.maintenanceScheduler.getScheduled(robotId);

    return {
      robotId,
      overallHealth: this.calculateOverallHealth(components),
      components,
      maintenanceNeeds,
      predictions,
      scheduledMaintenance,
      recommendations: this.generateMaintenanceRecommendations(
        maintenanceNeeds,
        predictions
      )
    };
  }

  async trackComponentWear(robotId: string): Promise<ComponentWearReport> {
    const robot = await this.getRobot(robotId);
    const telemetry = await this.getTelemetryHistory(robotId, 30);  // 30일

    const components: ComponentWear[] = [];

    // 메인 브러시 마모
    const mainBrush = await this.calculateBrushWear(telemetry, 'main');
    components.push({
      componentId: 'main_brush',
      componentName: '메인 브러시',
      currentCondition: mainBrush.condition,
      estimatedLife: mainBrush.remainingLife,
      usageHours: mainBrush.usageHours,
      replacementThreshold: 85,
      estimatedReplacementDate: mainBrush.replacementDate,
      partNumber: robot.model.parts.mainBrush
    });

    // 사이드 브러시 마모
    const sideBrush = await this.calculateBrushWear(telemetry, 'side');
    components.push({
      componentId: 'side_brush',
      componentName: '사이드 브러시',
      currentCondition: sideBrush.condition,
      estimatedLife: sideBrush.remainingLife,
      usageHours: sideBrush.usageHours,
      replacementThreshold: 90,
      estimatedReplacementDate: sideBrush.replacementDate,
      partNumber: robot.model.parts.sideBrush
    });

    // 필터 마모
    const filter = await this.calculateFilterWear(telemetry);
    components.push({
      componentId: 'filter',
      componentName: 'HEPA 필터',
      currentCondition: filter.condition,
      estimatedLife: filter.remainingLife,
      usageHours: filter.usageHours,
      replacementThreshold: 80,
      estimatedReplacementDate: filter.replacementDate,
      partNumber: robot.model.parts.filter
    });

    // 물걸레 패드 마모 (해당되는 경우)
    if (robot.capabilities.mopping) {
      const mopPad = await this.calculateMopPadWear(telemetry);
      components.push({
        componentId: 'mop_pad',
        componentName: '물걸레 패드',
        currentCondition: mopPad.condition,
        estimatedLife: mopPad.remainingLife,
        washCycles: mopPad.washCycles,
        replacementThreshold: 90,
        estimatedReplacementDate: mopPad.replacementDate,
        partNumber: robot.model.parts.mopPad
      });
    }

    // 배터리 상태
    const battery = await this.calculateBatteryHealth(telemetry);
    components.push({
      componentId: 'battery',
      componentName: '배터리',
      currentCondition: battery.healthPercentage,
      estimatedLife: battery.remainingCycles,
      chargeCycles: battery.totalCycles,
      replacementThreshold: 70,
      estimatedReplacementDate: battery.replacementDate,
      partNumber: robot.model.parts.battery
    });

    // 바퀴
    const wheels = await this.calculateWheelWear(telemetry);
    components.push({
      componentId: 'wheels',
      componentName: '구동 바퀴',
      currentCondition: wheels.condition,
      estimatedLife: wheels.remainingLife,
      distanceTraveled: wheels.totalDistance,
      replacementThreshold: 80,
      estimatedReplacementDate: wheels.replacementDate,
      partNumber: robot.model.parts.wheels
    });

    return {
      robotId,
      reportDate: new Date(),
      components,
      urgentReplacements: components.filter(c => c.currentCondition < c.replacementThreshold),
      upcomingReplacements: components.filter(c =>
        c.currentCondition < c.replacementThreshold + 10 &&
        c.currentCondition >= c.replacementThreshold
      )
    };
  }

  async scheduleMaintenance(
    robotId: string,
    maintenanceType: MaintenanceType
  ): Promise<ScheduledMaintenance> {
    const robot = await this.getRobot(robotId);
    const currentSchedule = await this.maintenanceScheduler.getScheduled(robotId);

    // 최적 유지보수 시간대 찾기
    const optimalWindow = await this.findOptimalMaintenanceWindow(
      robot,
      maintenanceType
    );

    // 부품 가용성 확인
    const partsNeeded = this.getPartsForMaintenance(maintenanceType, robot.model);
    const partsAvailability = await this.partsInventory.checkAvailability(partsNeeded);

    if (!partsAvailability.allAvailable) {
      // 부족 부품 주문
      await this.partsInventory.orderParts(partsAvailability.missing);
    }

    // 유지보수 스케줄 생성
    const scheduled = await this.maintenanceScheduler.schedule({
      robotId,
      maintenanceType,
      scheduledDate: optimalWindow.start,
      estimatedDuration: this.estimateMaintenanceDuration(maintenanceType),
      partsNeeded,
      technician: await this.assignTechnician(optimalWindow),
      instructions: this.getMaintenanceInstructions(maintenanceType, robot.model)
    });

    // 관련 당사자에게 알림
    await this.notifyMaintenanceScheduled(scheduled);

    return scheduled;
  }

  private async findOptimalMaintenanceWindow(
    robot: CleaningRobot,
    maintenanceType: MaintenanceType
  ): Promise<TimeWindow> {
    const schedule = await this.getCleaningSchedule(robot.id);
    const businessHours = await this.getBusinessHours(robot.siteId);

    // 청소 스케줄의 빈 시간 찾기
    const gaps = this.findScheduleGaps(schedule);

    // 유지보수 요구사항에 따른 필터링
    const duration = this.estimateMaintenanceDuration(maintenanceType);
    const suitableGaps = gaps.filter(g => g.duration >= duration);

    // 간단한 유지보수는 업무 시간 중 선호
    if (maintenanceType === 'ROUTINE') {
      return suitableGaps.find(g => this.isWithinBusinessHours(g, businessHours))
        || suitableGaps[0];
    }

    // 주요 유지보수는 업무 외 시간 선호
    return suitableGaps.find(g => !this.isWithinBusinessHours(g, businessHours))
      || suitableGaps[0];
  }
}

// 예측 유지보수 분석
class PredictiveMaintenanceAnalytics {
  private mlModel: MaintenancePredictor;

  async predictFailures(robotId: string): Promise<FailurePrediction[]> {
    // 과거 원격 측정 데이터 가져오기
    const telemetry = await this.getTelemetryHistory(robotId, 90);  // 90일

    // 특성 추출
    const features = this.extractFeatures(telemetry);

    // 예측 모델 실행
    const predictions: FailurePrediction[] = [];

    // 메인 브러시 고장 예측
    const brushPrediction = await this.mlModel.predict('main_brush', features);
    if (brushPrediction.probability > 0.3) {
      predictions.push({
        component: 'main_brush',
        failureType: 'WEAR_OUT',
        probability: brushPrediction.probability,
        estimatedTimeToFailure: brushPrediction.timeToFailure,
        confidence: brushPrediction.confidence,
        indicators: brushPrediction.indicators
      });
    }

    // 배터리 성능 저하 예측
    const batteryPrediction = await this.mlModel.predict('battery', features);
    if (batteryPrediction.probability > 0.2) {
      predictions.push({
        component: 'battery',
        failureType: 'CAPACITY_DEGRADATION',
        probability: batteryPrediction.probability,
        estimatedTimeToFailure: batteryPrediction.timeToFailure,
        confidence: batteryPrediction.confidence,
        indicators: batteryPrediction.indicators
      });
    }

    // 모터 문제 예측
    const motorPrediction = await this.mlModel.predict('drive_motor', features);
    if (motorPrediction.probability > 0.25) {
      predictions.push({
        component: 'drive_motor',
        failureType: 'MOTOR_DEGRADATION',
        probability: motorPrediction.probability,
        estimatedTimeToFailure: motorPrediction.timeToFailure,
        confidence: motorPrediction.confidence,
        indicators: motorPrediction.indicators
      });
    }

    return predictions.sort((a, b) => b.probability - a.probability);
  }

  private extractFeatures(telemetry: TelemetryData[]): FeatureVector {
    return {
      // 브러시 특성
      avgBrushCurrent: this.average(telemetry.map(t => t.motors.mainBrush.current)),
      brushCurrentVariance: this.variance(telemetry.map(t => t.motors.mainBrush.current)),
      brushCurrentTrend: this.calculateTrend(telemetry.map(t => t.motors.mainBrush.current)),

      // 모터 특성
      avgMotorCurrent: this.average(telemetry.map(t =>
        (t.motors.leftWheel.current + t.motors.rightWheel.current) / 2
      )),
      motorCurrentVariance: this.variance(telemetry.map(t =>
        (t.motors.leftWheel.current + t.motors.rightWheel.current) / 2
      )),

      // 배터리 특성
      avgBatteryVoltage: this.average(telemetry.map(t => t.power.batteryVoltage)),
      batteryVoltageDrop: this.calculateVoltageDrop(telemetry),
      chargeCycles: this.countChargeCycles(telemetry),

      // 사용 특성
      totalOperatingHours: this.calculateOperatingHours(telemetry),
      avgDailyUsage: this.calculateDailyUsage(telemetry),
      highLoadDuration: this.calculateHighLoadDuration(telemetry)
    };
  }
}
```

### 8.5 모니터링 및 알림

```typescript
// 플릿 모니터링 대시보드 서비스
class FleetMonitoringService {
  private robotMonitors: Map<string, RobotMonitor>;
  private alertManager: AlertManager;
  private dashboardService: DashboardService;

  async setupMonitoring(fleetId: string): Promise<void> {
    const robots = await this.getRobots(fleetId);

    for (const robot of robots) {
      const monitor = new RobotMonitor(robot);
      this.robotMonitors.set(robot.id, monitor);

      // 실시간 모니터링 시작
      monitor.startMonitoring({
        stateInterval: 1000,      // 1초
        telemetryInterval: 5000,  // 5초
        healthCheckInterval: 60000  // 1분
      });

      // 알림 핸들러 설정
      monitor.on('alert', (alert) => this.handleAlert(alert));
      monitor.on('stateChange', (state) => this.handleStateChange(robot.id, state));
    }

    // 대시보드 업데이트 시작
    this.startDashboardUpdates(fleetId);
  }

  private async handleAlert(alert: RobotAlert): Promise<void> {
    // 알림 심각도 분류
    const severity = this.classifyAlertSeverity(alert);

    // 알림 기록 생성
    const alertRecord = await this.alertManager.createAlert({
      ...alert,
      severity,
      timestamp: new Date(),
      status: 'ACTIVE'
    });

    // 심각도에 따른 알림
    switch (severity) {
      case 'CRITICAL':
        await this.notifyImmediate(alertRecord);
        await this.escalateAlert(alertRecord);
        break;

      case 'HIGH':
        await this.notifyImmediate(alertRecord);
        break;

      case 'MEDIUM':
        await this.notifyStandard(alertRecord);
        break;

      case 'LOW':
        await this.logAlert(alertRecord);
        break;
    }

    // 가능한 경우 자동 대응
    await this.attemptAutoResponse(alertRecord);
  }

  private async attemptAutoResponse(alert: RobotAlert): Promise<void> {
    const autoResponses: { [key: string]: () => Promise<boolean> } = {
      'DUSTBIN_FULL': async () => {
        // 자동 비우기 가능한 경우 도크로 복귀
        const robot = await this.getRobot(alert.robotId);
        if (robot.capabilities.docking.autoEmptyDust) {
          await this.robotService.returnToDock(alert.robotId);
          return true;
        }
        return false;
      },

      'WATER_EMPTY': async () => {
        // 자동 물 채우기 가능한 경우 도크로 복귀
        const robot = await this.getRobot(alert.robotId);
        if (robot.capabilities.docking.autoRefillWater) {
          await this.robotService.returnToDock(alert.robotId);
          return true;
        }
        return false;
      },

      'LOW_BATTERY': async () => {
        // 도크로 복귀
        await this.robotService.returnToDock(alert.robotId);
        return true;
      },

      'STUCK': async () => {
        // 복구 시도
        const recovered = await this.robotService.attemptRecovery(alert.robotId);
        return recovered;
      },

      'WIFI_DISCONNECTED': async () => {
        // 재연결 대기 (로봇에서 처리)
        return false;
      }
    };

    const responseHandler = autoResponses[alert.type];
    if (responseHandler) {
      const handled = await responseHandler();
      if (handled) {
        await this.alertManager.updateAlert(alert.id, {
          status: 'AUTO_RESOLVED',
          resolvedAt: new Date(),
          resolution: '자동 대응 실행됨'
        });
      }
    }
  }

  async getDashboardData(fleetId: string): Promise<DashboardData> {
    const status = await this.getFleetStatus(fleetId);
    const metrics = await this.getFleetMetrics(fleetId);
    const alerts = await this.getActiveAlerts(fleetId);
    const robotDetails = await this.getRobotDetails(fleetId);

    return {
      fleetId,
      timestamp: new Date(),

      summary: {
        totalRobots: status.robotCount,
        activeRobots: status.activeRobots,
        robotsOnDock: status.chargingRobots + status.idleRobots,
        robotsWithIssues: status.errorRobots
      },

      liveStatus: robotDetails.map(r => ({
        id: r.id,
        name: r.nickname,
        state: r.state,
        battery: r.batteryLevel,
        currentTask: r.currentTask,
        position: r.position,
        health: r.healthScore
      })),

      todayMetrics: {
        areaCleaned: metrics.coverage.todayTotal,
        sessionsCompleted: metrics.utilization.sessionsToday,
        averageCoverage: metrics.efficiency.averageCoverage,
        issues: alerts.filter(a => this.isToday(a.timestamp)).length
      },

      alerts: alerts.map(a => ({
        id: a.id,
        robotId: a.robotId,
        type: a.type,
        severity: a.severity,
        message: a.message,
        timestamp: a.timestamp
      })),

      schedule: await this.getUpcomingSchedule(fleetId),

      maintenanceAlerts: await this.getMaintenanceAlerts(fleetId)
    };
  }
}
```

### 8.6 구현 모범 사례 요약

```yaml
청소 로봇 구현 모범 사례:

  배포 계획:
    - 철저한 현장 평가 필수
    - WiFi 커버리지 사전 검증
    - 적절한 로봇 수량 산정
    - 단계별 배포 접근

  플릿 운영:
    - 실시간 모니터링 구축
    - 데이터 기반 최적화
    - 자동 알림 및 에스컬레이션
    - 정기적인 성능 분석

  유지보수:
    - 예측 유지보수 도입
    - 부품 재고 관리
    - 정기 점검 일정
    - 기술자 교육

  지속적 개선:
    - KPI 기반 관리
    - 사용자 피드백 수집
    - 운영 데이터 분석
    - 업계 모범 사례 벤치마킹

  弘益人間 구현 원칙:
    - 사용자 경험 최우선
    - 효율적인 자원 활용
    - 환경 영향 최소화
    - 접근 가능한 자동화
```

---

**WIA-CLEANING-ROBOT 구현**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
