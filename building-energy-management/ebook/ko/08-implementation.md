# 제8장: 구현 가이드 및 모범 사례

## 서론

WIA-BEMS의 성공적인 구현은 철저한 계획, 단계적 배포, 그리고 지속적인 최적화를 필요로 합니다. 본 장에서는 프로젝트 계획부터 운영 안정화까지의 전 과정에 대한 실무 가이드를 제공합니다.

---

## 8.1 프로젝트 계획 및 평가

### 8.1.1 사전 평가

#### 에너지 진단

```typescript
// 에너지 진단 프레임워크
interface EnergyAuditFramework {
  level1_walkthrough: {
    name: '예비 진단';
    duration: '1-3일';
    activities: [
      '현장 방문 및 관찰',
      '유틸리티 요금 분석',
      '설비 목록 작성',
      '운영 패턴 파악',
      '주요 에너지 소비처 식별'
    ];
    deliverables: ['예비 진단 보고서', '개략 절감 가능성'];
  };

  level2_energyAudit: {
    name: '표준 에너지 진단';
    duration: '2-4주';
    activities: [
      '상세 에너지 사용 분석',
      '설비 효율 측정',
      '부하 프로파일 분석',
      '에너지 기준선 수립',
      '절감 대안 분석',
      '투자 대비 효과 분석'
    ];
    deliverables: ['에너지 진단 보고서', '절감 대안 목록', '투자 회수 분석'];
  };

  level3_investmentGrade: {
    name: '투자 등급 진단';
    duration: '4-8주';
    activities: [
      '상세 계측 및 모니터링',
      '시뮬레이션 모델링',
      '상세 설계 및 사양',
      '정밀 비용 산정',
      '위험 분석'
    ];
    deliverables: ['상세 설계 문서', '정밀 투자 분석', '구현 계획'];
  };
}

// 에너지 진단 수행자
class EnergyAuditor {
  private building: Building;
  private utilityData: UtilityData;
  private measurementData: MeasurementData;

  async conductAudit(level: AuditLevel): Promise<AuditReport> {
    // 1. 데이터 수집
    const data = await this.collectData(level);

    // 2. 에너지 기준선 분석
    const baseline = this.establishBaseline(data);

    // 3. 절감 기회 식별
    const opportunities = await this.identifyOpportunities(data, baseline);

    // 4. 대안 분석
    const alternatives = await this.analyzeAlternatives(opportunities);

    // 5. 투자 분석
    const financialAnalysis = this.conductFinancialAnalysis(alternatives);

    // 6. 우선순위 설정
    const prioritized = this.prioritizeActions(alternatives, financialAnalysis);

    return {
      executiveSummary: this.generateExecutiveSummary(baseline, prioritized),
      baseline,
      opportunities,
      recommendations: prioritized,
      financialAnalysis,
      implementationRoadmap: this.createRoadmap(prioritized)
    };
  }

  private establishBaseline(data: AuditData): EnergyBaseline {
    // 에너지 사용 기준선 수립
    const annualConsumption = this.calculateAnnualConsumption(data.utilityBills);
    const loadProfile = this.analyzeLoadProfile(data.intervalData);
    const weatherNormalized = this.normalizeForWeather(annualConsumption, data.weather);

    return {
      period: data.period,
      annualConsumption: {
        electricity: annualConsumption.electricity,
        gas: annualConsumption.gas,
        water: annualConsumption.water
      },
      eui: weatherNormalized.electricity / this.building.grossFloorArea,
      peakDemand: loadProfile.peakDemand,
      loadFactor: loadProfile.loadFactor,
      baseload: loadProfile.baseload,
      weatherSensitivity: weatherNormalized.sensitivity,
      benchmark: this.compareToBenchmark(weatherNormalized)
    };
  }

  private async identifyOpportunities(
    data: AuditData,
    baseline: EnergyBaseline
  ): Promise<SavingsOpportunity[]> {
    const opportunities: SavingsOpportunity[] = [];

    // HVAC 관련 기회
    opportunities.push(...await this.assessHVACOpportunities(data));

    // 조명 관련 기회
    opportunities.push(...await this.assessLightingOpportunities(data));

    // 제어 관련 기회
    opportunities.push(...await this.assessControlOpportunities(data));

    // 외피 관련 기회
    opportunities.push(...await this.assessEnvelopeOpportunities(data));

    // 운영 관련 기회
    opportunities.push(...await this.assessOperationalOpportunities(data, baseline));

    return opportunities;
  }

  private async assessControlOpportunities(
    data: AuditData
  ): Promise<SavingsOpportunity[]> {
    const opportunities: SavingsOpportunity[] = [];

    // BEMS 도입/업그레이드
    if (!data.existingBEMS || data.existingBEMS.age > 10) {
      opportunities.push({
        id: 'ECM-CTRL-001',
        category: 'control',
        name: 'BEMS 신규 도입/업그레이드',
        description: 'WIA-BEMS 표준 기반 빌딩 에너지 관리 시스템 도입',
        estimatedSavings: {
          electricity: data.baseline.electricity * 0.15, // 15% 절감 예상
          gas: data.baseline.gas * 0.10,
          annual: this.calculateCostSavings(
            data.baseline.electricity * 0.15,
            data.baseline.gas * 0.10,
            data.utilityRates
          )
        },
        implementationCost: this.estimateBEMSCost(this.building),
        simplePayback: null, // 계산 필요
        complexity: 'medium',
        priority: 'high'
      });
    }

    // 최적 기동/정지
    if (!data.existingBEMS?.hasOptimalStartStop) {
      opportunities.push({
        id: 'ECM-CTRL-002',
        category: 'control',
        name: '최적 기동/정지 제어',
        description: '기상 예보 및 열적 모델 기반 HVAC 기동/정지 최적화',
        estimatedSavings: {
          electricity: data.baseline.electricity * 0.05,
          annual: this.calculateCostSavings(
            data.baseline.electricity * 0.05,
            0,
            data.utilityRates
          )
        },
        implementationCost: data.existingBEMS ? 5000000 : 0, // BEMS 있으면 별도 비용
        complexity: 'low',
        priority: 'high'
      });
    }

    // 수요 응답
    opportunities.push({
      id: 'ECM-CTRL-003',
      category: 'control',
      name: '수요 응답 참여',
      description: 'OpenADR 기반 자동 수요 응답 시스템 구축',
      estimatedSavings: {
        demandReduction: data.baseline.peakDemand * 0.15,
        incentive: this.estimateDRIncentive(data.baseline.peakDemand * 0.15),
        annual: this.estimateDRIncentive(data.baseline.peakDemand * 0.15)
      },
      implementationCost: 20000000,
      complexity: 'medium',
      priority: 'medium'
    });

    return opportunities;
  }

  private estimateBEMSCost(building: Building): number {
    // 건물 규모에 따른 BEMS 비용 추정
    const baseCost = 50000000; // 기본 비용 5천만원
    const perSqmCost = 500; // ㎡당 500원

    const hardwareCost = baseCost + building.grossFloorArea * perSqmCost;
    const softwareCost = hardwareCost * 0.3;
    const installationCost = hardwareCost * 0.4;
    const trainingCost = 10000000;

    return hardwareCost + softwareCost + installationCost + trainingCost;
  }

  private createRoadmap(
    recommendations: PrioritizedRecommendation[]
  ): ImplementationRoadmap {
    const phases: RoadmapPhase[] = [];

    // Phase 1: 즉시 실행 (비용 없음/낮음)
    const phase1Items = recommendations.filter(r =>
      r.implementationCost < 10000000 && r.complexity === 'low'
    );

    phases.push({
      name: 'Phase 1: 즉시 실행',
      duration: '1-3개월',
      items: phase1Items,
      totalCost: phase1Items.reduce((sum, r) => sum + r.implementationCost, 0),
      expectedSavings: phase1Items.reduce((sum, r) => sum + r.estimatedSavings.annual, 0)
    });

    // Phase 2: 단기 (BEMS 기반 구축)
    const phase2Items = recommendations.filter(r =>
      r.category === 'control' && r.priority === 'high'
    );

    phases.push({
      name: 'Phase 2: BEMS 기반 구축',
      duration: '3-6개월',
      items: phase2Items,
      totalCost: phase2Items.reduce((sum, r) => sum + r.implementationCost, 0),
      expectedSavings: phase2Items.reduce((sum, r) => sum + r.estimatedSavings.annual, 0)
    });

    // Phase 3: 중기 (고급 기능)
    const phase3Items = recommendations.filter(r =>
      r.priority === 'medium' && r.complexity !== 'high'
    );

    phases.push({
      name: 'Phase 3: 고급 기능 구현',
      duration: '6-12개월',
      items: phase3Items,
      totalCost: phase3Items.reduce((sum, r) => sum + r.implementationCost, 0),
      expectedSavings: phase3Items.reduce((sum, r) => sum + r.estimatedSavings.annual, 0)
    });

    return {
      phases,
      totalInvestment: phases.reduce((sum, p) => sum + p.totalCost, 0),
      totalAnnualSavings: phases.reduce((sum, p) => sum + p.expectedSavings, 0),
      overallPayback: this.calculateOverallPayback(phases)
    };
  }
}
```

### 8.1.2 요구사항 정의

```typescript
// 요구사항 정의 프레임워크
interface RequirementsDefinition {
  functionalRequirements: {
    dataCollection: {
      id: 'FR-DC';
      requirements: [
        'FR-DC-001: 15분 간격 에너지 데이터 수집',
        'FR-DC-002: 5분 간격 환경 데이터 수집',
        'FR-DC-003: 실시간 설비 상태 모니터링',
        'FR-DC-004: 다중 프로토콜 지원 (BACnet, Modbus)'
      ];
    };
    analytics: {
      id: 'FR-AN';
      requirements: [
        'FR-AN-001: 에너지 기준선 자동 생성',
        'FR-AN-002: 이상 감지 및 알림',
        'FR-AN-003: 부하 예측 (24시간)',
        'FR-AN-004: 에너지 성능 리포팅'
      ];
    };
    control: {
      id: 'FR-CT';
      requirements: [
        'FR-CT-001: 설정값 원격 변경',
        'FR-CT-002: 스케줄 관리',
        'FR-CT-003: 수요 응답 자동 실행',
        'FR-CT-004: 최적 기동/정지'
      ];
    };
    integration: {
      id: 'FR-IN';
      requirements: [
        'FR-IN-001: BAS 시스템 연동',
        'FR-IN-002: 기상 데이터 연동',
        'FR-IN-003: 전력 거래소 연동',
        'FR-IN-004: CMMS 연동'
      ];
    };
  };

  nonFunctionalRequirements: {
    performance: {
      id: 'NFR-PF';
      requirements: [
        'NFR-PF-001: API 응답 시간 < 500ms (95%)',
        'NFR-PF-002: 대시보드 로딩 < 3초',
        'NFR-PF-003: 1년 데이터 쿼리 < 10초'
      ];
    };
    availability: {
      id: 'NFR-AV';
      requirements: [
        'NFR-AV-001: 시스템 가용성 99.9%',
        'NFR-AV-002: 계획 정지 < 4시간/월',
        'NFR-AV-003: 장애 복구 시간 < 1시간'
      ];
    };
    security: {
      id: 'NFR-SC';
      requirements: [
        'NFR-SC-001: TLS 1.3 암호화',
        'NFR-SC-002: MFA 지원',
        'NFR-SC-003: 감사 로그 3년 보관'
      ];
    };
    scalability: {
      id: 'NFR-SL';
      requirements: [
        'NFR-SL-001: 100개 건물 지원',
        'NFR-SL-002: 10,000 포인트/건물',
        'NFR-SL-003: 100 동시 사용자'
      ];
    };
  };
}

// 요구사항 추적 매트릭스
class RequirementsTraceability {
  private requirements: Requirement[];
  private traceMatrix: TraceabilityMatrix;

  generateTraceabilityMatrix(): TraceabilityMatrix {
    return {
      headers: ['요구사항 ID', '설명', '설계 요소', '테스트 케이스', '상태'],
      rows: this.requirements.map(req => ({
        requirementId: req.id,
        description: req.description,
        designElements: this.getDesignElements(req.id),
        testCases: this.getTestCases(req.id),
        status: this.getStatus(req.id)
      }))
    };
  }

  validateCoverage(): CoverageReport {
    const totalRequirements = this.requirements.length;
    const designed = this.requirements.filter(r =>
      this.getDesignElements(r.id).length > 0
    ).length;
    const tested = this.requirements.filter(r =>
      this.getTestCases(r.id).length > 0
    ).length;

    return {
      totalRequirements,
      designedCount: designed,
      designedPercent: (designed / totalRequirements) * 100,
      testedCount: tested,
      testedPercent: (tested / totalRequirements) * 100,
      gaps: this.identifyGaps()
    };
  }
}
```

---

## 8.2 단계별 구현 전략

### 8.2.1 파일럿 배포

```typescript
// 파일럿 배포 전략
interface PilotDeploymentStrategy {
  siteSelection: {
    criteria: [
      '대표성: 포트폴리오 내 유사 건물이 많은 유형',
      '복잡성: 중간 수준의 복잡성',
      '협조: 시설 관리팀의 적극적 협조',
      '접근성: 빈번한 방문 가능',
      '안정성: 운영 안정성이 높은 건물'
    ];
    antiPattern: [
      '가장 문제가 많은 건물 선정',
      '가장 크거나 복잡한 건물 선정',
      '협조가 부족한 현장 선정'
    ];
  };

  scope: {
    systems: ['HVAC 제어 최적화', '에너지 모니터링', '기본 FDD'];
    duration: '3-6개월';
    successCriteria: [
      '시스템 안정성 99% 이상',
      '에너지 절감 10% 이상',
      '사용자 만족도 70점 이상'
    ];
  };

  rollbackPlan: {
    triggers: ['시스템 장애 지속', '안전 문제 발생', '쾌적성 민원 급증'];
    procedure: ['이전 제어 로직 복원', '수동 운전 전환', '원인 분석'];
  };
}

// 파일럿 관리자
class PilotManager {
  private pilot: PilotProject;
  private monitoring: PilotMonitoring;

  async executePilot(): Promise<PilotResult> {
    // 1. 사전 준비
    await this.preparePilot();

    // 2. 배포
    await this.deploySystem();

    // 3. 모니터링 (전 기간)
    const monitoringResults = await this.monitorPilot();

    // 4. 평가
    const evaluation = await this.evaluatePilot(monitoringResults);

    // 5. 결과 보고
    return this.generatePilotReport(evaluation);
  }

  private async preparePilot(): Promise<void> {
    // 기준선 데이터 수집 (최소 1개월)
    console.log('기준선 데이터 수집 중...');
    await this.collectBaselineData(30);

    // 현장 준비
    await this.prepareSite();

    // 교육
    await this.conductTraining();
  }

  private async deploySystem(): Promise<void> {
    // 단계별 배포
    const deploymentPhases = [
      {
        name: '모니터링 전용',
        duration: '2주',
        features: ['데이터 수집', '대시보드', '알람'],
        controlEnabled: false
      },
      {
        name: '제한적 제어',
        duration: '4주',
        features: ['설정값 변경', '스케줄 관리'],
        controlEnabled: true,
        controlScope: 'non_critical'
      },
      {
        name: '전체 기능',
        duration: '6주',
        features: ['최적 제어', '자동 DR', 'FDD'],
        controlEnabled: true,
        controlScope: 'full'
      }
    ];

    for (const phase of deploymentPhases) {
      console.log(`배포 단계: ${phase.name}`);
      await this.executeDeploymentPhase(phase);
      await this.validatePhase(phase);
    }
  }

  private async monitorPilot(): Promise<MonitoringResults> {
    const results: MonitoringResults = {
      systemHealth: [],
      energyPerformance: [],
      userFeedback: [],
      incidents: []
    };

    // 지속적 모니터링
    const monitoringInterval = setInterval(async () => {
      results.systemHealth.push(await this.checkSystemHealth());
      results.energyPerformance.push(await this.measureEnergyPerformance());
    }, 3600000); // 1시간 간격

    // 파일럿 기간 대기
    await this.waitForPilotDuration();

    clearInterval(monitoringInterval);

    // 사용자 피드백 수집
    results.userFeedback = await this.collectUserFeedback();

    return results;
  }

  private async evaluatePilot(
    results: MonitoringResults
  ): Promise<PilotEvaluation> {
    // 성공 기준 평가
    const criteria = this.pilot.successCriteria;

    const systemStability = this.calculateSystemStability(results.systemHealth);
    const energySavings = this.calculateEnergySavings(results.energyPerformance);
    const userSatisfaction = this.calculateUserSatisfaction(results.userFeedback);

    return {
      systemStability: {
        target: criteria.systemStability,
        actual: systemStability,
        passed: systemStability >= criteria.systemStability
      },
      energySavings: {
        target: criteria.energySavings,
        actual: energySavings,
        passed: energySavings >= criteria.energySavings
      },
      userSatisfaction: {
        target: criteria.userSatisfaction,
        actual: userSatisfaction,
        passed: userSatisfaction >= criteria.userSatisfaction
      },
      overallSuccess: systemStability >= criteria.systemStability &&
                      energySavings >= criteria.energySavings &&
                      userSatisfaction >= criteria.userSatisfaction,
      lessonsLearned: this.extractLessonsLearned(results),
      recommendationsForRollout: this.generateRolloutRecommendations(results)
    };
  }
}
```

### 8.2.2 전체 배포

```typescript
// 롤링 배포 전략
interface RollingDeploymentStrategy {
  approach: 'phased' | 'parallel' | 'big_bang';
  waveSize: number;
  waveDuration: string;
  stabilizationPeriod: string;
  gateConditions: string[];
}

// 배포 오케스트레이터
class DeploymentOrchestrator {
  private buildings: Building[];
  private deploymentPlan: DeploymentPlan;

  async executeRollingDeployment(): Promise<DeploymentResult> {
    const waves = this.createDeploymentWaves();
    const results: WaveResult[] = [];

    for (const wave of waves) {
      console.log(`배포 Wave ${wave.number} 시작: ${wave.buildings.length}개 건물`);

      // Wave 배포
      const waveResult = await this.deployWave(wave);
      results.push(waveResult);

      // 안정화 기간
      await this.stabilizationPeriod(wave);

      // 게이트 체크
      const gateCheck = await this.checkGateConditions(waveResult);

      if (!gateCheck.passed) {
        console.error(`Wave ${wave.number} 게이트 실패. 배포 중단.`);
        return {
          success: false,
          completedWaves: results,
          failedAt: wave.number,
          reason: gateCheck.failureReason
        };
      }

      console.log(`Wave ${wave.number} 완료. 다음 Wave 진행.`);
    }

    return {
      success: true,
      completedWaves: results,
      summary: this.generateDeploymentSummary(results)
    };
  }

  private createDeploymentWaves(): DeploymentWave[] {
    // 건물 우선순위에 따라 Wave 구성
    const sortedBuildings = this.prioritizeBuildings();
    const waves: DeploymentWave[] = [];
    const waveSize = this.deploymentPlan.waveSize;

    for (let i = 0; i < sortedBuildings.length; i += waveSize) {
      waves.push({
        number: waves.length + 1,
        buildings: sortedBuildings.slice(i, i + waveSize),
        scheduledStart: this.calculateWaveStartDate(waves.length)
      });
    }

    return waves;
  }

  private prioritizeBuildings(): Building[] {
    return this.buildings.sort((a, b) => {
      // 우선순위 점수 계산
      const scoreA = this.calculatePriorityScore(a);
      const scoreB = this.calculatePriorityScore(b);
      return scoreB - scoreA; // 높은 점수 우선
    });
  }

  private calculatePriorityScore(building: Building): number {
    let score = 0;

    // 절감 잠재력
    if (building.energyProfile.eui > 200) score += 30;
    else if (building.energyProfile.eui > 150) score += 20;
    else score += 10;

    // 시설 관리 역량
    if (building.facilityManagement.readiness === 'high') score += 25;
    else if (building.facilityManagement.readiness === 'medium') score += 15;
    else score += 5;

    // 기존 인프라
    if (building.basSystem.modern) score += 20;
    else if (building.basSystem.upgradable) score += 10;

    // 복잡성 (낮을수록 높은 점수)
    if (building.complexity === 'low') score += 15;
    else if (building.complexity === 'medium') score += 10;

    return score;
  }

  private async deployWave(wave: DeploymentWave): Promise<WaveResult> {
    const buildingResults: BuildingDeploymentResult[] = [];

    // 병렬 배포 (건물별)
    const deploymentPromises = wave.buildings.map(building =>
      this.deployToBuilding(building)
    );

    const results = await Promise.allSettled(deploymentPromises);

    for (let i = 0; i < results.length; i++) {
      const result = results[i];
      buildingResults.push({
        buildingId: wave.buildings[i].id,
        success: result.status === 'fulfilled' && result.value.success,
        details: result.status === 'fulfilled' ? result.value : result.reason
      });
    }

    return {
      waveNumber: wave.number,
      buildingResults,
      successRate: buildingResults.filter(r => r.success).length / buildingResults.length,
      completedAt: new Date()
    };
  }

  private async checkGateConditions(waveResult: WaveResult): Promise<GateCheckResult> {
    const conditions = this.deploymentPlan.gateConditions;
    const failures: string[] = [];

    // 성공률 체크
    if (waveResult.successRate < 0.9) {
      failures.push(`성공률 미달: ${(waveResult.successRate * 100).toFixed(1)}% < 90%`);
    }

    // 시스템 안정성 체크
    const stability = await this.checkSystemStability(waveResult);
    if (stability < 0.99) {
      failures.push(`시스템 안정성 미달: ${(stability * 100).toFixed(1)}% < 99%`);
    }

    // 성능 체크
    const performance = await this.checkPerformanceMetrics(waveResult);
    if (!performance.met) {
      failures.push(`성능 기준 미달: ${performance.details}`);
    }

    return {
      passed: failures.length === 0,
      failureReason: failures.length > 0 ? failures.join('; ') : undefined
    };
  }
}
```

---

## 8.3 커미셔닝

### 8.3.1 기능 테스트

```typescript
// 커미셔닝 테스트 프레임워크
interface CommissioningFramework {
  testCategories: {
    functional: {
      name: '기능 테스트';
      tests: [
        '데이터 수집 정확성',
        '제어 명령 실행',
        '알람 발생 및 알림',
        '리포팅 기능',
        '사용자 인터페이스'
      ];
    };
    performance: {
      name: '성능 테스트';
      tests: [
        'API 응답 시간',
        '대시보드 로딩 속도',
        '대용량 데이터 쿼리',
        '동시 사용자 처리'
      ];
    };
    integration: {
      name: '연동 테스트';
      tests: [
        'BAS 연동',
        '외부 API 연동',
        '데이터 동기화',
        '인증 연동'
      ];
    };
    control: {
      name: '제어 시퀀스 테스트';
      tests: [
        'AHU 제어 시퀀스',
        'VAV 제어 시퀀스',
        '냉동기 스테이징',
        '수요 응답 시퀀스'
      ];
    };
  };
}

// 커미셔닝 테스트 실행자
class CommissioningTestExecutor {
  private testSuite: CommissioningTestSuite;
  private testResults: TestResult[];

  async executeTestSuite(): Promise<CommissioningReport> {
    this.testResults = [];

    // 1. 기능 테스트
    await this.executeFunctionalTests();

    // 2. 성능 테스트
    await this.executePerformanceTests();

    // 3. 연동 테스트
    await this.executeIntegrationTests();

    // 4. 제어 시퀀스 테스트
    await this.executeControlSequenceTests();

    // 결과 보고서 생성
    return this.generateReport();
  }

  private async executeFunctionalTests(): Promise<void> {
    console.log('기능 테스트 시작...');

    // 데이터 수집 테스트
    await this.testDataCollection();

    // 제어 명령 테스트
    await this.testControlCommands();

    // 알람 테스트
    await this.testAlarmSystem();

    // 리포팅 테스트
    await this.testReporting();
  }

  private async testDataCollection(): Promise<void> {
    const testCases = [
      {
        name: '전력량계 데이터 수집',
        pointType: 'energy_meter',
        expectedInterval: 900, // 15분
        tolerancePercent: 5
      },
      {
        name: '환경 센서 데이터 수집',
        pointType: 'environment_sensor',
        expectedInterval: 300, // 5분
        tolerancePercent: 10
      },
      {
        name: '설비 상태 데이터 수집',
        pointType: 'equipment_status',
        expectedInterval: 60, // 1분
        tolerancePercent: 20
      }
    ];

    for (const testCase of testCases) {
      const result = await this.runDataCollectionTest(testCase);
      this.testResults.push(result);
    }
  }

  private async runDataCollectionTest(
    testCase: DataCollectionTestCase
  ): Promise<TestResult> {
    const startTime = Date.now();

    try {
      // 테스트 포인트 선택
      const testPoints = await this.selectTestPoints(testCase.pointType, 5);

      // 데이터 수집 확인 (10분간)
      const collectedData = await this.monitorDataCollection(
        testPoints,
        10 * 60 * 1000 // 10분
      );

      // 검증
      const validation = this.validateDataCollection(
        collectedData,
        testCase.expectedInterval,
        testCase.tolerancePercent
      );

      return {
        testId: `DC-${testCase.pointType}`,
        testName: testCase.name,
        category: 'functional',
        passed: validation.passed,
        duration: Date.now() - startTime,
        details: validation.details,
        issues: validation.issues
      };
    } catch (error) {
      return {
        testId: `DC-${testCase.pointType}`,
        testName: testCase.name,
        category: 'functional',
        passed: false,
        duration: Date.now() - startTime,
        error: error.message
      };
    }
  }

  private async testControlCommands(): Promise<void> {
    const testCases = [
      {
        name: '설정값 변경',
        commandType: 'setpoint_change',
        testSequence: [
          { action: 'read', point: 'AHU-1/SAT_SETPOINT' },
          { action: 'write', point: 'AHU-1/SAT_SETPOINT', value: 14 },
          { action: 'verify', point: 'AHU-1/SAT_SETPOINT', expectedValue: 14 },
          { action: 'restore' }
        ]
      },
      {
        name: '스케줄 변경',
        commandType: 'schedule_change',
        testSequence: [
          { action: 'read', schedule: 'AHU-1/SCHEDULE' },
          { action: 'modify', schedule: 'AHU-1/SCHEDULE', period: 'test' },
          { action: 'verify', schedule: 'AHU-1/SCHEDULE' },
          { action: 'restore' }
        ]
      }
    ];

    for (const testCase of testCases) {
      const result = await this.runControlCommandTest(testCase);
      this.testResults.push(result);
    }
  }

  private async executeControlSequenceTests(): Promise<void> {
    console.log('제어 시퀀스 테스트 시작...');

    // AHU 제어 시퀀스 테스트
    await this.testAHUControlSequence();

    // 최적 기동/정지 테스트
    await this.testOptimalStartStop();

    // 이코노마이저 테스트
    await this.testEconomizerSequence();
  }

  private async testAHUControlSequence(): Promise<void> {
    const ahuId = 'AHU-1';

    // 테스트 시나리오
    const scenarios = [
      {
        name: '냉방 모드 전환',
        initialConditions: {
          zoneTemp: 26,
          zoneTempSetpoint: 24,
          outsideAirTemp: 30
        },
        expectedBehavior: {
          mode: 'cooling',
          coolingValve: { min: 30, max: 100 },
          heatingValve: { max: 5 }
        }
      },
      {
        name: '외기 냉방 활성화',
        initialConditions: {
          zoneTemp: 25,
          zoneTempSetpoint: 24,
          outsideAirTemp: 18
        },
        expectedBehavior: {
          mode: 'economizer',
          outdoorAirDamper: { min: 50 },
          coolingValve: { max: 20 }
        }
      }
    ];

    for (const scenario of scenarios) {
      const result = await this.runControlSequenceTest(ahuId, scenario);
      this.testResults.push(result);
    }
  }

  private async runControlSequenceTest(
    equipmentId: string,
    scenario: ControlSequenceScenario
  ): Promise<TestResult> {
    const startTime = Date.now();

    try {
      // 1. 초기 조건 설정 (가능한 경우)
      await this.setTestConditions(equipmentId, scenario.initialConditions);

      // 2. 안정화 대기
      await this.wait(60000); // 1분

      // 3. 실제 동작 모니터링
      const actualBehavior = await this.monitorEquipmentBehavior(equipmentId, 300000); // 5분

      // 4. 예상 동작과 비교
      const comparison = this.compareBehavior(
        actualBehavior,
        scenario.expectedBehavior
      );

      // 5. 원래 상태 복원
      await this.restoreConditions(equipmentId);

      return {
        testId: `CS-${equipmentId}-${scenario.name}`,
        testName: `${equipmentId} ${scenario.name}`,
        category: 'control_sequence',
        passed: comparison.passed,
        duration: Date.now() - startTime,
        details: {
          scenario,
          actualBehavior,
          comparison
        }
      };
    } catch (error) {
      await this.restoreConditions(equipmentId);
      return {
        testId: `CS-${equipmentId}-${scenario.name}`,
        testName: `${equipmentId} ${scenario.name}`,
        category: 'control_sequence',
        passed: false,
        duration: Date.now() - startTime,
        error: error.message
      };
    }
  }
}
```

---

## 8.4 운영 및 유지보수

### 8.4.1 운영 절차

```typescript
// 일상 운영 절차
interface DailyOperationsProcedures {
  morningRoutine: {
    time: '08:00';
    tasks: [
      '야간 알람 검토',
      '설비 상태 확인',
      '에너지 사용량 확인',
      '당일 일정 확인',
      '기상 예보 확인'
    ];
  };

  continuousMonitoring: {
    tasks: [
      '실시간 대시보드 모니터링',
      '알람 대응',
      '이상 현상 조사',
      '민원 대응'
    ];
  };

  eveningRoutine: {
    time: '17:00';
    tasks: [
      '일일 에너지 리포트 생성',
      '야간 설정 확인',
      '내일 예정 작업 확인',
      '알람 임계값 검토'
    ];
  };
}

// 운영 대시보드 관리자
class OperationsDashboard {
  async getDailyView(): Promise<DailyOperationsView> {
    return {
      systemHealth: await this.getSystemHealth(),
      energyPerformance: await this.getEnergyPerformance(),
      activeAlarms: await this.getActiveAlarms(),
      scheduledMaintenance: await this.getScheduledMaintenance(),
      keyMetrics: await this.getKeyMetrics()
    };
  }

  private async getSystemHealth(): Promise<SystemHealth> {
    const components = await this.checkAllComponents();

    return {
      overall: this.calculateOverallHealth(components),
      components: components.map(c => ({
        name: c.name,
        status: c.status,
        lastCheck: c.lastCheck,
        issues: c.issues
      })),
      uptime: await this.calculateUptime()
    };
  }

  private async getEnergyPerformance(): Promise<EnergyPerformanceView> {
    const today = await this.getTodayConsumption();
    const baseline = await this.getBaselineConsumption();
    const target = await this.getTargetConsumption();

    return {
      todayConsumption: {
        electricity: today.electricity,
        gas: today.gas
      },
      vsBaseline: {
        electricity: ((today.electricity - baseline.electricity) / baseline.electricity) * 100,
        gas: ((today.gas - baseline.gas) / baseline.gas) * 100
      },
      vsTarget: {
        electricity: ((today.electricity - target.electricity) / target.electricity) * 100,
        gas: ((today.gas - target.gas) / target.gas) * 100
      },
      projectedDaily: await this.projectDailyConsumption(today),
      peakDemand: await this.getTodayPeakDemand()
    };
  }

  async generateDailyReport(): Promise<DailyReport> {
    const reportDate = new Date();
    const performance = await this.getEnergyPerformance();
    const alarms = await this.getAlarmSummary();
    const equipment = await this.getEquipmentSummary();
    const savings = await this.calculateDailySavings();

    return {
      date: reportDate,
      summary: {
        energyConsumption: performance.todayConsumption,
        savingsVsBaseline: savings,
        alarmCount: alarms.total,
        equipmentUptime: equipment.averageUptime
      },
      energyDetail: {
        hourlyConsumption: await this.getHourlyConsumption(reportDate),
        peakDemand: performance.peakDemand,
        loadProfile: await this.getLoadProfile(reportDate)
      },
      alarmDetail: {
        critical: alarms.critical,
        high: alarms.high,
        medium: alarms.medium,
        low: alarms.low,
        topIssues: alarms.topIssues
      },
      equipmentDetail: {
        runHours: equipment.runHours,
        efficiency: equipment.efficiency,
        issues: equipment.issues
      },
      recommendations: await this.generateDailyRecommendations()
    };
  }
}
```

---

## 8.5 장 요약

### 구현 단계 요약

| 단계 | 기간 | 주요 활동 | 산출물 |
|------|------|----------|--------|
| 진단 | 2-4주 | 에너지 진단, 현황 분석 | 진단 보고서 |
| 설계 | 4-8주 | 요구사항 정의, 아키텍처 설계 | 설계 문서 |
| 파일럿 | 3-6개월 | 시범 구축, 검증 | 파일럿 보고서 |
| 배포 | 6-12개월 | 전체 배포, 커미셔닝 | 준공 문서 |
| 운영 | 지속 | 일상 운영, 최적화 | 월간 보고서 |

### 성공 요인

1. **경영진 지원**: 지속적인 관심과 자원 배분
2. **단계적 접근**: 파일럿 검증 후 확대
3. **사용자 참여**: 현장 관리자의 적극적 참여
4. **명확한 KPI**: 측정 가능한 성과 지표
5. **지속적 개선**: 운영 중 최적화 반복

### 다음 장 미리보기

제9장에서는 빌딩 에너지 관리의 미래 트렌드와 신기술에 대해 다룹니다. AI/ML, 디지털 트윈, 자율 빌딩, 그리드 상호작용 빌딩 등을 살펴봅니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
