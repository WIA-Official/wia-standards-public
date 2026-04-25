# 제7장: 시스템 통합

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:

1. WIA 가뭄 모니터링 표준을 기존 농업 관리 시스템과 통합
2. 실시간 관개 제어 시스템과의 양방향 통신 구현
3. 국가 조기 경보 시스템과의 데이터 교환 설계
4. 위성 데이터 파이프라인 자동화 구축
5. 한국 농업 인프라와의 연계 시스템 개발

---

## 7.1 농업 관리 시스템 통합

### 7.1.1 스마트팜 플랫폼 연동

한국의 스마트팜 생태계는 농촌진흥청의 스마트팜 코리아와 민간 플랫폼이 공존합니다. WIA 표준은 이들 시스템과의 원활한 통합을 지원합니다.

```typescript
// 스마트팜 통합 어댑터
interface SmartFarmAdapter {
  // 플랫폼 정보
  platform: {
    name: string;           // "스마트팜코리아", "그린랩스", "팜에이트"
    version: string;
    apiEndpoint: string;
    authMethod: "oauth2" | "apikey" | "certificate";
  };

  // 농장 매핑
  farmMapping: {
    wiaFarmId: string;
    platformFarmId: string;
    location: {
      coordinates: [number, number];
      administrativeArea: string;    // "경기도 이천시"
      landParcelId: string;          // 토지 지번
    };
    crops: CropInfo[];
    sensors: SensorMapping[];
  };

  // 데이터 동기화 설정
  syncConfig: {
    droughtIndexInterval: number;    // 가뭄 지수 동기화 주기 (분)
    weatherDataInterval: number;     // 기상 데이터 동기화 주기 (분)
    soilMoistureInterval: number;    // 토양 수분 동기화 주기 (분)
    bidirectional: boolean;          // 양방향 동기화 여부
  };
}

// 작물 정보
interface CropInfo {
  cropId: string;
  cropType: string;                  // "벼", "배추", "사과"
  plantingDate: string;
  expectedHarvestDate: string;
  growthStage: string;
  droughtSensitivity: "low" | "medium" | "high" | "critical";
  waterRequirement: {
    daily: number;                   // mm/day
    growthStageMultiplier: number;
  };
}

// 센서 매핑
interface SensorMapping {
  wiaSensorId: string;
  platformSensorId: string;
  sensorType: "soil_moisture" | "weather" | "ndvi" | "flow_meter";
  calibrationOffset: number;
  lastCalibration: string;
}
```

### 7.1.2 통합 데이터 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│                    스마트팜 통합 아키텍처                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐         │
│  │   기상청      │    │  천리안위성   │    │  농진청 AWS   │         │
│  │  API 서버    │    │  수신센터     │    │   데이터      │         │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘         │
│         │                   │                   │                  │
│         └───────────────────┼───────────────────┘                  │
│                             ▼                                      │
│              ┌─────────────────────────────┐                       │
│              │    WIA 가뭄 모니터링 허브    │                       │
│              │  ┌─────────────────────────┐│                       │
│              │  │ • 데이터 정규화          ││                       │
│              │  │ • 가뭄 지수 계산         ││                       │
│              │  │ • 예측 모델 실행         ││                       │
│              │  │ • 경보 생성             ││                       │
│              │  └─────────────────────────┘│                       │
│              └──────────────┬──────────────┘                       │
│                             │                                      │
│         ┌───────────────────┼───────────────────┐                  │
│         ▼                   ▼                   ▼                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐         │
│  │  스마트팜     │    │   관개       │    │  조기경보     │         │
│  │  플랫폼      │    │  제어시스템   │    │   시스템      │         │
│  └──────────────┘    └──────────────┘    └──────────────┘         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.1.3 농업 데이터 교환 프로토콜

```typescript
// 농업 시스템 데이터 교환 클래스
class AgriculturalDataExchange {
  private adapter: SmartFarmAdapter;
  private syncQueue: SyncTask[];
  private conflictResolver: ConflictResolver;

  constructor(config: ExchangeConfig) {
    this.adapter = this.initializeAdapter(config);
    this.syncQueue = [];
    this.conflictResolver = new ConflictResolver(config.conflictStrategy);
  }

  // 가뭄 데이터를 스마트팜 플랫폼으로 푸시
  async pushDroughtData(data: DroughtIndexData): Promise<SyncResult> {
    const transformed = this.transformToplatformFormat(data);

    const payload = {
      messageId: generateUUID(),
      timestamp: new Date().toISOString(),
      source: "wia-drought-monitor",
      destination: this.adapter.platform.name,
      dataType: "drought_index",
      payload: transformed,
      priority: this.calculatePriority(data),
      ttl: 3600  // 1시간 유효
    };

    try {
      const response = await this.sendWithRetry(payload);
      await this.logSync(payload, response);
      return { success: true, messageId: payload.messageId };
    } catch (error) {
      await this.handleSyncError(payload, error);
      return { success: false, error: error.message };
    }
  }

  // 스마트팜 센서 데이터 수신
  async receiveSensorData(): Promise<SensorDataBatch> {
    const endpoint = `${this.adapter.platform.apiEndpoint}/sensors/data`;

    const params = {
      farmIds: this.adapter.farmMapping.wiaFarmId,
      sensorTypes: ["soil_moisture", "weather"],
      since: this.getLastSyncTimestamp(),
      format: "wia-compatible"
    };

    const response = await this.fetchWithAuth(endpoint, params);

    return {
      data: response.sensors.map(s => this.mapToWiaFormat(s)),
      syncTimestamp: new Date().toISOString(),
      recordCount: response.sensors.length
    };
  }

  // 우선순위 계산
  private calculatePriority(data: DroughtIndexData): number {
    if (data.severity === "extreme") return 1;
    if (data.severity === "severe") return 2;
    if (data.severity === "moderate") return 3;
    return 4;
  }

  // 플랫폼 형식으로 변환
  private transformToplatformFormat(data: DroughtIndexData): any {
    // 스마트팜 코리아 형식
    if (this.adapter.platform.name === "스마트팜코리아") {
      return {
        farm_cd: this.adapter.farmMapping.platformFarmId,
        drought_level: this.mapSeverityToLevel(data.severity),
        drought_idx: data.value,
        idx_type: data.indexType,
        valid_dt: data.validTime,
        forecast_yn: data.isForecast ? "Y" : "N"
      };
    }

    // 그린랩스 형식
    if (this.adapter.platform.name === "그린랩스") {
      return {
        farmId: this.adapter.farmMapping.platformFarmId,
        droughtInfo: {
          level: data.severity,
          index: data.value,
          type: data.indexType
        },
        timestamp: data.validTime
      };
    }

    // 기본 WIA 형식
    return data;
  }

  private mapSeverityToLevel(severity: string): number {
    const mapping = {
      "normal": 0,
      "abnormally_dry": 1,
      "moderate": 2,
      "severe": 3,
      "extreme": 4,
      "exceptional": 5
    };
    return mapping[severity] || 0;
  }
}
```

---

## 7.2 관개 제어 시스템 연동

### 7.2.1 자동 관개 시스템 아키텍처

```typescript
// 관개 제어 통합 인터페이스
interface IrrigationControlSystem {
  // 시스템 식별
  systemId: string;
  systemType: "drip" | "sprinkler" | "flood" | "center_pivot";
  manufacturer: string;
  controllerModel: string;

  // 연결 설정
  connection: {
    protocol: "modbus" | "mqtt" | "http" | "proprietary";
    endpoint: string;
    authentication: AuthConfig;
    timeout: number;
  };

  // 제어 영역
  zones: IrrigationZone[];

  // 실시간 상태
  status: SystemStatus;
}

interface IrrigationZone {
  zoneId: string;
  zoneName: string;
  area: number;                      // 면적 (㎡)
  crops: string[];
  soilType: string;

  // 관개 설비
  equipment: {
    valveIds: string[];
    pumpId: string;
    flowMeterId: string;
    pressureSensorId: string;
  };

  // 관개 매개변수
  parameters: {
    maxFlowRate: number;             // L/min
    minPressure: number;             // kPa
    uniformityCoefficient: number;   // 0-1
    applicationRate: number;         // mm/hour
  };

  // 토양 수분 목표
  moistureTargets: {
    fieldCapacity: number;           // 포장용수량 (%)
    permanentWiltingPoint: number;   // 영구위조점 (%)
    managementAllowedDepletion: number;  // 허용 고갈량 (%)
    triggerThreshold: number;        // 관개 시작 임계값 (%)
    cutoffThreshold: number;         // 관개 중단 임계값 (%)
  };
}

// 관개 제어 클래스
class DroughtBasedIrrigationController {
  private system: IrrigationControlSystem;
  private droughtMonitor: DroughtMonitorClient;
  private weatherForecast: WeatherForecastClient;
  private decisionEngine: IrrigationDecisionEngine;

  constructor(config: ControllerConfig) {
    this.system = config.system;
    this.droughtMonitor = new DroughtMonitorClient(config.droughtApi);
    this.weatherForecast = new WeatherForecastClient(config.weatherApi);
    this.decisionEngine = new IrrigationDecisionEngine(config.decisionRules);
  }

  // 관개 의사결정 수행
  async makeIrrigationDecision(zoneId: string): Promise<IrrigationDecision> {
    const zone = this.system.zones.find(z => z.zoneId === zoneId);
    if (!zone) throw new Error(`Zone ${zoneId} not found`);

    // 데이터 수집
    const [droughtData, weatherData, soilData] = await Promise.all([
      this.droughtMonitor.getCurrentIndices(zone),
      this.weatherForecast.get7DayForecast(zone),
      this.getSoilMoistureData(zone)
    ]);

    // 의사결정 입력 구성
    const decisionInput: DecisionInput = {
      zone,
      droughtIndices: {
        pdsi: droughtData.pdsi,
        spi: droughtData.spi,
        soilMoisturePercentile: droughtData.soilMoisturePercentile
      },
      currentSoilMoisture: soilData.current,
      soilMoistureTrend: soilData.trend,
      weatherForecast: {
        precipitationProbability: weatherData.precipProb,
        expectedPrecipitation: weatherData.expectedPrecip,
        maxTemperature: weatherData.maxTemp,
        evapotranspirationForecast: weatherData.etForecast
      },
      cropWaterRequirement: this.calculateCropWaterNeed(zone),
      restrictions: await this.getWaterRestrictions(zone)
    };

    // 의사결정 실행
    const decision = this.decisionEngine.decide(decisionInput);

    // 결정 로깅
    await this.logDecision(zone, decisionInput, decision);

    return decision;
  }

  // 관개 실행
  async executeIrrigation(decision: IrrigationDecision): Promise<ExecutionResult> {
    if (!decision.shouldIrrigate) {
      return { executed: false, reason: decision.reason };
    }

    const zone = this.system.zones.find(z => z.zoneId === decision.zoneId);

    // 사전 점검
    const preCheck = await this.performPreCheck(zone);
    if (!preCheck.passed) {
      return { executed: false, reason: preCheck.failureReason };
    }

    // 관개 시작
    const schedule = this.createIrrigationSchedule(zone, decision);

    try {
      // 펌프 시작
      await this.startPump(zone.equipment.pumpId);

      // 밸브 개방
      for (const valveId of zone.equipment.valveIds) {
        await this.openValve(valveId);
      }

      // 모니터링 시작
      const monitor = this.startIrrigationMonitor(zone, schedule);

      return {
        executed: true,
        schedule,
        monitorId: monitor.id,
        estimatedCompletion: schedule.endTime
      };
    } catch (error) {
      // 안전 정지
      await this.emergencyStop(zone);
      throw error;
    }
  }

  // 작물 물 필요량 계산
  private calculateCropWaterNeed(zone: IrrigationZone): number {
    // ETc = ET0 × Kc (작물 계수)
    const et0 = this.weatherForecast.getET0();
    const kc = this.getCropCoefficient(zone.crops[0]);
    const etc = et0 * kc;

    // 토양 수분 부족량
    const currentMoisture = zone.moistureTargets.triggerThreshold;
    const targetMoisture = zone.moistureTargets.fieldCapacity;
    const deficitPercent = targetMoisture - currentMoisture;

    // 면적당 필요량 (mm)
    const rootDepth = this.getRootDepth(zone.crops[0]);
    const soilWaterHoldingCapacity = this.getSoilWHC(zone.soilType);

    return (deficitPercent / 100) * rootDepth * soilWaterHoldingCapacity + etc;
  }
}
```

### 7.2.2 관개 의사결정 엔진

```typescript
// 관개 의사결정 규칙 엔진
class IrrigationDecisionEngine {
  private rules: DecisionRule[];
  private mlModel: IrrigationMLModel;

  constructor(ruleConfig: RuleConfig) {
    this.rules = this.loadRules(ruleConfig);
    this.mlModel = new IrrigationMLModel(ruleConfig.modelPath);
  }

  decide(input: DecisionInput): IrrigationDecision {
    // 규칙 기반 평가
    const ruleResults = this.evaluateRules(input);

    // ML 모델 예측
    const mlPrediction = this.mlModel.predict(input);

    // 최종 결정 (규칙 + ML 앙상블)
    const finalDecision = this.ensemble(ruleResults, mlPrediction, input);

    return finalDecision;
  }

  private evaluateRules(input: DecisionInput): RuleResult[] {
    const results: RuleResult[] = [];

    // 규칙 1: 심각한 가뭄 시 즉시 관개
    if (input.droughtIndices.pdsi < -3.0) {
      results.push({
        ruleId: "SEVERE_DROUGHT",
        action: "irrigate",
        priority: 1,
        amount: this.calculateEmergencyAmount(input),
        reason: "심각한 가뭄 상태 (PDSI < -3.0)"
      });
    }

    // 규칙 2: 토양 수분 임계값 미달
    if (input.currentSoilMoisture < input.zone.moistureTargets.triggerThreshold) {
      results.push({
        ruleId: "SOIL_MOISTURE_LOW",
        action: "irrigate",
        priority: 2,
        amount: this.calculateDeficitAmount(input),
        reason: `토양 수분 ${input.currentSoilMoisture}% < 임계값 ${input.zone.moistureTargets.triggerThreshold}%`
      });
    }

    // 규칙 3: 강우 예보 시 관개 연기
    if (input.weatherForecast.precipitationProbability > 0.7 &&
        input.weatherForecast.expectedPrecipitation > 10) {
      results.push({
        ruleId: "RAIN_EXPECTED",
        action: "delay",
        priority: 3,
        delayHours: 24,
        reason: `강우 예보 확률 ${input.weatherForecast.precipitationProbability * 100}%, 예상 강수량 ${input.weatherForecast.expectedPrecipitation}mm`
      });
    }

    // 규칙 4: 용수 제한 시 감량 관개
    if (input.restrictions?.active) {
      results.push({
        ruleId: "WATER_RESTRICTION",
        action: "reduce",
        reductionPercent: input.restrictions.reductionPercent,
        reason: `용수 제한 중: ${input.restrictions.reason}`
      });
    }

    // 규칙 5: 작물 생육 단계별 조절
    const growthStage = this.determineGrowthStage(input.zone.crops[0]);
    if (growthStage === "flowering") {
      results.push({
        ruleId: "CRITICAL_GROWTH_STAGE",
        action: "prioritize",
        priority: 1,
        reason: "개화기 - 수분 스트레스 민감 시기"
      });
    }

    return results;
  }

  private ensemble(
    ruleResults: RuleResult[],
    mlPrediction: MLPrediction,
    input: DecisionInput
  ): IrrigationDecision {
    // 최고 우선순위 규칙 확인
    const highPriorityRules = ruleResults.filter(r => r.priority === 1);

    // 긴급 관개 필요 시 규칙 우선
    if (highPriorityRules.some(r => r.action === "irrigate")) {
      const irrigateRule = highPriorityRules.find(r => r.action === "irrigate");
      return {
        shouldIrrigate: true,
        zoneId: input.zone.zoneId,
        amount: irrigateRule.amount,
        timing: "immediate",
        reason: irrigateRule.reason,
        confidence: 0.95,
        source: "rule_based"
      };
    }

    // 강우 연기 규칙 확인
    const delayRule = ruleResults.find(r => r.action === "delay");
    if (delayRule && mlPrediction.irrigateProb < 0.7) {
      return {
        shouldIrrigate: false,
        zoneId: input.zone.zoneId,
        amount: 0,
        timing: "delayed",
        nextEvaluation: this.addHours(new Date(), delayRule.delayHours),
        reason: delayRule.reason,
        confidence: 0.8,
        source: "rule_based"
      };
    }

    // ML 예측 활용
    if (mlPrediction.irrigateProb > 0.6) {
      let amount = mlPrediction.recommendedAmount;

      // 용수 제한 적용
      const restrictionRule = ruleResults.find(r => r.action === "reduce");
      if (restrictionRule) {
        amount *= (1 - restrictionRule.reductionPercent / 100);
      }

      return {
        shouldIrrigate: true,
        zoneId: input.zone.zoneId,
        amount,
        timing: mlPrediction.optimalTiming,
        reason: `ML 예측 (확률: ${(mlPrediction.irrigateProb * 100).toFixed(1)}%)`,
        confidence: mlPrediction.irrigateProb,
        source: "ml_model"
      };
    }

    // 관개 불필요
    return {
      shouldIrrigate: false,
      zoneId: input.zone.zoneId,
      amount: 0,
      timing: "none",
      reason: "현재 관개 불필요",
      confidence: 1 - mlPrediction.irrigateProb,
      source: "ensemble"
    };
  }
}
```

### 7.2.3 관개 모니터링 대시보드 데이터

| 모니터링 항목 | 측정 주기 | 임계값 | 알림 조건 |
|-------------|---------|-------|---------|
| 토양 수분 | 15분 | 20-80% | <20% 또는 >80% |
| 유량 | 실시간 | 설계값 ±10% | 편차 >15% |
| 압력 | 실시간 | 200-400 kPa | <150 또는 >450 kPa |
| 누적 관개량 | 관개 종료 시 | 목표량 ±5% | 편차 >10% |
| 균등도 계수 | 관개 종료 시 | >0.85 | <0.80 |
| 에너지 효율 | 일간 | >0.7 kWh/m³ | <0.5 kWh/m³ |

---

## 7.3 조기 경보 시스템 연계

### 7.3.1 국가 재난 경보 시스템 통합

```typescript
// 국가 재난 경보 시스템 인터페이스
interface NationalAlertSystemInterface {
  // 한국 재난안전통신망 (SAFETYNET)
  safetyNet: {
    endpoint: string;
    certificatePath: string;
    agencyCode: string;         // 발령 기관 코드
    alertTypes: AlertType[];
  };

  // 긴급재난문자 (CBS)
  cbsGateway: {
    endpoint: string;
    operatorKey: string;
    maxMessageLength: number;   // 90자 (한글 기준)
    areaCode: string[];
  };

  // 농업재해조기경보시스템
  agrimetAlert: {
    endpoint: string;
    apiKey: string;
    regions: RegionConfig[];
  };
}

// 가뭄 경보 발령 서비스
class DroughtAlertService {
  private nationalAlert: NationalAlertSystemInterface;
  private alertHistory: AlertHistory;
  private escalationPolicy: EscalationPolicy;

  constructor(config: AlertServiceConfig) {
    this.nationalAlert = config.nationalAlert;
    this.alertHistory = new AlertHistory();
    this.escalationPolicy = new EscalationPolicy(config.escalation);
  }

  // 가뭄 경보 발령
  async issueAlert(droughtEvent: DroughtEvent): Promise<AlertResult> {
    // 경보 등급 결정
    const alertLevel = this.determineAlertLevel(droughtEvent);

    // 영향 지역 산정
    const affectedAreas = this.calculateAffectedAreas(droughtEvent);

    // 경보 메시지 생성
    const alertMessage = this.generateAlertMessage(droughtEvent, alertLevel);

    // 중복 경보 확인
    if (this.alertHistory.isDuplicate(droughtEvent, alertLevel)) {
      return { issued: false, reason: "중복 경보" };
    }

    // 경보 채널별 발송
    const results: ChannelResult[] = [];

    // 농업재해조기경보 (항상 발송)
    results.push(await this.sendToAgrimet(alertMessage, affectedAreas));

    // 심각/극심 단계시 긴급재난문자 발송
    if (alertLevel >= AlertLevel.SEVERE) {
      results.push(await this.sendToCBS(alertMessage, affectedAreas));
    }

    // 극심 단계시 SAFETYNET 발송
    if (alertLevel === AlertLevel.EXCEPTIONAL) {
      results.push(await this.sendToSafetyNet(alertMessage, affectedAreas));
    }

    // 이력 저장
    await this.alertHistory.record({
      event: droughtEvent,
      level: alertLevel,
      message: alertMessage,
      channels: results,
      timestamp: new Date()
    });

    return {
      issued: true,
      alertId: generateAlertId(),
      level: alertLevel,
      channels: results
    };
  }

  // 경보 등급 결정
  private determineAlertLevel(event: DroughtEvent): AlertLevel {
    // 복합 지수 기반 등급 결정
    const compositeScore = this.calculateCompositeScore(event);

    // 등급 테이블
    // | 복합 점수 | 등급 | 한국어 명칭 |
    // |----------|------|-----------|
    // | 0-20     | 관심 | D0        |
    // | 21-40    | 주의 | D1        |
    // | 41-60    | 경계 | D2        |
    // | 61-80    | 심각 | D3        |
    // | 81-100   | 극심 | D4        |

    if (compositeScore >= 81) return AlertLevel.EXCEPTIONAL;
    if (compositeScore >= 61) return AlertLevel.SEVERE;
    if (compositeScore >= 41) return AlertLevel.MODERATE;
    if (compositeScore >= 21) return AlertLevel.ABNORMAL;
    return AlertLevel.WATCH;
  }

  // 긴급재난문자 발송
  private async sendToCBS(message: AlertMessage, areas: Area[]): Promise<ChannelResult> {
    // 90자 제한 메시지 생성
    const shortMessage = this.truncateToLimit(
      `[가뭄경보] ${message.region} ${message.levelName}. ${message.impact}. 상세: ${message.infoUrl}`,
      90
    );

    const cbsPayload = {
      messageType: "DISASTER",
      category: "DROUGHT",
      severity: message.level,
      areaCodes: areas.map(a => a.cbsAreaCode),
      message: shortMessage,
      validUntil: this.addHours(new Date(), 24),
      language: "ko"
    };

    try {
      const response = await fetch(this.nationalAlert.cbsGateway.endpoint, {
        method: "POST",
        headers: {
          "Authorization": `Bearer ${this.nationalAlert.cbsGateway.operatorKey}`,
          "Content-Type": "application/json"
        },
        body: JSON.stringify(cbsPayload)
      });

      if (response.ok) {
        return { channel: "CBS", success: true, messageId: (await response.json()).messageId };
      } else {
        return { channel: "CBS", success: false, error: response.statusText };
      }
    } catch (error) {
      return { channel: "CBS", success: false, error: error.message };
    }
  }

  // 농업재해조기경보 발송
  private async sendToAgrimet(message: AlertMessage, areas: Area[]): Promise<ChannelResult> {
    const agrimetPayload = {
      alertType: "DROUGHT",
      alertLevel: message.level,
      regions: areas.map(a => ({
        code: a.administrativeCode,
        name: a.name
      })),
      indices: message.droughtIndices,
      impacts: {
        agriculture: message.agriculturalImpact,
        water: message.waterSupplyImpact
      },
      recommendations: message.recommendations,
      validFrom: new Date(),
      validTo: this.addDays(new Date(), 7),
      nextUpdate: this.addDays(new Date(), 1)
    };

    try {
      const response = await fetch(this.nationalAlert.agrimetAlert.endpoint, {
        method: "POST",
        headers: {
          "X-API-Key": this.nationalAlert.agrimetAlert.apiKey,
          "Content-Type": "application/json"
        },
        body: JSON.stringify(agrimetPayload)
      });

      return { channel: "AGRIMET", success: response.ok };
    } catch (error) {
      return { channel: "AGRIMET", success: false, error: error.message };
    }
  }
}
```

### 7.3.2 경보 에스컬레이션 정책

| 등급 | 발령 조건 | 발령 채널 | 해제 조건 |
|-----|---------|---------|---------|
| D0 관심 | SPI -0.5 ~ -0.8 | 농업재해조기경보 | SPI > -0.5 (7일 연속) |
| D1 주의 | SPI -0.8 ~ -1.3 | 농업재해조기경보 | SPI > -0.8 (7일 연속) |
| D2 경계 | SPI -1.3 ~ -1.6, PDSI < -2 | 농업재해조기경보, 재난문자 | SPI > -1.3 (14일 연속) |
| D3 심각 | SPI -1.6 ~ -2.0, PDSI < -3 | 전체 채널 | SPI > -1.6 (21일 연속) |
| D4 극심 | SPI < -2.0, PDSI < -4 | 전체 채널 + SAFETYNET | SPI > -2.0 (30일 연속) |

### 7.3.3 경보 메시지 템플릿

```typescript
// 경보 메시지 템플릿
const alertTemplates = {
  ko: {
    title: "[가뭄{level}] {region} {levelName} 발령",
    body: `
{region}에 가뭄 {levelName}가 발령되었습니다.

■ 발령 일시: {issueTime}
■ 가뭄 지수
  - 표준강수지수(SPI): {spi}
  - 팔머가뭄심도지수(PDSI): {pdsi}
  - 토양수분: {soilMoisture}%

■ 예상 영향
  - 농업: {agriculturalImpact}
  - 용수 공급: {waterImpact}

■ 행동 요령
{recommendations}

■ 상세 정보: {infoUrl}
■ 문의: {contactInfo}
    `,
    sms: "[가뭄{levelName}] {region}. {shortImpact}. {infoUrl}"
  },

  en: {
    title: "[Drought {level}] {region} {levelName} Alert",
    body: `
Drought {levelName} alert has been issued for {region}.

■ Issue Time: {issueTime}
■ Drought Indices
  - SPI: {spi}
  - PDSI: {pdsi}
  - Soil Moisture: {soilMoisture}%

■ Expected Impacts
  - Agriculture: {agriculturalImpact}
  - Water Supply: {waterImpact}

■ Recommendations
{recommendations}

■ More Info: {infoUrl}
■ Contact: {contactInfo}
    `,
    sms: "[Drought {levelName}] {region}. {shortImpact}. {infoUrl}"
  }
};

// 권고 사항 생성
function generateRecommendations(level: AlertLevel, sector: string): string[] {
  const recommendations = {
    agriculture: {
      [AlertLevel.WATCH]: [
        "토양 수분 모니터링 강화",
        "관개용수 점검",
        "멀칭 등 수분 보존 조치 검토"
      ],
      [AlertLevel.ABNORMAL]: [
        "관개 시간 및 방법 조절 (아침/저녁 관개)",
        "작물별 물 요구량 확인",
        "비료 사용량 감소 검토"
      ],
      [AlertLevel.MODERATE]: [
        "우선순위 작물에 관개 집중",
        "신규 식재 연기",
        "잡초 제거로 수분 경쟁 감소"
      ],
      [AlertLevel.SEVERE]: [
        "긴급 관개 실시",
        "수확 가능 작물 조기 수확 검토",
        "가축 사료 및 용수 확보"
      ],
      [AlertLevel.EXCEPTIONAL]: [
        "농작물 피해 신고 및 지원 신청",
        "긴급 용수 지원 요청",
        "내년 영농 계획 조정"
      ]
    }
  };

  return recommendations[sector]?.[level] || [];
}
```

---

## 7.4 위성 데이터 파이프라인

### 7.4.1 다중 위성 데이터 통합

```typescript
// 위성 데이터 파이프라인 설정
interface SatelliteDataPipeline {
  // 위성 소스 설정
  sources: SatelliteSource[];

  // 처리 단계
  processingSteps: ProcessingStep[];

  // 출력 설정
  output: OutputConfig;
}

interface SatelliteSource {
  name: string;
  provider: string;
  resolution: {
    spatial: number;           // 미터
    temporal: string;          // "daily", "8-day", etc.
  };
  products: SatelliteProduct[];
  accessMethod: "api" | "ftp" | "s3" | "gcs";
  credentials: CredentialConfig;
}

// 위성 데이터 수집기
class SatelliteDataCollector {
  private sources: Map<string, SatelliteSource>;
  private downloadManager: DownloadManager;
  private storageBackend: CloudStorage;

  constructor(config: CollectorConfig) {
    this.sources = new Map();
    this.initializeSources(config.sources);
    this.downloadManager = new DownloadManager(config.download);
    this.storageBackend = new CloudStorage(config.storage);
  }

  // 한국 지역 위성 데이터 수집
  async collectKoreaData(date: Date): Promise<CollectionResult> {
    const tasks: CollectionTask[] = [];

    // 천리안 2A (GK-2A) - 한반도 고해상도
    tasks.push({
      source: "GK-2A",
      products: ["AMI_L2_NDVI", "AMI_L2_LST", "AMI_L2_TPW"],
      bbox: KOREA_BBOX,
      date,
      priority: 1
    });

    // Sentinel-2 - 10m 해상도 광학
    tasks.push({
      source: "SENTINEL-2",
      products: ["S2MSI2A"],
      bbox: KOREA_BBOX,
      date,
      maxCloudCover: 30,
      priority: 2
    });

    // Landsat 8/9 - 30m 해상도 열적외
    tasks.push({
      source: "LANDSAT-8",
      products: ["LC08_L2SP", "LC09_L2SP"],
      bbox: KOREA_BBOX,
      date,
      maxCloudCover: 30,
      priority: 2
    });

    // SMAP - 토양 수분
    tasks.push({
      source: "SMAP",
      products: ["SPL3SMP_E", "SPL4SMGP"],
      bbox: KOREA_BBOX,
      date,
      priority: 1
    });

    // MODIS - 8일 합성
    if (this.is8DayCompositeDate(date)) {
      tasks.push({
        source: "MODIS",
        products: ["MOD13A2", "MYD13A2", "MOD16A2"],
        bbox: KOREA_BBOX,
        date,
        priority: 3
      });
    }

    // 병렬 수집 실행
    const results = await Promise.allSettled(
      tasks.map(task => this.collectTask(task))
    );

    return this.summarizeResults(results);
  }

  // 개별 수집 작업
  private async collectTask(task: CollectionTask): Promise<TaskResult> {
    const source = this.sources.get(task.source);
    if (!source) throw new Error(`Unknown source: ${task.source}`);

    // 데이터 검색
    const catalog = await this.searchCatalog(source, task);

    if (catalog.items.length === 0) {
      return { source: task.source, status: "no_data", items: [] };
    }

    // 다운로드
    const downloads = await this.downloadManager.downloadBatch(
      catalog.items,
      {
        destination: this.getDestinationPath(task),
        concurrent: 4,
        retries: 3
      }
    );

    return {
      source: task.source,
      status: "success",
      items: downloads.map(d => ({
        filename: d.filename,
        size: d.size,
        checksum: d.checksum
      }))
    };
  }
}

// 위성 데이터 처리 파이프라인
class SatelliteProcessingPipeline {
  private steps: Map<string, ProcessingStep>;
  private executor: PipelineExecutor;

  constructor(config: PipelineConfig) {
    this.steps = this.initializeSteps(config.steps);
    this.executor = new PipelineExecutor(config.executor);
  }

  // NDVI 처리 파이프라인 실행
  async processNDVI(input: SatelliteScene): Promise<NDVIProduct> {
    const pipeline = [
      // 1. 대기 보정
      {
        step: "atmospheric_correction",
        processor: "sen2cor",
        params: { resolution: 10 }
      },

      // 2. 구름 마스킹
      {
        step: "cloud_masking",
        processor: "fmask",
        params: {
          cloudProbThreshold: 0.3,
          shadowProbThreshold: 0.3
        }
      },

      // 3. NDVI 계산
      {
        step: "ndvi_calculation",
        processor: "band_math",
        params: {
          formula: "(NIR - RED) / (NIR + RED)",
          nirBand: "B8",
          redBand: "B4"
        }
      },

      // 4. 품질 플래그 생성
      {
        step: "quality_flags",
        processor: "qa_generator",
        params: {
          includeCloudMask: true,
          includeSnowMask: true,
          includeWaterMask: true
        }
      },

      // 5. 리샘플링 및 투영 변환
      {
        step: "reproject",
        processor: "gdalwarp",
        params: {
          targetCrs: "EPSG:5179",  // Korea 2000
          resolution: 10,
          resampling: "bilinear"
        }
      },

      // 6. 타일링
      {
        step: "tiling",
        processor: "gdal_retile",
        params: {
          tileSize: 256,
          format: "COG"
        }
      }
    ];

    return await this.executor.run(pipeline, input);
  }

  // 토양 수분 융합 처리
  async processSoilMoistureFusion(
    smapData: SMAPProduct,
    sentinel1Data: Sentinel1Product
  ): Promise<FusedSoilMoistureProduct> {
    // SMAP 36km -> Sentinel-1 20m 다운스케일링

    const pipeline = [
      // 1. SMAP 데이터 전처리
      {
        step: "smap_preprocess",
        processor: "smap_handler",
        params: {
          qualityFilter: true,
          fillValue: -9999
        }
      },

      // 2. Sentinel-1 백스캐터 계산
      {
        step: "s1_backscatter",
        processor: "snap_toolbox",
        params: {
          polarization: ["VV", "VH"],
          terrainCorrection: true,
          speckleFilter: "refined_lee"
        }
      },

      // 3. 상관관계 분석
      {
        step: "correlation_analysis",
        processor: "sklearn",
        params: {
          method: "random_forest",
          features: ["VV", "VH", "VV/VH", "elevation", "slope", "landcover"]
        }
      },

      // 4. 다운스케일링 적용
      {
        step: "downscaling",
        processor: "ml_downscale",
        params: {
          targetResolution: 100,
          modelType: "random_forest"
        }
      },

      // 5. 검증 및 불확실성 산정
      {
        step: "validation",
        processor: "validator",
        params: {
          referenceStations: KOREA_SOIL_MOISTURE_STATIONS,
          metrics: ["rmse", "correlation", "bias"]
        }
      }
    ];

    return await this.executor.runFusion(pipeline, smapData, sentinel1Data);
  }
}
```

### 7.4.2 위성 데이터 처리 성능 지표

| 처리 단계 | 입력 크기 | 처리 시간 | 출력 크기 | GPU 가속 |
|----------|---------|---------|---------|---------|
| 대기 보정 (Sen2Cor) | 1 GB | 15분 | 1.2 GB | 지원 |
| 구름 마스킹 (Fmask) | 1.2 GB | 8분 | 100 MB | 지원 |
| NDVI 계산 | 200 MB | 30초 | 100 MB | 지원 |
| 토양수분 다운스케일링 | 50 MB + 500 MB | 45분 | 200 MB | 지원 |
| COG 타일링 | 100 MB | 2분 | 120 MB | 미지원 |

---

## 7.5 한국 농업 인프라 연계

### 7.5.1 농촌진흥청 시스템 연동

```typescript
// 농촌진흥청 시스템 통합
interface RDASystemIntegration {
  // 농업기상정보서비스
  agrimetService: {
    endpoint: "https://weather.rda.go.kr/api/v1";
    products: [
      "hourly_weather",     // 시간별 기상
      "daily_summary",      // 일별 요약
      "crop_weather",       // 작물별 기상
      "drought_index",      // 가뭄 지수
      "evapotranspiration"  // 증발산량
    ];
  };

  // 흙토람 (토양정보시스템)
  soilInfoSystem: {
    endpoint: "https://soil.rda.go.kr/api/v1";
    products: [
      "soil_map",           // 토양도
      "soil_properties",    // 토양 특성
      "fertility_map",      // 비옥도 지도
      "drainage_class"      // 배수 등급
    ];
  };

  // 농사로 (영농정보)
  nongsaroService: {
    endpoint: "https://api.nongsaro.go.kr/service";
    products: [
      "crop_calendar",      // 작물력
      "pest_alert",         // 병해충 예찰
      "farming_guide"       // 영농 지침
    ];
  };
}

// RDA 데이터 수집 클라이언트
class RDADataClient {
  private config: RDASystemIntegration;
  private cache: LRUCache;

  constructor(apiKey: string) {
    this.config = this.loadConfig();
    this.cache = new LRUCache({ maxSize: 1000, ttl: 3600000 });
  }

  // 농업기상 데이터 조회
  async getAgrimetData(params: AgrimetParams): Promise<AgrimetData> {
    const cacheKey = `agrimet:${params.stationId}:${params.date}`;

    if (this.cache.has(cacheKey)) {
      return this.cache.get(cacheKey);
    }

    const response = await this.fetchWithRetry(
      `${this.config.agrimetService.endpoint}/weather`,
      {
        stnId: params.stationId,
        date: params.date,
        dataType: params.dataType
      }
    );

    const data = this.parseAgrimetResponse(response);
    this.cache.set(cacheKey, data);

    return data;
  }

  // 토양 정보 조회
  async getSoilInfo(location: Coordinates): Promise<SoilInfo> {
    const response = await this.fetchWithRetry(
      `${this.config.soilInfoSystem.endpoint}/properties`,
      {
        lat: location.lat,
        lng: location.lng,
        detail: "full"
      }
    );

    return {
      soilType: response.soilType,
      textureClass: response.textureClass,
      organicMatter: response.organicMatter,
      fieldCapacity: response.fieldCapacity,
      wiltingPoint: response.wiltingPoint,
      bulkDensity: response.bulkDensity,
      infiltrationRate: response.infiltrationRate,
      drainageClass: response.drainageClass,
      availableWaterCapacity: response.awc
    };
  }

  // 작물 가뭄 취약성 정보
  async getCropDroughtVulnerability(cropCode: string): Promise<DroughtVulnerability> {
    const response = await this.fetchWithRetry(
      `${this.config.nongsaroService.endpoint}/cropInfo`,
      {
        cropCode,
        infoType: "drought_vulnerability"
      }
    );

    return {
      cropName: response.cropName,
      growthStages: response.stages.map(stage => ({
        stageName: stage.name,
        duration: stage.days,
        waterRequirement: stage.waterNeed,     // mm/day
        droughtSensitivity: stage.sensitivity, // 1-5
        criticalMoistureLevel: stage.criticalSM,
        recoveryPotential: stage.recovery
      })),
      totalWaterRequirement: response.totalWater,
      rootDepth: response.rootDepth,
      kcCoefficients: response.kc
    };
  }
}
```

### 7.5.2 한국수자원공사 연동

```typescript
// K-water 시스템 통합
interface KWaterIntegration {
  // 댐 저수량 정보
  damStorage: {
    endpoint: "https://api.kwater.or.kr/dam/v1";
    dams: string[];  // 소양강댐, 충주댐, 안동댐 등
  };

  // 하천 유량 정보
  riverFlow: {
    endpoint: "https://api.kwater.or.kr/river/v1";
    stations: string[];
  };

  // 지하수 수위
  groundwater: {
    endpoint: "https://api.kwater.or.kr/gw/v1";
    wells: string[];
  };
}

class KWaterClient {
  private integration: KWaterIntegration;

  // 댐 저수율 조회
  async getDamStorageRate(damCode: string, date: Date): Promise<DamStorage> {
    const response = await this.fetch(
      `${this.integration.damStorage.endpoint}/storage`,
      { damCode, date: formatDate(date) }
    );

    return {
      damName: response.damName,
      currentStorage: response.storage,        // 백만㎥
      effectiveStorage: response.effectiveStorage,
      storageRate: response.rate,              // %
      inflow: response.inflow,                 // ㎥/s
      outflow: response.outflow,               // ㎥/s
      avgStorageRate: response.avgRate,        // 평년 저수율
      deviation: response.rate - response.avgRate
    };
  }

  // 전국 주요댐 가뭄 상태 종합
  async getNationalDamDroughtStatus(): Promise<NationalDamStatus> {
    const majorDams = [
      "SOYANG", "CHUNGJU", "ANDONG", "IMHA",
      "HAPCHEON", "NAMGANG", "JUAM", "YONGDAM"
    ];

    const storages = await Promise.all(
      majorDams.map(dam => this.getDamStorageRate(dam, new Date()))
    );

    // 가뭄 단계 분류
    // | 저수율 | 단계 |
    // |-------|------|
    // | >70%  | 정상 |
    // | 50-70%| 관심 |
    // | 30-50%| 주의 |
    // | 20-30%| 경계 |
    // | <20%  | 심각 |

    const classifications = storages.map(s => ({
      dam: s.damName,
      rate: s.storageRate,
      level: this.classifyDroughtLevel(s.storageRate),
      trend: this.calculateTrend(s)
    }));

    return {
      timestamp: new Date(),
      dams: classifications,
      nationalAverage: this.calculateWeightedAverage(storages),
      criticalDams: classifications.filter(c => c.level >= 3),
      recommendation: this.generateWaterSupplyRecommendation(classifications)
    };
  }

  private classifyDroughtLevel(rate: number): number {
    if (rate >= 70) return 0;  // 정상
    if (rate >= 50) return 1;  // 관심
    if (rate >= 30) return 2;  // 주의
    if (rate >= 20) return 3;  // 경계
    return 4;                   // 심각
  }
}
```

### 7.5.3 농어촌공사 농업용수 관리

```typescript
// 한국농어촌공사 통합
interface KRCIntegration {
  // 저수지 정보
  reservoir: {
    endpoint: "https://api.ekr.or.kr/reservoir/v1";
  };

  // 용수로 정보
  canal: {
    endpoint: "https://api.ekr.or.kr/canal/v1";
  };

  // 지구별 용수 수급
  irrigationDistrict: {
    endpoint: "https://api.ekr.or.kr/district/v1";
  };
}

class KRCWaterManagement {
  private integration: KRCIntegration;

  // 관개지구 물 수급 현황
  async getDistrictWaterBalance(districtCode: string): Promise<WaterBalance> {
    const [supply, demand, forecast] = await Promise.all([
      this.getWaterSupply(districtCode),
      this.getWaterDemand(districtCode),
      this.getSeasonalForecast(districtCode)
    ]);

    const balance = supply.available - demand.current;
    const daysRemaining = balance / demand.dailyRate;

    return {
      district: districtCode,
      supply: {
        reservoirStorage: supply.reservoir,
        riverDiversion: supply.river,
        groundwater: supply.groundwater,
        totalAvailable: supply.available
      },
      demand: {
        paddy: demand.paddy,
        upland: demand.upland,
        other: demand.other,
        totalDemand: demand.current,
        peakDemand: demand.peak
      },
      balance: {
        current: balance,
        daysRemaining,
        status: this.classifyBalanceStatus(daysRemaining),
        restrictionNeeded: daysRemaining < 14
      },
      forecast: {
        nextMonth: forecast.nextMonth,
        season: forecast.season,
        uncertainty: forecast.uncertainty
      }
    };
  }

  // 용수 제한 권고 생성
  generateWaterRestriction(balance: WaterBalance): WaterRestriction {
    if (!balance.balance.restrictionNeeded) {
      return { active: false };
    }

    const severity = this.calculateRestrictionSeverity(balance);

    return {
      active: true,
      level: severity,
      measures: this.getRestrictionMeasures(severity),
      startDate: new Date(),
      reviewDate: this.addDays(new Date(), 7),
      affectedCrops: this.getPriorityCrops(balance),
      reductionTarget: this.calculateReductionTarget(balance)
    };
  }

  private getRestrictionMeasures(level: number): string[] {
    const measures = {
      1: [  // 1단계 - 자율 절감
        "관개 시간대 분산 (야간 관개 권장)",
        "물꼬 높이 조절",
        "누수 점검 및 보수"
      ],
      2: [  // 2단계 - 감량 공급
        "관개 주기 연장 (7일 → 10일)",
        "1회 관개량 20% 감소",
        "밭작물 스프링클러 사용 제한"
      ],
      3: [  // 3단계 - 순환 공급
        "지역별 순환 공급 실시",
        "관개 주기 14일 이상",
        "논물 걸러대기 실시"
      ],
      4: [  // 4단계 - 긴급 제한
        "우선순위 작물만 공급",
        "밭작물 공급 중단",
        "긴급 양수기 설치"
      ]
    };

    return measures[level] || measures[4];
  }
}
```

---

## 7.6 복습 문제

### 문제 1
스마트팜 플랫폼과 WIA 가뭄 모니터링 시스템을 통합할 때, 데이터 동기화에서 고려해야 할 주요 요소 3가지를 설명하시오.

### 문제 2
관개 의사결정 엔진에서 "강우 예보 시 관개 연기" 규칙이 발동되는 조건과, ML 모델 예측과 어떻게 앙상블되는지 설명하시오.

### 문제 3
국가 재난 경보 시스템에서 가뭄 등급 D3(심각) 발령 시 활성화되는 알림 채널과 각 채널의 특성을 비교하시오.

### 문제 4
위성 데이터 파이프라인에서 SMAP 토양 수분 데이터를 Sentinel-1과 융합하여 다운스케일링하는 과정을 단계별로 설명하시오.

### 문제 5
한국농어촌공사의 용수 제한 4단계 조치와 각 단계에서 권장되는 구체적인 절수 방안을 서술하시오.

---

## 7.7 핵심 요약

### 주요 학습 내용

| 항목 | 핵심 내용 |
|-----|---------|
| 농업 시스템 통합 | 스마트팜 플랫폼 어댑터, 양방향 데이터 동기화, 형식 변환 |
| 관개 제어 | 규칙+ML 앙상블 의사결정, 자동 관개 실행, 실시간 모니터링 |
| 조기 경보 | 5단계 경보 체계, 다중 채널 발송, 에스컬레이션 정책 |
| 위성 파이프라인 | 다중 위성 통합, 자동화 처리, 토양수분 다운스케일링 |
| 한국 인프라 | 농진청/K-water/농어촌공사 API 연동, 용수 관리 |

### 핵심 기억 사항

1. **스마트팜 통합**: 플랫폼별 데이터 형식 변환과 충돌 해결 전략이 필수
2. **관개 의사결정**: 긴급 상황은 규칙 우선, 일반 상황은 ML 예측 활용
3. **경보 에스컬레이션**: 등급별 발송 채널과 해제 조건이 다름
4. **위성 처리**: 대기 보정 → 구름 마스킹 → 지수 계산 → 타일링 순서
5. **용수 관리**: 4단계 제한 조치와 우선순위 작물 선정 기준 숙지

### 다음 장 예고

제8장에서는 가뭄 모니터링 시스템의 실제 구축 및 운영을 다룹니다. 인프라 구성, 배포 전략, 모니터링, 유지보수 및 확장 방안을 학습합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
