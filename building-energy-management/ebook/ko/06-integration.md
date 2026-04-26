# 제6장: 시스템 통합 (Phase 4)

## 서론

WIA-BEMS Phase 4는 빌딩 에너지 관리 시스템을 외부 시스템과 통합하여 시너지를 창출합니다. 본 장에서는 스마트 그리드 연계, 재생에너지 통합, 그리고 엔터프라이즈 시스템과의 연동 방법을 상세히 다룹니다.

---

## 6.1 스마트 그리드 연계

### 6.1.1 수요 응답 (Demand Response)

```typescript
// 수요 응답 시스템 아키텍처
interface DemandResponseArchitecture {
  protocols: {
    openADR: {
      version: '2.0b';
      profile: 'VEN (Virtual End Node)';
      transport: ['HTTP/TLS', 'XMPP'];
      services: [
        'EiEvent', 'EiReport', 'EiRegisterParty',
        'EiOpt', 'OadrPoll'
      ];
    };
    ieee2030_5: {
      name: 'Smart Energy Profile 2.0';
      transport: 'REST over HTTPS';
      functions: [
        'DERProgram', 'EndDeviceControl', 'FlowReservation',
        'Pricing', 'Response'
      ];
    };
    koreanDR: {
      name: '한전 DR 프로토콜';
      operator: '한국전력';
      programs: ['신뢰성 DR', '경제성 DR', '피크 관리'];
    };
  };

  eventTypes: {
    priceSignal: '가격 신호 기반 반응';
    loadControl: '직접 부하 제어';
    ancillaryService: '주파수 조정, 예비력';
    emergency: '비상 부하 차단';
  };
}

// OpenADR 2.0b VEN 구현
class OpenADRVEN {
  private config: VENConfig;
  private client: HTTPClient;
  private pollScheduler: PollScheduler;
  private eventHandler: EventHandler;
  private reportManager: ReportManager;

  constructor(config: VENConfig) {
    this.config = config;
    this.client = new HTTPClient({
      baseUrl: config.vtnUrl,
      certPath: config.clientCertPath,
      keyPath: config.clientKeyPath
    });
    this.eventHandler = new EventHandler(this);
    this.reportManager = new ReportManager(this);
  }

  async register(): Promise<RegistrationResponse> {
    const registrationPayload = {
      oadrCreatePartyRegistration: {
        requestID: generateRequestId(),
        venID: this.config.venId,
        oadrProfileName: '2.0b',
        oadrTransportName: 'simpleHttp',
        oadrReportOnly: false,
        oadrXmlSignature: false,
        oadrVenName: this.config.venName,
        oadrHttpPullModel: true
      }
    };

    const response = await this.client.post(
      '/EiRegisterParty',
      registrationPayload
    );

    if (response.oadrCreatedPartyRegistration) {
      this.config.registrationId = response.oadrCreatedPartyRegistration.registrationID;
      console.log('OpenADR VEN 등록 완료:', this.config.registrationId);

      // 폴링 시작
      await this.startPolling();
    }

    return response;
  }

  async poll(): Promise<PollResponse> {
    const pollPayload = {
      oadrPoll: {
        venID: this.config.venId
      }
    };

    const response = await this.client.post('/OadrPoll', pollPayload);

    // 응답 유형에 따른 처리
    if (response.oadrDistributeEvent) {
      await this.handleDistributeEvent(response.oadrDistributeEvent);
    } else if (response.oadrCreateReport) {
      await this.handleCreateReport(response.oadrCreateReport);
    } else if (response.oadrCancelReport) {
      await this.handleCancelReport(response.oadrCancelReport);
    }

    return response;
  }

  private async handleDistributeEvent(event: DistributeEvent): Promise<void> {
    for (const oadrEvent of event.oadrEvent) {
      const eventId = oadrEvent.eiEvent.eventDescriptor.eventID;
      const eventStatus = oadrEvent.eiEvent.eventDescriptor.eventStatus;

      console.log(`수신 DR 이벤트: ${eventId}, 상태: ${eventStatus}`);

      // 이벤트 상세 파싱
      const eventDetails = this.parseEventDetails(oadrEvent.eiEvent);

      // 이벤트 응답 결정
      const optType = await this.eventHandler.evaluateEvent(eventDetails);

      // CreatedEvent 응답 전송
      await this.sendCreatedEvent(eventId, optType, oadrEvent.oadrResponseRequired);
    }
  }

  private parseEventDetails(eiEvent: EiEvent): DREventDetails {
    const signals = eiEvent.eiEventSignals.eiEventSignal;

    return {
      eventId: eiEvent.eventDescriptor.eventID,
      modificationNumber: eiEvent.eventDescriptor.modificationNumber,
      priority: eiEvent.eventDescriptor.priority,
      marketContext: eiEvent.eventDescriptor.eiMarketContext.marketContext,
      status: eiEvent.eventDescriptor.eventStatus,
      startTime: new Date(eiEvent.eiActivePeriod.properties.dtstart.dateTime),
      duration: this.parseDuration(eiEvent.eiActivePeriod.properties.duration.duration),
      notification: this.parseDuration(eiEvent.eiActivePeriod.properties.xEiNotification?.duration),
      rampUp: this.parseDuration(eiEvent.eiActivePeriod.properties.xEiRampUp?.duration),
      signals: signals.map(signal => ({
        signalName: signal.signalName,
        signalType: signal.signalType,
        intervals: signal.intervals.interval.map(interval => ({
          start: new Date(interval.dtstart.dateTime),
          duration: this.parseDuration(interval.duration.duration),
          value: parseFloat(interval.signalPayload.payloadFloat.value)
        }))
      }))
    };
  }

  private async sendCreatedEvent(
    eventId: string,
    optType: 'optIn' | 'optOut',
    responseRequired: string
  ): Promise<void> {
    if (responseRequired === 'never') return;

    const createdEventPayload = {
      oadrCreatedEvent: {
        eiCreatedEvent: {
          eiResponse: {
            responseCode: '200',
            responseDescription: 'OK',
            requestID: generateRequestId()
          },
          eventResponses: {
            eventResponse: [{
              responseCode: '200',
              responseDescription: 'OK',
              requestID: generateRequestId(),
              qualifiedEventID: {
                eventID: eventId,
                modificationNumber: 0
              },
              optType
            }]
          },
          venID: this.config.venId
        }
      }
    };

    await this.client.post('/EiEvent', createdEventPayload);
  }

  async sendReport(report: TelemetryReport): Promise<void> {
    const reportPayload = {
      oadrUpdateReport: {
        requestID: generateRequestId(),
        venID: this.config.venId,
        oadrReport: [{
          eiReportID: report.reportId,
          eiReportDescriptions: report.descriptions,
          dtstart: { dateTime: report.startTime.toISOString() },
          duration: { duration: report.duration },
          intervals: {
            interval: report.intervals.map(interval => ({
              dtstart: { dateTime: interval.timestamp.toISOString() },
              streamPayloadBase: {
                oadrPayloadResourceStatus: interval.resourceStatus
              },
              reportPayload: {
                rID: interval.resourceId,
                payloadFloat: { value: interval.value }
              }
            }))
          }
        }]
      }
    };

    await this.client.post('/EiReport', reportPayload);
  }

  private async startPolling(): Promise<void> {
    const pollInterval = this.config.pollInterval || 60000; // 기본 1분

    this.pollScheduler = setInterval(async () => {
      try {
        await this.poll();
      } catch (error) {
        console.error('OpenADR 폴링 오류:', error);
      }
    }, pollInterval);
  }
}

// DR 이벤트 핸들러
class EventHandler {
  private ven: OpenADRVEN;
  private buildingController: BuildingController;
  private economicModel: EconomicModel;

  constructor(ven: OpenADRVEN) {
    this.ven = ven;
  }

  async evaluateEvent(event: DREventDetails): Promise<'optIn' | 'optOut'> {
    // 1. 운영 제약 확인
    const operationalFeasibility = await this.checkOperationalFeasibility(event);
    if (!operationalFeasibility.feasible) {
      console.log('DR 이벤트 옵트아웃: 운영 제약', operationalFeasibility.reason);
      return 'optOut';
    }

    // 2. 경제성 분석
    const economicAnalysis = await this.analyzeEconomics(event);
    if (!economicAnalysis.beneficial) {
      console.log('DR 이벤트 옵트아웃: 경제성 불리', economicAnalysis.reason);
      return 'optOut';
    }

    // 3. 쾌적성 영향 평가
    const comfortImpact = await this.assessComfortImpact(event);
    if (comfortImpact.unacceptable) {
      console.log('DR 이벤트 옵트아웃: 쾌적성 영향', comfortImpact.reason);
      return 'optOut';
    }

    console.log('DR 이벤트 옵트인: 참여 결정');

    // 4. DR 전략 수립 및 실행 예약
    await this.prepareDRStrategy(event);

    return 'optIn';
  }

  private async checkOperationalFeasibility(
    event: DREventDetails
  ): Promise<FeasibilityResult> {
    // 예정된 유지보수 확인
    const maintenance = await this.buildingController.getScheduledMaintenance(
      event.startTime,
      new Date(event.startTime.getTime() + event.duration)
    );

    if (maintenance.length > 0) {
      return { feasible: false, reason: '유지보수 예정됨' };
    }

    // 중요 이벤트 확인
    const criticalEvents = await this.buildingController.getCriticalEvents(
      event.startTime,
      event.duration
    );

    if (criticalEvents.length > 0) {
      return { feasible: false, reason: '중요 이벤트 예정됨' };
    }

    // 현재 설비 상태 확인
    const equipmentStatus = await this.buildingController.getEquipmentStatus();
    const criticalAlarms = equipmentStatus.filter(e => e.hasAlarm && e.alarmSeverity === 'critical');

    if (criticalAlarms.length > 0) {
      return { feasible: false, reason: '설비 장애 상태' };
    }

    return { feasible: true };
  }

  private async analyzeEconomics(event: DREventDetails): Promise<EconomicAnalysis> {
    // DR 인센티브 계산
    const incentive = this.calculateIncentive(event);

    // 에너지 비용 영향 계산
    const energyCostImpact = await this.calculateEnergyCostImpact(event);

    // 쾌적성 비용 (생산성 영향 등)
    const comfortCost = await this.calculateComfortCost(event);

    const netBenefit = incentive - energyCostImpact - comfortCost;

    return {
      beneficial: netBenefit > 0,
      netBenefit,
      breakdown: {
        incentive,
        energyCostImpact,
        comfortCost
      },
      reason: netBenefit <= 0 ? `순이익 ${netBenefit}원 (부적합)` : undefined
    };
  }

  private async prepareDRStrategy(event: DREventDetails): Promise<void> {
    // 신호 유형에 따른 전략 수립
    const strategies: DRStrategy[] = [];

    for (const signal of event.signals) {
      switch (signal.signalName) {
        case 'ELECTRICITY_PRICE':
          strategies.push(await this.createPriceResponseStrategy(signal));
          break;
        case 'LOAD_CONTROL':
          strategies.push(await this.createLoadControlStrategy(signal));
          break;
        case 'SIMPLE':
          strategies.push(await this.createSimpleLevelStrategy(signal));
          break;
      }
    }

    // 전략 스케줄링
    await this.buildingController.scheduleDRStrategies(strategies);
  }

  private async createLoadControlStrategy(
    signal: DRSignal
  ): Promise<DRStrategy> {
    const actions: DRAction[] = [];

    // 부하 우선순위에 따른 차단/저감 계획
    const shedableLoads = await this.buildingController.getShedableLoads();

    let targetReduction = 0;
    for (const interval of signal.intervals) {
      targetReduction = Math.max(targetReduction, interval.value);
    }

    let currentReduction = 0;
    for (const load of shedableLoads.sort((a, b) => a.priority - b.priority)) {
      if (currentReduction >= targetReduction) break;

      actions.push({
        type: 'shed',
        target: load.id,
        value: load.maxReduction,
        startTime: signal.intervals[0].start,
        duration: signal.intervals[0].duration
      });

      currentReduction += load.maxReduction;
    }

    return {
      signalName: signal.signalName,
      actions,
      expectedReduction: currentReduction
    };
  }
}
```

### 6.1.2 그리드 서비스 제공

```typescript
// 그리드 서비스 제공 시스템
interface GridServicesProvider {
  services: {
    frequencyRegulation: {
      name: '주파수 조정';
      responseTime: '< 4초';
      duration: '5-30분';
      accuracy: '±2%';
      compensation: '용량 (₩/MW) + 실행 (₩/MWh)';
    };
    spinningReserve: {
      name: '스피닝 예비력';
      responseTime: '< 10분';
      duration: '1-2시간';
      compensation: '용량 (₩/MW)';
    };
    nonSpinningReserve: {
      name: '비스피닝 예비력';
      responseTime: '< 30분';
      duration: '2-4시간';
    };
    voltageSupport: {
      name: '전압 지원';
      method: '무효 전력 제어';
      assets: ['인버터', 'SVC', '커패시터 뱅크'];
    };
  };
}

// 주파수 조정 서비스 제공자
class FrequencyRegulationProvider {
  private config: FRConfig;
  private signalReceiver: AGCSignalReceiver;
  private assetController: FlexibleAssetController;
  private performanceTracker: PerformanceTracker;

  constructor(config: FRConfig) {
    this.config = config;
    this.signalReceiver = new AGCSignalReceiver(config.agcEndpoint);
    this.assetController = new FlexibleAssetController();
    this.performanceTracker = new PerformanceTracker();
  }

  async startRegulationService(): Promise<void> {
    // AGC 신호 수신 시작
    this.signalReceiver.onSignal(async (signal: AGCSignal) => {
      await this.handleAGCSignal(signal);
    });

    // 성능 보고 시작
    this.startPerformanceReporting();
  }

  private async handleAGCSignal(signal: AGCSignal): Promise<void> {
    const targetPower = signal.setpoint; // MW
    const currentPower = await this.assetController.getCurrentOutput();

    const requiredChange = targetPower - currentPower;

    // 응답 시간 내 목표 출력 달성
    const responseStart = Date.now();

    // 자산별 출력 분배
    const distribution = await this.distributeOutput(requiredChange);

    // 자산 제어 명령 실행
    await Promise.all(
      distribution.map(d => this.assetController.setOutput(d.assetId, d.power))
    );

    // 성능 기록
    const responseTime = Date.now() - responseStart;
    const actualOutput = await this.assetController.getCurrentOutput();

    this.performanceTracker.record({
      timestamp: new Date(),
      signalSetpoint: targetPower,
      actualOutput,
      responseTime,
      accuracy: 1 - Math.abs(targetPower - actualOutput) / Math.abs(targetPower)
    });
  }

  private async distributeOutput(
    requiredChange: number
  ): Promise<AssetOutputDistribution[]> {
    const distribution: AssetOutputDistribution[] = [];
    const assets = await this.assetController.getAvailableAssets();

    // 우선순위 및 응답 속도 기반 분배
    const sortedAssets = assets.sort((a, b) => {
      // 응답 속도 우선
      return a.responseTime - b.responseTime;
    });

    let remainingChange = requiredChange;

    for (const asset of sortedAssets) {
      if (Math.abs(remainingChange) < 0.001) break;

      const capacity = requiredChange > 0 ?
        asset.maxOutput - asset.currentOutput :
        asset.currentOutput - asset.minOutput;

      const assignedChange = Math.sign(remainingChange) *
        Math.min(Math.abs(remainingChange), capacity);

      distribution.push({
        assetId: asset.id,
        power: asset.currentOutput + assignedChange
      });

      remainingChange -= assignedChange;
    }

    return distribution;
  }

  async calculateAvailableCapacity(): Promise<RegulationCapacity> {
    const assets = await this.assetController.getAvailableAssets();

    let regUp = 0;   // 상향 조정 용량
    let regDown = 0; // 하향 조정 용량

    for (const asset of assets) {
      regUp += asset.maxOutput - asset.currentOutput;
      regDown += asset.currentOutput - asset.minOutput;
    }

    return {
      regUp,
      regDown,
      responseTime: Math.max(...assets.map(a => a.responseTime)),
      accuracy: this.performanceTracker.getAverageAccuracy()
    };
  }
}
```

---

## 6.2 재생에너지 통합

### 6.2.1 태양광 발전 연계

```typescript
// 태양광 발전 연계 시스템
interface SolarPVIntegration {
  systemComponents: {
    pvArray: {
      capacity: number; // kWp
      orientation: number; // 방위각 (남향=180°)
      tilt: number; // 경사각
      efficiency: number;
    };
    inverter: {
      capacity: number; // kVA
      efficiency: number;
      mpptChannels: number;
      gridTie: boolean;
    };
    monitoring: {
      irradianceSensor: boolean;
      moduleTempSensor: boolean;
      stringMonitoring: boolean;
    };
  };

  operatingModes: {
    gridTie: 'Grid-Tie 방식';
    selfConsumption: '자가 소비 우선';
    peakShaving: '피크 저감';
    export: '잉여 전력 수출';
  };
}

// 태양광 발전 컨트롤러
class SolarPVController {
  private pvSystem: PVSystemConfig;
  private inverterInterface: InverterInterface;
  private forecaster: SolarForecastService;
  private buildingLoad: BuildingLoadMonitor;

  constructor(config: SolarPVConfig) {
    this.pvSystem = config.pvSystem;
    this.inverterInterface = new InverterInterface(config.inverterConfig);
    this.forecaster = new SolarForecastService(config.location);
  }

  async getCurrentGeneration(): Promise<SolarGenerationData> {
    const inverterData = await this.inverterInterface.getData();

    return {
      timestamp: new Date(),
      dcPower: inverterData.dcPower, // kW
      acPower: inverterData.acPower, // kW
      dailyEnergy: inverterData.dailyEnergy, // kWh
      totalEnergy: inverterData.totalEnergy, // kWh
      efficiency: inverterData.acPower / inverterData.dcPower,
      stringData: inverterData.strings,
      irradiance: inverterData.irradiance,
      moduleTemperature: inverterData.moduleTemperature,
      ambientTemperature: inverterData.ambientTemperature
    };
  }

  async getForecast(hours: number): Promise<SolarForecast> {
    // 기상 예보 기반 발전량 예측
    const weatherForecast = await this.forecaster.getWeatherForecast(hours);

    const forecast: SolarForecastPoint[] = [];

    for (const weather of weatherForecast) {
      // GHI (Global Horizontal Irradiance) → POA (Plane of Array) 변환
      const poaIrradiance = this.calculatePOAIrradiance(
        weather.ghi,
        weather.dni,
        weather.dhi,
        weather.timestamp
      );

      // 모듈 온도 추정
      const moduleTemp = this.estimateModuleTemperature(
        weather.ambientTemperature,
        poaIrradiance,
        weather.windSpeed
      );

      // DC 출력 계산
      const dcPower = this.calculateDCPower(poaIrradiance, moduleTemp);

      // AC 출력 계산 (인버터 효율 반영)
      const acPower = dcPower * this.getInverterEfficiency(dcPower);

      forecast.push({
        timestamp: weather.timestamp,
        expectedPower: acPower,
        confidence: weather.confidence,
        weather: {
          irradiance: poaIrradiance,
          temperature: moduleTemp,
          cloudCover: weather.cloudCover
        }
      });
    }

    return { forecast };
  }

  private calculatePOAIrradiance(
    ghi: number,
    dni: number,
    dhi: number,
    timestamp: Date
  ): number {
    // 태양 위치 계산
    const sunPosition = SolarGeometry.getSunPosition(
      timestamp,
      this.pvSystem.latitude,
      this.pvSystem.longitude
    );

    // POA 직달 성분
    const aoi = SolarGeometry.getAngleOfIncidence(
      sunPosition.zenith,
      sunPosition.azimuth,
      this.pvSystem.tilt,
      this.pvSystem.orientation
    );
    const poaBeam = dni * Math.cos(aoi * Math.PI / 180);

    // POA 확산 성분 (Perez 모델 간략화)
    const poaDiffuse = dhi * (1 + Math.cos(this.pvSystem.tilt * Math.PI / 180)) / 2;

    // POA 반사 성분
    const albedo = 0.2;
    const poaReflected = ghi * albedo * (1 - Math.cos(this.pvSystem.tilt * Math.PI / 180)) / 2;

    return Math.max(0, poaBeam + poaDiffuse + poaReflected);
  }

  private estimateModuleTemperature(
    ambientTemp: number,
    irradiance: number,
    windSpeed: number
  ): number {
    // NOCT 기반 모듈 온도 추정
    const noct = this.pvSystem.noct || 45; // Normal Operating Cell Temperature
    const tempRise = (noct - 20) * (irradiance / 800) * (9.5 / (5.7 + 3.8 * windSpeed));
    return ambientTemp + tempRise;
  }

  private calculateDCPower(irradiance: number, moduleTemp: number): number {
    // STC (1000 W/m², 25°C) 기준 출력에서 보정
    const stcPower = this.pvSystem.capacity;
    const tempCoeff = this.pvSystem.tempCoefficient || -0.004; // %/°C

    // 조도 보정
    const irradianceFactor = irradiance / 1000;

    // 온도 보정
    const tempDelta = moduleTemp - 25;
    const tempFactor = 1 + tempCoeff * tempDelta;

    // 기타 손실 (오염, 배선 등)
    const derateFactor = this.pvSystem.derateFactor || 0.85;

    return stcPower * irradianceFactor * tempFactor * derateFactor;
  }

  async optimizeOperation(
    buildingLoad: LoadProfile,
    gridPrices: PriceProfile
  ): Promise<PVOperationStrategy> {
    const forecast = await this.getForecast(24);
    const strategy: PVOperationStrategy = {
      mode: 'self_consumption',
      actions: []
    };

    for (let h = 0; h < 24; h++) {
      const generation = forecast.forecast[h]?.expectedPower || 0;
      const load = buildingLoad.hourly[h];
      const price = gridPrices.hourly[h];

      const surplus = generation - load;

      if (surplus > 0) {
        // 잉여 전력 처리
        if (price > gridPrices.exportPrice) {
          // 수출보다 자가 소비가 유리 → ESS 충전
          strategy.actions.push({
            hour: h,
            action: 'charge_ess',
            amount: surplus
          });
        } else {
          // 수출
          strategy.actions.push({
            hour: h,
            action: 'export',
            amount: surplus
          });
        }
      }
    }

    return strategy;
  }
}
```

### 6.2.2 에너지 저장 시스템 (ESS) 연계

```typescript
// ESS 통합 관리 시스템
interface ESSIntegration {
  systemConfig: {
    chemistry: 'LFP' | 'NMC' | 'NCA' | 'LTO';
    capacity: number; // kWh
    maxPower: number; // kW
    roundTripEfficiency: number;
    depthOfDischarge: number;
    cycleLife: number;
  };

  operatingModes: {
    peakShaving: '피크 저감';
    loadLeveling: '부하 평준화';
    selfConsumption: '자가 소비';
    arbitrage: '에너지 차익 거래';
    backup: '비상 전원';
    gridServices: '그리드 서비스';
  };
}

// ESS 컨트롤러
class ESSController {
  private config: ESSConfig;
  private bmsInterface: BMSInterface;
  private pcsInterface: PCSInterface;
  private optimizer: ESSOptimizer;

  constructor(config: ESSConfig) {
    this.config = config;
    this.bmsInterface = new BMSInterface(config.bmsConfig);
    this.pcsInterface = new PCSInterface(config.pcsConfig);
    this.optimizer = new ESSOptimizer(config);
  }

  async getStatus(): Promise<ESSStatus> {
    const bmsData = await this.bmsInterface.getData();
    const pcsData = await this.pcsInterface.getData();

    return {
      timestamp: new Date(),
      soc: bmsData.stateOfCharge,
      soh: bmsData.stateOfHealth,
      voltage: bmsData.packVoltage,
      current: bmsData.packCurrent,
      temperature: {
        min: bmsData.minCellTemperature,
        max: bmsData.maxCellTemperature,
        average: bmsData.avgCellTemperature
      },
      power: pcsData.activePower,
      mode: pcsData.operatingMode,
      alarms: [...bmsData.alarms, ...pcsData.alarms],
      availableEnergy: this.calculateAvailableEnergy(bmsData.stateOfCharge),
      availablePower: this.calculateAvailablePower(bmsData, pcsData)
    };
  }

  async setOperatingMode(mode: ESSOperatingMode): Promise<void> {
    switch (mode) {
      case 'peak_shaving':
        await this.enablePeakShaving();
        break;
      case 'self_consumption':
        await this.enableSelfConsumption();
        break;
      case 'grid_services':
        await this.enableGridServices();
        break;
      case 'backup':
        await this.enableBackupMode();
        break;
      case 'manual':
        await this.enableManualMode();
        break;
    }
  }

  async charge(power: number, duration?: number): Promise<ChargeResult> {
    // 충전 가능 여부 확인
    const status = await this.getStatus();

    if (status.soc >= this.config.maxSoc) {
      return { success: false, reason: 'SOC 상한 도달' };
    }

    // 전력 제한 적용
    const allowedPower = Math.min(
      power,
      this.config.maxChargePower,
      this.calculateAvailableChargePower(status)
    );

    // 충전 명령 전송
    await this.pcsInterface.setChargePower(allowedPower);

    return {
      success: true,
      requestedPower: power,
      actualPower: allowedPower,
      estimatedDuration: duration || this.estimateChargeDuration(allowedPower, status.soc)
    };
  }

  async discharge(power: number, duration?: number): Promise<DischargeResult> {
    const status = await this.getStatus();

    if (status.soc <= this.config.minSoc) {
      return { success: false, reason: 'SOC 하한 도달' };
    }

    const allowedPower = Math.min(
      power,
      this.config.maxDischargePower,
      this.calculateAvailableDischargePower(status)
    );

    await this.pcsInterface.setDischargePower(allowedPower);

    return {
      success: true,
      requestedPower: power,
      actualPower: allowedPower,
      estimatedDuration: duration || this.estimateDischargeDuration(allowedPower, status.soc)
    };
  }

  async optimizeSchedule(
    loadForecast: LoadProfile,
    pvForecast: SolarForecast,
    prices: PriceProfile
  ): Promise<ESSSchedule> {
    // MPC 기반 최적 충방전 스케줄 계산
    const schedule = await this.optimizer.optimize({
      loadForecast,
      pvForecast,
      prices,
      currentSoc: (await this.getStatus()).soc,
      constraints: {
        minSoc: this.config.minSoc,
        maxSoc: this.config.maxSoc,
        maxChargePower: this.config.maxChargePower,
        maxDischargePower: this.config.maxDischargePower,
        efficiency: this.config.roundTripEfficiency
      }
    });

    return schedule;
  }

  private async enablePeakShaving(): Promise<void> {
    // 피크 저감 모드
    // 실시간 부하 모니터링 및 피크 도달 시 방전
    const peakThreshold = this.config.peakShavingThreshold;

    const monitorLoop = async () => {
      const buildingLoad = await this.getBuildingLoad();
      const status = await this.getStatus();

      if (buildingLoad > peakThreshold && status.soc > this.config.minSoc) {
        // 피크 초과 → 방전
        const requiredDischarge = buildingLoad - peakThreshold;
        await this.discharge(requiredDischarge);
      } else if (buildingLoad < peakThreshold * 0.7 && status.soc < this.config.maxSoc) {
        // 낮은 부하 → 충전 (다음 피크 대비)
        const chargeCapacity = peakThreshold * 0.7 - buildingLoad;
        await this.charge(Math.min(chargeCapacity, this.config.maxChargePower * 0.5));
      }
    };

    // 모니터링 주기: 1분
    setInterval(monitorLoop, 60000);
  }

  private calculateAvailableEnergy(soc: number): number {
    const usableSoc = Math.max(0, soc - this.config.minSoc);
    return this.config.capacity * usableSoc / 100;
  }
}

// ESS 최적화 엔진
class ESSOptimizer {
  private config: ESSConfig;

  constructor(config: ESSConfig) {
    this.config = config;
  }

  async optimize(params: OptimizationParams): Promise<ESSSchedule> {
    const horizon = params.loadForecast.hourly.length;
    const schedule: ESSSchedulePoint[] = [];

    // 동적 프로그래밍 기반 최적화
    const dp: DPState[][] = [];

    // SOC 이산화
    const socSteps = 20;
    const socMin = params.constraints.minSoc;
    const socMax = params.constraints.maxSoc;
    const socDelta = (socMax - socMin) / socSteps;

    // 초기화
    for (let h = 0; h <= horizon; h++) {
      dp[h] = [];
      for (let s = 0; s <= socSteps; s++) {
        dp[h][s] = {
          cost: h === 0 ? 0 : Infinity,
          previousSoc: -1,
          action: 'idle'
        };
      }
    }

    // 초기 SOC에 해당하는 상태 설정
    const initialSocIndex = Math.round(
      (params.currentSoc - socMin) / socDelta
    );
    dp[0][initialSocIndex].cost = 0;

    // 순방향 계산
    for (let h = 0; h < horizon; h++) {
      const price = params.prices.hourly[h];
      const load = params.loadForecast.hourly[h];
      const pv = params.pvForecast?.forecast[h]?.expectedPower || 0;

      for (let s = 0; s <= socSteps; s++) {
        if (dp[h][s].cost === Infinity) continue;

        const currentSoc = socMin + s * socDelta;

        // 가능한 행동 탐색
        for (let nextS = 0; nextS <= socSteps; nextS++) {
          const nextSoc = socMin + nextS * socDelta;
          const socChange = nextSoc - currentSoc;

          // 에너지 변환 (효율 고려)
          let power: number;
          let action: 'charge' | 'discharge' | 'idle';

          if (socChange > 0) {
            // 충전
            power = socChange * this.config.capacity / 100 /
                    Math.sqrt(params.constraints.efficiency);
            action = 'charge';
          } else if (socChange < 0) {
            // 방전
            power = -socChange * this.config.capacity / 100 *
                    Math.sqrt(params.constraints.efficiency);
            action = 'discharge';
          } else {
            power = 0;
            action = 'idle';
          }

          // 전력 제한 확인
          if (action === 'charge' && power > params.constraints.maxChargePower) continue;
          if (action === 'discharge' && power > params.constraints.maxDischargePower) continue;

          // 비용 계산
          let netLoad = load - pv;
          if (action === 'charge') {
            netLoad += power;
          } else if (action === 'discharge') {
            netLoad -= power;
          }

          const energyCost = Math.max(0, netLoad) * price;
          const exportRevenue = Math.max(0, -netLoad) * (params.prices.exportPrice || price * 0.5);
          const stepCost = energyCost - exportRevenue;

          const totalCost = dp[h][s].cost + stepCost;

          if (totalCost < dp[h + 1][nextS].cost) {
            dp[h + 1][nextS] = {
              cost: totalCost,
              previousSoc: s,
              action,
              power
            };
          }
        }
      }
    }

    // 역추적으로 최적 스케줄 구성
    let currentSocIdx = 0;
    let minFinalCost = Infinity;
    for (let s = 0; s <= socSteps; s++) {
      if (dp[horizon][s].cost < minFinalCost) {
        minFinalCost = dp[horizon][s].cost;
        currentSocIdx = s;
      }
    }

    for (let h = horizon - 1; h >= 0; h--) {
      const state = dp[h + 1][currentSocIdx];
      schedule.unshift({
        hour: h,
        action: state.action,
        power: state.power || 0,
        expectedSoc: socMin + currentSocIdx * socDelta
      });
      currentSocIdx = state.previousSoc;
    }

    return {
      schedule,
      expectedCost: minFinalCost,
      peakReduction: this.calculatePeakReduction(schedule, params.loadForecast)
    };
  }
}
```

---

## 6.3 엔터프라이즈 시스템 연동

### 6.3.1 CMMS/ERP 연동

```typescript
// CMMS 연동 인터페이스
interface CMMSIntegration {
  workOrderManagement: {
    autoCreate: '자동 작업 지시서 생성';
    statusSync: '작업 상태 동기화';
    partTracking: '자재 추적';
    laborTracking: '인력 추적';
  };

  assetManagement: {
    assetSync: '자산 정보 동기화';
    maintenanceHistory: '유지보수 이력';
    warrantyTracking: '보증 추적';
    depreciationSync: '감가상각 동기화';
  };

  inventoryManagement: {
    partLevels: '부품 재고 수준';
    reorderAlerts: '재주문 알림';
    vendorManagement: '공급업체 관리';
  };
}

// CMMS 연동 서비스
class CMMSIntegrationService {
  private cmmcClient: CMMSAPIClient;
  private bemsData: BEMSDataService;
  private eventBus: EventBus;

  constructor(config: CMMSIntegrationConfig) {
    this.cmmcClient = new CMMSAPIClient(config.apiConfig);
    this.setupEventHandlers();
  }

  private setupEventHandlers(): void {
    // FDD 알람 → 자동 작업 지시서 생성
    this.eventBus.on('fdd:fault_detected', async (fault: FDDFault) => {
      if (fault.severity === 'high' || fault.severity === 'critical') {
        await this.createWorkOrder(fault);
      }
    });

    // 예방 유지보수 일정 동기화
    this.eventBus.on('maintenance:pm_due', async (pmSchedule: PMSchedule) => {
      await this.syncPreventiveMaintenance(pmSchedule);
    });
  }

  async createWorkOrder(fault: FDDFault): Promise<WorkOrderResponse> {
    // 기존 작업 지시서 확인
    const existingWO = await this.findExistingWorkOrder(fault.equipmentId, fault.faultCode);

    if (existingWO) {
      // 기존 WO에 이력 추가
      return this.updateWorkOrder(existingWO.id, fault);
    }

    // 새 작업 지시서 생성
    const workOrderData: CreateWorkOrderRequest = {
      type: 'corrective',
      priority: this.mapSeverityToPriority(fault.severity),
      assetId: fault.equipmentId,
      description: fault.description,
      symptomCode: fault.faultCode,
      reportedBy: 'BEMS-FDD',
      reportedDate: fault.detectedAt,
      estimatedDuration: fault.estimatedRepairTime,
      requiredSkills: fault.requiredSkills,
      recommendedActions: fault.recommendedActions,
      relatedParts: fault.possibleParts,
      bemsReference: {
        faultId: fault.id,
        detectionMethod: fault.detectionMethod,
        confidence: fault.confidence,
        energyImpact: fault.energyImpact
      }
    };

    const response = await this.cmmcClient.createWorkOrder(workOrderData);

    // BEMS에 WO 참조 저장
    await this.bemsData.linkWorkOrder(fault.id, response.workOrderId);

    return response;
  }

  async syncAssetData(): Promise<AssetSyncResult> {
    // CMMS에서 자산 목록 조회
    const cmmsAssets = await this.cmmcClient.getAssets();

    // BEMS 설비와 매칭
    const syncResults: AssetSyncItem[] = [];

    for (const cmmsAsset of cmmsAssets) {
      const bemsEquipment = await this.bemsData.findEquipmentByExternalId(cmmsAsset.id);

      if (bemsEquipment) {
        // 기존 설비 업데이트
        const updated = await this.updateEquipmentFromCMMS(bemsEquipment, cmmsAsset);
        syncResults.push({ assetId: cmmsAsset.id, action: 'updated', success: updated });
      } else {
        // 새 설비 → BEMS에 생성 또는 무시
        if (this.shouldImportAsset(cmmsAsset)) {
          const created = await this.createEquipmentFromCMMS(cmmsAsset);
          syncResults.push({ assetId: cmmsAsset.id, action: 'created', success: created });
        }
      }
    }

    return {
      totalAssets: cmmsAssets.length,
      synced: syncResults.filter(r => r.success).length,
      failed: syncResults.filter(r => !r.success).length,
      details: syncResults
    };
  }

  async reportEquipmentRuntime(): Promise<void> {
    // 설비 운전 시간 CMMS에 보고
    const equipment = await this.bemsData.getAllEquipment();

    for (const eq of equipment) {
      if (eq.cmmsAssetId) {
        const runtime = await this.bemsData.getEquipmentRuntime(eq.id, 'day');

        await this.cmmcClient.updateMeterReading({
          assetId: eq.cmmsAssetId,
          meterType: 'runtime_hours',
          reading: runtime.totalHours,
          readingDate: new Date()
        });

        // 시작 횟수도 보고
        await this.cmmcClient.updateMeterReading({
          assetId: eq.cmmsAssetId,
          meterType: 'start_count',
          reading: runtime.startCount,
          readingDate: new Date()
        });
      }
    }
  }
}

// ERP 연동 서비스
class ERPIntegrationService {
  private erpClient: ERPAPIClient;
  private bemsData: BEMSDataService;

  constructor(config: ERPIntegrationConfig) {
    this.erpClient = new ERPAPIClient(config);
  }

  async syncEnergyCosts(): Promise<void> {
    // 월별 에너지 비용 ERP에 전송
    const energyCosts = await this.bemsData.getMonthlyEnergyCosts();

    for (const cost of energyCosts) {
      await this.erpClient.postJournalEntry({
        documentDate: cost.periodEnd,
        postingDate: new Date(),
        reference: `BEMS-ENERGY-${cost.period}`,
        lineItems: [
          {
            accountCode: this.config.electricityAccountCode,
            amount: cost.electricityCost,
            costCenter: cost.costCenter,
            description: `전기 에너지 비용 ${cost.period}`
          },
          {
            accountCode: this.config.gasAccountCode,
            amount: cost.gasCost,
            costCenter: cost.costCenter,
            description: `가스 에너지 비용 ${cost.period}`
          },
          {
            accountCode: this.config.waterAccountCode,
            amount: cost.waterCost,
            costCenter: cost.costCenter,
            description: `수도 비용 ${cost.period}`
          }
        ]
      });
    }
  }

  async allocateEnergyCosts(period: string): Promise<CostAllocationResult> {
    // 테넌트별/부서별 에너지 비용 배분
    const consumption = await this.bemsData.getTenantConsumption(period);
    const totalCost = await this.bemsData.getTotalEnergyCost(period);

    const allocations: CostAllocation[] = [];

    for (const tenant of consumption) {
      const allocation = {
        tenantId: tenant.id,
        tenantName: tenant.name,
        consumptionKwh: tenant.electricityKwh,
        consumptionShare: tenant.electricityKwh / consumption.reduce((s, t) => s + t.electricityKwh, 0),
        allocatedCost: totalCost.electricity * tenant.electricityKwh /
                       consumption.reduce((s, t) => s + t.electricityKwh, 0),
        commonAreaShare: totalCost.commonArea * tenant.area / consumption.reduce((s, t) => s + t.area, 0)
      };

      allocations.push(allocation);

      // ERP에 청구서 생성
      await this.erpClient.createInvoice({
        customerId: tenant.erpCustomerId,
        invoiceDate: new Date(),
        dueDate: this.calculateDueDate(),
        lineItems: [
          {
            description: `전기 에너지 사용료 (${period})`,
            quantity: tenant.electricityKwh,
            unitPrice: totalCost.electricity / consumption.reduce((s, t) => s + t.electricityKwh, 0) / tenant.electricityKwh,
            amount: allocation.allocatedCost
          },
          {
            description: `공용부 에너지 분담금 (${period})`,
            quantity: tenant.area,
            unitPrice: allocation.commonAreaShare / tenant.area,
            amount: allocation.commonAreaShare
          }
        ],
        reference: `BEMS-BILL-${tenant.id}-${period}`
      });
    }

    return {
      period,
      totalCost,
      allocations,
      timestamp: new Date()
    };
  }
}
```

---

## 6.4 장 요약

### 시스템 통합 핵심 영역

| 통합 영역 | 프로토콜/방식 | 주요 기능 |
|----------|--------------|----------|
| 스마트 그리드 | OpenADR, IEEE 2030.5 | 수요 응답, 그리드 서비스 |
| 태양광 발전 | Modbus, SunSpec | 발전 모니터링, 예측 |
| ESS | Modbus, CAN | 충방전 최적화, 피크 저감 |
| CMMS | REST API | 작업 지시서, 자산 관리 |
| ERP | REST API, EDI | 비용 전송, 청구 |

### 통합 효과

- **수요 응답**: 피크 수요 15-30% 저감, 연간 인센티브 수익
- **재생에너지**: 자가 소비 극대화, 탄소 배출 저감
- **ESS**: 피크 저감, 에너지 비용 최적화
- **CMMS**: 유지보수 효율화, 설비 가용성 향상
- **ERP**: 에너지 비용 투명성, 자동화된 청구

### 다음 장 미리보기

제7장에서는 WIA-BEMS 보안 프레임워크에 대해 다룹니다. IT/OT 보안 아키텍처, 접근 제어, 데이터 보호, 사고 대응 절차를 학습합니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
