# 제5장: 제어 프로토콜 및 최적화 (Phase 3)

## 서론

WIA-BEMS Phase 3는 빌딩 에너지 관리의 핵심인 지능형 제어와 최적화를 다룹니다. 본 장에서는 HVAC 시스템의 표준 제어 시퀀스, 에너지 최적화 알고리즘, 모델 예측 제어(MPC), 그리고 고장 감지 및 진단(FDD) 방법론을 상세히 설명합니다.

---

## 5.1 HVAC 제어 시퀀스

### 5.1.1 AHU 제어 시퀀스

#### ASHRAE Guideline 36 기반 제어

```typescript
// AHU 제어 시퀀스 구현
interface AHUControlSequence {
  operatingModes: {
    heating: '난방 모드';
    cooling: '냉방 모드';
    deadband: '불감대';
    economizer: '외기 냉방';
    ventilation: '환기 전용';
    warmup: '예열';
    cooldown: '예냉';
    setback: '야간 설정값 후퇴';
    occupied: '재실';
    unoccupied: '비재실';
  };

  controlLoops: {
    supplyAirTemperature: '급기 온도 제어';
    staticPressure: '정압 제어';
    outdoorAir: '외기량 제어';
    economizer: '외기 냉방 제어';
    humidification: '가습 제어';
    dehumidification: '제습 제어';
  };
}

// AHU 제어기 구현
class AHUController {
  private state: AHUState;
  private config: AHUConfig;
  private pid: {
    sat: PIDController;
    staticPressure: PIDController;
    humidity: PIDController;
  };

  constructor(config: AHUConfig) {
    this.config = config;
    this.state = this.initializeState();
    this.pid = {
      sat: new PIDController(config.satPID),
      staticPressure: new PIDController(config.staticPressurePID),
      humidity: new PIDController(config.humidityPID)
    };
  }

  execute(inputs: AHUInputs): AHUOutputs {
    // 1. 운전 모드 결정
    const operatingMode = this.determineOperatingMode(inputs);

    // 2. 급기 온도 설정값 계산
    const satSetpoint = this.calculateSATSetpoint(inputs, operatingMode);

    // 3. 급기 온도 제어
    const satControl = this.controlSupplyAirTemperature(inputs, satSetpoint);

    // 4. 정압 제어
    const staticPressureControl = this.controlStaticPressure(inputs);

    // 5. 외기량 제어
    const outdoorAirControl = this.controlOutdoorAir(inputs, operatingMode);

    // 6. 이코노마이저 제어
    const economizerControl = this.controlEconomizer(inputs, outdoorAirControl);

    // 7. 가습/제습 제어
    const humidityControl = this.controlHumidity(inputs);

    // 8. 안전 제어 적용
    const safetyOverrides = this.applySafetyControls(inputs);

    return this.combineOutputs({
      operatingMode,
      satControl,
      staticPressureControl,
      economizerControl,
      humidityControl,
      safetyOverrides
    });
  }

  private determineOperatingMode(inputs: AHUInputs): OperatingMode {
    const { schedule, zoneRequests, outdoorAirTemperature } = inputs;

    // 스케줄 기반 재실/비재실 판단
    if (!schedule.isOccupied) {
      // 비재실 시 야간 설정값 후퇴 또는 정지
      if (this.shouldRunSetback(inputs)) {
        return 'setback';
      }
      return 'unoccupied';
    }

    // 예열/예냉 판단
    if (this.isInWarmupPeriod(inputs)) {
      return 'warmup';
    }
    if (this.isInCooldownPeriod(inputs)) {
      return 'cooldown';
    }

    // 냉난방 요구 분석
    const coolingRequests = zoneRequests.filter(r => r.type === 'cooling').length;
    const heatingRequests = zoneRequests.filter(r => r.type === 'heating').length;

    // 외기 냉방 가능 여부
    const economizerEnabled = this.isEconomizerEnabled(inputs);

    if (coolingRequests > heatingRequests && economizerEnabled) {
      return 'economizer';
    } else if (coolingRequests > heatingRequests) {
      return 'cooling';
    } else if (heatingRequests > coolingRequests) {
      return 'heating';
    }

    return 'deadband';
  }

  private calculateSATSetpoint(inputs: AHUInputs, mode: OperatingMode): number {
    const { zoneRequests, config } = inputs;

    switch (mode) {
      case 'cooling':
      case 'economizer':
        // 냉방 시: 냉방 요청에 따른 급기 온도 리셋
        return this.calculateCoolingSATSetpoint(zoneRequests);

      case 'heating':
        // 난방 시: 난방 요청에 따른 급기 온도 리셋
        return this.calculateHeatingSATSetpoint(zoneRequests);

      case 'warmup':
        // 예열: 최대 난방
        return config.maxSAT;

      case 'cooldown':
        // 예냉: 최소 냉방
        return config.minSAT;

      default:
        return config.defaultSAT;
    }
  }

  private calculateCoolingSATSetpoint(zoneRequests: ZoneRequest[]): number {
    // Trim & Respond 로직
    // 가장 큰 냉방 요구에 따라 SAT 조절

    const maxCoolingRequest = Math.max(
      ...zoneRequests.filter(r => r.type === 'cooling').map(r => r.intensity),
      0
    );

    // 냉방 요구가 클수록 낮은 SAT
    // 요구가 작으면 SAT 상승 (에너지 절약)
    const satRange = this.config.maxSAT - this.config.minSAT;

    if (maxCoolingRequest > 0.8) {
      // 높은 냉방 요구: SAT 하향
      this.state.satTrimAmount = Math.max(
        this.state.satTrimAmount - this.config.satTrimRate,
        -satRange / 2
      );
    } else if (maxCoolingRequest < 0.2) {
      // 낮은 냉방 요구: SAT 상향
      this.state.satTrimAmount = Math.min(
        this.state.satTrimAmount + this.config.satTrimRate,
        satRange / 2
      );
    }

    return this.config.defaultSAT + this.state.satTrimAmount;
  }

  private controlSupplyAirTemperature(
    inputs: AHUInputs,
    setpoint: number
  ): SATControlOutput {
    const { supplyAirTemperature, returnAirTemperature, mixedAirTemperature } = inputs;

    // PID 제어
    const pidOutput = this.pid.sat.calculate(setpoint, supplyAirTemperature);

    // 시퀀스 제어 (냉각 → 불감대 → 가열)
    let coolingOutput = 0;
    let heatingOutput = 0;

    if (pidOutput > 0) {
      // 냉각 필요
      coolingOutput = Math.min(pidOutput, 100);
    } else if (pidOutput < 0) {
      // 가열 필요
      heatingOutput = Math.min(-pidOutput, 100);
    }

    return {
      setpoint,
      actual: supplyAirTemperature,
      error: setpoint - supplyAirTemperature,
      coolingValvePosition: coolingOutput,
      heatingValvePosition: heatingOutput,
      pidOutput
    };
  }

  private controlStaticPressure(inputs: AHUInputs): StaticPressureOutput {
    const { staticPressure, vavPositions } = inputs;

    // 정압 리셋 로직
    // VAV 댐퍼 위치에 따라 정압 설정값 조절
    const setpoint = this.calculateStaticPressureSetpoint(vavPositions);

    // PID 제어로 팬 속도 조절
    const pidOutput = this.pid.staticPressure.calculate(setpoint, staticPressure);

    // 팬 속도 제한 적용
    const fanSpeed = Math.max(
      this.config.minFanSpeed,
      Math.min(pidOutput, 100)
    );

    return {
      setpoint,
      actual: staticPressure,
      fanSpeed,
      resetMode: this.state.staticPressureResetMode
    };
  }

  private calculateStaticPressureSetpoint(vavPositions: number[]): number {
    // Trim & Respond 정압 리셋
    // 가장 많이 열린 VAV 기준

    const maxVavPosition = Math.max(...vavPositions);
    const avgVavPosition = vavPositions.reduce((a, b) => a + b, 0) / vavPositions.length;

    // 목표: VAV 최대 개도 90% 이하 유지
    if (maxVavPosition > 95) {
      // VAV가 거의 다 열림 → 정압 증가
      this.state.staticPressureSetpoint = Math.min(
        this.state.staticPressureSetpoint + this.config.spTrimRate,
        this.config.maxStaticPressure
      );
      this.state.staticPressureResetMode = 'increasing';
    } else if (maxVavPosition < 70 && avgVavPosition < 50) {
      // VAV가 대부분 닫힘 → 정압 감소
      this.state.staticPressureSetpoint = Math.max(
        this.state.staticPressureSetpoint - this.config.spTrimRate,
        this.config.minStaticPressure
      );
      this.state.staticPressureResetMode = 'decreasing';
    } else {
      this.state.staticPressureResetMode = 'stable';
    }

    return this.state.staticPressureSetpoint;
  }

  private controlOutdoorAir(
    inputs: AHUInputs,
    mode: OperatingMode
  ): OutdoorAirOutput {
    const { occupancy, co2Levels, supplyAirflow } = inputs;

    // 최소 외기량 계산 (ASHRAE 62.1 기반)
    let minOutdoorAirflow: number;

    if (this.config.useCO2DCV) {
      // CO2 기반 수요 제어 환기
      minOutdoorAirflow = this.calculateCO2BasedVentilation(occupancy, co2Levels);
    } else {
      // 인원수 기반 환기
      minOutdoorAirflow = this.calculateOccupancyBasedVentilation(occupancy);
    }

    // 최소 외기 비율 계산
    const minOAFraction = Math.min(
      minOutdoorAirflow / supplyAirflow,
      1.0
    );

    return {
      minOutdoorAirflow,
      minOAFraction,
      method: this.config.useCO2DCV ? 'co2_dcv' : 'occupancy'
    };
  }

  private calculateCO2BasedVentilation(
    occupancy: OccupancyData,
    co2Levels: CO2Data
  ): number {
    // 구역별 CO2 레벨에 따른 환기량 조절
    const zoneCO2 = co2Levels.zoneAverage;
    const outdoorCO2 = co2Levels.outdoor || 400;
    const setpoint = this.config.co2Setpoint || 1000;

    // 비례 제어
    const co2Error = zoneCO2 - setpoint;
    const co2Range = setpoint - outdoorCO2;

    // 기본 환기량 + CO2 기반 추가 환기
    const baseVentilation = occupancy.count * this.config.cfmPerPerson;
    const additionalVentilation = Math.max(0, co2Error / co2Range) * baseVentilation;

    return baseVentilation + additionalVentilation;
  }

  private controlEconomizer(
    inputs: AHUInputs,
    oaControl: OutdoorAirOutput
  ): EconomizerOutput {
    const {
      outdoorAirTemperature,
      outdoorAirEnthalpy,
      returnAirTemperature,
      returnAirEnthalpy,
      supplyAirTemperatureSetpoint
    } = inputs;

    // 이코노마이저 활성화 조건 확인
    const enabled = this.isEconomizerEnabled(inputs);

    if (!enabled) {
      return {
        enabled: false,
        outdoorAirDamper: oaControl.minOAFraction * 100,
        returnAirDamper: 100 - oaControl.minOAFraction * 100,
        reliefDamper: oaControl.minOAFraction * 100,
        mode: 'minimum'
      };
    }

    // 이코노마이저 모드 결정
    const oaDamperPosition = this.calculateEconomizerDamperPosition(
      outdoorAirTemperature,
      supplyAirTemperatureSetpoint,
      oaControl.minOAFraction
    );

    return {
      enabled: true,
      outdoorAirDamper: oaDamperPosition,
      returnAirDamper: 100 - oaDamperPosition,
      reliefDamper: oaDamperPosition,
      mode: oaDamperPosition >= 90 ? 'full' : 'modulating'
    };
  }

  private isEconomizerEnabled(inputs: AHUInputs): boolean {
    const { outdoorAirTemperature, outdoorAirEnthalpy, returnAirTemperature, returnAirEnthalpy } = inputs;

    // 건구 온도 이코노마이저 (기후 구역에 따라)
    if (this.config.economizerType === 'dry_bulb') {
      const highLimit = this.config.economizerHighLimit || 21; // °C
      return outdoorAirTemperature < highLimit &&
             outdoorAirTemperature < returnAirTemperature - 1;
    }

    // 엔탈피 이코노마이저
    if (this.config.economizerType === 'enthalpy') {
      return outdoorAirEnthalpy < returnAirEnthalpy &&
             outdoorAirTemperature < returnAirTemperature;
    }

    // 비교 엔탈피 이코노마이저
    if (this.config.economizerType === 'differential_enthalpy') {
      return outdoorAirEnthalpy < returnAirEnthalpy - 2; // 2 kJ/kg 마진
    }

    return false;
  }

  private applySafetyControls(inputs: AHUInputs): SafetyOverrides {
    const overrides: SafetyOverrides = {
      freezeProtection: false,
      highStaticPressure: false,
      lowStaticPressure: false,
      highTemperature: false,
      smokeDetected: false
    };

    // 동파 방지
    if (inputs.mixedAirTemperature < this.config.freezeProtectionTemp) {
      overrides.freezeProtection = true;
      // 외기 댐퍼 최소, 가열 밸브 100%
    }

    // 고정압 보호
    if (inputs.staticPressure > this.config.maxStaticPressure * 1.1) {
      overrides.highStaticPressure = true;
      // 팬 속도 감소
    }

    // 화재 감지
    if (inputs.smokeDetector) {
      overrides.smokeDetected = true;
      // 팬 정지, 댐퍼 폐쇄
    }

    return overrides;
  }
}

// PID 제어기
class PIDController {
  private integral: number = 0;
  private previousError: number = 0;
  private lastTime: number = Date.now();

  constructor(private config: PIDConfig) {}

  calculate(setpoint: number, processVariable: number): number {
    const now = Date.now();
    const dt = (now - this.lastTime) / 1000; // 초 단위
    this.lastTime = now;

    const error = setpoint - processVariable;

    // 비례 항
    const P = this.config.kp * error;

    // 적분 항 (Anti-windup)
    this.integral += error * dt;
    this.integral = Math.max(
      -this.config.integralLimit,
      Math.min(this.integral, this.config.integralLimit)
    );
    const I = this.config.ki * this.integral;

    // 미분 항
    const derivative = (error - this.previousError) / dt;
    const D = this.config.kd * derivative;
    this.previousError = error;

    // 출력 제한
    const output = P + I + D;
    return Math.max(
      this.config.outputMin,
      Math.min(output, this.config.outputMax)
    );
  }

  reset() {
    this.integral = 0;
    this.previousError = 0;
  }
}
```

### 5.1.2 VAV 박스 제어

```typescript
// VAV 박스 제어 시퀀스
class VAVBoxController {
  private state: VAVState;
  private config: VAVConfig;
  private pidCooling: PIDController;
  private pidHeating: PIDController;

  constructor(config: VAVConfig) {
    this.config = config;
    this.pidCooling = new PIDController(config.coolingPID);
    this.pidHeating = new PIDController(config.heatingPID);
  }

  execute(inputs: VAVInputs): VAVOutputs {
    const { zoneTemperature, occupancyStatus, schedule, co2Level } = inputs;

    // 1. 설정값 결정
    const setpoints = this.determineSetpoints(inputs);

    // 2. 제어 모드 결정
    const mode = this.determineMode(zoneTemperature, setpoints);

    // 3. 풍량 제어
    const airflowControl = this.controlAirflow(inputs, mode, setpoints);

    // 4. 리히트 제어 (해당 시)
    const reheatControl = this.controlReheat(inputs, mode, setpoints);

    // 5. 환기 요구 확인
    const ventilationRequirement = this.calculateVentilationRequirement(inputs);

    // 6. 최종 출력 계산
    return this.calculateOutputs(airflowControl, reheatControl, ventilationRequirement);
  }

  private determineSetpoints(inputs: VAVInputs): VAVSetpoints {
    const { schedule, occupancyStatus, hasLocalSetpointAdjustment, localAdjustment } = inputs;

    let coolingSetpoint = this.config.defaultCoolingSetpoint;
    let heatingSetpoint = this.config.defaultHeatingSetpoint;

    // 재실 상태에 따른 설정값
    if (!schedule.isOccupied && !occupancyStatus.detected) {
      // 비재실 시 설정값 후퇴
      coolingSetpoint += this.config.unoccupiedSetback;
      heatingSetpoint -= this.config.unoccupiedSetback;
    }

    // 개인 설정값 조정 (허용된 범위 내)
    if (hasLocalSetpointAdjustment && localAdjustment) {
      const adjustmentLimit = this.config.localAdjustmentLimit || 2;
      const adjustment = Math.max(
        -adjustmentLimit,
        Math.min(localAdjustment, adjustmentLimit)
      );
      coolingSetpoint += adjustment;
      heatingSetpoint += adjustment;
    }

    return {
      coolingSetpoint,
      heatingSetpoint,
      deadband: this.config.deadband || 1
    };
  }

  private determineMode(
    zoneTemp: number,
    setpoints: VAVSetpoints
  ): VAVMode {
    if (zoneTemp > setpoints.coolingSetpoint) {
      return 'cooling';
    } else if (zoneTemp < setpoints.heatingSetpoint) {
      return 'heating';
    }
    return 'deadband';
  }

  private controlAirflow(
    inputs: VAVInputs,
    mode: VAVMode,
    setpoints: VAVSetpoints
  ): AirflowControl {
    const { zoneTemperature, dischargeAirTemperature } = inputs;

    let airflowSetpoint: number;
    let damperPosition: number;

    switch (mode) {
      case 'cooling':
        // 냉방: 풍량 증가
        const coolingError = zoneTemperature - setpoints.coolingSetpoint;
        const coolingOutput = this.pidCooling.calculate(
          setpoints.coolingSetpoint,
          zoneTemperature
        );

        // PID 출력을 풍량으로 변환
        airflowSetpoint = this.config.minAirflow +
          (coolingOutput / 100) * (this.config.maxAirflow - this.config.minAirflow);
        break;

      case 'heating':
        // 난방: 최소 풍량 + 리히트
        airflowSetpoint = this.config.minAirflow;
        break;

      case 'deadband':
        // 불감대: 최소 풍량
        airflowSetpoint = this.config.minAirflow;
        break;
    }

    // 풍량을 댐퍼 위치로 변환
    damperPosition = this.airflowToDamperPosition(airflowSetpoint);

    return {
      setpoint: airflowSetpoint,
      damperPosition,
      mode
    };
  }

  private controlReheat(
    inputs: VAVInputs,
    mode: VAVMode,
    setpoints: VAVSetpoints
  ): ReheatControl {
    if (!this.config.hasReheat) {
      return { enabled: false, output: 0 };
    }

    if (mode !== 'heating') {
      return { enabled: false, output: 0 };
    }

    // 난방 모드에서 리히트 제어
    const heatingOutput = this.pidHeating.calculate(
      setpoints.heatingSetpoint,
      inputs.zoneTemperature
    );

    // 리히트 유형에 따른 출력
    if (this.config.reheatType === 'hot_water') {
      return {
        enabled: true,
        output: Math.max(0, Math.min(heatingOutput, 100)),
        type: 'hot_water'
      };
    } else if (this.config.reheatType === 'electric') {
      // 전기 리히트: 단계 제어
      const stage = Math.ceil(heatingOutput / 33); // 3단계
      return {
        enabled: true,
        output: heatingOutput,
        stage: Math.min(stage, 3),
        type: 'electric'
      };
    }

    return { enabled: false, output: 0 };
  }

  private calculateVentilationRequirement(inputs: VAVInputs): number {
    const { occupancyStatus, co2Level, zoneArea } = inputs;

    // ASHRAE 62.1 기반 최소 환기량
    const peopleOutdoorAir = occupancyStatus.count * this.config.cfmPerPerson;
    const areaOutdoorAir = zoneArea * this.config.cfmPerSqft;

    let minOutdoorAir = peopleOutdoorAir + areaOutdoorAir;

    // CO2 기반 조정
    if (co2Level && this.config.useCO2Reset) {
      const co2Factor = Math.max(1, (co2Level - 400) / 600); // 400~1000 ppm
      minOutdoorAir *= co2Factor;
    }

    return minOutdoorAir;
  }

  private airflowToDamperPosition(airflow: number): number {
    // 풍량-댐퍼 특성 곡선 (비선형)
    const normalizedFlow = (airflow - this.config.minAirflow) /
      (this.config.maxAirflow - this.config.minAirflow);

    // 제곱근 특성 (일반적인 VAV 특성)
    return Math.sqrt(normalizedFlow) * 100;
  }
}
```

---

## 5.2 에너지 최적화 알고리즘

### 5.2.1 최적 기동/정지

```typescript
// 최적 기동/정지 알고리즘
class OptimalStartStopController {
  private learningData: StartStopLearningData;
  private model: ThermalResponseModel;

  constructor(config: OptimalStartConfig) {
    this.learningData = new StartStopLearningData();
    this.model = new ThermalResponseModel(config);
  }

  calculateOptimalStartTime(
    targetTime: Date,
    targetTemperature: number,
    currentConditions: BuildingConditions
  ): OptimalStartResult {
    const {
      indoorTemperature,
      outdoorTemperature,
      thermalMass,
      hvacCapacity
    } = currentConditions;

    // 필요 온도 변화량
    const deltaT = Math.abs(targetTemperature - indoorTemperature);

    // 열적 모델 기반 예열/예냉 시간 계산
    const estimatedTime = this.model.estimateWarmupCooldownTime({
      deltaT,
      outdoorTemperature,
      thermalMass,
      hvacCapacity,
      mode: targetTemperature > indoorTemperature ? 'heating' : 'cooling'
    });

    // 학습 데이터 기반 보정
    const adjustedTime = this.applyLearningCorrection(estimatedTime, currentConditions);

    // 최적 기동 시간
    const optimalStartTime = new Date(targetTime.getTime() - adjustedTime * 60 * 1000);

    return {
      optimalStartTime,
      estimatedWarmupTime: adjustedTime,
      currentTemperature: indoorTemperature,
      targetTemperature,
      outdoorTemperature,
      confidence: this.model.getConfidence()
    };
  }

  calculateOptimalStopTime(
    occupancyEndTime: Date,
    currentConditions: BuildingConditions,
    comfortConstraints: ComfortConstraints
  ): OptimalStopResult {
    const {
      indoorTemperature,
      outdoorTemperature,
      thermalMass
    } = currentConditions;

    // 자연 온도 drift 예측
    const driftRate = this.model.predictTemperatureDrift({
      indoorTemperature,
      outdoorTemperature,
      thermalMass
    });

    // 쾌적 범위 이탈 시간 계산
    const maxDrift = comfortConstraints.allowedDrift || 2; // °C
    const timeToExit = maxDrift / Math.abs(driftRate);

    // 최적 정지 시간 (재실 종료 전)
    const optimalStopTime = new Date(
      occupancyEndTime.getTime() - timeToExit * 60 * 1000
    );

    return {
      optimalStopTime,
      estimatedDriftRate: driftRate,
      estimatedSavings: this.estimateEnergySavings(timeToExit, currentConditions),
      confidence: this.model.getConfidence()
    };
  }

  updateLearningData(actual: ActualStartStopData): void {
    // 실제 결과로 모델 업데이트
    this.learningData.addRecord({
      date: actual.date,
      predictedTime: actual.predictedWarmupTime,
      actualTime: actual.actualWarmupTime,
      outdoorTemperature: actual.outdoorTemperature,
      startTemperature: actual.startTemperature,
      endTemperature: actual.endTemperature,
      success: actual.reachedSetpointOnTime
    });

    // 모델 재학습
    if (this.learningData.getRecordCount() >= 10) {
      this.model.retrain(this.learningData.getRecords());
    }
  }

  private applyLearningCorrection(
    estimatedTime: number,
    conditions: BuildingConditions
  ): number {
    const recentRecords = this.learningData.getRecentRecords(30);

    if (recentRecords.length < 5) {
      // 충분한 데이터 없음: 안전 마진 추가
      return estimatedTime * 1.2;
    }

    // 유사 조건의 과거 기록 필터링
    const similarRecords = recentRecords.filter(r =>
      Math.abs(r.outdoorTemperature - conditions.outdoorTemperature) < 5
    );

    if (similarRecords.length < 3) {
      return estimatedTime * 1.1;
    }

    // 예측 오차 분석
    const avgError = similarRecords.reduce((sum, r) =>
      sum + (r.actualTime - r.predictedTime), 0) / similarRecords.length;

    // 보정 적용
    return estimatedTime + avgError;
  }
}

// 열적 응답 모델
class ThermalResponseModel {
  private coefficients: ThermalCoefficients;

  constructor(config: ThermalModelConfig) {
    this.coefficients = this.initializeCoefficients(config);
  }

  estimateWarmupCooldownTime(params: WarmupCooldownParams): number {
    const { deltaT, outdoorTemperature, thermalMass, hvacCapacity, mode } = params;

    // 1차 RC 모델 기반
    // T(t) = T_outdoor + (T_initial - T_outdoor) * e^(-t/τ) + Q/(UA) * (1 - e^(-t/τ))

    const tau = this.calculateTimeConstant(thermalMass);
    const UA = this.coefficients.overallUA;
    const Q = mode === 'heating' ? hvacCapacity : -hvacCapacity;

    // 목표 온도 도달 시간 계산 (뉴턴-랩슨 또는 이분법)
    const timeToReach = this.solveForTime(deltaT, tau, Q, UA);

    return timeToReach;
  }

  predictTemperatureDrift(params: DriftParams): number {
    const { indoorTemperature, outdoorTemperature, thermalMass } = params;

    // 자연 온도 변화율 (°C/min)
    const tau = this.calculateTimeConstant(thermalMass);
    const driftRate = (outdoorTemperature - indoorTemperature) / tau;

    return driftRate;
  }

  retrain(records: LearningRecord[]): void {
    // 선형 회귀로 계수 업데이트
    // y = a0 + a1*deltaT + a2*outdoorT + a3*thermalMass

    const X = records.map(r => [
      1,
      r.endTemperature - r.startTemperature,
      r.outdoorTemperature,
      r.thermalMass || 1
    ]);
    const y = records.map(r => r.actualTime);

    const newCoeffs = this.linearRegression(X, y);
    this.coefficients = {
      ...this.coefficients,
      a0: newCoeffs[0],
      a1: newCoeffs[1],
      a2: newCoeffs[2],
      a3: newCoeffs[3]
    };
  }

  getConfidence(): number {
    // 모델 정확도 기반 신뢰도
    return this.coefficients.r2 || 0.8;
  }

  private calculateTimeConstant(thermalMass: number): number {
    // RC 시정수 계산
    const R = 1 / this.coefficients.overallUA;
    const C = thermalMass * this.coefficients.specificHeat;
    return R * C;
  }

  private linearRegression(X: number[][], y: number[]): number[] {
    // 최소자승법으로 계수 추정
    // 간단한 구현 (실제로는 라이브러리 사용)
    // β = (X'X)^(-1) X'y

    // 구현 생략...
    return [0, 1, 0.1, 0.01]; // 예시 값
  }
}
```

### 5.2.2 모델 예측 제어 (MPC)

```typescript
// 모델 예측 제어 구현
class MPCController {
  private model: BuildingThermalModel;
  private optimizer: ConvexOptimizer;
  private config: MPCConfig;

  constructor(config: MPCConfig) {
    this.config = config;
    this.model = new BuildingThermalModel(config.modelParams);
    this.optimizer = new ConvexOptimizer();
  }

  computeOptimalControl(
    currentState: BuildingState,
    forecasts: Forecasts,
    constraints: MPCConstraints
  ): MPCResult {
    const horizon = this.config.predictionHorizon;
    const dt = this.config.controlInterval;

    // 1. 상태 공간 모델 업데이트
    this.model.updateState(currentState);

    // 2. 최적화 문제 구성
    const problem = this.formulateOptimizationProblem(
      currentState,
      forecasts,
      constraints,
      horizon
    );

    // 3. 최적화 실행
    const solution = this.optimizer.solve(problem);

    // 4. 첫 번째 제어 입력 추출 (Receding Horizon)
    const optimalControl = this.extractFirstControl(solution);

    // 5. 예측 궤적 계산
    const predictedTrajectory = this.computePredictedTrajectory(solution);

    return {
      optimalControl,
      predictedTrajectory,
      objectiveValue: solution.objectiveValue,
      solveTime: solution.solveTime,
      status: solution.status
    };
  }

  private formulateOptimizationProblem(
    state: BuildingState,
    forecasts: Forecasts,
    constraints: MPCConstraints,
    horizon: number
  ): OptimizationProblem {
    // 결정 변수: u[k] = [Q_cooling, Q_heating, m_supply] for k = 0..N-1
    // 상태 변수: x[k] = [T_zone, T_wall, T_floor] for k = 0..N

    const N = horizon;
    const nx = this.model.stateSize;  // 상태 차원
    const nu = this.model.inputSize;  // 입력 차원

    // 목적 함수: min Σ (energy_cost + comfort_penalty)
    const objectiveFunction = (u: number[][], x: number[][]): number => {
      let cost = 0;

      for (let k = 0; k < N; k++) {
        // 에너지 비용
        const energyPrice = forecasts.electricityPrice[k];
        const coolingPower = u[k][0];
        const heatingPower = u[k][1];
        cost += energyPrice * (coolingPower + heatingPower) * this.config.controlInterval;

        // 쾌적성 페널티
        const zoneTemp = x[k][0];
        const setpoint = forecasts.temperatureSetpoint[k];
        const deviation = zoneTemp - setpoint;
        cost += this.config.comfortWeight * deviation * deviation;

        // 피크 수요 페널티
        const totalPower = coolingPower + heatingPower;
        if (totalPower > constraints.peakDemandLimit) {
          cost += this.config.peakPenalty * (totalPower - constraints.peakDemandLimit);
        }
      }

      return cost;
    };

    // 동적 제약 조건: x[k+1] = A*x[k] + B*u[k] + E*d[k]
    const dynamicsConstraints = [];
    for (let k = 0; k < N; k++) {
      const disturbance = [
        forecasts.outdoorTemperature[k],
        forecasts.solarRadiation[k],
        forecasts.internalLoads[k]
      ];

      dynamicsConstraints.push({
        type: 'equality',
        evaluate: (u: number[][], x: number[][]) => {
          const xNext = this.model.predict(x[k], u[k], disturbance);
          return x[k + 1].map((val, i) => val - xNext[i]);
        }
      });
    }

    // 입력 제약 조건
    const inputConstraints = {
      coolingPower: { min: 0, max: constraints.maxCoolingPower },
      heatingPower: { min: 0, max: constraints.maxHeatingPower },
      supplyAirflow: { min: constraints.minAirflow, max: constraints.maxAirflow }
    };

    // 상태 제약 조건
    const stateConstraints = {
      zoneTemperature: {
        min: constraints.minTemperature,
        max: constraints.maxTemperature
      }
    };

    return {
      objectiveFunction,
      dynamicsConstraints,
      inputConstraints,
      stateConstraints,
      initialState: [state.zoneTemperature, state.wallTemperature, state.floorTemperature],
      horizon: N
    };
  }

  private extractFirstControl(solution: OptimizationSolution): ControlInput {
    const u0 = solution.controlSequence[0];

    return {
      coolingPower: u0[0],
      heatingPower: u0[1],
      supplyAirflow: u0[2],
      timestamp: new Date()
    };
  }
}

// 빌딩 열적 모델 (상태 공간)
class BuildingThermalModel {
  stateSize: number = 3;  // [T_zone, T_wall, T_floor]
  inputSize: number = 3;  // [Q_cooling, Q_heating, m_supply]

  private A: number[][];  // 상태 행렬
  private B: number[][];  // 입력 행렬
  private E: number[][];  // 외란 행렬

  constructor(params: ThermalModelParams) {
    this.initializeMatrices(params);
  }

  predict(
    state: number[],
    input: number[],
    disturbance: number[]
  ): number[] {
    // x[k+1] = A*x[k] + B*u[k] + E*d[k]
    const xNext: number[] = new Array(this.stateSize).fill(0);

    for (let i = 0; i < this.stateSize; i++) {
      // A*x
      for (let j = 0; j < this.stateSize; j++) {
        xNext[i] += this.A[i][j] * state[j];
      }
      // B*u
      for (let j = 0; j < this.inputSize; j++) {
        xNext[i] += this.B[i][j] * input[j];
      }
      // E*d
      for (let j = 0; j < disturbance.length; j++) {
        xNext[i] += this.E[i][j] * disturbance[j];
      }
    }

    return xNext;
  }

  private initializeMatrices(params: ThermalModelParams): void {
    const dt = params.samplingTime;
    const Czone = params.zoneThermalCapacity;
    const Cwall = params.wallThermalCapacity;
    const Cfloor = params.floorThermalCapacity;
    const Rzo = params.zoneToOutdoorResistance;
    const Rzw = params.zoneToWallResistance;
    const Rzf = params.zoneToFloorResistance;
    const Rwo = params.wallToOutdoorResistance;

    // 연속 시간 시스템을 이산화
    // 간략화된 RC 네트워크 모델

    this.A = [
      [1 - dt / (Czone * Rzo) - dt / (Czone * Rzw) - dt / (Czone * Rzf),
       dt / (Czone * Rzw), dt / (Czone * Rzf)],
      [dt / (Cwall * Rzw),
       1 - dt / (Cwall * Rzw) - dt / (Cwall * Rwo), 0],
      [dt / (Cfloor * Rzf), 0, 1 - dt / (Cfloor * Rzf)]
    ];

    this.B = [
      [-1 / Czone, 1 / Czone, params.supplyAirFactor / Czone],
      [0, 0, 0],
      [0, 0, 0]
    ];

    this.E = [
      [dt / (Czone * Rzo), params.solarGain / Czone, 1 / Czone],
      [dt / (Cwall * Rwo), params.wallSolarGain / Cwall, 0],
      [0, 0, 0]
    ];
  }

  updateState(state: BuildingState): void {
    // 실측값으로 상태 업데이트 (상태 추정)
  }
}
```

---

## 5.3 고장 감지 및 진단 (FDD)

### 5.3.1 규칙 기반 FDD

```typescript
// 규칙 기반 FDD 시스템
interface FDDRuleEngine {
  rules: FDDRule[];
  faultLibrary: FaultLibrary;
}

interface FDDRule {
  id: string;
  name: string;
  equipment: string[];
  conditions: RuleCondition[];
  fault: FaultDescription;
  severity: 'low' | 'medium' | 'high' | 'critical';
  energyImpact?: EnergyImpactEstimate;
}

// FDD 규칙 엔진 구현
class RuleBasedFDD {
  private rules: Map<string, FDDRule>;
  private activeFaults: Map<string, ActiveFault>;

  constructor() {
    this.rules = new Map();
    this.activeFaults = new Map();
    this.loadDefaultRules();
  }

  evaluate(data: EquipmentData): FDDResult[] {
    const results: FDDResult[] = [];

    for (const [ruleId, rule] of this.rules) {
      // 적용 가능한 설비인지 확인
      if (!rule.equipment.includes(data.equipmentType)) {
        continue;
      }

      // 규칙 조건 평가
      const evaluation = this.evaluateRule(rule, data);

      if (evaluation.faultDetected) {
        const fault = this.createFault(rule, evaluation, data);
        results.push(fault);

        // 활성 고장 목록 업데이트
        this.updateActiveFaults(fault);
      } else {
        // 이전에 활성화된 고장 해제 확인
        this.checkFaultResolution(ruleId, data.equipmentId);
      }
    }

    return results;
  }

  private evaluateRule(rule: FDDRule, data: EquipmentData): RuleEvaluation {
    const conditionResults: ConditionResult[] = [];
    let allConditionsMet = true;

    for (const condition of rule.conditions) {
      const result = this.evaluateCondition(condition, data);
      conditionResults.push(result);

      if (!result.met) {
        allConditionsMet = false;
      }
    }

    return {
      faultDetected: allConditionsMet,
      conditionResults,
      confidence: this.calculateConfidence(conditionResults)
    };
  }

  private evaluateCondition(
    condition: RuleCondition,
    data: EquipmentData
  ): ConditionResult {
    const value = this.getDataValue(data, condition.variable);

    if (value === undefined || value === null) {
      return { met: false, reason: 'Data not available', value: null };
    }

    let met = false;
    switch (condition.operator) {
      case '>':
        met = value > condition.threshold;
        break;
      case '<':
        met = value < condition.threshold;
        break;
      case '>=':
        met = value >= condition.threshold;
        break;
      case '<=':
        met = value <= condition.threshold;
        break;
      case '==':
        met = value === condition.threshold;
        break;
      case 'between':
        met = value >= condition.threshold && value <= condition.threshold2!;
        break;
      case 'outside':
        met = value < condition.threshold || value > condition.threshold2!;
        break;
    }

    return {
      met,
      value,
      threshold: condition.threshold,
      variable: condition.variable
    };
  }

  private loadDefaultRules(): void {
    // AHU 관련 규칙
    this.rules.set('AHU_ECONOMIZER_STUCK', {
      id: 'AHU_ECONOMIZER_STUCK',
      name: '이코노마이저 댐퍼 고착',
      equipment: ['AHU', 'RTU'],
      conditions: [
        { variable: 'economizer.enabled', operator: '==', threshold: true },
        { variable: 'outdoorAirDamper.position', operator: '==', threshold: 100, duration: 3600 },
        { variable: 'outdoorAirTemperature', operator: '>', threshold: 24 }
      ],
      fault: {
        code: 'AHU-ECO-001',
        description: '이코노마이저 댐퍼가 완전 개방 상태로 고착됨',
        possibleCauses: ['액추에이터 고장', '제어 신호 오류', '기계적 고착'],
        recommendedActions: ['댐퍼 액추에이터 점검', '제어 신호 확인', '수동 조작 시험']
      },
      severity: 'high',
      energyImpact: { annualCost: 5000000, wastagePercent: 15 }
    });

    this.rules.set('AHU_SIMULTANEOUS_HEATING_COOLING', {
      id: 'AHU_SIMULTANEOUS_HEATING_COOLING',
      name: '동시 냉난방',
      equipment: ['AHU', 'RTU'],
      conditions: [
        { variable: 'coolingValve.position', operator: '>', threshold: 20 },
        { variable: 'heatingValve.position', operator: '>', threshold: 20 }
      ],
      fault: {
        code: 'AHU-CTL-001',
        description: '냉방과 난방이 동시에 작동 중',
        possibleCauses: ['제어 시퀀스 오류', '데드밴드 설정 오류', '밸브 고착'],
        recommendedActions: ['제어 시퀀스 검토', '데드밴드 확인', '밸브 작동 확인']
      },
      severity: 'high',
      energyImpact: { annualCost: 8000000, wastagePercent: 20 }
    });

    this.rules.set('AHU_SUPPLY_FAN_LOW_EFFICIENCY', {
      id: 'AHU_SUPPLY_FAN_LOW_EFFICIENCY',
      name: '공급 팬 효율 저하',
      equipment: ['AHU'],
      conditions: [
        { variable: 'supplyFan.speed', operator: '>', threshold: 80 },
        { variable: 'staticPressure', operator: '<', threshold: 'setpoint * 0.8' },
        { variable: 'supplyFan.power', operator: '>', threshold: 'rated * 0.9' }
      ],
      fault: {
        code: 'AHU-FAN-001',
        description: '공급 팬 효율이 저하됨',
        possibleCauses: ['벨트 슬립', '베어링 마모', '필터 막힘', '임펠러 손상'],
        recommendedActions: ['벨트 장력 확인', '베어링 점검', '필터 교체 확인']
      },
      severity: 'medium',
      energyImpact: { annualCost: 3000000, wastagePercent: 10 }
    });

    // VAV 관련 규칙
    this.rules.set('VAV_DAMPER_STUCK', {
      id: 'VAV_DAMPER_STUCK',
      name: 'VAV 댐퍼 고착',
      equipment: ['VAV'],
      conditions: [
        { variable: 'damper.command', operator: '!=', threshold: 'damper.position', tolerance: 10 },
        { variable: 'commandDuration', operator: '>', threshold: 300 }
      ],
      fault: {
        code: 'VAV-DMP-001',
        description: 'VAV 댐퍼가 명령 위치에 도달하지 못함',
        possibleCauses: ['액추에이터 고장', '댐퍼 기계적 문제', '에어 락'],
        recommendedActions: ['액추에이터 점검', '댐퍼 수동 조작', '덕트 점검']
      },
      severity: 'high'
    });

    // 냉동기 관련 규칙
    this.rules.set('CHILLER_LOW_DELTA_T', {
      id: 'CHILLER_LOW_DELTA_T',
      name: '냉동기 저온도차',
      equipment: ['chiller'],
      conditions: [
        { variable: 'loadPercentage', operator: '>', threshold: 30 },
        { variable: 'chilledWaterDeltaT', operator: '<', threshold: 3 }
      ],
      fault: {
        code: 'CHL-DTL-001',
        description: '냉수 온도차가 비정상적으로 낮음 (저온도차 증후군)',
        possibleCauses: ['과도한 냉수 유량', 'AHU 밸브 고장', '바이패스 발생'],
        recommendedActions: ['냉수 유량 확인', 'AHU 밸브 점검', '바이패스 밸브 확인']
      },
      severity: 'medium',
      energyImpact: { annualCost: 10000000, wastagePercent: 25 }
    });
  }
}

// 통계 기반 FDD
class StatisticalFDD {
  private baseline: BaselineModel;
  private thresholds: ThresholdConfig;

  detectAnomalies(data: TimeSeriesData): AnomalyResult[] {
    const anomalies: AnomalyResult[] = [];

    // 1. 이동 평균 대비 편차 분석
    const maAnomalies = this.detectMovingAverageAnomalies(data);
    anomalies.push(...maAnomalies);

    // 2. 계절성 분해 후 잔차 분석
    const residualAnomalies = this.detectResidualAnomalies(data);
    anomalies.push(...residualAnomalies);

    // 3. 상관관계 이탈 분석
    const correlationAnomalies = this.detectCorrelationBreaks(data);
    anomalies.push(...correlationAnomalies);

    return anomalies;
  }

  private detectMovingAverageAnomalies(data: TimeSeriesData): AnomalyResult[] {
    const anomalies: AnomalyResult[] = [];
    const windowSize = 24; // 24시간 이동 평균

    const movingAvg = this.calculateMovingAverage(data.values, windowSize);
    const movingStd = this.calculateMovingStd(data.values, windowSize);

    for (let i = windowSize; i < data.values.length; i++) {
      const value = data.values[i].value;
      const mean = movingAvg[i];
      const std = movingStd[i];

      // Z-score 계산
      const zscore = (value - mean) / std;

      if (Math.abs(zscore) > this.thresholds.zscoreThreshold) {
        anomalies.push({
          timestamp: data.values[i].timestamp,
          value,
          expectedValue: mean,
          zscore,
          type: 'statistical_outlier',
          confidence: 1 - this.normalCDF(-Math.abs(zscore))
        });
      }
    }

    return anomalies;
  }

  private detectResidualAnomalies(data: TimeSeriesData): AnomalyResult[] {
    // STL 분해 (Seasonal-Trend decomposition using LOESS)
    const decomposition = this.stlDecompose(data);

    const anomalies: AnomalyResult[] = [];
    const residualStd = this.calculateStd(decomposition.residual);

    for (let i = 0; i < decomposition.residual.length; i++) {
      const residual = decomposition.residual[i];

      if (Math.abs(residual) > 3 * residualStd) {
        anomalies.push({
          timestamp: data.values[i].timestamp,
          value: data.values[i].value,
          residual,
          type: 'seasonal_anomaly',
          confidence: Math.min(Math.abs(residual) / (3 * residualStd), 1)
        });
      }
    }

    return anomalies;
  }

  private stlDecompose(data: TimeSeriesData): STLDecomposition {
    // 간략화된 STL 분해 구현
    const values = data.values.map(v => v.value);
    const period = 24; // 일별 주기

    // 1. 트렌드 추출 (이동 평균)
    const trend = this.calculateMovingAverage(values, period * 2);

    // 2. 계절성 추출
    const detrended = values.map((v, i) => v - (trend[i] || v));
    const seasonal = this.extractSeasonality(detrended, period);

    // 3. 잔차 계산
    const residual = values.map((v, i) =>
      v - (trend[i] || v) - (seasonal[i % period] || 0)
    );

    return { trend, seasonal, residual };
  }
}
```

### 5.3.2 ML 기반 FDD

```typescript
// ML 기반 FDD 시스템
class MLBasedFDD {
  private isolationForest: IsolationForest;
  private autoencoder: Autoencoder;
  private classificationModel: FaultClassifier;

  constructor(config: MLFDDConfig) {
    this.isolationForest = new IsolationForest(config.isolationForest);
    this.autoencoder = new Autoencoder(config.autoencoder);
    this.classificationModel = new FaultClassifier(config.classifier);
  }

  async detectAndClassify(data: EquipmentData[]): Promise<MLFDDResult> {
    // 1. 이상 감지 (비지도 학습)
    const anomalyScores = await this.detectAnomalies(data);

    // 2. 이상 데이터 필터링
    const anomalousData = data.filter((_, i) =>
      anomalyScores[i] > this.config.anomalyThreshold
    );

    if (anomalousData.length === 0) {
      return { anomalies: [], faults: [] };
    }

    // 3. 고장 분류 (지도 학습)
    const faultClassifications = await this.classifyFaults(anomalousData);

    return {
      anomalies: anomalousData.map((d, i) => ({
        data: d,
        score: anomalyScores[data.indexOf(d)]
      })),
      faults: faultClassifications
    };
  }

  private async detectAnomalies(data: EquipmentData[]): Promise<number[]> {
    // 특징 추출
    const features = data.map(d => this.extractFeatures(d));

    // Isolation Forest로 이상 점수 계산
    const ifScores = this.isolationForest.predict(features);

    // Autoencoder 재구성 오차 계산
    const aeScores = await this.autoencoder.reconstructionError(features);

    // 앙상블 점수
    return ifScores.map((ifScore, i) =>
      (ifScore + aeScores[i]) / 2
    );
  }

  private extractFeatures(data: EquipmentData): number[] {
    // 설비 유형별 특징 추출
    const features: number[] = [];

    if (data.equipmentType === 'AHU') {
      features.push(
        data.ahuData?.supplyAirTemperature || 0,
        data.ahuData?.returnAirTemperature || 0,
        data.ahuData?.mixedAirTemperature || 0,
        data.ahuData?.outsideAirTemperature || 0,
        data.ahuData?.staticPressure || 0,
        data.ahuData?.supplyFanSpeed || 0,
        data.ahuData?.outdoorAirDamperPosition || 0,
        data.ahuData?.coolingValvePosition || 0,
        data.ahuData?.heatingValvePosition || 0
      );

      // 파생 특징
      const sat = data.ahuData?.supplyAirTemperature || 0;
      const rat = data.ahuData?.returnAirTemperature || 0;
      const mat = data.ahuData?.mixedAirTemperature || 0;
      const oat = data.ahuData?.outsideAirTemperature || 0;

      features.push(
        sat - rat, // 급기-환기 온도차
        (mat - oat) / (rat - oat + 0.01), // 외기 비율 추정
        data.ahuData?.coolingValvePosition! * data.ahuData?.heatingValvePosition! / 10000 // 동시 냉난방 지표
      );
    }

    return features;
  }

  private async classifyFaults(data: EquipmentData[]): Promise<FaultClassification[]> {
    const features = data.map(d => this.extractFeatures(d));
    const predictions = await this.classificationModel.predict(features);

    return predictions.map((pred, i) => ({
      equipmentId: data[i].equipmentId,
      faultType: pred.class,
      confidence: pred.probability,
      timestamp: data[i].timestamp
    }));
  }

  async train(trainingData: LabeledFaultData[]): Promise<TrainingResult> {
    // 1. 정상 데이터로 이상 감지 모델 학습
    const normalData = trainingData.filter(d => d.label === 'normal');
    const normalFeatures = normalData.map(d => this.extractFeatures(d.data));

    await this.isolationForest.fit(normalFeatures);
    await this.autoencoder.fit(normalFeatures);

    // 2. 레이블된 데이터로 분류 모델 학습
    const labeledFeatures = trainingData.map(d => ({
      features: this.extractFeatures(d.data),
      label: d.label
    }));

    const classificationResult = await this.classificationModel.fit(labeledFeatures);

    return {
      anomalyModelMetrics: {
        isolationForest: this.isolationForest.getMetrics(),
        autoencoder: this.autoencoder.getMetrics()
      },
      classificationMetrics: classificationResult.metrics
    };
  }
}
```

---

## 5.4 장 요약

### HVAC 제어 시퀀스 핵심

| 시스템 | 주요 제어 | 최적화 기법 |
|--------|----------|------------|
| AHU | SAT, 정압, 외기량 | Trim & Respond, 이코노마이저 |
| VAV | 풍량, 리히트 | 수요 제어 환기, 설정값 리셋 |
| 냉동기 | 스테이징, 냉수 온도 | 부하 기반 시퀀싱, 저온도차 방지 |

### 최적화 알고리즘 요약

| 기법 | 용도 | 절감 효과 |
|------|------|----------|
| 최적 기동/정지 | 예열/예냉 시간 최적화 | 5-15% |
| 설정값 리셋 | SAT, 정압 동적 조절 | 10-20% |
| MPC | 예측 기반 최적 제어 | 15-30% |

### FDD 방법론 비교

| 방식 | 장점 | 단점 |
|------|------|------|
| 규칙 기반 | 해석 용이, 즉시 적용 | 새로운 고장 탐지 어려움 |
| 통계 기반 | 기준선 대비 이상 탐지 | 복잡한 패턴 인식 한계 |
| ML 기반 | 복잡한 패턴 학습 | 학습 데이터 필요, 해석 어려움 |

### 다음 장 미리보기

제6장에서는 WIA-BEMS Phase 4인 시스템 통합에 대해 다룹니다. 스마트 그리드 연계, 재생에너지 통합, 엔터프라이즈 시스템 연동을 학습합니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
