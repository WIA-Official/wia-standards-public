# 05. 센서 관리
## Sensor Management - Configuration, Calibration, Status Monitoring

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [센서 관리 개요](#센서-관리-개요)
2. [센서 설정 및 등록](#센서-설정-및-등록)
3. [센서 교정 시스템](#센서-교정-시스템)
4. [상태 모니터링](#상태-모니터링)
5. [센서 진단 및 문제 해결](#센서-진단-및-문제-해결)
6. [센서 네트워크 관리](#센서-네트워크-관리)
7. [자동 복구 시스템](#자동-복구-시스템)
8. [한국 규제 준수](#한국-규제-준수)

---

## 센서 관리 개요

### 센서 생애주기 관리

```typescript
/**
 * 센서 생애주기 (Sensor Lifecycle)
 */
enum SensorLifecycleStage {
  PROCUREMENT = "procurement",           // 구매
  RECEIVING = "receiving",               // 수령
  INSTALLATION = "installation",         // 설치
  COMMISSIONING = "commissioning",       // 시운전
  CALIBRATION = "calibration",           // 교정
  OPERATION = "operation",               // 운영
  MAINTENANCE = "maintenance",           // 유지보수
  RECALIBRATION = "recalibration",      // 재교정
  DECOMMISSIONING = "decommissioning",  // 폐기
  DISPOSAL = "disposal"                  // 처분
}

const SensorLifecycleKr: Record<SensorLifecycleStage, string> = {
  [SensorLifecycleStage.PROCUREMENT]: "구매 - 센서 선정 및 발주",
  [SensorLifecycleStage.RECEIVING]: "수령 - 검수 및 입고",
  [SensorLifecycleStage.INSTALLATION]: "설치 - 물리적 설치 및 배선",
  [SensorLifecycleStage.COMMISSIONING]: "시운전 - 초기 설정 및 테스트",
  [SensorLifecycleStage.CALIBRATION]: "교정 - 정확도 확보",
  [SensorLifecycleStage.OPERATION]: "운영 - 정상 작동",
  [SensorLifecycleStage.MAINTENANCE]: "유지보수 - 정기 점검",
  [SensorLifecycleStage.RECALIBRATION]: "재교정 - 정확도 재확인",
  [SensorLifecycleStage.DECOMMISSIONING]: "폐기 - 사용 중지",
  [SensorLifecycleStage.DISPOSAL]: "처분 - 물리적 제거"
};

/**
 * 센서 생애주기 관리 시스템
 */
class SensorLifecycleManager {
  private sensors: Map<string, SensorLifecycleRecord> = new Map();

  /**
   * 새 센서 구매 기록
   */
  async procureSensor(procurement: {
    manufacturer: string;
    manufacturerKr: string;
    model: string;
    quantity: number;
    purpose: string;
    purposeKr: string;
    purchaseOrder: string;
    estimatedDelivery: Date;
  }): Promise<{
    procurementId: string;
    status: string;
    statusKr: string;
  }> {
    const procurementId = this.generateId();

    console.log(`[센서 구매] ${procurement.manufacturerKr} ${procurement.model}`);
    console.log(`수량: ${procurement.quantity}개`);
    console.log(`목적: ${procurement.purposeKr}`);
    console.log(`예상 납기: ${procurement.estimatedDelivery.toLocaleDateString("ko-KR")}`);

    return {
      procurementId,
      status: "pending_delivery",
      statusKr: "납품 대기 중"
    };
  }

  /**
   * 센서 수령 및 검수
   */
  async receiveSensor(receiving: {
    procurementId: string;
    serialNumbers: string[];
    condition: "good" | "damaged" | "defective";
    conditionKr: string;
    receivedBy: string;
    receivedDate: Date;
    inspectionNotes?: string;
  }): Promise<{
    receivingId: string;
    accepted: boolean;
    acceptedKr: string;
  }> {
    const receivingId = this.generateId();

    if (receiving.condition !== "good") {
      console.warn(`[검수 불합격] 상태: ${receiving.conditionKr}`);
      return {
        receivingId,
        accepted: false,
        acceptedKr: "검수 불합격 - 반품 처리"
      };
    }

    console.log(`[센서 수령] ${receiving.serialNumbers.length}개 센서 검수 완료`);
    console.log(`검수자: ${receiving.receivedBy}`);
    console.log(`상태: ${receiving.conditionKr}`);

    return {
      receivingId,
      accepted: true,
      acceptedKr: "검수 합격 - 설치 대기"
    };
  }

  /**
   * 센서 설치
   */
  async installSensor(installation: {
    serialNumber: string;
    facilityId: string;
    tankId?: string;
    location: {
      zone: string;
      position: { x: number; y: number; z: number };
      locationKr: string;
    };
    installedBy: string;
    installationDate: Date;
    verificationTest: boolean;
  }): Promise<{
    sensorId: string;
    installationKr: string;
  }> {
    const sensorId = this.generateId();

    console.log(`[센서 설치] ${installation.location.locationKr}`);
    console.log(`시리얼: ${installation.serialNumber}`);
    console.log(`설치자: ${installation.installedBy}`);
    console.log(`검증 테스트: ${installation.verificationTest ? "완료" : "미실시"}`);

    // 생애주기 기록 생성
    const record: SensorLifecycleRecord = {
      sensorId,
      serialNumber: installation.serialNumber,
      stage: SensorLifecycleStage.INSTALLATION,
      stageKr: SensorLifecycleKr[SensorLifecycleStage.INSTALLATION],
      history: [
        {
          stage: SensorLifecycleStage.INSTALLATION,
          timestamp: installation.installationDate,
          performedBy: installation.installedBy,
          notes: `설치 위치: ${installation.location.locationKr}`
        }
      ]
    };

    this.sensors.set(sensorId, record);

    return {
      sensorId,
      installationKr: "센서 설치 완료 - 시운전 대기"
    };
  }

  /**
   * 센서 시운전
   */
  async commissionSensor(commissioning: {
    sensorId: string;
    configuration: SensorConfig;
    testResults: {
      communicationTest: boolean;
      accuracyTest: boolean;
      rangeTest: boolean;
      alarmTest: boolean;
      testKr: string;
    };
    commissionedBy: string;
    commissioningDate: Date;
  }): Promise<{
    success: boolean;
    nextStep: string;
    nextStepKr: string;
  }> {
    const allTestsPassed =
      commissioning.testResults.communicationTest &&
      commissioning.testResults.accuracyTest &&
      commissioning.testResults.rangeTest &&
      commissioning.testResults.alarmTest;

    if (!allTestsPassed) {
      console.error(`[시운전 실패] ${commissioning.testResults.testKr}`);
      return {
        success: false,
        nextStep: "troubleshooting",
        nextStepKr: "문제 해결 필요"
      };
    }

    console.log(`[시운전 성공] 모든 테스트 통과`);
    console.log(`담당자: ${commissioning.commissionedBy}`);

    // 생애주기 업데이트
    const record = this.sensors.get(commissioning.sensorId);
    if (record) {
      record.stage = SensorLifecycleStage.COMMISSIONING;
      record.stageKr = SensorLifecycleKr[SensorLifecycleStage.COMMISSIONING];
      record.history.push({
        stage: SensorLifecycleStage.COMMISSIONING,
        timestamp: commissioning.commissioningDate,
        performedBy: commissioning.commissionedBy,
        notes: commissioning.testResults.testKr
      });
    }

    return {
      success: true,
      nextStep: "calibration",
      nextStepKr: "교정 단계로 진행"
    };
  }

  /**
   * 센서 폐기
   */
  async decommissionSensor(decommissioning: {
    sensorId: string;
    reason: string;
    reasonKr: string;
    decommissionedBy: string;
    decommissioningDate: Date;
    dataBackup: boolean;
    physicalRemoval: boolean;
  }): Promise<{
    success: boolean;
    disposalInstructions: string;
    disposalInstructionsKr: string;
  }> {
    console.log(`[센서 폐기] ${decommissioning.sensorId}`);
    console.log(`사유: ${decommissioning.reasonKr}`);
    console.log(`담당자: ${decommissioning.decommissionedBy}`);
    console.log(`데이터 백업: ${decommissioning.dataBackup ? "완료" : "미실시"}`);

    // 생애주기 업데이트
    const record = this.sensors.get(decommissioning.sensorId);
    if (record) {
      record.stage = SensorLifecycleStage.DECOMMISSIONING;
      record.stageKr = SensorLifecycleKr[SensorLifecycleStage.DECOMMISSIONING];
      record.history.push({
        stage: SensorLifecycleStage.DECOMMISSIONING,
        timestamp: decommissioning.decommissioningDate,
        performedBy: decommissioning.decommissionedBy,
        notes: decommissioning.reasonKr
      });
    }

    return {
      success: true,
      disposalInstructions: "Follow electronic waste disposal regulations",
      disposalInstructionsKr: "전자폐기물 처리 규정에 따라 폐기하세요"
    };
  }

  private generateId(): string {
    return `SENSOR-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

interface SensorLifecycleRecord {
  sensorId: string;
  serialNumber: string;
  stage: SensorLifecycleStage;
  stageKr: string;
  history: {
    stage: SensorLifecycleStage;
    timestamp: Date;
    performedBy: string;
    notes: string;
  }[];
}
```

---

## 센서 설정 및 등록

### 센서 등록 시스템

```typescript
/**
 * 센서 등록 시스템
 */
class SensorRegistrationSystem {
  /**
   * 센서 자동 검색 (Auto-Discovery)
   */
  async discoverSensors(network: {
    type: "modbus" | "ethernet" | "wireless";
    addressRange?: string;
    port?: number;
    networkKr: string;
  }): Promise<{
    discovered: {
      address: string;
      type: string;
      typeKr: string;
      manufacturer: string;
      model: string;
      serialNumber: string;
    }[];
    totalFound: number;
    discoveryKr: string;
  }> {
    console.log(`[센서 검색] ${network.networkKr} 스캔 중...`);

    // 실제 구현에서는 네트워크 스캔 수행
    const discovered = [
      {
        address: "192.168.1.100",
        type: "temperature",
        typeKr: "온도 센서",
        manufacturer: "Samsung Electronics",
        model: "SE-CRYO-2026",
        serialNumber: "SE2026-001-KR"
      },
      {
        address: "192.168.1.101",
        type: "level",
        typeKr: "레벨 센서",
        manufacturer: "LG Electronics",
        model: "LG-LVL-2026",
        serialNumber: "LG2026-002-KR"
      }
    ];

    console.log(`[검색 완료] ${discovered.length}개 센서 발견`);

    return {
      discovered,
      totalFound: discovered.length,
      discoveryKr: `${discovered.length}개 센서 자동 검색 완료`
    };
  }

  /**
   * 센서 등록
   */
  async registerSensor(registration: {
    // 기본 정보
    name: string;
    nameKr: string;
    type: SensorType;
    typeKr: string;

    // 하드웨어 정보
    hardware: {
      manufacturer: string;
      manufacturerKr: string;
      model: string;
      serialNumber: string;
      manufactureDate: Date;
      warrantyExpiry: Date;
      hardwareKr: string;
    };

    // 설치 위치
    location: {
      facilityId: string;
      tankId?: string;
      zone: string;
      position: { x: number; y: number; z: number };
      coordinates?: { lat: number; lng: number };
      locationKr: string;
    };

    // 통신 설정
    communication: {
      protocol: string;
      address: string;
      port?: number;
      updateInterval: number;
      timeout: number;
      commKr: string;
    };

    // 측정 사양
    specification: {
      unit: MeasurementUnit;
      unitKr: string;
      minValue: number;
      maxValue: number;
      accuracy: number;
      resolution: number;
      samplingRate: number;
      specKr: string;
    };

    // 알림 임계값
    thresholds: {
      warning: { upper: number; lower: number; warningKr: string };
      critical: { upper: number; lower: number; criticalKr: string };
      emergency: { upper: number; lower: number; emergencyKr: string };
    };

    // 교정 계획
    calibrationPlan: {
      interval: number;             // 교정 주기 (일)
      nextCalibration: Date;        // 다음 교정 예정일
      planKr: string;
    };

    // 등록 담당자
    registeredBy: string;
  }): Promise<{
    sensor: SensorConfig;
    registrationKr: string;
  }> {
    console.log(`[센서 등록] ${registration.nameKr}`);
    console.log(`제조사: ${registration.hardware.manufacturerKr}`);
    console.log(`모델: ${registration.hardware.model}`);
    console.log(`위치: ${registration.location.locationKr}`);
    console.log(`통신: ${registration.communication.commKr}`);

    const sensor: SensorConfig = {
      sensorId: this.generateSensorId(),
      name: registration.name,
      nameKr: registration.nameKr,
      type: registration.type,
      typeKr: registration.typeKr,
      location: registration.location,
      specification: registration.specification,
      calibration: {
        lastCalibration: new Date(),
        nextCalibration: registration.calibrationPlan.nextCalibration,
        calibrationInterval: registration.calibrationPlan.interval,
        calibratedBy: registration.registeredBy,
        calibrationKr: registration.calibrationPlan.planKr
      },
      manufacturer: registration.hardware,
      communication: registration.communication,
      thresholds: registration.thresholds,
      status: "active" as SensorStatus,
      statusKr: "정상 작동",
      metadata: {
        installDate: new Date(),
        installedBy: registration.registeredBy,
        notes: "",
        tags: [],
        metadataKr: `등록자: ${registration.registeredBy}`
      },
      createdAt: new Date(),
      updatedAt: new Date()
    };

    // 데이터베이스에 저장 (실제 구현)
    await this.saveSensorToDatabase(sensor);

    // 모니터링 시스템에 추가
    await this.addToMonitoringSystem(sensor);

    return {
      sensor,
      registrationKr: `센서 등록 완료 - ${sensor.nameKr} (${sensor.sensorId})`
    };
  }

  /**
   * 센서 벌크 등록
   */
  async registerSensorsBulk(sensors: any[]): Promise<{
    success: SensorConfig[];
    failed: { index: number; error: string; errorKr: string }[];
    summaryKr: string;
  }> {
    const success: SensorConfig[] = [];
    const failed: { index: number; error: string; errorKr: string }[] = [];

    for (let i = 0; i < sensors.length; i++) {
      try {
        const result = await this.registerSensor(sensors[i]);
        success.push(result.sensor);
      } catch (error) {
        failed.push({
          index: i,
          error: error.message,
          errorKr: `센서 ${i + 1} 등록 실패: ${error.message}`
        });
      }
    }

    console.log(`[벌크 등록] 성공: ${success.length}개, 실패: ${failed.length}개`);

    return {
      success,
      failed,
      summaryKr: `총 ${sensors.length}개 중 ${success.length}개 등록 성공, ${failed.length}개 실패`
    };
  }

  private generateSensorId(): string {
    return `SENSOR-${Date.now()}-${Math.random().toString(36).substr(2, 9).toUpperCase()}`;
  }

  private async saveSensorToDatabase(sensor: SensorConfig): Promise<void> {
    // 데이터베이스 저장 로직
    console.log(`[DB] 센서 저장: ${sensor.sensorId}`);
  }

  private async addToMonitoringSystem(sensor: SensorConfig): Promise<void> {
    // 모니터링 시스템에 추가
    console.log(`[모니터링] 센서 추가: ${sensor.nameKr}`);
  }
}
```

---

## 센서 교정 시스템

### 교정 관리

```typescript
/**
 * 센서 교정 시스템
 */
class SensorCalibrationSystem {
  /**
   * 교정 수행
   */
  async performCalibration(calibration: {
    sensorId: string;
    method: "single_point" | "two_point" | "multi_point" | "reference_comparison";
    methodKr: string;

    // 참조 표준
    reference: {
      type: string;
      typeKr: string;
      serialNumber: string;
      certificationDate: Date;
      traceable: boolean;              // KOLAS/NIST 추적성
      referenceKr: string;
    };

    // 교정 데이터
    measurements: {
      referenceValue: number;          // 참조값
      sensorValue: number;             // 센서값
      deviation: number;               // 편차
      unit: string;
      unitKr: string;
    }[];

    // 환경 조건
    environment: {
      temperature: number;             // 교정 시 온도
      humidity: number;                // 교정 시 습도
      pressure: number;                // 교정 시 압력
      environmentKr: string;
    };

    // 교정 결과
    result: {
      passed: boolean;
      accuracy: number;                // 실제 정확도
      uncertainty: number;             // 불확도
      adjustmentMade: boolean;         // 조정 여부
      resultKr: string;
    };

    // 교정 담당자
    calibratedBy: string;
    calibrationDate: Date;

    // 인증서
    certificateUrl?: string;
  }): Promise<{
    calibrationId: string;
    sensor: SensorConfig;
    certificate: {
      number: string;
      url: string;
      validUntil: Date;
      certificateKr: string;
    };
    calibrationKr: string;
  }> {
    console.log(`[센서 교정] ${calibration.sensorId}`);
    console.log(`방법: ${calibration.methodKr}`);
    console.log(`참조 표준: ${calibration.reference.referenceKr}`);
    console.log(`측정 포인트: ${calibration.measurements.length}개`);

    // 교정 데이터 분석
    const analysis = this.analyzeCalibrationData(calibration.measurements);

    // 합격/불합격 판정
    if (!calibration.result.passed) {
      console.warn(`[교정 불합격] ${calibration.result.resultKr}`);
      throw new Error(`센서 교정 실패: ${calibration.result.resultKr}`);
    }

    // 교정 인증서 생성
    const certificateNumber = this.generateCertificateNumber();
    const certificate = await this.generateCalibrationCertificate({
      certificateNumber,
      sensorId: calibration.sensorId,
      method: calibration.methodKr,
      reference: calibration.reference,
      measurements: calibration.measurements,
      result: calibration.result,
      environment: calibration.environment,
      calibratedBy: calibration.calibratedBy,
      calibrationDate: calibration.calibrationDate
    });

    // 센서 정보 업데이트
    const updatedSensor = await this.updateSensorCalibration(
      calibration.sensorId,
      {
        lastCalibration: calibration.calibrationDate,
        nextCalibration: this.calculateNextCalibration(calibration.calibrationDate),
        calibratedBy: calibration.calibratedBy,
        certificate: certificate.url
      }
    );

    console.log(`[교정 완료] 인증서 번호: ${certificateNumber}`);
    console.log(`다음 교정 예정일: ${updatedSensor.calibration.nextCalibration.toLocaleDateString("ko-KR")}`);

    return {
      calibrationId: this.generateCalibrationId(),
      sensor: updatedSensor,
      certificate: {
        number: certificateNumber,
        url: certificate.url,
        validUntil: updatedSensor.calibration.nextCalibration,
        certificateKr: `교정 인증서 - ${certificateNumber}`
      },
      calibrationKr: `센서 교정 완료 - 정확도 ${calibration.result.accuracy}%`
    };
  }

  /**
   * 교정 스케줄 관리
   */
  async getCalibrationSchedule(facilityId: string): Promise<{
    upcoming: {
      sensorId: string;
      nameKr: string;
      dueDate: Date;
      daysUntilDue: number;
      priority: "urgent" | "high" | "normal";
      priorityKr: string;
    }[];
    overdue: {
      sensorId: string;
      nameKr: string;
      dueDate: Date;
      daysOverdue: number;
      overdueKr: string;
    }[];
    scheduleKr: string;
  }> {
    // 교정 예정인 센서 조회
    const sensors = await this.getSensorsByFacility(facilityId);
    const now = new Date();

    const upcoming = sensors
      .filter(s => s.calibration.nextCalibration > now)
      .map(s => {
        const daysUntilDue = Math.floor(
          (s.calibration.nextCalibration.getTime() - now.getTime()) / (1000 * 60 * 60 * 24)
        );

        let priority: "urgent" | "high" | "normal";
        let priorityKr: string;

        if (daysUntilDue <= 7) {
          priority = "urgent";
          priorityKr = "긴급 - 7일 이내";
        } else if (daysUntilDue <= 30) {
          priority = "high";
          priorityKr = "높음 - 30일 이내";
        } else {
          priority = "normal";
          priorityKr = "보통";
        }

        return {
          sensorId: s.sensorId,
          nameKr: s.nameKr,
          dueDate: s.calibration.nextCalibration,
          daysUntilDue,
          priority,
          priorityKr
        };
      })
      .sort((a, b) => a.daysUntilDue - b.daysUntilDue);

    const overdue = sensors
      .filter(s => s.calibration.nextCalibration <= now)
      .map(s => {
        const daysOverdue = Math.floor(
          (now.getTime() - s.calibration.nextCalibration.getTime()) / (1000 * 60 * 60 * 24)
        );

        return {
          sensorId: s.sensorId,
          nameKr: s.nameKr,
          dueDate: s.calibration.nextCalibration,
          daysOverdue,
          overdueKr: `교정 기한 ${daysOverdue}일 초과`
        };
      })
      .sort((a, b) => b.daysOverdue - a.daysOverdue);

    console.log(`[교정 스케줄] 예정: ${upcoming.length}개, 기한 초과: ${overdue.length}개`);

    if (overdue.length > 0) {
      console.warn(`[경고] ${overdue.length}개 센서 교정 기한 초과!`);
    }

    return {
      upcoming,
      overdue,
      scheduleKr: `교정 예정 ${upcoming.length}개, 기한 초과 ${overdue.length}개`
    };
  }

  /**
   * 자동 교정 알림
   */
  async setupCalibrationReminders(settings: {
    reminderDays: number[];           // 교정 전 알림 (예: [30, 14, 7, 1])
    recipients: string[];             // 알림 수신자
    channels: ("email" | "sms" | "push")[];
    settingsKr: string;
  }): Promise<{
    success: boolean;
    settingsKr: string;
  }> {
    console.log(`[교정 알림 설정] ${settings.settingsKr}`);
    console.log(`알림 시점: ${settings.reminderDays.join(", ")}일 전`);
    console.log(`수신자: ${settings.recipients.length}명`);

    // 알림 스케줄러 설정 (실제 구현에서는 cron job 등 사용)

    return {
      success: true,
      settingsKr: `교정 알림 설정 완료 - ${settings.reminderDays.length}개 시점, ${settings.recipients.length}명 수신`
    };
  }

  /**
   * 교정 데이터 분석
   */
  private analyzeCalibrationData(measurements: {
    referenceValue: number;
    sensorValue: number;
    deviation: number;
  }[]): {
    meanDeviation: number;
    maxDeviation: number;
    linearity: number;
    analysisKr: string;
  } {
    const deviations = measurements.map(m => Math.abs(m.deviation));
    const meanDeviation = deviations.reduce((a, b) => a + b, 0) / deviations.length;
    const maxDeviation = Math.max(...deviations);

    // 선형성 계산 (실제로는 선형 회귀 분석 수행)
    const linearity = 0.999; // 예시

    return {
      meanDeviation,
      maxDeviation,
      linearity,
      analysisKr: `평균 편차: ${meanDeviation.toFixed(3)}, 최대 편차: ${maxDeviation.toFixed(3)}, 선형성: ${linearity}`
    };
  }

  /**
   * 교정 인증서 생성
   */
  private async generateCalibrationCertificate(data: any): Promise<{
    url: string;
    certificateKr: string;
  }> {
    // PDF 인증서 생성 (실제 구현)
    const url = `/certificates/${data.certificateNumber}.pdf`;

    return {
      url,
      certificateKr: `교정 인증서 생성 완료`
    };
  }

  /**
   * 다음 교정 날짜 계산
   */
  private calculateNextCalibration(lastCalibration: Date, interval: number = 180): Date {
    const nextDate = new Date(lastCalibration);
    nextDate.setDate(nextDate.getDate() + interval);
    return nextDate;
  }

  private generateCertificateNumber(): string {
    const year = new Date().getFullYear();
    const random = Math.random().toString(36).substr(2, 8).toUpperCase();
    return `CAL-${year}-${random}`;
  }

  private generateCalibrationId(): string {
    return `CALIB-${Date.now()}`;
  }

  private async updateSensorCalibration(sensorId: string, calibrationData: any): Promise<SensorConfig> {
    // 센서 업데이트 로직
    return {} as SensorConfig;
  }

  private async getSensorsByFacility(facilityId: string): Promise<SensorConfig[]> {
    // 시설의 모든 센서 조회
    return [];
  }
}
```

---

## 상태 모니터링

### 센서 건강 상태

```typescript
/**
 * 센서 건강 상태 모니터링
 */
class SensorHealthMonitor {
  /**
   * 센서 건강 점수 계산
   */
  calculateHealthScore(sensor: {
    sensorId: string;
    status: SensorStatus;
    lastReading: SensorReading | null;
    calibration: {
      lastCalibration: Date;
      nextCalibration: Date;
    };
    statistics: {
      uptimePercentage: number;        // 가동률
      dataQuality: {
        good: number;
        fair: number;
        poor: number;
      };
      errorRate: number;               // 오류율
    };
    communication: {
      signalStrength?: number;
      latency: number;                 // 지연 시간 (ms)
      packetLoss: number;              // 패킷 손실률 (%)
    };
  }): {
    score: number;                      // 0-100
    grade: "A" | "B" | "C" | "D" | "F";
    gradeKr: string;
    issues: {
      category: string;
      categoryKr: string;
      severity: "low" | "medium" | "high";
      severityKr: string;
      description: string;
      descriptionKr: string;
    }[];
    recommendations: {
      action: string;
      actionKr: string;
      priority: number;
    }[];
    healthKr: string;
  } {
    let score = 100;
    const issues: any[] = [];
    const recommendations: any[] = [];

    // 1. 센서 상태 평가 (30점)
    if (sensor.status === "active") {
      // 정상
    } else if (sensor.status === "calibrating" || sensor.status === "maintenance") {
      score -= 10;
    } else if (sensor.status === "error") {
      score -= 20;
      issues.push({
        category: "status",
        categoryKr: "센서 상태",
        severity: "high",
        severityKr: "높음",
        description: "Sensor in error state",
        descriptionKr: "센서 오류 상태"
      });
    } else if (sensor.status === "offline") {
      score -= 30;
      issues.push({
        category: "status",
        categoryKr: "센서 상태",
        severity: "high",
        severityKr: "높음",
        description: "Sensor offline",
        descriptionKr: "센서 오프라인"
      });
    }

    // 2. 가동률 평가 (20점)
    if (sensor.statistics.uptimePercentage < 95) {
      const deduction = Math.floor((95 - sensor.statistics.uptimePercentage) * 0.5);
      score -= Math.min(deduction, 20);
      issues.push({
        category: "uptime",
        categoryKr: "가동률",
        severity: sensor.statistics.uptimePercentage < 90 ? "high" : "medium",
        severityKr: sensor.statistics.uptimePercentage < 90 ? "높음" : "중간",
        description: `Low uptime: ${sensor.statistics.uptimePercentage}%`,
        descriptionKr: `낮은 가동률: ${sensor.statistics.uptimePercentage}%`
      });
      recommendations.push({
        action: "Investigate sensor reliability",
        actionKr: "센서 신뢰성 점검 필요",
        priority: 2
      });
    }

    // 3. 데이터 품질 평가 (20점)
    const goodPercentage = sensor.statistics.dataQuality.good;
    if (goodPercentage < 95) {
      const deduction = Math.floor((95 - goodPercentage) * 0.5);
      score -= Math.min(deduction, 20);
      issues.push({
        category: "data_quality",
        categoryKr: "데이터 품질",
        severity: goodPercentage < 90 ? "high" : "medium",
        severityKr: goodPercentage < 90 ? "높음" : "중간",
        description: `Low data quality: ${goodPercentage}% good`,
        descriptionKr: `낮은 데이터 품질: ${goodPercentage}% 양호`
      });
      recommendations.push({
        action: "Calibrate sensor",
        actionKr: "센서 교정 권장",
        priority: 1
      });
    }

    // 4. 교정 상태 평가 (15점)
    const now = new Date();
    const calibrationOverdue = sensor.calibration.nextCalibration < now;
    if (calibrationOverdue) {
      const daysOverdue = Math.floor(
        (now.getTime() - sensor.calibration.nextCalibration.getTime()) / (1000 * 60 * 60 * 24)
      );
      score -= Math.min(15, Math.floor(daysOverdue / 7));
      issues.push({
        category: "calibration",
        categoryKr: "교정",
        severity: daysOverdue > 30 ? "high" : "medium",
        severityKr: daysOverdue > 30 ? "높음" : "중간",
        description: `Calibration overdue by ${daysOverdue} days`,
        descriptionKr: `교정 기한 ${daysOverdue}일 초과`
      });
      recommendations.push({
        action: "Schedule calibration immediately",
        actionKr: "즉시 교정 일정 잡기",
        priority: 1
      });
    }

    // 5. 통신 상태 평가 (15점)
    if (sensor.communication.latency > 1000) {
      score -= 5;
      issues.push({
        category: "communication",
        categoryKr: "통신",
        severity: "medium",
        severityKr: "중간",
        description: "High latency",
        descriptionKr: "높은 지연 시간"
      });
    }
    if (sensor.communication.packetLoss > 5) {
      score -= 10;
      issues.push({
        category: "communication",
        categoryKr: "통신",
        severity: "high",
        severityKr: "높음",
        description: `High packet loss: ${sensor.communication.packetLoss}%`,
        descriptionKr: `높은 패킷 손실: ${sensor.communication.packetLoss}%`
      });
      recommendations.push({
        action: "Check network connection",
        actionKr: "네트워크 연결 점검",
        priority: 2
      });
    }

    // 등급 결정
    let grade: "A" | "B" | "C" | "D" | "F";
    let gradeKr: string;

    if (score >= 90) {
      grade = "A";
      gradeKr = "A등급 - 매우 양호";
    } else if (score >= 80) {
      grade = "B";
      gradeKr = "B등급 - 양호";
    } else if (score >= 70) {
      grade = "C";
      gradeKr = "C등급 - 보통";
    } else if (score >= 60) {
      grade = "D";
      gradeKr = "D등급 - 주의 필요";
    } else {
      grade = "F";
      gradeKr = "F등급 - 즉시 조치 필요";
    }

    return {
      score: Math.max(0, score),
      grade,
      gradeKr,
      issues,
      recommendations: recommendations.sort((a, b) => a.priority - b.priority),
      healthKr: `건강 점수: ${score}점 (${gradeKr})`
    };
  }

  /**
   * 시설 전체 센서 건강 대시보드
   */
  async getFacilityHealthDashboard(facilityId: string): Promise<{
    overallHealth: {
      averageScore: number;
      totalSensors: number;
      healthy: number;
      warning: number;
      critical: number;
      overallKr: string;
    };
    sensorsByGrade: {
      A: number;
      B: number;
      C: number;
      D: number;
      F: number;
    };
    topIssues: {
      category: string;
      categoryKr: string;
      count: number;
      affectedSensors: string[];
    }[];
    urgentActions: {
      sensorId: string;
      nameKr: string;
      actionKr: string;
      priority: number;
    }[];
    dashboardKr: string;
  }> {
    // 시설의 모든 센서 건강 점수 계산
    const sensors = await this.getSensorsByFacility(facilityId);
    const healthScores = sensors.map(s => this.calculateHealthScore(s));

    const totalSensors = sensors.length;
    const averageScore = healthScores.reduce((sum, h) => sum + h.score, 0) / totalSensors;

    const healthy = healthScores.filter(h => h.score >= 80).length;
    const warning = healthScores.filter(h => h.score >= 60 && h.score < 80).length;
    const critical = healthScores.filter(h => h.score < 60).length;

    const sensorsByGrade = {
      A: healthScores.filter(h => h.grade === "A").length,
      B: healthScores.filter(h => h.grade === "B").length,
      C: healthScores.filter(h => h.grade === "C").length,
      D: healthScores.filter(h => h.grade === "D").length,
      F: healthScores.filter(h => h.grade === "F").length
    };

    // 주요 문제 집계
    const issueCount = new Map<string, { count: number; sensors: string[] }>();
    healthScores.forEach((h, i) => {
      h.issues.forEach(issue => {
        const key = issue.categoryKr;
        if (!issueCount.has(key)) {
          issueCount.set(key, { count: 0, sensors: [] });
        }
        const data = issueCount.get(key)!;
        data.count++;
        data.sensors.push(sensors[i].sensorId);
      });
    });

    const topIssues = Array.from(issueCount.entries())
      .map(([category, data]) => ({
        category,
        categoryKr: category,
        count: data.count,
        affectedSensors: data.sensors
      }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 5);

    // 긴급 조치 필요 센서
    const urgentActions: any[] = [];
    healthScores.forEach((h, i) => {
      if (h.recommendations.length > 0) {
        h.recommendations.forEach(rec => {
          urgentActions.push({
            sensorId: sensors[i].sensorId,
            nameKr: sensors[i].nameKr,
            actionKr: rec.actionKr,
            priority: rec.priority
          });
        });
      }
    });

    urgentActions.sort((a, b) => a.priority - b.priority);

    return {
      overallHealth: {
        averageScore: Math.round(averageScore),
        totalSensors,
        healthy,
        warning,
        critical,
        overallKr: `평균 건강 점수: ${Math.round(averageScore)}점, 양호: ${healthy}개, 경고: ${warning}개, 위험: ${critical}개`
      },
      sensorsByGrade,
      topIssues,
      urgentActions: urgentActions.slice(0, 10),
      dashboardKr: `센서 건강 대시보드 - 총 ${totalSensors}개 센서`
    };
  }

  private async getSensorsByFacility(facilityId: string): Promise<any[]> {
    // 센서 조회 로직
    return [];
  }
}
```

---

## 센서 진단 및 문제 해결

### 자동 진단 시스템

```typescript
/**
 * 센서 자동 진단 시스템
 */
class SensorDiagnosticSystem {
  /**
   * 센서 진단 수행
   */
  async diagnose(sensorId: string): Promise<{
    diagnosticId: string;
    sensorId: string;
    timestamp: Date;
    tests: {
      name: string;
      nameKr: string;
      passed: boolean;
      result: string;
      resultKr: string;
      details?: any;
    }[];
    overallStatus: "pass" | "warn" | "fail";
    overallStatusKr: string;
    troubleshootingSteps: {
      step: number;
      actionKr: string;
      expectedResultKr: string;
    }[];
    diagnosticKr: string;
  }> {
    console.log(`[센서 진단] ${sensorId} 진단 시작...`);

    const tests = [];

    // 1. 통신 테스트
    const commTest = await this.testCommunication(sensorId);
    tests.push({
      name: "Communication Test",
      nameKr: "통신 테스트",
      passed: commTest.passed,
      result: commTest.result,
      resultKr: commTest.resultKr,
      details: commTest.details
    });

    // 2. 데이터 품질 테스트
    const dataQualityTest = await this.testDataQuality(sensorId);
    tests.push({
      name: "Data Quality Test",
      nameKr: "데이터 품질 테스트",
      passed: dataQualityTest.passed,
      result: dataQualityTest.result,
      resultKr: dataQualityTest.resultKr
    });

    // 3. 정확도 테스트
    const accuracyTest = await this.testAccuracy(sensorId);
    tests.push({
      name: "Accuracy Test",
      nameKr: "정확도 테스트",
      passed: accuracyTest.passed,
      result: accuracyTest.result,
      resultKr: accuracyTest.resultKr
    });

    // 4. 응답 시간 테스트
    const responseTest = await this.testResponseTime(sensorId);
    tests.push({
      name: "Response Time Test",
      nameKr: "응답 시간 테스트",
      passed: responseTest.passed,
      result: responseTest.result,
      resultKr: responseTest.resultKr
    });

    // 5. 전원 및 배터리 테스트
    const powerTest = await this.testPower(sensorId);
    tests.push({
      name: "Power Test",
      nameKr: "전원/배터리 테스트",
      passed: powerTest.passed,
      result: powerTest.result,
      resultKr: powerTest.resultKr
    });

    // 전체 상태 판정
    const failedTests = tests.filter(t => !t.passed);
    let overallStatus: "pass" | "warn" | "fail";
    let overallStatusKr: string;

    if (failedTests.length === 0) {
      overallStatus = "pass";
      overallStatusKr = "통과 - 모든 테스트 정상";
    } else if (failedTests.length <= 2 && failedTests.every(t => t.nameKr.includes("품질") || t.nameKr.includes("응답"))) {
      overallStatus = "warn";
      overallStatusKr = "경고 - 일부 테스트 실패";
    } else {
      overallStatus = "fail";
      overallStatusKr = "실패 - 즉시 조치 필요";
    }

    // 문제 해결 단계 생성
    const troubleshootingSteps = this.generateTroubleshootingSteps(failedTests);

    console.log(`[진단 완료] ${overallStatusKr}`);
    console.log(`통과: ${tests.length - failedTests.length}개, 실패: ${failedTests.length}개`);

    return {
      diagnosticId: this.generateDiagnosticId(),
      sensorId,
      timestamp: new Date(),
      tests,
      overallStatus,
      overallStatusKr,
      troubleshootingSteps,
      diagnosticKr: `진단 완료 - ${tests.length}개 테스트 중 ${failedTests.length}개 실패`
    };
  }

  /**
   * 통신 테스트
   */
  private async testCommunication(sensorId: string): Promise<{
    passed: boolean;
    result: string;
    resultKr: string;
    details: {
      latency: number;
      packetLoss: number;
      signalStrength?: number;
    };
  }> {
    // 실제 통신 테스트 수행
    const latency = Math.random() * 100; // 예시
    const packetLoss = Math.random() * 5;
    const signalStrength = Math.random() * 100;

    const passed = latency < 500 && packetLoss < 5;

    return {
      passed,
      result: passed ? "Communication OK" : "Communication issues detected",
      resultKr: passed ? "통신 정상" : "통신 문제 감지",
      details: {
        latency,
        packetLoss,
        signalStrength
      }
    };
  }

  /**
   * 데이터 품질 테스트
   */
  private async testDataQuality(sensorId: string): Promise<{
    passed: boolean;
    result: string;
    resultKr: string;
  }> {
    // 최근 데이터 품질 분석
    const goodPercentage = 95 + Math.random() * 5;
    const passed = goodPercentage >= 95;

    return {
      passed,
      result: `Data quality: ${goodPercentage.toFixed(1)}% good`,
      resultKr: `데이터 품질: ${goodPercentage.toFixed(1)}% 양호`
    };
  }

  /**
   * 정확도 테스트
   */
  private async testAccuracy(sensorId: string): Promise<{
    passed: boolean;
    result: string;
    resultKr: string;
  }> {
    // 정확도 확인 (교정 이력 기반)
    const accuracy = 99 + Math.random();
    const passed = accuracy >= 99.5;

    return {
      passed,
      result: `Accuracy: ${accuracy.toFixed(2)}%`,
      resultKr: `정확도: ${accuracy.toFixed(2)}%`
    };
  }

  /**
   * 응답 시간 테스트
   */
  private async testResponseTime(sensorId: string): Promise<{
    passed: boolean;
    result: string;
    resultKr: string;
  }> {
    // 응답 시간 측정
    const responseTime = Math.random() * 10; // 초
    const passed = responseTime < 5;

    return {
      passed,
      result: `Response time: ${responseTime.toFixed(2)}s`,
      resultKr: `응답 시간: ${responseTime.toFixed(2)}초`
    };
  }

  /**
   * 전원 테스트
   */
  private async testPower(sensorId: string): Promise<{
    passed: boolean;
    result: string;
    resultKr: string;
  }> {
    // 전원 및 배터리 상태 확인
    const batteryLevel = Math.random() * 100;
    const passed = batteryLevel > 20;

    return {
      passed,
      result: `Battery level: ${batteryLevel.toFixed(0)}%`,
      resultKr: `배터리: ${batteryLevel.toFixed(0)}%`
    };
  }

  /**
   * 문제 해결 단계 생성
   */
  private generateTroubleshootingSteps(failedTests: any[]): {
    step: number;
    actionKr: string;
    expectedResultKr: string;
  }[] {
    const steps: any[] = [];

    failedTests.forEach(test => {
      if (test.nameKr.includes("통신")) {
        steps.push({
          step: steps.length + 1,
          actionKr: "네트워크 케이블 연결 확인",
          expectedResultKr: "케이블이 단단히 연결되어 있어야 함"
        });
        steps.push({
          step: steps.length + 1,
          actionKr: "센서 전원 재부팅",
          expectedResultKr: "센서가 정상적으로 재시작되어야 함"
        });
      }

      if (test.nameKr.includes("품질")) {
        steps.push({
          step: steps.length + 1,
          actionKr: "센서 교정 수행",
          expectedResultKr: "데이터 품질이 95% 이상으로 개선되어야 함"
        });
      }

      if (test.nameKr.includes("정확도")) {
        steps.push({
          step: steps.length + 1,
          actionKr: "전문 교정 기관에 교정 의뢰",
          expectedResultKr: "정확도 99.5% 이상 회복"
        });
      }

      if (test.nameKr.includes("배터리")) {
        steps.push({
          step: steps.length + 1,
          actionKr: "배터리 교체 또는 전원 연결 확인",
          expectedResultKr: "배터리 레벨 100%로 회복"
        });
      }
    });

    return steps;
  }

  private generateDiagnosticId(): string {
    return `DIAG-${Date.now()}`;
  }
}
```

---

## 결론

WIA Cryo Monitoring Standard는 센서의 전체 생애주기를 관리하는 포괄적인 시스템을 제공합니다.

### 핵심 특징

1. **생애주기 관리**: 구매부터 폐기까지 완전한 추적
2. **자동 교정**: 스케줄 관리 및 알림
3. **건강 모니터링**: 실시간 센서 건강 점수
4. **자동 진단**: 문제 감지 및 해결 가이드

### 다음 장 예고

다음 장에서는 **알림 시스템**을 다룹니다:
- 알림 규칙 및 에스컬레이션
- 다채널 알림 전송
- 한국어 알림 메시지
- 자동 조치 시스템

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
