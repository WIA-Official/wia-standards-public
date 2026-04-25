# 06. 알림 시스템
## Alerting System - Rules, Escalation, Notification Channels

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [알림 시스템 개요](#알림-시스템-개요)
2. [알림 규칙 엔진](#알림-규칙-엔진)
3. [에스컬레이션 정책](#에스컬레이션-정책)
4. [알림 채널 관리](#알림-채널-관리)
5. [한국어 알림 메시지](#한국어-알림-메시지)
6. [알림 통계 및 분석](#알림-통계-및-분석)
7. [자동 조치 시스템](#자동-조치-시스템)
8. [알림 피로도 방지](#알림-피로도-방지)

---

## 알림 시스템 개요

### 알림 아키텍처

```typescript
/**
 * WIA Cryo Monitoring 알림 시스템 아키텍처
 */
interface AlertingSystemArchitecture {
  // 규칙 엔진
  ruleEngine: {
    name: string;
    nameKr: string;
    evaluation: "real-time" | "periodic";
    evaluationKr: string;
    scalability: string;
  };

  // 알림 큐
  alertQueue: {
    type: "priority" | "fifo";
    typeKr: string;
    maxSize: number;
    processingRate: number;
    queueKr: string;
  };

  // 알림 라우터
  router: {
    strategy: "round-robin" | "priority" | "intelligent";
    strategyKr: string;
    channels: string[];
    routerKr: string;
  };

  // 알림 저장소
  storage: {
    database: string;
    databaseKr: string;
    retention: number;            // 보관 기간 (일)
    archiving: boolean;
    storageKr: string;
  };

  // 에스컬레이션 관리자
  escalationManager: {
    enabled: boolean;
    maxLevels: number;
    defaultDelay: number;         // 분
    escalationKr: string;
  };

  archKr: string;
}

const alertingArchitecture: AlertingSystemArchitecture = {
  ruleEngine: {
    name: "WIA Alert Rule Engine",
    nameKr: "WIA 알림 규칙 엔진",
    evaluation: "real-time",
    evaluationKr: "실시간 평가",
    scalability: "Horizontally scalable, supports 100,000+ rules"
  },

  alertQueue: {
    type: "priority",
    typeKr: "우선순위 큐",
    maxSize: 100000,
    processingRate: 10000,        // 초당 10,000개
    queueKr: "긴급도 기반 우선순위 처리"
  },

  router: {
    strategy: "intelligent",
    strategyKr: "지능형 라우팅",
    channels: ["email", "sms", "push", "webhook", "voice", "slack", "teams"],
    routerKr: "상황별 최적 채널 자동 선택"
  },

  storage: {
    database: "PostgreSQL + TimescaleDB",
    databaseKr: "PostgreSQL 시계열 데이터베이스",
    retention: 365,               // 1년
    archiving: true,
    storageKr: "1년 온라인 보관, 이후 아카이브"
  },

  escalationManager: {
    enabled: true,
    maxLevels: 5,
    defaultDelay: 15,             // 15분
    escalationKr: "최대 5단계 에스컬레이션, 기본 15분 간격"
  },

  archKr: "확장 가능한 실시간 알림 아키텍처"
};
```

---

## 알림 규칙 엔진

### 규칙 정의 및 관리

```typescript
/**
 * 알림 규칙 빌더
 */
class AlertRuleBuilder {
  private rule: Partial<AlertRule>;

  constructor() {
    this.rule = {
      ruleId: this.generateRuleId(),
      name: "",
      nameKr: "",
      enabled: true
    };
  }

  /**
   * 규칙 이름 설정
   */
  withName(name: string, nameKr: string): this {
    this.rule.name = name;
    this.rule.nameKr = nameKr;
    return this;
  }

  /**
   * 조건 설정
   */
  withCondition(condition: {
    parameter: "temperature" | "level" | "pressure" | "humidity" | "custom";
    operator: ">" | "<" | "==" | "!=" | "between" | "outside";
    value: number | [number, number];
    duration: number;             // 지속 시간 (초)
    conditionKr: string;
  }): this {
    this.rule.condition = condition;
    return this;
  }

  /**
   * 심각도 설정
   */
  withSeverity(severity: AlertSeverity, severityKr: string): this {
    this.rule.severity = severity;
    this.rule.severityKr = severityKr;
    return this;
  }

  /**
   * 대상 센서 설정
   */
  forSensors(sensorIds: string[]): this {
    this.rule.applicableSensors = sensorIds;
    return this;
  }

  /**
   * 대상 탱크 설정
   */
  forTanks(tankIds: string[]): this {
    this.rule.applicableTanks = tankIds;
    return this;
  }

  /**
   * 대상 시설 설정
   */
  forFacility(facilityId: string): this {
    this.rule.applicableFacility = facilityId;
    return this;
  }

  /**
   * 조치 설정
   */
  withActions(actions: {
    notification: boolean;
    logging: boolean;
    automation?: string;          // 자동 조치 스크립트
    actionKr: string;
  }): this {
    this.rule.action = actions;
    return this;
  }

  /**
   * 알림 채널 설정
   */
  toChannels(channels: {
    channelId: string;
    type: "email" | "sms" | "push" | "webhook" | "voice";
    typeKr: string;
    recipients: string[];
  }[]): this {
    this.rule.notificationChannels = channels;
    return this;
  }

  /**
   * 시간 제약 설정
   */
  withTimeConstraints(constraints: {
    activeHours?: {
      start: string;              // "HH:MM"
      end: string;
      hoursKr: string;
    };
    activeDays?: string[];        // ["monday", "tuesday", ...]
    timezone?: string;            // "Asia/Seoul"
    constraintsKr: string;
  }): this {
    this.rule.timeConstraints = constraints;
    return this;
  }

  /**
   * 에스컬레이션 정책 설정
   */
  withEscalation(escalation: {
    enabled: boolean;
    policyId: string;
    escalationKr: string;
  }): this {
    this.rule.escalation = escalation;
    return this;
  }

  /**
   * 규칙 빌드
   */
  build(): AlertRule {
    // 유효성 검사
    if (!this.rule.name || !this.rule.nameKr) {
      throw new Error("규칙 이름을 설정해야 합니다");
    }
    if (!this.rule.condition) {
      throw new Error("조건을 설정해야 합니다");
    }
    if (!this.rule.severity) {
      throw new Error("심각도를 설정해야 합니다");
    }

    return this.rule as AlertRule;
  }

  private generateRuleId(): string {
    return `RULE-${Date.now()}-${Math.random().toString(36).substr(2, 9).toUpperCase()}`;
  }
}

// 사용 예시: 온도 위험 알림 규칙
const temperatureCriticalRule = new AlertRuleBuilder()
  .withName("Critical Temperature Alert", "온도 위험 알림")
  .withCondition({
    parameter: "temperature",
    operator: ">",
    value: -160,
    duration: 300,                // 5분 지속
    conditionKr: "온도가 -160°C 초과하고 5분간 지속될 때"
  })
  .withSeverity("critical", "위험")
  .forTanks(["TANK-A1", "TANK-A2"])
  .withActions({
    notification: true,
    logging: true,
    automation: "auto-refill-ln2.sh",
    actionKr: "알림 전송 + 로깅 + 자동 질소 보충"
  })
  .toChannels([
    {
      channelId: "CH-SMS-001",
      type: "sms",
      typeKr: "문자메시지",
      recipients: ["+82-10-1234-5678", "+82-10-9876-5432"]
    },
    {
      channelId: "CH-EMAIL-001",
      type: "email",
      typeKr: "이메일",
      recipients: ["manager@facility.kr", "engineer@facility.kr"]
    },
    {
      channelId: "CH-VOICE-001",
      type: "voice",
      typeKr: "음성 전화",
      recipients: ["+82-10-1234-5678"]
    }
  ])
  .withTimeConstraints({
    timezone: "Asia/Seoul",
    constraintsKr: "한국 시간대 기준"
  })
  .withEscalation({
    enabled: true,
    policyId: "ESC-CRITICAL-001",
    escalationKr: "3단계 에스컬레이션 정책 적용"
  })
  .build();

console.log(`[알림 규칙 생성] ${temperatureCriticalRule.nameKr}`);
console.log(`조건: ${temperatureCriticalRule.condition.conditionKr}`);
console.log(`조치: ${temperatureCriticalRule.action.actionKr}`);
```

### 규칙 평가 엔진

```typescript
/**
 * 알림 규칙 평가 엔진
 */
class AlertRuleEvaluationEngine {
  private rules: Map<string, AlertRule> = new Map();
  private dataBuffer: Map<string, SensorReading[]> = new Map();

  /**
   * 규칙 등록
   */
  registerRule(rule: AlertRule): void {
    this.rules.set(rule.ruleId, rule);
    console.log(`[규칙 등록] ${rule.nameKr} (${rule.ruleId})`);
  }

  /**
   * 규칙 제거
   */
  unregisterRule(ruleId: string): void {
    const rule = this.rules.get(ruleId);
    if (rule) {
      this.rules.delete(ruleId);
      console.log(`[규칙 제거] ${rule.nameKr}`);
    }
  }

  /**
   * 센서 데이터 평가
   */
  async evaluate(reading: SensorReading): Promise<Alert[]> {
    const alerts: Alert[] = [];

    // 데이터 버퍼에 추가
    if (!this.dataBuffer.has(reading.sensorId)) {
      this.dataBuffer.set(reading.sensorId, []);
    }
    const buffer = this.dataBuffer.get(reading.sensorId)!;
    buffer.push(reading);

    // 버퍼 크기 제한 (최근 1000개)
    if (buffer.length > 1000) {
      buffer.shift();
    }

    // 모든 규칙 평가
    for (const rule of this.rules.values()) {
      if (!rule.enabled) continue;

      // 규칙 적용 대상 확인
      if (!this.isApplicable(rule, reading)) continue;

      // 시간 제약 확인
      if (!this.isWithinTimeConstraints(rule)) continue;

      // 조건 평가
      const conditionMet = this.evaluateCondition(rule, reading, buffer);

      if (conditionMet) {
        // 알림 생성
        const alert = await this.createAlert(rule, reading);
        alerts.push(alert);

        // 조치 실행
        await this.executeActions(rule, alert);

        console.log(`[알림 발생] ${alert.titleKr} - ${alert.messageKr}`);
      }
    }

    return alerts;
  }

  /**
   * 규칙 적용 대상 확인
   */
  private isApplicable(rule: AlertRule, reading: SensorReading): boolean {
    if (rule.applicableSensors && rule.applicableSensors.length > 0) {
      return rule.applicableSensors.includes(reading.sensorId);
    }
    // 탱크 또는 시설 기반 필터링 로직...
    return true;
  }

  /**
   * 시간 제약 확인
   */
  private isWithinTimeConstraints(rule: AlertRule): boolean {
    if (!rule.timeConstraints) return true;

    const now = new Date();

    // 활성 시간 확인
    if (rule.timeConstraints.activeHours) {
      const currentTime = `${now.getHours().toString().padStart(2, "0")}:${now.getMinutes().toString().padStart(2, "0")}`;
      const start = rule.timeConstraints.activeHours.start;
      const end = rule.timeConstraints.activeHours.end;

      if (currentTime < start || currentTime > end) {
        return false;
      }
    }

    // 활성 요일 확인
    if (rule.timeConstraints.activeDays && rule.timeConstraints.activeDays.length > 0) {
      const days = ["sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"];
      const currentDay = days[now.getDay()];

      if (!rule.timeConstraints.activeDays.includes(currentDay)) {
        return false;
      }
    }

    return true;
  }

  /**
   * 조건 평가
   */
  private evaluateCondition(
    rule: AlertRule,
    reading: SensorReading,
    buffer: SensorReading[]
  ): boolean {
    const condition = rule.condition;

    // 지속 시간 확인
    const now = Date.now();
    const durationMs = condition.duration * 1000;
    const relevantReadings = buffer.filter(r =>
      now - new Date(r.timestamp).getTime() < durationMs
    );

    if (relevantReadings.length === 0) return false;

    // 모든 읽기가 조건을 만족하는지 확인
    return relevantReadings.every(r => {
      switch (condition.operator) {
        case ">":
          return r.value > (condition.value as number);
        case "<":
          return r.value < (condition.value as number);
        case "==":
          return r.value === condition.value;
        case "!=":
          return r.value !== condition.value;
        case "between":
          const [min, max] = condition.value as [number, number];
          return r.value >= min && r.value <= max;
        case "outside":
          const [min2, max2] = condition.value as [number, number];
          return r.value < min2 || r.value > max2;
        default:
          return false;
      }
    });
  }

  /**
   * 알림 생성
   */
  private async createAlert(rule: AlertRule, reading: SensorReading): Promise<Alert> {
    const alert: Alert = {
      alertId: this.generateAlertId(),
      ruleId: rule.ruleId,
      sensorId: reading.sensorId,
      severity: rule.severity,
      severityKr: rule.severityKr,
      status: "active",
      statusKr: "활성 - 조치 필요",
      title: rule.name,
      titleKr: rule.nameKr,
      message: `Sensor ${reading.sensorId}: ${rule.name}`,
      messageKr: `센서 ${reading.sensorId}: ${rule.nameKr}`,
      measurement: {
        parameter: rule.condition.parameter,
        parameterKr: rule.condition.parameter === "temperature" ? "온도" : rule.condition.parameter,
        value: reading.value,
        unit: reading.unit,
        unitKr: reading.unitKr,
        threshold: typeof rule.condition.value === "number" ? rule.condition.value : rule.condition.value[0],
        deviation: 0,
        measurementKr: `${reading.value}${reading.unitKr} (임계값: ${rule.condition.value})`
      },
      occurrence: {
        firstOccurred: new Date(),
        lastOccurred: new Date(),
        count: 1,
        duration: 0,
        occurrenceKr: "최초 발생"
      },
      handling: {
        handlingKr: "미처리"
      },
      escalation: {
        level: 0,
        escalatedTo: [],
        escalationKr: "에스컬레이션 대기"
      },
      notifications: [],
      automation: {
        executed: false,
        automationKr: "자동 조치 대기"
      },
      metadata: {
        priority: this.calculatePriority(rule.severity),
        tags: [],
        relatedAlerts: [],
        attachments: [],
        metadataKr: `우선순위: ${this.calculatePriority(rule.severity)}`
      },
      createdAt: new Date(),
      updatedAt: new Date()
    };

    // 알림 저장
    await this.saveAlert(alert);

    return alert;
  }

  /**
   * 조치 실행
   */
  private async executeActions(rule: AlertRule, alert: Alert): Promise<void> {
    // 1. 로깅
    if (rule.action.logging) {
      await this.logAlert(alert);
    }

    // 2. 알림 전송
    if (rule.action.notification && rule.notificationChannels) {
      await this.sendNotifications(rule.notificationChannels, alert);
    }

    // 3. 자동 조치 실행
    if (rule.action.automation) {
      await this.executeAutomation(rule.action.automation, alert);
    }

    // 4. 에스컬레이션 스케줄
    if (rule.escalation?.enabled) {
      await this.scheduleEscalation(rule.escalation.policyId, alert);
    }
  }

  /**
   * 우선순위 계산
   */
  private calculatePriority(severity: AlertSeverity): number {
    const priorityMap = {
      emergency: 1,
      critical: 2,
      warning: 3,
      info: 4
    };
    return priorityMap[severity] || 5;
  }

  private generateAlertId(): string {
    return `ALERT-${Date.now()}-${Math.random().toString(36).substr(2, 9).toUpperCase()}`;
  }

  private async saveAlert(alert: Alert): Promise<void> {
    // 데이터베이스 저장
    console.log(`[알림 저장] ${alert.alertId}`);
  }

  private async logAlert(alert: Alert): Promise<void> {
    console.log(`[알림 로그] ${alert.titleKr} - ${alert.messageKr}`);
  }

  private async sendNotifications(channels: any[], alert: Alert): Promise<void> {
    // 알림 전송 (다음 섹션에서 상세 구현)
    console.log(`[알림 전송] ${channels.length}개 채널`);
  }

  private async executeAutomation(script: string, alert: Alert): Promise<void> {
    console.log(`[자동 조치] ${script} 실행`);
    // 자동화 스크립트 실행
  }

  private async scheduleEscalation(policyId: string, alert: Alert): Promise<void> {
    console.log(`[에스컬레이션] 정책 ${policyId} 스케줄`);
  }
}
```

---

## 에스컬레이션 정책

### 에스컬레이션 관리

```typescript
/**
 * 에스컬레이션 정책 관리
 */
class EscalationPolicyManager {
  private policies: Map<string, EscalationPolicy> = new Map();

  /**
   * 에스컬레이션 정책 생성
   */
  createPolicy(policy: {
    policyId: string;
    name: string;
    nameKr: string;
    levels: {
      level: number;
      delay: number;              // 분
      recipients: string[];
      channels: ("email" | "sms" | "push" | "voice")[];
      levelKr: string;
    }[];
    maxRetries: number;
    policyKr: string;
  }): EscalationPolicy {
    const escalationPolicy: EscalationPolicy = {
      policyId: policy.policyId,
      levels: policy.levels,
      maxRetries: policy.maxRetries,
      policyKr: policy.policyKr
    };

    this.policies.set(policy.policyId, escalationPolicy);

    console.log(`[에스컬레이션 정책 생성] ${policy.nameKr}`);
    console.log(`총 ${policy.levels.length}단계, 최대 ${policy.maxRetries}회 재시도`);

    return escalationPolicy;
  }

  /**
   * 에스컬레이션 실행
   */
  async executeEscalation(alert: Alert, policyId: string): Promise<{
    success: boolean;
    currentLevel: number;
    nextLevel: number | null;
    nextEscalation: Date | null;
    escalationKr: string;
  }> {
    const policy = this.policies.get(policyId);
    if (!policy) {
      throw new Error(`에스컬레이션 정책을 찾을 수 없습니다: ${policyId}`);
    }

    const currentLevel = alert.escalation.level;
    const nextLevel = currentLevel + 1;

    if (nextLevel > policy.levels.length) {
      console.warn(`[에스컬레이션 한계] 최대 레벨 ${policy.levels.length} 도달`);
      return {
        success: false,
        currentLevel,
        nextLevel: null,
        nextEscalation: null,
        escalationKr: "에스컬레이션 최대 레벨 도달"
      };
    }

    const levelConfig = policy.levels[nextLevel - 1];

    // 에스컬레이션 알림 전송
    await this.sendEscalationNotifications(alert, levelConfig);

    // 알림 업데이트
    alert.escalation.level = nextLevel;
    alert.escalation.escalatedTo = levelConfig.recipients;
    alert.escalation.escalatedAt = new Date();
    alert.status = "escalated";
    alert.statusKr = "에스컬레이션됨";
    alert.escalation.escalationKr = `레벨 ${nextLevel} 에스컬레이션 - ${levelConfig.levelKr}`;

    // 다음 에스컬레이션 스케줄
    let nextEscalation: Date | null = null;
    if (nextLevel < policy.levels.length) {
      const nextLevelConfig = policy.levels[nextLevel];
      nextEscalation = new Date();
      nextEscalation.setMinutes(nextEscalation.getMinutes() + nextLevelConfig.delay);

      await this.scheduleNextEscalation(alert, policyId, nextEscalation);
    }

    console.log(`[에스컬레이션 실행] 레벨 ${nextLevel} - ${levelConfig.levelKr}`);
    console.log(`수신자: ${levelConfig.recipients.join(", ")}`);

    return {
      success: true,
      currentLevel: nextLevel,
      nextLevel: nextLevel < policy.levels.length ? nextLevel + 1 : null,
      nextEscalation,
      escalationKr: `레벨 ${nextLevel} 에스컬레이션 완료`
    };
  }

  /**
   * 에스컬레이션 알림 전송
   */
  private async sendEscalationNotifications(
    alert: Alert,
    levelConfig: EscalationPolicy["levels"][0]
  ): Promise<void> {
    console.log(`[에스컬레이션 알림] ${levelConfig.levelKr}`);

    for (const channel of levelConfig.channels) {
      for (const recipient of levelConfig.recipients) {
        await this.sendNotification({
          channel,
          recipient,
          alert,
          escalation: true,
          level: levelConfig.level
        });
      }
    }
  }

  /**
   * 다음 에스컬레이션 스케줄
   */
  private async scheduleNextEscalation(
    alert: Alert,
    policyId: string,
    nextTime: Date
  ): Promise<void> {
    console.log(`[에스컬레이션 스케줄] ${nextTime.toLocaleString("ko-KR")}에 다음 레벨 실행 예정`);

    // 스케줄러에 등록 (실제 구현에서는 cron job 또는 메시지 큐 사용)
    setTimeout(async () => {
      // 알림이 여전히 해결되지 않았는지 확인
      const currentAlert = await this.getAlert(alert.alertId);
      if (currentAlert.status === "escalated" || currentAlert.status === "active") {
        await this.executeEscalation(currentAlert, policyId);
      }
    }, nextTime.getTime() - Date.now());
  }

  private async sendNotification(params: any): Promise<void> {
    // 알림 전송 로직
  }

  private async getAlert(alertId: string): Promise<Alert> {
    // 알림 조회 로직
    return {} as Alert;
  }
}

// 사용 예시: 3단계 에스컬레이션 정책
const criticalEscalationPolicy = new EscalationPolicyManager().createPolicy({
  policyId: "ESC-CRITICAL-001",
  name: "Critical Alert Escalation",
  nameKr: "위험 알림 에스컬레이션",
  levels: [
    {
      level: 1,
      delay: 0,                     // 즉시
      recipients: ["duty-engineer@facility.kr"],
      channels: ["sms", "push"],
      levelKr: "1단계: 당직 엔지니어 (즉시)"
    },
    {
      level: 2,
      delay: 15,                    // 15분 후
      recipients: ["supervisor@facility.kr", "manager@facility.kr"],
      channels: ["sms", "email", "voice"],
      levelKr: "2단계: 관리 감독자 (15분 후)"
    },
    {
      level: 3,
      delay: 30,                    // 30분 후
      recipients: ["director@facility.kr", "ceo@facility.kr"],
      channels: ["sms", "voice"],
      levelKr: "3단계: 시설 책임자 및 CEO (30분 후)"
    }
  ],
  maxRetries: 3,
  policyKr: "3단계 위험 알림 에스컬레이션 정책"
});
```

---

## 알림 채널 관리

### 다채널 알림 전송

```typescript
/**
 * 알림 채널 관리자
 */
class NotificationChannelManager {
  /**
   * 이메일 알림
   */
  async sendEmail(params: {
    recipients: string[];
    subject: string;
    subjectKr: string;
    body: string;
    bodyKr: string;
    priority: "high" | "normal" | "low";
    alert: Alert;
  }): Promise<{
    sent: boolean;
    messageId: string;
    sentKr: string;
  }> {
    console.log(`[이메일 알림] 수신자: ${params.recipients.join(", ")}`);
    console.log(`제목: ${params.subjectKr}`);

    // 실제 이메일 전송 (예: SendGrid, AWS SES, Gmail API)
    const messageId = `EMAIL-${Date.now()}`;

    // HTML 이메일 템플릿
    const htmlBody = this.generateEmailTemplate(params.alert, params.bodyKr);

    // 이메일 전송 로직...

    return {
      sent: true,
      messageId,
      sentKr: `이메일 전송 완료 - ${params.recipients.length}명`
    };
  }

  /**
   * SMS 알림
   */
  async sendSMS(params: {
    recipients: string[];
    message: string;
    messageKr: string;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    messageIds: string[];
    sentKr: string;
  }> {
    console.log(`[SMS 알림] 수신자: ${params.recipients.length}명`);
    console.log(`메시지: ${params.messageKr}`);

    const messageIds: string[] = [];

    // 한국 SMS 서비스 (예: 알리고, 카카오 알림톡, NHN Cloud SMS)
    for (const recipient of params.recipients) {
      const messageId = await this.sendSMSToRecipient(recipient, params.messageKr);
      messageIds.push(messageId);
    }

    return {
      sent: true,
      messageIds,
      sentKr: `SMS 전송 완료 - ${params.recipients.length}명`
    };
  }

  /**
   * 푸시 알림
   */
  async sendPush(params: {
    recipients: string[];          // 사용자 ID 또는 디바이스 토큰
    title: string;
    titleKr: string;
    body: string;
    bodyKr: string;
    data: any;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    successCount: number;
    failureCount: number;
    sentKr: string;
  }> {
    console.log(`[푸시 알림] 수신자: ${params.recipients.length}명`);
    console.log(`제목: ${params.titleKr}`);

    let successCount = 0;
    let failureCount = 0;

    // FCM (Firebase Cloud Messaging) 또는 APNs (Apple Push Notification service)
    for (const recipient of params.recipients) {
      try {
        await this.sendPushToDevice(recipient, {
          title: params.titleKr,
          body: params.bodyKr,
          data: params.data
        });
        successCount++;
      } catch (error) {
        console.error(`푸시 전송 실패: ${recipient}`, error);
        failureCount++;
      }
    }

    return {
      sent: true,
      successCount,
      failureCount,
      sentKr: `푸시 전송 완료 - 성공: ${successCount}명, 실패: ${failureCount}명`
    };
  }

  /**
   * 음성 전화 알림
   */
  async sendVoiceCall(params: {
    recipients: string[];
    message: string;
    messageKr: string;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    callIds: string[];
    sentKr: string;
  }> {
    console.log(`[음성 전화] 수신자: ${params.recipients.length}명`);
    console.log(`메시지: ${params.messageKr}`);

    const callIds: string[] = [];

    // TTS (Text-to-Speech) 음성 전화 (예: Twilio, AWS Connect)
    for (const recipient of params.recipients) {
      const callId = await this.makeVoiceCall(recipient, params.messageKr);
      callIds.push(callId);
    }

    return {
      sent: true,
      callIds,
      sentKr: `음성 전화 발신 완료 - ${params.recipients.length}명`
    };
  }

  /**
   * Webhook 알림
   */
  async sendWebhook(params: {
    url: string;
    method: "POST" | "PUT";
    headers?: Record<string, string>;
    payload: any;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    statusCode: number;
    response: any;
    sentKr: string;
  }> {
    console.log(`[Webhook 알림] URL: ${params.url}`);

    const response = await fetch(params.url, {
      method: params.method,
      headers: {
        "Content-Type": "application/json",
        ...params.headers
      },
      body: JSON.stringify(params.payload)
    });

    return {
      sent: response.ok,
      statusCode: response.status,
      response: await response.json(),
      sentKr: `Webhook 전송 완료 - HTTP ${response.status}`
    };
  }

  /**
   * Slack 알림
   */
  async sendSlack(params: {
    webhookUrl: string;
    channel: string;
    message: string;
    messageKr: string;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    sentKr: string;
  }> {
    console.log(`[Slack 알림] 채널: ${params.channel}`);

    const slackMessage = {
      channel: params.channel,
      username: "WIA Cryo Monitor",
      icon_emoji: ":warning:",
      attachments: [
        {
          color: this.getSeverityColor(params.alert.severity),
          title: params.alert.titleKr,
          text: params.messageKr,
          fields: [
            {
              title: "심각도",
              value: params.alert.severityKr,
              short: true
            },
            {
              title: "상태",
              value: params.alert.statusKr,
              short: true
            },
            {
              title: "발생 시간",
              value: params.alert.occurrence.firstOccurred.toLocaleString("ko-KR"),
              short: true
            }
          ],
          footer: "WIA Cryo Monitoring System",
          ts: Math.floor(Date.now() / 1000)
        }
      ]
    };

    await fetch(params.webhookUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(slackMessage)
    });

    return {
      sent: true,
      sentKr: "Slack 알림 전송 완료"
    };
  }

  /**
   * Microsoft Teams 알림
   */
  async sendTeams(params: {
    webhookUrl: string;
    message: string;
    messageKr: string;
    alert: Alert;
  }): Promise<{
    sent: boolean;
    sentKr: string;
  }> {
    console.log(`[Teams 알림] 전송 중...`);

    const teamsMessage = {
      "@type": "MessageCard",
      "@context": "https://schema.org/extensions",
      summary: params.alert.titleKr,
      themeColor: this.getSeverityColor(params.alert.severity),
      title: params.alert.titleKr,
      sections: [
        {
          activityTitle: "WIA Cryo Monitoring System",
          activitySubtitle: params.alert.occurrence.firstOccurred.toLocaleString("ko-KR"),
          facts: [
            {
              name: "심각도:",
              value: params.alert.severityKr
            },
            {
              name: "상태:",
              value: params.alert.statusKr
            },
            {
              name: "메시지:",
              value: params.messageKr
            }
          ]
        }
      ]
    };

    await fetch(params.webhookUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(teamsMessage)
    });

    return {
      sent: true,
      sentKr: "Teams 알림 전송 완료"
    };
  }

  /**
   * 이메일 템플릿 생성
   */
  private generateEmailTemplate(alert: Alert, bodyKr: string): string {
    return `
      <!DOCTYPE html>
      <html>
      <head>
        <meta charset="UTF-8">
        <style>
          body { font-family: 'Noto Sans KR', sans-serif; }
          .header { background: #f44336; color: white; padding: 20px; }
          .content { padding: 20px; }
          .footer { background: #f5f5f5; padding: 10px; text-align: center; }
        </style>
      </head>
      <body>
        <div class="header">
          <h1>🚨 ${alert.titleKr}</h1>
        </div>
        <div class="content">
          <p><strong>심각도:</strong> ${alert.severityKr}</p>
          <p><strong>발생 시간:</strong> ${alert.occurrence.firstOccurred.toLocaleString("ko-KR")}</p>
          <p><strong>메시지:</strong> ${bodyKr}</p>
        </div>
        <div class="footer">
          <p>WIA Cryo Monitoring System</p>
          <p>弘益人間 (홍익인간) - Benefit All Humanity</p>
        </div>
      </body>
      </html>
    `;
  }

  /**
   * 심각도별 색상
   */
  private getSeverityColor(severity: AlertSeverity): string {
    const colorMap = {
      emergency: "#f44336",    // 빨강
      critical: "#ff9800",     // 주황
      warning: "#ffeb3b",      // 노랑
      info: "#2196f3"          // 파랑
    };
    return colorMap[severity];
  }

  private async sendSMSToRecipient(recipient: string, message: string): Promise<string> {
    // SMS 전송 로직
    return `SMS-${Date.now()}`;
  }

  private async sendPushToDevice(deviceToken: string, payload: any): Promise<void> {
    // 푸시 전송 로직
  }

  private async makeVoiceCall(phoneNumber: string, message: string): Promise<string> {
    // 음성 전화 발신 로직
    return `CALL-${Date.now()}`;
  }
}
```

---

## 한국어 알림 메시지

### 알림 메시지 템플릿

```typescript
/**
 * 한국어 알림 메시지 템플릿
 */
class KoreanAlertMessageTemplates {
  /**
   * 온도 알림 메시지
   */
  static temperature(params: {
    severity: AlertSeverity;
    tankName: string;
    currentTemp: number;
    threshold: number;
    duration: number;
  }): {
    title: string;
    message: string;
  } {
    const templates = {
      emergency: {
        title: `🚨 긴급! ${params.tankName} 온도 비상 상황`,
        message: `${params.tankName}의 온도가 ${params.currentTemp.toFixed(1)}°C로 측정되었습니다. 이는 긴급 임계값 ${params.threshold}°C를 초과하며, ${params.duration}초 동안 지속되고 있습니다. 즉시 현장 확인 및 조치가 필요합니다!`
      },
      critical: {
        title: `⚠️ 위험! ${params.tankName} 온도 이상`,
        message: `${params.tankName}의 온도가 ${params.currentTemp.toFixed(1)}°C로 위험 수준입니다. 임계값 ${params.threshold}°C를 초과하여 ${params.duration}초간 유지되고 있습니다. 빠른 조치가 필요합니다.`
      },
      warning: {
        title: `⚡ 주의! ${params.tankName} 온도 경고`,
        message: `${params.tankName}의 온도가 ${params.currentTemp.toFixed(1)}°C로 경고 수준입니다. 임계값 ${params.threshold}°C에 근접하여 ${params.duration}초간 유지되고 있습니다. 모니터링을 강화해 주세요.`
      },
      info: {
        title: `ℹ️ 정보: ${params.tankName} 온도 알림`,
        message: `${params.tankName}의 온도가 ${params.currentTemp.toFixed(1)}°C입니다. 참고용 정보입니다.`
      }
    };

    return templates[params.severity];
  }

  /**
   * 액체질소 레벨 알림
   */
  static ln2Level(params: {
    severity: AlertSeverity;
    tankName: string;
    currentLevel: number;
    threshold: number;
    estimatedEmpty: Date;
  }): {
    title: string;
    message: string;
  } {
    const daysUntilEmpty = Math.floor(
      (params.estimatedEmpty.getTime() - Date.now()) / (1000 * 60 * 60 * 24)
    );

    const templates = {
      emergency: {
        title: `🚨 긴급! ${params.tankName} 질소 부족`,
        message: `${params.tankName}의 액체질소 레벨이 ${params.currentLevel.toFixed(1)}%로 매우 낮습니다! 예상 소진 시간은 ${params.estimatedEmpty.toLocaleString("ko-KR")}입니다. 즉시 보충이 필요합니다!`
      },
      critical: {
        title: `⚠️ 위험! ${params.tankName} 질소 레벨 낮음`,
        message: `${params.tankName}의 액체질소 레벨이 ${params.currentLevel.toFixed(1)}%입니다. 약 ${daysUntilEmpty}일 후 소진 예상됩니다. 보충을 준비해 주세요.`
      },
      warning: {
        title: `⚡ 주의! ${params.tankName} 질소 보충 필요`,
        message: `${params.tankName}의 액체질소 레벨이 ${params.currentLevel.toFixed(1)}%로 낮아지고 있습니다. ${daysUntilEmpty}일 이내에 보충이 필요합니다.`
      },
      info: {
        title: `ℹ️ 정보: ${params.tankName} 질소 레벨`,
        message: `${params.tankName}의 액체질소 레벨이 ${params.currentLevel.toFixed(1)}%입니다.`
      }
    };

    return templates[params.severity];
  }

  /**
   * 센서 오류 알림
   */
  static sensorError(params: {
    sensorName: string;
    errorType: string;
    errorTypeKr: string;
    location: string;
  }): {
    title: string;
    message: string;
  } {
    return {
      title: `⚠️ 센서 오류: ${params.sensorName}`,
      message: `${params.location}의 ${params.sensorName}에서 오류가 발생했습니다.\n오류 유형: ${params.errorTypeKr}\n센서 상태를 확인하고 필요시 엔지니어에게 연락하세요.`
    };
  }

  /**
   * 교정 만료 알림
   */
  static calibrationDue(params: {
    sensorName: string;
    dueDate: Date;
    daysOverdue: number;
  }): {
    title: string;
    message: string;
  } {
    if (params.daysOverdue > 0) {
      return {
        title: `⚠️ 교정 기한 초과: ${params.sensorName}`,
        message: `${params.sensorName}의 교정 기한이 ${params.daysOverdue}일 초과되었습니다. 즉시 교정을 실시해 주세요.\n교정 예정일: ${params.dueDate.toLocaleDateString("ko-KR")}`
      };
    } else {
      return {
        title: `ℹ️ 교정 예정: ${params.sensorName}`,
        message: `${params.sensorName}의 교정 예정일이 ${Math.abs(params.daysOverdue)}일 후입니다.\n교정 예정일: ${params.dueDate.toLocaleDateString("ko-KR")}`
      };
    }
  }

  /**
   * 정전 알림
   */
  static powerFailure(params: {
    facilityName: string;
    affectedTanks: string[];
    backupPowerStatus: "active" | "unavailable";
  }): {
    title: string;
    message: string;
  } {
    return {
      title: `🚨 긴급! ${params.facilityName} 정전 발생`,
      message: `${params.facilityName}에 정전이 발생했습니다!\n영향받는 탱크: ${params.affectedTanks.join(", ")}\n백업 전원: ${params.backupPowerStatus === "active" ? "정상 작동 중" : "사용 불가"}\n${params.backupPowerStatus === "unavailable" ? "즉시 현장 확인이 필요합니다!" : "백업 전원으로 정상 작동 중입니다."}`
    };
  }
}
```

---

## 결론

WIA Cryo Monitoring Standard는 포괄적인 다채널 알림 시스템을 제공합니다.

### 핵심 특징

1. **유연한 규칙 엔진**: 복잡한 조건 설정 가능
2. **다단계 에스컬레이션**: 자동 상향 보고
3. **다채널 알림**: 이메일, SMS, 푸시, 음성, Slack, Teams
4. **한국어 지원**: 완전한 한국어 메시지

### 다음 장 예고

다음 장에서는 **보안**을 다룹니다:
- 데이터 암호화
- 접근 제어
- 감사 로깅
- 규제 준수

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
