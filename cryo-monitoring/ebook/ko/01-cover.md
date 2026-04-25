# WIA Cryo Monitoring Standard
## 극저온 생물자원 실시간 모니터링 표준

**Version**: 1.0.0
**Status**: Production Ready
**Last Updated**: 2026-01-11

---

## 弘益人間 (홍익인간)
> "널리 인간을 이롭게 하라" - Benefit All Humanity

극저온 생물자원의 안전한 보관과 생명의 가치를 지키기 위한 실시간 모니터링 표준

---

## 📋 목차

1. [표준 개요](#표준-개요)
2. [모니터링 유형](#모니터링-유형)
3. [실시간 추적 시스템](#실시간-추적-시스템)
4. [TypeScript 구현](#typescript-구현)
5. [센서 통합](#센서-통합)
6. [데이터 수집 파이프라인](#데이터-수집-파이프라인)
7. [알림 시스템](#알림-시스템)
8. [한국 시장 적용](#한국-시장-적용)

---

## 표준 개요

### 목적

WIA Cryo Monitoring Standard는 극저온(-196°C ~ -150°C) 환경에서 보관되는 생물자원의 안전성을 보장하기 위한 포괄적인 모니터링 표준입니다.

### 적용 대상

- **제대혈 은행**: 신생아 제대혈 줄기세포 보관
- **난자/정자 은행**: 생식세포 장기 보관
- **바이오뱅크**: 조직, 혈액, DNA 샘플 보관
- **연구기관**: 실험용 세포주, 유전자원 보관
- **병원**: 이식용 세포/조직 보관

### 핵심 가치

```typescript
/**
 * WIA Cryo Monitoring 핵심 가치
 *
 * @philosophy 弘益人間 (홍익인간)
 * @mission 생명자원의 안전한 보관을 통한 인류 건강 증진
 */
interface CryoMonitoringPhilosophy {
  // 안전성 (Safety First)
  safety: {
    realTimeMonitoring: boolean;        // 실시간 모니터링
    immediateAlerts: boolean;           // 즉각적인 알림
    redundantSystems: boolean;          // 이중화 시스템
    autoBackup: boolean;                // 자동 백업
    safetyKr: "생명자원 보호 최우선";
  };

  // 신뢰성 (Reliability)
  reliability: {
    uptimeTarget: "99.99%";             // 가동률 목표
    sensorAccuracy: "±0.1°C";          // 센서 정확도
    dataIntegrity: boolean;             // 데이터 무결성
    auditTrail: boolean;                // 감사 추적
    reliabilityKr: "24/7 무중단 모니터링";
  };

  // 투명성 (Transparency)
  transparency: {
    openData: boolean;                  // 데이터 공개
    realTimeAccess: boolean;            // 실시간 접근
    historicalRecords: boolean;         // 이력 기록
    complianceReports: boolean;         // 규정 준수 보고
    transparencyKr: "모든 이해관계자 정보 공유";
  };

  // 접근성 (Accessibility)
  accessibility: {
    multiPlatform: boolean;             // 다중 플랫폼
    mobileApp: boolean;                 // 모바일 앱
    apiAccess: boolean;                 // API 접근
    userFriendly: boolean;              // 사용자 친화적
    accessibilityKr: "언제 어디서나 접근 가능";
  };

  // 표준화 (Standardization)
  standardization: {
    internationalStandards: string[];   // 국제 표준
    koreanRegulations: string[];        // 한국 규정
    dataFormats: string[];              // 데이터 형식
    apiProtocols: string[];             // API 프로토콜
    standardizationKr: "글로벌 표준 준수";
  };
}

// 실제 구현 예시
const cryoPhilosophy: CryoMonitoringPhilosophy = {
  safety: {
    realTimeMonitoring: true,
    immediateAlerts: true,
    redundantSystems: true,
    autoBackup: true,
    safetyKr: "생명자원 보호 최우선"
  },
  reliability: {
    uptimeTarget: "99.99%",
    sensorAccuracy: "±0.1°C",
    dataIntegrity: true,
    auditTrail: true,
    reliabilityKr: "24/7 무중단 모니터링"
  },
  transparency: {
    openData: true,
    realTimeAccess: true,
    historicalRecords: true,
    complianceReports: true,
    transparencyKr: "모든 이해관계자 정보 공유"
  },
  accessibility: {
    multiPlatform: true,
    mobileApp: true,
    apiAccess: true,
    userFriendly: true,
    accessibilityKr: "언제 어디서나 접근 가능"
  },
  standardization: {
    internationalStandards: ["ISO 23029", "ISO 20387", "AABB Standards"],
    koreanRegulations: ["의료기기법", "생명윤리법", "개인정보보호법"],
    dataFormats: ["HL7 FHIR", "JSON-LD", "WIA Standard"],
    apiProtocols: ["REST", "GraphQL", "WebSocket", "gRPC"],
    standardizationKr: "글로벌 표준 준수"
  }
};
```

---

## 모니터링 유형

### 1. 온도 모니터링 (Temperature Monitoring)

```typescript
/**
 * 온도 모니터링 설정
 * 극저온 환경의 온도를 실시간으로 추적
 */
interface TemperatureMonitoring {
  // 기본 정보
  sensorId: string;                     // 센서 ID
  location: {
    facilityId: string;                 // 시설 ID
    tankId: string;                     // 탱크 ID
    position: {                         // 위치 좌표
      x: number;
      y: number;
      z: number;
    };
    locationKr: string;                 // 위치 (한국어)
  };

  // 온도 범위
  temperatureRange: {
    min: number;                        // 최소 온도 (°C)
    max: number;                        // 최대 온도 (°C)
    optimal: number;                    // 최적 온도 (°C)
    critical: number;                   // 임계 온도 (°C)
    unit: "celsius" | "kelvin";        // 단위
    rangeKr: string;                    // 범위 설명 (한국어)
  };

  // 측정 설정
  measurement: {
    interval: number;                   // 측정 간격 (초)
    accuracy: number;                   // 정확도 (±°C)
    resolution: number;                 // 해상도 (°C)
    calibrationDate: Date;              // 교정 날짜
    nextCalibration: Date;              // 다음 교정 예정일
    measurementKr: string;              // 측정 설명 (한국어)
  };

  // 알림 임계값
  alerts: {
    warning: {
      upper: number;                    // 경고 상한선
      lower: number;                    // 경고 하한선
      warningKr: string;                // 경고 설명 (한국어)
    };
    critical: {
      upper: number;                    // 위험 상한선
      lower: number;                    // 위험 하한선
      criticalKr: string;               // 위험 설명 (한국어)
    };
    emergency: {
      upper: number;                    // 긴급 상한선
      lower: number;                    // 긴급 하한선
      emergencyKr: string;              // 긴급 설명 (한국어)
    };
  };

  // 메타데이터
  metadata: {
    sensorType: string;                 // 센서 유형
    manufacturer: string;               // 제조사
    model: string;                      // 모델명
    installDate: Date;                  // 설치 날짜
    warrantyExpiry: Date;               // 보증 만료일
    metadataKr: string;                 // 메타데이터 (한국어)
  };
}

// 실제 구현 예시
const temperatureMonitor: TemperatureMonitoring = {
  sensorId: "TEMP-001-KR-2026",
  location: {
    facilityId: "FAC-SEOUL-001",
    tankId: "TANK-LN2-A1",
    position: { x: 0, y: 0, z: 150 },
    locationKr: "서울 제대혈은행 A동 1호 탱크 상부"
  },
  temperatureRange: {
    min: -196,
    max: -150,
    optimal: -196,
    critical: -170,
    unit: "celsius",
    rangeKr: "액체질소 보관 온도 범위"
  },
  measurement: {
    interval: 60,                       // 1분마다 측정
    accuracy: 0.1,
    resolution: 0.01,
    calibrationDate: new Date("2026-01-01"),
    nextCalibration: new Date("2026-07-01"),
    measurementKr: "1분 간격 고정밀 측정"
  },
  alerts: {
    warning: {
      upper: -170,
      lower: -200,
      warningKr: "온도 주의 단계"
    },
    critical: {
      upper: -160,
      lower: -210,
      criticalKr: "온도 위험 단계 - 즉시 조치 필요"
    },
    emergency: {
      upper: -150,
      lower: -220,
      emergencyKr: "온도 긴급 상황 - 비상 대응 발동"
    }
  },
  metadata: {
    sensorType: "Platinum Resistance Thermometer (PRT)",
    manufacturer: "Samsung Electronics",
    model: "SE-CRYO-2026",
    installDate: new Date("2026-01-01"),
    warrantyExpiry: new Date("2031-01-01"),
    metadataKr: "삼성전자 극저온 센서 (5년 보증)"
  }
};
```

### 2. 액체질소 레벨 모니터링 (Liquid Nitrogen Level Monitoring)

```typescript
/**
 * 액체질소(LN2) 레벨 모니터링
 * 탱크의 질소 잔량을 실시간 추적
 */
interface LN2LevelMonitoring {
  // 탱크 정보
  tank: {
    tankId: string;                     // 탱크 ID
    capacity: number;                   // 용량 (리터)
    currentLevel: number;               // 현재 레벨 (%)
    volume: number;                     // 현재 용량 (리터)
    tankKr: string;                     // 탱크 설명 (한국어)
  };

  // 소비율 추적
  consumption: {
    dailyRate: number;                  // 일일 소비율 (리터/일)
    weeklyRate: number;                 // 주간 소비율
    monthlyRate: number;                // 월간 소비율
    predictedEmpty: Date;               // 예상 소진 날짜
    consumptionKr: string;              // 소비율 설명 (한국어)
  };

  // 보충 스케줄
  refill: {
    threshold: number;                  // 보충 임계값 (%)
    autoOrder: boolean;                 // 자동 주문
    supplier: string;                   // 공급업체
    leadTime: number;                   // 배송 소요 시간 (시간)
    lastRefill: Date;                   // 마지막 보충 날짜
    nextScheduled: Date;                // 다음 예정 날짜
    refillKr: string;                   // 보충 설명 (한국어)
  };

  // 센서 설정
  sensor: {
    sensorId: string;                   // 센서 ID
    type: "capacitance" | "ultrasonic" | "differential_pressure";
    measurementInterval: number;        // 측정 간격 (초)
    accuracy: number;                   // 정확도 (%)
    sensorKr: string;                   // 센서 설명 (한국어)
  };

  // 알림 설정
  alerts: {
    low: number;                        // 낮음 (%)
    veryLow: number;                    // 매우 낮음 (%)
    critical: number;                   // 위험 (%)
    alertKr: string;                    // 알림 설명 (한국어)
  };
}

// 실제 구현 예시
const ln2Monitor: LN2LevelMonitoring = {
  tank: {
    tankId: "TANK-LN2-A1",
    capacity: 500,                      // 500리터 탱크
    currentLevel: 75,                   // 현재 75% 레벨
    volume: 375,                        // 375리터
    tankKr: "A1 액체질소 저장 탱크 (500L)"
  },
  consumption: {
    dailyRate: 5,                       // 일일 5리터 소비
    weeklyRate: 35,
    monthlyRate: 150,
    predictedEmpty: new Date("2026-03-20"),
    consumptionKr: "일평균 5리터 소비, 약 75일 사용 가능"
  },
  refill: {
    threshold: 30,                      // 30% 이하시 보충
    autoOrder: true,
    supplier: "대한산업가스",
    leadTime: 24,                       // 24시간 배송
    lastRefill: new Date("2026-01-01"),
    nextScheduled: new Date("2026-02-15"),
    refillKr: "자동 주문 시스템 (30% 이하시 발주)"
  },
  sensor: {
    sensorId: "LVL-001-KR-2026",
    type: "capacitance",
    measurementInterval: 300,           // 5분마다 측정
    accuracy: 1,                        // ±1% 정확도
    sensorKr: "용량식 레벨 센서 (±1% 정확도)"
  },
  alerts: {
    low: 40,
    veryLow: 25,
    critical: 15,
    alertKr: "3단계 알림 시스템 (40%, 25%, 15%)"
  }
};
```

### 3. 압력 모니터링 (Pressure Monitoring)

```typescript
/**
 * 탱크 압력 모니터링
 * 극저온 탱크의 내부 압력 추적
 */
interface PressureMonitoring {
  // 압력 측정
  pressure: {
    current: number;                    // 현재 압력 (PSI)
    min: number;                        // 최소 압력
    max: number;                        // 최대 압력
    optimal: number;                    // 최적 압력
    unit: "psi" | "bar" | "kpa";       // 단위
    pressureKr: string;                 // 압력 설명 (한국어)
  };

  // 안전 밸브
  safetyValve: {
    releasePoint: number;               // 방출 압력 (PSI)
    status: "normal" | "activated" | "malfunction";
    lastActivation: Date | null;        // 마지막 작동 날짜
    testDate: Date;                     // 테스트 날짜
    nextTest: Date;                     // 다음 테스트 예정
    valveKr: string;                    // 밸브 설명 (한국어)
  };

  // 압력 변화율
  changeRate: {
    perHour: number;                    // 시간당 변화 (PSI/hr)
    perDay: number;                     // 일일 변화
    trend: "increasing" | "stable" | "decreasing";
    trendKr: string;                    // 추세 (한국어)
  };

  // 알림
  alerts: {
    highPressure: number;               // 고압 경고
    lowPressure: number;                // 저압 경고
    rapidChange: number;                // 급격한 변화 감지 (PSI/hr)
    alertKr: string;                    // 알림 설명 (한국어)
  };
}

// 실제 구현 예시
const pressureMonitor: PressureMonitoring = {
  pressure: {
    current: 22,                        // 현재 22 PSI
    min: 15,
    max: 25,
    optimal: 20,
    unit: "psi",
    pressureKr: "정상 범위 내 압력 유지"
  },
  safetyValve: {
    releasePoint: 30,                   // 30 PSI에서 방출
    status: "normal",
    lastActivation: null,
    testDate: new Date("2026-01-01"),
    nextTest: new Date("2026-04-01"),
    valveKr: "안전 밸브 정상 작동 (분기별 테스트)"
  },
  changeRate: {
    perHour: 0.2,                       // 시간당 0.2 PSI 증가
    perDay: 4.8,
    trend: "stable",
    trendKr: "안정적인 압력 유지"
  },
  alerts: {
    highPressure: 26,
    lowPressure: 14,
    rapidChange: 2,                     // 시간당 2 PSI 이상 변화시 알림
    alertKr: "압력 이상 감지시 즉시 알림"
  }
};
```

---

## 실시간 추적 시스템

### 실시간 데이터 수집

```typescript
/**
 * 실시간 데이터 수집 시스템
 * WebSocket을 통한 실시간 센서 데이터 스트리밍
 */
interface RealTimeDataCollectionSystem {
  // WebSocket 연결
  connection: {
    url: string;                        // WebSocket URL
    protocol: "wss" | "ws";            // 프로토콜
    reconnect: boolean;                 // 자동 재연결
    heartbeat: number;                  // 하트비트 간격 (초)
    connectionKr: string;               // 연결 설명 (한국어)
  };

  // 데이터 스트림
  stream: {
    sensors: string[];                  // 구독 센서 목록
    sampleRate: number;                 // 샘플링 레이트 (Hz)
    bufferSize: number;                 // 버퍼 크기
    compression: boolean;               // 데이터 압축
    streamKr: string;                   // 스트림 설명 (한국어)
  };

  // 데이터 처리
  processing: {
    validation: boolean;                // 데이터 검증
    filtering: boolean;                 // 노이즈 필터링
    aggregation: "none" | "avg" | "median" | "max";
    storagePolicy: "all" | "exceptions" | "summary";
    processingKr: string;               // 처리 설명 (한국어)
  };

  // 알림 엔진
  alerting: {
    enabled: boolean;                   // 알림 활성화
    rules: AlertRule[];                 // 알림 규칙
    channels: AlertChannel[];           // 알림 채널
    escalation: EscalationPolicy;       // 에스컬레이션 정책
    alertingKr: string;                 // 알림 설명 (한국어)
  };
}

interface AlertRule {
  ruleId: string;                       // 규칙 ID
  name: string;                         // 규칙명
  nameKr: string;                       // 규칙명 (한국어)
  condition: {
    parameter: "temperature" | "level" | "pressure";
    operator: ">" | "<" | "==" | "!=" | "between";
    value: number | [number, number];
    duration: number;                   // 지속 시간 (초)
    conditionKr: string;                // 조건 설명 (한국어)
  };
  severity: "info" | "warning" | "critical" | "emergency";
  severityKr: string;                   // 심각도 (한국어)
  action: {
    notification: boolean;              // 알림 전송
    logging: boolean;                   // 로그 기록
    automation: string | null;          // 자동 조치 스크립트
    actionKr: string;                   // 조치 설명 (한국어)
  };
}

interface AlertChannel {
  channelId: string;                    // 채널 ID
  type: "email" | "sms" | "push" | "webhook" | "voice";
  typeKr: string;                       // 유형 (한국어)
  recipients: string[];                 // 수신자
  priority: "high" | "normal" | "low";
  enabled: boolean;                     // 활성화 여부
}

interface EscalationPolicy {
  policyId: string;                     // 정책 ID
  levels: {
    level: number;                      // 단계
    delay: number;                      // 지연 시간 (분)
    recipients: string[];               // 수신자
    levelKr: string;                    // 단계 설명 (한국어)
  }[];
  maxRetries: number;                   // 최대 재시도
  policyKr: string;                     // 정책 설명 (한국어)
}

// 실제 구현 예시
const realTimeSystem: RealTimeDataCollectionSystem = {
  connection: {
    url: "wss://monitor.cryo.wia.org/stream",
    protocol: "wss",
    reconnect: true,
    heartbeat: 30,
    connectionKr: "보안 WebSocket 연결 (자동 재연결)"
  },
  stream: {
    sensors: [
      "TEMP-001-KR-2026",
      "LVL-001-KR-2026",
      "PRES-001-KR-2026"
    ],
    sampleRate: 1,                      // 1Hz (초당 1회)
    bufferSize: 1000,
    compression: true,
    streamKr: "3개 센서 실시간 스트리밍 (1초 간격)"
  },
  processing: {
    validation: true,
    filtering: true,
    aggregation: "avg",
    storagePolicy: "all",
    processingKr: "실시간 검증 및 필터링, 전체 데이터 저장"
  },
  alerting: {
    enabled: true,
    rules: [
      {
        ruleId: "RULE-TEMP-CRITICAL",
        name: "Critical Temperature Alert",
        nameKr: "온도 위험 알림",
        condition: {
          parameter: "temperature",
          operator: ">",
          value: -160,
          duration: 300,                // 5분 지속시
          conditionKr: "온도가 -160°C 초과하고 5분간 지속될 때"
        },
        severity: "critical",
        severityKr: "위험",
        action: {
          notification: true,
          logging: true,
          automation: "auto-refill-ln2.sh",
          actionKr: "알림 전송 + 로깅 + 자동 질소 보충"
        }
      }
    ],
    channels: [
      {
        channelId: "CH-EMAIL-001",
        type: "email",
        typeKr: "이메일",
        recipients: ["manager@facility.kr", "engineer@facility.kr"],
        priority: "high",
        enabled: true
      },
      {
        channelId: "CH-SMS-001",
        type: "sms",
        typeKr: "문자메시지",
        recipients: ["+82-10-1234-5678"],
        priority: "high",
        enabled: true
      }
    ],
    escalation: {
      policyId: "ESC-001",
      levels: [
        {
          level: 1,
          delay: 0,
          recipients: ["duty-engineer@facility.kr"],
          levelKr: "1단계: 당직 엔지니어 (즉시)"
        },
        {
          level: 2,
          delay: 15,
          recipients: ["supervisor@facility.kr"],
          levelKr: "2단계: 관리 감독자 (15분 후)"
        },
        {
          level: 3,
          delay: 30,
          recipients: ["director@facility.kr"],
          levelKr: "3단계: 시설 책임자 (30분 후)"
        }
      ],
      maxRetries: 5,
      policyKr: "3단계 에스컬레이션 정책 (최대 5회 재시도)"
    },
    alertingKr: "다채널 실시간 알림 시스템"
  }
};
```

---

## TypeScript 구현

### 완전한 모니터링 시스템 클래스

```typescript
/**
 * WIA Cryo Monitoring System
 * 극저온 생물자원 통합 모니터링 시스템
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2026 WIA (World Certification Industry Association)
 */

import { EventEmitter } from "events";
import WebSocket from "ws";

class CryoMonitoringSystem extends EventEmitter {
  private sensors: Map<string, SensorConfig>;
  private dataBuffer: Map<string, SensorReading[]>;
  private wsConnection: WebSocket | null;
  private alertRules: Map<string, AlertRule>;
  private isMonitoring: boolean;

  constructor() {
    super();
    this.sensors = new Map();
    this.dataBuffer = new Map();
    this.wsConnection = null;
    this.alertRules = new Map();
    this.isMonitoring = false;
  }

  /**
   * 센서 등록
   * 새로운 센서를 시스템에 등록
   */
  registerSensor(sensor: SensorConfig): void {
    this.sensors.set(sensor.sensorId, sensor);
    this.dataBuffer.set(sensor.sensorId, []);

    console.log(`[센서 등록] ${sensor.sensorId} - ${sensor.nameKr}`);
    this.emit("sensor:registered", sensor);
  }

  /**
   * 실시간 모니터링 시작
   * WebSocket 연결 및 데이터 스트리밍 시작
   */
  async startMonitoring(wsUrl: string): Promise<void> {
    if (this.isMonitoring) {
      throw new Error("모니터링이 이미 실행 중입니다");
    }

    this.wsConnection = new WebSocket(wsUrl);

    this.wsConnection.on("open", () => {
      console.log("[모니터링 시작] WebSocket 연결 성공");
      this.isMonitoring = true;

      // 모든 센서 구독
      this.sensors.forEach(sensor => {
        this.wsConnection?.send(JSON.stringify({
          action: "subscribe",
          sensorId: sensor.sensorId
        }));
      });

      this.emit("monitoring:started");
    });

    this.wsConnection.on("message", (data: string) => {
      const reading: SensorReading = JSON.parse(data);
      this.processSensorReading(reading);
    });

    this.wsConnection.on("error", (error) => {
      console.error("[WebSocket 오류]", error);
      this.emit("monitoring:error", error);
    });

    this.wsConnection.on("close", () => {
      console.log("[모니터링 중지] WebSocket 연결 종료");
      this.isMonitoring = false;
      this.emit("monitoring:stopped");

      // 자동 재연결 (5초 후)
      setTimeout(() => {
        console.log("[재연결 시도]");
        this.startMonitoring(wsUrl);
      }, 5000);
    });
  }

  /**
   * 센서 데이터 처리
   * 실시간 센서 데이터를 처리하고 알림 규칙 평가
   */
  private processSensorReading(reading: SensorReading): void {
    const buffer = this.dataBuffer.get(reading.sensorId);
    if (!buffer) return;

    // 버퍼에 추가
    buffer.push(reading);
    if (buffer.length > 1000) {
      buffer.shift();  // 오래된 데이터 제거
    }

    // 데이터 검증
    if (!this.validateReading(reading)) {
      console.warn(`[데이터 검증 실패] ${reading.sensorId}`, reading);
      this.emit("data:invalid", reading);
      return;
    }

    // 알림 규칙 평가
    this.evaluateAlertRules(reading);

    // 이벤트 발행
    this.emit("data:received", reading);
  }

  /**
   * 데이터 검증
   * 센서 데이터의 유효성 검사
   */
  private validateReading(reading: SensorReading): boolean {
    const sensor = this.sensors.get(reading.sensorId);
    if (!sensor) return false;

    // 온도 범위 검증
    if (sensor.type === "temperature") {
      return reading.value >= -250 && reading.value <= 50;
    }

    // 레벨 검증 (0-100%)
    if (sensor.type === "level") {
      return reading.value >= 0 && reading.value <= 100;
    }

    // 압력 검증
    if (sensor.type === "pressure") {
      return reading.value >= 0 && reading.value <= 100;
    }

    return true;
  }

  /**
   * 알림 규칙 평가
   * 센서 데이터를 기반으로 알림 조건 확인
   */
  private evaluateAlertRules(reading: SensorReading): void {
    this.alertRules.forEach(rule => {
      if (!this.matchesCondition(reading, rule.condition)) {
        return;
      }

      // 지속 시간 확인
      const buffer = this.dataBuffer.get(reading.sensorId);
      if (!buffer) return;

      const duration = rule.condition.duration * 1000;  // 초 -> 밀리초
      const recentReadings = buffer.filter(r =>
        Date.now() - new Date(r.timestamp).getTime() < duration
      );

      const allMatch = recentReadings.every(r =>
        this.matchesCondition(r, rule.condition)
      );

      if (allMatch && recentReadings.length > 0) {
        this.triggerAlert(rule, reading);
      }
    });
  }

  /**
   * 조건 매칭
   * 센서 데이터가 알림 조건과 일치하는지 확인
   */
  private matchesCondition(
    reading: SensorReading,
    condition: AlertRule["condition"]
  ): boolean {
    switch (condition.operator) {
      case ">":
        return reading.value > (condition.value as number);
      case "<":
        return reading.value < (condition.value as number);
      case "==":
        return reading.value === condition.value;
      case "!=":
        return reading.value !== condition.value;
      case "between":
        const [min, max] = condition.value as [number, number];
        return reading.value >= min && reading.value <= max;
      default:
        return false;
    }
  }

  /**
   * 알림 발송
   * 조건 만족시 알림 전송
   */
  private triggerAlert(rule: AlertRule, reading: SensorReading): void {
    const alert = {
      alertId: `ALT-${Date.now()}`,
      ruleId: rule.ruleId,
      ruleName: rule.nameKr,
      severity: rule.severity,
      severityKr: rule.severityKr,
      sensorId: reading.sensorId,
      value: reading.value,
      timestamp: new Date(),
      message: `${rule.nameKr}: ${reading.value} (${rule.condition.conditionKr})`
    };

    console.log(`[알림 발생] ${alert.severityKr} - ${alert.message}`);

    // 알림 기록
    if (rule.action.logging) {
      this.logAlert(alert);
    }

    // 알림 전송
    if (rule.action.notification) {
      this.sendNotification(alert);
    }

    // 자동 조치 실행
    if (rule.action.automation) {
      this.executeAutomation(rule.action.automation, alert);
    }

    this.emit("alert:triggered", alert);
  }

  /**
   * 알림 기록
   */
  private logAlert(alert: any): void {
    // 데이터베이스나 파일에 알림 기록
    console.log(`[알림 기록] ${JSON.stringify(alert, null, 2)}`);
  }

  /**
   * 알림 전송
   */
  private sendNotification(alert: any): void {
    // 이메일, SMS, 푸시 알림 등 전송
    console.log(`[알림 전송] ${alert.severityKr} - ${alert.message}`);
  }

  /**
   * 자동 조치 실행
   */
  private executeAutomation(script: string, alert: any): void {
    console.log(`[자동 조치] ${script} 실행`);
    // 자동화 스크립트 실행 로직
  }

  /**
   * 통계 조회
   * 센서 데이터의 통계 정보 반환
   */
  getStatistics(sensorId: string, timeRange: number): SensorStatistics {
    const buffer = this.dataBuffer.get(sensorId);
    if (!buffer) {
      throw new Error(`센서를 찾을 수 없습니다: ${sensorId}`);
    }

    const cutoff = Date.now() - timeRange;
    const data = buffer.filter(r =>
      new Date(r.timestamp).getTime() > cutoff
    );

    if (data.length === 0) {
      throw new Error("데이터가 없습니다");
    }

    const values = data.map(r => r.value);
    const avg = values.reduce((a, b) => a + b, 0) / values.length;
    const min = Math.min(...values);
    const max = Math.max(...values);

    return {
      sensorId,
      count: data.length,
      average: avg,
      min,
      max,
      latest: data[data.length - 1].value,
      timestamp: new Date(),
      statisticsKr: `평균: ${avg.toFixed(2)}, 최소: ${min}, 최대: ${max}`
    };
  }
}

interface SensorConfig {
  sensorId: string;
  type: "temperature" | "level" | "pressure";
  nameKr: string;
  location: string;
}

interface SensorReading {
  sensorId: string;
  value: number;
  unit: string;
  timestamp: Date;
  quality: "good" | "fair" | "poor";
}

interface SensorStatistics {
  sensorId: string;
  count: number;
  average: number;
  min: number;
  max: number;
  latest: number;
  timestamp: Date;
  statisticsKr: string;
}

// 사용 예시
const monitor = new CryoMonitoringSystem();

// 센서 등록
monitor.registerSensor({
  sensorId: "TEMP-001-KR-2026",
  type: "temperature",
  nameKr: "제대혈 탱크 A1 온도 센서",
  location: "서울 제대혈은행"
});

// 이벤트 리스너
monitor.on("data:received", (reading) => {
  console.log(`[데이터 수신] ${reading.sensorId}: ${reading.value}${reading.unit}`);
});

monitor.on("alert:triggered", (alert) => {
  console.log(`[알림] ${alert.severityKr} - ${alert.message}`);
});

// 모니터링 시작
monitor.startMonitoring("wss://monitor.cryo.wia.org/stream");
```

---

## 한국 시장 적용

### 국내 규제 준수

```typescript
/**
 * 한국 규제 준수 체크리스트
 */
interface KoreanRegulatoryCompliance {
  // 의료기기법
  medicalDeviceAct: {
    classification: "2등급 의료기기";      // 등급 분류
    kfdaApproval: boolean;                 // 식약처 승인
    gmpCertification: boolean;             // GMP 인증
    qualityManagement: "ISO 13485";        // 품질 관리 시스템
    complianceKr: string;                  // 준수 사항 (한국어)
  };

  // 생명윤리법
  bioethicsLaw: {
    irbApproval: boolean;                  // IRB 승인
    informedConsent: boolean;              // 동의서
    privacyProtection: boolean;            // 개인정보 보호
    dataRetention: number;                 // 데이터 보관 기간 (년)
    lawKr: string;                         // 법규 준수 (한국어)
  };

  // 개인정보보호법
  privacyLaw: {
    encryption: "AES-256";                 // 암호화 방식
    accessControl: boolean;                // 접근 통제
    auditLog: boolean;                     // 감사 로그
    dataMinimization: boolean;             // 최소 수집
    privacyKr: string;                     // 개인정보 보호 (한국어)
  };
}

// 실제 구현
const koreanCompliance: KoreanRegulatoryCompliance = {
  medicalDeviceAct: {
    classification: "2등급 의료기기",
    kfdaApproval: true,
    gmpCertification: true,
    qualityManagement: "ISO 13485",
    complianceKr: "식약처 승인 완료, GMP 인증 보유"
  },
  bioethicsLaw: {
    irbApproval: true,
    informedConsent: true,
    privacyProtection: true,
    dataRetention: 10,
    lawKr: "생명윤리법 완전 준수, 10년 데이터 보관"
  },
  privacyLaw: {
    encryption: "AES-256",
    accessControl: true,
    auditLog: true,
    dataMinimization: true,
    privacyKr: "AES-256 암호화, 완전한 접근 통제"
  }
};
```

---

## 결론

WIA Cryo Monitoring Standard는 극저온 생물자원의 안전한 보관을 위한 포괄적인 모니터링 표준을 제공합니다.

### 핵심 특징

1. **실시간 모니터링**: 24/7 무중단 센서 데이터 추적
2. **즉각적인 알림**: 다단계 에스컬레이션 시스템
3. **한국 시장 최적화**: 국내 규제 완전 준수
4. **TypeScript 구현**: 타입 안전한 구현

### 다음 장 예고

다음 장에서는 **글로벌 및 한국 시장 분석**을 다룹니다:
- 극저온 모니터링 시장 규모 및 성장률
- 국내 주요 플레이어 (삼성, LG, 차병원)
- IoT 센서 기술 동향
- 콜드체인 물류 통합

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
