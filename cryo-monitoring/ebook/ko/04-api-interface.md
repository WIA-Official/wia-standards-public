# 04. API 인터페이스
## API Interface - REST, GraphQL, WebSocket

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [API 개요](#api-개요)
2. [REST API](#rest-api)
3. [GraphQL API](#graphql-api)
4. [WebSocket 실시간 스트리밍](#websocket-실시간-스트리밍)
5. [인증 및 보안](#인증-및-보안)
6. [속도 제한 및 할당량](#속도-제한-및-할당량)
7. [오류 처리](#오류-처리)
8. [SDK 및 클라이언트 라이브러리](#sdk-및-클라이언트-라이브러리)

---

## API 개요

### API 엔드포인트

```typescript
/**
 * WIA Cryo Monitoring API 엔드포인트
 */
interface APIEndpoints {
  // 기본 URL
  baseUrl: string;
  baseUrlKr: string;

  // REST API
  rest: {
    production: string;
    staging: string;
    development: string;
    restKr: string;
  };

  // GraphQL
  graphql: {
    endpoint: string;
    playground: string;
    graphqlKr: string;
  };

  // WebSocket
  websocket: {
    stream: string;
    streamKr: string;
  };

  // 인증
  auth: {
    login: string;
    logout: string;
    refresh: string;
    authKr: string;
  };
}

const endpoints: APIEndpoints = {
  baseUrl: "https://api.cryo-monitor.wia.org",
  baseUrlKr: "WIA 극저온 모니터링 API",

  rest: {
    production: "https://api.cryo-monitor.wia.org/v1",
    staging: "https://staging-api.cryo-monitor.wia.org/v1",
    development: "https://dev-api.cryo-monitor.wia.org/v1",
    restKr: "REST API 버전 1"
  },

  graphql: {
    endpoint: "https://api.cryo-monitor.wia.org/graphql",
    playground: "https://api.cryo-monitor.wia.org/playground",
    graphqlKr: "GraphQL API 및 Playground"
  },

  websocket: {
    stream: "wss://stream.cryo-monitor.wia.org/v1",
    streamKr: "실시간 센서 데이터 스트리밍"
  },

  auth: {
    login: "/auth/login",
    logout: "/auth/logout",
    refresh: "/auth/refresh",
    authKr: "인증 엔드포인트"
  }
};
```

### API 버전 관리

```typescript
/**
 * API 버전 정책
 */
interface APIVersionPolicy {
  current: string;
  supported: string[];
  deprecated: string[];
  sunset: {
    version: string;
    date: Date;
    sunsetKr: string;
  }[];
  policyKr: string;
}

const versionPolicy: APIVersionPolicy = {
  current: "v1",
  supported: ["v1"],
  deprecated: [],
  sunset: [],
  policyKr: "현재 v1만 지원, 하위 호환성 보장"
};
```

---

## REST API

### 센서 관리 엔드포인트

```typescript
/**
 * 센서 REST API
 */
class SensorAPI {
  private baseUrl: string;
  private apiKey: string;

  constructor(baseUrl: string, apiKey: string) {
    this.baseUrl = baseUrl;
    this.apiKey = apiKey;
  }

  /**
   * 센서 목록 조회
   * GET /sensors
   */
  async listSensors(params?: {
    facilityId?: string;
    type?: SensorType;
    status?: SensorStatus;
    page?: number;
    limit?: number;
  }): Promise<{
    sensors: SensorConfig[];
    total: number;
    page: number;
    limit: number;
    totalPages: number;
  }> {
    const queryParams = new URLSearchParams();
    if (params?.facilityId) queryParams.append("facilityId", params.facilityId);
    if (params?.type) queryParams.append("type", params.type);
    if (params?.status) queryParams.append("status", params.status);
    if (params?.page) queryParams.append("page", params.page.toString());
    if (params?.limit) queryParams.append("limit", params.limit.toString());

    const response = await fetch(
      `${this.baseUrl}/sensors?${queryParams.toString()}`,
      {
        headers: {
          "Authorization": `Bearer ${this.apiKey}`,
          "Accept": "application/json"
        }
      }
    );

    if (!response.ok) {
      throw new Error(`센서 목록 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 상세 조회
   * GET /sensors/:sensorId
   */
  async getSensor(sensorId: string): Promise<SensorConfig> {
    const response = await fetch(`${this.baseUrl}/sensors/${sensorId}`, {
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Accept": "application/json"
      }
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error(`센서를 찾을 수 없습니다: ${sensorId}`);
      }
      throw new Error(`센서 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 생성
   * POST /sensors
   */
  async createSensor(sensor: Omit<SensorConfig, "sensorId" | "createdAt" | "updatedAt">): Promise<SensorConfig> {
    const response = await fetch(`${this.baseUrl}/sensors`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(sensor)
    });

    if (!response.ok) {
      throw new Error(`센서 생성 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 수정
   * PUT /sensors/:sensorId
   */
  async updateSensor(sensorId: string, updates: Partial<SensorConfig>): Promise<SensorConfig> {
    const response = await fetch(`${this.baseUrl}/sensors/${sensorId}`, {
      method: "PUT",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(updates)
    });

    if (!response.ok) {
      throw new Error(`센서 수정 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 삭제
   * DELETE /sensors/:sensorId
   */
  async deleteSensor(sensorId: string): Promise<{ success: boolean; message: string; messageKr: string }> {
    const response = await fetch(`${this.baseUrl}/sensors/${sensorId}`, {
      method: "DELETE",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Accept": "application/json"
      }
    });

    if (!response.ok) {
      throw new Error(`센서 삭제 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 교정
   * POST /sensors/:sensorId/calibrate
   */
  async calibrateSensor(sensorId: string, calibrationData: {
    calibratedBy: string;
    certificate?: string;
    notes?: string;
  }): Promise<SensorConfig> {
    const response = await fetch(`${this.baseUrl}/sensors/${sensorId}/calibrate`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(calibrationData)
    });

    if (!response.ok) {
      throw new Error(`센서 교정 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 데이터 조회
   * GET /sensors/:sensorId/readings
   */
  async getSensorReadings(sensorId: string, params?: {
    start?: Date;
    end?: Date;
    limit?: number;
    quality?: string;
  }): Promise<{
    readings: SensorReading[];
    total: number;
    statistics: SensorStatistics;
  }> {
    const queryParams = new URLSearchParams();
    if (params?.start) queryParams.append("start", params.start.toISOString());
    if (params?.end) queryParams.append("end", params.end.toISOString());
    if (params?.limit) queryParams.append("limit", params.limit.toString());
    if (params?.quality) queryParams.append("quality", params.quality);

    const response = await fetch(
      `${this.baseUrl}/sensors/${sensorId}/readings?${queryParams.toString()}`,
      {
        headers: {
          "Authorization": `Bearer ${this.apiKey}`,
          "Accept": "application/json"
        }
      }
    );

    if (!response.ok) {
      throw new Error(`센서 데이터 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 센서 통계 조회
   * GET /sensors/:sensorId/statistics
   */
  async getSensorStatistics(sensorId: string, timeRange: {
    start: Date;
    end: Date;
  }): Promise<SensorStatistics> {
    const queryParams = new URLSearchParams({
      start: timeRange.start.toISOString(),
      end: timeRange.end.toISOString()
    });

    const response = await fetch(
      `${this.baseUrl}/sensors/${sensorId}/statistics?${queryParams.toString()}`,
      {
        headers: {
          "Authorization": `Bearer ${this.apiKey}`,
          "Accept": "application/json"
        }
      }
    );

    if (!response.ok) {
      throw new Error(`센서 통계 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }
}

// 사용 예시
const sensorAPI = new SensorAPI(
  "https://api.cryo-monitor.wia.org/v1",
  "your-api-key-here"
);

// 센서 목록 조회
const sensors = await sensorAPI.listSensors({
  facilityId: "123e4567-e89b-12d3-a456-426614174000",
  type: "temperature",
  status: "active",
  page: 1,
  limit: 20
});

console.log(`총 ${sensors.total}개 센서 중 ${sensors.sensors.length}개 조회`);

// 센서 생성
const newSensor = await sensorAPI.createSensor({
  name: "Tank A1 Temperature Sensor",
  nameKr: "탱크 A1 온도 센서",
  type: "temperature",
  typeKr: "온도 센서",
  // ... 나머지 필드
});

console.log(`센서 생성 완료: ${newSensor.nameKr}`);
```

### 탱크 관리 엔드포인트

```typescript
/**
 * 탱크 REST API
 */
class TankAPI {
  private baseUrl: string;
  private apiKey: string;

  constructor(baseUrl: string, apiKey: string) {
    this.baseUrl = baseUrl;
    this.apiKey = apiKey;
  }

  /**
   * 탱크 목록 조회
   * GET /tanks
   */
  async listTanks(params?: {
    facilityId?: string;
    type?: TankType;
    status?: TankStatus;
    page?: number;
    limit?: number;
  }): Promise<{
    tanks: Tank[];
    total: number;
    page: number;
    limit: number;
  }> {
    const queryParams = new URLSearchParams();
    if (params?.facilityId) queryParams.append("facilityId", params.facilityId);
    if (params?.type) queryParams.append("type", params.type);
    if (params?.status) queryParams.append("status", params.status);
    if (params?.page) queryParams.append("page", params.page.toString());
    if (params?.limit) queryParams.append("limit", params.limit.toString());

    const response = await fetch(
      `${this.baseUrl}/tanks?${queryParams.toString()}`,
      {
        headers: {
          "Authorization": `Bearer ${this.apiKey}`,
          "Accept": "application/json"
        }
      }
    );

    if (!response.ok) {
      throw new Error(`탱크 목록 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 탱크 상세 조회
   * GET /tanks/:tankId
   */
  async getTank(tankId: string): Promise<Tank> {
    const response = await fetch(`${this.baseUrl}/tanks/${tankId}`, {
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Accept": "application/json"
      }
    });

    if (!response.ok) {
      throw new Error(`탱크 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 탱크 보충 기록
   * POST /tanks/:tankId/refill
   */
  async recordRefill(tankId: string, refillData: {
    volume: number;              // 보충량 (리터)
    performedBy: string;         // 담당자
    supplier: string;            // 공급업체
    batchNumber?: string;        // 배치 번호
    notes?: string;              // 비고
  }): Promise<{
    success: boolean;
    tank: Tank;
    message: string;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/tanks/${tankId}/refill`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(refillData)
    });

    if (!response.ok) {
      throw new Error(`탱크 보충 기록 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 탱크 점검 기록
   * POST /tanks/:tankId/maintenance
   */
  async recordMaintenance(tankId: string, maintenanceData: {
    type: string;
    typeKr: string;
    performedBy: string;
    notes?: string;
    documentUrl?: string;
  }): Promise<{
    success: boolean;
    tank: Tank;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/tanks/${tankId}/maintenance`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(maintenanceData)
    });

    if (!response.ok) {
      throw new Error(`탱크 점검 기록 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 탱크 현황 대시보드
   * GET /tanks/:tankId/dashboard
   */
  async getTankDashboard(tankId: string): Promise<{
    tank: Tank;
    currentReadings: {
      temperature: SensorReading[];
      pressure: SensorReading[];
      level: SensorReading[];
    };
    statistics: {
      last24Hours: SensorStatistics[];
      last7Days: SensorStatistics[];
      last30Days: SensorStatistics[];
    };
    alerts: Alert[];
    predictions: {
      ln2EmptyDate: Date;
      nextRefillDate: Date;
      predictionsKr: string;
    };
    dashboardKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/tanks/${tankId}/dashboard`, {
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Accept": "application/json"
      }
    });

    if (!response.ok) {
      throw new Error(`탱크 대시보드 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }
}
```

### 알림 관리 엔드포인트

```typescript
/**
 * 알림 REST API
 */
class AlertAPI {
  private baseUrl: string;
  private apiKey: string;

  constructor(baseUrl: string, apiKey: string) {
    this.baseUrl = baseUrl;
    this.apiKey = apiKey;
  }

  /**
   * 활성 알림 조회
   * GET /alerts
   */
  async listActiveAlerts(params?: {
    facilityId?: string;
    severity?: AlertSeverity;
    status?: AlertStatus;
    page?: number;
    limit?: number;
  }): Promise<{
    alerts: Alert[];
    total: number;
    summary: {
      emergency: number;
      critical: number;
      warning: number;
      info: number;
      summaryKr: string;
    };
  }> {
    const queryParams = new URLSearchParams();
    if (params?.facilityId) queryParams.append("facilityId", params.facilityId);
    if (params?.severity) queryParams.append("severity", params.severity);
    if (params?.status) queryParams.append("status", params.status);
    if (params?.page) queryParams.append("page", params.page.toString());
    if (params?.limit) queryParams.append("limit", params.limit.toString());

    const response = await fetch(
      `${this.baseUrl}/alerts?${queryParams.toString()}`,
      {
        headers: {
          "Authorization": `Bearer ${this.apiKey}`,
          "Accept": "application/json"
        }
      }
    );

    if (!response.ok) {
      throw new Error(`알림 조회 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 알림 확인
   * POST /alerts/:alertId/acknowledge
   */
  async acknowledgeAlert(alertId: string, data: {
    acknowledgedBy: string;
    notes?: string;
  }): Promise<{
    success: boolean;
    alert: Alert;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/alerts/${alertId}/acknowledge`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(data)
    });

    if (!response.ok) {
      throw new Error(`알림 확인 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 알림 해결
   * POST /alerts/:alertId/resolve
   */
  async resolveAlert(alertId: string, data: {
    resolvedBy: string;
    resolutionNotes: string;
  }): Promise<{
    success: boolean;
    alert: Alert;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/alerts/${alertId}/resolve`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(data)
    });

    if (!response.ok) {
      throw new Error(`알림 해결 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 알림 에스컬레이션
   * POST /alerts/:alertId/escalate
   */
  async escalateAlert(alertId: string, data: {
    escalatedBy: string;
    escalateTo: string[];
    reason: string;
    reasonKr: string;
  }): Promise<{
    success: boolean;
    alert: Alert;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/alerts/${alertId}/escalate`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.apiKey}`,
        "Content-Type": "application/json",
        "Accept": "application/json"
      },
      body: JSON.stringify(data)
    });

    if (!response.ok) {
      throw new Error(`알림 에스컬레이션 실패: ${response.statusText}`);
    }

    return response.json();
  }
}
```

---

## GraphQL API

### GraphQL 스키마

```graphql
# WIA Cryo Monitoring GraphQL Schema
# 한국어 주석 포함

"""
센서 유형
"""
enum SensorType {
  TEMPERATURE     # 온도 센서
  LEVEL          # 레벨 센서
  PRESSURE       # 압력 센서
  HUMIDITY       # 습도 센서
  CO2            # CO2 센서
  O2             # 산소 센서
  VIBRATION      # 진동 센서
  DOOR           # 도어 센서
  POWER          # 전원 센서
}

"""
센서 상태
"""
enum SensorStatus {
  ACTIVE         # 정상 작동
  CALIBRATING    # 교정 중
  MAINTENANCE    # 점검 중
  ERROR          # 오류
  OFFLINE        # 오프라인
  DECOMMISSIONED # 폐기
}

"""
센서 설정
"""
type SensorConfig {
  sensorId: ID!
  name: String!
  nameKr: String!            # 한국어 이름
  type: SensorType!
  typeKr: String!            # 한국어 유형
  status: SensorStatus!
  statusKr: String!          # 한국어 상태

  """
  위치 정보
  """
  location: SensorLocation!

  """
  측정 사양
  """
  specification: SensorSpecification!

  """
  교정 정보
  """
  calibration: CalibrationInfo!

  """
  제조사 정보
  """
  manufacturer: ManufacturerInfo!

  """
  현재 읽기값
  """
  currentReading: SensorReading

  """
  최근 읽기값들
  """
  recentReadings(limit: Int = 100): [SensorReading!]!

  """
  통계 (시간 범위 지정)
  """
  statistics(start: DateTime!, end: DateTime!): SensorStatistics

  createdAt: DateTime!
  updatedAt: DateTime!
}

"""
센서 위치
"""
type SensorLocation {
  facilityId: ID!
  tankId: ID
  zone: String!
  position: Position!
  coordinates: Coordinates
  locationKr: String!        # 한국어 위치 설명
}

"""
3D 위치 좌표
"""
type Position {
  x: Float!
  y: Float!
  z: Float!
}

"""
GPS 좌표
"""
type Coordinates {
  lat: Float!
  lng: Float!
}

"""
센서 읽기
"""
type SensorReading {
  readingId: ID!
  sensorId: ID!
  value: Float!
  unit: String!
  unitKr: String!            # 한국어 단위
  quality: DataQuality!
  qualityKr: String!         # 한국어 품질
  signalStrength: Float
  batteryLevel: Float
  timestamp: DateTime!
  receivedAt: DateTime!
}

"""
데이터 품질
"""
enum DataQuality {
  GOOD           # 좋음
  FAIR           # 보통
  POOR           # 나쁨
  UNCERTAIN      # 불확실
}

"""
센서 통계
"""
type SensorStatistics {
  sensorId: ID!
  timeRange: TimeRange!
  count: Int!
  min: Float!
  max: Float!
  mean: Float!
  median: Float!
  stdDev: Float!
  trend: TrendAnalysis!
  outliers: OutlierInfo!
  alerts: AlertCounts!
  statisticsKr: String!      # 한국어 통계 설명
}

"""
시간 범위
"""
type TimeRange {
  start: DateTime!
  end: DateTime!
  duration: Int!             # 초
  rangeKr: String!           # 한국어 범위 설명
}

"""
추세 분석
"""
type TrendAnalysis {
  direction: TrendDirection!
  directionKr: String!       # 한국어 방향
  slope: Float!
  correlation: Float!
  trendKr: String!           # 한국어 추세 설명
}

enum TrendDirection {
  INCREASING    # 상승
  STABLE        # 안정
  DECREASING    # 하강
}

"""
이상치 정보
"""
type OutlierInfo {
  count: Int!
  percentage: Float!
  values: [Float!]!
  outliersKr: String!        # 한국어 이상치 설명
}

"""
알림 발생 횟수
"""
type AlertCounts {
  warning: Int!
  critical: Int!
  emergency: Int!
  total: Int!
  alertsKr: String!          # 한국어 알림 설명
}

"""
탱크 정보
"""
type Tank {
  tankId: ID!
  facilityId: ID!
  name: String!
  nameKr: String!            # 한국어 이름
  type: TankType!
  typeKr: String!            # 한국어 유형
  status: TankStatus!
  statusKr: String!          # 한국어 상태

  """
  연결된 센서들
  """
  sensors: [SensorConfig!]!

  """
  현재 상태
  """
  currentState: TankState!

  """
  보관 내용물
  """
  inventory: TankInventory!

  """
  대시보드 데이터
  """
  dashboard: TankDashboard!

  createdAt: DateTime!
  updatedAt: DateTime!
}

enum TankType {
  LN2_STORAGE        # 액체질소 저장 탱크
  LN2_DEWAR         # 듀어 플라스크
  FREEZER_MINUS_80  # -80°C 냉동고
  FREEZER_MINUS_150 # -150°C 냉동고
  DRY_SHIPPER       # 드라이 시퍼
  VAPOR_SHIPPER     # 증기 시퍼
}

enum TankStatus {
  OPERATIONAL    # 정상 운영
  FILLING        # 보충 중
  MAINTENANCE    # 점검 중
  WARNING        # 경고
  CRITICAL       # 위험
  OFFLINE        # 오프라인
}

"""
탱크 현재 상태
"""
type TankState {
  temperature: Float!
  pressure: Float!
  ln2Level: Float!           # 퍼센트
  sampleCount: Int!
  lastRefill: DateTime!
  nextScheduledRefill: DateTime!
  stateKr: String!           # 한국어 상태 설명
}

"""
탱크 보관 내용물
"""
type TankInventory {
  racks: Int!
  canes: Int!
  vials: Int!
  totalSamples: Int!
  sampleTypes: [SampleTypeCount!]!
  inventoryKr: String!       # 한국어 내용 설명
}

type SampleTypeCount {
  type: String!
  typeKr: String!
  count: Int!
}

"""
탱크 대시보드
"""
type TankDashboard {
  currentReadings: CurrentReadings!
  statistics: DashboardStatistics!
  alerts: [Alert!]!
  predictions: Predictions!
  dashboardKr: String!       # 한국어 대시보드 설명
}

type CurrentReadings {
  temperature: [SensorReading!]!
  pressure: [SensorReading!]!
  level: [SensorReading!]!
}

type DashboardStatistics {
  last24Hours: [SensorStatistics!]!
  last7Days: [SensorStatistics!]!
  last30Days: [SensorStatistics!]!
}

type Predictions {
  ln2EmptyDate: DateTime!
  nextRefillDate: DateTime!
  predictionsKr: String!     # 한국어 예측 설명
}

"""
알림
"""
type Alert {
  alertId: ID!
  severity: AlertSeverity!
  severityKr: String!        # 한국어 심각도
  status: AlertStatus!
  statusKr: String!          # 한국어 상태
  title: String!
  titleKr: String!           # 한국어 제목
  message: String!
  messageKr: String!         # 한국어 메시지
  sensor: SensorConfig
  tank: Tank
  facility: Facility!
  occurrence: OccurrenceInfo!
  handling: HandlingInfo!
  createdAt: DateTime!
  updatedAt: DateTime!
}

enum AlertSeverity {
  INFO       # 정보
  WARNING    # 경고
  CRITICAL   # 위험
  EMERGENCY  # 긴급
}

enum AlertStatus {
  ACTIVE         # 활성
  ACKNOWLEDGED   # 확인됨
  RESOLVED       # 해결됨
  ESCALATED      # 에스컬레이션됨
  SUPPRESSED     # 억제됨
}

type OccurrenceInfo {
  firstOccurred: DateTime!
  lastOccurred: DateTime!
  count: Int!
  duration: Int!             # 초
  occurrenceKr: String!      # 한국어 발생 정보
}

type HandlingInfo {
  acknowledgedBy: String
  acknowledgedAt: DateTime
  resolvedBy: String
  resolvedAt: DateTime
  resolutionNotes: String
  handlingKr: String!        # 한국어 처리 정보
}

"""
시설
"""
type Facility {
  facilityId: ID!
  name: String!
  nameKr: String!            # 한국어 이름
  type: FacilityType!
  typeKr: String!            # 한국어 유형
  contact: ContactInfo!
  tanks: [Tank!]!
  sensors: [SensorConfig!]!
  activeAlerts: [Alert!]!
  wiaCompliance: WIACompliance!
  createdAt: DateTime!
  updatedAt: DateTime!
}

enum FacilityType {
  CORD_BLOOD_BANK    # 제대혈 은행
  FERTILITY_CLINIC   # 난임 클리닉
  BIOBANK            # 바이오뱅크
  HOSPITAL           # 병원
  RESEARCH_INSTITUTE # 연구소
  PHARMACEUTICAL     # 제약회사
  LOGISTICS          # 물류 센터
}

type ContactInfo {
  address: String!
  addressKr: String!         # 한국어 주소
  coordinates: Coordinates!
  phone: String!
  email: String!
  website: String
  contactKr: String!         # 한국어 연락처 설명
}

type WIACompliance {
  adopted: Boolean!
  version: String!
  implementationDate: DateTime
  certificationDate: DateTime
  complianceKr: String!      # 한국어 준수 설명
}

"""
쿼리
"""
type Query {
  """
  센서 조회
  """
  sensor(sensorId: ID!): SensorConfig
  sensors(
    facilityId: ID
    type: SensorType
    status: SensorStatus
    page: Int = 1
    limit: Int = 20
  ): SensorPage!

  """
  탱크 조회
  """
  tank(tankId: ID!): Tank
  tanks(
    facilityId: ID
    type: TankType
    status: TankStatus
    page: Int = 1
    limit: Int = 20
  ): TankPage!

  """
  알림 조회
  """
  alert(alertId: ID!): Alert
  activeAlerts(
    facilityId: ID
    severity: AlertSeverity
    page: Int = 1
    limit: Int = 50
  ): AlertPage!

  """
  시설 조회
  """
  facility(facilityId: ID!): Facility
  facilities(
    type: FacilityType
    page: Int = 1
    limit: Int = 20
  ): FacilityPage!

  """
  대시보드
  """
  dashboard(facilityId: ID!): Dashboard!
}

type SensorPage {
  sensors: [SensorConfig!]!
  total: Int!
  page: Int!
  limit: Int!
  totalPages: Int!
}

type TankPage {
  tanks: [Tank!]!
  total: Int!
  page: Int!
  limit: Int!
}

type AlertPage {
  alerts: [Alert!]!
  total: Int!
  page: Int!
  limit: Int!
  summary: AlertSummary!
}

type AlertSummary {
  emergency: Int!
  critical: Int!
  warning: Int!
  info: Int!
  summaryKr: String!         # 한국어 요약
}

type FacilityPage {
  facilities: [Facility!]!
  total: Int!
  page: Int!
  limit: Int!
}

type Dashboard {
  facility: Facility!
  overviewStats: OverviewStats!
  recentAlerts: [Alert!]!
  tankSummary: [TankSummary!]!
  sensorHealth: SensorHealthSummary!
  dashboardKr: String!       # 한국어 대시보드 설명
}

type OverviewStats {
  totalTanks: Int!
  totalSensors: Int!
  totalSamples: Int!
  activeAlerts: Int!
  statsKr: String!           # 한국어 통계 설명
}

type TankSummary {
  tank: Tank!
  status: String!
  statusKr: String!
  temperature: Float!
  ln2Level: Float!
}

type SensorHealthSummary {
  total: Int!
  active: Int!
  error: Int!
  offline: Int!
  healthKr: String!          # 한국어 건강 상태
}

"""
뮤테이션
"""
type Mutation {
  """
  센서 관리
  """
  createSensor(input: CreateSensorInput!): SensorConfig!
  updateSensor(sensorId: ID!, input: UpdateSensorInput!): SensorConfig!
  deleteSensor(sensorId: ID!): DeleteResult!
  calibrateSensor(sensorId: ID!, input: CalibrationInput!): SensorConfig!

  """
  탱크 관리
  """
  createTank(input: CreateTankInput!): Tank!
  updateTank(tankId: ID!, input: UpdateTankInput!): Tank!
  deleteTank(tankId: ID!): DeleteResult!
  recordRefill(tankId: ID!, input: RefillInput!): RefillResult!
  recordMaintenance(tankId: ID!, input: MaintenanceInput!): MaintenanceResult!

  """
  알림 관리
  """
  acknowledgeAlert(alertId: ID!, input: AcknowledgeInput!): AlertResult!
  resolveAlert(alertId: ID!, input: ResolveInput!): AlertResult!
  escalateAlert(alertId: ID!, input: EscalateInput!): AlertResult!
}

input CreateSensorInput {
  name: String!
  nameKr: String!
  type: SensorType!
  facilityId: ID!
  # ... 기타 필드
}

input UpdateSensorInput {
  name: String
  nameKr: String
  status: SensorStatus
  # ... 기타 필드
}

input CalibrationInput {
  calibratedBy: String!
  certificate: String
  notes: String
}

input CreateTankInput {
  name: String!
  nameKr: String!
  type: TankType!
  facilityId: ID!
  # ... 기타 필드
}

input UpdateTankInput {
  name: String
  nameKr: String
  status: TankStatus
  # ... 기타 필드
}

input RefillInput {
  volume: Float!
  performedBy: String!
  supplier: String!
  batchNumber: String
  notes: String
}

input MaintenanceInput {
  type: String!
  typeKr: String!
  performedBy: String!
  notes: String
  documentUrl: String
}

input AcknowledgeInput {
  acknowledgedBy: String!
  notes: String
}

input ResolveInput {
  resolvedBy: String!
  resolutionNotes: String!
}

input EscalateInput {
  escalatedBy: String!
  escalateTo: [String!]!
  reason: String!
  reasonKr: String!
}

type DeleteResult {
  success: Boolean!
  message: String!
  messageKr: String!
}

type RefillResult {
  success: Boolean!
  tank: Tank!
  message: String!
  messageKr: String!
}

type MaintenanceResult {
  success: Boolean!
  tank: Tank!
  messageKr: String!
}

type AlertResult {
  success: Boolean!
  alert: Alert!
  messageKr: String!
}

"""
구독 (실시간)
"""
type Subscription {
  """
  센서 읽기 구독
  """
  sensorReadings(sensorId: ID!): SensorReading!
  sensorReadingsBatch(sensorIds: [ID!]!): [SensorReading!]!

  """
  알림 구독
  """
  newAlerts(facilityId: ID!): Alert!
  alertUpdates(alertId: ID!): Alert!

  """
  탱크 상태 구독
  """
  tankState(tankId: ID!): TankState!
}

scalar DateTime
```

### GraphQL TypeScript 클라이언트

```typescript
/**
 * GraphQL TypeScript 클라이언트
 */
import { GraphQLClient, gql } from "graphql-request";

class CryoGraphQLClient {
  private client: GraphQLClient;

  constructor(endpoint: string, apiKey: string) {
    this.client = new GraphQLClient(endpoint, {
      headers: {
        Authorization: `Bearer ${apiKey}`
      }
    });
  }

  /**
   * 센서 조회
   */
  async getSensor(sensorId: string) {
    const query = gql`
      query GetSensor($sensorId: ID!) {
        sensor(sensorId: $sensorId) {
          sensorId
          name
          nameKr
          type
          typeKr
          status
          statusKr
          location {
            locationKr
          }
          currentReading {
            value
            unit
            unitKr
            quality
            qualityKr
            timestamp
          }
        }
      }
    `;

    return this.client.request(query, { sensorId });
  }

  /**
   * 탱크 대시보드
   */
  async getTankDashboard(tankId: string) {
    const query = gql`
      query GetTankDashboard($tankId: ID!) {
        tank(tankId: $tankId) {
          tankId
          name
          nameKr
          dashboard {
            currentReadings {
              temperature {
                value
                unitKr
                timestamp
              }
              pressure {
                value
                unitKr
                timestamp
              }
              level {
                value
                unitKr
                timestamp
              }
            }
            alerts {
              alertId
              severity
              severityKr
              titleKr
              messageKr
            }
            predictions {
              ln2EmptyDate
              nextRefillDate
              predictionsKr
            }
            dashboardKr
          }
        }
      }
    `;

    return this.client.request(query, { tankId });
  }

  /**
   * 활성 알림 조회
   */
  async getActiveAlerts(facilityId: string) {
    const query = gql`
      query GetActiveAlerts($facilityId: ID!) {
        activeAlerts(facilityId: $facilityId, page: 1, limit: 50) {
          alerts {
            alertId
            severity
            severityKr
            status
            statusKr
            titleKr
            messageKr
            occurrence {
              firstOccurred
              count
              occurrenceKr
            }
          }
          summary {
            emergency
            critical
            warning
            info
            summaryKr
          }
        }
      }
    `;

    return this.client.request(query, { facilityId });
  }

  /**
   * 알림 확인
   */
  async acknowledgeAlert(alertId: string, acknowledgedBy: string, notes?: string) {
    const mutation = gql`
      mutation AcknowledgeAlert($alertId: ID!, $input: AcknowledgeInput!) {
        acknowledgeAlert(alertId: $alertId, input: $input) {
          success
          alert {
            alertId
            statusKr
          }
          messageKr
        }
      }
    `;

    return this.client.request(mutation, {
      alertId,
      input: { acknowledgedBy, notes }
    });
  }

  /**
   * 실시간 센서 읽기 구독
   */
  subscribeToSensor(sensorId: string, callback: (reading: any) => void) {
    const subscription = gql`
      subscription SensorReadings($sensorId: ID!) {
        sensorReadings(sensorId: $sensorId) {
          readingId
          value
          unitKr
          qualityKr
          timestamp
        }
      }
    `;

    // WebSocket 구독 (실제 구현은 WebSocket 라이브러리 필요)
    console.log(`센서 ${sensorId} 구독 시작`);
  }
}

// 사용 예시
const graphqlClient = new CryoGraphQLClient(
  "https://api.cryo-monitor.wia.org/graphql",
  "your-api-key"
);

// 센서 조회
const sensor = await graphqlClient.getSensor("sensor-id");
console.log(`센서: ${sensor.sensor.nameKr}`);
console.log(`현재 온도: ${sensor.sensor.currentReading.value}${sensor.sensor.currentReading.unitKr}`);

// 탱크 대시보드
const dashboard = await graphqlClient.getTankDashboard("tank-id");
console.log(dashboard.tank.dashboard.dashboardKr);

// 알림 확인
const result = await graphqlClient.acknowledgeAlert(
  "alert-id",
  "김영희",
  "확인했습니다. 조치 중입니다."
);
console.log(result.acknowledgeAlert.messageKr);
```

---

## WebSocket 실시간 스트리밍

### WebSocket 프로토콜

```typescript
/**
 * WebSocket 실시간 스트리밍 클라이언트
 */
import WebSocket from "ws";

interface WebSocketMessage {
  type: "subscribe" | "unsubscribe" | "data" | "error" | "ping" | "pong";
  payload: any;
  timestamp: Date;
}

class CryoWebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private apiKey: string;
  private reconnectInterval: number = 5000;
  private pingInterval: NodeJS.Timeout | null = null;
  private subscriptions: Set<string> = new Set();

  constructor(url: string, apiKey: string) {
    this.url = url;
    this.apiKey = apiKey;
  }

  /**
   * 연결
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url, {
        headers: {
          Authorization: `Bearer ${this.apiKey}`
        }
      });

      this.ws.on("open", () => {
        console.log("[WebSocket] 연결 성공");
        this.startPing();

        // 이전 구독 복원
        this.subscriptions.forEach(sensorId => {
          this.subscribe(sensorId);
        });

        resolve();
      });

      this.ws.on("message", (data: string) => {
        try {
          const message: WebSocketMessage = JSON.parse(data);
          this.handleMessage(message);
        } catch (error) {
          console.error("[WebSocket] 메시지 파싱 오류:", error);
        }
      });

      this.ws.on("error", (error) => {
        console.error("[WebSocket] 오류:", error);
        reject(error);
      });

      this.ws.on("close", () => {
        console.log("[WebSocket] 연결 종료");
        this.stopPing();
        this.reconnect();
      });
    });
  }

  /**
   * 센서 구독
   */
  subscribe(sensorId: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn("[WebSocket] 연결되지 않음. 재연결 후 구독됩니다.");
      this.subscriptions.add(sensorId);
      return;
    }

    const message: WebSocketMessage = {
      type: "subscribe",
      payload: { sensorId },
      timestamp: new Date()
    };

    this.ws.send(JSON.stringify(message));
    this.subscriptions.add(sensorId);
    console.log(`[WebSocket] 센서 구독: ${sensorId}`);
  }

  /**
   * 센서 구독 해제
   */
  unsubscribe(sensorId: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const message: WebSocketMessage = {
      type: "unsubscribe",
      payload: { sensorId },
      timestamp: new Date()
    };

    this.ws.send(JSON.stringify(message));
    this.subscriptions.delete(sensorId);
    console.log(`[WebSocket] 센서 구독 해제: ${sensorId}`);
  }

  /**
   * 메시지 처리
   */
  private handleMessage(message: WebSocketMessage): void {
    switch (message.type) {
      case "data":
        this.emit("data", message.payload);
        break;
      case "error":
        this.emit("error", message.payload);
        break;
      case "pong":
        // 하트비트 응답
        break;
      default:
        console.log("[WebSocket] 알 수 없는 메시지 유형:", message.type);
    }
  }

  /**
   * 하트비트 시작
   */
  private startPing(): void {
    this.pingInterval = setInterval(() => {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        const message: WebSocketMessage = {
          type: "ping",
          payload: {},
          timestamp: new Date()
        };
        this.ws.send(JSON.stringify(message));
      }
    }, 30000); // 30초마다
  }

  /**
   * 하트비트 중지
   */
  private stopPing(): void {
    if (this.pingInterval) {
      clearInterval(this.pingInterval);
      this.pingInterval = null;
    }
  }

  /**
   * 재연결
   */
  private reconnect(): void {
    console.log(`[WebSocket] ${this.reconnectInterval / 1000}초 후 재연결 시도...`);
    setTimeout(() => {
      this.connect().catch(error => {
        console.error("[WebSocket] 재연결 실패:", error);
      });
    }, this.reconnectInterval);
  }

  /**
   * 이벤트 에미터
   */
  private listeners: Map<string, Function[]> = new Map();

  on(event: string, callback: Function): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, []);
    }
    this.listeners.get(event)!.push(callback);
  }

  private emit(event: string, data: any): void {
    const callbacks = this.listeners.get(event);
    if (callbacks) {
      callbacks.forEach(callback => callback(data));
    }
  }

  /**
   * 연결 종료
   */
  disconnect(): void {
    if (this.ws) {
      this.stopPing();
      this.ws.close();
      this.ws = null;
    }
  }
}

// 사용 예시
const wsClient = new CryoWebSocketClient(
  "wss://stream.cryo-monitor.wia.org/v1",
  "your-api-key"
);

// 데이터 수신 이벤트
wsClient.on("data", (reading: SensorReading) => {
  console.log(`[센서 데이터] ${reading.sensorId}: ${reading.value}${reading.unitKr}`);
  console.log(`품질: ${reading.qualityKr}, 시간: ${reading.timestamp}`);
});

// 오류 이벤트
wsClient.on("error", (error: any) => {
  console.error("[오류]", error);
});

// 연결
await wsClient.connect();

// 센서 구독
wsClient.subscribe("sensor-id-1");
wsClient.subscribe("sensor-id-2");
wsClient.subscribe("sensor-id-3");

// 나중에 구독 해제
// wsClient.unsubscribe("sensor-id-1");

// 연결 종료
// wsClient.disconnect();
```

---

## 인증 및 보안

### API 키 관리

```typescript
/**
 * API 인증
 */
interface APIAuthentication {
  // API 키
  apiKey: {
    key: string;
    type: "production" | "staging" | "development";
    permissions: string[];
    rateLimit: {
      requests: number;          // 요청 수
      period: number;            // 기간 (초)
      rateLimitKr: string;       // 한국어 설명
    };
    createdAt: Date;
    expiresAt: Date;
    authKr: string;
  };

  // JWT 토큰
  jwt: {
    token: string;
    refreshToken: string;
    expiresIn: number;           // 초
    tokenType: "Bearer";
    jwtKr: string;
  };
}

/**
 * 인증 클라이언트
 */
class AuthClient {
  private baseUrl: string;

  constructor(baseUrl: string) {
    this.baseUrl = baseUrl;
  }

  /**
   * 로그인
   */
  async login(credentials: {
    email: string;
    password: string;
    mfaCode?: string;            // 2단계 인증 코드
  }): Promise<{
    user: User;
    apiKey: string;
    jwt: {
      accessToken: string;
      refreshToken: string;
      expiresIn: number;
    };
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/auth/login`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(credentials)
    });

    if (!response.ok) {
      throw new Error(`로그인 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 토큰 갱신
   */
  async refreshToken(refreshToken: string): Promise<{
    accessToken: string;
    expiresIn: number;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/auth/refresh`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify({ refreshToken })
    });

    if (!response.ok) {
      throw new Error(`토큰 갱신 실패: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * 로그아웃
   */
  async logout(token: string): Promise<{
    success: boolean;
    messageKr: string;
  }> {
    const response = await fetch(`${this.baseUrl}/auth/logout`, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${token}`
      }
    });

    if (!response.ok) {
      throw new Error(`로그아웃 실패: ${response.statusText}`);
    }

    return response.json();
  }
}
```

---

## 속도 제한 및 할당량

```typescript
/**
 * 속도 제한 정책
 */
interface RateLimitPolicy {
  tier: "free" | "basic" | "pro" | "enterprise";
  tierKr: string;
  limits: {
    rest: {
      requests: number;          // 시간당 요청 수
      period: "hour" | "day";
      limitKr: string;
    };
    graphql: {
      queries: number;           // 시간당 쿼리 수
      complexity: number;        // 최대 복잡도
      limitKr: string;
    };
    websocket: {
      connections: number;       // 동시 연결 수
      messagesPerSecond: number; // 초당 메시지 수
      limitKr: string;
    };
  };
  policyKr: string;
}

const rateLimits: Record<string, RateLimitPolicy> = {
  free: {
    tier: "free",
    tierKr: "무료 플랜",
    limits: {
      rest: {
        requests: 1000,
        period: "hour",
        limitKr: "시간당 1,000 요청"
      },
      graphql: {
        queries: 500,
        complexity: 100,
        limitKr: "시간당 500 쿼리, 복잡도 100"
      },
      websocket: {
        connections: 5,
        messagesPerSecond: 10,
        limitKr: "동시 연결 5개, 초당 10 메시지"
      }
    },
    policyKr: "무료 플랜 - 기본 모니터링"
  },
  enterprise: {
    tier: "enterprise",
    tierKr: "엔터프라이즈 플랜",
    limits: {
      rest: {
        requests: 100000,
        period: "hour",
        limitKr: "시간당 100,000 요청"
      },
      graphql: {
        queries: 50000,
        complexity: 10000,
        limitKr: "시간당 50,000 쿼리, 복잡도 10,000"
      },
      websocket: {
        connections: 1000,
        messagesPerSecond: 1000,
        limitKr: "동시 연결 1,000개, 초당 1,000 메시지"
      }
    },
    policyKr: "엔터프라이즈 플랜 - 무제한 모니터링"
  }
};
```

---

## 결론

WIA Cryo Monitoring API는 REST, GraphQL, WebSocket을 지원하는 완전한 API 플랫폼을 제공합니다.

### 핵심 특징

1. **다중 프로토콜**: REST, GraphQL, WebSocket
2. **한국어 지원**: 모든 응답에 한국어 필드
3. **실시간 스트리밍**: WebSocket 기반
4. **타입 안전성**: TypeScript + Zod

### 다음 장 예고

다음 장에서는 **센서 관리**를 다룹니다:
- 센서 설정 및 교정
- 상태 모니터링
- 센서 네트워크 관리
- 한국어 관리 인터페이스

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
