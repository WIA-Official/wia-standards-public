# 제4장: 커넥티드카 API 인터페이스

## 차량 데이터 접근을 위한 REST, GraphQL 및 스트리밍 API

이 장에서는 커넥티드카 데이터에 접근하기 위한 포괄적인 API 명세를 정의하여 서드파티 개발자와 에코시스템 파트너들이 혁신적인 모빌리티 애플리케이션을 구축할 수 있도록 합니다.

---

## RESTful API 명세

### 기본 구성

```typescript
// WIA 커넥티드카 API - OpenAPI 3.1 명세
// 차량 데이터 및 제어를 위한 완전한 REST API

/**
 * API 구성
 */
interface APIConfiguration {
  baseUrl: "https://api.wia-connectedcar.org";
  version: "v1";
  authentication: "OAuth 2.0 + mTLS";
  rateLimiting: RateLimitConfig;
  throttling: ThrottlingConfig;
}

interface RateLimitConfig {
  standard: {
    requestsPerMinute: 60;
    requestsPerHour: 1000;
    requestsPerDay: 10000;
  };
  premium: {
    requestsPerMinute: 600;
    requestsPerHour: 10000;
    requestsPerDay: 100000;
  };
  enterprise: {
    requestsPerMinute: 6000;
    requestsPerHour: 100000;
    requestsPerDay: 1000000;
  };
}

interface ThrottlingConfig {
  burstLimit: number;
  sustainedLimit: number;
  backoffStrategy: "exponential";
  maxBackoffSeconds: 300;
}

/**
 * OpenAPI 스키마 정의
 */
const openAPISchema = {
  openapi: "3.1.0",
  info: {
    title: "WIA 커넥티드카 API",
    version: "1.0.0",
    description: "차량 데이터 접근 및 제어를 위한 포괄적 API",
    contact: {
      name: "WIA 기술 지원",
      email: "api-support@wia.org",
      url: "https://developers.wia.org"
    },
    license: {
      name: "WIA API 라이선스",
      url: "https://wia.org/api-license"
    }
  },
  servers: [
    {
      url: "https://api.wia-connectedcar.org/v1",
      description: "프로덕션 서버"
    },
    {
      url: "https://sandbox.wia-connectedcar.org/v1",
      description: "개발용 샌드박스"
    }
  ],
  paths: {
    "/vehicles": {
      get: {
        operationId: "listVehicles",
        summary: "인증된 사용자의 모든 차량 목록",
        tags: ["Vehicles"],
        security: [{ oauth2: ["vehicles:read"] }],
        parameters: [
          { name: "page", in: "query", schema: { type: "integer", default: 1 } },
          { name: "limit", in: "query", schema: { type: "integer", default: 20, max: 100 } },
          { name: "status", in: "query", schema: { type: "string", enum: ["active", "inactive", "all"] } }
        ],
        responses: {
          200: {
            description: "차량 목록",
            content: {
              "application/json": {
                schema: { $ref: "#/components/schemas/VehicleListResponse" }
              }
            }
          }
        }
      }
    },
    "/vehicles/{vehicleId}": {
      get: {
        operationId: "getVehicle",
        summary: "차량 상세 정보 조회",
        tags: ["Vehicles"],
        security: [{ oauth2: ["vehicles:read"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } }
        ],
        responses: {
          200: {
            description: "차량 상세 정보",
            content: {
              "application/json": {
                schema: { $ref: "#/components/schemas/Vehicle" }
              }
            }
          }
        }
      }
    },
    "/vehicles/{vehicleId}/location": {
      get: {
        operationId: "getVehicleLocation",
        summary: "현재 차량 위치 조회",
        tags: ["Telematics"],
        security: [{ oauth2: ["location:read"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } }
        ],
        responses: {
          200: {
            description: "현재 위치",
            content: {
              "application/json": {
                schema: { $ref: "#/components/schemas/VehicleLocation" }
              }
            }
          }
        }
      }
    },
    "/vehicles/{vehicleId}/telemetry": {
      get: {
        operationId: "getVehicleTelemetry",
        summary: "차량 텔레매틱스 데이터 조회",
        tags: ["Telematics"],
        security: [{ oauth2: ["telemetry:read"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } },
          { name: "signals", in: "query", schema: { type: "array", items: { type: "string" } } },
          { name: "from", in: "query", schema: { type: "string", format: "date-time" } },
          { name: "to", in: "query", schema: { type: "string", format: "date-time" } }
        ],
        responses: {
          200: {
            description: "텔레매틱스 데이터",
            content: {
              "application/json": {
                schema: { $ref: "#/components/schemas/TelemetryResponse" }
              }
            }
          }
        }
      }
    },
    "/vehicles/{vehicleId}/commands": {
      post: {
        operationId: "sendCommand",
        summary: "차량에 명령 전송",
        tags: ["Commands"],
        security: [{ oauth2: ["commands:write"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } }
        ],
        requestBody: {
          required: true,
          content: {
            "application/json": {
              schema: { $ref: "#/components/schemas/VehicleCommand" }
            }
          }
        },
        responses: {
          202: {
            description: "명령 수락됨",
            content: {
              "application/json": {
                schema: { $ref: "#/components/schemas/CommandResponse" }
              }
            }
          }
        }
      }
    }
  },
  components: {
    schemas: {
      Vehicle: {
        type: "object",
        properties: {
          id: { type: "string", format: "uuid" },
          vin: { type: "string", pattern: "^[A-HJ-NPR-Z0-9]{17}$" },
          make: { type: "string" },
          model: { type: "string" },
          year: { type: "integer" },
          color: { type: "string" },
          powertrainType: { type: "string", enum: ["ICE", "HYBRID", "PHEV", "BEV", "FCEV"] },
          status: { $ref: "#/components/schemas/VehicleStatus" },
          capabilities: { type: "array", items: { type: "string" } },
          lastSeen: { type: "string", format: "date-time" }
        }
      },
      VehicleLocation: {
        type: "object",
        properties: {
          latitude: { type: "number", minimum: -90, maximum: 90 },
          longitude: { type: "number", minimum: -180, maximum: 180 },
          altitude: { type: "number" },
          heading: { type: "number", minimum: 0, maximum: 360 },
          speed: { type: "number", minimum: 0 },
          accuracy: { type: "number" },
          timestamp: { type: "string", format: "date-time" }
        }
      },
      VehicleCommand: {
        type: "object",
        required: ["command"],
        properties: {
          command: {
            type: "string",
            enum: [
              "LOCK", "UNLOCK", "START_ENGINE", "STOP_ENGINE",
              "CLIMATE_ON", "CLIMATE_OFF", "SET_CLIMATE_TEMP",
              "FLASH_LIGHTS", "HONK_HORN", "OPEN_TRUNK",
              "START_CHARGING", "STOP_CHARGING", "SET_CHARGE_LIMIT"
            ]
          },
          parameters: { type: "object" },
          timeout: { type: "integer", default: 30 }
        }
      }
    },
    securitySchemes: {
      oauth2: {
        type: "oauth2",
        flows: {
          authorizationCode: {
            authorizationUrl: "https://auth.wia-connectedcar.org/authorize",
            tokenUrl: "https://auth.wia-connectedcar.org/token",
            scopes: {
              "vehicles:read": "차량 정보 읽기",
              "location:read": "차량 위치 읽기",
              "telemetry:read": "차량 텔레매틱스 읽기",
              "commands:write": "차량에 명령 전송",
              "diagnostics:read": "진단 데이터 읽기",
              "charging:write": "충전 제어"
            }
          }
        }
      }
    }
  }
};

/**
 * REST API 클라이언트 구현
 */
class ConnectedCarAPIClient {
  private baseUrl: string;
  private accessToken: string;
  private refreshToken: string;
  private tokenExpiry: Date;

  constructor(config: ClientConfig) {
    this.baseUrl = config.baseUrl || "https://api.wia-connectedcar.org/v1";
  }

  /**
   * OAuth 2.0 인가 코드 플로우
   */
  async authorize(authCode: string): Promise<TokenResponse> {
    const response = await fetch(`${this.baseUrl}/auth/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code: authCode,
        client_id: process.env.CLIENT_ID!,
        client_secret: process.env.CLIENT_SECRET!,
        redirect_uri: process.env.REDIRECT_URI!
      })
    });

    const tokens = await response.json();
    this.accessToken = tokens.access_token;
    this.refreshToken = tokens.refresh_token;
    this.tokenExpiry = new Date(Date.now() + tokens.expires_in * 1000);

    return tokens;
  }

  /**
   * 접근 토큰 갱신
   */
  async refreshAccessToken(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/auth/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "refresh_token",
        refresh_token: this.refreshToken,
        client_id: process.env.CLIENT_ID!,
        client_secret: process.env.CLIENT_SECRET!
      })
    });

    const tokens = await response.json();
    this.accessToken = tokens.access_token;
    this.tokenExpiry = new Date(Date.now() + tokens.expires_in * 1000);
  }

  /**
   * 인증된 API 요청 수행
   */
  private async request<T>(
    method: string,
    path: string,
    options: RequestOptions = {}
  ): Promise<T> {
    // 만료된 경우 토큰 자동 갱신
    if (this.tokenExpiry && new Date() >= this.tokenExpiry) {
      await this.refreshAccessToken();
    }

    const url = new URL(path, this.baseUrl);

    if (options.query) {
      Object.entries(options.query).forEach(([key, value]) => {
        if (value !== undefined) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    const response = await fetch(url.toString(), {
      method,
      headers: {
        "Authorization": `Bearer ${this.accessToken}`,
        "Content-Type": "application/json",
        "X-Request-ID": crypto.randomUUID(),
        ...options.headers
      },
      body: options.body ? JSON.stringify(options.body) : undefined
    });

    if (!response.ok) {
      const error = await response.json();
      throw new APIError(response.status, error.code, error.message);
    }

    return response.json();
  }

  // 차량 엔드포인트
  async listVehicles(params?: ListVehiclesParams): Promise<VehicleListResponse> {
    return this.request("GET", "/vehicles", { query: params });
  }

  async getVehicle(vehicleId: string): Promise<Vehicle> {
    return this.request("GET", `/vehicles/${vehicleId}`);
  }

  async getVehicleLocation(vehicleId: string): Promise<VehicleLocation> {
    return this.request("GET", `/vehicles/${vehicleId}/location`);
  }

  async getVehicleTelemetry(
    vehicleId: string,
    params?: TelemetryParams
  ): Promise<TelemetryResponse> {
    return this.request("GET", `/vehicles/${vehicleId}/telemetry`, { query: params });
  }

  // 명령 엔드포인트
  async sendCommand(
    vehicleId: string,
    command: VehicleCommand
  ): Promise<CommandResponse> {
    return this.request("POST", `/vehicles/${vehicleId}/commands`, { body: command });
  }

  async getCommandStatus(
    vehicleId: string,
    commandId: string
  ): Promise<CommandStatusResponse> {
    return this.request("GET", `/vehicles/${vehicleId}/commands/${commandId}`);
  }

  // 충전 엔드포인트
  async getChargingStatus(vehicleId: string): Promise<ChargingStatus> {
    return this.request("GET", `/vehicles/${vehicleId}/charging`);
  }

  async startCharging(vehicleId: string): Promise<CommandResponse> {
    return this.sendCommand(vehicleId, { command: "START_CHARGING" });
  }

  async stopCharging(vehicleId: string): Promise<CommandResponse> {
    return this.sendCommand(vehicleId, { command: "STOP_CHARGING" });
  }

  async setChargeLimit(vehicleId: string, limit: number): Promise<CommandResponse> {
    return this.sendCommand(vehicleId, {
      command: "SET_CHARGE_LIMIT",
      parameters: { limit }
    });
  }

  // 위치 이력
  async getLocationHistory(
    vehicleId: string,
    params: LocationHistoryParams
  ): Promise<LocationHistoryResponse> {
    return this.request("GET", `/vehicles/${vehicleId}/location/history`, { query: params });
  }

  // 트립
  async listTrips(
    vehicleId: string,
    params?: ListTripsParams
  ): Promise<TripListResponse> {
    return this.request("GET", `/vehicles/${vehicleId}/trips`, { query: params });
  }

  async getTrip(vehicleId: string, tripId: string): Promise<Trip> {
    return this.request("GET", `/vehicles/${vehicleId}/trips/${tripId}`);
  }
}

// 타입 정의
interface ClientConfig {
  baseUrl?: string;
}

interface TokenResponse {
  access_token: string;
  refresh_token: string;
  expires_in: number;
  token_type: string;
  scope: string;
}

interface RequestOptions {
  query?: Record<string, any>;
  body?: any;
  headers?: Record<string, string>;
}

class APIError extends Error {
  constructor(
    public status: number,
    public code: string,
    message: string
  ) {
    super(message);
    this.name = "APIError";
  }
}

interface VehicleListResponse {
  vehicles: Vehicle[];
  pagination: Pagination;
}

interface Pagination {
  page: number;
  limit: number;
  total: number;
  totalPages: number;
}

interface Vehicle {
  id: string;
  vin: string;
  make: string;
  model: string;
  year: number;
  color: string;
  powertrainType: string;
  status: VehicleStatus;
  capabilities: string[];
  lastSeen: string;
}

interface VehicleStatus {
  online: boolean;
  locked: boolean;
  engineRunning: boolean;
  charging: boolean;
}

interface VehicleLocation {
  latitude: number;
  longitude: number;
  altitude: number;
  heading: number;
  speed: number;
  accuracy: number;
  timestamp: string;
}

interface VehicleCommand {
  command: string;
  parameters?: Record<string, any>;
  timeout?: number;
}

interface CommandResponse {
  commandId: string;
  status: "ACCEPTED" | "PENDING" | "IN_PROGRESS" | "COMPLETED" | "FAILED";
  message?: string;
  estimatedCompletion?: string;
}

interface TelemetryResponse {
  vehicleId: string;
  timestamp: string;
  signals: Record<string, TelemetrySignal>;
}

interface TelemetrySignal {
  value: any;
  timestamp: string;
  quality: string;
}

interface ChargingStatus {
  isCharging: boolean;
  chargeType?: string;
  stateOfCharge: number;
  chargeLimit: number;
  chargePower?: number;
  timeToFull?: number;
  estimatedRange: number;
}
```

---

## GraphQL API

### 스키마 정의

```typescript
// 커넥티드카 API용 GraphQL 스키마
// 실시간 구독을 통한 유연한 쿼리

const graphqlSchema = `
"""
루트 쿼리 타입
"""
type Query {
  """현재 사용자의 차량 조회"""
  vehicles(
    page: Int = 1
    limit: Int = 20
    status: VehicleStatusFilter
  ): VehicleConnection!

  """ID로 특정 차량 조회"""
  vehicle(id: ID!): Vehicle

  """VIN으로 차량 조회"""
  vehicleByVin(vin: String!): Vehicle

  """유연한 신호 선택으로 텔레매틱스 데이터 조회"""
  telemetry(
    vehicleId: ID!
    signals: [String!]
    from: DateTime
    to: DateTime
    resolution: TimeResolution
  ): TelemetryData

  """트립 이력 조회"""
  trips(
    vehicleId: ID!
    from: DateTime
    to: DateTime
    first: Int
    after: String
  ): TripConnection!

  """위치 근처 충전소 검색"""
  chargingStations(
    latitude: Float!
    longitude: Float!
    radius: Float! = 10
    connectorTypes: [ConnectorType!]
    available: Boolean
  ): [ChargingStation!]!
}

"""
루트 뮤테이션 타입
"""
type Mutation {
  """차량에 명령 전송"""
  sendCommand(
    vehicleId: ID!
    command: CommandInput!
  ): CommandResult!

  """차량 설정 업데이트"""
  updateVehicleSettings(
    vehicleId: ID!
    settings: VehicleSettingsInput!
  ): Vehicle!

  """충전 스케줄 설정"""
  setChargingSchedule(
    vehicleId: ID!
    schedule: ChargingScheduleInput!
  ): ChargingSchedule!

  """지오펜스 생성"""
  createGeofence(
    vehicleId: ID!
    geofence: GeofenceInput!
  ): Geofence!

  """지오펜스 삭제"""
  deleteGeofence(
    vehicleId: ID!
    geofenceId: ID!
  ): Boolean!
}

"""
루트 구독 타입
"""
type Subscription {
  """차량 위치 업데이트 구독"""
  locationUpdated(vehicleId: ID!): VehicleLocation!

  """텔레매틱스 스트림 구독"""
  telemetryStream(
    vehicleId: ID!
    signals: [String!]!
    interval: Int = 1000
  ): TelemetryUpdate!

  """차량 이벤트 구독"""
  vehicleEvents(
    vehicleId: ID!
    eventTypes: [EventType!]
  ): VehicleEvent!

  """명령 상태 업데이트 구독"""
  commandStatus(commandId: ID!): CommandResult!

  """충전 상태 업데이트 구독"""
  chargingStatus(vehicleId: ID!): ChargingStatus!
}

"""
모든 관련 데이터가 포함된 차량 타입
"""
type Vehicle {
  id: ID!
  vin: String!
  make: String!
  model: String!
  year: Int!
  color: String
  nickname: String
  powertrainType: PowertrainType!

  """현재 차량 상태"""
  status: VehicleStatus!

  """현재 위치 (location:read 스코프 필요)"""
  location: VehicleLocation

  """최신 텔레매틱스 스냅샷"""
  telemetry(signals: [String!]): TelemetryData

  """배터리/연료 상태"""
  energy: EnergyStatus!

  """충전 정보 (EV 전용)"""
  charging: ChargingStatus

  """활성 진단 고장 코드"""
  diagnostics: [DiagnosticCode!]!

  """차량 기능"""
  capabilities: [Capability!]!

  """공조 상태"""
  climate: ClimateStatus

  """설정된 지오펜스"""
  geofences: [Geofence!]!

  """최근 트립"""
  recentTrips(limit: Int = 5): [Trip!]!

  """마지막 통신 타임스탬프"""
  lastSeen: DateTime!

  """차량 설정"""
  settings: VehicleSettings!
}

type VehicleStatus {
  online: Boolean!
  locked: Boolean!
  doorsOpen: [Door!]!
  windowsOpen: [Window!]!
  trunkOpen: Boolean!
  hoodOpen: Boolean!
  engineRunning: Boolean!
  ignitionStatus: IgnitionStatus!
  odometer: Float!
  timestamp: DateTime!
}

type VehicleLocation {
  latitude: Float!
  longitude: Float!
  altitude: Float
  heading: Float
  speed: Float
  accuracy: Float
  source: PositionSource!
  timestamp: DateTime!
  address: Address
}

type Address {
  street: String
  city: String
  state: String
  postalCode: String
  country: String
  formatted: String!
}

type EnergyStatus {
  type: EnergyType!
  level: Float!
  range: Float!

  """배터리 전용 필드"""
  stateOfHealth: Float
  voltage: Float
  current: Float
  power: Float
  temperature: BatteryTemperature

  """연료 전용 필드"""
  tankCapacity: Float
  consumption: Float
}

type ChargingStatus {
  isCharging: Boolean!
  isPluggedIn: Boolean!
  chargeType: ChargeType
  stateOfCharge: Float!
  chargeLimit: Float!
  chargePower: Float
  energyAdded: Float
  timeToFull: Int
  estimatedCompletion: DateTime
  schedule: ChargingSchedule
}

type TelemetryData {
  vehicleId: ID!
  timestamp: DateTime!
  signals: [SignalValue!]!
}

type SignalValue {
  path: String!
  value: JSON!
  unit: String
  quality: SignalQuality!
  timestamp: DateTime!
}

type Trip {
  id: ID!
  vehicleId: ID!
  startTime: DateTime!
  endTime: DateTime
  duration: Int!
  distance: Float!
  startLocation: VehicleLocation!
  endLocation: VehicleLocation
  route: [VehicleLocation!]
  averageSpeed: Float!
  maxSpeed: Float!
  energyConsumed: Float
  efficiency: Float
  drivingScore: Float
  events: [TripEvent!]!
}

type CommandResult {
  commandId: ID!
  command: String!
  status: CommandStatus!
  message: String
  progress: Float
  startedAt: DateTime
  completedAt: DateTime
  error: CommandError
}

# 열거형
enum PowertrainType {
  ICE
  HYBRID
  PHEV
  BEV
  FCEV
}

enum EnergyType {
  BATTERY
  FUEL
  HYDROGEN
}

enum ChargeType {
  AC_LEVEL_1
  AC_LEVEL_2
  DC_FAST
  WIRELESS
}

enum ConnectorType {
  J1772
  CCS1
  CCS2
  CHADEMO
  TESLA
  TYPE2
  GB_T
}

enum CommandType {
  LOCK
  UNLOCK
  START_ENGINE
  STOP_ENGINE
  CLIMATE_ON
  CLIMATE_OFF
  SET_CLIMATE
  FLASH_LIGHTS
  HONK_HORN
  OPEN_TRUNK
  CLOSE_TRUNK
  START_CHARGING
  STOP_CHARGING
  SET_CHARGE_LIMIT
}

enum CommandStatus {
  PENDING
  QUEUED
  SENT
  IN_PROGRESS
  COMPLETED
  FAILED
  TIMEOUT
  CANCELLED
}

enum EventType {
  SECURITY_ALERT
  CRASH_DETECTED
  THEFT_ALERT
  GEOFENCE_ENTER
  GEOFENCE_EXIT
  LOW_BATTERY
  LOW_FUEL
  MAINTENANCE_DUE
  DTC_ACTIVE
  CHARGING_STARTED
  CHARGING_COMPLETE
  CHARGING_INTERRUPTED
  RECALL_NOTICE
}

# 스칼라
scalar DateTime
scalar Time
scalar JSON
`;

/**
 * 구독을 지원하는 GraphQL 클라이언트
 */
class ConnectedCarGraphQLClient {
  private endpoint: string;
  private wsEndpoint: string;
  private accessToken: string;
  private subscriptionClient: any;

  constructor(config: GraphQLClientConfig) {
    this.endpoint = config.endpoint || "https://api.wia-connectedcar.org/graphql";
    this.wsEndpoint = config.wsEndpoint || "wss://api.wia-connectedcar.org/graphql";
  }

  /**
   * GraphQL 쿼리 실행
   */
  async query<T>(
    query: string,
    variables?: Record<string, any>
  ): Promise<T> {
    const response = await fetch(this.endpoint, {
      method: "POST",
      headers: {
        "Authorization": `Bearer ${this.accessToken}`,
        "Content-Type": "application/json"
      },
      body: JSON.stringify({ query, variables })
    });

    const result = await response.json();

    if (result.errors) {
      throw new GraphQLError(result.errors);
    }

    return result.data;
  }

  /**
   * GraphQL 뮤테이션 실행
   */
  async mutate<T>(
    mutation: string,
    variables?: Record<string, any>
  ): Promise<T> {
    return this.query<T>(mutation, variables);
  }

  /**
   * 유연한 필드 선택으로 차량 조회
   */
  async getVehicle(vehicleId: string, fields: string[]): Promise<any> {
    const query = `
      query GetVehicle($id: ID!) {
        vehicle(id: $id) {
          ${fields.join("\n")}
        }
      }
    `;

    return this.query<{ vehicle: any }>(query, { id: vehicleId });
  }

  /**
   * 특정 신호로 텔레매틱스 조회
   */
  async getTelemetry(
    vehicleId: string,
    signals: string[],
    timeRange?: { from: string; to: string }
  ): Promise<any> {
    const query = `
      query GetTelemetry($vehicleId: ID!, $signals: [String!], $from: DateTime, $to: DateTime) {
        telemetry(vehicleId: $vehicleId, signals: $signals, from: $from, to: $to) {
          vehicleId
          timestamp
          signals {
            path
            value
            unit
            quality
            timestamp
          }
        }
      }
    `;

    return this.query<{ telemetry: any }>(query, {
      vehicleId,
      signals,
      from: timeRange?.from,
      to: timeRange?.to
    });
  }

  /**
   * 차량 명령 전송
   */
  async sendCommand(vehicleId: string, command: any): Promise<any> {
    const mutation = `
      mutation SendCommand($vehicleId: ID!, $command: CommandInput!) {
        sendCommand(vehicleId: $vehicleId, command: $command) {
          commandId
          command
          status
          message
        }
      }
    `;

    return this.mutate<{ sendCommand: any }>(mutation, { vehicleId, command });
  }

  /**
   * 차량 위치 업데이트 구독
   */
  subscribeToLocation(
    vehicleId: string,
    callback: (location: any) => void
  ): () => void {
    const subscription = `
      subscription LocationUpdated($vehicleId: ID!) {
        locationUpdated(vehicleId: $vehicleId) {
          latitude
          longitude
          altitude
          heading
          speed
          timestamp
        }
      }
    `;

    return this.createSubscription(subscription, { vehicleId }, callback);
  }

  /**
   * 텔레매틱스 스트림 구독
   */
  subscribeToTelemetry(
    vehicleId: string,
    signals: string[],
    interval: number,
    callback: (update: any) => void
  ): () => void {
    const subscription = `
      subscription TelemetryStream($vehicleId: ID!, $signals: [String!]!, $interval: Int) {
        telemetryStream(vehicleId: $vehicleId, signals: $signals, interval: $interval) {
          vehicleId
          timestamp
          signal {
            path
            value
            unit
            quality
            timestamp
          }
        }
      }
    `;

    return this.createSubscription(
      subscription,
      { vehicleId, signals, interval },
      callback
    );
  }

  private createSubscription(
    subscription: string,
    variables: Record<string, any>,
    callback: (data: any) => void
  ): () => void {
    console.log("구독 생성:", { subscription, variables });
    return () => console.log("구독 해제됨");
  }
}

interface GraphQLClientConfig {
  endpoint?: string;
  wsEndpoint?: string;
}

class GraphQLError extends Error {
  constructor(public errors: any[]) {
    super(errors.map(e => e.message).join(", "));
    this.name = "GraphQLError";
  }
}
```

---

## WebSocket 스트리밍 API

```typescript
/**
 * 차량 데이터 스트리밍을 위한 실시간 WebSocket API
 */
interface WebSocketAPIConfig {
  url: string;
  protocols: string[];
  heartbeatInterval: number;
  reconnectPolicy: ReconnectPolicy;
}

interface ReconnectPolicy {
  maxRetries: number;
  initialDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}

/**
 * WebSocket 메시지 유형
 */
enum WSMessageType {
  // 클라이언트 -> 서버
  SUBSCRIBE = "SUBSCRIBE",
  UNSUBSCRIBE = "UNSUBSCRIBE",
  SEND_COMMAND = "SEND_COMMAND",
  PING = "PING",

  // 서버 -> 클라이언트
  SUBSCRIBED = "SUBSCRIBED",
  UNSUBSCRIBED = "UNSUBSCRIBED",
  DATA = "DATA",
  COMMAND_RESULT = "COMMAND_RESULT",
  ERROR = "ERROR",
  PONG = "PONG"
}

interface WSMessage {
  type: WSMessageType;
  id: string;
  payload: any;
  timestamp: string;
}

/**
 * 커넥티드카 API용 WebSocket 클라이언트
 */
class ConnectedCarWebSocketClient {
  private ws: WebSocket | null = null;
  private subscriptions: Map<string, SubscriptionHandler> = new Map();
  private pendingMessages: Map<string, PendingMessage> = new Map();
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private reconnectAttempt = 0;

  constructor(
    private config: WebSocketClientConfig,
    private authToken: string
  ) {}

  /**
   * WebSocket 서버에 연결
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const url = new URL(this.config.url);
      url.searchParams.set("token", this.authToken);

      this.ws = new WebSocket(url.toString(), ["wia-v1"]);

      this.ws.onopen = () => {
        console.log("WebSocket 연결됨");
        this.reconnectAttempt = 0;
        this.startHeartbeat();
        resolve();
      };

      this.ws.onclose = (event) => {
        console.log("WebSocket 닫힘:", event.code, event.reason);
        this.stopHeartbeat();
        this.attemptReconnect();
      };

      this.ws.onerror = (error) => {
        console.error("WebSocket 오류:", error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(JSON.parse(event.data));
      };
    });
  }

  /**
   * 차량 데이터 스트림 구독
   */
  async subscribe(
    vehicleId: string,
    type: SubscriptionType,
    options?: SubscriptionOptions,
    handler?: (data: any) => void
  ): Promise<string> {
    const subscriptionId = crypto.randomUUID();

    const message: WSMessage = {
      type: WSMessageType.SUBSCRIBE,
      id: subscriptionId,
      payload: {
        subscriptionType: type,
        vehicleId,
        options
      },
      timestamp: new Date().toISOString()
    };

    return new Promise((resolve, reject) => {
      this.pendingMessages.set(subscriptionId, {
        resolve: () => {
          if (handler) {
            this.subscriptions.set(subscriptionId, {
              vehicleId,
              type,
              handler
            });
          }
          resolve(subscriptionId);
        },
        reject,
        timeout: setTimeout(() => {
          this.pendingMessages.delete(subscriptionId);
          reject(new Error("구독 타임아웃"));
        }, 10000)
      });

      this.send(message);
    });
  }

  /**
   * 데이터 스트림 구독 해제
   */
  async unsubscribe(subscriptionId: string): Promise<void> {
    const message: WSMessage = {
      type: WSMessageType.UNSUBSCRIBE,
      id: subscriptionId,
      payload: { subscriptionId },
      timestamp: new Date().toISOString()
    };

    return new Promise((resolve, reject) => {
      this.pendingMessages.set(`unsub-${subscriptionId}`, {
        resolve: () => {
          this.subscriptions.delete(subscriptionId);
          resolve();
        },
        reject,
        timeout: setTimeout(() => reject(new Error("구독 해제 타임아웃")), 10000)
      });

      this.send(message);
    });
  }

  /**
   * WebSocket을 통해 차량에 명령 전송
   */
  async sendCommand(
    vehicleId: string,
    command: string,
    parameters?: Record<string, any>
  ): Promise<any> {
    const commandId = crypto.randomUUID();

    const message: WSMessage = {
      type: WSMessageType.SEND_COMMAND,
      id: commandId,
      payload: {
        vehicleId,
        command,
        parameters
      },
      timestamp: new Date().toISOString()
    };

    return new Promise((resolve, reject) => {
      this.pendingMessages.set(commandId, {
        resolve,
        reject,
        timeout: setTimeout(() => {
          this.pendingMessages.delete(commandId);
          reject(new Error("명령 타임아웃"));
        }, 30000)
      });

      this.send(message);
    });
  }

  /**
   * 수신 WebSocket 메시지 처리
   */
  private handleMessage(message: WSMessage): void {
    switch (message.type) {
      case WSMessageType.SUBSCRIBED:
        this.resolvePending(message.id);
        break;

      case WSMessageType.UNSUBSCRIBED:
        this.resolvePending(`unsub-${message.payload.subscriptionId}`);
        break;

      case WSMessageType.DATA:
        const subscription = this.subscriptions.get(message.payload.subscriptionId);
        if (subscription?.handler) {
          subscription.handler(message.payload.data);
        }
        break;

      case WSMessageType.COMMAND_RESULT:
        this.resolvePending(message.id, message.payload);
        break;

      case WSMessageType.ERROR:
        this.rejectPending(message.id, new Error(message.payload.message));
        break;

      case WSMessageType.PONG:
        // 하트비트 확인
        break;
    }
  }

  private send(message: WSMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      throw new Error("WebSocket이 연결되지 않음");
    }
  }

  private resolvePending(id: string, data?: any): void {
    const pending = this.pendingMessages.get(id);
    if (pending) {
      clearTimeout(pending.timeout);
      this.pendingMessages.delete(id);
      pending.resolve(data);
    }
  }

  private rejectPending(id: string, error: Error): void {
    const pending = this.pendingMessages.get(id);
    if (pending) {
      clearTimeout(pending.timeout);
      this.pendingMessages.delete(id);
      pending.reject(error);
    }
  }

  private startHeartbeat(): void {
    this.heartbeatTimer = setInterval(() => {
      this.send({
        type: WSMessageType.PING,
        id: crypto.randomUUID(),
        payload: {},
        timestamp: new Date().toISOString()
      });
    }, this.config.heartbeatInterval || 30000);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  private async attemptReconnect(): Promise<void> {
    const policy = this.config.reconnectPolicy || {
      maxRetries: 5,
      initialDelay: 1000,
      maxDelay: 30000,
      backoffMultiplier: 2
    };

    if (this.reconnectAttempt >= policy.maxRetries) {
      console.error("최대 재연결 시도 횟수 도달");
      return;
    }

    const delay = Math.min(
      policy.initialDelay * Math.pow(policy.backoffMultiplier, this.reconnectAttempt),
      policy.maxDelay
    );

    this.reconnectAttempt++;

    console.log(`${delay}ms 후 재연결 (시도 ${this.reconnectAttempt})`);

    await new Promise(resolve => setTimeout(resolve, delay));

    try {
      await this.connect();
      // 모든 활성 구독 재구독
      await this.resubscribeAll();
    } catch (error) {
      console.error("재연결 실패:", error);
      this.attemptReconnect();
    }
  }

  private async resubscribeAll(): Promise<void> {
    for (const [id, sub] of this.subscriptions) {
      try {
        await this.subscribe(sub.vehicleId, sub.type, undefined, sub.handler);
      } catch (error) {
        console.error(`${id} 재구독 실패:`, error);
      }
    }
  }

  /**
   * WebSocket 서버에서 연결 해제
   */
  disconnect(): void {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close(1000, "클라이언트 연결 해제");
      this.ws = null;
    }
    this.subscriptions.clear();
    this.pendingMessages.clear();
  }
}

type SubscriptionType =
  | "LOCATION"
  | "TELEMETRY"
  | "EVENTS"
  | "CHARGING"
  | "COMMAND_STATUS";

interface SubscriptionOptions {
  signals?: string[];
  interval?: number;
  eventTypes?: string[];
}

interface WebSocketClientConfig {
  url: string;
  heartbeatInterval?: number;
  reconnectPolicy?: ReconnectPolicy;
}

interface SubscriptionHandler {
  vehicleId: string;
  type: SubscriptionType;
  handler: (data: any) => void;
}

interface PendingMessage {
  resolve: (data?: any) => void;
  reject: (error: Error) => void;
  timeout: NodeJS.Timeout;
}
```

---

## 요약

| API 유형 | 사용 사례 | 프로토콜 | 실시간 | 유연성 |
|---------|----------|---------|-------|-------|
| **REST** | 표준 CRUD 작업 | HTTPS | 아니오 | 중간 |
| **GraphQL** | 유연한 데이터 쿼리 | HTTPS/WSS | 예 (구독) | 높음 |
| **WebSocket** | 라이브 스트리밍 데이터 | WSS | 예 | 중간 |
| **gRPC** | 고성능 백엔드 | HTTP/2 | 예 (스트리밍) | 낮음 |

---

**다음 장:** [제5장: 제어 프로토콜](./05-control-protocols.md) - V2X 통신 및 차량 제어 프로토콜.

---

© 2025 World Industry Association (WIA). All rights reserved.
