# 4장: 청소 로봇 API 인터페이스

## 로봇 제어 및 관리 API

### 4.1 API 아키텍처 개요

WIA-CLEANING-ROBOT 표준은 로봇 제어, 모니터링 및 플릿 관리를 위한 종합 API를 정의합니다. 이러한 API는 스마트 홈 플랫폼, 시설 관리 시스템 및 타사 애플리케이션과의 원활한 통합을 가능하게 합니다.

```typescript
// API 아키텍처 정의
interface CleaningRobotAPIArchitecture {
  version: '1.0.0';

  apiLayers: {
    localAPI: {
      description: '로컬 네트워크를 통한 직접 로봇 통신';
      protocols: ['HTTP를 통한 REST', 'WebSocket', 'mDNS 검색'];
      authentication: '로컬 토큰 인증';
      latency: '<50ms';
      useCase: '실시간 제어, 저지연 작업';
    };
    cloudAPI: {
      description: '클라우드 기반 관리 및 원격 접근';
      protocols: ['REST HTTPS', 'WebSocket WSS', 'MQTT'];
      authentication: 'OAuth 2.0 / API 키';
      latency: '100-500ms';
      useCase: '원격 모니터링, 플릿 관리, 분석';
    };
    streamingAPI: {
      description: '실시간 데이터 스트림';
      protocols: ['WebSocket', 'Server-Sent Events', 'MQTT'];
      dataTypes: ['텔레메트리', '이벤트', '지도 업데이트'];
      useCase: '라이브 모니터링, 실시간 대시보드';
    };
  };

  baseURLs: {
    local: 'http://{robot-ip}:8080/api/v1';
    cloud: 'https://api.wia-robot.io/v1';
    websocket: 'wss://stream.wia-robot.io/v1';
  };
}

// API 응답 래퍼
interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata: ResponseMetadata;
}

interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  retryable: boolean;
  retryAfter?: number;
}

interface ResponseMetadata {
  requestId: string;
  timestamp: Date;
  processingTime: number;
  robotId?: string;
  version: string;
}
```

### 4.2 로봇 제어 API

```typescript
// 로봇 제어 API 서비스
class RobotControlAPI {
  private httpClient: HttpClient;
  private wsClient: WebSocketClient;

  constructor(config: APIClientConfig) {
    this.httpClient = new HttpClient(config);
    this.wsClient = new WebSocketClient(config);
  }

  // ====================
  // 로봇 상태
  // ====================

  /**
   * 현재 로봇 상태 가져오기
   * GET /robots/{robotId}/state
   */
  async getRobotState(robotId: string): Promise<APIResponse<RobotState>> {
    return this.httpClient.get<RobotState>(`/robots/${robotId}/state`);
  }

  /**
   * 로봇 정보 가져오기
   * GET /robots/{robotId}
   */
  async getRobotInfo(robotId: string): Promise<APIResponse<RobotInfo>> {
    return this.httpClient.get<RobotInfo>(`/robots/${robotId}`);
  }

  /**
   * 로봇 기능 가져오기
   * GET /robots/{robotId}/capabilities
   */
  async getCapabilities(robotId: string): Promise<APIResponse<RobotCapabilities>> {
    return this.httpClient.get<RobotCapabilities>(`/robots/${robotId}/capabilities`);
  }

  // ====================
  // 청소 제어
  // ====================

  /**
   * 청소 시작
   * POST /robots/{robotId}/clean/start
   */
  async startCleaning(
    robotId: string,
    request: StartCleaningRequest
  ): Promise<APIResponse<CleaningTask>> {
    return this.httpClient.post<CleaningTask>(
      `/robots/${robotId}/clean/start`,
      request
    );
  }

  /**
   * 청소 중지
   * POST /robots/{robotId}/clean/stop
   */
  async stopCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/stop`);
  }

  /**
   * 청소 일시 중지
   * POST /robots/{robotId}/clean/pause
   */
  async pauseCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/pause`);
  }

  /**
   * 청소 재개
   * POST /robots/{robotId}/clean/resume
   */
  async resumeCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/resume`);
  }

  /**
   * 도크로 복귀
   * POST /robots/{robotId}/dock
   */
  async returnToDock(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/dock`);
  }

  /**
   * 스팟 청소 시작
   * POST /robots/{robotId}/clean/spot
   */
  async startSpotCleaning(
    robotId: string,
    request: SpotCleaningRequest
  ): Promise<APIResponse<CleaningTask>> {
    return this.httpClient.post<CleaningTask>(
      `/robots/${robotId}/clean/spot`,
      request
    );
  }

  // ====================
  // 방/구역 청소
  // ====================

  /**
   * 특정 방 청소
   * POST /robots/{robotId}/clean/rooms
   */
  async cleanRooms(
    robotId: string,
    request: RoomCleaningRequest
  ): Promise<APIResponse<CleaningTask>> {
    return this.httpClient.post<CleaningTask>(
      `/robots/${robotId}/clean/rooms`,
      request
    );
  }

  /**
   * 특정 구역 청소
   * POST /robots/{robotId}/clean/zones
   */
  async cleanZones(
    robotId: string,
    request: ZoneCleaningRequest
  ): Promise<APIResponse<CleaningTask>> {
    return this.httpClient.post<CleaningTask>(
      `/robots/${robotId}/clean/zones`,
      request
    );
  }

  // ====================
  // 설정 제어
  // ====================

  /**
   * 청소 설정 지정
   * PUT /robots/{robotId}/settings/cleaning
   */
  async setCleaningSettings(
    robotId: string,
    settings: CleaningSettings
  ): Promise<APIResponse<CleaningSettings>> {
    return this.httpClient.put<CleaningSettings>(
      `/robots/${robotId}/settings/cleaning`,
      settings
    );
  }

  /**
   * 흡입력 설정
   * PUT /robots/{robotId}/settings/suction
   */
  async setSuctionPower(
    robotId: string,
    level: SuctionLevel
  ): Promise<APIResponse<void>> {
    return this.httpClient.put<void>(
      `/robots/${robotId}/settings/suction`,
      { level }
    );
  }

  /**
   * 걸레 물량 설정
   * PUT /robots/{robotId}/settings/mop
   */
  async setMopWetness(
    robotId: string,
    level: WetnessLevel
  ): Promise<APIResponse<void>> {
    return this.httpClient.put<void>(
      `/robots/${robotId}/settings/mop`,
      { level }
    );
  }
}

// 요청/응답 타입
interface StartCleaningRequest {
  type: CleaningTaskType;
  settings?: Partial<CleaningSettings>;
  target?: CleaningTarget;
  constraints?: Partial<TaskConstraints>;
}

interface SpotCleaningRequest {
  center?: Position2D;            // 지정하지 않으면 현재 위치
  radius?: number;                // 기본값 1.5m
  settings?: Partial<CleaningSettings>;
}

interface RoomCleaningRequest {
  roomIds: string[];
  order?: 'SPECIFIED' | 'OPTIMIZED' | 'NEAREST_FIRST';
  settings?: Partial<CleaningSettings>;
  roomSettings?: Map<string, Partial<CleaningSettings>>;
}

interface ZoneCleaningRequest {
  zoneIds: string[];
  customZones?: CustomZone[];
  settings?: Partial<CleaningSettings>;
}
```

### 4.3 지도 관리 API

```typescript
// 지도 관리 API
class MapManagementAPI {
  private httpClient: HttpClient;

  // ====================
  // 지도 CRUD 작업
  // ====================

  /**
   * 로봇의 모든 지도 가져오기
   * GET /robots/{robotId}/maps
   */
  async getMaps(robotId: string): Promise<APIResponse<MapSummary[]>> {
    return this.httpClient.get<MapSummary[]>(`/robots/${robotId}/maps`);
  }

  /**
   * 특정 지도 상세 정보 가져오기
   * GET /robots/{robotId}/maps/{mapId}
   */
  async getMap(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<RobotMap>> {
    return this.httpClient.get<RobotMap>(`/robots/${robotId}/maps/${mapId}`);
  }

  /**
   * 지도 이미지 가져오기
   * GET /robots/{robotId}/maps/{mapId}/image
   */
  async getMapImage(
    robotId: string,
    mapId: string,
    options?: MapImageOptions
  ): Promise<APIResponse<MapImage>> {
    const params = new URLSearchParams();
    if (options?.width) params.set('width', options.width.toString());
    if (options?.height) params.set('height', options.height.toString());
    if (options?.format) params.set('format', options.format);
    if (options?.layers) params.set('layers', options.layers.join(','));

    return this.httpClient.get<MapImage>(
      `/robots/${robotId}/maps/${mapId}/image?${params}`
    );
  }

  /**
   * 새 지도 생성 (매핑 시작)
   * POST /robots/{robotId}/maps
   */
  async createMap(
    robotId: string,
    request: CreateMapRequest
  ): Promise<APIResponse<MapSummary>> {
    return this.httpClient.post<MapSummary>(`/robots/${robotId}/maps`, request);
  }

  // ====================
  // 방 관리
  // ====================

  /**
   * 지도의 방 가져오기
   * GET /robots/{robotId}/maps/{mapId}/rooms
   */
  async getRooms(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<Room[]>> {
    return this.httpClient.get<Room[]>(
      `/robots/${robotId}/maps/${mapId}/rooms`
    );
  }

  /**
   * 방 업데이트
   * PATCH /robots/{robotId}/maps/{mapId}/rooms/{roomId}
   */
  async updateRoom(
    robotId: string,
    mapId: string,
    roomId: string,
    update: RoomUpdateRequest
  ): Promise<APIResponse<Room>> {
    return this.httpClient.patch<Room>(
      `/robots/${robotId}/maps/${mapId}/rooms/${roomId}`,
      update
    );
  }

  /**
   * 방 병합
   * POST /robots/{robotId}/maps/{mapId}/rooms/merge
   */
  async mergeRooms(
    robotId: string,
    mapId: string,
    roomIds: string[],
    newName?: string
  ): Promise<APIResponse<Room>> {
    return this.httpClient.post<Room>(
      `/robots/${robotId}/maps/${mapId}/rooms/merge`,
      { roomIds, name: newName }
    );
  }

  // ====================
  // 구역 관리
  // ====================

  /**
   * 지도의 구역 가져오기
   * GET /robots/{robotId}/maps/{mapId}/zones
   */
  async getZones(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<Zone[]>> {
    return this.httpClient.get<Zone[]>(
      `/robots/${robotId}/maps/${mapId}/zones`
    );
  }

  /**
   * 구역 생성
   * POST /robots/{robotId}/maps/{mapId}/zones
   */
  async createZone(
    robotId: string,
    mapId: string,
    zone: CreateZoneRequest
  ): Promise<APIResponse<Zone>> {
    return this.httpClient.post<Zone>(
      `/robots/${robotId}/maps/${mapId}/zones`,
      zone
    );
  }

  /**
   * 구역 삭제
   * DELETE /robots/{robotId}/maps/{mapId}/zones/{zoneId}
   */
  async deleteZone(
    robotId: string,
    mapId: string,
    zoneId: string
  ): Promise<APIResponse<void>> {
    return this.httpClient.delete<void>(
      `/robots/${robotId}/maps/${mapId}/zones/${zoneId}`
    );
  }

  // ====================
  // 경계 관리
  // ====================

  /**
   * 가상 경계 가져오기
   * GET /robots/{robotId}/maps/{mapId}/boundaries
   */
  async getBoundaries(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<VirtualBoundary[]>> {
    return this.httpClient.get<VirtualBoundary[]>(
      `/robots/${robotId}/maps/${mapId}/boundaries`
    );
  }

  /**
   * 가상 경계 생성
   * POST /robots/{robotId}/maps/{mapId}/boundaries
   */
  async createBoundary(
    robotId: string,
    mapId: string,
    boundary: CreateBoundaryRequest
  ): Promise<APIResponse<VirtualBoundary>> {
    return this.httpClient.post<VirtualBoundary>(
      `/robots/${robotId}/maps/${mapId}/boundaries`,
      boundary
    );
  }
}

// 지도 API 타입
interface MapImageOptions {
  width?: number;
  height?: number;
  format?: 'png' | 'jpeg' | 'svg';
  layers?: ('grid' | 'rooms' | 'zones' | 'boundaries' | 'path' | 'robot')[];
  showLabels?: boolean;
}

interface CreateMapRequest {
  name?: string;
  floor?: number;
  mode?: 'FULL_EXPLORATION' | 'QUICK_MAP' | 'GUIDED_MAP';
}

interface CreateZoneRequest {
  name: string;
  type: ZoneType;
  polygon: Position2D[];
  settings?: ZoneSettings;
}
```

### 4.4 일정 관리 API

```typescript
// 일정 관리 API
class ScheduleManagementAPI {
  private httpClient: HttpClient;

  /**
   * 모든 일정 가져오기
   * GET /robots/{robotId}/schedules
   */
  async getSchedules(robotId: string): Promise<APIResponse<CleaningSchedule[]>> {
    return this.httpClient.get<CleaningSchedule[]>(
      `/robots/${robotId}/schedules`
    );
  }

  /**
   * 특정 일정 가져오기
   * GET /robots/{robotId}/schedules/{scheduleId}
   */
  async getSchedule(
    robotId: string,
    scheduleId: string
  ): Promise<APIResponse<CleaningSchedule>> {
    return this.httpClient.get<CleaningSchedule>(
      `/robots/${robotId}/schedules/${scheduleId}`
    );
  }

  /**
   * 일정 생성
   * POST /robots/{robotId}/schedules
   */
  async createSchedule(
    robotId: string,
    schedule: CreateScheduleRequest
  ): Promise<APIResponse<CleaningSchedule>> {
    return this.httpClient.post<CleaningSchedule>(
      `/robots/${robotId}/schedules`,
      schedule
    );
  }

  /**
   * 일정 업데이트
   * PATCH /robots/{robotId}/schedules/{scheduleId}
   */
  async updateSchedule(
    robotId: string,
    scheduleId: string,
    update: ScheduleUpdateRequest
  ): Promise<APIResponse<CleaningSchedule>> {
    return this.httpClient.patch<CleaningSchedule>(
      `/robots/${robotId}/schedules/${scheduleId}`,
      update
    );
  }

  /**
   * 일정 삭제
   * DELETE /robots/{robotId}/schedules/{scheduleId}
   */
  async deleteSchedule(
    robotId: string,
    scheduleId: string
  ): Promise<APIResponse<void>> {
    return this.httpClient.delete<void>(
      `/robots/${robotId}/schedules/${scheduleId}`
    );
  }

  /**
   * 일정 활성화/비활성화
   * POST /robots/{robotId}/schedules/{scheduleId}/toggle
   */
  async toggleSchedule(
    robotId: string,
    scheduleId: string,
    enabled: boolean
  ): Promise<APIResponse<CleaningSchedule>> {
    return this.httpClient.post<CleaningSchedule>(
      `/robots/${robotId}/schedules/${scheduleId}/toggle`,
      { enabled }
    );
  }
}

// 일정 타입
interface CleaningSchedule {
  id: string;
  robotId: string;
  name: string;

  enabled: boolean;
  type: ScheduleType;

  recurrence: ScheduleRecurrence;
  task: ScheduledTask;

  createdAt: Date;
  updatedAt: Date;
  lastExecution: Date | null;
  nextExecution: Date | null;
}

interface ScheduleRecurrence {
  frequency: RecurrenceFrequency;
  daysOfWeek: DayOfWeek[];
  time: TimeOfDay;
  timezone: string;

  startDate?: Date;
  endDate?: Date;

  exceptions?: Date[];           // 건너뛸 날짜
}

interface CreateScheduleRequest {
  name: string;
  type: ScheduleType;
  recurrence: ScheduleRecurrence;
  task: ScheduledTask;
  enabled?: boolean;
}
```

### 4.5 실시간 스트리밍 API

```typescript
// WebSocket 스트리밍 API
class RobotStreamingAPI {
  private wsClient: WebSocketClient;
  private subscriptions: Map<string, Subscription>;

  constructor(config: APIClientConfig) {
    this.wsClient = new WebSocketClient(config);
    this.subscriptions = new Map();
  }

  /**
   * 로봇 스트림에 연결
   */
  async connect(robotId: string): Promise<void> {
    await this.wsClient.connect(`/robots/${robotId}/stream`);
  }

  /**
   * 로봇 스트림에서 연결 해제
   */
  async disconnect(): Promise<void> {
    this.subscriptions.clear();
    await this.wsClient.disconnect();
  }

  /**
   * 로봇 상태 업데이트 구독
   */
  subscribeToState(
    callback: (state: RobotState) => void,
    interval?: number
  ): Subscription {
    return this.subscribe('state', callback, { interval });
  }

  /**
   * 위치 업데이트 구독
   */
  subscribeToPosition(
    callback: (pose: RobotPose) => void,
    interval?: number
  ): Subscription {
    return this.subscribe('position', callback, { interval });
  }

  /**
   * 청소 진행 상황 구독
   */
  subscribeToProgress(
    callback: (progress: TaskProgress) => void
  ): Subscription {
    return this.subscribe('progress', callback);
  }

  /**
   * 이벤트 구독
   */
  subscribeToEvents(
    callback: (event: RobotEvent) => void,
    eventTypes?: RobotEventType[]
  ): Subscription {
    return this.subscribe('events', callback, { types: eventTypes });
  }

  /**
   * 지도 업데이트 구독
   */
  subscribeToMapUpdates(
    callback: (update: MapUpdate) => void
  ): Subscription {
    return this.subscribe('map', callback);
  }

  /**
   * 텔레메트리 구독
   */
  subscribeToTelemetry(
    callback: (telemetry: TelemetrySample) => void,
    fields?: string[],
    interval?: number
  ): Subscription {
    return this.subscribe('telemetry', callback, { fields, interval });
  }

  private subscribe<T>(
    channel: string,
    callback: (data: T) => void,
    options?: SubscriptionOptions
  ): Subscription {
    const subscriptionId = generateId();

    this.wsClient.send({
      type: 'subscribe',
      channel,
      subscriptionId,
      options
    });

    const subscription: Subscription = {
      id: subscriptionId,
      channel,
      unsubscribe: () => this.unsubscribe(subscriptionId)
    };

    this.wsClient.on(`message:${channel}`, (data: T) => {
      callback(data);
    });

    this.subscriptions.set(subscriptionId, subscription);
    return subscription;
  }
}

// 스트리밍 타입
interface Subscription {
  id: string;
  channel: string;
  unsubscribe: () => void;
}

type RobotEventType =
  | 'STATE_CHANGED'
  | 'CLEANING_STARTED'
  | 'CLEANING_COMPLETED'
  | 'CLEANING_PAUSED'
  | 'CLEANING_RESUMED'
  | 'ROOM_COMPLETED'
  | 'ERROR_OCCURRED'
  | 'ERROR_CLEARED'
  | 'BATTERY_LOW'
  | 'DUSTBIN_FULL'
  | 'WATER_EMPTY'
  | 'STUCK'
  | 'STUCK_CLEARED'
  | 'DOCKING'
  | 'DOCKED'
  | 'CHARGING_STARTED'
  | 'CHARGING_COMPLETED'
  | 'FIRMWARE_UPDATE_AVAILABLE'
  | 'MAINTENANCE_REQUIRED';
```

### 4.6 GraphQL API

```typescript
// GraphQL 스키마 정의
const graphqlSchema = `
  type Query {
    # 로봇 쿼리
    robot(id: ID!): Robot
    robots(filter: RobotFilter): [Robot!]!

    # 지도 쿼리
    map(robotId: ID!, mapId: ID!): Map
    maps(robotId: ID!): [Map!]!

    # 히스토리 쿼리
    cleaningHistory(
      robotId: ID!
      filter: HistoryFilter
      pagination: PaginationInput
    ): CleaningHistoryConnection!

    cleaningSession(robotId: ID!, sessionId: ID!): CleaningSession

    # 통계 쿼리
    statistics(robotId: ID!, period: StatisticsPeriod!): Statistics!
  }

  type Mutation {
    # 청소 제어
    startCleaning(robotId: ID!, input: StartCleaningInput!): CleaningTask!
    stopCleaning(robotId: ID!): Boolean!
    pauseCleaning(robotId: ID!): Boolean!
    resumeCleaning(robotId: ID!): Boolean!
    returnToDock(robotId: ID!): Boolean!

    # 지도 관리
    createMap(robotId: ID!, input: CreateMapInput!): Map!
    updateMap(robotId: ID!, mapId: ID!, input: UpdateMapInput!): Map!
    deleteMap(robotId: ID!, mapId: ID!): Boolean!

    # 방 관리
    updateRoom(robotId: ID!, mapId: ID!, roomId: ID!, input: UpdateRoomInput!): Room!
    mergeRooms(robotId: ID!, mapId: ID!, roomIds: [ID!]!, name: String): Room!

    # 구역 관리
    createZone(robotId: ID!, mapId: ID!, input: CreateZoneInput!): Zone!
    updateZone(robotId: ID!, mapId: ID!, zoneId: ID!, input: UpdateZoneInput!): Zone!
    deleteZone(robotId: ID!, mapId: ID!, zoneId: ID!): Boolean!

    # 일정 관리
    createSchedule(robotId: ID!, input: CreateScheduleInput!): Schedule!
    updateSchedule(robotId: ID!, scheduleId: ID!, input: UpdateScheduleInput!): Schedule!
    deleteSchedule(robotId: ID!, scheduleId: ID!): Boolean!
  }

  type Subscription {
    # 실시간 업데이트
    robotState(robotId: ID!): RobotState!
    robotPosition(robotId: ID!): Position!
    cleaningProgress(robotId: ID!): TaskProgress!
    robotEvents(robotId: ID!, types: [EventType!]): RobotEvent!
    mapUpdates(robotId: ID!, mapId: ID!): MapUpdate!
  }

  type Robot {
    id: ID!
    serialNumber: String!
    model: RobotModel!
    nickname: String

    state: RobotState!
    capabilities: RobotCapabilities!
    configuration: RobotConfiguration!

    maps: [Map!]!
    currentMap: Map

    schedules: [Schedule!]!
    currentTask: CleaningTask

    statistics(period: StatisticsPeriod!): Statistics!
    history(filter: HistoryFilter, pagination: PaginationInput): CleaningHistoryConnection!
  }
`;

// GraphQL 리졸버
class GraphQLResolvers {
  constructor(
    private robotService: RobotService,
    private mapService: MapService,
    private cleaningService: CleaningService,
    private historyService: HistoryService
  ) {}

  Query = {
    robot: async (_: any, { id }: { id: string }) => {
      return this.robotService.getRobot(id);
    },

    robots: async (_: any, { filter }: { filter?: RobotFilter }) => {
      return this.robotService.getRobots(filter);
    },

    map: async (_: any, { robotId, mapId }: { robotId: string; mapId: string }) => {
      return this.mapService.getMap(robotId, mapId);
    },

    cleaningHistory: async (
      _: any,
      { robotId, filter, pagination }: {
        robotId: string;
        filter?: HistoryFilter;
        pagination?: PaginationInput;
      }
    ) => {
      return this.historyService.getHistory(robotId, filter, pagination);
    }
  };

  Mutation = {
    startCleaning: async (
      _: any,
      { robotId, input }: { robotId: string; input: StartCleaningInput }
    ) => {
      return this.cleaningService.startCleaning(robotId, input);
    },

    stopCleaning: async (_: any, { robotId }: { robotId: string }) => {
      await this.cleaningService.stopCleaning(robotId);
      return true;
    }
  };
}
```

---

**WIA-CLEANING-ROBOT API 인터페이스**
**버전**: 1.0.0
**최종 업데이트**: 2025
**라이선스**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
