# Chapter 4: Cleaning Robot API Interface

## Robot Control and Management APIs

### 4.1 API Architecture Overview

The WIA-CLEANING-ROBOT standard defines comprehensive APIs for robot control, monitoring, and fleet management. These APIs enable seamless integration with smart home platforms, facility management systems, and third-party applications.

```typescript
// API Architecture Definition
interface CleaningRobotAPIArchitecture {
  version: '1.0.0';

  apiLayers: {
    localAPI: {
      description: 'Direct robot communication via local network';
      protocols: ['REST over HTTP', 'WebSocket', 'mDNS discovery'];
      authentication: 'Local token authentication';
      latency: '<50ms';
      useCase: 'Real-time control, low-latency operations';
    };
    cloudAPI: {
      description: 'Cloud-based management and remote access';
      protocols: ['REST HTTPS', 'WebSocket WSS', 'MQTT'];
      authentication: 'OAuth 2.0 / API Keys';
      latency: '100-500ms';
      useCase: 'Remote monitoring, fleet management, analytics';
    };
    streamingAPI: {
      description: 'Real-time data streams';
      protocols: ['WebSocket', 'Server-Sent Events', 'MQTT'];
      dataTypes: ['Telemetry', 'Events', 'Map updates'];
      useCase: 'Live monitoring, real-time dashboards';
    };
  };

  baseURLs: {
    local: 'http://{robot-ip}:8080/api/v1';
    cloud: 'https://api.wia-robot.io/v1';
    websocket: 'wss://stream.wia-robot.io/v1';
  };
}

// API Client Configuration
interface APIClientConfig {
  mode: 'local' | 'cloud' | 'hybrid';
  baseURL: string;
  timeout: number;

  authentication: {
    type: 'bearer' | 'api_key' | 'oauth2';
    credentials: AuthCredentials;
  };

  retryPolicy: {
    maxRetries: number;
    backoffMultiplier: number;
    maxBackoff: number;
  };

  rateLimiting: {
    maxRequestsPerSecond: number;
    burstLimit: number;
  };
}

// API Response Wrapper
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

### 4.2 Robot Control API

```typescript
// Robot Control API Service
class RobotControlAPI {
  private httpClient: HttpClient;
  private wsClient: WebSocketClient;

  constructor(config: APIClientConfig) {
    this.httpClient = new HttpClient(config);
    this.wsClient = new WebSocketClient(config);
  }

  // ====================
  // Robot State
  // ====================

  /**
   * Get current robot state
   * GET /robots/{robotId}/state
   */
  async getRobotState(robotId: string): Promise<APIResponse<RobotState>> {
    return this.httpClient.get<RobotState>(`/robots/${robotId}/state`);
  }

  /**
   * Get robot information
   * GET /robots/{robotId}
   */
  async getRobotInfo(robotId: string): Promise<APIResponse<RobotInfo>> {
    return this.httpClient.get<RobotInfo>(`/robots/${robotId}`);
  }

  /**
   * Get robot capabilities
   * GET /robots/{robotId}/capabilities
   */
  async getCapabilities(robotId: string): Promise<APIResponse<RobotCapabilities>> {
    return this.httpClient.get<RobotCapabilities>(`/robots/${robotId}/capabilities`);
  }

  // ====================
  // Cleaning Control
  // ====================

  /**
   * Start cleaning
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
   * Stop cleaning
   * POST /robots/{robotId}/clean/stop
   */
  async stopCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/stop`);
  }

  /**
   * Pause cleaning
   * POST /robots/{robotId}/clean/pause
   */
  async pauseCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/pause`);
  }

  /**
   * Resume cleaning
   * POST /robots/{robotId}/clean/resume
   */
  async resumeCleaning(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/clean/resume`);
  }

  /**
   * Return to dock
   * POST /robots/{robotId}/dock
   */
  async returnToDock(robotId: string): Promise<APIResponse<void>> {
    return this.httpClient.post<void>(`/robots/${robotId}/dock`);
  }

  /**
   * Start spot cleaning
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
  // Room/Zone Cleaning
  // ====================

  /**
   * Clean specific rooms
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
   * Clean specific zones
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
  // Settings Control
  // ====================

  /**
   * Set cleaning settings
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
   * Set suction power
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
   * Set mop wetness
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

// Request/Response Types
interface StartCleaningRequest {
  type: CleaningTaskType;
  settings?: Partial<CleaningSettings>;
  target?: CleaningTarget;
  constraints?: Partial<TaskConstraints>;
}

interface SpotCleaningRequest {
  center?: Position2D;            // Current position if not specified
  radius?: number;                // Default 1.5m
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

interface CustomZone {
  name?: string;
  polygon: Position2D[];
  settings?: Partial<CleaningSettings>;
}
```

### 4.3 Map Management API

```typescript
// Map Management API
class MapManagementAPI {
  private httpClient: HttpClient;

  // ====================
  // Map CRUD Operations
  // ====================

  /**
   * Get all maps for a robot
   * GET /robots/{robotId}/maps
   */
  async getMaps(robotId: string): Promise<APIResponse<MapSummary[]>> {
    return this.httpClient.get<MapSummary[]>(`/robots/${robotId}/maps`);
  }

  /**
   * Get specific map details
   * GET /robots/{robotId}/maps/{mapId}
   */
  async getMap(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<RobotMap>> {
    return this.httpClient.get<RobotMap>(`/robots/${robotId}/maps/${mapId}`);
  }

  /**
   * Get map image
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
   * Create new map (start mapping)
   * POST /robots/{robotId}/maps
   */
  async createMap(
    robotId: string,
    request: CreateMapRequest
  ): Promise<APIResponse<MapSummary>> {
    return this.httpClient.post<MapSummary>(`/robots/${robotId}/maps`, request);
  }

  /**
   * Update map metadata
   * PATCH /robots/{robotId}/maps/{mapId}
   */
  async updateMap(
    robotId: string,
    mapId: string,
    update: MapUpdateRequest
  ): Promise<APIResponse<MapSummary>> {
    return this.httpClient.patch<MapSummary>(
      `/robots/${robotId}/maps/${mapId}`,
      update
    );
  }

  /**
   * Delete map
   * DELETE /robots/{robotId}/maps/{mapId}
   */
  async deleteMap(
    robotId: string,
    mapId: string
  ): Promise<APIResponse<void>> {
    return this.httpClient.delete<void>(`/robots/${robotId}/maps/${mapId}`);
  }

  // ====================
  // Room Management
  // ====================

  /**
   * Get rooms in map
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
   * Update room
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
   * Merge rooms
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

  /**
   * Split room
   * POST /robots/{robotId}/maps/{mapId}/rooms/{roomId}/split
   */
  async splitRoom(
    robotId: string,
    mapId: string,
    roomId: string,
    splitLine: LineString
  ): Promise<APIResponse<Room[]>> {
    return this.httpClient.post<Room[]>(
      `/robots/${robotId}/maps/${mapId}/rooms/${roomId}/split`,
      { splitLine }
    );
  }

  // ====================
  // Zone Management
  // ====================

  /**
   * Get zones in map
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
   * Create zone
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
   * Update zone
   * PATCH /robots/{robotId}/maps/{mapId}/zones/{zoneId}
   */
  async updateZone(
    robotId: string,
    mapId: string,
    zoneId: string,
    update: ZoneUpdateRequest
  ): Promise<APIResponse<Zone>> {
    return this.httpClient.patch<Zone>(
      `/robots/${robotId}/maps/${mapId}/zones/${zoneId}`,
      update
    );
  }

  /**
   * Delete zone
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
  // Boundary Management
  // ====================

  /**
   * Get virtual boundaries
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
   * Create virtual boundary
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

  /**
   * Update boundary
   * PATCH /robots/{robotId}/maps/{mapId}/boundaries/{boundaryId}
   */
  async updateBoundary(
    robotId: string,
    mapId: string,
    boundaryId: string,
    update: BoundaryUpdateRequest
  ): Promise<APIResponse<VirtualBoundary>> {
    return this.httpClient.patch<VirtualBoundary>(
      `/robots/${robotId}/maps/${mapId}/boundaries/${boundaryId}`,
      update
    );
  }

  /**
   * Delete boundary
   * DELETE /robots/{robotId}/maps/{mapId}/boundaries/{boundaryId}
   */
  async deleteBoundary(
    robotId: string,
    mapId: string,
    boundaryId: string
  ): Promise<APIResponse<void>> {
    return this.httpClient.delete<void>(
      `/robots/${robotId}/maps/${mapId}/boundaries/${boundaryId}`
    );
  }
}

// Map API Types
interface MapImageOptions {
  width?: number;
  height?: number;
  format?: 'png' | 'jpeg' | 'svg';
  layers?: ('grid' | 'rooms' | 'zones' | 'boundaries' | 'path' | 'robot')[];
  showLabels?: boolean;
}

interface MapImage {
  data: string;                   // Base64 encoded
  format: string;
  width: number;
  height: number;
  bounds: MapBounds;
}

interface CreateMapRequest {
  name?: string;
  floor?: number;
  mode?: 'FULL_EXPLORATION' | 'QUICK_MAP' | 'GUIDED_MAP';
}

interface MapUpdateRequest {
  name?: string;
  floor?: number;
  rotation?: number;
}

interface RoomUpdateRequest {
  name?: string;
  type?: RoomType;
  cleaningSettings?: Partial<RoomCleaningSettings>;
}

interface CreateZoneRequest {
  name: string;
  type: ZoneType;
  polygon: Position2D[];
  settings?: ZoneSettings;
}

interface ZoneUpdateRequest {
  name?: string;
  type?: ZoneType;
  polygon?: Position2D[];
  settings?: Partial<ZoneSettings>;
  active?: boolean;
}

interface CreateBoundaryRequest {
  name?: string;
  type: BoundaryType;
  geometry: LineString | Polygon;
  active?: boolean;
}

interface BoundaryUpdateRequest {
  name?: string;
  type?: BoundaryType;
  geometry?: LineString | Polygon;
  active?: boolean;
}
```

### 4.4 Schedule Management API

```typescript
// Schedule Management API
class ScheduleManagementAPI {
  private httpClient: HttpClient;

  /**
   * Get all schedules
   * GET /robots/{robotId}/schedules
   */
  async getSchedules(robotId: string): Promise<APIResponse<CleaningSchedule[]>> {
    return this.httpClient.get<CleaningSchedule[]>(
      `/robots/${robotId}/schedules`
    );
  }

  /**
   * Get specific schedule
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
   * Create schedule
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
   * Update schedule
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
   * Delete schedule
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
   * Enable/disable schedule
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

  /**
   * Get upcoming scheduled cleanings
   * GET /robots/{robotId}/schedules/upcoming
   */
  async getUpcomingCleanings(
    robotId: string,
    days?: number
  ): Promise<APIResponse<ScheduledExecution[]>> {
    const params = days ? `?days=${days}` : '';
    return this.httpClient.get<ScheduledExecution[]>(
      `/robots/${robotId}/schedules/upcoming${params}`
    );
  }
}

// Schedule Types
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

  exceptions?: Date[];           // Skip dates
}

interface ScheduledTask {
  type: CleaningTaskType;
  target: CleaningTarget;
  settings: CleaningSettings;
}

interface CreateScheduleRequest {
  name: string;
  type: ScheduleType;
  recurrence: ScheduleRecurrence;
  task: ScheduledTask;
  enabled?: boolean;
}

interface ScheduleUpdateRequest {
  name?: string;
  recurrence?: Partial<ScheduleRecurrence>;
  task?: Partial<ScheduledTask>;
  enabled?: boolean;
}

interface ScheduledExecution {
  scheduleId: string;
  scheduleName: string;
  executionTime: Date;
  task: ScheduledTask;
}
```

### 4.5 Cleaning History API

```typescript
// Cleaning History API
class CleaningHistoryAPI {
  private httpClient: HttpClient;

  /**
   * Get cleaning history
   * GET /robots/{robotId}/history
   */
  async getCleaningHistory(
    robotId: string,
    options?: HistoryQueryOptions
  ): Promise<APIResponse<PaginatedResponse<CleaningSession>>> {
    const params = this.buildQueryParams(options);
    return this.httpClient.get<PaginatedResponse<CleaningSession>>(
      `/robots/${robotId}/history?${params}`
    );
  }

  /**
   * Get specific cleaning session
   * GET /robots/{robotId}/history/{sessionId}
   */
  async getCleaningSession(
    robotId: string,
    sessionId: string
  ): Promise<APIResponse<CleaningSessionDetail>> {
    return this.httpClient.get<CleaningSessionDetail>(
      `/robots/${robotId}/history/${sessionId}`
    );
  }

  /**
   * Get cleaning session path
   * GET /robots/{robotId}/history/{sessionId}/path
   */
  async getCleaningPath(
    robotId: string,
    sessionId: string
  ): Promise<APIResponse<CleaningPath>> {
    return this.httpClient.get<CleaningPath>(
      `/robots/${robotId}/history/${sessionId}/path`
    );
  }

  /**
   * Get cleaning session map snapshot
   * GET /robots/{robotId}/history/{sessionId}/map
   */
  async getSessionMapSnapshot(
    robotId: string,
    sessionId: string
  ): Promise<APIResponse<MapSnapshot>> {
    return this.httpClient.get<MapSnapshot>(
      `/robots/${robotId}/history/${sessionId}/map`
    );
  }

  /**
   * Get cleaning statistics
   * GET /robots/{robotId}/statistics
   */
  async getCleaningStatistics(
    robotId: string,
    period: StatisticsPeriod
  ): Promise<APIResponse<CleaningStatistics>> {
    return this.httpClient.get<CleaningStatistics>(
      `/robots/${robotId}/statistics?period=${period}`
    );
  }

  /**
   * Get room cleaning statistics
   * GET /robots/{robotId}/statistics/rooms
   */
  async getRoomStatistics(
    robotId: string,
    options?: RoomStatisticsOptions
  ): Promise<APIResponse<RoomStatistics[]>> {
    const params = this.buildQueryParams(options);
    return this.httpClient.get<RoomStatistics[]>(
      `/robots/${robotId}/statistics/rooms?${params}`
    );
  }

  private buildQueryParams(options?: Record<string, any>): URLSearchParams {
    const params = new URLSearchParams();
    if (options) {
      for (const [key, value] of Object.entries(options)) {
        if (value !== undefined && value !== null) {
          if (value instanceof Date) {
            params.set(key, value.toISOString());
          } else {
            params.set(key, String(value));
          }
        }
      }
    }
    return params;
  }
}

// History Types
interface HistoryQueryOptions {
  startDate?: Date;
  endDate?: Date;
  status?: TaskCompletionStatus[];
  type?: CleaningTaskType[];
  mapId?: string;
  roomId?: string;
  limit?: number;
  offset?: number;
  sort?: 'asc' | 'desc';
}

interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

interface CleaningSession {
  id: string;
  robotId: string;
  startedAt: Date;
  completedAt: Date | null;

  status: TaskCompletionStatus;
  type: CleaningTaskType;

  summary: {
    duration: number;             // minutes
    area: number;                 // square meters
    rooms: number;
    coverage: number;             // percent
  };

  mapId: string;
  targetRooms?: string[];
}

interface CleaningSessionDetail extends CleaningSession {
  task: CleaningTask;
  result: TaskResult;
  events: TaskEvent[];
  issues: CleaningIssue[];
}

interface MapSnapshot {
  sessionId: string;
  mapId: string;
  image: string;                  // Base64
  coverageMap: string;            // Base64, heatmap of cleaned areas
  timestamp: Date;
}

type StatisticsPeriod = 'day' | 'week' | 'month' | 'year' | 'all';

interface CleaningStatistics {
  period: StatisticsPeriod;
  startDate: Date;
  endDate: Date;

  totals: {
    sessions: number;
    duration: number;             // minutes
    area: number;                 // square meters
    distance: number;             // meters
  };

  averages: {
    sessionsPerDay: number;
    durationPerSession: number;
    areaPerSession: number;
    coveragePerSession: number;
  };

  byDay: Array<{
    date: Date;
    sessions: number;
    duration: number;
    area: number;
  }>;

  completion: {
    completed: number;
    partial: number;
    cancelled: number;
    failed: number;
  };

  issues: {
    total: number;
    byType: Map<CleaningIssueType, number>;
  };
}

interface RoomStatistics {
  roomId: string;
  roomName: string;

  totalCleanings: number;
  totalDuration: number;
  totalArea: number;

  averageDuration: number;
  averageCoverage: number;
  averageDirtLevel: number;

  lastCleaned: Date | null;
  cleaningFrequency: number;      // times per week
}
```

### 4.6 Real-Time Streaming API

```typescript
// WebSocket Streaming API
class RobotStreamingAPI {
  private wsClient: WebSocketClient;
  private subscriptions: Map<string, Subscription>;

  constructor(config: APIClientConfig) {
    this.wsClient = new WebSocketClient(config);
    this.subscriptions = new Map();
  }

  /**
   * Connect to robot stream
   */
  async connect(robotId: string): Promise<void> {
    await this.wsClient.connect(`/robots/${robotId}/stream`);
  }

  /**
   * Disconnect from robot stream
   */
  async disconnect(): Promise<void> {
    this.subscriptions.clear();
    await this.wsClient.disconnect();
  }

  /**
   * Subscribe to robot state updates
   */
  subscribeToState(
    callback: (state: RobotState) => void,
    interval?: number
  ): Subscription {
    return this.subscribe('state', callback, { interval });
  }

  /**
   * Subscribe to position updates
   */
  subscribeToPosition(
    callback: (pose: RobotPose) => void,
    interval?: number
  ): Subscription {
    return this.subscribe('position', callback, { interval });
  }

  /**
   * Subscribe to cleaning progress
   */
  subscribeToProgress(
    callback: (progress: TaskProgress) => void
  ): Subscription {
    return this.subscribe('progress', callback);
  }

  /**
   * Subscribe to events
   */
  subscribeToEvents(
    callback: (event: RobotEvent) => void,
    eventTypes?: RobotEventType[]
  ): Subscription {
    return this.subscribe('events', callback, { types: eventTypes });
  }

  /**
   * Subscribe to map updates
   */
  subscribeToMapUpdates(
    callback: (update: MapUpdate) => void
  ): Subscription {
    return this.subscribe('map', callback);
  }

  /**
   * Subscribe to telemetry
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

  private unsubscribe(subscriptionId: string): void {
    this.wsClient.send({
      type: 'unsubscribe',
      subscriptionId
    });
    this.subscriptions.delete(subscriptionId);
  }
}

// Streaming Types
interface Subscription {
  id: string;
  channel: string;
  unsubscribe: () => void;
}

interface SubscriptionOptions {
  interval?: number;              // ms
  types?: string[];
  fields?: string[];
}

interface RobotEvent {
  type: RobotEventType;
  timestamp: Date;
  robotId: string;
  data: Record<string, any>;
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

interface MapUpdate {
  type: MapUpdateType;
  mapId: string;
  timestamp: Date;
  data: MapUpdateData;
}

type MapUpdateType =
  | 'GRID_UPDATE'
  | 'ROOM_ADDED'
  | 'ROOM_MODIFIED'
  | 'ZONE_ADDED'
  | 'ZONE_MODIFIED'
  | 'BOUNDARY_ADDED'
  | 'PATH_UPDATE';

interface MapUpdateData {
  region?: MapRegion;
  gridData?: Uint8Array;
  room?: Room;
  zone?: Zone;
  boundary?: VirtualBoundary;
  pathPoints?: PathPoint[];
}
```

### 4.7 GraphQL API

```typescript
// GraphQL Schema Definition
const graphqlSchema = `
  type Query {
    # Robot queries
    robot(id: ID!): Robot
    robots(filter: RobotFilter): [Robot!]!

    # Map queries
    map(robotId: ID!, mapId: ID!): Map
    maps(robotId: ID!): [Map!]!

    # History queries
    cleaningHistory(
      robotId: ID!
      filter: HistoryFilter
      pagination: PaginationInput
    ): CleaningHistoryConnection!

    cleaningSession(robotId: ID!, sessionId: ID!): CleaningSession

    # Statistics queries
    statistics(robotId: ID!, period: StatisticsPeriod!): Statistics!
  }

  type Mutation {
    # Cleaning control
    startCleaning(robotId: ID!, input: StartCleaningInput!): CleaningTask!
    stopCleaning(robotId: ID!): Boolean!
    pauseCleaning(robotId: ID!): Boolean!
    resumeCleaning(robotId: ID!): Boolean!
    returnToDock(robotId: ID!): Boolean!

    # Map management
    createMap(robotId: ID!, input: CreateMapInput!): Map!
    updateMap(robotId: ID!, mapId: ID!, input: UpdateMapInput!): Map!
    deleteMap(robotId: ID!, mapId: ID!): Boolean!

    # Room management
    updateRoom(robotId: ID!, mapId: ID!, roomId: ID!, input: UpdateRoomInput!): Room!
    mergeRooms(robotId: ID!, mapId: ID!, roomIds: [ID!]!, name: String): Room!

    # Zone management
    createZone(robotId: ID!, mapId: ID!, input: CreateZoneInput!): Zone!
    updateZone(robotId: ID!, mapId: ID!, zoneId: ID!, input: UpdateZoneInput!): Zone!
    deleteZone(robotId: ID!, mapId: ID!, zoneId: ID!): Boolean!

    # Schedule management
    createSchedule(robotId: ID!, input: CreateScheduleInput!): Schedule!
    updateSchedule(robotId: ID!, scheduleId: ID!, input: UpdateScheduleInput!): Schedule!
    deleteSchedule(robotId: ID!, scheduleId: ID!): Boolean!
    toggleSchedule(robotId: ID!, scheduleId: ID!, enabled: Boolean!): Schedule!

    # Settings
    updateCleaningSettings(robotId: ID!, input: CleaningSettingsInput!): CleaningSettings!
  }

  type Subscription {
    # Real-time updates
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

  type RobotState {
    operatingState: OperatingState!
    pose: Position
    batteryLevel: Int!
    isCharging: Boolean!

    vacuum: VacuumState!
    mop: MopState!
    dustbinLevel: Int!
    waterTankLevel: Int!

    errors: [Error!]!
    warnings: [Warning!]!
  }

  type Map {
    id: ID!
    name: String!
    floor: Int!

    rooms: [Room!]!
    zones: [Zone!]!
    boundaries: [VirtualBoundary!]!

    totalArea: Float!
    cleanableArea: Float!

    image(width: Int, height: Int, layers: [MapLayer!]): MapImage!

    createdAt: DateTime!
    updatedAt: DateTime!
  }

  type CleaningSession {
    id: ID!
    startedAt: DateTime!
    completedAt: DateTime

    status: CompletionStatus!
    type: CleaningType!

    duration: Int!
    area: Float!
    coverage: Float!

    task: CleaningTask!
    result: TaskResult
    path: CleaningPath
    events: [TaskEvent!]!
  }
`;

// GraphQL Resolvers
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
    },

    statistics: async (
      _: any,
      { robotId, period }: { robotId: string; period: StatisticsPeriod }
    ) => {
      return this.historyService.getStatistics(robotId, period);
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
    },

    createZone: async (
      _: any,
      { robotId, mapId, input }: { robotId: string; mapId: string; input: CreateZoneInput }
    ) => {
      return this.mapService.createZone(robotId, mapId, input);
    }
  };

  Subscription = {
    robotState: {
      subscribe: (_: any, { robotId }: { robotId: string }) => {
        return this.robotService.subscribeToState(robotId);
      }
    },

    cleaningProgress: {
      subscribe: (_: any, { robotId }: { robotId: string }) => {
        return this.cleaningService.subscribeToProgress(robotId);
      }
    }
  };
}
```

---

**WIA-CLEANING-ROBOT API Interface**
**Version**: 1.0.0
**Last Updated**: 2025
**License**: MIT

© 2025 World Interoperability Alliance (WIA)
弘益人間 (홍익인간) - Benefit All Humanity
