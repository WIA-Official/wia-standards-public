# Chapter 4: Connected Car API Interface

## REST, GraphQL, and Streaming APIs for Vehicle Data Access

This chapter defines the comprehensive API specifications for accessing connected car data, enabling third-party developers and ecosystem partners to build innovative mobility applications.

---

## RESTful API Specification

### Base Configuration

```typescript
// WIA Connected Car API - OpenAPI 3.1 Specification
// Complete REST API for vehicle data and control

/**
 * API Configuration
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
 * OpenAPI Schema Definition
 */
const openAPISchema = {
  openapi: "3.1.0",
  info: {
    title: "WIA Connected Car API",
    version: "1.0.0",
    description: "Comprehensive API for vehicle data access and control",
    contact: {
      name: "WIA Technical Support",
      email: "api-support@wia.org",
      url: "https://developers.wia.org"
    },
    license: {
      name: "WIA API License",
      url: "https://wia.org/api-license"
    }
  },
  servers: [
    {
      url: "https://api.wia-connectedcar.org/v1",
      description: "Production server"
    },
    {
      url: "https://sandbox.wia-connectedcar.org/v1",
      description: "Sandbox for development"
    }
  ],
  paths: {
    "/vehicles": {
      get: {
        operationId: "listVehicles",
        summary: "List all vehicles for authenticated user",
        tags: ["Vehicles"],
        security: [{ oauth2: ["vehicles:read"] }],
        parameters: [
          { name: "page", in: "query", schema: { type: "integer", default: 1 } },
          { name: "limit", in: "query", schema: { type: "integer", default: 20, max: 100 } },
          { name: "status", in: "query", schema: { type: "string", enum: ["active", "inactive", "all"] } }
        ],
        responses: {
          200: {
            description: "List of vehicles",
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
        summary: "Get vehicle details",
        tags: ["Vehicles"],
        security: [{ oauth2: ["vehicles:read"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } }
        ],
        responses: {
          200: {
            description: "Vehicle details",
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
        summary: "Get current vehicle location",
        tags: ["Telematics"],
        security: [{ oauth2: ["location:read"] }],
        parameters: [
          { name: "vehicleId", in: "path", required: true, schema: { type: "string", format: "uuid" } }
        ],
        responses: {
          200: {
            description: "Current location",
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
        summary: "Get vehicle telemetry data",
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
            description: "Telemetry data",
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
        summary: "Send command to vehicle",
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
            description: "Command accepted",
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
              "vehicles:read": "Read vehicle information",
              "location:read": "Read vehicle location",
              "telemetry:read": "Read vehicle telemetry",
              "commands:write": "Send commands to vehicle",
              "diagnostics:read": "Read diagnostic data",
              "charging:write": "Control charging"
            }
          }
        }
      }
    }
  }
};

/**
 * REST API Client Implementation
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
   * OAuth 2.0 Authorization Code Flow
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
   * Refresh access token
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
   * Make authenticated API request
   */
  private async request<T>(
    method: string,
    path: string,
    options: RequestOptions = {}
  ): Promise<T> {
    // Auto-refresh token if expired
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

  // Vehicle endpoints
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

  // Command endpoints
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

  // Charging endpoints
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

  // Location history
  async getLocationHistory(
    vehicleId: string,
    params: LocationHistoryParams
  ): Promise<LocationHistoryResponse> {
    return this.request("GET", `/vehicles/${vehicleId}/location/history`, { query: params });
  }

  // Trips
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

// Type definitions
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

interface CommandStatusResponse extends CommandResponse {
  result?: any;
  completedAt?: string;
  error?: {
    code: string;
    message: string;
  };
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

interface ListVehiclesParams {
  page?: number;
  limit?: number;
  status?: string;
}

interface TelemetryParams {
  signals?: string[];
  from?: string;
  to?: string;
}

interface LocationHistoryParams {
  from: string;
  to: string;
  interval?: number;
}

interface LocationHistoryResponse {
  vehicleId: string;
  locations: VehicleLocation[];
  distance: number;
}

interface ListTripsParams {
  from?: string;
  to?: string;
  page?: number;
  limit?: number;
}

interface TripListResponse {
  trips: TripSummary[];
  pagination: Pagination;
}

interface TripSummary {
  id: string;
  startTime: string;
  endTime: string;
  distance: number;
  duration: number;
}

interface Trip extends TripSummary {
  route: VehicleLocation[];
  energyConsumed: number;
  averageSpeed: number;
  maxSpeed: number;
}
```

---

## GraphQL API

### Schema Definition

```typescript
// GraphQL Schema for Connected Car API
// Flexible querying with real-time subscriptions

const graphqlSchema = `
"""
Root Query Type
"""
type Query {
  """Get current user's vehicles"""
  vehicles(
    page: Int = 1
    limit: Int = 20
    status: VehicleStatusFilter
  ): VehicleConnection!

  """Get specific vehicle by ID"""
  vehicle(id: ID!): Vehicle

  """Get vehicle by VIN"""
  vehicleByVin(vin: String!): Vehicle

  """Get telemetry data with flexible signal selection"""
  telemetry(
    vehicleId: ID!
    signals: [String!]
    from: DateTime
    to: DateTime
    resolution: TimeResolution
  ): TelemetryData

  """Get trip history"""
  trips(
    vehicleId: ID!
    from: DateTime
    to: DateTime
    first: Int
    after: String
  ): TripConnection!

  """Search for charging stations near location"""
  chargingStations(
    latitude: Float!
    longitude: Float!
    radius: Float! = 10
    connectorTypes: [ConnectorType!]
    available: Boolean
  ): [ChargingStation!]!
}

"""
Root Mutation Type
"""
type Mutation {
  """Send command to vehicle"""
  sendCommand(
    vehicleId: ID!
    command: CommandInput!
  ): CommandResult!

  """Update vehicle settings"""
  updateVehicleSettings(
    vehicleId: ID!
    settings: VehicleSettingsInput!
  ): Vehicle!

  """Set charging schedule"""
  setChargingSchedule(
    vehicleId: ID!
    schedule: ChargingScheduleInput!
  ): ChargingSchedule!

  """Create geofence"""
  createGeofence(
    vehicleId: ID!
    geofence: GeofenceInput!
  ): Geofence!

  """Delete geofence"""
  deleteGeofence(
    vehicleId: ID!
    geofenceId: ID!
  ): Boolean!
}

"""
Root Subscription Type
"""
type Subscription {
  """Subscribe to vehicle location updates"""
  locationUpdated(vehicleId: ID!): VehicleLocation!

  """Subscribe to telemetry stream"""
  telemetryStream(
    vehicleId: ID!
    signals: [String!]!
    interval: Int = 1000
  ): TelemetryUpdate!

  """Subscribe to vehicle events"""
  vehicleEvents(
    vehicleId: ID!
    eventTypes: [EventType!]
  ): VehicleEvent!

  """Subscribe to command status updates"""
  commandStatus(commandId: ID!): CommandResult!

  """Subscribe to charging status updates"""
  chargingStatus(vehicleId: ID!): ChargingStatus!
}

"""
Vehicle type with all associated data
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

  """Current vehicle status"""
  status: VehicleStatus!

  """Current location (requires location:read scope)"""
  location: VehicleLocation

  """Latest telemetry snapshot"""
  telemetry(signals: [String!]): TelemetryData

  """Battery/fuel status"""
  energy: EnergyStatus!

  """Charging information (EVs only)"""
  charging: ChargingStatus

  """Active diagnostic trouble codes"""
  diagnostics: [DiagnosticCode!]!

  """Vehicle capabilities"""
  capabilities: [Capability!]!

  """Climate control status"""
  climate: ClimateStatus

  """Configured geofences"""
  geofences: [Geofence!]!

  """Recent trips"""
  recentTrips(limit: Int = 5): [Trip!]!

  """Last communication timestamp"""
  lastSeen: DateTime!

  """Vehicle settings"""
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

  """Battery-specific fields"""
  stateOfHealth: Float
  voltage: Float
  current: Float
  power: Float
  temperature: BatteryTemperature

  """Fuel-specific fields"""
  tankCapacity: Float
  consumption: Float
}

type BatteryTemperature {
  average: Float!
  min: Float!
  max: Float!
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

type ChargingSchedule {
  id: ID!
  enabled: Boolean!
  startTime: Time!
  endTime: Time
  targetSoC: Float!
  daysOfWeek: [DayOfWeek!]!
  preferOffPeak: Boolean!
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

type TelemetryUpdate {
  vehicleId: ID!
  timestamp: DateTime!
  signal: SignalValue!
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

type TripEvent {
  type: TripEventType!
  timestamp: DateTime!
  location: VehicleLocation
  severity: Severity!
  details: JSON
}

type VehicleEvent {
  id: ID!
  vehicleId: ID!
  type: EventType!
  severity: Severity!
  timestamp: DateTime!
  message: String!
  location: VehicleLocation
  data: JSON
  acknowledged: Boolean!
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

type CommandError {
  code: String!
  message: String!
  retryable: Boolean!
}

type Geofence {
  id: ID!
  name: String!
  type: GeofenceType!
  coordinates: [Coordinate!]!
  radius: Float
  enabled: Boolean!
  alerts: GeofenceAlerts!
  createdAt: DateTime!
}

type GeofenceAlerts {
  onEnter: Boolean!
  onExit: Boolean!
  onDwell: Boolean!
  dwellTime: Int
}

type Coordinate {
  latitude: Float!
  longitude: Float!
}

type ClimateStatus {
  isOn: Boolean!
  currentTemperature: Float
  targetTemperature: Float
  fanSpeed: Int
  mode: ClimateMode!
  zones: [ClimateZone!]!
  seatHeating: [SeatHeating!]
  defrost: DefrostStatus
}

type ClimateZone {
  zone: String!
  temperature: Float!
  airflow: AirflowDirection!
}

type SeatHeating {
  seat: String!
  level: Int!
}

type DefrostStatus {
  front: Boolean!
  rear: Boolean!
}

type ChargingStation {
  id: ID!
  name: String!
  operator: String!
  location: Coordinate!
  address: Address!
  distance: Float!
  connectors: [Connector!]!
  amenities: [String!]!
  pricing: Pricing
  availability: StationAvailability!
}

type Connector {
  type: ConnectorType!
  power: Float!
  available: Boolean!
  status: ConnectorStatus!
}

type Pricing {
  currency: String!
  perKwh: Float
  perMinute: Float
  perSession: Float
}

type StationAvailability {
  total: Int!
  available: Int!
  inUse: Int!
  outOfService: Int!
}

type DiagnosticCode {
  code: String!
  description: String!
  severity: Severity!
  system: VehicleSystem!
  active: Boolean!
  firstOccurrence: DateTime!
  occurrenceCount: Int!
}

type Capability {
  name: String!
  available: Boolean!
  requiredScope: String
}

type VehicleSettings {
  units: UnitPreferences!
  notifications: NotificationSettings!
  privacy: PrivacySettings!
}

type UnitPreferences {
  distance: DistanceUnit!
  temperature: TemperatureUnit!
  pressure: PressureUnit!
}

type NotificationSettings {
  chargingComplete: Boolean!
  lowBattery: Boolean!
  geofenceAlerts: Boolean!
  maintenanceDue: Boolean!
  securityAlerts: Boolean!
}

type PrivacySettings {
  shareLocation: Boolean!
  shareDrivingData: Boolean!
  shareUsageAnalytics: Boolean!
}

# Connection types for pagination
type VehicleConnection {
  edges: [VehicleEdge!]!
  pageInfo: PageInfo!
  totalCount: Int!
}

type VehicleEdge {
  node: Vehicle!
  cursor: String!
}

type TripConnection {
  edges: [TripEdge!]!
  pageInfo: PageInfo!
  totalCount: Int!
}

type TripEdge {
  node: Trip!
  cursor: String!
}

type PageInfo {
  hasNextPage: Boolean!
  hasPreviousPage: Boolean!
  startCursor: String
  endCursor: String
}

# Input types
input CommandInput {
  type: CommandType!
  parameters: JSON
  timeout: Int
}

input VehicleSettingsInput {
  nickname: String
  units: UnitPreferencesInput
  notifications: NotificationSettingsInput
  privacy: PrivacySettingsInput
}

input UnitPreferencesInput {
  distance: DistanceUnit
  temperature: TemperatureUnit
  pressure: PressureUnit
}

input NotificationSettingsInput {
  chargingComplete: Boolean
  lowBattery: Boolean
  geofenceAlerts: Boolean
  maintenanceDue: Boolean
  securityAlerts: Boolean
}

input PrivacySettingsInput {
  shareLocation: Boolean
  shareDrivingData: Boolean
  shareUsageAnalytics: Boolean
}

input ChargingScheduleInput {
  enabled: Boolean!
  startTime: Time!
  endTime: Time
  targetSoC: Float!
  daysOfWeek: [DayOfWeek!]!
  preferOffPeak: Boolean
}

input GeofenceInput {
  name: String!
  type: GeofenceType!
  coordinates: [CoordinateInput!]!
  radius: Float
  alerts: GeofenceAlertsInput!
}

input CoordinateInput {
  latitude: Float!
  longitude: Float!
}

input GeofenceAlertsInput {
  onEnter: Boolean!
  onExit: Boolean!
  onDwell: Boolean
  dwellTime: Int
}

# Enums
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

enum PositionSource {
  GPS
  GLONASS
  GALILEO
  BEIDOU
  CELLULAR
  WIFI
  FUSED
}

enum IgnitionStatus {
  OFF
  ACCESSORY
  ON
  START
}

enum Door {
  FRONT_LEFT
  FRONT_RIGHT
  REAR_LEFT
  REAR_RIGHT
}

enum Window {
  FRONT_LEFT
  FRONT_RIGHT
  REAR_LEFT
  REAR_RIGHT
  SUNROOF
}

enum ClimateMode {
  OFF
  COOL
  HEAT
  AUTO
  DEFROST
}

enum AirflowDirection {
  FACE
  FEET
  FACE_AND_FEET
  DEFROST
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

enum Severity {
  INFO
  WARNING
  CRITICAL
  EMERGENCY
}

enum TripEventType {
  HARD_BRAKE
  HARD_ACCELERATION
  SHARP_TURN
  SPEEDING
  PHONE_USAGE
  DROWSY_DRIVING
}

enum GeofenceType {
  CIRCLE
  POLYGON
}

enum SignalQuality {
  GOOD
  UNCERTAIN
  BAD
  NOT_AVAILABLE
}

enum TimeResolution {
  RAW
  SECOND
  MINUTE
  HOUR
  DAY
}

enum VehicleStatusFilter {
  ACTIVE
  INACTIVE
  ALL
}

enum DayOfWeek {
  MONDAY
  TUESDAY
  WEDNESDAY
  THURSDAY
  FRIDAY
  SATURDAY
  SUNDAY
}

enum DistanceUnit {
  KILOMETERS
  MILES
}

enum TemperatureUnit {
  CELSIUS
  FAHRENHEIT
}

enum PressureUnit {
  KPA
  PSI
  BAR
}

enum VehicleSystem {
  ENGINE
  TRANSMISSION
  FUEL
  EMISSION
  ELECTRICAL
  HVAC
  BRAKES
  STEERING
  SUSPENSION
  AIRBAGS
  ADAS
  BATTERY
  CHARGING
}

enum ConnectorStatus {
  AVAILABLE
  IN_USE
  OUT_OF_SERVICE
  RESERVED
}

# Scalars
scalar DateTime
scalar Time
scalar JSON
`;

/**
 * GraphQL Client with Subscriptions
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
   * Execute GraphQL query
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
   * Execute GraphQL mutation
   */
  async mutate<T>(
    mutation: string,
    variables?: Record<string, any>
  ): Promise<T> {
    return this.query<T>(mutation, variables);
  }

  /**
   * Get vehicle with flexible field selection
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
   * Get telemetry with specific signals
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
   * Send vehicle command
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
   * Subscribe to vehicle location updates
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

    // Implementation would use graphql-ws or subscriptions-transport-ws
    return this.createSubscription(subscription, { vehicleId }, callback);
  }

  /**
   * Subscribe to telemetry stream
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
    // Placeholder - actual implementation would use WebSocket client
    console.log("Creating subscription:", { subscription, variables });
    return () => console.log("Unsubscribed");
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

## WebSocket Streaming API

```typescript
/**
 * Real-time WebSocket API for vehicle data streaming
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
 * WebSocket message types
 */
enum WSMessageType {
  // Client -> Server
  SUBSCRIBE = "SUBSCRIBE",
  UNSUBSCRIBE = "UNSUBSCRIBE",
  SEND_COMMAND = "SEND_COMMAND",
  PING = "PING",

  // Server -> Client
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
 * Subscription types
 */
type SubscriptionType =
  | "LOCATION"
  | "TELEMETRY"
  | "EVENTS"
  | "CHARGING"
  | "COMMAND_STATUS";

interface SubscribePayload {
  subscriptionType: SubscriptionType;
  vehicleId: string;
  options?: SubscriptionOptions;
}

interface SubscriptionOptions {
  signals?: string[];
  interval?: number;
  eventTypes?: string[];
}

/**
 * WebSocket Client for Connected Car API
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
   * Connect to WebSocket server
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const url = new URL(this.config.url);
      url.searchParams.set("token", this.authToken);

      this.ws = new WebSocket(url.toString(), ["wia-v1"]);

      this.ws.onopen = () => {
        console.log("WebSocket connected");
        this.reconnectAttempt = 0;
        this.startHeartbeat();
        resolve();
      };

      this.ws.onclose = (event) => {
        console.log("WebSocket closed:", event.code, event.reason);
        this.stopHeartbeat();
        this.attemptReconnect();
      };

      this.ws.onerror = (error) => {
        console.error("WebSocket error:", error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(JSON.parse(event.data));
      };
    });
  }

  /**
   * Subscribe to vehicle data stream
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
          reject(new Error("Subscription timeout"));
        }, 10000)
      });

      this.send(message);
    });
  }

  /**
   * Unsubscribe from data stream
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
        timeout: setTimeout(() => reject(new Error("Unsubscribe timeout")), 10000)
      });

      this.send(message);
    });
  }

  /**
   * Send command to vehicle via WebSocket
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
          reject(new Error("Command timeout"));
        }, 30000)
      });

      this.send(message);
    });
  }

  /**
   * Handle incoming WebSocket message
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
        // Heartbeat acknowledged
        break;
    }
  }

  private send(message: WSMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      throw new Error("WebSocket not connected");
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
      console.error("Max reconnection attempts reached");
      return;
    }

    const delay = Math.min(
      policy.initialDelay * Math.pow(policy.backoffMultiplier, this.reconnectAttempt),
      policy.maxDelay
    );

    this.reconnectAttempt++;

    console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempt})`);

    await new Promise(resolve => setTimeout(resolve, delay));

    try {
      await this.connect();
      // Resubscribe to all active subscriptions
      await this.resubscribeAll();
    } catch (error) {
      console.error("Reconnection failed:", error);
      this.attemptReconnect();
    }
  }

  private async resubscribeAll(): Promise<void> {
    for (const [id, sub] of this.subscriptions) {
      try {
        await this.subscribe(sub.vehicleId, sub.type, undefined, sub.handler);
      } catch (error) {
        console.error(`Failed to resubscribe ${id}:`, error);
      }
    }
  }

  /**
   * Disconnect from WebSocket server
   */
  disconnect(): void {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close(1000, "Client disconnecting");
      this.ws = null;
    }
    this.subscriptions.clear();
    this.pendingMessages.clear();
  }
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

## Summary

| API Type | Use Case | Protocol | Real-time | Flexibility |
|----------|----------|----------|-----------|-------------|
| **REST** | Standard CRUD operations | HTTPS | No | Medium |
| **GraphQL** | Flexible data queries | HTTPS/WSS | Yes (subscriptions) | High |
| **WebSocket** | Live streaming data | WSS | Yes | Medium |
| **gRPC** | High-performance backend | HTTP/2 | Yes (streaming) | Low |

---

**Next Chapter:** [Chapter 5: Control Protocols](./05-control-protocols.md) - V2X communication and vehicle control protocols.

---

© 2025 World Industry Association (WIA). All rights reserved.
