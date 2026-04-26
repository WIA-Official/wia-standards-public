# Chapter 6: Connected Car Integration

## OEM Systems, Third-Party Services, and Ecosystem Integration

This chapter provides comprehensive guidance on integrating connected car platforms with OEM systems, third-party services, and the broader mobility ecosystem.

---

## OEM Integration Architecture

### Multi-OEM Platform Architecture

```typescript
// WIA Connected Car Multi-OEM Integration Framework
// Unified interface for diverse vehicle platforms

/**
 * Multi-OEM Integration Architecture
 */
interface MultiOEMPlatform {
  adapters: OEMAdapter[];
  dataHarmonization: DataHarmonizationLayer;
  commandNormalization: CommandNormalizationLayer;
  authenticationBridge: AuthenticationBridge;
  eventAggregation: EventAggregationLayer;
}

/**
 * OEM Adapter Interface
 * Standardized interface for OEM-specific implementations
 */
interface OEMAdapter {
  oem: OEMIdentifier;
  capabilities: OEMCapabilities;
  authentication: OEMAuthConfig;

  // Connection management
  connect(credentials: OEMCredentials): Promise<ConnectionResult>;
  disconnect(): Promise<void>;
  healthCheck(): Promise<HealthStatus>;

  // Vehicle operations
  listVehicles(): Promise<OEMVehicle[]>;
  getVehicleStatus(vehicleId: string): Promise<VehicleStatus>;
  getVehicleLocation(vehicleId: string): Promise<VehicleLocation>;
  sendCommand(vehicleId: string, command: VehicleCommand): Promise<CommandResult>;

  // Data streaming
  subscribeToUpdates(vehicleId: string, callback: UpdateCallback): Unsubscribe;
}

type OEMIdentifier =
  | "TESLA" | "GM" | "FORD" | "VW" | "BMW"
  | "MERCEDES" | "TOYOTA" | "HONDA" | "HYUNDAI"
  | "RIVIAN" | "LUCID" | "NIO" | "BYD" | "GENERIC";

interface OEMCapabilities {
  supportedCommands: VehicleCommandType[];
  telemetrySignals: string[];
  streamingSupported: boolean;
  locationAccuracy: LocationAccuracyLevel;
  commandLatency: CommandLatencyTier;
  apiVersion: string;
}

type LocationAccuracyLevel = "HIGH" | "MEDIUM" | "LOW" | "CELL_TOWER";
type CommandLatencyTier = "INSTANT" | "FAST" | "NORMAL" | "SLOW";

/**
 * Tesla Adapter Implementation
 */
class TeslaAdapter implements OEMAdapter {
  oem: OEMIdentifier = "TESLA";
  capabilities: OEMCapabilities = {
    supportedCommands: [
      VehicleCommandType.LOCK,
      VehicleCommandType.UNLOCK,
      VehicleCommandType.CLIMATE_ON,
      VehicleCommandType.CLIMATE_OFF,
      VehicleCommandType.SET_TEMPERATURE,
      VehicleCommandType.START_CHARGING,
      VehicleCommandType.STOP_CHARGING,
      VehicleCommandType.SET_CHARGE_LIMIT,
      VehicleCommandType.OPEN_TRUNK,
      VehicleCommandType.OPEN_FRUNK,
      VehicleCommandType.FLASH_LIGHTS,
      VehicleCommandType.HONK_HORN,
      VehicleCommandType.REMOTE_START,
      VehicleCommandType.SUMMON,
      VehicleCommandType.VENT_WINDOWS
    ],
    telemetrySignals: [
      "battery.level", "battery.range", "battery.charging",
      "climate.inside_temp", "climate.outside_temp",
      "location.latitude", "location.longitude",
      "vehicle.odometer", "vehicle.locked"
    ],
    streamingSupported: true,
    locationAccuracy: "HIGH",
    commandLatency: "INSTANT",
    apiVersion: "v3"
  };
  authentication: OEMAuthConfig = {
    type: "OAUTH2",
    authorizationUrl: "https://auth.tesla.com/oauth2/v3/authorize",
    tokenUrl: "https://auth.tesla.com/oauth2/v3/token",
    scopes: ["openid", "vehicle_device_data", "vehicle_cmds", "vehicle_charging_cmds"]
  };

  private accessToken?: string;
  private refreshToken?: string;
  private baseUrl = "https://fleet-api.prd.na.vn.cloud.tesla.com";
  private streamingUrl = "wss://streaming.vn.tesla.com/streaming/";

  async connect(credentials: OEMCredentials): Promise<ConnectionResult> {
    if (credentials.type === "oauth") {
      this.accessToken = credentials.accessToken;
      this.refreshToken = credentials.refreshToken;
    } else if (credentials.type === "authCode") {
      const tokens = await this.exchangeAuthCode(credentials.authCode);
      this.accessToken = tokens.accessToken;
      this.refreshToken = tokens.refreshToken;
    }

    // Verify connection
    const health = await this.healthCheck();
    return {
      success: health.status === "HEALTHY",
      message: health.message
    };
  }

  async disconnect(): Promise<void> {
    this.accessToken = undefined;
    this.refreshToken = undefined;
  }

  async healthCheck(): Promise<HealthStatus> {
    try {
      const response = await this.apiRequest("GET", "/api/1/products");
      return { status: "HEALTHY", message: "Connected to Tesla API" };
    } catch (error) {
      return { status: "UNHEALTHY", message: String(error) };
    }
  }

  async listVehicles(): Promise<OEMVehicle[]> {
    const response = await this.apiRequest<TeslaVehiclesResponse>(
      "GET",
      "/api/1/products"
    );

    return response.response
      .filter(p => p.vehicle_id)
      .map(v => this.mapTeslaVehicle(v));
  }

  async getVehicleStatus(vehicleId: string): Promise<VehicleStatus> {
    // Wake up vehicle if needed
    await this.wakeUpVehicle(vehicleId);

    const response = await this.apiRequest<TeslaVehicleDataResponse>(
      "GET",
      `/api/1/vehicles/${vehicleId}/vehicle_data`
    );

    return this.mapTeslaStatus(response.response);
  }

  async getVehicleLocation(vehicleId: string): Promise<VehicleLocation> {
    const response = await this.apiRequest<TeslaVehicleDataResponse>(
      "GET",
      `/api/1/vehicles/${vehicleId}/vehicle_data?endpoints=location_data`
    );

    const drive = response.response.drive_state;
    return {
      latitude: drive.latitude,
      longitude: drive.longitude,
      heading: drive.heading,
      speed: drive.speed ? drive.speed * 1.60934 : 0, // mph to km/h
      timestamp: new Date(drive.timestamp),
      accuracy: 5  // Tesla doesn't provide accuracy, estimate
    };
  }

  async sendCommand(
    vehicleId: string,
    command: VehicleCommand
  ): Promise<CommandResult> {
    await this.wakeUpVehicle(vehicleId);

    const teslaCommand = this.mapCommandToTesla(command);

    try {
      const response = await this.apiRequest<TeslaCommandResponse>(
        "POST",
        `/api/1/vehicles/${vehicleId}/${teslaCommand.endpoint}`,
        teslaCommand.body
      );

      return {
        commandId: crypto.randomUUID(),
        status: response.response.result ? "COMPLETED" : "FAILED",
        message: response.response.reason
      };
    } catch (error: any) {
      return {
        commandId: crypto.randomUUID(),
        status: "FAILED",
        error: {
          code: error.code || "UNKNOWN",
          message: error.message,
          retryable: true
        }
      };
    }
  }

  subscribeToUpdates(
    vehicleId: string,
    callback: UpdateCallback
  ): Unsubscribe {
    const ws = new WebSocket(this.streamingUrl);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        msg_type: "data:subscribe_oauth",
        token: this.accessToken,
        value: "speed,odometer,soc,elevation,est_heading,est_lat,est_lng,power,shift_state,range,est_range,heading",
        tag: vehicleId
      }));
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.msg_type === "data:update") {
        callback(this.mapStreamingData(data));
      }
    };

    return () => ws.close();
  }

  private async wakeUpVehicle(vehicleId: string): Promise<void> {
    const maxAttempts = 5;
    let attempts = 0;

    while (attempts < maxAttempts) {
      try {
        const response = await this.apiRequest<TeslaVehicleResponse>(
          "POST",
          `/api/1/vehicles/${vehicleId}/wake_up`
        );

        if (response.response.state === "online") {
          return;
        }
      } catch (error) {
        // Ignore errors during wake up
      }

      await new Promise(resolve => setTimeout(resolve, 2000));
      attempts++;
    }

    throw new Error("Failed to wake up vehicle");
  }

  private async apiRequest<T>(
    method: string,
    endpoint: string,
    body?: any
  ): Promise<T> {
    const response = await fetch(`${this.baseUrl}${endpoint}`, {
      method,
      headers: {
        "Authorization": `Bearer ${this.accessToken}`,
        "Content-Type": "application/json"
      },
      body: body ? JSON.stringify(body) : undefined
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error || response.statusText);
    }

    return response.json();
  }

  private mapCommandToTesla(command: VehicleCommand): TeslaCommand {
    const mapping: Record<VehicleCommandType, TeslaCommand> = {
      [VehicleCommandType.LOCK]: { endpoint: "command/door_lock", body: {} },
      [VehicleCommandType.UNLOCK]: { endpoint: "command/door_unlock", body: {} },
      [VehicleCommandType.CLIMATE_ON]: { endpoint: "command/auto_conditioning_start", body: {} },
      [VehicleCommandType.CLIMATE_OFF]: { endpoint: "command/auto_conditioning_stop", body: {} },
      [VehicleCommandType.SET_TEMPERATURE]: {
        endpoint: "command/set_temps",
        body: {
          driver_temp: command.parameters?.temperature,
          passenger_temp: command.parameters?.temperature
        }
      },
      [VehicleCommandType.START_CHARGING]: { endpoint: "command/charge_start", body: {} },
      [VehicleCommandType.STOP_CHARGING]: { endpoint: "command/charge_stop", body: {} },
      [VehicleCommandType.SET_CHARGE_LIMIT]: {
        endpoint: "command/set_charge_limit",
        body: { percent: command.parameters?.limit }
      },
      [VehicleCommandType.FLASH_LIGHTS]: { endpoint: "command/flash_lights", body: {} },
      [VehicleCommandType.HONK_HORN]: { endpoint: "command/honk_horn", body: {} },
      [VehicleCommandType.OPEN_TRUNK]: { endpoint: "command/actuate_trunk", body: { which_trunk: "rear" } },
      [VehicleCommandType.OPEN_FRUNK]: { endpoint: "command/actuate_trunk", body: { which_trunk: "front" } },
      [VehicleCommandType.REMOTE_START]: { endpoint: "command/remote_start_drive", body: {} },
      [VehicleCommandType.VENT_WINDOWS]: { endpoint: "command/window_control", body: { command: "vent" } }
    } as any;

    return mapping[command.type] || { endpoint: "unknown", body: {} };
  }

  private mapTeslaVehicle(v: any): OEMVehicle {
    return {
      id: v.id_s,
      vin: v.vin,
      displayName: v.display_name,
      model: this.parseModelFromVIN(v.vin),
      year: this.parseYearFromVIN(v.vin),
      state: v.state
    };
  }

  private mapTeslaStatus(data: any): VehicleStatus {
    return {
      online: data.state === "online",
      locked: data.vehicle_state?.locked,
      doorsOpen: this.mapDoorState(data.vehicle_state),
      windowsOpen: this.mapWindowState(data.vehicle_state),
      engineRunning: data.drive_state?.shift_state !== null,
      batteryLevel: data.charge_state?.battery_level,
      range: data.charge_state?.battery_range * 1.60934, // miles to km
      charging: data.charge_state?.charging_state === "Charging",
      insideTemp: data.climate_state?.inside_temp,
      outsideTemp: data.climate_state?.outside_temp,
      odometer: data.vehicle_state?.odometer * 1.60934,
      timestamp: new Date()
    };
  }

  private mapDoorState(vs: any): string[] {
    const doors: string[] = [];
    if (vs?.df) doors.push("FRONT_LEFT");
    if (vs?.pf) doors.push("FRONT_RIGHT");
    if (vs?.dr) doors.push("REAR_LEFT");
    if (vs?.pr) doors.push("REAR_RIGHT");
    return doors;
  }

  private mapWindowState(vs: any): string[] {
    const windows: string[] = [];
    // Tesla API uses fd_window, fp_window, rd_window, rp_window
    if (vs?.fd_window) windows.push("FRONT_LEFT");
    if (vs?.fp_window) windows.push("FRONT_RIGHT");
    if (vs?.rd_window) windows.push("REAR_LEFT");
    if (vs?.rp_window) windows.push("REAR_RIGHT");
    return windows;
  }

  private mapStreamingData(data: any): VehicleUpdate {
    const values = data.value.split(",");
    return {
      vehicleId: data.tag,
      timestamp: new Date(parseInt(data.time)),
      signals: {
        speed: parseFloat(values[0]) * 1.60934,
        odometer: parseFloat(values[1]) * 1.60934,
        soc: parseFloat(values[2]),
        elevation: parseFloat(values[3]),
        heading: parseFloat(values[4]),
        latitude: parseFloat(values[5]),
        longitude: parseFloat(values[6]),
        power: parseFloat(values[7]),
        shiftState: values[8],
        range: parseFloat(values[9]) * 1.60934,
        estRange: parseFloat(values[10]) * 1.60934
      }
    };
  }

  private parseModelFromVIN(vin: string): string {
    const modelChar = vin.charAt(3);
    const models: Record<string, string> = {
      'S': 'Model S', 'X': 'Model X', '3': 'Model 3',
      'Y': 'Model Y', 'R': 'Roadster', 'T': 'Cybertruck'
    };
    return models[modelChar] || 'Unknown';
  }

  private parseYearFromVIN(vin: string): number {
    const yearChar = vin.charAt(9);
    const yearMap: Record<string, number> = {
      'A': 2010, 'B': 2011, 'C': 2012, 'D': 2013, 'E': 2014,
      'F': 2015, 'G': 2016, 'H': 2017, 'J': 2018, 'K': 2019,
      'L': 2020, 'M': 2021, 'N': 2022, 'P': 2023, 'R': 2024
    };
    return yearMap[yearChar] || 2020;
  }

  private async exchangeAuthCode(code: string): Promise<any> {
    // Implementation for OAuth code exchange
    return { accessToken: "", refreshToken: "" };
  }
}

// Type definitions
interface TeslaVehiclesResponse {
  response: any[];
  count: number;
}

interface TeslaVehicleDataResponse {
  response: any;
}

interface TeslaVehicleResponse {
  response: { state: string };
}

interface TeslaCommandResponse {
  response: { result: boolean; reason?: string };
}

interface TeslaCommand {
  endpoint: string;
  body: any;
}

interface OEMVehicle {
  id: string;
  vin: string;
  displayName: string;
  model: string;
  year: number;
  state: string;
}

interface VehicleStatus {
  online: boolean;
  locked: boolean;
  doorsOpen: string[];
  windowsOpen: string[];
  engineRunning: boolean;
  batteryLevel: number;
  range: number;
  charging: boolean;
  insideTemp: number;
  outsideTemp: number;
  odometer: number;
  timestamp: Date;
}

interface VehicleUpdate {
  vehicleId: string;
  timestamp: Date;
  signals: Record<string, any>;
}

interface OEMCredentials {
  type: "oauth" | "authCode";
  accessToken?: string;
  refreshToken?: string;
  authCode?: string;
}

interface ConnectionResult {
  success: boolean;
  message?: string;
}

interface HealthStatus {
  status: "HEALTHY" | "UNHEALTHY" | "DEGRADED";
  message?: string;
}

interface OEMAuthConfig {
  type: "OAUTH2" | "API_KEY" | "CERTIFICATE";
  authorizationUrl?: string;
  tokenUrl?: string;
  scopes?: string[];
}

type UpdateCallback = (update: VehicleUpdate) => void;
type Unsubscribe = () => void;

enum VehicleCommandType {
  LOCK = "LOCK",
  UNLOCK = "UNLOCK",
  CLIMATE_ON = "CLIMATE_ON",
  CLIMATE_OFF = "CLIMATE_OFF",
  SET_TEMPERATURE = "SET_TEMPERATURE",
  START_CHARGING = "START_CHARGING",
  STOP_CHARGING = "STOP_CHARGING",
  SET_CHARGE_LIMIT = "SET_CHARGE_LIMIT",
  OPEN_TRUNK = "OPEN_TRUNK",
  OPEN_FRUNK = "OPEN_FRUNK",
  FLASH_LIGHTS = "FLASH_LIGHTS",
  HONK_HORN = "HONK_HORN",
  REMOTE_START = "REMOTE_START",
  SUMMON = "SUMMON",
  VENT_WINDOWS = "VENT_WINDOWS"
}

interface VehicleCommand {
  type: VehicleCommandType;
  parameters?: Record<string, any>;
}

interface CommandResult {
  commandId: string;
  status: string;
  message?: string;
  error?: CommandError;
}

interface CommandError {
  code: string;
  message: string;
  retryable: boolean;
}

interface VehicleLocation {
  latitude: number;
  longitude: number;
  heading: number;
  speed: number;
  timestamp: Date;
  accuracy: number;
}
```

---

## Data Harmonization Layer

```typescript
/**
 * Data Harmonization Layer
 * Normalizing data from diverse OEM sources to WIA standard format
 */
class DataHarmonizationService {
  private signalMappings: Map<OEMIdentifier, SignalMappingConfig>;
  private unitConverters: Map<string, UnitConverter>;
  private validationRules: ValidationRuleSet;

  constructor() {
    this.signalMappings = this.loadSignalMappings();
    this.unitConverters = this.initializeUnitConverters();
    this.validationRules = this.loadValidationRules();
  }

  /**
   * Harmonize OEM data to WIA standard format
   */
  harmonize(
    oemData: OEMTelemetryData,
    oem: OEMIdentifier
  ): WIAStandardTelemetry {
    const mapping = this.signalMappings.get(oem);
    if (!mapping) {
      throw new Error(`No signal mapping for OEM: ${oem}`);
    }

    const harmonized: WIAStandardTelemetry = {
      vehicleId: oemData.vehicleId,
      timestamp: new Date(oemData.timestamp),
      source: oem,
      signals: {}
    };

    for (const [oemSignal, value] of Object.entries(oemData.signals)) {
      const wiaSignal = mapping.signals[oemSignal];
      if (!wiaSignal) continue;

      // Convert value
      const convertedValue = this.convertValue(
        value,
        wiaSignal.sourceUnit,
        wiaSignal.targetUnit
      );

      // Validate
      const validation = this.validateSignal(
        wiaSignal.wiaPath,
        convertedValue
      );

      if (validation.valid) {
        this.setNestedValue(
          harmonized.signals,
          wiaSignal.wiaPath,
          {
            value: convertedValue,
            unit: wiaSignal.targetUnit,
            quality: validation.quality,
            timestamp: oemData.timestamp
          }
        );
      } else {
        console.warn(
          `Signal validation failed: ${wiaSignal.wiaPath}`,
          validation.errors
        );
      }
    }

    return harmonized;
  }

  /**
   * Convert units between OEM and WIA standard
   */
  private convertValue(
    value: any,
    sourceUnit: string,
    targetUnit: string
  ): any {
    if (sourceUnit === targetUnit) return value;
    if (typeof value !== "number") return value;

    const converter = this.unitConverters.get(`${sourceUnit}->${targetUnit}`);
    if (!converter) {
      console.warn(`No converter for ${sourceUnit} -> ${targetUnit}`);
      return value;
    }

    return converter.convert(value);
  }

  /**
   * Validate signal against WIA schema
   */
  private validateSignal(
    path: string,
    value: any
  ): SignalValidationResult {
    const rule = this.validationRules.rules[path];
    if (!rule) {
      return { valid: true, quality: "UNCERTAIN" };
    }

    const errors: string[] = [];

    // Type check
    if (rule.type && typeof value !== rule.type) {
      errors.push(`Expected type ${rule.type}, got ${typeof value}`);
    }

    // Range check
    if (typeof value === "number") {
      if (rule.min !== undefined && value < rule.min) {
        errors.push(`Value ${value} below minimum ${rule.min}`);
      }
      if (rule.max !== undefined && value > rule.max) {
        errors.push(`Value ${value} above maximum ${rule.max}`);
      }
    }

    // Enum check
    if (rule.enum && !rule.enum.includes(value)) {
      errors.push(`Value ${value} not in allowed values: ${rule.enum}`);
    }

    return {
      valid: errors.length === 0,
      quality: errors.length === 0 ? "GOOD" : "BAD",
      errors: errors.length > 0 ? errors : undefined
    };
  }

  private setNestedValue(obj: any, path: string, value: any): void {
    const parts = path.split(".");
    let current = obj;

    for (let i = 0; i < parts.length - 1; i++) {
      if (!current[parts[i]]) {
        current[parts[i]] = {};
      }
      current = current[parts[i]];
    }

    current[parts[parts.length - 1]] = value;
  }

  private loadSignalMappings(): Map<OEMIdentifier, SignalMappingConfig> {
    const mappings = new Map<OEMIdentifier, SignalMappingConfig>();

    // Tesla signal mappings
    mappings.set("TESLA", {
      oem: "TESLA",
      version: "1.0",
      signals: {
        "battery_level": {
          wiaPath: "battery.stateOfCharge",
          sourceUnit: "percent",
          targetUnit: "percent"
        },
        "battery_range": {
          wiaPath: "battery.range",
          sourceUnit: "miles",
          targetUnit: "km"
        },
        "charging_state": {
          wiaPath: "battery.charging.isCharging",
          sourceUnit: "string",
          targetUnit: "boolean",
          transform: (v: string) => v === "Charging"
        },
        "inside_temp": {
          wiaPath: "climate.insideTemperature",
          sourceUnit: "celsius",
          targetUnit: "celsius"
        },
        "outside_temp": {
          wiaPath: "climate.outsideTemperature",
          sourceUnit: "celsius",
          targetUnit: "celsius"
        },
        "latitude": {
          wiaPath: "location.latitude",
          sourceUnit: "degrees",
          targetUnit: "degrees"
        },
        "longitude": {
          wiaPath: "location.longitude",
          sourceUnit: "degrees",
          targetUnit: "degrees"
        },
        "heading": {
          wiaPath: "location.heading",
          sourceUnit: "degrees",
          targetUnit: "degrees"
        },
        "speed": {
          wiaPath: "motion.speed",
          sourceUnit: "mph",
          targetUnit: "km/h"
        },
        "odometer": {
          wiaPath: "motion.odometer",
          sourceUnit: "miles",
          targetUnit: "km"
        },
        "locked": {
          wiaPath: "security.locked",
          sourceUnit: "boolean",
          targetUnit: "boolean"
        },
        "sentry_mode": {
          wiaPath: "security.alarmArmed",
          sourceUnit: "boolean",
          targetUnit: "boolean"
        }
      }
    });

    // GM signal mappings
    mappings.set("GM", {
      oem: "GM",
      version: "1.0",
      signals: {
        "BATTERY_CHARGE_LEVEL": {
          wiaPath: "battery.stateOfCharge",
          sourceUnit: "percent",
          targetUnit: "percent"
        },
        "EV_RANGE": {
          wiaPath: "battery.range",
          sourceUnit: "km",
          targetUnit: "km"
        },
        "AMBIENT_AIR_TEMP": {
          wiaPath: "climate.outsideTemperature",
          sourceUnit: "celsius",
          targetUnit: "celsius"
        },
        "VEHICLE_SPEED": {
          wiaPath: "motion.speed",
          sourceUnit: "km/h",
          targetUnit: "km/h"
        },
        "ODOMETER": {
          wiaPath: "motion.odometer",
          sourceUnit: "km",
          targetUnit: "km"
        }
      }
    });

    return mappings;
  }

  private initializeUnitConverters(): Map<string, UnitConverter> {
    const converters = new Map<string, UnitConverter>();

    // Distance
    converters.set("miles->km", {
      convert: (v) => v * 1.60934
    });
    converters.set("km->miles", {
      convert: (v) => v / 1.60934
    });

    // Speed
    converters.set("mph->km/h", {
      convert: (v) => v * 1.60934
    });
    converters.set("km/h->mph", {
      convert: (v) => v / 1.60934
    });

    // Temperature
    converters.set("fahrenheit->celsius", {
      convert: (v) => (v - 32) * 5 / 9
    });
    converters.set("celsius->fahrenheit", {
      convert: (v) => (v * 9 / 5) + 32
    });

    // Pressure
    converters.set("psi->kpa", {
      convert: (v) => v * 6.89476
    });
    converters.set("kpa->psi", {
      convert: (v) => v / 6.89476
    });

    return converters;
  }

  private loadValidationRules(): ValidationRuleSet {
    return {
      version: "1.0",
      rules: {
        "battery.stateOfCharge": {
          type: "number",
          min: 0,
          max: 100
        },
        "battery.range": {
          type: "number",
          min: 0,
          max: 1000
        },
        "location.latitude": {
          type: "number",
          min: -90,
          max: 90
        },
        "location.longitude": {
          type: "number",
          min: -180,
          max: 180
        },
        "location.heading": {
          type: "number",
          min: 0,
          max: 360
        },
        "motion.speed": {
          type: "number",
          min: 0,
          max: 400
        },
        "climate.insideTemperature": {
          type: "number",
          min: -50,
          max: 80
        },
        "climate.outsideTemperature": {
          type: "number",
          min: -50,
          max: 60
        }
      }
    };
  }
}

interface OEMTelemetryData {
  vehicleId: string;
  timestamp: string;
  signals: Record<string, any>;
}

interface WIAStandardTelemetry {
  vehicleId: string;
  timestamp: Date;
  source: OEMIdentifier;
  signals: Record<string, SignalData>;
}

interface SignalData {
  value: any;
  unit: string;
  quality: SignalQuality;
  timestamp: string;
}

type SignalQuality = "GOOD" | "UNCERTAIN" | "BAD";

interface SignalMappingConfig {
  oem: OEMIdentifier;
  version: string;
  signals: Record<string, SignalMapping>;
}

interface SignalMapping {
  wiaPath: string;
  sourceUnit: string;
  targetUnit: string;
  transform?: (value: any) => any;
}

interface UnitConverter {
  convert: (value: number) => number;
}

interface ValidationRuleSet {
  version: string;
  rules: Record<string, ValidationRule>;
}

interface ValidationRule {
  type?: string;
  min?: number;
  max?: number;
  enum?: any[];
}

interface SignalValidationResult {
  valid: boolean;
  quality: SignalQuality;
  errors?: string[];
}
```

---

## Third-Party Service Integration

### Charging Network Integration

```typescript
/**
 * EV Charging Network Integration
 * Unified access to multiple charging networks
 */
interface ChargingNetworkIntegration {
  networks: ChargingNetwork[];
  aggregator: ChargingAggregator;
  roaming: RoamingProtocol;
}

interface ChargingNetwork {
  id: string;
  name: string;
  protocol: ChargingProtocol;
  coverage: NetworkCoverage;
}

type ChargingProtocol = "OCPI" | "OCPP" | "OICP" | "PROPRIETARY";

/**
 * OCPI (Open Charge Point Interface) Client
 */
class OCPIClient {
  private baseUrl: string;
  private token: string;
  private countryCode: string;
  private partyId: string;

  constructor(config: OCPIClientConfig) {
    this.baseUrl = config.baseUrl;
    this.token = config.token;
    this.countryCode = config.countryCode;
    this.partyId = config.partyId;
  }

  /**
   * Get charging locations
   */
  async getLocations(
    params?: LocationQueryParams
  ): Promise<OCPILocation[]> {
    const queryString = new URLSearchParams();
    if (params?.offset) queryString.set("offset", String(params.offset));
    if (params?.limit) queryString.set("limit", String(params.limit));
    if (params?.dateFrom) queryString.set("date_from", params.dateFrom);
    if (params?.dateTo) queryString.set("date_to", params.dateTo);

    const response = await this.request<OCPILocationResponse>(
      "GET",
      `/ocpi/2.2/locations?${queryString}`
    );

    return response.data;
  }

  /**
   * Get specific location details
   */
  async getLocation(locationId: string): Promise<OCPILocation> {
    const response = await this.request<OCPILocationDetailResponse>(
      "GET",
      `/ocpi/2.2/locations/${this.countryCode}/${this.partyId}/${locationId}`
    );

    return response.data;
  }

  /**
   * Start charging session
   */
  async startSession(
    locationId: string,
    evseUid: string,
    connectorId: string,
    authorizationId: string
  ): Promise<OCPISession> {
    const sessionId = crypto.randomUUID();

    const session: OCPISessionCreate = {
      id: sessionId,
      start_date_time: new Date().toISOString(),
      kwh: 0,
      auth_id: authorizationId,
      auth_method: "AUTH_REQUEST",
      location_id: locationId,
      evse_uid: evseUid,
      connector_id: connectorId,
      currency: "USD",
      total_cost: 0,
      status: "ACTIVE"
    };

    const response = await this.request<OCPISessionResponse>(
      "PUT",
      `/ocpi/2.2/sessions/${this.countryCode}/${this.partyId}/${sessionId}`,
      session
    );

    return response.data;
  }

  /**
   * Stop charging session
   */
  async stopSession(sessionId: string): Promise<OCPISession> {
    const response = await this.request<OCPISessionResponse>(
      "PATCH",
      `/ocpi/2.2/sessions/${this.countryCode}/${this.partyId}/${sessionId}`,
      { status: "COMPLETED" }
    );

    return response.data;
  }

  /**
   * Get session details
   */
  async getSession(sessionId: string): Promise<OCPISession> {
    const response = await this.request<OCPISessionResponse>(
      "GET",
      `/ocpi/2.2/sessions/${this.countryCode}/${this.partyId}/${sessionId}`
    );

    return response.data;
  }

  /**
   * Send command to charging station
   */
  async sendCommand(
    locationId: string,
    evseUid: string,
    command: OCPICommand
  ): Promise<OCPICommandResponse> {
    const response = await this.request<OCPICommandResponse>(
      "POST",
      `/ocpi/2.2/commands/${command.type}`,
      {
        response_url: `${this.baseUrl}/ocpi/2.2/commands/${command.type}/callback`,
        ...command.payload
      }
    );

    return response;
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method,
      headers: {
        "Authorization": `Token ${this.token}`,
        "Content-Type": "application/json",
        "X-Request-ID": crypto.randomUUID(),
        "X-Correlation-ID": crypto.randomUUID()
      },
      body: body ? JSON.stringify(body) : undefined
    });

    if (!response.ok) {
      throw new Error(`OCPI request failed: ${response.status}`);
    }

    return response.json();
  }
}

interface OCPIClientConfig {
  baseUrl: string;
  token: string;
  countryCode: string;
  partyId: string;
}

interface LocationQueryParams {
  offset?: number;
  limit?: number;
  dateFrom?: string;
  dateTo?: string;
}

interface OCPILocation {
  id: string;
  type: "ON_STREET" | "PARKING_GARAGE" | "UNDERGROUND_GARAGE" | "PARKING_LOT" | "OTHER" | "UNKNOWN";
  name?: string;
  address: string;
  city: string;
  postal_code: string;
  country: string;
  coordinates: { latitude: string; longitude: string };
  evses: OCPIEVSE[];
  directions?: OCPIDisplayText[];
  operator?: OCPIBusinessDetails;
  opening_times?: OCPIHours;
  charging_when_closed?: boolean;
  facilities?: string[];
  time_zone: string;
  last_updated: string;
}

interface OCPIEVSE {
  uid: string;
  evse_id?: string;
  status: "AVAILABLE" | "BLOCKED" | "CHARGING" | "INOPERATIVE" | "OUTOFORDER" | "PLANNED" | "REMOVED" | "RESERVED" | "UNKNOWN";
  status_schedule?: OCPIStatusSchedule[];
  capabilities?: string[];
  connectors: OCPIConnector[];
  floor_level?: string;
  coordinates?: { latitude: string; longitude: string };
  physical_reference?: string;
  directions?: OCPIDisplayText[];
  parking_restrictions?: string[];
  last_updated: string;
}

interface OCPIConnector {
  id: string;
  standard: ConnectorType;
  format: "SOCKET" | "CABLE";
  power_type: "AC_1_PHASE" | "AC_3_PHASE" | "DC";
  max_voltage: number;
  max_amperage: number;
  max_electric_power?: number;
  tariff_ids?: string[];
  last_updated: string;
}

type ConnectorType =
  | "CHADEMO" | "IEC_62196_T1" | "IEC_62196_T1_COMBO"
  | "IEC_62196_T2" | "IEC_62196_T2_COMBO" | "IEC_62196_T3A"
  | "IEC_62196_T3C" | "DOMESTIC_A" | "DOMESTIC_B" | "DOMESTIC_C"
  | "DOMESTIC_D" | "DOMESTIC_E" | "DOMESTIC_F" | "DOMESTIC_G"
  | "DOMESTIC_H" | "DOMESTIC_I" | "DOMESTIC_J" | "DOMESTIC_K"
  | "DOMESTIC_L" | "TESLA_R" | "TESLA_S" | "IEC_60309_2_single_16"
  | "IEC_60309_2_three_16" | "IEC_60309_2_three_32" | "IEC_60309_2_three_64";

interface OCPISession {
  id: string;
  start_date_time: string;
  end_date_time?: string;
  kwh: number;
  auth_id: string;
  auth_method: string;
  location_id: string;
  evse_uid: string;
  connector_id: string;
  meter_id?: string;
  currency: string;
  charging_periods?: OCPIChargingPeriod[];
  total_cost?: number;
  status: "ACTIVE" | "COMPLETED" | "INVALID" | "PENDING";
  last_updated: string;
}

interface OCPIChargingPeriod {
  start_date_time: string;
  dimensions: OCPIDimension[];
}

interface OCPIDimension {
  type: "ENERGY" | "FLAT" | "MAX_CURRENT" | "MIN_CURRENT" | "PARKING_TIME" | "TIME";
  volume: number;
}

interface OCPICommand {
  type: "CANCEL_RESERVATION" | "RESERVE_NOW" | "START_SESSION" | "STOP_SESSION" | "UNLOCK_CONNECTOR";
  payload: any;
}

interface OCPICommandResponse {
  result: "ACCEPTED" | "CANCELED_RESERVATION" | "EVSE_OCCUPIED" | "EVSE_INOPERATIVE" | "FAILED" | "NOT_SUPPORTED" | "REJECTED" | "TIMEOUT" | "UNKNOWN_RESERVATION";
  timeout: number;
  message?: OCPIDisplayText[];
}

interface OCPIDisplayText {
  language: string;
  text: string;
}

interface OCPIBusinessDetails {
  name: string;
  website?: string;
  logo?: OCPIImage;
}

interface OCPIImage {
  url: string;
  thumbnail?: string;
  category: string;
  type: string;
  width?: number;
  height?: number;
}

interface OCPIHours {
  regular_hours?: OCPIRegularHours[];
  twentyfourseven: boolean;
  exceptional_openings?: OCPIExceptionalPeriod[];
  exceptional_closings?: OCPIExceptionalPeriod[];
}

interface OCPIRegularHours {
  weekday: number;
  period_begin: string;
  period_end: string;
}

interface OCPIExceptionalPeriod {
  period_begin: string;
  period_end: string;
}

interface OCPIStatusSchedule {
  period_begin: string;
  period_end?: string;
  status: string;
}

interface NetworkCoverage {
  countries: string[];
  stationCount: number;
  connectorCount: number;
}

// Response types
interface OCPILocationResponse {
  data: OCPILocation[];
  status_code: number;
  timestamp: string;
}

interface OCPILocationDetailResponse {
  data: OCPILocation;
  status_code: number;
  timestamp: string;
}

interface OCPISessionResponse {
  data: OCPISession;
  status_code: number;
  timestamp: string;
}

interface OCPISessionCreate {
  id: string;
  start_date_time: string;
  kwh: number;
  auth_id: string;
  auth_method: string;
  location_id: string;
  evse_uid: string;
  connector_id: string;
  currency: string;
  total_cost: number;
  status: string;
}
```

---

## Event Aggregation

```typescript
/**
 * Event Aggregation Service
 * Consolidating events from multiple sources
 */
class EventAggregationService {
  private eventBus: EventBus;
  private subscribers: Map<string, EventSubscriber[]> = new Map();
  private eventStore: EventStore;

  constructor(
    eventBus: EventBus,
    eventStore: EventStore
  ) {
    this.eventBus = eventBus;
    this.eventStore = eventStore;

    // Subscribe to all OEM events
    this.setupOEMSubscriptions();
  }

  /**
   * Subscribe to vehicle events
   */
  subscribe(
    vehicleId: string,
    eventTypes: VehicleEventType[],
    callback: EventCallback
  ): Unsubscribe {
    const subscriberId = crypto.randomUUID();

    for (const eventType of eventTypes) {
      const key = `${vehicleId}:${eventType}`;
      if (!this.subscribers.has(key)) {
        this.subscribers.set(key, []);
      }
      this.subscribers.get(key)!.push({ id: subscriberId, callback });
    }

    return () => {
      for (const eventType of eventTypes) {
        const key = `${vehicleId}:${eventType}`;
        const subs = this.subscribers.get(key) || [];
        this.subscribers.set(
          key,
          subs.filter(s => s.id !== subscriberId)
        );
      }
    };
  }

  /**
   * Publish event from any source
   */
  async publishEvent(event: VehicleEvent): Promise<void> {
    // Store event
    await this.eventStore.store(event);

    // Notify subscribers
    const key = `${event.vehicleId}:${event.type}`;
    const subscribers = this.subscribers.get(key) || [];

    for (const subscriber of subscribers) {
      try {
        subscriber.callback(event);
      } catch (error) {
        console.error("Event callback error:", error);
      }
    }

    // Broadcast to event bus
    this.eventBus.publish(event);
  }

  /**
   * Query event history
   */
  async queryEvents(
    query: EventQuery
  ): Promise<VehicleEvent[]> {
    return this.eventStore.query(query);
  }

  private setupOEMSubscriptions(): void {
    // Setup would connect to various OEM event sources
    // and forward events through publishEvent
  }
}

interface VehicleEvent {
  id: string;
  vehicleId: string;
  type: VehicleEventType;
  severity: EventSeverity;
  timestamp: Date;
  source: string;
  data: Record<string, any>;
  location?: VehicleLocation;
}

type VehicleEventType =
  | "SECURITY_ALERT"
  | "CRASH_DETECTED"
  | "THEFT_ALERT"
  | "GEOFENCE_ENTER"
  | "GEOFENCE_EXIT"
  | "LOW_BATTERY"
  | "LOW_FUEL"
  | "MAINTENANCE_DUE"
  | "DTC_ACTIVE"
  | "CHARGING_STARTED"
  | "CHARGING_COMPLETE"
  | "CHARGING_INTERRUPTED"
  | "DOOR_UNLOCKED"
  | "SPEED_LIMIT_EXCEEDED"
  | "RECALL_NOTICE";

type EventSeverity = "INFO" | "WARNING" | "CRITICAL" | "EMERGENCY";

interface EventQuery {
  vehicleId?: string;
  types?: VehicleEventType[];
  severity?: EventSeverity[];
  from?: Date;
  to?: Date;
  limit?: number;
  offset?: number;
}

interface EventSubscriber {
  id: string;
  callback: EventCallback;
}

type EventCallback = (event: VehicleEvent) => void;

interface EventBus {
  publish(event: VehicleEvent): void;
  subscribe(callback: EventCallback): Unsubscribe;
}

interface EventStore {
  store(event: VehicleEvent): Promise<void>;
  query(query: EventQuery): Promise<VehicleEvent[]>;
}
```

---

## Summary

| Integration Type | Protocol | Latency | Complexity |
|------------------|----------|---------|------------|
| **OEM API** | REST/OAuth2 | 1-30s | Medium |
| **Charging (OCPI)** | REST | 1-5s | Medium |
| **V2X Infrastructure** | MQTT/AMQP | <100ms | High |
| **Fleet Management** | WebSocket | <1s | Low |
| **Insurance Telematics** | REST/Batch | Minutes | Low |

---

**Next Chapter:** [Chapter 7: Security](./07-security.md) - Cybersecurity and privacy frameworks.

---

© 2025 World Industry Association (WIA). All rights reserved.
