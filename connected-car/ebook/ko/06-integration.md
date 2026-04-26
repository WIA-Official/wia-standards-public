# 제6장: 커넥티드카 통합

## OEM 및 에코시스템 통합 패턴

이 장에서는 여러 OEM, 서드파티 서비스 및 모빌리티 플랫폼 간의 원활한 통합을 가능하게 하는 아키텍처 패턴과 구현 전략을 정의합니다.

---

## 멀티-OEM 통합 플랫폼

### 플랫폼 아키텍처

```typescript
// WIA 커넥티드카 통합 플랫폼
// 멀티-OEM 차량 데이터 통합 및 정규화

/**
 * 멀티-OEM 통합 플랫폼 아키텍처
 */
interface IntegrationPlatformArchitecture {
  adapters: OEMAdapterRegistry;
  dataHarmonization: DataHarmonizationEngine;
  apiGateway: UnifiedAPIGateway;
  eventBus: EventBroker;
  dataStore: TimeSeriesDataStore;
  analytics: AnalyticsPipeline;
}

/**
 * OEM 어댑터 레지스트리
 * 여러 OEM 통합을 위한 플러그인 시스템
 */
class OEMAdapterRegistry {
  private adapters: Map<string, OEMAdapter> = new Map();
  private configs: Map<string, OEMConfiguration> = new Map();

  /**
   * OEM 어댑터 등록
   */
  registerAdapter(oemId: string, adapter: OEMAdapter, config: OEMConfiguration): void {
    this.adapters.set(oemId, adapter);
    this.configs.set(oemId, config);

    console.log(`OEM 어댑터 등록됨: ${oemId}`);
  }

  /**
   * OEM용 어댑터 가져오기
   */
  getAdapter(oemId: string): OEMAdapter | undefined {
    return this.adapters.get(oemId);
  }

  /**
   * VIN에서 OEM 탐지
   */
  detectOEMFromVIN(vin: string): string | null {
    const wmi = vin.substring(0, 3);

    const wmiMapping: Record<string, string> = {
      "1G1": "chevrolet",
      "1HG": "honda",
      "2HG": "honda_canada",
      "3FA": "ford",
      "5YJ": "tesla",
      "5YJ": "tesla",
      "KND": "kia",
      "KMH": "hyundai",
      "WAU": "audi",
      "WBA": "bmw",
      "WDB": "mercedes",
      "WVW": "volkswagen",
      "JTD": "toyota",
      "JN1": "nissan"
    };

    return wmiMapping[wmi] || null;
  }

  /**
   * 등록된 모든 OEM 나열
   */
  listRegisteredOEMs(): string[] {
    return Array.from(this.adapters.keys());
  }
}

/**
 * OEM 어댑터 인터페이스
 */
interface OEMAdapter {
  oemId: string;
  displayName: string;
  supportedFeatures: VehicleFeature[];

  // 인증
  authenticate(credentials: OEMCredentials): Promise<AuthToken>;
  refreshToken(token: AuthToken): Promise<AuthToken>;

  // 차량 검색
  listVehicles(token: AuthToken): Promise<OEMVehicle[]>;
  getVehicle(token: AuthToken, vehicleId: string): Promise<OEMVehicle>;

  // 데이터 조회
  getVehicleStatus(token: AuthToken, vehicleId: string): Promise<OEMVehicleStatus>;
  getLocation(token: AuthToken, vehicleId: string): Promise<OEMLocation>;
  getTelemetry(token: AuthToken, vehicleId: string, signals: string[]): Promise<OEMTelemetry>;

  // 명령
  sendCommand(token: AuthToken, vehicleId: string, command: OEMCommand): Promise<OEMCommandResult>;

  // 데이터 변환
  normalizeVehicle(oemVehicle: OEMVehicle): NormalizedVehicle;
  normalizeStatus(oemStatus: OEMVehicleStatus): NormalizedVehicleStatus;
  normalizeLocation(oemLocation: OEMLocation): NormalizedLocation;
}

type VehicleFeature =
  | "REMOTE_LOCK"
  | "REMOTE_START"
  | "CLIMATE_CONTROL"
  | "CHARGING_CONTROL"
  | "LOCATION_TRACKING"
  | "TELEMETRY"
  | "DIAGNOSTICS"
  | "OTA_UPDATE"
  | "SUMMON"
  | "AUTOPILOT_STATUS";

interface OEMConfiguration {
  apiBaseUrl: string;
  authUrl: string;
  clientId: string;
  clientSecret: string;
  scopes: string[];
  rateLimit: RateLimitConfig;
  retryPolicy: RetryPolicyConfig;
}

interface AuthToken {
  accessToken: string;
  refreshToken: string;
  expiresAt: Date;
  scopes: string[];
}

interface OEMCredentials {
  username?: string;
  password?: string;
  oauthCode?: string;
  apiKey?: string;
}
```

---

## Tesla API 어댑터 구현

```typescript
/**
 * Tesla API 어댑터
 * 전체 Tesla Fleet API 통합
 */
class TeslaAdapter implements OEMAdapter {
  oemId = "tesla";
  displayName = "Tesla";
  supportedFeatures: VehicleFeature[] = [
    "REMOTE_LOCK",
    "REMOTE_START",
    "CLIMATE_CONTROL",
    "CHARGING_CONTROL",
    "LOCATION_TRACKING",
    "TELEMETRY",
    "DIAGNOSTICS",
    "OTA_UPDATE",
    "SUMMON",
    "AUTOPILOT_STATUS"
  ];

  private baseUrl: string;
  private authUrl: string;

  constructor(config: TeslaConfig) {
    this.baseUrl = config.baseUrl || "https://fleet-api.prd.na.vn.cloud.tesla.com";
    this.authUrl = config.authUrl || "https://auth.tesla.com";
  }

  /**
   * Tesla OAuth 2.0 인증
   */
  async authenticate(credentials: OEMCredentials): Promise<AuthToken> {
    const response = await fetch(`${this.authUrl}/oauth2/v3/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code: credentials.oauthCode!,
        client_id: process.env.TESLA_CLIENT_ID!,
        client_secret: process.env.TESLA_CLIENT_SECRET!,
        redirect_uri: process.env.TESLA_REDIRECT_URI!,
        scope: "openid offline_access vehicle_device_data vehicle_cmds"
      })
    });

    const data = await response.json();

    return {
      accessToken: data.access_token,
      refreshToken: data.refresh_token,
      expiresAt: new Date(Date.now() + data.expires_in * 1000),
      scopes: data.scope.split(" ")
    };
  }

  /**
   * 토큰 갱신
   */
  async refreshToken(token: AuthToken): Promise<AuthToken> {
    const response = await fetch(`${this.authUrl}/oauth2/v3/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "refresh_token",
        refresh_token: token.refreshToken,
        client_id: process.env.TESLA_CLIENT_ID!,
        client_secret: process.env.TESLA_CLIENT_SECRET!
      })
    });

    const data = await response.json();

    return {
      accessToken: data.access_token,
      refreshToken: data.refresh_token,
      expiresAt: new Date(Date.now() + data.expires_in * 1000),
      scopes: data.scope.split(" ")
    };
  }

  /**
   * 차량 목록 조회
   */
  async listVehicles(token: AuthToken): Promise<OEMVehicle[]> {
    const response = await this.apiRequest(token, "GET", "/api/1/vehicles");
    return response.response.map((v: any) => this.mapTeslaVehicle(v));
  }

  /**
   * 차량 상태 조회
   */
  async getVehicleStatus(token: AuthToken, vehicleId: string): Promise<OEMVehicleStatus> {
    const response = await this.apiRequest(
      token,
      "GET",
      `/api/1/vehicles/${vehicleId}/vehicle_data`
    );

    return this.mapTeslaStatus(response.response);
  }

  /**
   * 위치 조회
   */
  async getLocation(token: AuthToken, vehicleId: string): Promise<OEMLocation> {
    const response = await this.apiRequest(
      token,
      "GET",
      `/api/1/vehicles/${vehicleId}/vehicle_data?endpoints=location_data`
    );

    const driveState = response.response.drive_state;

    return {
      latitude: driveState.latitude,
      longitude: driveState.longitude,
      heading: driveState.heading,
      speed: driveState.speed,
      timestamp: new Date(driveState.timestamp)
    };
  }

  /**
   * 명령 전송
   */
  async sendCommand(
    token: AuthToken,
    vehicleId: string,
    command: OEMCommand
  ): Promise<OEMCommandResult> {
    const teslaCommand = this.mapToTeslaCommand(command);

    // 먼저 차량 깨우기
    await this.wakeVehicle(token, vehicleId);

    const response = await this.apiRequest(
      token,
      "POST",
      `/api/1/vehicles/${vehicleId}/command/${teslaCommand.endpoint}`,
      teslaCommand.body
    );

    return {
      success: response.response.result,
      reason: response.response.reason
    };
  }

  /**
   * Tesla 차량을 정규화된 형식으로 변환
   */
  normalizeVehicle(oemVehicle: OEMVehicle): NormalizedVehicle {
    return {
      id: oemVehicle.id,
      vin: oemVehicle.vin,
      make: "Tesla",
      model: this.mapTeslaModel(oemVehicle.raw.vehicle_type),
      year: this.extractYearFromVIN(oemVehicle.vin),
      color: oemVehicle.raw.color,
      powertrainType: "BEV",
      capabilities: this.supportedFeatures,
      oemData: {
        displayName: oemVehicle.raw.display_name,
        optionCodes: oemVehicle.raw.option_codes
      }
    };
  }

  /**
   * Tesla 상태를 정규화된 형식으로 변환
   */
  normalizeStatus(oemStatus: OEMVehicleStatus): NormalizedVehicleStatus {
    const raw = oemStatus.raw;

    return {
      online: raw.state === "online",
      locked: raw.vehicle_state?.locked,
      doorsOpen: this.extractOpenDoors(raw.vehicle_state),
      windowsOpen: this.extractOpenWindows(raw.vehicle_state),
      trunkOpen: raw.vehicle_state?.rt > 0,
      frunkOpen: raw.vehicle_state?.ft > 0,
      charging: {
        isCharging: raw.charge_state?.charging_state === "Charging",
        isPluggedIn: raw.charge_state?.charge_port_door_open,
        stateOfCharge: raw.charge_state?.battery_level,
        chargeLimit: raw.charge_state?.charge_limit_soc,
        chargePower: raw.charge_state?.charger_power,
        chargeRate: raw.charge_state?.charge_rate,
        timeToFull: raw.charge_state?.minutes_to_full_charge,
        range: raw.charge_state?.battery_range * 1.60934  // 마일을 km로
      },
      climate: {
        isOn: raw.climate_state?.is_climate_on,
        insideTemp: raw.climate_state?.inside_temp,
        outsideTemp: raw.climate_state?.outside_temp,
        targetTemp: raw.climate_state?.driver_temp_setting,
        seatHeaters: {
          frontLeft: raw.climate_state?.seat_heater_left,
          frontRight: raw.climate_state?.seat_heater_right,
          rearLeft: raw.climate_state?.seat_heater_rear_left,
          rearRight: raw.climate_state?.seat_heater_rear_right
        }
      },
      odometer: raw.vehicle_state?.odometer * 1.60934,  // 마일을 km로
      softwareVersion: raw.vehicle_state?.car_version,
      timestamp: new Date()
    };
  }

  normalizeLocation(oemLocation: OEMLocation): NormalizedLocation {
    return {
      latitude: oemLocation.latitude,
      longitude: oemLocation.longitude,
      altitude: oemLocation.altitude,
      heading: oemLocation.heading,
      speed: oemLocation.speed ? oemLocation.speed * 1.60934 : undefined,
      accuracy: 10,
      source: "GPS",
      timestamp: oemLocation.timestamp
    };
  }

  /**
   * API 요청 헬퍼
   */
  private async apiRequest(
    token: AuthToken,
    method: string,
    path: string,
    body?: any
  ): Promise<any> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method,
      headers: {
        "Authorization": `Bearer ${token.accessToken}`,
        "Content-Type": "application/json"
      },
      body: body ? JSON.stringify(body) : undefined
    });

    if (!response.ok) {
      throw new APIError(response.status, await response.text());
    }

    return response.json();
  }

  /**
   * 차량 깨우기
   */
  private async wakeVehicle(token: AuthToken, vehicleId: string): Promise<void> {
    const maxAttempts = 5;
    const delayMs = 2000;

    for (let i = 0; i < maxAttempts; i++) {
      const response = await this.apiRequest(
        token,
        "POST",
        `/api/1/vehicles/${vehicleId}/wake_up`
      );

      if (response.response.state === "online") {
        return;
      }

      await new Promise(resolve => setTimeout(resolve, delayMs));
    }

    throw new Error("차량 깨우기 실패");
  }

  /**
   * 명령을 Tesla API 엔드포인트로 매핑
   */
  private mapToTeslaCommand(command: OEMCommand): TeslaCommandMapping {
    const mappings: Record<string, TeslaCommandMapping> = {
      LOCK: { endpoint: "door_lock", body: {} },
      UNLOCK: { endpoint: "door_unlock", body: {} },
      CLIMATE_ON: { endpoint: "auto_conditioning_start", body: {} },
      CLIMATE_OFF: { endpoint: "auto_conditioning_stop", body: {} },
      SET_TEMPERATURE: {
        endpoint: "set_temps",
        body: {
          driver_temp: command.parameters?.temperature,
          passenger_temp: command.parameters?.temperature
        }
      },
      START_CHARGING: { endpoint: "charge_start", body: {} },
      STOP_CHARGING: { endpoint: "charge_stop", body: {} },
      SET_CHARGE_LIMIT: {
        endpoint: "set_charge_limit",
        body: { percent: command.parameters?.limit }
      },
      FLASH_LIGHTS: { endpoint: "flash_lights", body: {} },
      HONK_HORN: { endpoint: "honk_horn", body: {} },
      OPEN_TRUNK: { endpoint: "actuate_trunk", body: { which_trunk: "rear" } },
      OPEN_FRUNK: { endpoint: "actuate_trunk", body: { which_trunk: "front" } }
    };

    return mappings[command.type] || { endpoint: command.type.toLowerCase(), body: {} };
  }

  private mapTeslaVehicle(raw: any): OEMVehicle {
    return {
      id: raw.id_s,
      vin: raw.vin,
      displayName: raw.display_name,
      state: raw.state,
      raw
    };
  }

  private mapTeslaStatus(raw: any): OEMVehicleStatus {
    return {
      vehicleId: raw.id_s,
      state: raw.state,
      raw
    };
  }

  private mapTeslaModel(vehicleType: string): string {
    const models: Record<string, string> = {
      "model3": "Model 3",
      "modely": "Model Y",
      "models": "Model S",
      "modelx": "Model X",
      "cybertruck": "Cybertruck",
      "semi": "Semi"
    };
    return models[vehicleType?.toLowerCase()] || vehicleType;
  }

  private extractYearFromVIN(vin: string): number {
    const yearChar = vin.charAt(9);
    const yearMap: Record<string, number> = {
      "A": 2010, "B": 2011, "C": 2012, "D": 2013, "E": 2014,
      "F": 2015, "G": 2016, "H": 2017, "J": 2018, "K": 2019,
      "L": 2020, "M": 2021, "N": 2022, "P": 2023, "R": 2024,
      "S": 2025
    };
    return yearMap[yearChar] || 2020;
  }

  private extractOpenDoors(vehicleState: any): string[] {
    const doors: string[] = [];
    if (vehicleState?.df) doors.push("FRONT_LEFT");
    if (vehicleState?.pf) doors.push("FRONT_RIGHT");
    if (vehicleState?.dr) doors.push("REAR_LEFT");
    if (vehicleState?.pr) doors.push("REAR_RIGHT");
    return doors;
  }

  private extractOpenWindows(vehicleState: any): string[] {
    const windows: string[] = [];
    if (vehicleState?.fd_window) windows.push("FRONT_LEFT");
    if (vehicleState?.fp_window) windows.push("FRONT_RIGHT");
    if (vehicleState?.rd_window) windows.push("REAR_LEFT");
    if (vehicleState?.rp_window) windows.push("REAR_RIGHT");
    return windows;
  }
}

interface TeslaConfig {
  baseUrl?: string;
  authUrl?: string;
}

interface TeslaCommandMapping {
  endpoint: string;
  body: any;
}
```

---

## 데이터 조화 서비스

```typescript
/**
 * 데이터 조화 서비스
 * 여러 OEM 데이터 소스를 정규화된 WIA 형식으로 변환
 */
class DataHarmonizationService {
  private adapterRegistry: OEMAdapterRegistry;
  private signalMappings: SignalMappingRegistry;
  private unitConverter: UnitConversionService;
  private validator: DataValidator;

  constructor(
    adapterRegistry: OEMAdapterRegistry,
    signalMappings: SignalMappingRegistry
  ) {
    this.adapterRegistry = adapterRegistry;
    this.signalMappings = signalMappings;
    this.unitConverter = new UnitConversionService();
    this.validator = new DataValidator();
  }

  /**
   * OEM 차량 데이터를 WIA 표준으로 조화
   */
  async harmonizeVehicleData(
    oemId: string,
    oemData: OEMVehicleData
  ): Promise<WIAVehicleData> {
    const adapter = this.adapterRegistry.getAdapter(oemId);
    if (!adapter) {
      throw new Error(`OEM에 대한 어댑터를 찾을 수 없음: ${oemId}`);
    }

    // 기본 데이터 정규화
    const normalizedVehicle = adapter.normalizeVehicle(oemData.vehicle);
    const normalizedStatus = adapter.normalizeStatus(oemData.status);
    const normalizedLocation = oemData.location
      ? adapter.normalizeLocation(oemData.location)
      : undefined;

    // WIA 데이터 모델에 매핑
    const wiaData: WIAVehicleData = {
      vehicleId: normalizedVehicle.id,
      vin: normalizedVehicle.vin,
      timestamp: new Date(),
      identity: {
        make: normalizedVehicle.make,
        model: normalizedVehicle.model,
        year: normalizedVehicle.year,
        powertrainType: normalizedVehicle.powertrainType,
        color: normalizedVehicle.color
      },
      status: {
        online: normalizedStatus.online,
        ignition: normalizedStatus.ignition || "OFF",
        locked: normalizedStatus.locked,
        doors: normalizedStatus.doorsOpen || [],
        windows: normalizedStatus.windowsOpen || [],
        trunk: normalizedStatus.trunkOpen,
        hood: normalizedStatus.hoodOpen
      },
      location: normalizedLocation ? {
        latitude: normalizedLocation.latitude,
        longitude: normalizedLocation.longitude,
        altitude: normalizedLocation.altitude,
        heading: normalizedLocation.heading,
        speed: normalizedLocation.speed,
        accuracy: normalizedLocation.accuracy,
        source: normalizedLocation.source,
        timestamp: normalizedLocation.timestamp
      } : undefined,
      energy: this.harmonizeEnergyData(normalizedStatus, normalizedVehicle.powertrainType),
      climate: normalizedStatus.climate ? {
        isOn: normalizedStatus.climate.isOn,
        currentTemp: normalizedStatus.climate.insideTemp,
        targetTemp: normalizedStatus.climate.targetTemp,
        outsideTemp: normalizedStatus.climate.outsideTemp,
        seatHeating: normalizedStatus.climate.seatHeaters
      } : undefined,
      odometer: normalizedStatus.odometer,
      capabilities: normalizedVehicle.capabilities,
      oemSpecific: normalizedVehicle.oemData
    };

    // 데이터 검증
    const validationResult = this.validator.validate(wiaData);
    if (!validationResult.valid) {
      console.warn("데이터 검증 경고:", validationResult.warnings);
    }

    return wiaData;
  }

  /**
   * OEM 특정 에너지 데이터를 WIA 형식으로 조화
   */
  private harmonizeEnergyData(
    status: NormalizedVehicleStatus,
    powertrainType: string
  ): WIAEnergyData {
    const energy: WIAEnergyData = {
      type: this.mapPowertrainToEnergyType(powertrainType)
    };

    if (status.charging) {
      energy.battery = {
        stateOfCharge: status.charging.stateOfCharge,
        stateOfHealth: status.charging.stateOfHealth,
        range: status.charging.range,
        charging: {
          isCharging: status.charging.isCharging,
          isPluggedIn: status.charging.isPluggedIn,
          chargeType: status.charging.chargeType,
          power: status.charging.chargePower,
          rate: status.charging.chargeRate,
          limit: status.charging.chargeLimit,
          timeToFull: status.charging.timeToFull
        }
      };
    }

    if (status.fuel) {
      energy.fuel = {
        level: status.fuel.level,
        range: status.fuel.range,
        consumption: status.fuel.consumption
      };
    }

    return energy;
  }

  private mapPowertrainToEnergyType(powertrain: string): string {
    const mapping: Record<string, string> = {
      "BEV": "BATTERY",
      "ICE": "FUEL",
      "HYBRID": "HYBRID",
      "PHEV": "HYBRID",
      "FCEV": "HYDROGEN"
    };
    return mapping[powertrain] || "UNKNOWN";
  }
}

// 정규화된 데이터 인터페이스
interface NormalizedVehicle {
  id: string;
  vin: string;
  make: string;
  model: string;
  year: number;
  color?: string;
  powertrainType: string;
  capabilities: VehicleFeature[];
  oemData?: Record<string, any>;
}

interface NormalizedVehicleStatus {
  online: boolean;
  ignition?: string;
  locked?: boolean;
  doorsOpen?: string[];
  windowsOpen?: string[];
  trunkOpen?: boolean;
  frunkOpen?: boolean;
  hoodOpen?: boolean;
  charging?: {
    isCharging: boolean;
    isPluggedIn?: boolean;
    stateOfCharge?: number;
    stateOfHealth?: number;
    chargeLimit?: number;
    chargeType?: string;
    chargePower?: number;
    chargeRate?: number;
    timeToFull?: number;
    range?: number;
  };
  fuel?: {
    level: number;
    range: number;
    consumption?: number;
  };
  climate?: {
    isOn: boolean;
    insideTemp?: number;
    outsideTemp?: number;
    targetTemp?: number;
    seatHeaters?: Record<string, number>;
  };
  odometer?: number;
  softwareVersion?: string;
  timestamp: Date;
}

interface NormalizedLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  heading?: number;
  speed?: number;
  accuracy?: number;
  source: string;
  timestamp: Date;
}

// WIA 데이터 모델
interface WIAVehicleData {
  vehicleId: string;
  vin: string;
  timestamp: Date;
  identity: {
    make: string;
    model: string;
    year: number;
    powertrainType: string;
    color?: string;
  };
  status: {
    online: boolean;
    ignition: string;
    locked?: boolean;
    doors: string[];
    windows: string[];
    trunk?: boolean;
    hood?: boolean;
  };
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
    heading?: number;
    speed?: number;
    accuracy?: number;
    source: string;
    timestamp: Date;
  };
  energy: WIAEnergyData;
  climate?: {
    isOn: boolean;
    currentTemp?: number;
    targetTemp?: number;
    outsideTemp?: number;
    seatHeating?: Record<string, number>;
  };
  odometer?: number;
  capabilities: VehicleFeature[];
  oemSpecific?: Record<string, any>;
}

interface WIAEnergyData {
  type: string;
  battery?: {
    stateOfCharge?: number;
    stateOfHealth?: number;
    range?: number;
    charging: {
      isCharging: boolean;
      isPluggedIn?: boolean;
      chargeType?: string;
      power?: number;
      rate?: number;
      limit?: number;
      timeToFull?: number;
    };
  };
  fuel?: {
    level: number;
    range: number;
    consumption?: number;
  };
}
```

---

## OCPI 충전 네트워크 통합

```typescript
/**
 * OCPI (Open Charge Point Interface) 클라이언트
 * 충전 네트워크 통합
 */
class OCPIClient {
  private baseUrl: string;
  private token: string;
  private version: string = "2.2.1";

  constructor(config: OCPIConfig) {
    this.baseUrl = config.baseUrl;
    this.token = config.token;
  }

  /**
   * 근처 위치 검색
   */
  async getLocations(
    latitude: number,
    longitude: number,
    radius: number
  ): Promise<OCPILocation[]> {
    const response = await this.request(
      "GET",
      `/ocpi/cpo/${this.version}/locations`,
      {
        latitude: latitude.toString(),
        longitude: longitude.toString(),
        radius: radius.toString()
      }
    );

    return response.data.map((loc: any) => this.mapLocation(loc));
  }

  /**
   * 특정 위치 조회
   */
  async getLocation(locationId: string): Promise<OCPILocation> {
    const response = await this.request(
      "GET",
      `/ocpi/cpo/${this.version}/locations/${locationId}`
    );

    return this.mapLocation(response.data);
  }

  /**
   * 충전 세션 시작
   */
  async startSession(
    locationId: string,
    evseId: string,
    connectorId: string,
    tokenUid: string
  ): Promise<OCPISession> {
    const response = await this.request(
      "POST",
      `/ocpi/emsp/${this.version}/commands/START_SESSION`,
      undefined,
      {
        response_url: `${process.env.CALLBACK_URL}/ocpi/sessions`,
        token: { uid: tokenUid, type: "RFID" },
        location_id: locationId,
        evse_uid: evseId,
        connector_id: connectorId
      }
    );

    return {
      id: response.data.id,
      status: "PENDING",
      locationId,
      evseId,
      connectorId
    };
  }

  /**
   * 충전 세션 중지
   */
  async stopSession(sessionId: string): Promise<void> {
    await this.request(
      "POST",
      `/ocpi/emsp/${this.version}/commands/STOP_SESSION`,
      undefined,
      {
        response_url: `${process.env.CALLBACK_URL}/ocpi/sessions`,
        session_id: sessionId
      }
    );
  }

  /**
   * 세션 상태 조회
   */
  async getSession(sessionId: string): Promise<OCPISession> {
    const response = await this.request(
      "GET",
      `/ocpi/emsp/${this.version}/sessions/${sessionId}`
    );

    return this.mapSession(response.data);
  }

  /**
   * 실시간 요금 조회
   */
  async getTariff(tariffId: string): Promise<OCPITariff> {
    const response = await this.request(
      "GET",
      `/ocpi/cpo/${this.version}/tariffs/${tariffId}`
    );

    return this.mapTariff(response.data);
  }

  private async request(
    method: string,
    path: string,
    query?: Record<string, string>,
    body?: any
  ): Promise<any> {
    const url = new URL(path, this.baseUrl);
    if (query) {
      Object.entries(query).forEach(([k, v]) => url.searchParams.set(k, v));
    }

    const response = await fetch(url.toString(), {
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
      throw new OCPIError(response.status, await response.text());
    }

    return response.json();
  }

  private mapLocation(data: any): OCPILocation {
    return {
      id: data.id,
      name: data.name,
      address: data.address,
      city: data.city,
      postalCode: data.postal_code,
      country: data.country,
      coordinates: {
        latitude: parseFloat(data.coordinates.latitude),
        longitude: parseFloat(data.coordinates.longitude)
      },
      evses: data.evses?.map((evse: any) => this.mapEVSE(evse)) || [],
      operator: data.operator,
      openingTimes: data.opening_times,
      facilities: data.facilities,
      lastUpdated: new Date(data.last_updated)
    };
  }

  private mapEVSE(data: any): OCPIEVSE {
    return {
      uid: data.uid,
      evseId: data.evse_id,
      status: data.status,
      connectors: data.connectors?.map((c: any) => this.mapConnector(c)) || [],
      coordinates: data.coordinates ? {
        latitude: parseFloat(data.coordinates.latitude),
        longitude: parseFloat(data.coordinates.longitude)
      } : undefined,
      lastUpdated: new Date(data.last_updated)
    };
  }

  private mapConnector(data: any): OCPIConnector {
    return {
      id: data.id,
      standard: data.standard,
      format: data.format,
      powerType: data.power_type,
      maxVoltage: data.max_voltage,
      maxAmperage: data.max_amperage,
      maxElectricPower: data.max_electric_power,
      tariffIds: data.tariff_ids,
      lastUpdated: new Date(data.last_updated)
    };
  }

  private mapSession(data: any): OCPISession {
    return {
      id: data.id,
      status: data.status,
      locationId: data.location_id,
      evseId: data.evse_uid,
      connectorId: data.connector_id,
      startDateTime: new Date(data.start_date_time),
      endDateTime: data.end_date_time ? new Date(data.end_date_time) : undefined,
      kwh: data.kwh,
      authMethod: data.auth_method,
      totalCost: data.total_cost,
      currency: data.currency
    };
  }

  private mapTariff(data: any): OCPITariff {
    return {
      id: data.id,
      currency: data.currency,
      elements: data.elements,
      lastUpdated: new Date(data.last_updated)
    };
  }
}

// OCPI 인터페이스
interface OCPIConfig {
  baseUrl: string;
  token: string;
}

interface OCPILocation {
  id: string;
  name: string;
  address: string;
  city: string;
  postalCode: string;
  country: string;
  coordinates: { latitude: number; longitude: number };
  evses: OCPIEVSE[];
  operator?: any;
  openingTimes?: any;
  facilities?: string[];
  lastUpdated: Date;
}

interface OCPIEVSE {
  uid: string;
  evseId: string;
  status: string;
  connectors: OCPIConnector[];
  coordinates?: { latitude: number; longitude: number };
  lastUpdated: Date;
}

interface OCPIConnector {
  id: string;
  standard: string;
  format: string;
  powerType: string;
  maxVoltage: number;
  maxAmperage: number;
  maxElectricPower: number;
  tariffIds?: string[];
  lastUpdated: Date;
}

interface OCPISession {
  id: string;
  status: string;
  locationId: string;
  evseId: string;
  connectorId: string;
  startDateTime?: Date;
  endDateTime?: Date;
  kwh?: number;
  authMethod?: string;
  totalCost?: number;
  currency?: string;
}

interface OCPITariff {
  id: string;
  currency: string;
  elements: any[];
  lastUpdated: Date;
}

class OCPIError extends Error {
  constructor(public status: number, message: string) {
    super(message);
    this.name = "OCPIError";
  }
}
```

---

## 요약

| 통합 유형 | 프로토콜 | 사용 사례 | 복잡도 |
|----------|---------|----------|--------|
| **OEM API** | REST/OAuth 2.0 | 차량 데이터 및 제어 | 높음 |
| **OCPI** | REST | 충전 네트워크 | 중간 |
| **OCPP** | WebSocket | 충전소 | 높음 |
| **MaaS** | REST/GTFS | 대중교통 통합 | 중간 |
| **보험** | REST/EDR | 사용 기반 보험 | 낮음 |

---

**다음 장:** [제7장: 보안](./07-security.md) - 자동차 사이버보안 및 개인정보 보호 프레임워크.

---

© 2025 World Industry Association (WIA). All rights reserved.
