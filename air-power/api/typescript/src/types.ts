/**
 * WIA-AIR-POWER Type Definitions
 *
 * 삼촌처럼 힘을 나눠주는 표준
 */

// ============================================================
// Basic Types
// ============================================================

export type Watts = number;
export type Percentage = number;
export type DeviceId = string;
export type TxId = string;

// ============================================================
// Device Types
// ============================================================

export type DeviceType =
  | 'smartphone'
  | 'tablet'
  | 'laptop'
  | 'wearable'
  | 'earbud'
  | 'iot'
  | 'appliance';

export type PowerClass =
  | 'class_a'   // 0-100mW (센서, 태그)
  | 'class_b'   // 100mW-2W (웨어러블)
  | 'class_c'   // 2W-15W (스마트폰)
  | 'class_d'   // 15W-100W (노트북)
  | 'class_e';  // 100W+ (가전)

export type TxType =
  | 'home'
  | 'commercial'
  | 'public'
  | 'vehicle';

export type PowerMode =
  | 'rf'
  | 'resonance'
  | 'infrared'
  | 'hybrid';

export type Priority =
  | 'critical'
  | 'high'
  | 'normal'
  | 'low';

// ============================================================
// Transmitter Types
// ============================================================

export interface Transmitter {
  id: TxId;
  type: TxType;
  name: string;
  location: GeoLocation;
  capabilities: TxCapabilities;
  status: TxStatus;
}

export interface TxCapabilities {
  maxPower: Watts;
  supportedModes: PowerMode[];
  coverageZone: CoverageZone;
  maxDevices: number;
  frequencies: number[];
  security: 'open' | 'authenticated' | 'private';
}

export interface TxStatus {
  online: boolean;
  availablePower: Watts;
  allocatedPower: Watts;
  connectedDevices: number;
  temperature: number;
  safetyOk: boolean;
}

export interface CoverageZone {
  shape: 'circle' | 'polygon';
  center?: GeoLocation;
  radius?: number;  // meters
  points?: GeoLocation[];
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

// ============================================================
// Receiver Types
// ============================================================

export interface Receiver {
  id: DeviceId;
  type: DeviceType;
  name: string;
  powerClass: PowerClass;
  capabilities: RxCapabilities;
  status: RxStatus;
}

export interface RxCapabilities {
  maxReceivePower: Watts;
  supportedModes: PowerMode[];
  batteryCapacity: number;  // mAh
  supportedFrequencies: number[];
}

export interface RxStatus {
  batteryLevel: Percentage;
  isCharging: boolean;
  currentPower: Watts;
  connectedTx: TxId | null;
  temperature: number;
}

// ============================================================
// Power Transfer Types
// ============================================================

export interface PowerRequest {
  rxId: DeviceId;
  txId: TxId;
  requestedPower: Watts;
  priority: Priority;
  duration: number | 'continuous';
  timestamp: Date;
}

export interface PowerGrant {
  requestId: string;
  rxId: DeviceId;
  txId: TxId;
  grantedPower: Watts;
  channel: PowerChannel;
  startTime: Date;
  expiresAt?: Date;
  conditions: PowerConditions;
}

export interface PowerChannel {
  mode: PowerMode;
  frequency?: number;
  beamId?: string;
}

export interface PowerConditions {
  maxPower: Watts;
  safetyLimits: SafetyLimits;
  canPreempt: boolean;
  canBePreempted: boolean;
}

export interface SafetyLimits {
  maxSar: number;
  maxTemperature: number;
  humanDetectionEnabled: boolean;
}

// ============================================================
// Discovery Types
// ============================================================

export interface TxBeacon {
  txId: TxId;
  txType: TxType;
  availablePower: Watts;
  supportedModes: PowerMode[];
  coverage: CoverageZone;
  security: string;
  timestamp: Date;
}

export interface RxAnnouncement {
  rxId: DeviceId;
  rxType: DeviceType;
  powerNeeded: Watts;
  batteryLevel: Percentage;
  priority: Priority;
  timestamp: Date;
}

// ============================================================
// Event Types
// ============================================================

export type AirPowerEvent =
  | ChargingStartedEvent
  | ChargingStoppedEvent
  | BatteryFullEvent
  | PowerChangedEvent
  | SafetyAlertEvent
  | TxDiscoveredEvent
  | TxLostEvent;

export interface ChargingStartedEvent {
  type: 'charging_started';
  rxId: DeviceId;
  txId: TxId;
  power: Watts;
  timestamp: Date;
}

export interface ChargingStoppedEvent {
  type: 'charging_stopped';
  rxId: DeviceId;
  reason: 'full' | 'disconnected' | 'safety' | 'preempted' | 'manual';
  timestamp: Date;
}

export interface BatteryFullEvent {
  type: 'battery_full';
  rxId: DeviceId;
  timestamp: Date;
}

export interface PowerChangedEvent {
  type: 'power_changed';
  rxId: DeviceId;
  oldPower: Watts;
  newPower: Watts;
  reason: string;
  timestamp: Date;
}

export interface SafetyAlertEvent {
  type: 'safety_alert';
  alertType: 'human_detected' | 'overheat' | 'interference' | 'sar_limit';
  txId?: TxId;
  rxId?: DeviceId;
  action: string;
  timestamp: Date;
}

export interface TxDiscoveredEvent {
  type: 'tx_discovered';
  tx: Transmitter;
  timestamp: Date;
}

export interface TxLostEvent {
  type: 'tx_lost';
  txId: TxId;
  timestamp: Date;
}

// ============================================================
// Configuration Types
// ============================================================

export interface AirPowerConfig {
  autoConnect: boolean;
  preferredMode: PowerMode | 'auto';
  maxReceivePower: Watts;
  priority: Priority;
  safetyLevel: 'standard' | 'strict' | 'permissive';
  notifications: NotificationConfig;
}

export interface NotificationConfig {
  onChargingStart: boolean;
  onChargingStop: boolean;
  onBatteryFull: boolean;
  onSafetyAlert: boolean;
}

// ============================================================
// Analytics Types
// ============================================================

export interface PowerUsageStats {
  deviceId: DeviceId;
  period: 'day' | 'week' | 'month';
  totalEnergyReceived: number;  // Wh
  averagePower: Watts;
  chargingSessions: number;
  topTxUsed: TxId[];
  savings: {
    cablesReplaced: number;
    timeSaved: number;  // minutes
  };
}

export interface NetworkHealth {
  region: string;
  txCount: number;
  txOnline: number;
  totalCapacity: Watts;
  currentLoad: Watts;
  efficiency: Percentage;
  safetyIncidents: number;
}

// ============================================================
// Error Types
// ============================================================

export class AirPowerError extends Error {
  constructor(
    message: string,
    public code: string,
    public recoverable: boolean = true
  ) {
    super(message);
    this.name = 'AirPowerError';
  }
}

export class NoTxFoundError extends AirPowerError {
  constructor() {
    super(
      '주변에 전력 송신기가 없어요',
      'NO_TX_FOUND',
      true
    );
  }
}

export class InsufficientPowerError extends AirPowerError {
  constructor(requested: Watts, available: Watts) {
    super(
      `요청한 전력(${requested}W)보다 가용 전력(${available}W)이 부족해요`,
      'INSUFFICIENT_POWER',
      true
    );
  }
}

export class SafetyViolationError extends AirPowerError {
  constructor(reason: string) {
    super(
      `안전을 위해 충전을 중단했어요: ${reason}`,
      'SAFETY_VIOLATION',
      false
    );
  }
}
