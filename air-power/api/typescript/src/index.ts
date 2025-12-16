/**
 * WIA-AIR-POWER SDK
 *
 * "충전"이라는 단어가 사라지는 세상
 * 삼촌처럼 힘을 나눠주는 표준
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 *
 * @version 1.0.0
 * @author World Certification Industry Association (WIA)
 */

export * from './types';

import {
  Transmitter,
  Receiver,
  TxId,
  DeviceId,
  DeviceType,
  PowerClass,
  PowerMode,
  Priority,
  Watts,
  Percentage,
  PowerRequest,
  PowerGrant,
  AirPowerEvent,
  AirPowerConfig,
  PowerUsageStats,
  TxBeacon,
  NoTxFoundError,
  InsufficientPowerError,
  GeoLocation,
  TxCapabilities,
  TxStatus,
  RxCapabilities,
  RxStatus,
} from './types';

// ============================================================
// Default Configuration
// ============================================================

const DEFAULT_CONFIG: AirPowerConfig = {
  autoConnect: true,
  preferredMode: 'auto',
  maxReceivePower: 15,  // 스마트폰 기준
  priority: 'normal',
  safetyLevel: 'standard',
  notifications: {
    onChargingStart: true,
    onChargingStop: true,
    onBatteryFull: true,
    onSafetyAlert: true,
  },
};

// ============================================================
// Event Emitter
// ============================================================

type EventHandler = (event: AirPowerEvent) => void;

class EventEmitter {
  private handlers: EventHandler[] = [];

  on(handler: EventHandler): void {
    this.handlers.push(handler);
  }

  off(handler: EventHandler): void {
    this.handlers = this.handlers.filter((h) => h !== handler);
  }

  emit(event: AirPowerEvent): void {
    this.handlers.forEach((handler) => handler(event));
  }
}

// ============================================================
// Simulated TX (for demo)
// ============================================================

class SimulatedTx implements Transmitter {
  id: TxId;
  type: 'home' | 'commercial' | 'public' | 'vehicle';
  name: string;
  location: GeoLocation;
  capabilities: TxCapabilities;
  status: TxStatus;

  constructor(id: string, name: string, type: 'home' | 'commercial' | 'public' | 'vehicle' = 'home') {
    this.id = id;
    this.name = name;
    this.type = type;
    this.location = { latitude: 37.5665, longitude: 126.9780 };
    this.capabilities = {
      maxPower: 30,
      supportedModes: ['rf', 'resonance', 'hybrid'],
      coverageZone: { shape: 'circle', center: this.location, radius: 10 },
      maxDevices: 10,
      frequencies: [2.4e9, 5e9],
      security: 'open',
    };
    this.status = {
      online: true,
      availablePower: 25,
      allocatedPower: 5,
      connectedDevices: 2,
      temperature: 35,
      safetyOk: true,
    };
  }

  allocatePower(deviceId: DeviceId, watts: Watts): PowerGrant {
    const granted = Math.min(watts, this.status.availablePower);
    this.status.allocatedPower += granted;
    this.status.availablePower -= granted;
    this.status.connectedDevices++;

    return {
      requestId: `grant_${Date.now()}`,
      rxId: deviceId,
      txId: this.id,
      grantedPower: granted,
      channel: { mode: 'hybrid', frequency: 2.4e9 },
      startTime: new Date(),
      conditions: {
        maxPower: granted,
        safetyLimits: {
          maxSar: 1.6,
          maxTemperature: 40,
          humanDetectionEnabled: true,
        },
        canPreempt: false,
        canBePreempted: true,
      },
    };
  }

  releasePower(deviceId: DeviceId, watts: Watts): void {
    this.status.allocatedPower -= watts;
    this.status.availablePower += watts;
    this.status.connectedDevices--;
  }
}

// ============================================================
// Power Receiver (RX) Client
// ============================================================

export class AirPowerReceiver {
  private config: AirPowerConfig;
  private events: EventEmitter;
  private nearbyTx: Map<TxId, Transmitter> = new Map();
  private currentGrant: PowerGrant | null = null;
  private batteryLevel: Percentage = 50;
  private isCharging: boolean = false;

  // Device info
  readonly deviceId: DeviceId;
  readonly deviceType: DeviceType;
  readonly powerClass: PowerClass;

  constructor(
    deviceId: string,
    deviceType: DeviceType = 'smartphone',
    config: Partial<AirPowerConfig> = {}
  ) {
    this.deviceId = deviceId;
    this.deviceType = deviceType;
    this.powerClass = this.determinePowerClass(deviceType);
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.events = new EventEmitter();

    // Demo: 시뮬레이션된 TX 추가
    this.simulateNearbyTx();
  }

  private determinePowerClass(type: DeviceType): PowerClass {
    switch (type) {
      case 'iot': return 'class_a';
      case 'wearable':
      case 'earbud': return 'class_b';
      case 'smartphone':
      case 'tablet': return 'class_c';
      case 'laptop': return 'class_d';
      case 'appliance': return 'class_e';
      default: return 'class_c';
    }
  }

  private simulateNearbyTx(): void {
    // 시뮬레이션용 TX 생성
    const homeTx = new SimulatedTx('tx_home_001', '거실 WiFi 라우터', 'home');
    const cafeTx = new SimulatedTx('tx_cafe_001', '스타벅스 강남점', 'commercial');
    const publicTx = new SimulatedTx('tx_public_001', '지하철 역사', 'public');

    this.nearbyTx.set(homeTx.id, homeTx);
    this.nearbyTx.set(cafeTx.id, cafeTx);
    this.nearbyTx.set(publicTx.id, publicTx);
  }

  // ============================================================
  // Discovery
  // ============================================================

  /**
   * 주변 TX 스캔
   */
  async scan(): Promise<Transmitter[]> {
    // 실제로는 RF 스캔 수행
    return Array.from(this.nearbyTx.values()).filter(
      (tx) => tx.status.online && tx.status.safetyOk
    );
  }

  /**
   * 가장 좋은 TX 찾기
   */
  async findBestTx(): Promise<Transmitter | null> {
    const available = await this.scan();
    if (available.length === 0) return null;

    // 가용 전력이 가장 많은 TX 선택
    return available.reduce((best, tx) =>
      tx.status.availablePower > best.status.availablePower ? tx : best
    );
  }

  // ============================================================
  // Charging
  // ============================================================

  /**
   * 자동 충전 시작
   */
  async startCharging(): Promise<PowerGrant> {
    const tx = await this.findBestTx();
    if (!tx) {
      throw new NoTxFoundError();
    }

    return this.requestPowerFrom(tx.id);
  }

  /**
   * 특정 TX에서 전력 요청
   */
  async requestPowerFrom(txId: TxId, watts?: Watts): Promise<PowerGrant> {
    const tx = this.nearbyTx.get(txId);
    if (!tx) {
      throw new NoTxFoundError();
    }

    const requestedWatts = watts || this.getRecommendedPower();

    if (tx.status.availablePower < requestedWatts) {
      throw new InsufficientPowerError(requestedWatts, tx.status.availablePower);
    }

    // 전력 요청 (시뮬레이션)
    const simTx = tx as SimulatedTx;
    const grant = simTx.allocatePower(this.deviceId, requestedWatts);

    this.currentGrant = grant;
    this.isCharging = true;

    // 이벤트 발생
    this.events.emit({
      type: 'charging_started',
      rxId: this.deviceId,
      txId: txId,
      power: grant.grantedPower,
      timestamp: new Date(),
    });

    // 충전 시뮬레이션 시작
    this.simulateCharging(grant.grantedPower);

    return grant;
  }

  /**
   * 충전 중지
   */
  stopCharging(): void {
    if (!this.currentGrant) return;

    const txId = this.currentGrant.txId;
    const tx = this.nearbyTx.get(txId) as SimulatedTx;
    if (tx) {
      tx.releasePower(this.deviceId, this.currentGrant.grantedPower);
    }

    this.isCharging = false;
    this.currentGrant = null;

    this.events.emit({
      type: 'charging_stopped',
      rxId: this.deviceId,
      reason: 'manual',
      timestamp: new Date(),
    });
  }

  private getRecommendedPower(): Watts {
    switch (this.powerClass) {
      case 'class_a': return 0.05;
      case 'class_b': return 1;
      case 'class_c': return 10;
      case 'class_d': return 30;
      case 'class_e': return 100;
      default: return 10;
    }
  }

  private simulateCharging(power: Watts): void {
    // 간단한 충전 시뮬레이션
    const interval = setInterval(() => {
      if (!this.isCharging) {
        clearInterval(interval);
        return;
      }

      // 배터리 증가 (1분에 약 1% 가정)
      this.batteryLevel = Math.min(100, this.batteryLevel + 0.5);

      if (this.batteryLevel >= 100) {
        clearInterval(interval);
        this.events.emit({
          type: 'battery_full',
          rxId: this.deviceId,
          timestamp: new Date(),
        });
      }
    }, 3000); // 3초마다 업데이트 (데모용)
  }

  // ============================================================
  // Status
  // ============================================================

  /**
   * 현재 상태
   */
  getStatus(): RxStatus {
    return {
      batteryLevel: this.batteryLevel,
      isCharging: this.isCharging,
      currentPower: this.currentGrant?.grantedPower || 0,
      connectedTx: this.currentGrant?.txId || null,
      temperature: 25,
    };
  }

  /**
   * 배터리 레벨
   */
  getBatteryLevel(): Percentage {
    return this.batteryLevel;
  }

  /**
   * 충전 중 여부
   */
  isCurrentlyCharging(): boolean {
    return this.isCharging;
  }

  // ============================================================
  // Events
  // ============================================================

  /**
   * 이벤트 구독
   */
  onEvent(handler: (event: AirPowerEvent) => void): void {
    this.events.on(handler);
  }

  /**
   * 이벤트 구독 해제
   */
  offEvent(handler: (event: AirPowerEvent) => void): void {
    this.events.off(handler);
  }

  // ============================================================
  // Configuration
  // ============================================================

  /**
   * 설정 변경
   */
  configure(config: Partial<AirPowerConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * 현재 설정
   */
  getConfig(): AirPowerConfig {
    return { ...this.config };
  }
}

// ============================================================
// Convenience Functions
// ============================================================

let defaultReceiver: AirPowerReceiver | null = null;

/**
 * 기본 수신기 생성/가져오기
 */
export function getReceiver(
  deviceId?: string,
  deviceType?: DeviceType
): AirPowerReceiver {
  if (!defaultReceiver) {
    defaultReceiver = new AirPowerReceiver(
      deviceId || `device_${Date.now()}`,
      deviceType || 'smartphone'
    );
  }
  return defaultReceiver;
}

/**
 * 간단한 충전 시작
 */
export async function startCharging(): Promise<PowerGrant> {
  const receiver = getReceiver();
  return receiver.startCharging();
}

/**
 * 충전 중지
 */
export function stopCharging(): void {
  if (defaultReceiver) {
    defaultReceiver.stopCharging();
  }
}

/**
 * 배터리 상태
 */
export function getBatteryStatus(): { level: Percentage; charging: boolean } {
  const receiver = getReceiver();
  return {
    level: receiver.getBatteryLevel(),
    charging: receiver.isCurrentlyCharging(),
  };
}

/**
 * 주변 TX 찾기
 */
export async function findNearbyPowerSources(): Promise<Transmitter[]> {
  const receiver = getReceiver();
  return receiver.scan();
}

// ============================================================
// Version & Export
// ============================================================

export const VERSION = '1.0.0';
export const WIA_AIR_POWER_VERSION = '1.0';

export default AirPowerReceiver;
