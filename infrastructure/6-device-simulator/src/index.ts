/**
 * WIA Device Simulator
 *
 * WIA 표준 디바이스 시뮬레이터
 * - Voice: 스마트 스피커, 음성 어시스턴트
 * - Auto: 자율주행차, V2X 통신
 * - CI: 인공와우, 보청기
 * - Home: 스마트홈 디바이스
 * - Quantum: 양자 프로세서
 * - Medical: 의료 기기
 * - Climate: 환경 센서
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 */

import { EventEmitter } from 'events';

// ============================================
// 디바이스 타입
// ============================================

export type DeviceType =
  | 'voice-speaker'
  | 'voice-assistant'
  | 'auto-vehicle'
  | 'auto-v2x'
  | 'ci-implant'
  | 'ci-processor'
  | 'home-hub'
  | 'home-sensor'
  | 'quantum-processor'
  | 'quantum-network'
  | 'medical-monitor'
  | 'medical-implant'
  | 'climate-sensor'
  | 'climate-station';

// ============================================
// 디바이스 상태
// ============================================

export interface DeviceState {
  id: string;
  type: DeviceType;
  status: 'online' | 'offline' | 'error' | 'standby';
  battery?: number;
  signal?: number;
  lastUpdate: string;
  data: Record<string, unknown>;
}

// ============================================
// 시뮬레이터 설정
// ============================================

export interface SimulatorConfig {
  deviceType: DeviceType;
  deviceId?: string;
  updateInterval?: number; // ms
  simulateErrors?: boolean;
  errorRate?: number; // 0-1
}

// ============================================
// Base Device Simulator
// ============================================

export abstract class DeviceSimulator extends EventEmitter {
  protected state: DeviceState;
  protected config: SimulatorConfig;
  protected intervalId?: NodeJS.Timeout;

  constructor(config: SimulatorConfig) {
    super();
    this.config = {
      updateInterval: 1000,
      simulateErrors: false,
      errorRate: 0.01,
      ...config,
    };

    this.state = {
      id: config.deviceId || `${config.deviceType}-${Date.now()}`,
      type: config.deviceType,
      status: 'offline',
      lastUpdate: new Date().toISOString(),
      data: {},
    };
  }

  /**
   * 시뮬레이션 시작
   */
  start(): void {
    this.state.status = 'online';
    this.emit('start', this.state);

    this.intervalId = setInterval(() => {
      this.update();
    }, this.config.updateInterval);
  }

  /**
   * 시뮬레이션 중지
   */
  stop(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
    }
    this.state.status = 'offline';
    this.emit('stop', this.state);
  }

  /**
   * 상태 업데이트 (서브클래스에서 구현)
   */
  protected abstract update(): void;

  /**
   * 현재 상태 반환
   */
  getState(): DeviceState {
    return { ...this.state };
  }

  /**
   * 에러 시뮬레이션
   */
  protected maybeError(): boolean {
    if (this.config.simulateErrors && Math.random() < (this.config.errorRate || 0)) {
      this.state.status = 'error';
      this.emit('error', { device: this.state, message: 'Simulated error' });
      return true;
    }
    return false;
  }
}

// ============================================
// Voice Device Simulator
// ============================================

export class VoiceDeviceSimulator extends DeviceSimulator {
  constructor(config: Omit<SimulatorConfig, 'deviceType'>) {
    super({ ...config, deviceType: 'voice-speaker' });
  }

  protected update(): void {
    if (this.maybeError()) return;

    this.state.lastUpdate = new Date().toISOString();
    this.state.data = {
      volume: Math.floor(Math.random() * 100),
      isListening: Math.random() > 0.7,
      lastCommand: this.generateVoiceCommand(),
      wakeWordDetected: Math.random() > 0.9,
    };

    this.emit('update', this.state);
  }

  private generateVoiceCommand(): string {
    const commands = [
      '음악 틀어줘',
      '날씨 알려줘',
      '불 꺼줘',
      '에어컨 켜줘',
      '알람 맞춰줘',
    ];
    return commands[Math.floor(Math.random() * commands.length)];
  }
}

// ============================================
// Auto (Vehicle) Simulator
// ============================================

export class AutoVehicleSimulator extends DeviceSimulator {
  private latitude: number = 37.5665;
  private longitude: number = 126.978;

  constructor(config: Omit<SimulatorConfig, 'deviceType'>) {
    super({ ...config, deviceType: 'auto-vehicle' });
  }

  protected update(): void {
    if (this.maybeError()) return;

    // 위치 이동 시뮬레이션
    this.latitude += (Math.random() - 0.5) * 0.001;
    this.longitude += (Math.random() - 0.5) * 0.001;

    this.state.lastUpdate = new Date().toISOString();
    this.state.battery = Math.max(0, (this.state.battery || 100) - Math.random() * 0.1);
    this.state.data = {
      speed: Math.floor(Math.random() * 120),
      location: { latitude: this.latitude, longitude: this.longitude },
      autonomyLevel: 4,
      sensors: {
        lidar: 'active',
        camera: 'active',
        radar: 'active',
      },
      nearbyVehicles: Math.floor(Math.random() * 10),
    };

    this.emit('update', this.state);
  }
}

// ============================================
// CI (Cochlear Implant) Simulator
// ============================================

export class CIDeviceSimulator extends DeviceSimulator {
  constructor(config: Omit<SimulatorConfig, 'deviceType'>) {
    super({ ...config, deviceType: 'ci-processor' });
  }

  protected update(): void {
    if (this.maybeError()) return;

    this.state.lastUpdate = new Date().toISOString();
    this.state.battery = Math.max(0, (this.state.battery || 100) - Math.random() * 0.05);
    this.state.signal = 80 + Math.random() * 20;
    this.state.data = {
      // 옥타브 감지 (WIA-CI)
      currentOctave: Math.floor(Math.random() * 7) + 1,
      frequency: 100 + Math.random() * 8000,
      // TFS 인코딩
      tfsEnabled: true,
      electrodeChannels: 22,
      activeChannels: Math.floor(Math.random() * 22) + 1,
      // 음악 모드
      musicMode: Math.random() > 0.5,
      speechEnhancement: true,
    };

    this.emit('update', this.state);
  }
}

// ============================================
// Home Device Simulator
// ============================================

export class HomeDeviceSimulator extends DeviceSimulator {
  constructor(config: Omit<SimulatorConfig, 'deviceType'>) {
    super({ ...config, deviceType: 'home-hub' });
  }

  protected update(): void {
    if (this.maybeError()) return;

    this.state.lastUpdate = new Date().toISOString();
    this.state.data = {
      connectedDevices: Math.floor(Math.random() * 20) + 5,
      temperature: 20 + Math.random() * 10,
      humidity: 40 + Math.random() * 30,
      lightLevel: Math.floor(Math.random() * 100),
      securityStatus: 'armed',
      energyUsage: Math.random() * 5000, // watts
    };

    this.emit('update', this.state);
  }
}

// ============================================
// Climate Sensor Simulator
// ============================================

export class ClimateSensorSimulator extends DeviceSimulator {
  constructor(config: Omit<SimulatorConfig, 'deviceType'>) {
    super({ ...config, deviceType: 'climate-sensor' });
  }

  protected update(): void {
    if (this.maybeError()) return;

    this.state.lastUpdate = new Date().toISOString();
    this.state.data = {
      co2Level: 400 + Math.random() * 100, // ppm
      temperature: 15 + Math.random() * 20, // celsius
      humidity: 30 + Math.random() * 50, // percent
      airQualityIndex: Math.floor(Math.random() * 300),
      pm25: Math.random() * 100, // µg/m³
      pm10: Math.random() * 150, // µg/m³
    };

    this.emit('update', this.state);
  }
}

// ============================================
// Simulator Factory
// ============================================

export function createSimulator(
  type: DeviceType,
  config?: Partial<SimulatorConfig>
): DeviceSimulator {
  const baseConfig = { deviceId: `${type}-${Date.now()}`, ...config };

  switch (type) {
    case 'voice-speaker':
    case 'voice-assistant':
      return new VoiceDeviceSimulator(baseConfig);
    case 'auto-vehicle':
    case 'auto-v2x':
      return new AutoVehicleSimulator(baseConfig);
    case 'ci-implant':
    case 'ci-processor':
      return new CIDeviceSimulator(baseConfig);
    case 'home-hub':
    case 'home-sensor':
      return new HomeDeviceSimulator(baseConfig);
    case 'climate-sensor':
    case 'climate-station':
      return new ClimateSensorSimulator(baseConfig);
    default:
      throw new Error(`Unknown device type: ${type}`);
  }
}

// ============================================
// Multi-Device Simulator
// ============================================

export class MultiDeviceSimulator extends EventEmitter {
  private simulators: Map<string, DeviceSimulator> = new Map();

  /**
   * 디바이스 추가
   */
  addDevice(type: DeviceType, id?: string): DeviceSimulator {
    const simulator = createSimulator(type, { deviceId: id });
    this.simulators.set(simulator.getState().id, simulator);

    simulator.on('update', (state) => {
      this.emit('deviceUpdate', state);
    });

    simulator.on('error', (error) => {
      this.emit('deviceError', error);
    });

    return simulator;
  }

  /**
   * 모든 디바이스 시작
   */
  startAll(): void {
    this.simulators.forEach((sim) => sim.start());
    this.emit('allStarted');
  }

  /**
   * 모든 디바이스 중지
   */
  stopAll(): void {
    this.simulators.forEach((sim) => sim.stop());
    this.emit('allStopped');
  }

  /**
   * 모든 디바이스 상태
   */
  getAllStates(): DeviceState[] {
    return Array.from(this.simulators.values()).map((sim) => sim.getState());
  }
}

// ============================================
// Export
// ============================================

export default {
  createSimulator,
  VoiceDeviceSimulator,
  AutoVehicleSimulator,
  CIDeviceSimulator,
  HomeDeviceSimulator,
  ClimateSensorSimulator,
  MultiDeviceSimulator,
};

// 홍익인간 (弘益人間)
// 인류를 널리 이롭게 하라
// Benefit All Humanity
