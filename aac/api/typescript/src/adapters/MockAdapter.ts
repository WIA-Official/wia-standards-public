/**
 * WIA AAC Mock Adapter
 * For testing and development purposes
 */

import { BaseAdapter, SignalHandler } from './BaseAdapter';
import {
  WiaAacSignal,
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  EyeTrackerData,
  SwitchData,
  MuscleSensorData,
  BrainInterfaceData,
  BreathData,
  HeadMovementData
} from '../types';

export interface MockAdapterOptions {
  type: SensorType;
  simulateSignals?: boolean;
  signalInterval?: number;
}

export class MockAdapter extends BaseAdapter {
  readonly type: SensorType;
  private intervalId: NodeJS.Timeout | null = null;
  private mockOptions: MockAdapterOptions;

  constructor(options: MockAdapterOptions) {
    super();
    this.type = options.type;
    this.mockOptions = {
      simulateSignals: false,
      signalInterval: 100,
      ...options
    };
  }

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);

    if (this.mockOptions.simulateSignals) {
      this.startSimulation();
    }
  }

  async disconnect(): Promise<void> {
    this.stopSimulation();
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'sampleRate',
        type: 'number',
        description: 'Sample rate in Hz',
        default: 60,
        min: 1,
        max: 1000
      },
      {
        name: 'dwellTime',
        type: 'number',
        description: 'Dwell time for selection in ms',
        default: 1000,
        min: 100,
        max: 5000
      }
    ];
  }

  /**
   * Manually emit a mock signal
   */
  emitMockSignal(data?: Partial<WiaAacSignal['data']>): void {
    const signal = this.generateSignal(data);
    this.emitSignal(signal);
  }

  private startSimulation(): void {
    this.intervalId = setInterval(() => {
      const signal = this.generateSignal();
      this.emitSignal(signal);
    }, this.mockOptions.signalInterval);
  }

  private stopSimulation(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
  }

  private generateSignal(overrideData?: Partial<WiaAacSignal['data']>): WiaAacSignal {
    let data: WiaAacSignal['data'];

    switch (this.type) {
      case 'eye_tracker':
        data = this.generateEyeTrackerData();
        break;
      case 'switch':
        data = this.generateSwitchData();
        break;
      case 'muscle_sensor':
        data = this.generateMuscleSensorData();
        break;
      case 'brain_interface':
        data = this.generateBrainInterfaceData();
        break;
      case 'breath':
        data = this.generateBreathData();
        break;
      case 'head_movement':
        data = this.generateHeadMovementData();
        break;
      default:
        data = { custom_type: 'mock', custom_data: {} };
    }

    if (overrideData) {
      data = { ...data, ...overrideData } as typeof data;
    }

    return this.createSignal(this.type, data as WiaAacSignal['data']);
  }

  private generateEyeTrackerData(): EyeTrackerData {
    return {
      gaze: {
        x: Math.random(),
        y: Math.random(),
        z: null
      },
      fixation: {
        active: Math.random() > 0.7,
        duration_ms: Math.floor(Math.random() * 2000),
        target_id: 'mock_target'
      },
      pupil: {
        left_diameter_mm: 3 + Math.random(),
        right_diameter_mm: 3 + Math.random()
      },
      blink: {
        detected: Math.random() > 0.95,
        duration_ms: 0
      },
      eye_validity: {
        left: true,
        right: true
      }
    };
  }

  private generateSwitchData(): SwitchData {
    return {
      switch_id: 1,
      channel: 'primary',
      state: 'released',
      duration_ms: 0,
      pressure: null,
      repeat_count: 1
    };
  }

  private generateMuscleSensorData(): MuscleSensorData {
    return {
      channel_id: 1,
      muscle_group: 'cheek_left',
      activation: Math.random(),
      raw_uv: Math.random() * 200,
      envelope_uv: Math.random() * 100,
      threshold_exceeded: Math.random() > 0.8,
      gesture: 'none'
    };
  }

  private generateBrainInterfaceData(): BrainInterfaceData {
    return {
      channel_count: 8,
      sample_rate_hz: 250,
      channels: [
        { id: 'Fp1', value_uv: Math.random() * 50 - 25 },
        { id: 'Fp2', value_uv: Math.random() * 50 - 25 },
        { id: 'C3', value_uv: Math.random() * 50 - 25 },
        { id: 'C4', value_uv: Math.random() * 50 - 25 },
        { id: 'P3', value_uv: Math.random() * 50 - 25 },
        { id: 'P4', value_uv: Math.random() * 50 - 25 },
        { id: 'O1', value_uv: Math.random() * 50 - 25 },
        { id: 'O2', value_uv: Math.random() * 50 - 25 }
      ],
      bands: {
        delta: 0.2,
        theta: 0.2,
        alpha: 0.3,
        beta: 0.2,
        gamma: 0.1
      },
      classification: {
        intent: 'rest',
        confidence: 0.7
      }
    };
  }

  private generateBreathData(): BreathData {
    return {
      action: 'neutral',
      pressure_kpa: 101.3,
      pressure_normalized: 0.5,
      duration_ms: 0,
      intensity: 'soft',
      baseline_kpa: 101.3
    };
  }

  private generateHeadMovementData(): HeadMovementData {
    return {
      position: {
        x: 0.5 + (Math.random() - 0.5) * 0.1,
        y: 0.5 + (Math.random() - 0.5) * 0.1
      },
      rotation: {
        pitch: Math.random() * 10 - 5,
        yaw: Math.random() * 10 - 5,
        roll: Math.random() * 5 - 2.5
      },
      velocity: {
        x: 0,
        y: 0
      },
      gesture: 'none',
      dwell_time_ms: 0,
      face_detected: true
    };
  }
}
