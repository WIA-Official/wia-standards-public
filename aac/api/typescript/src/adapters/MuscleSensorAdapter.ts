/**
 * WIA AAC Muscle Sensor Adapter
 * Adapter for EMG/muscle sensors
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  MuscleSensorOptions
} from '../types';

export class MuscleSensorAdapter extends BaseAdapter {
  readonly type: SensorType = 'muscle_sensor';
  protected options: MuscleSensorOptions = {
    activationThreshold: 0.5,
    gestureRecognition: true,
    channels: [1],
    sampleRate: 100
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);
    console.log('[MuscleSensorAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    console.log('[MuscleSensorAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'activationThreshold',
        type: 'number',
        description: 'Activation threshold (0.0-1.0)',
        default: 0.5,
        min: 0.1,
        max: 0.9
      },
      {
        name: 'gestureRecognition',
        type: 'boolean',
        description: 'Enable gesture recognition',
        default: true
      },
      {
        name: 'channels',
        type: 'array',
        description: 'Active channel IDs'
      },
      {
        name: 'sampleRate',
        type: 'number',
        description: 'Sample rate in Hz',
        default: 100,
        min: 50,
        max: 1000
      }
    ];
  }
}
