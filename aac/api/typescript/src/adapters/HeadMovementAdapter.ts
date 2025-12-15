/**
 * WIA AAC Head Movement Adapter
 * Adapter for head tracking devices
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  HeadMovementOptions
} from '../types';

export class HeadMovementAdapter extends BaseAdapter {
  readonly type: SensorType = 'head_movement';
  protected options: HeadMovementOptions = {
    trackRotation: true,
    gestureRecognition: true,
    dwellRadius: 0.05,
    dwellTime: 1000,
    sampleRate: 60
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);
    console.log('[HeadMovementAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    console.log('[HeadMovementAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'trackRotation',
        type: 'boolean',
        description: 'Track head rotation (pitch/yaw/roll)',
        default: true
      },
      {
        name: 'gestureRecognition',
        type: 'boolean',
        description: 'Enable gesture recognition (nod/shake)',
        default: true
      },
      {
        name: 'dwellRadius',
        type: 'number',
        description: 'Dwell detection radius (normalized)',
        default: 0.05,
        min: 0.01,
        max: 0.2
      },
      {
        name: 'dwellTime',
        type: 'number',
        description: 'Dwell time for selection in ms',
        default: 1000,
        min: 200,
        max: 5000
      },
      {
        name: 'sampleRate',
        type: 'number',
        description: 'Sample rate in Hz',
        default: 60,
        min: 30,
        max: 120
      }
    ];
  }
}
