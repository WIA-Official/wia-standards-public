/**
 * WIA AAC Breath Adapter
 * Adapter for sip-and-puff devices
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  BreathOptions
} from '../types';

export class BreathAdapter extends BaseAdapter {
  readonly type: SensorType = 'breath';
  protected options: BreathOptions = {
    sipThreshold: 0.3,
    puffThreshold: 0.3,
    hardThreshold: 0.7,
    sampleRate: 50
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);
    console.log('[BreathAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    console.log('[BreathAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'sipThreshold',
        type: 'number',
        description: 'Sip detection threshold (kPa)',
        default: 0.3,
        min: 0.1,
        max: 1.0
      },
      {
        name: 'puffThreshold',
        type: 'number',
        description: 'Puff detection threshold (kPa)',
        default: 0.3,
        min: 0.1,
        max: 1.0
      },
      {
        name: 'hardThreshold',
        type: 'number',
        description: 'Hard action threshold (kPa)',
        default: 0.7,
        min: 0.3,
        max: 2.0
      },
      {
        name: 'sampleRate',
        type: 'number',
        description: 'Sample rate in Hz',
        default: 50,
        min: 20,
        max: 200
      }
    ];
  }
}
