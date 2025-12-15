/**
 * WIA AAC Switch Adapter
 * Adapter for switch/button input devices
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  SwitchOptions
} from '../types';

export class SwitchAdapter extends BaseAdapter {
  readonly type: SensorType = 'switch';
  protected options: SwitchOptions = {
    debounceTime: 50,
    holdThreshold: 500,
    multiPressWindow: 300
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);
    console.log('[SwitchAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    console.log('[SwitchAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'debounceTime',
        type: 'number',
        description: 'Debounce time in ms',
        default: 50,
        min: 10,
        max: 200
      },
      {
        name: 'holdThreshold',
        type: 'number',
        description: 'Time before "held" state in ms',
        default: 500,
        min: 100,
        max: 2000
      },
      {
        name: 'multiPressWindow',
        type: 'number',
        description: 'Window for detecting multi-press in ms',
        default: 300,
        min: 100,
        max: 1000
      }
    ];
  }
}
