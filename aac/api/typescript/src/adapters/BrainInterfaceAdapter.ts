/**
 * WIA AAC Brain Interface Adapter
 * Adapter for EEG/BCI devices
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  BrainInterfaceOptions
} from '../types';

export class BrainInterfaceAdapter extends BaseAdapter {
  readonly type: SensorType = 'brain_interface';
  protected options: BrainInterfaceOptions = {
    channels: ['Fp1', 'Fp2', 'C3', 'C4', 'P3', 'P4', 'O1', 'O2'],
    bandPassFilter: { low: 1, high: 50 },
    artifactRejection: true,
    sampleRate: 250
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);
    console.log('[BrainInterfaceAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    console.log('[BrainInterfaceAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'channels',
        type: 'array',
        description: 'EEG channels to record (10-20 system)'
      },
      {
        name: 'sampleRate',
        type: 'number',
        description: 'Sample rate in Hz',
        default: 250,
        min: 125,
        max: 1000
      },
      {
        name: 'artifactRejection',
        type: 'boolean',
        description: 'Enable artifact rejection',
        default: true
      },
      {
        name: 'classificationModel',
        type: 'string',
        description: 'BCI classification model name'
      }
    ];
  }
}
