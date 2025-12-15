/**
 * WIA AAC Eye Tracker Adapter
 * Adapter for eye tracking sensors
 */

import { BaseAdapter } from './BaseAdapter';
import {
  SensorType,
  SensorConfig,
  SensorOptionDescriptor,
  EyeTrackerOptions
} from '../types';

export class EyeTrackerAdapter extends BaseAdapter {
  readonly type: SensorType = 'eye_tracker';
  protected options: EyeTrackerOptions = {
    trackBothEyes: true,
    trackPupil: true,
    trackBlink: true,
    fixationThreshold: 100,
    gazeFilter: 'average',
    dwellTime: 1000,
    sampleRate: 60
  };

  async connect(config: SensorConfig): Promise<void> {
    await super.connect(config);

    // In a real implementation, this would:
    // 1. Connect to the eye tracker hardware/SDK
    // 2. Start the gaze data stream
    // 3. Apply calibration if needed

    console.log('[EyeTrackerAdapter] Connected (mock)');
  }

  async disconnect(): Promise<void> {
    // In a real implementation, stop the data stream
    console.log('[EyeTrackerAdapter] Disconnected');
    await super.disconnect();
  }

  getSupportedOptions(): SensorOptionDescriptor[] {
    return [
      {
        name: 'trackBothEyes',
        type: 'boolean',
        description: 'Track both eyes separately',
        default: true
      },
      {
        name: 'trackPupil',
        type: 'boolean',
        description: 'Track pupil diameter',
        default: true
      },
      {
        name: 'trackBlink',
        type: 'boolean',
        description: 'Detect blinks',
        default: true
      },
      {
        name: 'fixationThreshold',
        type: 'number',
        description: 'Minimum fixation duration in ms',
        default: 100,
        min: 50,
        max: 500
      },
      {
        name: 'gazeFilter',
        type: 'string',
        description: 'Gaze smoothing filter',
        default: 'average',
        options: ['none', 'average', 'kalman']
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
        max: 300
      }
    ];
  }
}
