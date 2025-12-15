/**
 * WIA AAC Standard API
 * TypeScript Implementation
 *
 * @packageDocumentation
 * @module wia-aac
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

// Core
export { WiaAac } from './core/WiaAac';
export { WiaAacEventEmitter } from './core/EventEmitter';
export { SignalValidator, getSignalValidator } from './core/SignalValidator';

// Types
export * from './types';

// Adapters
export {
  BaseAdapter,
  ISensorAdapter,
  SignalHandler,
  MockAdapter,
  MockAdapterOptions,
  EyeTrackerAdapter,
  SwitchAdapter,
  MuscleSensorAdapter,
  BrainInterfaceAdapter,
  BreathAdapter,
  HeadMovementAdapter
} from './adapters';

// Protocol (Phase 3)
export * from './protocol';

// Transport (Phase 3)
export * from './transport';

// Output (Phase 4)
export * from './output';

// Version
export const VERSION = '1.0.0';

// SensorType enum-like object for convenience
export const SensorType = {
  EYE_TRACKER: 'eye_tracker' as const,
  SWITCH: 'switch' as const,
  MUSCLE_SENSOR: 'muscle_sensor' as const,
  BRAIN_INTERFACE: 'brain_interface' as const,
  BREATH: 'breath' as const,
  HEAD_MOVEMENT: 'head_movement' as const,
  CUSTOM: 'custom' as const
};
