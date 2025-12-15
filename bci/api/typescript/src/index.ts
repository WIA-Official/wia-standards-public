/**
 * WIA BCI - Brain-Computer Interface Standard API
 *
 * @packageDocumentation
 * @module wia-bci
 *
 * @example
 * ```typescript
 * import { WiaBci } from 'wia-bci';
 *
 * const bci = new WiaBci();
 * await bci.connect({ type: 'eeg_headset' });
 *
 * bci.on('signal', (event) => {
 *   console.log('Signal:', event.data);
 * });
 *
 * await bci.startStream();
 * ```
 */

// Core classes
export { WiaBci } from './core/WiaBci';
export { BciEventEmitter } from './core/EventEmitter';
export { BciError, ErrorCodes, type ErrorCode } from './core/BciError';
export { SignalProcessor } from './core/SignalProcessor';

// Adapters
export { BaseAdapter, type IBciAdapter } from './adapters/BaseAdapter';
export { SimulatorAdapter } from './adapters/SimulatorAdapter';

// Types - Config
export type {
  DeviceType,
  ConnectionProtocol,
  LogLevel,
  FilterType,
  FilterConfig,
  WiaBciOptions,
  DeviceConfig,
  DeviceIdentifier,
  ConnectionConfig,
  AcquisitionConfig,
} from './types/config';

// Types - Device
export type {
  DeviceStatus,
  DeviceCapabilities,
  DeviceInfo,
  ChannelInfo,
  BciState,
} from './types/device';

// Types - Events
export type {
  EventType,
  SignalEvent,
  MarkerEvent,
  ClassificationEvent,
  ChannelQuality,
  QualityEvent,
  ErrorEvent,
  EventDataMap,
  EventHandler,
} from './types/events';

// Types - Signal
export type {
  FrequencyBand,
  BandPowers,
  PowerSpectrum,
  ComplexNumber,
  ComplexArray,
  HjorthParams,
  FeatureVector,
  ArtifactType,
  ArtifactSegment,
  RecordingInfo,
  SessionInfo,
  WiaBciRecording,
} from './types/signal';

// Protocol (Phase 3)
export * from './protocol';

// Transport (Phase 3)
export * from './transport';

// Output (Phase 4)
export * from './output';

// Constants
export { STANDARD_10_20_CHANNELS, STANDARD_10_10_CHANNELS } from './types/device';
export { FREQUENCY_BANDS } from './types/signal';
export { DEFAULT_OPTIONS } from './types/config';

/**
 * Package version
 */
export const VERSION = '1.0.0';
