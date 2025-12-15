/**
 * WIA Eye Gaze Interoperability Protocol SDK
 *
 * Standard interface for eye tracking devices and gaze-aware applications.
 *
 * @packageDocumentation
 *
 * @example Basic Usage
 * ```typescript
 * import {
 *   createEyeTracker,
 *   createMockAdapter,
 *   createDwellController,
 * } from '@anthropics/wia-eye-gaze';
 *
 * // Create tracker with mock adapter (for testing)
 * const adapter = createMockAdapter({ samplingRate: 60 });
 * const tracker = createEyeTracker(adapter);
 *
 * // Connect and start tracking
 * await tracker.connect();
 * await tracker.startCalibration();
 *
 * // Subscribe to gaze data
 * tracker.subscribe(gaze => {
 *   console.log(`Gaze: (${gaze.x.toFixed(2)}, ${gaze.y.toFixed(2)})`);
 * });
 *
 * tracker.startTracking();
 *
 * // Create dwell controller for selection
 * const dwell = createDwellController({
 *   tracker,
 *   threshold: 800,
 * });
 *
 * dwell.registerTarget({
 *   elementId: 'btn-ok',
 *   boundingBox: { x: 0.4, y: 0.5, width: 0.2, height: 0.1 },
 *   semanticType: 'button',
 *   label: 'OK',
 * });
 *
 * dwell.onDwellComplete(target => {
 *   console.log(`Selected: ${target.label}`);
 * });
 *
 * dwell.start();
 * ```
 *
 * @example Using Real Hardware
 * ```typescript
 * import {
 *   createEyeTracker,
 *   createTobiiAdapter,
 * } from '@anthropics/wia-eye-gaze';
 *
 * // Use Tobii adapter for real hardware
 * const adapter = createTobiiAdapter({ samplingRate: 120 });
 * const tracker = createEyeTracker(adapter);
 *
 * await tracker.connect();
 * console.log(tracker.getCapabilities());
 * ```
 *
 * 弘益人間 (홍익인간) - 널리 인간을 이롭게
 *
 * @license MIT
 * @author SmileStory Inc. / WIA
 */

// ============================================
// Types
// ============================================
export * from './types';

// ============================================
// Tracker
// ============================================
export {
  IEyeTracker,
  IEyeTrackerAdapter,
  WiaEyeTracker,
  createEyeTracker,
  // Adapters
  MockAdapter,
  createMockAdapter,
  TobiiAdapter,
  createTobiiAdapter,
  GazepointAdapter,
  createGazepointAdapter,
  PupilLabsAdapter,
  createPupilLabsAdapter,
} from './tracker';

export type {
  MockAdapterOptions,
  TobiiAdapterOptions,
  GazepointAdapterOptions,
  PupilLabsAdapterOptions,
} from './tracker';

// ============================================
// Dwell
// ============================================
export {
  DwellController,
  createDwellController,
} from './dwell';

export type {
  DwellControllerOptions,
  DwellEventHandler,
} from './dwell';

// ============================================
// App
// ============================================
export {
  GazeAwareApp,
  createGazeAwareApp,
} from './app';

export type {
  GazeAwareAppOptions,
  ControlRequestHandler,
  MessageHandler,
} from './app';

// ============================================
// Version
// ============================================
export const VERSION = '1.0.0-alpha';
export const PROTOCOL_VERSION = '1.0.0';
