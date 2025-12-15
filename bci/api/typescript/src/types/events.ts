/**
 * WIA BCI Event Types
 * @module wia-bci/types/events
 */

/**
 * Event Types
 */
export type EventType =
  // Connection events
  | 'connected'
  | 'disconnected'
  | 'reconnecting'
  | 'connection_error'
  // Streaming events
  | 'stream_started'
  | 'stream_stopped'
  | 'data'
  | 'signal'
  // Marker events
  | 'marker'
  | 'trigger'
  // Classification events
  | 'classification'
  | 'prediction'
  // Device events
  | 'battery'
  | 'impedance'
  | 'quality'
  // Error events
  | 'error'
  | 'warning';

/**
 * Signal Event - Raw data from BCI device
 */
export interface SignalEvent {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Sample index in the recording */
  sampleIndex: number;
  /** Channel indices */
  channels: number[];
  /** Signal data (microvolts) */
  data: Float32Array;
}

/**
 * Marker Event - Event markers/triggers
 */
export interface MarkerEvent {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Sample index at marker time */
  sampleIndex: number;
  /** Numeric event code */
  code: number;
  /** Event label */
  label?: string;
  /** Additional value */
  value?: unknown;
  /** Duration in milliseconds (0 for instantaneous) */
  duration?: number;
}

/**
 * Classification Event - ML model output
 */
export interface ClassificationEvent {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Predicted class ID */
  classId: number;
  /** Class name */
  className: string;
  /** Confidence score (0-1) */
  confidence: number;
  /** Per-class probabilities */
  probabilities?: Record<string, number>;
}

/**
 * Channel Quality Information
 */
export interface ChannelQuality {
  /** Channel index */
  channel: number;
  /** Channel label */
  label: string;
  /** Impedance in kOhm */
  impedance?: number;
  /** Signal quality (0-1) */
  signalQuality: number;
  /** Detected artifacts */
  artifacts: string[];
}

/**
 * Quality Event - Signal quality update
 */
export interface QualityEvent {
  /** Timestamp in milliseconds */
  timestamp: number;
  /** Per-channel quality info */
  channelQualities: ChannelQuality[];
  /** Overall quality score (0-1) */
  overallQuality: number;
}

/**
 * Error Event
 */
export interface ErrorEvent {
  /** Error code */
  code: string;
  /** Human-readable message */
  message: string;
  /** Additional details */
  details?: unknown;
  /** Whether error is recoverable */
  recoverable: boolean;
}

/**
 * Event Handler Type Map
 */
export type EventDataMap = {
  connected: void;
  disconnected: void;
  reconnecting: { attempt: number; maxAttempts: number };
  connection_error: ErrorEvent;
  stream_started: void;
  stream_stopped: void;
  data: SignalEvent;
  signal: SignalEvent;
  marker: MarkerEvent;
  trigger: MarkerEvent;
  classification: ClassificationEvent;
  prediction: ClassificationEvent;
  battery: number;
  impedance: QualityEvent;
  quality: QualityEvent;
  error: ErrorEvent;
  warning: ErrorEvent;
};

/**
 * Event Handler Function Type
 */
export type EventHandler<T extends EventType> =
  EventDataMap[T] extends void
    ? () => void
    : (data: EventDataMap[T]) => void;
