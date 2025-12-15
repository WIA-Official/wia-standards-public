/**
 * WIA AAC Event Types
 */

import { WiaAacSignal, DeviceInfo, SensorType } from './signal';

// Event Types
export type EventType =
  | 'signal'
  | 'selection'
  | 'gesture'
  | 'error'
  | 'connected'
  | 'disconnected';

// Selection Event
export interface SelectionEvent {
  timestamp: number;
  targetId: string;
  targetType: 'key' | 'button' | 'area' | 'custom';
  selectionMethod: 'dwell' | 'click' | 'gesture' | 'switch';
  position?: { x: number; y: number };
  confidence: number;
}

// Gesture Event
export interface GestureEvent {
  timestamp: number;
  gesture: string;
  sensorType: SensorType;
  confidence: number;
  metadata?: Record<string, unknown>;
}

// Error Codes
export enum ErrorCode {
  // Connection errors (1xx)
  CONNECTION_FAILED = 100,
  CONNECTION_LOST = 101,
  CONNECTION_TIMEOUT = 102,
  DEVICE_NOT_FOUND = 103,
  DEVICE_BUSY = 104,
  PERMISSION_DENIED = 105,

  // Configuration errors (2xx)
  INVALID_CONFIG = 200,
  UNSUPPORTED_OPTION = 201,
  INVALID_SENSOR_TYPE = 202,

  // Runtime errors (3xx)
  SIGNAL_VALIDATION_FAILED = 300,
  ADAPTER_ERROR = 301,
  INTERNAL_ERROR = 302,

  // Protocol errors (4xx)
  PROTOCOL_ERROR = 400,
  MESSAGE_PARSE_ERROR = 401,
  UNSUPPORTED_VERSION = 402
}

// WIA AAC Error
export interface WiaAacError {
  code: ErrorCode;
  message: string;
  timestamp: number;
  recoverable: boolean;
  details?: Record<string, unknown>;
}

// Disconnect Reason
export interface DisconnectReason {
  reason: 'user' | 'error' | 'timeout' | 'device_removed';
  message?: string;
}

// Event Data Map
export interface EventDataMap {
  signal: WiaAacSignal;
  selection: SelectionEvent;
  gesture: GestureEvent;
  error: WiaAacError;
  connected: DeviceInfo;
  disconnected: DisconnectReason;
}

// Event Handler Type
export type EventHandler<T extends EventType> = (data: EventDataMap[T]) => void;

// Event Handler Options
export interface EventHandlerOptions<T extends EventType> {
  filter?: (data: EventDataMap[T]) => boolean;
  once?: boolean;
}
