/**
 * WIA AAC Protocol Message Types
 */

import { WiaAacSignal, SensorType } from '../types/signal';

// Protocol constants
export const PROTOCOL_NAME = 'wia-aac';
export const PROTOCOL_VERSION = '1.0.0';
export const DEFAULT_PORT = 8765;
export const SUB_PROTOCOL = 'wia-aac-v1';

// Message types
export type MessageType =
  | 'connect'
  | 'connect_ack'
  | 'disconnect'
  | 'disconnect_ack'
  | 'signal'
  | 'command'
  | 'command_ack'
  | 'subscribe'
  | 'subscribe_ack'
  | 'unsubscribe'
  | 'unsubscribe_ack'
  | 'error'
  | 'ping'
  | 'pong';

// Base message structure
export interface WiaAacMessage<T = unknown> {
  protocol: 'wia-aac';
  version: string;
  messageId: string;
  timestamp: number;
  type: MessageType;
  payload: T;
}

// Connect payload
export interface ConnectPayload {
  clientId: string;
  clientName?: string;
  clientVersion?: string;
  capabilities?: SensorType[];
  authToken?: string;
  options?: {
    signalRate?: number;
    compression?: boolean;
    heartbeatInterval?: number;
  };
}

// Connect acknowledgment payload
export interface ConnectAckPayload {
  success: boolean;
  sessionId?: string;
  serverName?: string;
  serverVersion?: string;
  availableSensors?: Array<{
    type: SensorType;
    manufacturer: string;
    model: string;
    deviceId: string;
  }>;
  config?: {
    maxSignalRate?: number;
    heartbeatInterval?: number;
  };
  permissions?: Record<string, string[]>;
  error?: ProtocolError;
}

// Disconnect payload
export interface DisconnectPayload {
  reason: 'user_request' | 'timeout' | 'error' | 'server_shutdown';
  message?: string;
}

// Signal payload (uses Phase 1 Signal Format)
export type SignalPayload = WiaAacSignal;

// Subscribe payload
export interface SubscribePayload {
  streams: SensorType[];
  deviceFilter?: {
    type?: SensorType;
    manufacturer?: string;
    model?: string;
    deviceId?: string;
  };
  options?: {
    signalRate?: number;
    includeRaw?: boolean;
  };
}

// Subscribe acknowledgment payload
export interface SubscribeAckPayload {
  success: boolean;
  subscriptionId?: string;
  activeStreams?: SensorType[];
  actualSignalRate?: number;
  error?: ProtocolError;
}

// Unsubscribe payload
export interface UnsubscribePayload {
  subscriptionId?: string;
  streams?: SensorType[];
}

// Command payload
export interface CommandPayload {
  command: 'calibrate' | 'configure' | 'reset' | 'get_status' | 'get_config';
  target: SensorType | 'all';
  parameters?: Record<string, unknown>;
}

// Command acknowledgment payload
export interface CommandAckPayload {
  success: boolean;
  command: string;
  result?: unknown;
  error?: ProtocolError;
}

// Error payload
export interface ProtocolError {
  code: number;
  name: string;
  message: string;
  recoverable: boolean;
  details?: Record<string, unknown>;
  relatedMessageId?: string;
}

// Ping/Pong payload
export interface PingPayload {
  sequence: number;
}

export interface PongPayload {
  sequence: number;
  latency_ms?: number;
}

// Error codes
export const ErrorCodes = {
  // Connection errors (1xxx)
  CONNECTION_CLOSED: 1000,
  CONNECTION_LOST: 1001,
  CONNECTION_TIMEOUT: 1002,
  PROTOCOL_ERROR: 1003,
  VERSION_MISMATCH: 1004,
  INVALID_MESSAGE: 1005,

  // Sensor errors (2xxx)
  SENSOR_NOT_FOUND: 2001,
  SENSOR_BUSY: 2002,
  SENSOR_ERROR: 2003,
  SENSOR_DISCONNECTED: 2004,
  CALIBRATION_REQUIRED: 2005,
  CALIBRATION_FAILED: 2006,

  // Auth errors (3xxx)
  AUTH_REQUIRED: 3001,
  AUTH_FAILED: 3002,
  PERMISSION_DENIED: 3003,
  SESSION_EXPIRED: 3004,

  // Subscription errors (4xxx)
  SUBSCRIPTION_FAILED: 4001,
  STREAM_NOT_AVAILABLE: 4002,
  RATE_LIMIT_EXCEEDED: 4003,
} as const;

export type ErrorCode = typeof ErrorCodes[keyof typeof ErrorCodes];

// Type-safe message types
export type ConnectMessage = WiaAacMessage<ConnectPayload>;
export type ConnectAckMessage = WiaAacMessage<ConnectAckPayload>;
export type DisconnectMessage = WiaAacMessage<DisconnectPayload>;
export type SignalMessage = WiaAacMessage<SignalPayload>;
export type SubscribeMessage = WiaAacMessage<SubscribePayload>;
export type SubscribeAckMessage = WiaAacMessage<SubscribeAckPayload>;
export type CommandMessage = WiaAacMessage<CommandPayload>;
export type CommandAckMessage = WiaAacMessage<CommandAckPayload>;
export type ErrorMessage = WiaAacMessage<ProtocolError>;
export type PingMessage = WiaAacMessage<PingPayload>;
export type PongMessage = WiaAacMessage<PongPayload>;
