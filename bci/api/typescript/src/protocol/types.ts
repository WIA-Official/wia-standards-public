/**
 * WIA BCI Protocol Types
 *
 * Phase 3: Communication Protocol
 */

// Protocol constants
export const PROTOCOL_NAME = 'wia-bci';
export const PROTOCOL_VERSION = '1.0.0';
export const DEFAULT_PORT = 9876;
export const BINARY_MAGIC = 0x57494142; // "WIAB"

// Message types
export type MessageType =
  | 'connect'
  | 'connect_ack'
  | 'disconnect'
  | 'start_stream'
  | 'stop_stream'
  | 'stream_ack'
  | 'signal'
  | 'signal_batch'
  | 'marker'
  | 'command'
  | 'command_ack'
  | 'config'
  | 'status'
  | 'error'
  | 'ping'
  | 'pong';

// Base message structure
export interface BciMessage<T = unknown> {
  protocol: 'wia-bci';
  version: string;
  messageId: string;
  timestamp: number;
  type: MessageType;
  payload: T;
  sequence?: number;
  sessionId?: string;
}

// Connect payload
export interface ConnectPayload {
  clientId: string;
  clientName?: string;
  clientVersion?: string;
  capabilities?: string[];
  options?: ConnectOptions;
  auth?: AuthPayload;
}

export interface ConnectOptions {
  samplingRate?: number;
  channels?: string[];
  compression?: boolean;
  binaryMode?: boolean;
  encryption?: EncryptionOptions;
}

export interface EncryptionOptions {
  enabled: boolean;
  algorithm?: string;
}

export interface AuthPayload {
  type: 'token' | 'api_key';
  token?: string;
  apiKey?: string;
}

// Connect ack payload
export interface ConnectAckPayload {
  sessionId: string;
  status: 'connected' | 'rejected';
  serverInfo?: ServerInfo;
  deviceInfo?: DeviceInfoPayload;
  negotiated?: NegotiatedOptions;
  error?: ErrorPayload;
}

export interface ServerInfo {
  name: string;
  version: string;
}

export interface DeviceInfoPayload {
  type: string;
  manufacturer?: string;
  model?: string;
  channels: number;
  samplingRate: number;
}

export interface NegotiatedOptions {
  samplingRate: number;
  channels: string[];
  compression: boolean;
  binaryMode?: boolean;
}

// Disconnect payload
export interface DisconnectPayload {
  reason?: string;
  code?: number;
}

// Stream control payloads
export interface StartStreamPayload {
  channels?: string[];
  samplingRate?: number;
}

export interface StopStreamPayload {
  reason?: string;
}

export interface StreamAckPayload {
  status: 'started' | 'stopped' | 'error';
  error?: ErrorPayload;
}

// Signal payload
export interface SignalPayload {
  sampleIndex: number;
  timestamp: number;
  channels: number[];
  data: number[];
}

// Signal batch payload
export interface SignalBatchPayload {
  startSampleIndex: number;
  sampleCount: number;
  channels: number[];
  data: number[][];
  timestamps: number[];
}

// Marker payload
export interface MarkerPayload {
  sampleIndex: number;
  code: number;
  label: string;
  value?: string;
  duration?: number;
}

// Command payloads
export interface CommandPayload {
  command: string;
  params?: Record<string, unknown>;
}

export interface CommandAckPayload {
  command: string;
  status: 'success' | 'error';
  result?: unknown;
  error?: ErrorPayload;
}

// Config payload
export interface ConfigPayload {
  key: string;
  value: unknown;
}

// Status payload
export interface StatusPayload {
  state: ConnectionState;
  streaming: boolean;
  deviceStatus?: DeviceStatus;
  stats?: StreamStats;
}

export interface DeviceStatus {
  battery?: number;
  signal_quality?: number;
  impedances?: Record<string, number>;
}

export interface StreamStats {
  samplesReceived: number;
  packetsDropped: number;
  latencyMs: number;
}

// Error payload
export interface ErrorPayload {
  code: number;
  name: string;
  message: string;
  recoverable: boolean;
  details?: Record<string, unknown>;
}

// Ping/Pong payloads
export interface PingPayload {
  clientTime?: number;
}

export interface PongPayload {
  serverTime: number;
  clientTime?: number;
}

// Connection states
export type ConnectionState =
  | 'disconnected'
  | 'connecting'
  | 'connected'
  | 'reconnecting'
  | 'error';

// Error codes
export const ErrorCodes = {
  // Connection errors (1xxx)
  CONNECTION_CLOSED: 1000,
  CONNECTION_LOST: 1001,
  CONNECTION_TIMEOUT: 1002,
  PROTOCOL_ERROR: 1003,
  VERSION_MISMATCH: 1004,

  // Device errors (2xxx)
  DEVICE_NOT_FOUND: 2001,
  DEVICE_BUSY: 2002,
  DEVICE_ERROR: 2003,
  STREAM_ERROR: 2004,

  // Message errors (3xxx)
  INVALID_MESSAGE: 3001,
  INVALID_PAYLOAD: 3002,
  UNSUPPORTED_TYPE: 3003,

  // Auth errors (4xxx)
  AUTH_FAILED: 4001,
  PERMISSION_DENIED: 4002,
} as const;

export type ErrorCode = (typeof ErrorCodes)[keyof typeof ErrorCodes];

// Heartbeat configuration
export interface HeartbeatConfig {
  pingInterval: number;    // Default: 30000ms
  pongTimeout: number;     // Default: 10000ms
  maxMissedPongs: number;  // Default: 3
}

// Reconnection configuration
export interface ReconnectConfig {
  enabled: boolean;
  maxAttempts: number;       // Default: 5
  initialDelay: number;      // Default: 1000ms
  maxDelay: number;          // Default: 30000ms
  backoffMultiplier: number; // Default: 2
}

// Transport options
export interface TransportOptions {
  url?: string;
  binaryMode?: boolean;
  heartbeat?: HeartbeatConfig;
  reconnect?: ReconnectConfig;
}

// Binary message header (16 bytes)
export interface BinaryHeader {
  magic: number;       // 4 bytes: 0x57494142
  version: number;     // 2 bytes: 0x0100
  messageType: number; // 2 bytes
  sequence: number;    // 4 bytes
  payloadLength: number; // 4 bytes
}

// Binary signal header (16 bytes)
export interface BinarySignalHeader {
  timestamp: bigint;   // 8 bytes: microseconds
  sampleIndex: number; // 4 bytes
  channelCount: number; // 2 bytes
  reserved: number;    // 2 bytes
}
