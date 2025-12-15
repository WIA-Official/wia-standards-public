/**
 * WIA BCI Message Parser
 *
 * Parse and validate protocol messages.
 */

import {
  BciMessage,
  MessageType,
  PROTOCOL_NAME,
  PROTOCOL_VERSION,
  ErrorCodes,
} from './types';

/**
 * Parse result
 */
export interface ParseResult<T = unknown> {
  success: boolean;
  message?: BciMessage<T>;
  error?: {
    code: number;
    message: string;
  };
}

/**
 * Valid message types
 */
const VALID_MESSAGE_TYPES: MessageType[] = [
  'connect',
  'connect_ack',
  'disconnect',
  'start_stream',
  'stop_stream',
  'stream_ack',
  'signal',
  'signal_batch',
  'marker',
  'command',
  'command_ack',
  'config',
  'status',
  'error',
  'ping',
  'pong',
];

/**
 * Message parser for WIA BCI protocol
 */
export class MessageParser {
  private strictMode: boolean;

  constructor(options: { strictMode?: boolean } = {}) {
    this.strictMode = options.strictMode ?? true;
  }

  /**
   * Parse JSON string to message
   */
  parse<T = unknown>(data: string | ArrayBuffer): ParseResult<T> {
    try {
      // Handle binary data
      if (data instanceof ArrayBuffer) {
        return this.parseBinary<T>(data);
      }

      // Parse JSON
      const json = JSON.parse(data);
      return this.validate<T>(json);
    } catch (err) {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_MESSAGE,
          message: `Parse error: ${(err as Error).message}`,
        },
      };
    }
  }

  /**
   * Validate message structure
   */
  validate<T = unknown>(obj: unknown): ParseResult<T> {
    // Check object
    if (!obj || typeof obj !== 'object') {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_MESSAGE,
          message: 'Message must be an object',
        },
      };
    }

    const msg = obj as Record<string, unknown>;

    // Required fields
    const requiredFields = ['protocol', 'version', 'messageId', 'timestamp', 'type', 'payload'];
    for (const field of requiredFields) {
      if (!(field in msg)) {
        return {
          success: false,
          error: {
            code: ErrorCodes.INVALID_MESSAGE,
            message: `Missing required field: ${field}`,
          },
        };
      }
    }

    // Validate protocol
    if (msg.protocol !== PROTOCOL_NAME) {
      return {
        success: false,
        error: {
          code: ErrorCodes.PROTOCOL_ERROR,
          message: `Invalid protocol: ${msg.protocol}`,
        },
      };
    }

    // Validate version (check major version only)
    if (this.strictMode) {
      const version = msg.version as string;
      const majorVersion = version.split('.')[0];
      const expectedMajor = PROTOCOL_VERSION.split('.')[0];

      if (majorVersion !== expectedMajor) {
        return {
          success: false,
          error: {
            code: ErrorCodes.VERSION_MISMATCH,
            message: `Version mismatch: ${version} (expected ${PROTOCOL_VERSION})`,
          },
        };
      }
    }

    // Validate message type
    if (!VALID_MESSAGE_TYPES.includes(msg.type as MessageType)) {
      return {
        success: false,
        error: {
          code: ErrorCodes.UNSUPPORTED_TYPE,
          message: `Invalid message type: ${msg.type}`,
        },
      };
    }

    // Validate timestamp
    if (typeof msg.timestamp !== 'number' || msg.timestamp < 0) {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_PAYLOAD,
          message: 'Invalid timestamp',
        },
      };
    }

    // Validate payload is object
    if (typeof msg.payload !== 'object') {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_PAYLOAD,
          message: 'Payload must be an object',
        },
      };
    }

    return {
      success: true,
      message: msg as BciMessage<T>,
    };
  }

  /**
   * Parse binary message
   */
  private parseBinary<T>(buffer: ArrayBuffer): ParseResult<T> {
    const view = new DataView(buffer);

    // Check minimum size (16 byte header)
    if (buffer.byteLength < 16) {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_MESSAGE,
          message: 'Binary message too short',
        },
      };
    }

    // Read header
    const magic = view.getUint32(0, false);
    if (magic !== 0x57494142) {
      return {
        success: false,
        error: {
          code: ErrorCodes.PROTOCOL_ERROR,
          message: 'Invalid binary magic number',
        },
      };
    }

    const version = view.getUint16(4, false);
    const messageType = view.getUint16(6, false);
    const sequence = view.getUint32(8, false);
    const payloadLength = view.getUint32(12, false);

    // Check payload length
    if (buffer.byteLength < 16 + payloadLength) {
      return {
        success: false,
        error: {
          code: ErrorCodes.INVALID_MESSAGE,
          message: 'Binary payload incomplete',
        },
      };
    }

    // Parse payload based on message type
    const payloadBuffer = buffer.slice(16, 16 + payloadLength);
    const payload = this.parseBinaryPayload(messageType, payloadBuffer);

    // Build message
    const message: BciMessage<T> = {
      protocol: PROTOCOL_NAME,
      version: `${(version >> 8) & 0xff}.${version & 0xff}.0`,
      messageId: `binary-${sequence}`,
      timestamp: Date.now(),
      type: this.messageTypeFromCode(messageType),
      payload: payload as T,
      sequence,
    };

    return { success: true, message };
  }

  /**
   * Parse binary payload
   */
  private parseBinaryPayload(type: number, buffer: ArrayBuffer): unknown {
    const view = new DataView(buffer);

    // Signal type (1)
    if (type === 1) {
      // Read signal header (16 bytes)
      const timestampLow = view.getUint32(0, false);
      const timestampHigh = view.getUint32(4, false);
      const timestamp = timestampHigh * 0x100000000 + timestampLow;
      const sampleIndex = view.getUint32(8, false);
      const channelCount = view.getUint16(12, false);

      // Read channel data
      const data: number[] = [];
      const channels: number[] = [];
      const dataOffset = 16;

      for (let i = 0; i < channelCount; i++) {
        channels.push(i);
        data.push(view.getFloat32(dataOffset + i * 4, false));
      }

      return {
        sampleIndex,
        timestamp: timestamp / 1000, // Convert to milliseconds
        channels,
        data,
      };
    }

    // Default: decode as JSON
    const decoder = new TextDecoder();
    const json = decoder.decode(buffer);
    return JSON.parse(json);
  }

  /**
   * Convert message type code to string
   */
  private messageTypeFromCode(code: number): MessageType {
    const types: Record<number, MessageType> = {
      0: 'connect',
      1: 'signal',
      2: 'signal_batch',
      3: 'marker',
      4: 'error',
      5: 'ping',
      6: 'pong',
      10: 'connect_ack',
      11: 'disconnect',
      12: 'start_stream',
      13: 'stop_stream',
      14: 'stream_ack',
      20: 'command',
      21: 'command_ack',
      22: 'config',
      23: 'status',
    };
    return types[code] ?? 'error';
  }
}

/**
 * Serialize message to JSON string
 */
export function serialize(message: BciMessage): string {
  return JSON.stringify(message);
}

/**
 * Serialize message to binary
 */
export function serializeBinary(message: BciMessage): ArrayBuffer {
  const typeCode = messageTypeToCode(message.type);

  // For signal messages, use binary encoding
  if (message.type === 'signal') {
    return serializeSignalBinary(message);
  }

  // For other messages, encode payload as JSON
  const encoder = new TextEncoder();
  const payloadBytes = encoder.encode(JSON.stringify(message.payload));

  // Create buffer (16 byte header + payload)
  const buffer = new ArrayBuffer(16 + payloadBytes.length);
  const view = new DataView(buffer);

  // Write header
  view.setUint32(0, 0x57494142, false); // Magic
  view.setUint16(4, 0x0100, false);      // Version 1.0
  view.setUint16(6, typeCode, false);    // Message type
  view.setUint32(8, message.sequence ?? 0, false); // Sequence
  view.setUint32(12, payloadBytes.length, false);  // Payload length

  // Write payload
  const payloadView = new Uint8Array(buffer, 16);
  payloadView.set(payloadBytes);

  return buffer;
}

/**
 * Serialize signal message to binary
 */
function serializeSignalBinary(message: BciMessage): ArrayBuffer {
  const payload = message.payload as { sampleIndex: number; timestamp: number; data: number[] };
  const channelCount = payload.data.length;

  // Header (16) + Signal header (16) + Data (channels * 4)
  const buffer = new ArrayBuffer(32 + channelCount * 4);
  const view = new DataView(buffer);

  // Protocol header
  view.setUint32(0, 0x57494142, false); // Magic
  view.setUint16(4, 0x0100, false);      // Version 1.0
  view.setUint16(6, 1, false);           // Signal type
  view.setUint32(8, message.sequence ?? 0, false);
  view.setUint32(12, 16 + channelCount * 4, false);

  // Signal header
  const timestampMicro = Math.floor(payload.timestamp * 1000);
  view.setUint32(16, timestampMicro & 0xffffffff, false);
  view.setUint32(20, Math.floor(timestampMicro / 0x100000000), false);
  view.setUint32(24, payload.sampleIndex, false);
  view.setUint16(28, channelCount, false);
  view.setUint16(30, 0, false); // Reserved

  // Channel data
  for (let i = 0; i < channelCount; i++) {
    view.setFloat32(32 + i * 4, payload.data[i], false);
  }

  return buffer;
}

/**
 * Convert message type to code
 */
function messageTypeToCode(type: MessageType): number {
  const codes: Record<MessageType, number> = {
    connect: 0,
    signal: 1,
    signal_batch: 2,
    marker: 3,
    error: 4,
    ping: 5,
    pong: 6,
    connect_ack: 10,
    disconnect: 11,
    start_stream: 12,
    stop_stream: 13,
    stream_ack: 14,
    command: 20,
    command_ack: 21,
    config: 22,
    status: 23,
  };
  return codes[type] ?? 4;
}

/**
 * Default parser instance
 */
export const messageParser = new MessageParser();
