/**
 * WIA AAC Protocol Message Parser
 */

import {
  WiaAacMessage,
  MessageType,
  PROTOCOL_NAME,
  PROTOCOL_VERSION,
  ErrorCodes,
  ProtocolError,
} from './message';
import { MessageBuilder } from './MessageBuilder';

export interface ParseResult<T = unknown> {
  success: boolean;
  message?: WiaAacMessage<T>;
  error?: ProtocolError;
}

export class MessageParser {
  private supportedVersions: string[];

  constructor(supportedVersions: string[] = [PROTOCOL_VERSION]) {
    this.supportedVersions = supportedVersions;
  }

  /**
   * Parse a JSON string into a WiaAacMessage
   */
  parse(data: string | object): ParseResult {
    try {
      // Parse JSON if string
      const obj = typeof data === 'string' ? JSON.parse(data) : data;

      // Validate required fields
      const validationError = this.validate(obj);
      if (validationError) {
        return {
          success: false,
          error: validationError,
        };
      }

      return {
        success: true,
        message: obj as WiaAacMessage,
      };
    } catch (e) {
      return {
        success: false,
        error: MessageBuilder.createError(
          ErrorCodes.INVALID_MESSAGE,
          'INVALID_MESSAGE',
          `Failed to parse message: ${e instanceof Error ? e.message : 'Unknown error'}`,
          false
        ),
      };
    }
  }

  /**
   * Validate a message object
   */
  private validate(obj: unknown): ProtocolError | null {
    if (!obj || typeof obj !== 'object') {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        'Message must be an object',
        false
      );
    }

    const msg = obj as Record<string, unknown>;

    // Check protocol
    if (msg.protocol !== PROTOCOL_NAME) {
      return MessageBuilder.createError(
        ErrorCodes.PROTOCOL_ERROR,
        'PROTOCOL_ERROR',
        `Invalid protocol: expected '${PROTOCOL_NAME}', got '${msg.protocol}'`,
        false
      );
    }

    // Check version
    if (typeof msg.version !== 'string') {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        'Missing or invalid version field',
        false
      );
    }

    if (!this.isVersionSupported(msg.version)) {
      return MessageBuilder.createError(
        ErrorCodes.VERSION_MISMATCH,
        'VERSION_MISMATCH',
        `Unsupported protocol version: ${msg.version}`,
        true,
        { supportedVersions: this.supportedVersions }
      );
    }

    // Check messageId
    if (typeof msg.messageId !== 'string' || !msg.messageId) {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        'Missing or invalid messageId field',
        false
      );
    }

    // Check timestamp
    if (typeof msg.timestamp !== 'number' || msg.timestamp < 0) {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        'Missing or invalid timestamp field',
        false
      );
    }

    // Check type
    if (!this.isValidMessageType(msg.type)) {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        `Invalid message type: ${msg.type}`,
        false
      );
    }

    // Check payload
    if (typeof msg.payload !== 'object') {
      return MessageBuilder.createError(
        ErrorCodes.INVALID_MESSAGE,
        'INVALID_MESSAGE',
        'Missing or invalid payload field',
        false
      );
    }

    return null;
  }

  /**
   * Check if version is supported
   */
  private isVersionSupported(version: string): boolean {
    // Check exact match
    if (this.supportedVersions.includes(version)) {
      return true;
    }

    // Check major version compatibility (1.x.x is compatible with 1.0.0)
    const [major] = version.split('.');
    return this.supportedVersions.some((v) => v.startsWith(`${major}.`));
  }

  /**
   * Check if message type is valid
   */
  private isValidMessageType(type: unknown): type is MessageType {
    const validTypes: MessageType[] = [
      'connect',
      'connect_ack',
      'disconnect',
      'disconnect_ack',
      'signal',
      'command',
      'command_ack',
      'subscribe',
      'subscribe_ack',
      'unsubscribe',
      'unsubscribe_ack',
      'error',
      'ping',
      'pong',
    ];
    return typeof type === 'string' && validTypes.includes(type as MessageType);
  }

  /**
   * Serialize a message to JSON string
   */
  serialize(message: WiaAacMessage): string {
    return JSON.stringify(message);
  }
}

// Export singleton instance
export const messageParser = new MessageParser();
