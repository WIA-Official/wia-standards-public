/**
 * WIA AAC Protocol Message Builder
 */

import { v4 as uuidv4 } from 'uuid';
import {
  WiaAacMessage,
  MessageType,
  PROTOCOL_NAME,
  PROTOCOL_VERSION,
  ConnectPayload,
  DisconnectPayload,
  SubscribePayload,
  UnsubscribePayload,
  CommandPayload,
  ProtocolError,
  PingPayload,
  PongPayload,
} from './message';
import { WiaAacSignal, SensorType } from '../types/signal';

export class MessageBuilder {
  private version: string;

  constructor(version: string = PROTOCOL_VERSION) {
    this.version = version;
  }

  /**
   * Create a base message
   */
  private createMessage<T>(type: MessageType, payload: T): WiaAacMessage<T> {
    return {
      protocol: PROTOCOL_NAME,
      version: this.version,
      messageId: uuidv4(),
      timestamp: Date.now(),
      type,
      payload,
    };
  }

  /**
   * Create a connect message
   */
  connect(payload: ConnectPayload): WiaAacMessage<ConnectPayload> {
    return this.createMessage('connect', payload);
  }

  /**
   * Create a connect acknowledgment message
   */
  connectAck(
    success: boolean,
    options?: {
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
      error?: ProtocolError;
    }
  ): WiaAacMessage<any> {
    return this.createMessage('connect_ack', {
      success,
      ...options,
    });
  }

  /**
   * Create a disconnect message
   */
  disconnect(
    reason: DisconnectPayload['reason'],
    message?: string
  ): WiaAacMessage<DisconnectPayload> {
    return this.createMessage('disconnect', { reason, message });
  }

  /**
   * Create a disconnect acknowledgment message
   */
  disconnectAck(): WiaAacMessage<{}> {
    return this.createMessage('disconnect_ack', {});
  }

  /**
   * Create a signal message (wraps Phase 1 Signal)
   */
  signal(signalData: WiaAacSignal): WiaAacMessage<WiaAacSignal> {
    return this.createMessage('signal', signalData);
  }

  /**
   * Create a subscribe message
   */
  subscribe(payload: SubscribePayload): WiaAacMessage<SubscribePayload> {
    return this.createMessage('subscribe', payload);
  }

  /**
   * Create a subscribe acknowledgment message
   */
  subscribeAck(
    success: boolean,
    options?: {
      subscriptionId?: string;
      activeStreams?: SensorType[];
      actualSignalRate?: number;
      error?: ProtocolError;
    }
  ): WiaAacMessage<any> {
    return this.createMessage('subscribe_ack', {
      success,
      ...options,
    });
  }

  /**
   * Create an unsubscribe message
   */
  unsubscribe(payload: UnsubscribePayload): WiaAacMessage<UnsubscribePayload> {
    return this.createMessage('unsubscribe', payload);
  }

  /**
   * Create an unsubscribe acknowledgment message
   */
  unsubscribeAck(success: boolean): WiaAacMessage<{ success: boolean }> {
    return this.createMessage('unsubscribe_ack', { success });
  }

  /**
   * Create a command message
   */
  command(payload: CommandPayload): WiaAacMessage<CommandPayload> {
    return this.createMessage('command', payload);
  }

  /**
   * Create a command acknowledgment message
   */
  commandAck(
    success: boolean,
    command: string,
    result?: unknown,
    error?: ProtocolError
  ): WiaAacMessage<any> {
    return this.createMessage('command_ack', {
      success,
      command,
      result,
      error,
    });
  }

  /**
   * Create an error message
   */
  error(error: ProtocolError): WiaAacMessage<ProtocolError> {
    return this.createMessage('error', error);
  }

  /**
   * Create a ping message
   */
  ping(sequence: number): WiaAacMessage<PingPayload> {
    return this.createMessage('ping', { sequence });
  }

  /**
   * Create a pong message
   */
  pong(sequence: number, latency_ms?: number): WiaAacMessage<PongPayload> {
    return this.createMessage('pong', { sequence, latency_ms });
  }

  /**
   * Create an error from code
   */
  static createError(
    code: number,
    name: string,
    message: string,
    recoverable: boolean = true,
    details?: Record<string, unknown>,
    relatedMessageId?: string
  ): ProtocolError {
    return {
      code,
      name,
      message,
      recoverable,
      details,
      relatedMessageId,
    };
  }
}

// Export singleton instance
export const messageBuilder = new MessageBuilder();
