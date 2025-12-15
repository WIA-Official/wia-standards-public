/**
 * WIA BCI Message Builder
 *
 * Factory for creating protocol messages.
 */

import {
  BciMessage,
  MessageType,
  ConnectPayload,
  ConnectAckPayload,
  DisconnectPayload,
  StartStreamPayload,
  StopStreamPayload,
  StreamAckPayload,
  SignalPayload,
  SignalBatchPayload,
  MarkerPayload,
  CommandPayload,
  CommandAckPayload,
  ConfigPayload,
  StatusPayload,
  ErrorPayload,
  PingPayload,
  PongPayload,
  PROTOCOL_NAME,
  PROTOCOL_VERSION,
} from './types';

/**
 * Generate UUID v4
 */
function generateUuid(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Message builder for WIA BCI protocol
 */
export class MessageBuilder {
  private sequence = 0;
  private sessionId?: string;

  /**
   * Set session ID for messages
   */
  setSessionId(sessionId: string): void {
    this.sessionId = sessionId;
  }

  /**
   * Reset sequence counter
   */
  resetSequence(): void {
    this.sequence = 0;
  }

  /**
   * Create base message
   */
  private createBase<T>(type: MessageType, payload: T): BciMessage<T> {
    const message: BciMessage<T> = {
      protocol: PROTOCOL_NAME,
      version: PROTOCOL_VERSION,
      messageId: generateUuid(),
      timestamp: Date.now(),
      type,
      payload,
    };

    if (this.sessionId) {
      message.sessionId = this.sessionId;
    }

    return message;
  }

  /**
   * Create base message with sequence
   */
  private createWithSequence<T>(type: MessageType, payload: T): BciMessage<T> {
    const message = this.createBase(type, payload);
    message.sequence = this.sequence++;
    return message;
  }

  // Connection messages

  /**
   * Create connect message
   */
  connect(payload: ConnectPayload): BciMessage<ConnectPayload> {
    return this.createBase('connect', payload);
  }

  /**
   * Create connect ack message
   */
  connectAck(payload: ConnectAckPayload): BciMessage<ConnectAckPayload> {
    if (payload.sessionId) {
      this.sessionId = payload.sessionId;
    }
    return this.createBase('connect_ack', payload);
  }

  /**
   * Create disconnect message
   */
  disconnect(payload: DisconnectPayload = {}): BciMessage<DisconnectPayload> {
    return this.createBase('disconnect', payload);
  }

  // Stream control messages

  /**
   * Create start stream message
   */
  startStream(payload: StartStreamPayload = {}): BciMessage<StartStreamPayload> {
    return this.createBase('start_stream', payload);
  }

  /**
   * Create stop stream message
   */
  stopStream(payload: StopStreamPayload = {}): BciMessage<StopStreamPayload> {
    return this.createBase('stop_stream', payload);
  }

  /**
   * Create stream ack message
   */
  streamAck(payload: StreamAckPayload): BciMessage<StreamAckPayload> {
    return this.createBase('stream_ack', payload);
  }

  // Signal messages

  /**
   * Create signal message
   */
  signal(payload: SignalPayload): BciMessage<SignalPayload> {
    return this.createWithSequence('signal', payload);
  }

  /**
   * Create signal batch message
   */
  signalBatch(payload: SignalBatchPayload): BciMessage<SignalBatchPayload> {
    return this.createWithSequence('signal_batch', payload);
  }

  // Marker message

  /**
   * Create marker message
   */
  marker(payload: MarkerPayload): BciMessage<MarkerPayload> {
    return this.createWithSequence('marker', payload);
  }

  // Command messages

  /**
   * Create command message
   */
  command(payload: CommandPayload): BciMessage<CommandPayload> {
    return this.createBase('command', payload);
  }

  /**
   * Create command ack message
   */
  commandAck(payload: CommandAckPayload): BciMessage<CommandAckPayload> {
    return this.createBase('command_ack', payload);
  }

  // Config message

  /**
   * Create config message
   */
  config(payload: ConfigPayload): BciMessage<ConfigPayload> {
    return this.createBase('config', payload);
  }

  // Status message

  /**
   * Create status message
   */
  status(payload: StatusPayload): BciMessage<StatusPayload> {
    return this.createBase('status', payload);
  }

  // Error message

  /**
   * Create error message
   */
  error(payload: ErrorPayload): BciMessage<ErrorPayload> {
    return this.createBase('error', payload);
  }

  // Heartbeat messages

  /**
   * Create ping message
   */
  ping(payload: PingPayload = {}): BciMessage<PingPayload> {
    return this.createBase('ping', {
      ...payload,
      clientTime: Date.now(),
    });
  }

  /**
   * Create pong message
   */
  pong(clientTime?: number): BciMessage<PongPayload> {
    return this.createBase('pong', {
      serverTime: Date.now(),
      clientTime,
    });
  }
}

/**
 * Default message builder instance
 */
export const messageBuilder = new MessageBuilder();
