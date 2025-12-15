/**
 * WIA AAC Protocol Handler
 * Handles protocol-level operations (connection, subscription, commands)
 */

import { SimpleEventEmitter } from './SimpleEventEmitter';
import { MessageBuilder } from './MessageBuilder';
import { MessageParser, ParseResult } from './MessageParser';
import {
  WiaAacMessage,
  MessageType,
  ConnectPayload,
  ConnectAckPayload,
  SubscribePayload,
  CommandPayload,
  ProtocolError,
  ErrorCodes,
  PROTOCOL_VERSION,
} from './message';
import { ITransport } from '../transport/ITransport';
import { WiaAacSignal, SensorType } from '../types/signal';

// Protocol states
export type ProtocolState =
  | 'disconnected'
  | 'connecting'
  | 'connected'
  | 'reconnecting'
  | 'error';

// Protocol events
export type ProtocolEventType =
  | 'stateChange'
  | 'signal'
  | 'error'
  | 'message';

export interface ProtocolOptions {
  autoReconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  heartbeatInterval?: number;
  heartbeatTimeout?: number;
}

const DEFAULT_OPTIONS: Required<ProtocolOptions> = {
  autoReconnect: true,
  reconnectInterval: 1000,
  maxReconnectAttempts: 5,
  heartbeatInterval: 30000,
  heartbeatTimeout: 10000,
};

export class ProtocolHandler {
  private transport: ITransport | null = null;
  private builder: MessageBuilder;
  private parser: MessageParser;
  private emitter: SimpleEventEmitter;
  private options: Required<ProtocolOptions>;

  private state: ProtocolState = 'disconnected';
  private sessionId: string | null = null;
  private subscriptionId: string | null = null;
  private reconnectAttempts = 0;
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private heartbeatSequence = 0;
  private pendingResponses: Map<string, {
    resolve: (msg: WiaAacMessage) => void;
    reject: (error: ProtocolError) => void;
    timeout: NodeJS.Timeout;
  }> = new Map();

  constructor(options: ProtocolOptions = {}) {
    this.options = { ...DEFAULT_OPTIONS, ...options };
    this.builder = new MessageBuilder();
    this.parser = new MessageParser();
    this.emitter = new SimpleEventEmitter();
  }

  /**
   * Set the transport layer
   */
  setTransport(transport: ITransport): void {
    if (this.transport) {
      this.transport.onMessage(() => {});
      this.transport.onClose(() => {});
      this.transport.onError(() => {});
    }

    this.transport = transport;

    // Setup transport event handlers
    this.transport.onMessage((data) => this.handleMessage(data));
    this.transport.onClose((reason) => this.handleClose(reason));
    this.transport.onError((error) => this.handleError(error));
  }

  /**
   * Connect to a server
   */
  async connect(url: string, payload: ConnectPayload): Promise<ConnectAckPayload> {
    if (!this.transport) {
      throw new Error('Transport not set. Call setTransport() first.');
    }

    this.setState('connecting');

    try {
      // Connect transport
      await this.transport.connect(url);

      // Send connect message
      const connectMsg = this.builder.connect(payload);
      const response = await this.sendAndWait<ConnectAckPayload>(connectMsg, 'connect_ack');

      if (response.payload.success) {
        this.sessionId = response.payload.sessionId || null;
        this.setState('connected');
        this.startHeartbeat();
        this.reconnectAttempts = 0;
      } else {
        this.setState('error');
        throw response.payload.error || { code: ErrorCodes.CONNECTION_LOST, name: 'CONNECTION_FAILED', message: 'Connection failed', recoverable: true };
      }

      return response.payload;
    } catch (error) {
      this.setState('error');
      throw error;
    }
  }

  /**
   * Disconnect from the server
   */
  async disconnect(reason: 'user_request' | 'error' = 'user_request'): Promise<void> {
    if (!this.transport || this.state === 'disconnected') {
      return;
    }

    this.stopHeartbeat();

    try {
      const disconnectMsg = this.builder.disconnect(reason);
      await this.transport.send(disconnectMsg);
    } catch (e) {
      // Ignore send errors during disconnect
    }

    await this.transport.disconnect();
    this.setState('disconnected');
    this.sessionId = null;
    this.subscriptionId = null;
  }

  /**
   * Subscribe to sensor streams
   */
  async subscribe(payload: SubscribePayload): Promise<string | null> {
    if (this.state !== 'connected') {
      throw new Error('Not connected');
    }

    const subscribeMsg = this.builder.subscribe(payload);
    const response = await this.sendAndWait<any>(subscribeMsg, 'subscribe_ack');

    if (response.payload.success) {
      this.subscriptionId = response.payload.subscriptionId || null;
      return this.subscriptionId;
    } else {
      throw response.payload.error || { code: ErrorCodes.SUBSCRIPTION_FAILED, name: 'SUBSCRIPTION_FAILED', message: 'Subscription failed', recoverable: true };
    }
  }

  /**
   * Unsubscribe from sensor streams
   */
  async unsubscribe(): Promise<void> {
    if (this.state !== 'connected' || !this.subscriptionId) {
      return;
    }

    const unsubscribeMsg = this.builder.unsubscribe({ subscriptionId: this.subscriptionId });
    await this.sendAndWait(unsubscribeMsg, 'unsubscribe_ack');
    this.subscriptionId = null;
  }

  /**
   * Send a command to the sensor
   */
  async sendCommand(payload: CommandPayload): Promise<unknown> {
    if (this.state !== 'connected') {
      throw new Error('Not connected');
    }

    const commandMsg = this.builder.command(payload);
    const response = await this.sendAndWait<any>(commandMsg, 'command_ack');

    if (response.payload.success) {
      return response.payload.result;
    } else {
      throw response.payload.error || { code: ErrorCodes.SENSOR_ERROR, name: 'COMMAND_FAILED', message: 'Command failed', recoverable: true };
    }
  }

  /**
   * Get current protocol state
   */
  getState(): ProtocolState {
    return this.state;
  }

  /**
   * Get session ID
   */
  getSessionId(): string | null {
    return this.sessionId;
  }

  /**
   * Subscribe to protocol events
   */
  on(event: ProtocolEventType, handler: (data: any) => void): void {
    this.emitter.on(event, handler);
  }

  /**
   * Unsubscribe from protocol events
   */
  off(event: ProtocolEventType, handler: (data: any) => void): void {
    this.emitter.off(event, handler);
  }

  // ========================================
  // Private methods
  // ========================================

  private setState(newState: ProtocolState): void {
    if (this.state !== newState) {
      const oldState = this.state;
      this.state = newState;
      this.emitter.emit('stateChange', { oldState, newState });
    }
  }

  private async sendAndWait<T>(
    message: WiaAacMessage,
    expectedType: MessageType,
    timeout: number = 10000
  ): Promise<WiaAacMessage<T>> {
    return new Promise((resolve, reject) => {
      const timeoutHandle = setTimeout(() => {
        this.pendingResponses.delete(message.messageId);
        reject(MessageBuilder.createError(
          ErrorCodes.CONNECTION_TIMEOUT,
          'TIMEOUT',
          `Timeout waiting for ${expectedType}`,
          true,
          { messageId: message.messageId }
        ));
      }, timeout);

      this.pendingResponses.set(message.messageId, {
        resolve: resolve as (msg: WiaAacMessage) => void,
        reject,
        timeout: timeoutHandle,
      });

      this.transport!.send(message).catch((error) => {
        this.pendingResponses.delete(message.messageId);
        clearTimeout(timeoutHandle);
        reject(error);
      });
    });
  }

  private handleMessage(data: string | WiaAacMessage): void {
    const result = this.parser.parse(data);

    if (!result.success || !result.message) {
      this.emitter.emit('error', result.error);
      return;
    }

    const message = result.message;

    // Check for pending response
    // For ack messages, try to match with a pending request
    if (message.type.endsWith('_ack')) {
      // Find the matching request by checking recent pending responses
      for (const [messageId, pending] of this.pendingResponses) {
        // Match based on message type correspondence
        const expectedRequestType = message.type.replace('_ack', '');
        clearTimeout(pending.timeout);
        this.pendingResponses.delete(messageId);
        pending.resolve(message);
        return;
      }
    }

    // Handle specific message types
    switch (message.type) {
      case 'signal':
        this.emitter.emit('signal', message.payload);
        break;

      case 'error':
        this.emitter.emit('error', message.payload);
        break;

      case 'ping':
        this.sendPong((message.payload as { sequence: number }).sequence);
        break;

      case 'pong':
        // Heartbeat response received
        break;

      case 'disconnect':
        this.handleDisconnect(message);
        break;

      default:
        this.emitter.emit('message', message);
    }
  }

  private handleClose(reason: string): void {
    this.stopHeartbeat();

    if (this.state === 'connected' && this.options.autoReconnect) {
      this.attemptReconnect();
    } else {
      this.setState('disconnected');
    }
  }

  private handleError(error: Error): void {
    this.emitter.emit('error', MessageBuilder.createError(
      ErrorCodes.CONNECTION_LOST,
      'TRANSPORT_ERROR',
      error.message,
      true
    ));
  }

  private handleDisconnect(message: WiaAacMessage): void {
    this.stopHeartbeat();
    this.setState('disconnected');
    this.sessionId = null;
  }

  private async attemptReconnect(): Promise<void> {
    if (this.reconnectAttempts >= this.options.maxReconnectAttempts) {
      this.setState('error');
      this.emitter.emit('error', MessageBuilder.createError(
        ErrorCodes.CONNECTION_LOST,
        'MAX_RECONNECT_ATTEMPTS',
        'Maximum reconnection attempts exceeded',
        false
      ));
      return;
    }

    this.setState('reconnecting');
    this.reconnectAttempts++;

    const delay = Math.min(
      this.options.reconnectInterval * Math.pow(2, this.reconnectAttempts - 1),
      30000
    );

    await new Promise((resolve) => setTimeout(resolve, delay));

    // Reconnection logic would go here
    // This requires storing the original URL and connect payload
  }

  private startHeartbeat(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
    }

    this.heartbeatTimer = setInterval(() => {
      this.sendPing();
    }, this.options.heartbeatInterval);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  private async sendPing(): Promise<void> {
    if (!this.transport || this.state !== 'connected') {
      return;
    }

    const pingMsg = this.builder.ping(++this.heartbeatSequence);
    try {
      await this.transport.send(pingMsg);
    } catch (e) {
      // Ping failed, connection might be lost
    }
  }

  private async sendPong(sequence: number): Promise<void> {
    if (!this.transport || this.state !== 'connected') {
      return;
    }

    const pongMsg = this.builder.pong(sequence);
    try {
      await this.transport.send(pongMsg);
    } catch (e) {
      // Pong failed
    }
  }
}
