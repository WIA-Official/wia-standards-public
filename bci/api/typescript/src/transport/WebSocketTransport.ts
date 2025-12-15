/**
 * WIA BCI WebSocket Transport
 *
 * WebSocket-based transport implementation.
 */

import { BaseTransport } from './ITransport';
import { BciMessage, TransportOptions, HeartbeatConfig, ReconnectConfig } from '../protocol/types';
import { MessageParser, serialize, serializeBinary } from '../protocol/MessageParser';

/**
 * WebSocket transport options
 */
export interface WebSocketTransportOptions extends TransportOptions {
  protocols?: string[];
  binaryMode?: boolean;
  heartbeat?: HeartbeatConfig;
  reconnect?: ReconnectConfig;
}

/**
 * Default configurations
 */
const DEFAULT_HEARTBEAT: HeartbeatConfig = {
  pingInterval: 30000,
  pongTimeout: 10000,
  maxMissedPongs: 3,
};

const DEFAULT_RECONNECT: ReconnectConfig = {
  enabled: true,
  maxAttempts: 5,
  initialDelay: 1000,
  maxDelay: 30000,
  backoffMultiplier: 2,
};

/**
 * WebSocket transport implementation
 */
export class WebSocketTransport extends BaseTransport {
  private ws: WebSocket | null = null;
  private parser: MessageParser;
  private url: string = '';
  private binaryMode: boolean = false;
  private heartbeatConfig: HeartbeatConfig;
  private reconnectConfig: ReconnectConfig;
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private pongTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempts: number = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private missedPongs: number = 0;
  private intentionalClose: boolean = false;

  constructor(options: WebSocketTransportOptions = {}) {
    super();
    this.parser = new MessageParser();
    this.binaryMode = options.binaryMode ?? false;
    this.heartbeatConfig = { ...DEFAULT_HEARTBEAT, ...options.heartbeat };
    this.reconnectConfig = { ...DEFAULT_RECONNECT, ...options.reconnect };
  }

  /**
   * Connect to WebSocket server
   */
  async connect(url: string, options?: WebSocketTransportOptions): Promise<void> {
    if (this.state !== 'disconnected') {
      throw new Error('Transport already connected or connecting');
    }

    this.url = url;
    this.intentionalClose = false;

    if (options?.binaryMode !== undefined) {
      this.binaryMode = options.binaryMode;
    }
    if (options?.heartbeat) {
      this.heartbeatConfig = { ...this.heartbeatConfig, ...options.heartbeat };
    }
    if (options?.reconnect) {
      this.reconnectConfig = { ...this.reconnectConfig, ...options.reconnect };
    }

    return this.doConnect();
  }

  /**
   * Internal connect
   */
  private doConnect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.state = 'connecting';

      try {
        this.ws = new WebSocket(this.url, ['wia-bci-v1']);
        this.ws.binaryType = 'arraybuffer';

        this.ws.onopen = () => {
          this.state = 'connected';
          this.reconnectAttempts = 0;
          this.startHeartbeat();
          this.emitOpen();
          resolve();
        };

        this.ws.onclose = (event) => {
          this.handleClose(event.code, event.reason);
        };

        this.ws.onerror = (event) => {
          const error = new Error('WebSocket error');
          this.emitError(error);
          if (this.state === 'connecting') {
            reject(error);
          }
        };

        this.ws.onmessage = (event) => {
          this.handleMessage(event.data);
        };
      } catch (err) {
        this.state = 'disconnected';
        reject(err);
      }
    });
  }

  /**
   * Disconnect from server
   */
  async disconnect(): Promise<void> {
    this.intentionalClose = true;
    this.stopHeartbeat();
    this.clearReconnectTimer();

    if (this.ws && this.state === 'connected') {
      this.state = 'closing';
      this.ws.close(1000, 'Client disconnect');
    }

    this.state = 'disconnected';
    this.ws = null;
  }

  /**
   * Send message
   */
  async send(message: BciMessage): Promise<void> {
    if (!this.ws || this.state !== 'connected') {
      throw new Error('Not connected');
    }

    if (this.binaryMode) {
      const buffer = serializeBinary(message);
      this.ws.send(buffer);
    } else {
      const json = serialize(message);
      this.ws.send(json);
    }
  }

  /**
   * Handle incoming message
   */
  private handleMessage(data: string | ArrayBuffer): void {
    const result = this.parser.parse(data);

    if (result.success && result.message) {
      // Handle pong internally
      if (result.message.type === 'pong') {
        this.handlePong();
        return;
      }

      this.emitMessage(result.message);
    } else if (result.error) {
      this.emitError(new Error(result.error.message));
    }
  }

  /**
   * Handle connection close
   */
  private handleClose(code: number, reason: string): void {
    this.stopHeartbeat();

    if (this.intentionalClose) {
      this.state = 'disconnected';
      this.emitClose(code, reason);
      return;
    }

    // Attempt reconnection
    if (this.reconnectConfig.enabled && this.reconnectAttempts < this.reconnectConfig.maxAttempts) {
      this.attemptReconnect();
    } else {
      this.state = 'disconnected';
      this.emitClose(code, reason);
    }
  }

  /**
   * Attempt reconnection
   */
  private attemptReconnect(): void {
    this.reconnectAttempts++;

    const delay = Math.min(
      this.reconnectConfig.initialDelay *
        Math.pow(this.reconnectConfig.backoffMultiplier, this.reconnectAttempts - 1),
      this.reconnectConfig.maxDelay
    );

    this.reconnectTimer = setTimeout(async () => {
      try {
        await this.doConnect();
      } catch {
        // Will be handled by handleClose
      }
    }, delay);
  }

  /**
   * Clear reconnect timer
   */
  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  /**
   * Start heartbeat
   */
  private startHeartbeat(): void {
    this.missedPongs = 0;

    this.pingTimer = setInterval(() => {
      this.sendPing();
    }, this.heartbeatConfig.pingInterval);
  }

  /**
   * Stop heartbeat
   */
  private stopHeartbeat(): void {
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    if (this.pongTimer) {
      clearTimeout(this.pongTimer);
      this.pongTimer = null;
    }
  }

  /**
   * Send ping
   */
  private sendPing(): void {
    if (!this.ws || this.state !== 'connected') return;

    const ping = {
      protocol: 'wia-bci' as const,
      version: '1.0.0',
      messageId: `ping-${Date.now()}`,
      timestamp: Date.now(),
      type: 'ping' as const,
      payload: { clientTime: Date.now() },
    };

    try {
      this.ws.send(serialize(ping));

      // Start pong timeout
      this.pongTimer = setTimeout(() => {
        this.handlePongTimeout();
      }, this.heartbeatConfig.pongTimeout);
    } catch {
      // Connection may be lost
    }
  }

  /**
   * Handle pong response
   */
  private handlePong(): void {
    this.missedPongs = 0;
    if (this.pongTimer) {
      clearTimeout(this.pongTimer);
      this.pongTimer = null;
    }
  }

  /**
   * Handle pong timeout
   */
  private handlePongTimeout(): void {
    this.missedPongs++;

    if (this.missedPongs >= this.heartbeatConfig.maxMissedPongs) {
      // Connection lost
      this.ws?.close(4000, 'Heartbeat timeout');
    }
  }

  /**
   * Dispose transport
   */
  dispose(): void {
    this.disconnect();
    super.dispose();
  }
}
