/**
 * WIA AAC WebSocket Transport
 * WebSocket implementation of the transport interface
 */

import {
  ITransport,
  TransportState,
  TransportConfig,
  DEFAULT_TRANSPORT_CONFIG,
  MessageHandler,
  OpenHandler,
  CloseHandler,
  ErrorHandler,
} from './ITransport';
import { WiaAacMessage, SUB_PROTOCOL } from '../protocol/message';

export class WebSocketTransport implements ITransport {
  private ws: WebSocket | null = null;
  private config: Required<TransportConfig>;
  private state: TransportState = 'disconnected';

  private messageHandler: MessageHandler = () => {};
  private openHandler: OpenHandler = () => {};
  private closeHandler: CloseHandler = () => {};
  private errorHandler: ErrorHandler = () => {};

  constructor(config: TransportConfig = {}) {
    this.config = { ...DEFAULT_TRANSPORT_CONFIG, ...config };
  }

  /**
   * Connect to WebSocket server
   */
  async connect(url: string): Promise<void> {
    if (this.state === 'connected') {
      throw new Error('Already connected');
    }

    this.state = 'connecting';

    return new Promise((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        if (this.ws) {
          this.ws.close();
        }
        this.state = 'error';
        reject(new Error('Connection timeout'));
      }, this.config.connectionTimeout);

      try {
        // Create WebSocket with sub-protocol
        this.ws = new WebSocket(url, [this.config.subProtocol || SUB_PROTOCOL]);

        this.ws.onopen = () => {
          clearTimeout(timeoutId);
          this.state = 'connected';
          this.openHandler();
          resolve();
        };

        this.ws.onmessage = (event) => {
          this.handleMessage(event.data);
        };

        this.ws.onclose = (event) => {
          clearTimeout(timeoutId);
          this.state = 'disconnected';
          this.closeHandler(event.reason || `Code: ${event.code}`);
        };

        this.ws.onerror = (event) => {
          clearTimeout(timeoutId);
          const wasConnecting = this.state === 'connecting';
          this.state = 'error';
          const error = new Error('WebSocket error');
          this.errorHandler(error);
          if (wasConnecting) {
            reject(error);
          }
        };
      } catch (error) {
        clearTimeout(timeoutId);
        this.state = 'error';
        reject(error);
      }
    });
  }

  /**
   * Disconnect from WebSocket server
   */
  async disconnect(): Promise<void> {
    if (!this.ws || this.state === 'disconnected') {
      return;
    }

    return new Promise((resolve) => {
      if (!this.ws) {
        resolve();
        return;
      }

      const originalOnClose = this.ws.onclose;
      this.ws.onclose = (event: Event) => {
        this.state = 'disconnected';
        if (originalOnClose) {
          (originalOnClose as (event: Event) => void)(event);
        }
        resolve();
      };

      this.ws.close(1000, 'Client disconnect');
    });
  }

  /**
   * Send a message
   */
  async send(message: WiaAacMessage): Promise<void> {
    if (!this.ws || this.state !== 'connected') {
      throw new Error('Not connected');
    }

    const data = JSON.stringify(message);
    this.ws.send(data);
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.state === 'connected' && this.ws !== null;
  }

  /**
   * Get transport state
   */
  getState(): TransportState {
    return this.state;
  }

  /**
   * Set message handler
   */
  onMessage(handler: MessageHandler): void {
    this.messageHandler = handler;
  }

  /**
   * Set open handler
   */
  onOpen(handler: OpenHandler): void {
    this.openHandler = handler;
  }

  /**
   * Set close handler
   */
  onClose(handler: CloseHandler): void {
    this.closeHandler = handler;
  }

  /**
   * Set error handler
   */
  onError(handler: ErrorHandler): void {
    this.errorHandler = handler;
  }

  /**
   * Handle incoming message
   */
  private handleMessage(data: string | ArrayBuffer | Blob): void {
    if (typeof data === 'string') {
      this.messageHandler(data);
    } else if (data instanceof ArrayBuffer) {
      // Handle binary data
      const text = new TextDecoder().decode(data);
      this.messageHandler(text);
    } else if (data instanceof Blob) {
      // Handle Blob data
      data.text().then((text) => {
        this.messageHandler(text);
      });
    }
  }
}
