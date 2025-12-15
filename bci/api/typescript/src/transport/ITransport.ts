/**
 * WIA BCI Transport Interface
 *
 * Phase 3: Transport Layer Abstraction
 */

import { BciMessage, TransportOptions } from '../protocol/types';

/**
 * Transport event handlers
 */
export interface TransportEventHandlers {
  onOpen?: () => void;
  onClose?: (code: number, reason: string) => void;
  onError?: (error: Error) => void;
  onMessage?: (message: BciMessage) => void;
}

/**
 * Transport state
 */
export type TransportState = 'disconnected' | 'connecting' | 'connected' | 'closing';

/**
 * Transport interface
 *
 * All transport implementations must implement this interface.
 */
export interface ITransport {
  /**
   * Get transport state
   */
  getState(): TransportState;

  /**
   * Check if connected
   */
  isConnected(): boolean;

  /**
   * Connect to server
   */
  connect(url: string, options?: TransportOptions): Promise<void>;

  /**
   * Disconnect from server
   */
  disconnect(): Promise<void>;

  /**
   * Send message
   */
  send(message: BciMessage): Promise<void>;

  /**
   * Set message handler
   */
  onMessage(handler: (message: BciMessage) => void): void;

  /**
   * Set open handler
   */
  onOpen(handler: () => void): void;

  /**
   * Set close handler
   */
  onClose(handler: (code: number, reason: string) => void): void;

  /**
   * Set error handler
   */
  onError(handler: (error: Error) => void): void;

  /**
   * Dispose transport
   */
  dispose(): void;
}

/**
 * Base transport class with common functionality
 */
export abstract class BaseTransport implements ITransport {
  protected state: TransportState = 'disconnected';
  protected handlers: TransportEventHandlers = {};
  protected options: TransportOptions = {};

  getState(): TransportState {
    return this.state;
  }

  isConnected(): boolean {
    return this.state === 'connected';
  }

  abstract connect(url: string, options?: TransportOptions): Promise<void>;
  abstract disconnect(): Promise<void>;
  abstract send(message: BciMessage): Promise<void>;

  onMessage(handler: (message: BciMessage) => void): void {
    this.handlers.onMessage = handler;
  }

  onOpen(handler: () => void): void {
    this.handlers.onOpen = handler;
  }

  onClose(handler: (code: number, reason: string) => void): void {
    this.handlers.onClose = handler;
  }

  onError(handler: (error: Error) => void): void {
    this.handlers.onError = handler;
  }

  protected emitOpen(): void {
    this.handlers.onOpen?.();
  }

  protected emitClose(code: number, reason: string): void {
    this.handlers.onClose?.(code, reason);
  }

  protected emitError(error: Error): void {
    this.handlers.onError?.(error);
  }

  protected emitMessage(message: BciMessage): void {
    this.handlers.onMessage?.(message);
  }

  dispose(): void {
    this.handlers = {};
  }
}
