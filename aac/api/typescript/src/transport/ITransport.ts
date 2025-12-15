/**
 * WIA AAC Transport Interface
 * Abstraction layer for different transport mechanisms
 */

import { WiaAacMessage } from '../protocol/message';

export type TransportState = 'disconnected' | 'connecting' | 'connected' | 'error';

export type MessageHandler = (data: string | WiaAacMessage) => void;
export type OpenHandler = () => void;
export type CloseHandler = (reason: string) => void;
export type ErrorHandler = (error: Error) => void;

/**
 * Transport interface for WIA AAC protocol
 */
export interface ITransport {
  /**
   * Connect to the server
   */
  connect(url: string): Promise<void>;

  /**
   * Disconnect from the server
   */
  disconnect(): Promise<void>;

  /**
   * Send a message
   */
  send(message: WiaAacMessage): Promise<void>;

  /**
   * Check if connected
   */
  isConnected(): boolean;

  /**
   * Get transport state
   */
  getState(): TransportState;

  /**
   * Set message handler
   */
  onMessage(handler: MessageHandler): void;

  /**
   * Set open handler
   */
  onOpen(handler: OpenHandler): void;

  /**
   * Set close handler
   */
  onClose(handler: CloseHandler): void;

  /**
   * Set error handler
   */
  onError(handler: ErrorHandler): void;
}

/**
 * Transport configuration
 */
export interface TransportConfig {
  /**
   * Connection timeout in milliseconds
   */
  connectionTimeout?: number;

  /**
   * Whether to use binary messages
   */
  binaryMode?: boolean;

  /**
   * Sub-protocol to use
   */
  subProtocol?: string;
}

export const DEFAULT_TRANSPORT_CONFIG: Required<TransportConfig> = {
  connectionTimeout: 10000,
  binaryMode: false,
  subProtocol: 'wia-aac-v1',
};
