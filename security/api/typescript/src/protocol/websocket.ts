/**
 * WIA Security WebSocket Protocol
 * Real-time event streaming
 */

import { WiaSecurityEvent, EventType } from '../types';
import { validateEvent } from '../validator';

// ============================================================================
// Types
// ============================================================================

export interface WebSocketConfig {
  url: string;
  reconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  heartbeatInterval?: number;
  authToken?: string;
}

export interface WebSocketMessage {
  type: 'event' | 'subscribe' | 'unsubscribe' | 'ack' | 'error' | 'heartbeat';
  payload?: unknown;
  id?: string;
  timestamp?: string;
}

export interface SubscriptionFilter {
  eventTypes?: EventType[];
  severityMin?: number;
  severityMax?: number;
  tags?: string[];
}

export type EventHandler = (event: WiaSecurityEvent) => void;
export type ErrorHandler = (error: Error) => void;
export type ConnectionHandler = () => void;

// ============================================================================
// WebSocket Client
// ============================================================================

export class WiaWebSocketClient {
  private config: WebSocketConfig;
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private eventHandlers: Map<string, EventHandler[]> = new Map();
  private onError: ErrorHandler | null = null;
  private onConnect: ConnectionHandler | null = null;
  private onDisconnect: ConnectionHandler | null = null;
  private subscriptions: Map<string, SubscriptionFilter> = new Map();

  constructor(config: WebSocketConfig) {
    this.config = {
      reconnect: true,
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      heartbeatInterval: 30000,
      ...config
    };
  }

  /**
   * Connect to WebSocket server
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        const url = new URL(this.config.url);
        if (this.config.authToken) {
          url.searchParams.set('token', this.config.authToken);
        }

        this.ws = new WebSocket(url.toString());

        this.ws.onopen = () => {
          this.reconnectAttempts = 0;
          this.startHeartbeat();
          this.resubscribe();
          this.onConnect?.();
          resolve();
        };

        this.ws.onmessage = (event) => {
          this.handleMessage(event.data);
        };

        this.ws.onerror = (event) => {
          const error = new Error('WebSocket error');
          this.onError?.(error);
          reject(error);
        };

        this.ws.onclose = () => {
          this.stopHeartbeat();
          this.onDisconnect?.();
          this.attemptReconnect();
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Disconnect from WebSocket server
   */
  disconnect(): void {
    this.config.reconnect = false;
    this.stopHeartbeat();
    this.ws?.close();
    this.ws = null;
  }

  /**
   * Subscribe to events
   */
  subscribe(channel: string, filter?: SubscriptionFilter): void {
    this.subscriptions.set(channel, filter || {});

    if (this.isConnected()) {
      this.send({
        type: 'subscribe',
        payload: { channel, filter }
      });
    }
  }

  /**
   * Unsubscribe from events
   */
  unsubscribe(channel: string): void {
    this.subscriptions.delete(channel);

    if (this.isConnected()) {
      this.send({
        type: 'unsubscribe',
        payload: { channel }
      });
    }
  }

  /**
   * Send an event
   */
  sendEvent(event: WiaSecurityEvent): void {
    const result = validateEvent(event);
    if (!result.valid) {
      throw new Error(`Invalid event: ${result.errors.join(', ')}`);
    }

    this.send({
      type: 'event',
      payload: event,
      id: event.id,
      timestamp: event.timestamp
    });
  }

  /**
   * Register event handler
   */
  on(eventType: string, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType) || [];
    handlers.push(handler);
    this.eventHandlers.set(eventType, handlers);
  }

  /**
   * Remove event handler
   */
  off(eventType: string, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType) || [];
    const index = handlers.indexOf(handler);
    if (index > -1) {
      handlers.splice(index, 1);
      this.eventHandlers.set(eventType, handlers);
    }
  }

  /**
   * Set error handler
   */
  onErrorHandler(handler: ErrorHandler): void {
    this.onError = handler;
  }

  /**
   * Set connection handler
   */
  onConnectHandler(handler: ConnectionHandler): void {
    this.onConnect = handler;
  }

  /**
   * Set disconnection handler
   */
  onDisconnectHandler(handler: ConnectionHandler): void {
    this.onDisconnect = handler;
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  // -------------------------------------------------------------------------
  // Private Methods
  // -------------------------------------------------------------------------

  private send(message: WebSocketMessage): void {
    if (this.isConnected()) {
      this.ws!.send(JSON.stringify(message));
    }
  }

  private handleMessage(data: string): void {
    try {
      const message: WebSocketMessage = JSON.parse(data);

      switch (message.type) {
        case 'event':
          this.handleEvent(message.payload as WiaSecurityEvent);
          break;
        case 'ack':
          // Handle acknowledgment
          break;
        case 'error':
          this.onError?.(new Error(String(message.payload)));
          break;
        case 'heartbeat':
          // Respond to heartbeat
          this.send({ type: 'heartbeat' });
          break;
      }
    } catch (error) {
      this.onError?.(error as Error);
    }
  }

  private handleEvent(event: WiaSecurityEvent): void {
    // Notify all handlers for this event type
    const handlers = this.eventHandlers.get(event.type) || [];
    handlers.forEach(handler => handler(event));

    // Notify wildcard handlers
    const wildcardHandlers = this.eventHandlers.get('*') || [];
    wildcardHandlers.forEach(handler => handler(event));
  }

  private startHeartbeat(): void {
    if (this.config.heartbeatInterval) {
      this.heartbeatTimer = setInterval(() => {
        this.send({ type: 'heartbeat' });
      }, this.config.heartbeatInterval);
    }
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  private attemptReconnect(): void {
    if (!this.config.reconnect) return;
    if (this.reconnectAttempts >= (this.config.maxReconnectAttempts || 10)) {
      this.onError?.(new Error('Max reconnection attempts reached'));
      return;
    }

    this.reconnectAttempts++;
    setTimeout(() => {
      this.connect().catch(() => {
        // Reconnection failed, will try again
      });
    }, this.config.reconnectInterval);
  }

  private resubscribe(): void {
    for (const [channel, filter] of this.subscriptions) {
      this.send({
        type: 'subscribe',
        payload: { channel, filter }
      });
    }
  }
}

// ============================================================================
// Factory
// ============================================================================

export function createWebSocketClient(config: WebSocketConfig): WiaWebSocketClient {
  return new WiaWebSocketClient(config);
}
