/**
 * WIA AAC Output Adapter Interface
 * Phase 4: WIA Ecosystem Integration
 */

import {
  OutputType,
  OutputState,
  OutputOptions,
  OutputEventType,
  OutputEventHandler
} from './types';

/**
 * Base output adapter interface
 */
export interface IOutputAdapter {
  /** Output type */
  readonly type: OutputType;

  /** Adapter name */
  readonly name: string;

  /** Current state */
  readonly state: OutputState;

  /**
   * Initialize the adapter
   * @param options - Output options
   */
  initialize(options?: OutputOptions): Promise<void>;

  /**
   * Output text
   * @param text - Text to output
   * @param options - Output options
   */
  output(text: string, options?: OutputOptions): Promise<void>;

  /**
   * Stop output
   */
  stop(): void;

  /**
   * Check if adapter is available
   */
  isAvailable(): boolean;

  /**
   * Dispose resources
   */
  dispose(): Promise<void>;

  /**
   * Register event handler
   * @param event - Event type
   * @param handler - Event handler
   */
  on(event: OutputEventType, handler: OutputEventHandler): void;

  /**
   * Unregister event handler
   * @param event - Event type
   * @param handler - Event handler
   */
  off(event: OutputEventType, handler: OutputEventHandler): void;
}

/**
 * Base output adapter class
 */
export abstract class BaseOutputAdapter implements IOutputAdapter {
  abstract readonly type: OutputType;
  abstract readonly name: string;

  protected _state: OutputState = 'idle';
  protected handlers: Map<OutputEventType, Set<OutputEventHandler>> = new Map();

  get state(): OutputState {
    return this._state;
  }

  abstract initialize(options?: OutputOptions): Promise<void>;
  abstract output(text: string, options?: OutputOptions): Promise<void>;
  abstract stop(): void;
  abstract isAvailable(): boolean;
  abstract dispose(): Promise<void>;

  on(event: OutputEventType, handler: OutputEventHandler): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  off(event: OutputEventType, handler: OutputEventHandler): void {
    const eventHandlers = this.handlers.get(event);
    if (eventHandlers) {
      eventHandlers.delete(handler);
    }
  }

  protected emit(event: OutputEventType, data?: unknown): void {
    const eventHandlers = this.handlers.get(event);
    if (eventHandlers) {
      const outputEvent = {
        type: event,
        adapter: this.type,
        timestamp: Date.now(),
        data
      };
      eventHandlers.forEach(handler => handler(outputEvent));
    }
  }
}
