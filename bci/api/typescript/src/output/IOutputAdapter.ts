/**
 * WIA BCI Output Adapter Interface
 *
 * Phase 4: Ecosystem Integration
 */

import {
  OutputType,
  OutputContent,
  OutputOptions,
  OutputEvent,
  OutputEventHandler,
} from './types';

/**
 * Output adapter interface
 */
export interface IOutputAdapter {
  // Adapter info
  readonly type: OutputType;
  readonly name: string;
  readonly version: string;

  // Initialization
  initialize(options?: OutputOptions): Promise<void>;

  // Output
  output(content: OutputContent): Promise<void>;

  // Status
  isAvailable(): boolean;
  isReady(): boolean;

  // Events
  on(event: OutputEvent, handler: OutputEventHandler): void;
  off(event: OutputEvent, handler: OutputEventHandler): void;

  // Cleanup
  dispose(): Promise<void>;
}

/**
 * Base output adapter with common functionality
 */
export abstract class BaseOutputAdapter implements IOutputAdapter {
  abstract readonly type: OutputType;
  abstract readonly name: string;
  readonly version = '1.0.0';

  protected ready = false;
  protected available = true;
  protected handlers: Map<OutputEvent, Set<OutputEventHandler>> = new Map();

  abstract initialize(options?: OutputOptions): Promise<void>;
  abstract output(content: OutputContent): Promise<void>;

  isAvailable(): boolean {
    return this.available;
  }

  isReady(): boolean {
    return this.ready;
  }

  on(event: OutputEvent, handler: OutputEventHandler): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  off(event: OutputEvent, handler: OutputEventHandler): void {
    this.handlers.get(event)?.delete(handler);
  }

  protected emit(event: OutputEvent, data: Partial<OutputEventData>): void {
    const eventData: OutputEventData = {
      type: event,
      adapter: this.type,
      timestamp: Date.now(),
      ...data,
    };
    this.handlers.get(event)?.forEach((handler) => handler(eventData));
  }

  async dispose(): Promise<void> {
    this.ready = false;
    this.handlers.clear();
  }
}

// Re-export for convenience
interface OutputEventData {
  type: OutputEvent;
  adapter: OutputType;
  timestamp: number;
  content?: OutputContent;
  error?: Error;
}
