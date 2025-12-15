/**
 * WIA AAC Output Manager
 * Phase 4: WIA Ecosystem Integration
 *
 * Unified output management for all adapters
 */

import { IOutputAdapter } from './IOutputAdapter';
import {
  OutputType,
  OutputOptions,
  OutputError,
  OutputErrorCode,
  OutputEventType,
  OutputEventHandler
} from './types';

/**
 * Output manager interface
 */
export interface IOutputManager {
  /**
   * Register an adapter
   */
  register(adapter: IOutputAdapter): void;

  /**
   * Unregister an adapter
   */
  unregister(type: OutputType): void;

  /**
   * Get adapter by type
   */
  getAdapter<T extends IOutputAdapter>(type: OutputType): T | undefined;

  /**
   * Get all active adapters
   */
  getActiveAdapters(): IOutputAdapter[];

  /**
   * Broadcast to all active adapters
   */
  broadcast(text: string, options?: OutputOptions): Promise<void>;

  /**
   * Output to specific adapter
   */
  outputTo(type: OutputType, text: string, options?: OutputOptions): Promise<void>;

  /**
   * Stop all output
   */
  stopAll(): void;

  /**
   * Dispose all resources
   */
  dispose(): Promise<void>;
}

/**
 * Output manager events
 */
export type OutputManagerEventType =
  | 'adapterRegistered'
  | 'adapterUnregistered'
  | 'outputStart'
  | 'outputEnd'
  | 'error';

export type OutputManagerEventHandler = (data: unknown) => void;

/**
 * Output Manager
 * Manages multiple output adapters and provides unified interface
 */
export class OutputManager implements IOutputManager {
  private adapters: Map<OutputType, IOutputAdapter> = new Map();
  private handlers: Map<OutputManagerEventType, Set<OutputManagerEventHandler>> = new Map();

  /**
   * Register an output adapter
   */
  register(adapter: IOutputAdapter): void {
    if (this.adapters.has(adapter.type)) {
      // Dispose existing adapter first
      const existing = this.adapters.get(adapter.type);
      existing?.dispose();
    }

    this.adapters.set(adapter.type, adapter);
    this.emit('adapterRegistered', {
      type: adapter.type,
      name: adapter.name
    });
  }

  /**
   * Unregister an output adapter
   */
  unregister(type: OutputType): void {
    const adapter = this.adapters.get(type);
    if (adapter) {
      adapter.dispose();
      this.adapters.delete(type);
      this.emit('adapterUnregistered', { type });
    }
  }

  /**
   * Get adapter by type
   */
  getAdapter<T extends IOutputAdapter>(type: OutputType): T | undefined {
    return this.adapters.get(type) as T | undefined;
  }

  /**
   * Get all active adapters
   */
  getActiveAdapters(): IOutputAdapter[] {
    return Array.from(this.adapters.values()).filter(a => a.isAvailable());
  }

  /**
   * Get all registered adapters (including unavailable)
   */
  getAllAdapters(): IOutputAdapter[] {
    return Array.from(this.adapters.values());
  }

  /**
   * Check if an adapter type is registered
   */
  hasAdapter(type: OutputType): boolean {
    return this.adapters.has(type);
  }

  /**
   * Broadcast to all active adapters
   * Errors from individual adapters are caught and emitted as events
   */
  async broadcast(text: string, options?: OutputOptions): Promise<void> {
    const activeAdapters = this.getActiveAdapters();

    if (activeAdapters.length === 0) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_FOUND,
        'No active adapters available',
        true
      );
    }

    this.emit('outputStart', {
      text,
      adapters: activeAdapters.map(a => a.type)
    });

    const results = await Promise.allSettled(
      activeAdapters.map(adapter =>
        adapter.output(text, options)
      )
    );

    // Collect errors
    const errors: { type: OutputType; error: unknown }[] = [];
    results.forEach((result, index) => {
      if (result.status === 'rejected') {
        const adapter = activeAdapters[index];
        errors.push({
          type: adapter.type,
          error: result.reason
        });
        this.emit('error', {
          adapter: adapter.type,
          error: result.reason
        });
      }
    });

    this.emit('outputEnd', {
      text,
      errors: errors.length > 0 ? errors : undefined
    });
  }

  /**
   * Output to specific adapter
   */
  async outputTo(
    type: OutputType,
    text: string,
    options?: OutputOptions
  ): Promise<void> {
    const adapter = this.adapters.get(type);

    if (!adapter) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_FOUND,
        `Adapter not found: ${type}`,
        true
      );
    }

    if (!adapter.isAvailable()) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        `Adapter not available: ${type}`,
        true
      );
    }

    this.emit('outputStart', {
      text,
      adapters: [type]
    });

    try {
      await adapter.output(text, options);
      this.emit('outputEnd', { text });
    } catch (error) {
      this.emit('error', { adapter: type, error });
      throw error;
    }
  }

  /**
   * Output to multiple specific adapters
   */
  async outputToMultiple(
    types: OutputType[],
    text: string,
    options?: OutputOptions
  ): Promise<void> {
    const adapters = types
      .map(type => this.adapters.get(type))
      .filter((a): a is IOutputAdapter => a !== undefined && a.isAvailable());

    if (adapters.length === 0) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_FOUND,
        'No specified adapters available',
        true
      );
    }

    this.emit('outputStart', {
      text,
      adapters: adapters.map(a => a.type)
    });

    const results = await Promise.allSettled(
      adapters.map(adapter => adapter.output(text, options))
    );

    const errors: { type: OutputType; error: unknown }[] = [];
    results.forEach((result, index) => {
      if (result.status === 'rejected') {
        errors.push({
          type: adapters[index].type,
          error: result.reason
        });
      }
    });

    this.emit('outputEnd', {
      text,
      errors: errors.length > 0 ? errors : undefined
    });

    if (errors.length > 0) {
      throw new OutputError(
        OutputErrorCode.OUTPUT_FAILED,
        `Output failed for: ${errors.map(e => e.type).join(', ')}`,
        true,
        errors
      );
    }
  }

  /**
   * Stop all output
   */
  stopAll(): void {
    for (const adapter of this.adapters.values()) {
      adapter.stop();
    }
  }

  /**
   * Dispose all resources
   */
  async dispose(): Promise<void> {
    for (const adapter of this.adapters.values()) {
      await adapter.dispose();
    }
    this.adapters.clear();
    this.handlers.clear();
  }

  /**
   * Register event handler
   */
  on(event: OutputManagerEventType, handler: OutputManagerEventHandler): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  off(event: OutputManagerEventType, handler: OutputManagerEventHandler): void {
    const eventHandlers = this.handlers.get(event);
    if (eventHandlers) {
      eventHandlers.delete(handler);
    }
  }

  /**
   * Emit event
   */
  private emit(event: OutputManagerEventType, data?: unknown): void {
    const eventHandlers = this.handlers.get(event);
    if (eventHandlers) {
      eventHandlers.forEach(handler => handler(data));
    }
  }
}
