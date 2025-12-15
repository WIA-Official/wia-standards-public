/**
 * WIA AAC Event Emitter
 * Type-safe event emitter for AAC signals and events
 */

import { EventType, EventDataMap, EventHandler, EventHandlerOptions } from '../types';

interface HandlerEntry {
  handler: EventHandler<EventType>;
  options: EventHandlerOptions<EventType>;
}

export class WiaAacEventEmitter {
  private handlers: Map<EventType, Set<HandlerEntry>> = new Map();

  /**
   * Subscribe to an event
   */
  on<T extends EventType>(
    event: T,
    handler: EventHandler<T>,
    options: EventHandlerOptions<T> = {}
  ): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add({
      handler: handler as EventHandler<EventType>,
      options: options as EventHandlerOptions<EventType>
    });
  }

  /**
   * Unsubscribe from an event
   */
  off<T extends EventType>(event: T, handler: EventHandler<T>): void {
    const handlers = this.handlers.get(event);
    if (!handlers) return;

    for (const entry of handlers) {
      if (entry.handler === handler) {
        handlers.delete(entry);
        break;
      }
    }
  }

  /**
   * Subscribe to an event once
   */
  once<T extends EventType>(event: T, handler: EventHandler<T>): void {
    this.on(event, handler, { once: true });
  }

  /**
   * Emit an event
   */
  emit<T extends EventType>(event: T, data: EventDataMap[T]): void {
    const handlers = this.handlers.get(event);
    if (!handlers) return;

    const toRemove: HandlerEntry[] = [];

    for (const entry of handlers) {
      // Apply filter if exists
      if (entry.options.filter && !entry.options.filter(data)) {
        continue;
      }

      // Call handler
      try {
        entry.handler(data);
      } catch (error) {
        console.error(`Error in event handler for '${event}':`, error);
      }

      // Mark for removal if once
      if (entry.options.once) {
        toRemove.push(entry);
      }
    }

    // Remove once handlers
    for (const entry of toRemove) {
      handlers.delete(entry);
    }
  }

  /**
   * Remove all handlers for an event or all events
   */
  removeAllListeners(event?: EventType): void {
    if (event) {
      this.handlers.delete(event);
    } else {
      this.handlers.clear();
    }
  }

  /**
   * Get listener count for an event
   */
  listenerCount(event: EventType): number {
    return this.handlers.get(event)?.size ?? 0;
  }
}
