/**
 * WIA BCI Event Emitter
 * @module wia-bci/core/EventEmitter
 */

import type { EventType, EventHandler, EventDataMap } from '../types';

/**
 * Type-safe Event Emitter for BCI events
 */
export class BciEventEmitter {
  private listeners: Map<EventType, Set<EventHandler<EventType>>> = new Map();

  /**
   * Subscribe to an event
   */
  on<T extends EventType>(event: T, handler: EventHandler<T>): this {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(handler as EventHandler<EventType>);
    return this;
  }

  /**
   * Unsubscribe from an event
   */
  off<T extends EventType>(event: T, handler: EventHandler<T>): this {
    const handlers = this.listeners.get(event);
    if (handlers) {
      handlers.delete(handler as EventHandler<EventType>);
    }
    return this;
  }

  /**
   * Subscribe to an event once
   */
  once<T extends EventType>(event: T, handler: EventHandler<T>): this {
    const onceHandler = ((data: EventDataMap[T]) => {
      this.off(event, onceHandler as EventHandler<T>);
      (handler as (data: EventDataMap[T]) => void)(data);
    }) as EventHandler<T>;

    return this.on(event, onceHandler);
  }

  /**
   * Emit an event
   */
  emit<T extends EventType>(event: T, data?: EventDataMap[T]): this {
    const handlers = this.listeners.get(event);
    if (handlers) {
      handlers.forEach((handler) => {
        try {
          if (data === undefined) {
            (handler as () => void)();
          } else {
            (handler as (d: EventDataMap[T]) => void)(data);
          }
        } catch (error) {
          console.error(`Error in event handler for '${event}':`, error);
        }
      });
    }
    return this;
  }

  /**
   * Remove all listeners for an event (or all events)
   */
  removeAllListeners(event?: EventType): this {
    if (event) {
      this.listeners.delete(event);
    } else {
      this.listeners.clear();
    }
    return this;
  }

  /**
   * Get listener count for an event
   */
  listenerCount(event: EventType): number {
    return this.listeners.get(event)?.size ?? 0;
  }

  /**
   * Get all events with listeners
   */
  eventNames(): EventType[] {
    return Array.from(this.listeners.keys());
  }
}
