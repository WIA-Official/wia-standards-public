/**
 * WIA BCI Output Manager
 *
 * Central controller for all output adapters.
 */

import { IOutputAdapter } from './IOutputAdapter';
import {
  OutputType,
  OutputContent,
  OutputPreferences,
  ManagerOptions,
  OutputEvent,
  OutputEventHandler,
  OutputEventData,
  OutputError,
  OutputErrorCode,
} from './types';
import { WebSpeechTTSAdapter } from './TTSAdapter';
import { MockSignLanguageAdapter, MockBrailleAdapter } from './MockOutputAdapter';
import { CanvasNeurofeedbackAdapter } from './NeurofeedbackAdapter';

/**
 * Default preferences
 */
const DEFAULT_PREFERENCES: OutputPreferences = {
  primaryOutput: 'tts',
  enabledOutputs: ['tts', 'neurofeedback'],
  language: 'ko',
};

/**
 * Output Manager
 *
 * Manages multiple output adapters and provides unified output interface.
 */
export class OutputManager {
  private adapters: Map<OutputType, IOutputAdapter> = new Map();
  private preferences: OutputPreferences;
  private handlers: Map<string, Set<OutputEventHandler>> = new Map();
  private initialized = false;

  constructor(options?: ManagerOptions) {
    this.preferences = options?.preferences ?? { ...DEFAULT_PREFERENCES };
  }

  /**
   * Initialize manager and all adapters
   */
  async initialize(options?: ManagerOptions): Promise<void> {
    if (options?.preferences) {
      this.preferences = { ...this.preferences, ...options.preferences };
    }

    // Register default adapters
    if (typeof window !== 'undefined' && window.speechSynthesis) {
      this.register(new WebSpeechTTSAdapter());
    }
    this.register(new MockSignLanguageAdapter());
    this.register(new MockBrailleAdapter());
    this.register(new CanvasNeurofeedbackAdapter());

    // Initialize all adapters
    const initPromises = Array.from(this.adapters.values()).map(async (adapter) => {
      try {
        await adapter.initialize({ language: this.preferences.language });
      } catch (error) {
        console.warn(`Failed to initialize ${adapter.name}:`, error);
      }
    });

    await Promise.all(initPromises);
    this.initialized = true;
    this.emit('ready', { adapters: this.getAvailable() });
  }

  /**
   * Register an adapter
   */
  register(adapter: IOutputAdapter): void {
    this.adapters.set(adapter.type, adapter);

    // Forward adapter events
    const events: OutputEvent[] = ['start', 'end', 'error', 'ready', 'busy'];
    events.forEach((event) => {
      adapter.on(event, (data) => this.emit(event, data));
    });
  }

  /**
   * Unregister an adapter
   */
  unregister(type: OutputType): void {
    const adapter = this.adapters.get(type);
    if (adapter) {
      adapter.dispose();
      this.adapters.delete(type);
    }
  }

  /**
   * Get an adapter by type
   */
  get(type: OutputType): IOutputAdapter | undefined {
    return this.adapters.get(type);
  }

  /**
   * Get all adapters
   */
  getAll(): IOutputAdapter[] {
    return Array.from(this.adapters.values());
  }

  /**
   * Get available adapters
   */
  getAvailable(): IOutputAdapter[] {
    return this.getAll().filter((a) => a.isAvailable() && a.isReady());
  }

  /**
   * Output content to specific targets
   */
  async output(content: OutputContent, targets?: OutputType[]): Promise<void> {
    if (!this.initialized) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        'OutputManager not initialized'
      );
    }

    const outputTypes = targets ?? this.preferences.enabledOutputs;
    const results: Array<{ type: OutputType; success: boolean; error?: Error }> = [];

    await Promise.all(
      outputTypes.map(async (type) => {
        const adapter = this.adapters.get(type);
        if (adapter?.isAvailable() && adapter?.isReady()) {
          try {
            await adapter.output(content);
            results.push({ type, success: true });
          } catch (error) {
            results.push({ type, success: false, error: error as Error });
          }
        }
      })
    );

    // Check for failures
    const failures = results.filter((r) => !r.success);
    if (failures.length > 0 && failures.length === results.length) {
      throw new OutputError(
        OutputErrorCode.ADAPTER_NOT_READY,
        'All output adapters failed',
        undefined,
        failures[0].error
      );
    }
  }

  /**
   * Broadcast content to all enabled outputs
   */
  async broadcast(content: OutputContent): Promise<void> {
    await this.output(content, this.preferences.enabledOutputs);
  }

  /**
   * Output to primary adapter only
   */
  async outputPrimary(content: OutputContent): Promise<void> {
    await this.output(content, [this.preferences.primaryOutput]);
  }

  /**
   * Set preferences
   */
  setPreferences(prefs: Partial<OutputPreferences>): void {
    this.preferences = { ...this.preferences, ...prefs };
  }

  /**
   * Get preferences
   */
  getPreferences(): OutputPreferences {
    return { ...this.preferences };
  }

  /**
   * Enable an output type
   */
  enableOutput(type: OutputType): void {
    if (!this.preferences.enabledOutputs.includes(type)) {
      this.preferences.enabledOutputs.push(type);
    }
  }

  /**
   * Disable an output type
   */
  disableOutput(type: OutputType): void {
    this.preferences.enabledOutputs = this.preferences.enabledOutputs.filter(
      (t) => t !== type
    );
  }

  /**
   * Check if output type is enabled
   */
  isOutputEnabled(type: OutputType): boolean {
    return this.preferences.enabledOutputs.includes(type);
  }

  /**
   * Event handlers
   */
  on(event: string, handler: OutputEventHandler): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  off(event: string, handler: OutputEventHandler): void {
    this.handlers.get(event)?.delete(handler);
  }

  private emit(event: string, data: Partial<OutputEventData>): void {
    const eventData: OutputEventData = {
      type: event as OutputEvent,
      adapter: data.adapter ?? ('manager' as OutputType),
      timestamp: Date.now(),
      ...data,
    };
    this.handlers.get(event)?.forEach((handler) => handler(eventData));
  }

  /**
   * Dispose manager and all adapters
   */
  async dispose(): Promise<void> {
    const disposePromises = Array.from(this.adapters.values()).map((adapter) =>
      adapter.dispose()
    );
    await Promise.all(disposePromises);
    this.adapters.clear();
    this.handlers.clear();
    this.initialized = false;
  }
}
